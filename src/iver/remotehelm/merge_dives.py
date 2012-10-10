#!/usr/bin/env python

import sys
import os
import shutil
import struct
import time
import errno
import re
from glob import glob

import lcm

VEHICLE_TOPSIDE = 'topside'
VEHICLE_IVER28 = 'iver28'
VEHICLE_IVER31 = 'iver31'

KNOWN_VEHICLES = [VEHICLE_IVER28, VEHICLE_IVER31, VEHICLE_TOPSIDE]

# filename for topside lcm log
TOPSIDE_LCMLOG_NAME = 'lcmlog'
TOPSIDE_LOGBOOK_NAME = 'logbook'

# dive dir constants and glob patterns
IMAGES = 'images'
LCMDEFS = 'lcmdefs'
NOT_FOUND = 'NOT_FOUND'
UVC_LOG_PATT = '*.log'
UVC_MIS_PATT = '*_UVC.mis'

def isLcmLog (filename):
    log = lcm.EventLog (filename, 'r')
    event = log.read_next_event ()
    log.close ()
    return event is not None

def toMicroSecs (secs):
    return secs*1e6

def toSecs (microSecs):
    return microSecs/1e6

def getStartEndTime (logfile):
    log = lcm.EventLog(logfile, 'r')

    size = log.size()

    # seek to (end minus 5 MB )
    position = max(0, size-5e6)

    # get start time
    event = log.read_next_event()
    startTime = event.timestamp

    # go to the end-ish of file to look for end time
    log.seek(position)
    event = log.read_next_event()

    while event:
        event = log.read_next_event()
        if event:
            endTime = event.timestamp

    # clean up
    log.close()

    return (startTime, endTime)

def mkdir (filename):
    try:
        os.mkdir(filename)
        return 0
    except OSError, err:
        if (err.errno == errno.EEXIST):
            return 0
        else:
            raise

# soft link
def ln_sf (src, dest):
    try:
        os.symlink (os.path.relpath(src, os.path.dirname(dest)), dest)
    except OSError, err:
        if (err.errno == errno.EEXIST):
            return 0
        else:
            raise

# hard link
def ln_f (src, dest):
    try:
        os.link (src, dest)
    except OSError, err:
        if (err.errno == errno.EEXIST):
            return 0
        else:
            raise

def rm_fr(path):
    try:
        os.remove(path)
    except OSError, err:
        if (err.errno == errno.EISDIR or
            err.errno == errno.ENOTEMPTY):
            shutil.rmtree(path)
        elif (err.errno == errno.ENOENT):
            pass
        else:
            raise

class TimeRange():
    
    def __init__(self, timeStart, timeEnd):
        self.timeStart = timeStart
        self.timeEnd = timeEnd

    def captures (self, t):
        return (self.timeStart <= t) and (self.timeEnd >= t)

class DiveOrganizer():

    def __init__ (self, toplevelDir, mergedDir):
        self.toplevelDir = toplevelDir
        self.mergedDir = mergedDir

        # These map lcm logs to various pieces of information needed for making symlinks
        # in "merged" directory
        self.lcmlogToVehicle = {}
        self.lcmlogToUtime = {}
        self.lcmlogToDate = {}
        self.lcmlogToLcmdefs = {}
        self.lcmlogToImages = {}
        self.lcmlogToUvcLog = {}
        self.lcmlogToUvcMis = {}
        self.lcmlogToLogbook = {}
        self.lcmlogToTimeRange = {}
        self.lcmlogToTopsideLogs = {}
        self.lcmlogToMultiLog = {}
        self.multiAuvLogs = []       # A list of tuples where each tuple corresponds to
                                     # iver logs in a mutli-auv mission

        self.progressState = 0
        self.progressCounter = 0

    def _updateProgress (self):
        if self.progressState == 0:
            sys.stdout.write ('....|\r')
        elif self.progressState == 1:
            sys.stdout.write ('..../\r')
        elif self.progressState == 2:
            sys.stdout.write ('....-\r')
        elif self.progressState == 3:
            sys.stdout.write ('....\\\r')

        sys.stdout.flush ()
        self.progressCounter += 1
            
        if (self.progressCounter % 50) == 0:
            self.progressState += 1
            self.progressState %= 4

        self.progressCounter %= 50

    def _getVehicles (self):
        vehicles = os.listdir (self.toplevelDir)

        # Be noisey if weird stuff is in the toplevelDir
        for vehicle in vehicles:
            if vehicle not in KNOWN_VEHICLES:
                print 'WARNING: %s is not a known vehicle' % vehicle

        return vehicles

    # Recursively iterate though directory, look for lcm logs, associate with vehicle, get mtime, lcmdefs, etc...
    def _getLcmlogInfo (self):

        print 'Associating lcm logs with (vehicle), (start_time), and (end_time)...'

        currentVehicle = None
        # diveLegends = []

        for root, dirs, files in os.walk (self.toplevelDir):
            self._updateProgress ()

            for vehicle in self._getVehicles():
                if vehicle in os.path.basename (root):
                    currentVehicle = vehicle
            
            for thing in files:
                self._updateProgress ()
                fullpath = os.path.join (root, thing)
                if isLcmLog (fullpath):
                    self.lcmlogToVehicle[fullpath] = currentVehicle
                    startTime, endTime = getStartEndTime (fullpath)
                    self.lcmlogToUtime[fullpath] = endTime
                    self.lcmlogToDate[fullpath] = time.strftime('%Y-%m-%d', time.localtime(toSecs(self.lcmlogToUtime[fullpath])))
                    try:
                        self.lcmlogToLcmdefs[fullpath] = glob(os.path.join (root, LCMDEFS))[0]
                    except IndexError, err:
                        # print 'WARNING: lcmdefs not found for %s' % (root)
                        self.lcmlogToLcmdefs[fullpath] = NOT_FOUND

                    try:
                        self.lcmlogToImages[fullpath] = glob(os.path.join (root, IMAGES))[0]
                    except IndexError, err:
                        # we don't need to be noisy if there are no images found...
                        pass

                    if currentVehicle == VEHICLE_TOPSIDE:
                        # only add logbook for topside lcmlogs
                        try:
                            self.lcmlogToLogbook[fullpath] = glob(os.path.join (root, TOPSIDE_LOGBOOK_NAME))[0]
                        except IndexError, err:
                            # print 'WARNING: logbook not found for %s' % (root)
                            self.lcmlogToLogbook[fullpath] = NOT_FOUND
                    else:
                        # These UVC files are only available on the auv's
                        try:
                            self.lcmlogToUvcLog[fullpath] = glob(os.path.join (root, UVC_LOG_PATT))[0]
                        except IndexError, err:
                            print 'WARNING: UVC log not found for %s' % (root)
                            self.lcmlogToUvcLog[fullpath] = NOT_FOUND
                        try:
                            self.lcmlogToUvcMis[fullpath] = glob(os.path.join (root, UVC_MIS_PATT))[0]
                        except IndexError, err:
                            print 'WARNING: UVC mission not found for %s' % (root)
                            self.lcmlogToUvcMis[fullpath] = NOT_FOUND

        print 'Done.'
                
    def _getTimeRanges (self):
        print 'Associating lcm logs with <START TIME/END TIME> ...'

        for lcmlog in self.lcmlogToVehicle.keys():
            self._getStartEndTimes (lcmlog)

        print 'Done.'

    def _getStartEndTimes (self, lcmlog):
        self._updateProgress ()

        timeRanges = []
        startTime, endTime = getStartEndTime (lcmlog)
        tr = TimeRange (startTime, endTime);
        self.lcmlogToTimeRange[lcmlog] = tr

    def _createDateDirs (self):
        print 'Clearing merged directory %s...' % self.mergedDir

        rm_fr (self.mergedDir)
        
        mkdir (self.mergedDir)        
        for date in set(self.lcmlogToDate.values()):
            mkdir (os.path.join(self.mergedDir, date))
            time.sleep(.1)     # Somehow needed to preserved timestamp order
            self._updateProgress ()

        print 'Done.'

    def _getTopsideLcmlogs (self):
        print 'Associating iver logs with topside log(s)...'

        # assign an empty list to each iver28/iver31 lcm log.  This empty list will be
        # filled with topside lcm logs that correspond to that dive
        for lcmlog in self.lcmlogToVehicle.keys():
            self._updateProgress ()

            if self.lcmlogToVehicle[lcmlog] == VEHICLE_TOPSIDE:
                continue

            self.lcmlogToTopsideLogs[lcmlog] = []
            
        # Iterate through every topside lcm log.  If it's utime falls in the <start
        # time, end time> of a vehicle's lcm log, then add it to a list of topside lcm
        # logs
        for topsideLcmlog in self.lcmlogToVehicle.keys():
            self._updateProgress ()

            if not self.lcmlogToVehicle[topsideLcmlog] == VEHICLE_TOPSIDE:
                continue

            for vehLcmlog, timeRange in self.lcmlogToTimeRange.iteritems():
                if timeRange.captures (self.lcmlogToUtime[topsideLcmlog]):
                    if self.lcmlogToVehicle[vehLcmlog] != VEHICLE_TOPSIDE:
                        self.lcmlogToTopsideLogs[vehLcmlog].append (topsideLcmlog)

        # sort lcmlogToTopsideLogs by start time
        for vehLcmlog in self.lcmlogToTopsideLogs.keys():
            self._updateProgress ()
            self.lcmlogToTopsideLogs[vehLcmlog].sort(key=lambda tsideLog: self.lcmlogToTimeRange[tsideLog].timeStart)


        # Next, iterate though every vehicle lcm log.  Add the *closest* (in utime)
        # topside lcm log to the list
        for thisLcmlog in self.lcmlogToVehicle.keys():
            self._updateProgress ()

            if self.lcmlogToVehicle[thisLcmlog] == VEHICLE_TOPSIDE:
                continue
            
            min = float('inf')
            closestTopsideLcm = None
 
            for thatLcmlog in self.lcmlogToVehicle.keys():
                self._updateProgress ()
                
                if thisLcmlog is thatLcmlog:
                    continue
                 
                if self.lcmlogToVehicle[thatLcmlog] != VEHICLE_TOPSIDE:
                    continue
 
                timeDelta = abs(self.lcmlogToUtime[thisLcmlog] - self.lcmlogToUtime[thatLcmlog])
                if timeDelta < min:
                    min = timeDelta
                    closestTopsideLcm = thatLcmlog
 
            if closestTopsideLcm not in self.lcmlogToTopsideLogs[thisLcmlog]:
                self.lcmlogToTopsideLogs[thisLcmlog].append (closestTopsideLcm)

        print 'Done.'

    def _getMultiAuvLcmlogs (self):
        print 'Looking for any multi-auv missions...'

        for lcmlog in self.lcmlogToTimeRange.keys():
            self._updateProgress ()

            if self.lcmlogToVehicle[lcmlog] == VEHICLE_TOPSIDE:
                continue

            for thatLcmlog in self.lcmlogToTimeRange.keys():
                self._updateProgress ()

                if lcmlog == thatLcmlog:
                    continue

                if self.lcmlogToVehicle[thatLcmlog] == VEHICLE_TOPSIDE:
                    continue

                timeRange = self.lcmlogToTimeRange[lcmlog]


                if (timeRange.captures (self.lcmlogToUtime[thatLcmlog]) and 
                    self.lcmlogToVehicle[lcmlog] != self.lcmlogToVehicle[thatLcmlog]):

                    # ADD LOGIC TO IGNORE MULTI-AUV MISSIONS HERE

                    thatTimeRange = self.lcmlogToTimeRange[thatLcmlog]

                    # check mission length greater than 5 minutes
                    if timeRange.timeEnd - timeRange.timeStart < toMicroSecs(5*60):
                        continue
                    if thatTimeRange.timeEnd - thatTimeRange.timeStart < toMicroSecs(5*60):
                        continue

                    # check start time of auv1 - end time of auv2 is large enough
                    if abs(timeRange.timeStart - thatTimeRange.timeEnd) < toMicroSecs(5*60):
                        continue

                    # check end time of auv1 - start time of auv2 is large enough
                    if abs(timeRange.timeEnd - thatTimeRange.timeStart) < toMicroSecs(5*60):
                        continue

                    print 'Multi-auv mission detected: %s --- %s' % (lcmlog, thatLcmlog)
                    self.multiAuvLogs.append ((lcmlog, thatLcmlog))

        print 'Done.'

    def _populateVehicleDir (self, diveDir, lcmlog, spliceTimeRange=None, topsideLcmlogs=None):
        vehicleDir = os.path.join (diveDir, self.lcmlogToVehicle[lcmlog])
        mkdir (vehicleDir)

        # populate the merged directory with links
        ln_sf (lcmlog, os.path.join(vehicleDir, 'lcmlog'))

        # lcmdefs
        ln_sf (self.lcmlogToLcmdefs[lcmlog], os.path.join(vehicleDir, LCMDEFS))
        # uvc log
        ln_sf (self.lcmlogToUvcLog[lcmlog], os.path.join(vehicleDir, 'uvc-log'))
        # uvc mission
        ln_sf (self.lcmlogToUvcMis[lcmlog], os.path.join(vehicleDir, 'uvc-mis'))

        # images (if present)
        if self.lcmlogToImages.has_key (lcmlog):
            ln_sf (self.lcmlogToImages[lcmlog], os.path.join(vehicleDir, IMAGES))

        # topside directory
        topsideDir = os.path.join (diveDir, VEHICLE_TOPSIDE)
        mkdir (topsideDir)

        # overriding topsideLcmlogs is useful for multi-auv missions
        if topsideLcmlogs is None:
            topsideLcmlogs = self.lcmlogToTopsideLogs[lcmlog]

        # for each associated topside log, make a symlink
        i = 0
        for topsideLog in topsideLcmlogs:
            ln_sf (topsideLog, os.path.join(topsideDir, 'lcmlog%.2d' % i))
            # potentially called multiple times, but it looks cleaner this way
            ln_sf (self.lcmlogToLcmdefs[topsideLog], os.path.join(topsideDir, LCMDEFS))
            ln_sf (self.lcmlogToLogbook[topsideLog], os.path.join(topsideDir, TOPSIDE_LOGBOOK_NAME))
            i = i+1

        # if there's more than 1 topside lcm log, splice
        if len(topsideLcmlogs) > 1:

            filesToSplice = ' '.join(topsideLcmlogs)
            if spliceTimeRange is not None:
                cmd = '%s -s %f -e %f %s %s' % ('bot-lcm-logsplice', 
                                                toSecs(spliceTimeRange.timeStart), 
                                                toSecs(spliceTimeRange.timeEnd), 
                                                filesToSplice, 
                                                os.path.join(topsideDir, TOPSIDE_LCMLOG_NAME))
            else:
                cmd = '%s %s %s' % ('bot-lcm-logsplice', 
                                    filesToSplice, 
                                    os.path.join(topsideDir, TOPSIDE_LCMLOG_NAME))
            os.system (cmd)

        # otherwise, filter to provided start/end times
        elif len(topsideLcmlogs) == 1:
            if spliceTimeRange is not None:
                cmd = '%s -s %f -e %f %s %s' % ('bot-lcm-logfilter', 
                                                toSecs(spliceTimeRange.timeStart), 
                                                toSecs(spliceTimeRange.timeEnd), 
                                                topsideLcmlogs[0],
                                                os.path.join(topsideDir, TOPSIDE_LCMLOG_NAME))
            else:
                cmd = '%s %s %s' % ('bot-lcm-logfilter', 
                                    topsideLcmlogs[0],
                                    os.path.join(topsideDir, TOPSIDE_LCMLOG_NAME))
            os.system (cmd)

    def _createSymlinks (self):
        print 'Creating directory tree and symbolic links to lcm logs...'

        handledLcmlog = {}
        for lcmlog in self.lcmlogToVehicle.keys():
            handledLcmlog[lcmlog] = False

        # Create directories by day, in order
        for date in set(self.lcmlogToDate.values()):
            self._updateProgress()

            diveNum = 1
            # Get the lcmlogs, oldest first
            for lcmlog in sorted (self.lcmlogToUtime.items(), key=lambda(k,v):(v,k)):
                self._updateProgress()
                lcmlog = lcmlog[0]

                # skip topside
                if self.lcmlogToVehicle[lcmlog] == VEHICLE_TOPSIDE:
                    continue

                # Does the lcmlog correspond to the current day?
                if self.lcmlogToDate[lcmlog] == date:

                    # If we've already created symlinks for this dive, skip. (happens for
                    # multi-auv missions)
                    if handledLcmlog[lcmlog]:
                        continue

                    # Create the current dive folder for the current day
                    diveDir = os.path.join (self.mergedDir, self.lcmlogToDate[lcmlog], 'dive%.3d' % diveNum)
                    diveNum += 1
                    mkdir (diveDir)

                    # Handle multi-auv missions --- check to see if the lcm log
                    # corresponds to a multi-auv mission
                    for tup in self.multiAuvLogs:

                        if lcmlog in tup:

                            # ------------------------------------------------------------------------
                            # decide what the start and end time of the spliced topside
                            # log should be
                            # ------------------------------------------------------------------------

                            # first, get the smallest mission start time and largest
                            # mission end time
                            logsByStartTime = sorted(tup, key=lambda lcmlogMulti: self.lcmlogToTimeRange[lcmlogMulti].timeStart)
                            logsByEndTime = sorted(tup, key=lambda lcmlogMulti: self.lcmlogToTimeRange[lcmlogMulti].timeEnd)
                            firstAuvLcmlog = logsByStartTime[0]
                            lastAuvLcmlog = logsByEndTime[-1]

                            # The first/last topside log for the first/last auv mission,
                            # respectively
                            firstAuvTopsideLcmlog = self.lcmlogToTopsideLogs[firstAuvLcmlog][0]
                            lastAuvTopsideLcmlog = self.lcmlogToTopsideLogs[lastAuvLcmlog][-1]

                            # the start/end times of the previous two topside logs,
                            # respectively
                            firstAuvTopsideLcmlogStartTime = self.lcmlogToTimeRange[firstAuvTopsideLcmlog].timeStart
                            lastAuvTopsideLcmlogEndTime = self.lcmlogToTimeRange[lastAuvTopsideLcmlog].timeEnd

                            # the start/end times of the first/last auv missions in the
                            # multi-auv dive
                            firstAuvLcmlogStartTime = self.lcmlogToTimeRange[firstAuvLcmlog].timeStart
                            lastAuvLcmlogEndTime = self.lcmlogToTimeRange[lastAuvLcmlog].timeEnd

                            # compute the start and end times for splicing
                            spliceStartTime = max (firstAuvLcmlogStartTime - firstAuvTopsideLcmlogStartTime, 0)
                            spliceEndTime = lastAuvLcmlogEndTime - firstAuvLcmlogStartTime + spliceStartTime

                            spliceTimeRange = TimeRange (spliceStartTime, spliceEndTime)

                            # get a sorted list of topside lcm logs by unioning the
                            # topside logs for the first/last auv missions
                            topsideLcmlogs = list (set (self.lcmlogToTopsideLogs[firstAuvLcmlog]) | set (self.lcmlogToTopsideLogs[lastAuvLcmlog]))
                            topsideLcmlogs.sort(key=lambda tsideLog: self.lcmlogToTimeRange[tsideLog].timeStart)

                            for lcmlogMulti in tup:
                                # populate the dive dir for each lcm log in the multi-auv
                                # mission, overriding the time range and topside lcm logs
                                self._populateVehicleDir (diveDir, lcmlogMulti, spliceTimeRange=spliceTimeRange, topsideLcmlogs=topsideLcmlogs)
                                handledLcmlog[lcmlogMulti] = True

                    if handledLcmlog[lcmlog]:
                        continue

                    # For single-auv dives, compute start/end times for the concated topside logs
                    topsideLcmlogs = self.lcmlogToTopsideLogs[lcmlog]
                    spliceStartTime = max(self.lcmlogToTimeRange[lcmlog].timeStart - self.lcmlogToTimeRange[topsideLcmlogs[0]].timeStart, 0)
                    spliceEndTime = self.lcmlogToTimeRange[lcmlog].timeEnd - self.lcmlogToTimeRange[lcmlog].timeStart + spliceStartTime
                    spliceTimeRange = TimeRange (spliceStartTime, spliceEndTime)
                    self._populateVehicleDir (diveDir, lcmlog, spliceTimeRange=spliceTimeRange)

        print 'Done.'

    def run (self):
        # Create datastructures that encode structure of directory tree/symbolic links
        self._getLcmlogInfo ()
        self._getTimeRanges ()
        self._getTopsideLcmlogs ()
        self._getMultiAuvLcmlogs ()

        # Create the directories and symbolic links, then we're done
        self._createDateDirs ()
        self._createSymlinks()

if __name__ == '__main__':

    if len (sys.argv) != 3:
        print 'Usage %s <data dir> <merged dir>' % os.path.basename (sys.argv[0])
        print ' where <data dir> is the directory containing the <%s> directories and <merged dir> is the directory in which to create links' % ' '.join(KNOWN_VEHICLES)
        sys.exit (1)

    do = DiveOrganizer (sys.argv[1], sys.argv[2])
    do.run ()
