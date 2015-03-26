######################################################################
# This file handles all the platform pose updates.
#
# There are some TODOS listed throughout the file with instructions.
# See documentation below for more info.
#
# Contact me if you have any questions.
#
# Author:
#   Ariell Friedman
#   ariell.friedman@gmail.com
#   25 AUG 2014
#
######################################################################

from random import random, randint
import time
import threading

import sys
import math
import lcm
import ConfigParser
import select
import string
import os
import pyproj

LCMROOT='/home/auv/git/acfr_lcm'
#SEABEDGUIROOT='/home/auv/git/seabed_gui'
sys.path.append('{0}/build/lib/python{1}.{2}/dist-packages/perls/lcmtypes/'.format(LCMROOT, sys.version_info[0], sys.version_info[1]))
#sys.path.append('{0}/build/lib/'.format(LCMROOT))
#sys.path.append('{0}/lib/'.format(SEABEDGUIROOT))


# TODO: remove try-except statement
try:
    import missionMP
except:
    print "Unable to load missionMP (Sirius)"
try:
    import missionXML
except:
    print "Unable to load missionXML (Iver)"


from acfrlcm import auv_status_short_t, ship_status_t
from senlcm import usbl_fix_t
#except:
#    pass

# This global dictionary stores all the platform information updates
platformdata = {}


######################################################################
# Start threads for platform updates
# These are currently fake threads that update the vehicle
# poses. TODO: Make them real or replace them with something similar!
######################################################################
def init_platformdata_threads():
    LcmThread().start()


######################################################################
# Get data for a specific platform
# The global variable platformdata is updated by another process/thread.
# This function simply reads the output for a specific platform
######################################################################
def get_platformdata(platform):
    return platformdata[platform]

######################################################################
# Parse mission file
# This platform
# Outputs latlngs and origin from mission file
# TODO: parse actual mission file
######################################################################
def parse_mission (filepath, cfgorigin=[0, 0]):
    # TODO: parse mission file and origin and return in LAT/LON
    # filepath will contain the filepath to the mission file

    # from the filename we need to work out which mission parser to use
    try:
        fileName, fileExtension = os.path.splitext(filepath)
        if fileExtension.lower() == '.xml':
            mission = missionXML.Mission()
        elif fileExtension.lower() == '.mp':
            mission = missionMP.Mission()
        else:
            print 'Incorrect mission type'
            return

        mission.load(filepath)
        origin = [mission.getOriginLat(), mission.getOriginLon()]
        if origin[0] == 0 or origin[1] == 0:
            origin = cfgorigin

        waypoints = mission.dumpSimple()

        # convert the waypoints to latitude and longitude
        projStr = '+proj=tmerc +lon_0={} +lat_0={} +units=m'.format(origin[1], origin[0])
        p = pyproj.Proj(projStr)

        latlngs = []
        for wpt in waypoints:
            ll = p(wpt.y, wpt.x, inverse=True)
            latlng = [ll[1], ll[0]]
            latlngs.append(latlng)

    except:
        print "Unable to parse mission!!!"
        return
    return latlngs, origin

######################################################################
# Set waypoint
# TODO: make real
######################################################################
def send_to_platform(platform, lat, lon):

    response = "This feature has not been implemented yet!!!"
    return platform, response


######################################################################
#
######################################################################

class LcmThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.exitFlag = False
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit


    def usblFixHandler(self, channel, data):
        msg = usbl_fix_t.decode(data)
        msgid = msg.utime
        platform = '{}'.format(msg.remote_id)
        platformdata[platform] = {
            'msgid': msgid,                                 # REQUIRED (number)
            'pose': {
                'lat': round(math.degrees(msg.latitude), 8),          # REQUIRED (decimal degrees)
                'lon': round(math.degrees(msg.longitude), 8)          # REQUIRED (decimal degrees)
            }
        }



    def shipStatusHandler(self, channel, data):
        msg = ship_status_t.decode(data)
        msgid = msg.utime
        #platform = '{}'.format(msg.ship_id)
        platform = msg.name
        platformdata[platform] = {
            'msgid': msgid,                                 # REQUIRED (number)
            'pose': {
                'lat': round(math.degrees(msg.latitude), 8),          # REQUIRED (decimal degrees)
                'lon': round(math.degrees(msg.longitude), 8),         # REQUIRED (decimal degrees)
                'heading': round(math.degrees(msg.heading), 2),        # REQUIRED (degrees)
                'roll': round(math.degrees(msg.roll), 2),        # REQUIRED (degrees)
                'pitch': round(math.degrees(msg.pitch), 2)        # REQUIRED (degrees)
            }
        }



    def auvStatusHandler(self, channel, data):
        print channel
        msg = auv_status_short_t.decode(data)
        print msg
        platform = '{}'.format(msg.target_id)
        msgid = msg.utime
        platformdata[platform] = {
            'msgid': msgid,                                 # REQUIRED (number)
            'pose': {
                'lat': round(math.degrees(msg.latitude), 8),                  # REQUIRED (decimal degrees)
                'lon': round(math.degrees(msg.longitude), 8),                 # REQUIRED (decimal degrees)
                'heading': round(msg.heading/10.0,1), # REQUIRED (degrees)
                'alt': float(msg.altitude)/10.0,                           # OPTIONAL (m)
                'depth': float(msg.depth)/10.0,                            # OPTIONAL (m)
                'roll': round(msg.roll/10.0,1),       # OPTIONAL / REQUIRED for dashboard (degrees)
                'pitch': round(msg.pitch/10.0,1)      # OPTIONAL / REQUIRED for dashboard (degrees)
                #'uncertainty': 1
            },
            'stat': {
                # You can add whatever custom status messages you like here
                # NB: 'bat' has a special display widget
                # Any others, just get displayed normally
                'bat': msg.charge,                     # REQUIRED for dashboard (int 0-100)
                'waypt': msg.waypoint,
                '#imgs': msg.img_count
                #'state': 'OK'
            },
            'alert': {
                # You can add whatever custom alerts you like here.
                # Must return 1 for error, 0, for OK.
                'dvl':  msg.status & (1 << 0),
                'dvl_bl':  msg.status & (1 << 1),
                'gps':  msg.status & (1 << 2),
                'depth':  msg.status & (1 << 3),
                'comp':  msg.status & (1 << 4),
                'imu':  msg.status & (1 << 5),
                'oas':  msg.status & (1 << 6),
                'nav':  msg.status & (1 << 7),
                'epuck':  msg.status & (1 << 8),
                'abort':  msg.status & (1 << 9),
                'press':  msg.status & (1 << 13),
                'leak':  msg.status & (1 << 15)
            }
        }
        # Alert if AUV > 6 m
        if platformdata[platform]['pose']['depth'] < 6:
            platformdata[platform]['alert']['DEP<6'] = 1

    def run(self):
        self.lc = lcm.LCM()
        self.lc.subscribe("AUVSTAT.*", self.auvStatusHandler)
        self.lc.subscribe("SHIP_STATUS.*", self.shipStatusHandler)
        self.lc.subscribe("USBL_FIX.*", self.usblFixHandler)

        timeout = 1  # amount of time to wait, in seconds
        while not self.exitFlag:
            rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
            if rfds:
                self.lc.handle()
