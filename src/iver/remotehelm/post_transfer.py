#!/usr/bin/env python

## @package post_transfer
#
# @author Paul Ozog - paulozog@umich.edu 
#
# @details Run this after every mission - even if there's an abort or
# other failure
#
# @brief Post-mission file transfer and checking
#
import sys, os, shutil
import re
import getopt
import commands
import socket
import glob
import operator

## 
# if true, don't execute any system calls
DEBUG = 0

SCRIPT_PATH        = os.path.dirname(__file__)
SCRIPT_NAME        = os.path.basename(sys.argv[0])

# Use BotParam to check the master.cfg in topside directory
#sys.path.append(PATH_TO_BOT_PARAM)
from perls import BotParam

##
# The location of topside dive directories. Must exist
BASE_DIR           = os.path.join(os.getenv('HOME'), 'mission-data')

##
# The location of dive directories on the AUV.  Must exist
AUV_BASE_DIR       = '/home/auv/mission-data'

##
# The user who created the logs, etc. on the vehicle (either 'auv' or
# 'root')
AUV_USERNAME       = 'root'

##
# Since topside will be calling this script in auvMode, we need to
# know the path to the script on the vehicle
PATH_TO_AUV_SCRIPT = os.path.join('/home/auv', 'perls', 'bash', SCRIPT_NAME)

##
# Path to topside lcm defs
PATH_TO_LCMDEFS    = os.path.join(os.getenv('HOME'), 'perls', 'lcmdefs')

RSYNC_DIVE_DATA    = ' '.join(['rsync',
                               '-ak',
                               '--exclude=".svn"',
                               '--exclude="lcmlog*"',
                               '--exclude="*PROSILICA*"',
                               '--exclude="cam-tmp"'])
RSYNC_LCMDEFS      = ' '.join(['rsync',
                               '-ak',
                               '--exclude=".svn"'])
##
# This editor will execute on the vehicle so the operator can write a
# short README
AUV_EDITOR_COMMAND = 'gvim'

##
# This should be in mission-data on the vehicle.  lcm-logger will
# place a log here
LCMLOG_TMP         = 'lcmlog-tmp'

##
# This should be in mission-data on the vehicle.  cam-logger will
# place a log here
CAMLOG_TMP         = 'cam-tmp'

##
# Path to topside master.cfg
LOCAL_MASTER_CFG   = os.path.join(os.getenv('HOME'), 'perls', 'config', 'master.cfg')

##
# Basename of concat-ed READMEs
CONCAT_README      = "README"

##
# If someFile does not exist, raise a CheckFailed
#
# @param someFile
# @param userArgs : Not used
# @throws CheckFailed
def checkExists(someFile, userArgs):
    if not os.path.exists(someFile):
        raise CheckFailed('%s does not exist' % someFile)

#    print '%s passed %s' % (someFile, checkExists.__name__)

##
# If someDir does not have any files matching userArgs, raise a
# CheckFailed
#
# @param someDir
# @param userArgs : Regex patterns
# @throws CheckFailed
def checkDirHasFiles(someDir, userArgs):
    matchPatterns = [re.compile(pattern) for pattern in userArgs]

    for i in range(0, len(matchPatterns)):
        pattern = matchPatterns[i]
        if not any(pattern.match(file) for file in os.listdir(someDir)):
            raise CheckFailed("%s does not contain files matching '%s'" % (someDir, userArgs[i]))

#    print '%s passed %s' % (someDir, checkDirHasFiles.__name__)

##
# Check that master.cfg is valid iver config
# @param someDir
# @param userArgs : Not used
# @throws CheckFailed
def checkMasterConfigInDir(someDir, userArgs):
    pass
#     try:
#         param = BotParam(os.path.join(someDir, 'master.cfg'))
#         vehicleName = param.get_str('vehicle.name')
#     except Exception, err:
#         raise CheckFailed("%s does contain a valid iver master.cfg" % (someDir))

##
# Exception that should be raised if a check fails
class CheckFailed(BaseException):
    pass

##
# Exception that should be raised to display some information about a
# check
class CheckInfo(BaseException):
    pass

##
# Raised if command line arguments are nonsensical
class UsageException(BaseException):
    pass

############################################################
# Post-transfer functions and classes
############################################################

##
# @param d : some directory
#
# @details os.listdir() only returns relative paths, but this is a
# simple way around that
#
# @return Full path of the contents of d (non-recursive)
def listdirFullpath(d):
    return [os.path.join(d, f) for f in os.listdir(d)]

##
# @param command : argument to pass to func
# @param func : function to call
#
# @details Used for debugging system calls, etc.
def commandWrapperOneArg(func, command):
    if DEBUG:
        print func, command
    else:
        func(command)

##
# @param src : first argument to pass to func
# @param dest : second argument to pass to func
# @param func : function to call
#
# @details Used for debugging system calls, etc.
def commandWrapperTwoArgs(func, src, dest):
    if DEBUG:
        print func, src, dest
    else:
        func(src, dest)

##
# @param dir : some directory
#
# @return The most recently modified file/directory in dir
def getMostRecentFile(dir):
    flist = glob.glob(os.path.join(dir, '*'))

    for i in range(len(flist)):
        statinfo = os.stat(flist[i])
        flist[i] = flist[i],statinfo.st_ctime

    flist.sort(key=operator.itemgetter(1))
    
    try:
        returnVal = flist[-1][0]
    except Exception, err:
        returnVal = None

    return returnVal

## The main post transfer class
# @author Paul Ozog - paulozog@umich.edu
#
class PostTransfer(object):
    
    ## The constructor
    # @param interface : ie, iver28-wifi iver31, etc.
    #
    # @param baseDir : in case the user wants to change the path to
    # mission-data/ on topside
    #
    # @param auvBaseDir : in case the user wants to change the path to
    # mission-data/ on the vehicle
    #
    # @param multiMode : should be 1 during multi-auv post transfer
    #
    def __init__(self, interface,
                 baseDir=BASE_DIR, auvBaseDir=AUV_BASE_DIR,
                 multiMode=False):
        self.baseDir     = baseDir
        self.auvBaseDir  = auvBaseDir
        self.interface   = interface
        self.vehicleName = self.interface.split('-')[0]
        self.multiMode   = multiMode

        try:
            (self.auv, self.mode) = interface.split('-')
        except ValueError, err:
            (self.auv, self.mode) = (interface, None)

    ##
    # Returns a 2-tuple: (most recent dive number, most recent dive directory)
    #
    # @param dir : some directory
    #
    # @return (0, None) if there was no dive found
    #
    def getDiveNumber(self, dir):

        diveNumber    = 0
        recentDiveDir = None

        for fullpath in listdirFullpath(dir):
            if os.path.isdir(fullpath):
                basename = os.path.basename(fullpath)
                if re.match('.*dive\.[0-9]+', basename):
                    number = int(basename.split('dive.')[-1])
                    if number > diveNumber:
                        diveNumber = number
                        recentDiveDir  = fullpath

        return (diveNumber, recentDiveDir)

    ##
    # Sends an ssh-command to run post_transfer.py in multiMode for
    # multi-auv dives
    # 
    # @param None
    def kickoffAuv(self):

        try:
            (localDiveNumber, localRecentDir) = self.getDiveNumber(dir=self.baseDir)
        except OSError, err:
            print ('Error: Cannot continue unless your home directory has a mission-data/ dir')
            sys.exit(1)
            
        #send the command: ssh AUV_USERNAME@iver28 post-transfer.py -a localDiveNumber
        if self.multiMode:
            command = ' '.join(['ssh', 
                                '%s@%s' % (AUV_USERNAME, self.interface),
                                'nohup',
                                PATH_TO_AUV_SCRIPT,
                                '-a',
                                '-m', #use multi mode for multi-auv dives
                                str(localDiveNumber)])
        else:
            command = ' '.join(['ssh', 
                                '%s@%s' % (AUV_USERNAME, self.interface),
                                'nohup',
                                PATH_TO_AUV_SCRIPT,
                                '-a',
                                str(localDiveNumber)])
        commandWrapperOneArg(os.system, command)

    ##
    # After being kicked off, the auv will call this routine to move
    # cam/lcm logs to the appropriate dive directory
    #
    # @param localDiveNumber : The most recent dive number on topside.
    # Automatically passed via command line argument
    def prepAuv(self, localDiveNumber):

        (auvDiveNumber, auvRecentDir) = self.getDiveNumber(dir=self.auvBaseDir)

        #if multiple auv post transfer, don't increment to a new dive once the first
        # vehicle has been post transfered.
        if (self.multiMode):
            newDive = localDiveNumber
        else:
            newDive = localDiveNumber + 1

        if (localDiveNumber >= auvDiveNumber and
            os.path.isdir(auvRecentDir)):

            newName = auvRecentDir.split('.')[0] + '.%03d' % newDive
            commandWrapperTwoArgs(shutil.move, auvRecentDir, newName)
            auvRecentDir = newName

        else:
            newDive = auvDiveNumber

        commandWrapperOneArg(os.system, 'killall lcm-logger')

        lcmlog = getMostRecentFile(os.path.join(self.auvBaseDir, LCMLOG_TMP))
        if lcmlog is not None: 

            lcmlogNoDive = lcmlog.split('.')[0] #take out the dive number

            commandWrapperTwoArgs(shutil.move, os.path.join(self.auvBaseDir, LCMLOG_TMP, lcmlog),
                                  os.path.join(auvRecentDir, self.auv, 
                                               '%s.%03d' % (os.path.basename(lcmlogNoDive), newDive)))

        else:
            print 'Warning : There is no lcmlog file in %s...' % os.path.join(self.auvBaseDir, LCMLOG_TMP)

        camlog = getMostRecentFile(os.path.join(self.auvBaseDir, CAMLOG_TMP))
        if camlog is not None:

            commandWrapperOneArg(os.mkdir, os.path.join(auvRecentDir, self.auv, 'images'))

            command = ' '.join(['mv',
                                os.path.join(self.auvBaseDir, CAMLOG_TMP, '*'),
                                os.path.join(auvRecentDir, self.auv, 'images')])
            commandWrapperOneArg(os.system, command)

        else:
            print 'Warning : There are no images in %s...' % os.path.join(self.auvBaseDir, CAMLOG_TMP)

        auvDir = os.path.join(auvRecentDir, self.auv) # ie: /data/UMBS/2011-blah-dive.123/iver28
        if not os.path.isdir(auvDir):
            os.mkdir(auvDir)

        #only create a README when multiMode is off
        if not self.multiMode:
            readmeFile = os.path.join(auvRecentDir, 'README.%03d' % newDive)
            command    = ' '.join(['echo',
                                   'DIVE %03d' % newDive,
                                   '>',
                                   readmeFile])
            commandWrapperOneArg(os.system, command)
            print 'Starting editor... you might have to press Ctrl+C to continue with post transfer...'
            editCommand = '%s %s' % (AUV_EDITOR_COMMAND, 
                                     readmeFile)
            commandWrapperOneArg(os.system, editCommand)


    ##
    # Call rsync to get the important auv data.  
    # @param None
    def rsyncAuvData(self):
        print '*** SYNCING DIVE DATA FROM VEHICLE ***'
        command = ' '.join([RSYNC_DIVE_DATA,
                            '%s@%s:%s/' % (AUV_USERNAME, self.interface, self.auvBaseDir),
                            self.baseDir + '/'])
        commandWrapperOneArg(os.system, command)
        print 'Done'
        print

    ##
    # Move the relevant topside files to the dive folder under
    # topside/.  Check the contents of the dive folder to make sure
    # everything is there
    # @param None
    def moveTopsideData(self):
        (localDiveNumber, recentDiveDir)  = self.getDiveNumber(dir=self.baseDir)

        topsideDir = os.path.join(recentDiveDir, 'topside')

        if not os.path.isdir(topsideDir): os.mkdir(topsideDir)

        commandWrapperTwoArgs(shutil.copy, LOCAL_MASTER_CFG, topsideDir)

        print '*** SYNCING LCMDEFS FROM LOCALHOST ***'
        command = ' '.join([RSYNC_LCMDEFS,
                            PATH_TO_LCMDEFS,
                            topsideDir + '/'])
        commandWrapperOneArg(os.system, command)
        print 'Done'
        print
 
        command = ' '.join(['mv',
                            self.baseDir + '/lcmlog-tmp/lcmlog*',
                            recentDiveDir + '/topside'])
        commandWrapperOneArg(os.system, command)

        if os.path.exists(self.baseDir + '/lcmlog-tmp/LOGBOOK*'):
            command = ' '.join(['mv',
                                self.baseDir + '/lcmlog-tmp/LOGBOOK*',
                                recentDiveDir])
            commandWrapperOneArg(os.system, command)
    
        ## 
        # A weird dictionary datastructure that defines how to run
        # the post transfer checks.
        #
        # @verbatim
        # Format:
        # file1:
        # [ [func1, func2, ...], 
        # [ [argsForF1], [argsForF2], ... ] ],
        # file2: 
        # [ [func1, func2, ...], 
        # [ [argsForF1], [argsForF2], ... ] ],
        # @endverbatim
        #
        # @details Each of func1, func2, etc. takes two arguments: the
        # file to check and any extra args.  We will call each of the functions as follows:
        #
        # @code
        # func1(file1, [argsForF1])
        # func2(file1, [argsForF2])
        # ...
        # func1(file2, [argsForF1])
        # func2(file2, [argsForF2])
        # @endcode
        #
        # The idea is that if one of the functions throws an
        # exception, the check fails and post_transfer screwed up
        # somewhere
        #
        self.requiredChecks = {

            recentDiveDir:
                [ [checkExists, checkDirHasFiles],
                  [ [None],     ['^%s.*' % self.vehicleName, '^README.*', '^topside$'] ] ],

            os.path.join(recentDiveDir, self.auv):
                [ [checkExists, checkDirHasFiles, checkMasterConfigInDir],
                  [ [None],     ['^lcmdefs$', 'master\.cfg', 
                                 '.*\.log$', '.*\.mis$'], [None] ] ],

            topsideDir:
            [ [checkExists, checkDirHasFiles, checkMasterConfigInDir],
              [ [None],       ['master\.cfg', 'lcmdefs', 'lcmlog-.*'], [None] ] ]

            }
        
        self.checkTransferedFiles()

    ## 
    # Runs all the functions in self.requiredChecks (see moveTopsideData)
    #
    # If an CheckFailed is raised, we failed the test and something
    # wasn't transfered properly
    #
    #@param None
    def checkTransferedFiles(self):

        for file, checkList in self.requiredChecks.items():
            errorEncounterd = 0
            funcList        = checkList[0]
            argList         = checkList[1]

            for i in range(0, len(funcList)):
                func = funcList[i]
                args = argList[i]
                
                try:
                    func(file, args)
                except CheckFailed, err:
                    errorEncounterd = 1
                    print '\n', file, '***check FAILED:', err, '***\n'
                except CheckInfo, err:
                    errorEncounterd = 1
                    print file, 'check info:', err

            if not errorEncounterd:
                print '%s passed all checks!' % file

    def concatReadmes(self):

        allReadmes = os.path.join(self.baseDir, CONCAT_README)

        if os.path.exists(allReadmes):
            os.remove(allReadmes)

        dirs = listdirFullpath(self.baseDir)
        dirs.sort()
        pattern = re.compile('.*README\.[0-9][0-9][0-9]')

        for fullpath in dirs:
            for contents in glob.glob(os.path.join(fullpath, 'README.*')):
                if pattern.match(contents):
                    os.system('echo %s: >> %s' % (os.path.basename(fullpath), allReadmes))
                    os.system('cat %s >> %s' % (contents, allReadmes))
                    os.system('echo >> %s' % allReadmes)
                    os.system('echo %s >> %s' % ('-'*50, allReadmes))
                    os.system('echo >> %s' % allReadmes)
            
##
# Show the usage of post_transfer.py
# @param None
def printUsage():
    print ' '.join([sys.argv[0],
                    '<-i auv-interface...>',
                    '[-a]',
                    '[-m, --multiauv]',
                    '[-h]',
                    '[dive number]'])

##
# Parse command line arguments
# @param : array of arguments (like sys.argv)
#
# @return tuple of command line arguments:
# (interface, auvMode, localDiveNumber, multiMode)
def parseArgs(args):

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hli:am", ["help", "multiauv", "interface="])
    except getopt.GetoptError, err:
        raise UsageException(err)

    interface       = None
    auvMode         = 0
    localDiveNumber = None
    multiMode       = 0

    for o, a in opts:
        if o in ("-h", "--help"):
            raise UsageException('')
        elif o in ("-i", "--interface"):
            interface = a
        elif o == '-a':
            auvMode = 1
        elif o in ('-m', '--multiauv'):
            multiMode = 1
    
    if interface is None and not auvMode:
        raise UsageException('You must provide an interface')

    if len(args) == 1: 
        try:
            localDiveNumber = int(args[0])
        except Exception, err:
            raise UsageException("Given dive number isn't an int")
            sys.exit(1)

    if auvMode and localDiveNumber is None:
        raise UsageException('You must provide the topside dive number')

    return (interface, auvMode, localDiveNumber, multiMode)

##
# @param args: command line arguments
#
# @return
# 0 for success, 1 for failure
def main(args):

    global RSYNC_DIVE_DATA

    try:
        (interface, auvMode, localDiveNumber, multiMode) = parseArgs(args)
    except UsageException, err:
        print err
        printUsage()
        return 1

    if not auvMode:
        obj = PostTransfer(interface, multiMode=multiMode)
        obj.kickoffAuv()
        obj.rsyncAuvData()
        obj.moveTopsideData()
        obj.concatReadmes()
    else:
        #Then stuff needs to happen on the iver.  We should only get
        #here if a client calls kickoffAuv(), which sends a shell
        #command via ssh...
        obj = PostTransfer(socket.gethostname(), multiMode=multiMode)
        obj.prepAuv(localDiveNumber)

    return 0

if __name__ == '__main__':

    sys.exit(main(sys.argv))
