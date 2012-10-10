#!/usr/bin/env python

import sys
import os
import time
import commands
from optparse import OptionParser

import lcm

import perls

# lcm types
remotehelm_cmd_t = perls.lcmtypes.perllcm.auv_remotehelm_cmd_t
logger_cmd_t = perls.lcmtypes.perllcm.auv_logger_cmd_t

#constants
LCM_LOGGER = 'lcm-logger'
START_STR  = 'start'
PAUSE_STR  = 'pause'

def timestamp_now():
    return int(time.time() * 1e6)

class RemotehelmController():

    def __init__ (self, botParamArgs):
        self.lcm = lcm.LCM()
        self.param = perls.BotParam.BotParam(botParamArgs)

        self.rh_cmd_channel = self.param.get_str ('os-remotehelm.cmd_channel', default='OS_REMOTEHELM_CMD')
        self.logger_cmd_channel = self.param.get_str ('persistent-lcm-logger.cmd_channel', default='PLCM_LOGGER_CMD')

    #Send command for remote helm to start
    def sendStartRh (self, mission_name, sleep_time):
        print 'Starting remotehelm at %s' % time.ctime()
        packet = remotehelm_cmd_t()
        packet.type = packet.START_RH
        packet.mission_name = mission_name
        packet.utime = timestamp_now ()
        packet.start_mission_sleep = sleep_time

        self.lcm.publish(self.rh_cmd_channel, packet.encode())

    #Send command for remote helm to pause
    def sendPauseRh (self):
        print 'Pausing remotehelm at %s' % time.ctime()
        packet = remotehelm_cmd_t()
        packet.type = packet.PAUSE_RH
        packet.utime = timestamp_now ()

        self.lcm.publish(self.rh_cmd_channel, packet.encode())

    #Send start logging command to persistent logger (DEBUG)
    def sendStartLogging (self, lcmlog_name, cam_dir):
        packet = logger_cmd_t()
        packet.type = packet.START_LOGGING
        packet.lcmlog_name = lcmlog_name
        packet.utime = timestamp_now ()
        packet.camlog_dir = cam_dir

        self.lcm.publish(self.logger_cmd_channel, packet.encode())

    #Send stop logging command to persistent logger (DEBUG)
    def sendStopLogging (self, lcmlog_name):
        packet = logger_cmd_t()
        packet.type = packet.STOP_LOGGING
        packet.lcmlog_name = lcmlog_name
        packet.utime = timestamp_now ()

        self.lcm.publish(self.logger_cmd_channel, packet.encode())

if __name__ == '__main__':

    usage = "usage: %prog [options] mission_file"
    parser = OptionParser(usage=usage)
    parser.add_option ('-t',
                       type="string",
                       dest="cmdType",
                       help="remotehelm command type: start|pause (default: %default)",
                       default="start")
    parser.add_option ('-s',
                       type="int",
                       dest="sleepTime",
                       help="integer number of seconds to sleep before mission start (default: %default)",
                       default=0)
    parser.add_option ('--%s' % perls.BotParam.BOTU_PARAM_LONG_OPT_SERVER_NAME,
                       type="string",
                       dest="pserver_name",
                       help="name of bot param server (default: %default)",
                       default='')
    (options, args) = parser.parse_args()

    if not args and options.cmdType == START_STR:
        print "usage: RemotehelmController.py [options] mission_file"
        sys.exit(1)

    botParamArgs = perls.BotParam.bot_param_args_to_pyargv ('none', 'true', options.pserver_name)

    rhc = RemotehelmController(botParamArgs)

    # start remote helm:
    if options.cmdType == START_STR:
        missionFile = args[0]
        rhc.sendStartRh (missionFile, options.sleepTime)
    elif options.cmdType == PAUSE_STR:
        rhc.sendPauseRh ()
    else:
        print "Unknown command type: %s" % options.cmdType
        sys.exit (1)
    
    # For debugging persistent lcmlogger
    # rhc.sendStartLogging('/home/paul/mission-data/scooby_doo.log', '/home/paul/mission-data/images/')
    # rhc.sendStopLogging('scooby_doo.log')
