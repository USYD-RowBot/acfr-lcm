#!/usr/bin/env python 
#############################################################
# Script for triggering cameras
# 
# Usage: 
#       python camcommand.py command [value]
#           commands : start, stop, path
#
# History
#   2011-06-02      ALF         Created
#   2011-06-04      CL          Added freq + width commands
# 
#############################################################



import lcm
import time
import threading
import sys

sys.path.append('/home/auv/git/acfr_lcm/build/lib/python2.6/dist-packages/perls/lcmtypes')
from acfrlcm.auv_camera_control_t import auv_camera_control_t


LOG_START = 1
LOG_STOP = 2
SET_PATH = 3

lc = lcm.LCM()

def print_usage():
	print "Usage: cam_control command [value]"
	print "		commands : start, stop, path"
	exit



if (len(sys.argv) > 1):
    msg = auv_camera_control_t()
    msg.utime = int(time.time() * 1000000)
    
    if( sys.argv[1] == 'start'):
        msg.command = LOG_START
    elif(sys.argv[1] == 'stop'):
        msg.command = LOG_STOP
    elif(sys.argv[1] == 'path'):
        if(len(sys.argv) < 3):
            print_usage()
        msg.command = SET_PATH
        msg.path = sys.argv[2]
    else:
        print_usage()
        
    lc.publish("CAMERA_CONTROL", msg.encode())
    
else :
	print_usage()
	print "No argument found"
	exit
    

