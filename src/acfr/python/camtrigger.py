#!/usr/bin/env python 
#############################################################
# Script for triggering cameras
# 
# Usage: 
#       python camtrigger.py command [value]
#           commands : start, stop, freq, width
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

#import acfrlcm
#from senlcm.raw_t import raw_t
#from senlcm.raw_ascii_t import raw_ascii_t
sys.path.append('/home/auv/git/acfr_lcm/build/lib/python2.6/dist-packages/perls/lcmtypes')
from acfrlcm.auv_camera_trigger_t import auv_camera_trigger_t

lc = lcm.LCM()

SET_FREQ = 1
SET_DELAY = 2
SET_WIDTH = 3
SET_STATE = 4
SET_ALL = 5

def print_usage():
	print "Usage: camtrigger command [value]"
	print "		commands : start, stop, freq, width"
	exit



if (len(sys.argv) > 1):
	msg = auv_camera_trigger_t()
	msg.command = SET_STATE
	msg.freq = 1
	msg.pulseWidthUs = -1
	msg.strobeDelayUs = 1
	msg.utime = int(time.time() * 1000000)
    
	if( sys.argv[1] == 'start'):
		msg.enabled = 1
	elif(sys.argv[1] == 'stop'):
		msg.enabled = 0
	elif(sys.argv[1] == 'freq'):
		if(len(sys.argv) < 3):
			print_usage()
		else:
			msg.command = SET_FREQ
			msg.freq = int(sys.argv[2])
	elif(sys.argv[1] == 'width'):
		if(len(sys.argv) < 3):
			print_usage();
		else:	
			msg.command = SET_WIDTH
			msg.pulseWidthUs = int(sys.argv[2])
 	else:
		print_usage()
        
	lc.publish("CAMERA_TRIGGER", msg.encode())
    
else :
	print_usage()
	print "No argument found"
	exit
    

