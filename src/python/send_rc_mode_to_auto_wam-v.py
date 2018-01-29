#!/usr/bin/python

import sys
import lcm
import time
import readchar
from enum import Enum


sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import auv_spektrum_control_command_t


def Main():

    channel = 'WAMV.SPEKTRUM_CONTROL'

    print('ACFR USYD LCM send_rc_mode_to_auto Vehicle: WAM-V')
    print('Override utility to transfer control from ZERO or RC to AUTO')
    print('Send channel: {}\n'.format(channel))
    print('Proceed with send?   Y/N')

    choice = (readchar.readkey())
    if choice == 'Y':
	
	# Setup LCM
    	lc = lcm.LCM()
	# Create msg structure
    	msg = auv_spektrum_control_command_t()

        # Enter mode value into  message
        # Switch Constants (3 position flick switches)
        # rc_control_source_t enum: RC_MODE_RC = 0, RC_MODE_ZERO = 1, RC_MODE_AUTO = 2
        # REAR_POS_CUTOFF 1300, CENTER_POS_CUTOFF 700
	# > REAR_POS_CUTOFF = RC_MODE_RC, > CENTRE_POS_CUTOFF = RC_MODE_ZERO, < CENTRE_POS_CUTOFF = RC_MODE_AUTO
        # Test values front: 342, center: 1024, rear: 1706 DX6 24012018 JM
        msg.channels = 6
        # msg.values = [RC_THROTTLE, RC_AILERON, RC_ELEVATOR, RC_RUDDER, RC_GEAR, RC_AUX1 = 342 value from test values (see notes above)]
        msg.values= [0, 0, 0, 0, 0, 342]
	msg.utime = long(time.time())*1000000
	
	# and send msg
	lc.publish(channel, msg.encode())
	print('Spektrum Control Command Sent')
        
    exit()

if __name__ == '__main__':
    Main()

