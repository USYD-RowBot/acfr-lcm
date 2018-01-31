#!/usr/bin/env python


import lcm
import sys
import math

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_spektrum_control_command_t import auv_spektrum_control_command_t 

lc = lcm.LCM();

msg = auv_spektrum_control_command_t()
msg.channels = 6
msg.values = [0]*6
msg.values[auv_spektrum_control_command_t.RC_AUX1] = 1500

if (len(sys.argv) > 1):
    vehicle_name = sys.argv[1]

    lc.publish(vehicle_name+'.SPEKTRUM_CONTROL', msg.encode())
else:
    print 'Wrong number of args'
