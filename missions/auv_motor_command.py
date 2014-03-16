#!/usr/bin/env python


import lcm
import sys
import math

sys.path.append('/usr/local/lib/python2.6/dist-packages/perls/lcmtypes')

from acfrlcm.auv_iver_motor_command_t import auv_iver_motor_command_t

lc = lcm.LCM();

msg = auv_iver_motor_command_t()
msg.source = auv_iver_motor_command_t.REMOTE


if (len(sys.argv) > 3):
    msg.main = float(sys.argv[1])
    msg.top = float(sys.argv[2]) / 180 * math.pi
    msg.bottom = float(sys.argv[2]) / 180 * math.pi
    msg.port = float(sys.argv[3]) / 180 * math.pi
    msg.starboard = float(sys.argv[3]) / 180 * math.pi
    
    lc.publish('IVER_MOTOR', msg.encode())
    lc.publish('IVER_MOTOR.TOP', msg.encode())
else:
    print 'Wrong number of args. Expect 3 <RPM> <TOP/BOTTOM> <PORT/STARBOARD>'

