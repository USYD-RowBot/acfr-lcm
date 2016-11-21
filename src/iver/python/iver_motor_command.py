#!/usr/bin/env python


import lcm
import sys

sys.path.append('/home/auv/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_iver_motor_command_t import auv_iver_motor_command_t

lc = lcm.LCM();

msg = auv_iver_motor_command_t()
msg.source = auv_iver_motor_command_t.AUTO

if (len(sys.argv) > 5):
    msg.top = float(sys.argv[2])
    msg.bottom = float(sys.argv[2])
    msg.port = float(sys.argv[3])
    msg.starboard = float(sys.argv[4])
    msg.main = float(sys.argv[1])

    lc.publish('IVER_MOTOR', msg.encode())
else:
    print 'Wrong number of args'

