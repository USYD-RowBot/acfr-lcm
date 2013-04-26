#!/usr/bin/env python


import lcm
import sys
import math

sys.path.append('/home/clees/auv/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_control_t import auv_control_t

lc = lcm.LCM();

msg = auv_control_t()
msg.depth_mode = auv_control_t.DEPTH_MODE
msg.run_mode = auv_control_t.RUN

if (len(sys.argv) > 3):
    msg.heading = float(sys.argv[2]) / 180 * math.pi
    msg.depth = float(sys.argv[3])
    msg.altitude = float(sys.argv[3])
    msg.pitch = float(sys.argv[3]) / 180 * math.pi
    msg.vx = float(sys.argv[1])

    lc.publish('AUV_CONTROL', msg.encode())
else:
    print 'Wrong number of args'

