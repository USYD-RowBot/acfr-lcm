#!/usr/bin/env python


import lcm
import sys
import time

sys.path.append('/home/auv/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_control_t import auv_control_t

lc = lcm.LCM();

msg = auv_control_t()
msg.depth_mode = auv_control_t.DEPTH_MODE
msg.run_mode = auv_control_t.DIVE

if (len(sys.argv) > 1):
    msg.heading = float(0.0)
    msg.depth = float(0.5)
    msg.altitude = float(0.0)
    msg.pitch = float(0.0)
    msg.vx = float(0.0)

    lc.publish('AUV_CONTROL', msg.encode())

    time.sleep(10)

    msg.depth = 0.0

    lc.publish('AUV_CONTROL', msg.encode())
else:
    print 'Wrong number of args'

