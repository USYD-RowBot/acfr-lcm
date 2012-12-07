#!/usr/bin/env python


import lcm
import sys

sys.path.append('/home/clees/auv/acfr_lcm/build/lib/python2.6/dist-packages/perls/lcmtypes')

from acfrlcm.auv_control_t import auv_control_t

lc = lcm.LCM();

msg = auv_control_t()

msg.depth_mode = auv_control_t.DEPTH_MODE
msg.run_mode = auv_control_t.RUN
msg.heading = 1.57
msg.depth = 10.0
msg.altitude = 0.0
msg.pitch = 0.0
msg.vx = 1.0

lc.publish('AUV_CONTROL', msg.encode())

