#!/usr/bin/env python


import lcm
import sys
import math

sys.path.append('/usr/local/lib/python2.6/dist-packages/perls/lcmtypes')

from acfrlcm.auv_control_t import auv_control_t

lc = lcm.LCM();

msg = auv_control_t()
msg.depth_mode = auv_control_t.DEPTH_MODE
msg.run_mode = auv_control_t.STOP
msg.vx = 0.0

lc.publish('AUV_CONTROL', msg.encode())

