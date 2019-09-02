#!/usr/bin/env python


import lcm
import sys
import time


sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes/')

from acfrlcm.auv_control_t import auv_control_t

lc = lcm.LCM();

msg = auv_control_t()
msg.utime = int(time.time() * 1e6)


msg.depth_mode = auv_control_t.DEPTH_MODE
msg.run_mode = auv_control_t.RUN
msg.heading = -1.57/2
msg.depth = 1.5
msg.altitude = 0.0
msg.pitch = 0.0
msg.vx = 1.0

lc.publish('SIRIUS.AUV_CONTROL', msg.encode())

