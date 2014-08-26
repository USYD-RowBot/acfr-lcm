#!/usr/bin/env python


import lcm
import sys
import math

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_control_t import auv_control_t

lc = lcm.LCM();

msg = auv_control_t()
msg.depth_mode = auv_control_t.DEPTH_MODE
msg.run_mode = auv_control_t.RUN

if (len(sys.argv) > 3):
    msg.vx = float(sys.argv[1])
    msg.heading = float(sys.argv[2]) / 180 * math.pi
    # Only one of these will ever be use depending on the depth_mode (set above)
    msg.depth = float(sys.argv[3])
    msg.altitude = float(sys.argv[3])
    msg.pitch = float(sys.argv[3]) / 180 * math.pi
    

    lc.publish('AUV_CONTROL', msg.encode())
else:
    print 'Wrong number of args'
