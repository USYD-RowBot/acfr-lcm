#!/usr/bin/env python


import lcm
import sys

sys.path.append('/home/auv/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_path_command_t import auv_path_command_t

lc = lcm.LCM();

msg = auv_path_command_t()
msg.depth_mode = auv_path_command_t.DEPTH

if (len(sys.argv) > 4):
    msg.waypoint[0] = float(sys.argv[1])
    msg.waypoint[1] = float(sys.argv[2])
    msg.waypoint[2] = float(sys.argv[3])
    msg.waypoint[3] = 0
    msg.waypoint[4] = 0
    msg.waypoint[5] = 0
    msg.waypoint[6] = float(sys.argv[4])
    
    
    lc.publish('PATH_COMMAND', msg.encode())
else:
    print 'Wrong number of args'

