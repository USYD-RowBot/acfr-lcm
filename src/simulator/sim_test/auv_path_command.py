#!/usr/bin/env python


import lcm
import sys

sys.path.append('/home/clees/auv/acfr_lcm/build/lib/python2.6/dist-packages/perls/lcmtypes')

from acfrlcm.auv_path_command_t import auv_path_command_t

lc = lcm.LCM();

msg = auv_path_command_t()
msg.command_type = auv_path_command_t.SINGLE
msg.depth_mode = auv_path_command_t.DEPTH

if (len(sys.argv) > 7):
    msg.wpt_one[0] = float(sys.argv[1])
    msg.wpt_one[1] = float(sys.argv[2])
    msg.wpt_one[2] = float(sys.argv[3])
    msg.wpt_one[3] = 0
    msg.wpt_one[4] = 0
    msg.wpt_one[5] = 0
    msg.wpt_one[6] = float(sys.argv[7])
    
    msg.wpt_two[0] = float(sys.argv[4])
    msg.wpt_two[1] = float(sys.argv[5])
    msg.wpt_two[2] = float(sys.argv[6])
    msg.wpt_two[3] = 0
    msg.wpt_two[4] = 0
    msg.wpt_two[5] = 0
    msg.wpt_two[6] = float(sys.argv[7])

    lc.publish('PATH_COMMAND', msg.encode())
else:
    print 'Wrong number of args'

