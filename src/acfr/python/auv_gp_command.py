#!/usr/bin/env python


import lcm
import sys

sys.path.append('/home/clees/auv/acfr_lcm/build/lib/python2.6/dist-packages/perls/lcmtypes')

from acfrlcm.auv_global_planner_t import auv_global_planner_t

lc = lcm.LCM();

msg = auv_global_planner_t()
msg.command = auv_global_planner_t.LOAD
msg.str = "/home/clees/auv/acfr_lcm/src/acfr/global-planner/test.xml"

#if (len(sys.argv) > 4):
#    msg.waypoint[0] = float(sys.argv[1])
#    msg.waypoint[1] = float(sys.argv[2])
#    msg.waypoint[2] = float(sys.argv[3])
#    msg.waypoint[3] = 0
#    msg.waypoint[4] = 0
#    msg.waypoint[5] = 0
#    msg.waypoint[6] = float(sys.argv[4])
    
    
lc.publish('TASK_PLANNER_COMMAND', msg.encode())
#else:
#    print 'Wrong number of args'

