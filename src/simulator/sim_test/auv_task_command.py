#!/usr/bin/env python


import lcm
import sys

sys.path.append('/home/auv/git/acfr_lcm/build/lib/python2.6/dist-packages/perls/lcmtypes')

from acfrlcm.auv_global_planner_t import auv_global_planner_t

lc = lcm.LCM();

msg = auv_global_planner_t()
msg.command = auv_global_planner_t.LOAD
msg.str = "/home/auv/git/acfr_lcm/src/acfr/global-planner/test.xml"
    
    
lc.publish('TASK_PLANNER_COMMAND', msg.encode())

