#!/usr/bin/env python


import lcm
import sys

sys.path.append('/usr/local/lib/python2.6/dist-packages/perls/lcmtypes')

from acfrlcm.auv_global_planner_t import auv_global_planner_t

lc = lcm.LCM();

msg = auv_global_planner_t()
msg.command = auv_global_planner_t.ABORT
msg.str = "MANUAL"
    
    
lc.publish('TASK_PLANNER_COMMAND', msg.encode())

