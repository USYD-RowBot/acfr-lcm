#!/usr/bin/env python


import lcm
import sys

sys.path.append('/home/auv/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_global_planner_t import auv_global_planner_t

lc = lcm.LCM();

msg = auv_global_planner_t()
msg.command = auv_global_planner_t.ABORT
msg.str = ""

lc.publish('TASK_PLANNER_COMMAND.IVERAME', msg.encode())

