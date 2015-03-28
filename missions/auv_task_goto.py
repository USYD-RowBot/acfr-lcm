#!/usr/bin/env python


import lcm
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_global_planner_t import auv_global_planner_t

lc = lcm.LCM();

if (len(sys.argv) >= 9):
    x = sys.argv[1]
    y = sys.argv[2]
    z = sys.argv[3]
    hdg = sys.argv[4]
    depth = sys.argv[5]
    depth_mode = sys.argv[6]
    vel = sys.argv[7]
    timeout = sys.argv[8]
    camera_cmd = sys.argv[9]
    msg = auv_global_planner_t()
    msg.command = auv_global_planner_t.GOTO
    msg.str = '<mission><primitive><goto><position x="'+x+'" y="'+y+'" z="'+z+'"/><heading deg="'+hdg+'" /><timeout t="'+timeout+'" /><velocity x="'+vel+'" /><depth mode = "'+depth_mode+'" /><command device="camera" onoff="'+camera_cmd+'"/></goto></primitive></mission>'

    print msg.str 
    
    lc.publish('TASK_PLANNER_COMMAND.SIRIUS', msg.encode())
else:
    print "Usage: auv_task_goto x y z hdg depth depth_mode vel timeout camera_cmd"

