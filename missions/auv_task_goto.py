#!/usr/bin/env python


import lcm
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_global_planner_t import auv_global_planner_t

lc = lcm.LCM();

if (len(sys.argv) >= 8):
    x = sys.argv[2]
    y = sys.argv[3]
    z = sys.argv[4]
    hdg = sys.argv[5]
    vel = sys.argv[6]
    timeout = sys.argv[7] 
    msg = auv_global_planner_t()
    msg.point2_x = float(x)
    msg.point2_y = float(y)
    msg.point2_z = float(z)
    msg.point2_att[2] = float(hdg)
    msg.velocity[1] = float(vel)
    msg.timeout = float(timeout)
    msg.command = auv_global_planner_t.GOTO
    msg.str = '<mission><primitive><goto><position x="'+x+'" y="'+y+'" z="'+z+'"/><heading deg="'+hdg+'" /><timeout t="'+timeout+'" /><velocity x="'+vel+'" /><depth mode = "depth"/></goto></primitive></mission>'

    print 'Sending goto ({}, {}, {})'.format(msg.point2_x, msg.point2_y, msg.point2_z) 
    
    lc.publish('TASK_PLANNER_COMMAND.'+sys.argv[1], msg.encode())
else:
    print "Usage: auv_task_goto platform_name x y z hdg vel timeout"

