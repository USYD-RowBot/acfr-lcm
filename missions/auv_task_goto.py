#!/usr/bin/env python


import lcm
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_global_planner_t import auv_global_planner_t

lc = lcm.LCM();

if (len(sys.argv) >= 8):
    msg = auv_global_planner_t()
    msg.point2_x = float(sys.argv[2])
    msg.point2_y = float(sys.argv[3])
    msg.point2_z = float(sys.argv[4])
    msg.point2_att[2] = float(sys.argv[5])
    #msg.depth_mode = float(sys.argv[6])
    msg.velocity[1] = float(sys.argv[6])
    msg.timeout = float(sys.argv[7])
    msg.command = auv_global_planner_t.GOTO
    #msg.str = '<mission><primitive><goto><position x="'+x+'" y="'+y+'" z="'+z+'"/><heading deg="'+hdg+'" /><timeout t="'+timeout+'" /><velocity x="'+vel+'" /><depth mode = "'+depth_mode+'" /><command device="camera" onoff="'+camera_cmd+'"/></goto></primitive></mission>'

    print 'Sending goto ({}, {}, {})'.format(msg.point2_x, msg.point2_y, msg.point2_z) 
    
    lc.publish('TASK_PLANNER_COMMAND.'+sys.argv[1], msg.encode())
else:
    print "Usage: auv_task_goto platform_name x y z hdg vel timeout"

