#!/usr/bin/python


import lcm
import sys
import time

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))

from acfrlcm import auv_control_t

lc = lcm.LCM();

msg = auv_control_t()
msg.depth_mode = auv_control_t.DEPTH_MODE
msg.run_mode = auv_control_t.DIVE

if (len(sys.argv) > 1):
    msg.heading = float(5.31)
    msg.depth = float(1.0)
    msg.altitude = float(0.0)
    msg.pitch = float(0.0)
    msg.vx = float(0.0)

    msg.utime = int(time.time() * 1000000)

    lc.publish('AUV_CONTROL', msg.encode())
    print "Sent dive command"

    time.sleep(120)

    msg.utime = int(time.time() * 1000000)
    msg.depth = -0.04
    lc.publish('AUV_CONTROL', msg.encode())
    print "Sent surface command"

    time.sleep(5)

    msg.utime = int(time.time() * 1000000)
    msg.run_mode = auv_control_t.STOP
    lc.publish('AUV_CONTROL', msg.encode())
    print "Sent STOP command"
else:
    print 'Wrong number of args'

