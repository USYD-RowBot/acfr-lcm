#!/usr/bin/python

import lcm
import sys
import time

LCMROOT='/home/auv/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from acfrlcm import seabotix_joystick_t
 
if (len(sys.argv) > 4):
   
    lc = lcm.LCM()

    msg = seabotix_joystick_t()

    msg.utime = int(time.time() * 1000000) -1000000
    msg.x = int(sys.argv[1])
    msg.y = int(sys.argv[2])
    msg.z = int(sys.argv[3])
    msg.v = int(sys.argv[4])

    lc.publish("SEABOTIX_JOYSTICK", msg.encode())
else:
    print 'Usage: seabotix_joystick.py [x] [y] [z] [v]'
    
