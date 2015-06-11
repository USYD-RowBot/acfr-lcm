#!/usr/bin/python


import lcm
import sys
import math
import time

LCMROOT='/home/auv/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import evologics_usbl_t



lc = lcm.LCM()

msg = evologics_usbl_t()

count = 0


msg.utime = int(time.time() * 1000000) -1000000
msg.x = 100
msg.y = 100
msg.z = 20
msg.accuracy=1
msg.remote_id = 3
lc.publish("EVOLOGICS_USBL", msg.encode())

