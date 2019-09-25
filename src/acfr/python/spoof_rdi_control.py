#!/usr/bin/python
import lcm
import sys
import time
import math

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))

from senlcm import rdi_control_t

lc = lcm.LCM();

msg = rdi_control_t()

msg.command = 2
msg.d = 30

msg.utime = int(time.time() * 1000000)
lc.publish('RDI_CONTROL', msg.encode())
print "Sending rdi control msg"
