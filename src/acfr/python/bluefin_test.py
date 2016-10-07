#!/usr/bin/env python


import lcm
import sys
import time

sys.path.append('/home/auv/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_bluefin_tail_command_t import auv_bluefin_tail_command_t

lc = lcm.LCM();

if (len(sys.argv) == 4):
    msg = auv_bluefin_tail_command_t()
    msg.utime = int(time.time() * 1000000)
    msg.main = int(sys.argv[1])
    msg.rudder = float(sys.argv[2])/180*3.14
    msg.elevator = float(sys.argv[3])/180*3.14

    lc.publish('BLUEFIN_COMMAND', msg.encode())

