#!/usr/bin/env python


import lcm
import sys
import time
import numpy

sys.path.append('/home/auv/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_bluefin_tail_command_t import auv_bluefin_tail_command_t

lc = lcm.LCM();

msg = auv_bluefin_tail_command_t()
for r in numpy.arange(-12, 12, 1.0):
	for e in numpy.arange(-12, 12, 1.0):
    		msg.utime = int(time.time() * 1000000)
    		msg.main = 0
    		msg.rudder = r/180*3.15
    		msg.elevator = e/180*3.14
    		lc.publish('BLUEFIN_COMMAND', msg.encode())
		print 'r:{} e:{}'.format(r,e)
		time.sleep(.15)

