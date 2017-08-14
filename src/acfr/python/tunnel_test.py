#!/usr/bin/env python


import lcm
import sys
import time

sys.path.append('/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.tunnel_thruster_command_t import tunnel_thruster_command_t

# [start_speed, end_speed,  seconds]
# seed is a number detween -2048 and 2047
tests = [
	[0, 2000, 0, 0, 5],
	[2000, 2000, 0, 0, 20],
	[0, 0, 0, 2000, 5],
	[0, 0, 2000, 2000, 20],
	[500, 500, 500, 500, 60],
	[1500, 1500, 1500, 1500, 60],
	[-500, -500, -500, -500, 60],
	[-1500, -1500, -1500, -1500, 60],
	#[1500, 1500, -1500, -1500, 20],
	#[0, 1800, 0, 1800, 30],
]


lc = lcm.LCM();

msg = tunnel_thruster_command_t()

for test in tests:
    inc1 = (test[1] - test[0]) / test[4]
    inc2 = (test[2] - test[3]) / test[4]
    for t in range(0, test[4]+1):
        thrust1 = (t * inc1) + test[0]
        thrust2 = (t * inc2) + test[2]
        msg.utime = int(time.time() * 1000000)
        msg.fore_horiz = thrust1
        msg.fore_vert = thrust2
        msg.aft_horiz = thrust1
        msg.aft_vert = thrust2
        lc.publish('TUNNEL_THRUSTER_COMMAND', msg.encode())
        time.sleep(1)		
		



