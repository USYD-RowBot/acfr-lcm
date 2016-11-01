#!/usr/bin/env python


import lcm
import sys
import time

sys.path.append('/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.tunnel_thruster_command_t import tunnel_thruster_command_t

# [start_speed, end_speed,  seconds]
# seed is a number detween -2048 and 2047
tests = [
#	[100, 100, 30],
	[100, 500, 30]
]


lc = lcm.LCM();

msg = tunnel_thruster_command_t()

for test in tests:
    inc = (test[1] - test[0]) / test[2]
    for t in range(0, test[2]+1):
        thrust = (t * inc) + test[0]
        msg.utime = int(time.time() * 1000000)
        msg.fore_horiz = thrust
        msg.fore_vert = thrust
        msg.aft_horiz = thrust
        msg.aft_vert = thrust
        lc.publish('TUNNEL_THRUSTER_COMMAND', msg.encode())
        time.sleep(1)		
		



