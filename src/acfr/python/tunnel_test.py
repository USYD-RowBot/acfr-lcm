#!/usr/bin/env python


import lcm
import sys
import time

sys.path.append('/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_nga_motor_command_t import auv_nga_motor_command_t

# [start_speed, end_speed,  seconds]
# seed is a number detween -2048 and 2047
tests = [
#	[0, 2000, 0, 0, 5],
	[000, 000, 2000, 2000, 10],
#	[0, 0, 0, 2000, 5],
	[2000, 2000, 000, 000, 10],
	[500, 500, 500, 500, 10],
	[1500, 1500, 1500, 1500, 10],
	[-500, -500, -500, -500, 10],
	[-1500, -1500, -1500, -1500, 10],
	#[1500, 1500, -1500, -1500, 20],
	#[0, 1800, 0, 1800, 30],
]


lc = lcm.LCM();

msg = auv_nga_motor_command_t()

for test in tests:
    inc1 = (test[1] - test[0]) / test[4]
    inc2 = (test[2] - test[3]) / test[4]
    for t in range(0, test[4]+1):
        thrust1 = (t * inc1) + test[0]
        thrust2 = (t * inc2) + test[2]
        msg.utime = int(time.time() * 1000000)
        msg.lat_fore = thrust1
        msg.vert_fore = thrust2
        msg.lat_aft = thrust1
        msg.vert_aft = thrust2
        lc.publish('NGA_MOTOR', msg.encode())
        time.sleep(1)		
		



