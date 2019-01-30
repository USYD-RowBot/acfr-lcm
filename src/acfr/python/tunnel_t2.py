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
	[000, 000, 2000, 2000, 100],
	[0, 0, 0, 0, 110],
	[2000, 2000, 000, 000, 100],
	[500, 500, 500, 500, 100],
	[1500, 1500, 1500, 1500, 100],
	[-500, -500, -500, -500, 100],
	[-1500, -1500, -1500, -1500, 100],
#[1500, 1500, -1500, -1500, 20],
	#[0, 1800, 0, 1800, 30],
]


lc = lcm.LCM();

msg = auv_nga_motor_command_t()

thrust1 = 1000
thrust2 = 1000

while True:
        msg.utime = int(time.time() * 1000000)
        msg.lat_fore = thrust1
        msg.vert_fore = thrust2
        msg.lat_aft = thrust1
        msg.vert_aft = thrust2
        lc.publish('NGA.NEXTGEN_MOTOR', msg.encode())
        time.sleep(.1)		
		
