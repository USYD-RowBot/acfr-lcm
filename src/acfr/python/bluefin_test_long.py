#!/usr/bin/env python


import lcm
import sys
import time
import numpy

sys.path.append('/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_nga_motor_command_t import auv_nga_motor_command_t

lc = lcm.LCM();
start_time = time.time()

if (len(sys.argv) == 1):
    for t in numpy.arange(-950, 950, 20):
        if (abs(t) > 500):
            for e in numpy.arange(-12, 12, 1.0):
                for r in numpy.arange(-12, 12, 1.0):
                    msg = auv_nga_motor_command_t()
                    msg.utime = int(time.time() * 1000000)
                    msg.tail_thruster = t
                    msg.tail_rudder = r/180*3.15
                    msg.tail_elevator = e/180*3.14
                    msg.vert_fore = 0
                    msg.vert_aft = 0
                    msg.lat_fore = 0
                    msg.lat_aft = 0
                    lc.publish('NGA.NEXTGEN_MOTOR', msg.encode())
                    time_so_far = time.time() - start_time
                    print(time_so_far)
                    time.sleep(.15)

if (len(sys.argv) == 4):
    msg = auv_nga_motor_command_t()
    msg.utime = int(time.time() * 1000000)
    msg.tail_thruster = int(sys.argv[1])
    msg.tail_rudder = float(sys.argv[2])/180*3.14
    msg.tail_elevator = float(sys.argv[3])/180*3.14
    msg.vert_fore = 0
    msg.vert_aft = 0
    msg.lat_fore = 0
    msg.lat_aft = 0

    lc.publish('NGA.NEXTGEN_MOTOR', msg.encode())
elapsed_time = time.time() - start_time
print(elapsed_time)
