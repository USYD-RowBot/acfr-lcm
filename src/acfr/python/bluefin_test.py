#!/usr/bin/env python


import lcm
import sys
import time
import numpy

sys.path.append('/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_nga_motor_command_t import auv_nga_motor_command_t

lc = lcm.LCM();

if (len(sys.argv) == 1):
    for e in numpy.arange(0.0, -600, -50.0):
        for r in numpy.arange(-12, 12, 6.0):
            msg = auv_nga_motor_command_t()
            msg.utime = int(time.time() * 1000000)
            msg.tail_thruster = e
            msg.tail_rudder = r/180*3.15
            msg.tail_elevator = 0
            msg.vert_fore = 0
            msg.vert_aft = 0
            msg.lat_fore = 0
            msg.lat_aft = 0
            lc.publish('NGA.NEXTGEN_MOTOR', msg.encode())
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

