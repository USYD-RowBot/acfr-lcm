#!/usr/bin/python
import lcm
import sys
import time
import math

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))

from acfrlcm import auv_nga_motor_command_t

lc = lcm.LCM();

msg = auv_nga_motor_command_t()

msg.vert_fore = 0
msg.vert_aft = 0
msg.lat_fore = 0
msg.lat_aft = 0
msg.tail_thruster = 50
msg.tail_rudder = 0
msg.tail_elevator = 0


for i in xrange(80):
    msg.utime = int(time.time() * 1000000)
    time.sleep(0.5)

    lc.publish('NGA.NEXTGEN_MOTOR', msg.encode())
    print "Sending Motor Command"

msg.vert_fore = 0
msg.vert_aft = 0
msg.lat_fore = 0
msg.lat_aft = 0
msg.tail_thruster = 0
msg.tail_rudder = 0
msg.tail_elevator = 0

for i in xrange(10):
    msg.utime = int(time.time() * 1000000)

    lc.publish('NGA.NEXTGEN_MOTOR', msg.encode())
    print "Sent STOP command"
    time.sleep(0.5)
