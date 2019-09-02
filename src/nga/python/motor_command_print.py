#!/usr/bin/env python

import lcm
import sys

sys.path.append('/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm import auv_nga_motor_command_t

lc = lcm.LCM();

interval = 10
count = 0

def handle(channel, data):
    global count
    count += 1
    if count % interval == 0:
        msg = auv_nga_motor_command_t.decode(data)

        print "Tail (t/r/e):", msg.tail_thruster, msg.tail_rudder, msg.tail_elevator,
        print "Vert (f/a):", msg.vert_fore, msg.vert_aft,
        print "Lat (f/a):", msg.lat_fore, msg.lat_aft

lc.subscribe("NGA.NEXTGEN_MOTOR", handle)

while True:
    lc.handle()
