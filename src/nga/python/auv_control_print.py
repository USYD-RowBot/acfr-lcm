#!/usr/bin/env python

import lcm
import sys

sys.path.append('/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm import auv_control_t

lc = lcm.LCM();

interval = 1
count = 0


depth_mode = ["DEPTH", "ALT", "PITCH"]
run_mode = ["STOP", "RUN", "DIVE"]

def print_header():
    print "Run Mode\tvx\theading\tDive Mode\tdepth\tpitch\taltitude"

def handle(channel, data):
    global count
    count += 1
    if count % interval == 0:
        msg = auv_control_t.decode(data)

        print "{}\t".format(run_mode(msg.run_mode)),
        print "{}\t{}\t".format(msg.vx, msg.heading),
        print "{}\t".format(depth_mode[msg.depth_mode]),
        print "{}\t{}\t{}".format(msg.depth, msg.pitch, msg.altitude)

        

lc.subscribe("NGA.AUV_CONTROL", handle)

while True:
    lc.handle()
