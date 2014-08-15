#!/usr/bin/python

import lcm
import time
import sys

LCMROOT='/home/auv/git/acfr_lcm'
sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import ysi_t

count = 0
depths = []

def ysi_handler(channel, data):
    global count
    msg = ysi_t.decode(data)
    count = count + 1
    depths.append(msg.depth)
    
    
lc = lcm.LCM()
lc.subscribe("YSI", ysi_handler)
print "Collecting 10 samples"
while count < 10:
    lc.handle()

avg = sum(depths)/len(depths)
print "Tare depth = {}".format(avg)
