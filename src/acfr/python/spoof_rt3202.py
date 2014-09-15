#!/usr/bin/python


import lcm
import sys
import math
import time

LCMROOT='/home/auv/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import rt3202_t


olon = 152.092212561092
olat = -23.5074821434718
inc = 1.0 / 111128.0
angle = 0.0

lc = lcm.LCM()

msg = rt3202_t()
msg.r = 0
msg.p = 0
rad = 100
rad2 = 90



while True:
    msg.utime = int(time.time() * 1000000)
    lat = rad * math.cos(math.radians(angle))     
    lon = rad * math.sin(math.radians(angle))
    h = (angle + 90.0) % 360.0
    
    msg.lat = math.radians(olat + lat * inc )
    msg.lon = math.radians(olon + lon * inc)
    msg.h = math.radians(h)
    
    
    
    lc.publish("RT3202", msg.encode())
    
    angle = angle + 0.5
    
    time.sleep(0.2)
