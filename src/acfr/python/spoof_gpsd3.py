#!/usr/bin/python


import lcm
import sys
import math
import time

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import gpsd3_t, gpsd3_fix_t, gpsd3_dop_t, gpsd3_devconfig_t


olon = 151.254255
olat = -33.839919
inc = 1.0 / 111128.0
angle = 0.0

lc = lcm.LCM()

msg = gpsd3_t()
rad = 100
rad2 = 90

count = 0

while True:
    msg.utime = int(time.time() * 1000000)
    msg.status = 1

    
    lat = rad * math.cos(math.radians(angle))     
    lon = rad * math.sin(math.radians(angle))
    
    fix = gpsd3_fix_t()
    fix.latitude = math.radians(olat + lat * inc )
    fix.longitude = math.radians(olon + lon * inc)
    fix.mode = 2

    dop = gpsd3_dop_t()
    devconfig = gpsd3_devconfig_t()

    msg.fix = fix
    msg.dop = dop
    msg.dev = devconfig
    
    lc.publish("NGA.GPSD_CLIENT", msg.encode())
    
    angle = angle + 0.5
    
    time.sleep(0.2)
