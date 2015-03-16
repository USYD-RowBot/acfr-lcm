#!/usr/bin/python


import lcm
import sys
import math
import time

LCMROOT='/home/stefanw/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import novatel_t, usbl_fix_t
from acfrlcm import auv_status_t


olon = 151.254255
olat = -33.839919
inc = 1.0 / 111128.0

angle = 0.0

rad = 90

lc = lcm.LCM()

msg = auv_status_t()

count = 0

while True:
    angle = 0.0
    msg.utime = int(time.time() * 1000000)
    
    lat = rad * math.cos(math.radians(angle))     
    lon = rad * math.sin(math.radians(angle))
    h = (angle + 90.0) % 360.0
    
    msg.latitude = math.radians(olat + lat * inc )
    msg.longitude = math.radians(olon + lon * inc)
    msg.status = 1<<9;
    msg.target_id = 3;

    lc.publish("AUV_STATUS.SIRIUS", msg.encode())
    
    angle = angle + 0.5
    
    time.sleep(5.0)
