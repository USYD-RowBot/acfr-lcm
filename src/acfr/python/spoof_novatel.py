#!/usr/bin/python


import lcm
import sys
import math
import time

LCMROOT='/home/auv/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import novatel_t, usbl_fix_t
from acfrlcm import auv_status_t


olon = 151.254255
olat = -33.839919
inc = 1.0 / 111128.0
angle = 0.0

lc = lcm.LCM()

msg = novatel_t()
msg.roll = 0
msg.pitch = 0
rad = 100
rad2 = 90

msg2 = usbl_fix_t()
msg3 = auv_status_t()

count = 0

while True:
    msg.utime = int(time.time() * 1000000)
    msg2.utime = int(time.time() * 1000000)
    msg2.utime = int(time.time() * 1000000)
    
    lat = rad * math.cos(math.radians(angle))     
    lon = rad * math.sin(math.radians(angle))
    h = (angle + 90.0) % 360.0
    
    msg.latitude = math.radians(olat + lat * inc )
    msg.longitude = math.radians(olon + lon * inc)
    msg.heading = math.radians(h)
    msg.status = "INS_SOLUTION_GOOD"
    
    lon = rad2 * math.cos(math.radians(angle-45))     
    lat = rad2 * math.sin(math.radians(angle-45))
    h = math.radians((angle + 45) % 360.0)
    
    msg2.latitude = math.radians(olat + lat * inc )
    msg2.longitude = math.radians(olon + lon * inc)
    msg2.accuracy = 10
    
    msg3.latitude = math.radians(olat + lat * inc )
    msg3.longitude = math.radians(olon + lon * inc)
    msg3.status = 1<<9;

    lc.publish("NOVATEL", msg.encode())
    #lc.publish("USBL_FIX", msg2.encode())
    
    #if count == 20:
    #    lc.publish("AUV_STATUS.IVER", msg3.encode())
    #    count = 0
    #else:
    #    count = count + 1
    
    angle = angle + 0.5
    
    time.sleep(0.2)
