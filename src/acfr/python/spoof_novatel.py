#!/usr/bin/python


import lcm
import sys
import math
import time

LCMROOT='/home/auv/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import novatel_t, usbl_fix_t


olat = 152.092212561092
olon = -23.5074821434718
inc = 1.0 / 111128.0
angle = 0.0

lc = lcm.LCM()

msg = novatel_t()
msg.roll = 0
msg.pitch = 0
rad = 100

msg2 = usbl_fix_t()

while True:
    msg.utime = int(time.time() * 1000000)
    msg2.utime = int(time.time() * 1000000)
    lon = rad * math.cos(math.radians(angle))     
    lat = rad * math.sin(math.radians(angle))
    h = (angle + 90.0) % 360.0
    
    msg.latitude = olat + lat * inc 
    msg.longitude = olon + lon * inc
    msg.heading = h
    
    lon = rad * math.cos(math.radians(angle-45))     
    lat = rad * math.sin(math.radians(angle-45))
    h = (angle + 45) % 360.0
    
    msg2.latitude = olat + lat * inc 
    msg2.longitude = olon + lon * inc
    msg2.accuracy = 10
    
    
    lc.publish("NOVATEL", msg.encode())
    lc.publish("USBL_FIX", msg2.encode())
    
    angle = angle + 0.5
    
    time.sleep(0.2)
