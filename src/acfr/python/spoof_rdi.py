#!/usr/bin/python


import lcm
import sys
import time

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import rdi_pd5_t

lc = lcm.LCM()

msg = rdi_pd5_t()

msg.depth = 0.0
msg.salinity = 13
msg.pitch = 0.0
msg.roll = 0.0
msg.heading = 0.0
msg.dmg_wtv = (0.0, 0.0, 0.0, 1.0)
msg.dmg_btv = (0.0, 0.0, 0.0, 1.0)

msg.pd4.btv = (0.0, 0.0, 0.0, 1.0)
msg.pd4.btv_status = 0
msg.pd4.wtv = (0.0, 0.0, 0.0, 1.0)
msg.pd4.wtv_status = 0

msg.pd4.range = (5.0, 5.0, 5.0, 5.0)
msg.pd4.builtin_test = (0, 0)

msg.pd4.altitude = 4.0
msg.pd4.speed_of_sound = 1535



while True:
    msg.utime = int(time.time() * 1000000)
    msg.pd4.utime = msg.utime
 
    lc.publish("NGA.RDI", msg.encode())
    
    time.sleep(1)
