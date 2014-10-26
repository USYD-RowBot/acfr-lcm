#!/usr/bin/python


import lcm
import sys
import time

LCMROOT='/home/auv/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import ysi_t

lc = lcm.LCM()

msg = ysi_t()



while True:
    msg.utime = int(time.time() * 1000000)
    msg.depth = 0.0
 
    lc.publish("YSI", msg.encode())
    
    time.sleep(1)
