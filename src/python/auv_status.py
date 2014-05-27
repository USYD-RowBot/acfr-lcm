#!/usr/bin/env python


import lcm
import sys
import time

sys.path.append('/home/clees/auv/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_status_t import auv_status_t

lc = lcm.LCM();

msg = auv_status_t()
msg.utime = int(time.time() * 1000000)
msg.status = 1;
lc.publish('AUV_STATUS', msg.encode())

