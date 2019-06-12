#!/usr/bin/python
import lcm
import sys
import time
import math

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))

from senlcm import micron_sounder_t

lc = lcm.LCM();

msg = micron_sounder_t()

msg.altitude = float(sys.argv[2])

for i in xrange(100):
    msg.utime = int(time.time() * 1000000)
    time.sleep(0.5)

    lc.publish('NGA.MICRON_SOUNDER_'+sys.argv[1], msg.encode())
    print "Sending oas"