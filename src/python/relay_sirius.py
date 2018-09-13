#!/usr/bin/env python

import lcm
import time
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import relay_command_t


channel = 'SIRIUS.RELAY_CONTROL'

if len(sys.argv) < 2:
    print "Usage: relay_sirius [relay number] [relay_state]"
    sys.exit()

# Setup LCM
lc = lcm.LCM()

msg = relay_command_t()
msg.utime = time.time() * 1e6
msg.relay_number = int(sys.argv[1])
msg.relay_request = int(sys.argv[2])
lc.publish(channel, msg.encode())



