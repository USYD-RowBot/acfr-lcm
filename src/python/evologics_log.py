#!/usr/bin/env python
import lcm
import sys

sys.path.append('/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from senlcm import evologics_modem_t

lc = lcm.LCM()

def handler(channel, data):
    msg = evologics_modem_t.decode(data)
    print "==========="
    print msg.data[:-2]
    
lc.subscribe('.*\.EVOLOGICS_LOG', handler)
while True:
    try:
        lc.handle()
    except IOError:
        break
