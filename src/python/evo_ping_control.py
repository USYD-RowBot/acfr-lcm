#!/usr/bin/env python
import lcm
import sys
import time


sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from senlcm import  evologics_ping_control_t


if __name__ == "__main__":
    msg = evologics_ping_control_t()

    msg.utime = long(time.time())*1000000
    msg.target_id = 2
    msg.send_fixes = True
    msg.send_pings = True
    msg.ping_rate = 10

    lc = lcm.LCM()

    channel = "{}.EVOLOGICS_PING_CONTROL".format(sys.argv[1])

    lc.publish(channel, msg.encode())
