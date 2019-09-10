#!/usr/bin/env python
import lcm
import sys
import time


sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from senlcm import  evologics_raw_message_t


if __name__ == "__main__":
    msg = evologics_raw_message_t()

    msg.utime = long(time.time())*1000000
    msg.msg = sys.argv[2]

    lc = lcm.LCM()

    channel = "EVOLOGICS_RAW_MESSAGE"

    lc.publish(channel, msg.encode())
