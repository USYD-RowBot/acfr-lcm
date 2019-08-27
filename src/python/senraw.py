#!/usr/bin/env python
from __future__ import print_function

import lcm
import sys
import curses
import curses.ascii
import time

try:
    from cStringIO.StringIO import BytesIO
except:
    from io import BytesIO

def handle_raw(channel_name, raw_data):
    msg = senlcm.raw_t.decode(raw_data)

    no_control = "".join([ch for ch in msg.data if ord(ch) > 32 or ch == '\n'])
    
    print("\n==================\n{} ***{}***\n".format(msg.utime, no_control))


if __name__ == '__main__':
    sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
    import senlcm

    if len(sys.argv) >= 2:
        channel = sys.argv[1]
    else:
        print("Must specify a channel and optional lcm log")
        quit()

    if len(sys.argv) == 3:
        lcm_url = "file://"+sys.argv[2]+"?speed=0"
    else:
        lcm_url = ""

    lc = lcm.LCM(lcm_url)

    lc.subscribe(channel, handle_raw)

    while True:
        try:
            lc.handle_timeout(100)
        except IOError:
            break

