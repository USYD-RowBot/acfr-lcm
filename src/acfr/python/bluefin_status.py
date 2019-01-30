#!/usr/bin/env python
import lcm
import sys


sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import auv_bluefin_tail_status_t



def handle_acfr_psu(channel_name, data):
    msg = auv_bluefin_tail_status_t.decode(data)
    status = "{}V {}A {}W".format(msg.voltage, msg.current, msg.voltage * msg.current)
    print status


if __name__ == '__main__':
    lc = lcm.LCM()
    lc.subscribe('NGA.BLUEFIN_STATUS', handle_acfr_psu)
    while(1):
        lc.handle()

