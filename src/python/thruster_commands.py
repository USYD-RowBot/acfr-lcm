#!/usr/bin/env python
import lcm
import sys


sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import auv_nga_motor_command_t


def handle_acfr_psu(channel_name, data):
    msg = auv_nga_motor_command_t.decode(data)
    print msg.vert_fore, msg.vert_aft

if __name__ == '__main__':
    lc = lcm.LCM()
    lc.subscribe('NGA.NEXTGEN_MOTOR', handle_acfr_psu)
    while(1):
        lc.handle()

