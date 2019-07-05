#!/usr/bin/env python
import lcm
import sys


sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from acfrlcm import auv_acfr_nav_t
from senlcm import tcm_mag_t


def handle_acfr_nav(channel_name, data):
    msg = auv_acfr_nav_t.decode(data)
    global nav_pitch
    nav_pitch = msg.pitch;

def handle_tcm(channel_name, data):
    msg = tcm_mag_t.decode(data)
    global sen_pitch
    sen_pitch = msg.pitch;


nav_pitch = 0.0
sen_pitch = 0.0

if __name__ == '__main__':
    lc = lcm.LCM()
    lc.subscribe('NGA.ACFR_NAV', handle_acfr_nav)
    lc.subscribe('NGA.TCM_MAG', handle_tcm)
    while(1):
        lc.handle()
        print "{:8.1f} {:8.1f}".format(nav_pitch * 180 / 3.14159, sen_pitch * 180 / 3.14159)

