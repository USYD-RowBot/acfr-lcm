#!/usr/bin/env python

import lcm
import sys


sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm import auv_vis_rawlog_t, auv_acfr_nav_t

class ImageAltitude(object):
    def __init__(self, lcm_source=None):
        if lcm_source is None:
            lcm_source = ""
        else:
            lcm_source = "file://{}?speed=0".format(lcm_source) or ""
        self.lcmhandler = lcm.LCM(lcm_source)
        self.last_alt = None

        # accept with/without vehicle names
        self.lcmhandler.subscribe(".*ACFR_NAV", self.handle_nav)
        self.lcmhandler.subscribe(".*AUV_VIS_RAWLOG", self.handle_vis)

    def handle_nav(self, channel, data):
        msg = auv_acfr_nav_t.decode(data)
        self.last_alt = msg.altitude

    def handle_vis(self, channel, data):
        msg = auv_vis_rawlog_t.decode(data)
        print "{} {: >6.3f}ms {:.2f}m".format(msg.image_name, msg.exp_time / 1000.0, self.last_alt)

    def run(self):
        while True:
            try:
                self.lcmhandler.handle_timeout(100)
            except IOError:
                break

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # can pass in a filename
        ia = ImageAltitude(sys.argv[1])
    else:
        # or log it live
        ia = ImageAltitude()
    ia.run()

