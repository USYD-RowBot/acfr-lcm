#!/usr/bin/python


import lcm
import sys
import math
import time

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from acfrlcm import auv_acfr_nav_t
from senlcm import parosci_t, tcm_t
from perllcm import heartbeat_t

class NoFilter(object):
    def __init__(self, lcm_handle):
        lcm_handle.subscribe("TCM", self.tcm)
        lcm_handle.subscribe("PAROSCI", self.parosci)
        lcm_handle.subscribe("HEARTBEAT_10HZ", self.heartbeat)
        self.lcm = lcm_handle

        self.nav = auv_acfr_nav_t()
        self.nav.vz = 0.0

        self.last_depth = 0.0
        self.last_depth_time = None

    def tcm(self, channel, data):
        msg = tcm_t.decode(data)
        self.nav.pitch = msg.pitch# + 5 * math.pi / 180.0
        self.nav.heading = msg.heading
        self.nav.roll = msg.roll

    def parosci(self, channel, data):
        msg = parosci_t.decode(data)
        self.nav.depth = msg.depth

        alpha = 0.1

        if self.last_depth_time is not None:
            self.nav.vz = self.nav.vz * alpha + (1.0 - alpha) * (msg.depth - self.last_depth) * 1e6 / (msg.utime - self.last_depth_time)
        self.last_depth = msg.depth
        self.last_depth_time = msg.utime

    def heartbeat(self, channel, data):
        msg = heartbeat_t.decode(data)
        if self.last_depth_time is not None:
            self.nav.depth = self.last_depth + self.nav.vz * (msg.utime - self.last_depth_time) / 1e6
        self.nav.utime = msg.utime
        self.lcm.publish("ACFR_NAV", self.nav.encode())

lcm_handle = lcm.LCM()
nf = NoFilter(lcm_handle)
while True:
    lcm_handle.handle()
    
