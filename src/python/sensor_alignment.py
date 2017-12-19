#!/usr/bin/env python
import lcm
import sys
import curses
import curses.ascii

try:
    from cStringIO.StringIO import BytesIO
except:
    from io import BytesIO

#sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
sys.path.insert(0, '/home/lachlan/Scratch/auvgit/acfrlcm/build/lib/python2.7/dist-packages/perls/lcmtypes')

import acfrlcm
import senlcm
import bot_core
import bot_frames
import bot_lcmgl
import bot_param
import bot_procman

class SensorCalibration(object):
    def __init__(self, lcmh):
        lcmh.subscribe("TCM", self.handle_tcm)
        lcmh.subscribe("KVH1750", self.handle_imu)
        lcmh.subscribe("RDI", self.handle_rdi)
        lcmh.subscribe("HEARTBEAT_5HZ", self.print_status)

        self.compass_angle = (0.0) * 3
        self.velocity = (0.0) * 3

    def handle_tcm(self, channel_name, raw_data):
        msg = senlcm.tcm_t.decode(raw_data)

        self.compass_angle = (msg.roll, msg.pitch, msg.heading)

    def handle_imu(self, channel_name, raw_data):
        msg = senlcm.kvh1750_t.decode(raw_data)

        # there are many assumptions that can be made here
        # about angle to local gravity potential and rotation of the
        # earth, but only when still
        angular = msg.angular
        acceleration = msg.linear

    def handle_rdi(self, channel_name, raw_data):
        msg = senlcm.rdi_pd5_t.decode(raw_data)

        # RDI only supplies compass information on Sirius
        #angle = (msg.roll, msg.pitch, msg.heading)
        self.velocity = (msg.pd4.btv[1], msg.pd4.btv[0], -msg.pd4.btv[2])


    def print_status(self, channel_name, raw_data):
        print "======================"
        print self.compass_angle
        print self.velocity

if __name__ == '__main__':
    lc = lcm.LCM()

    sc = SensorCalibration(lc)

    while True:
        try:
            lc.handle()
        except IOError:
            break
