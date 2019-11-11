#!/usr/bin/python

import lcm
import sys
import argparse
import time
import math

sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from senlcm import rdi_control_t, micron_sounder_t, rdi_pd5_t

def publish_range_update(lcmh, new_range):
    msg = rdi_control_t()

    # to microseconds
    msg.utime = int(time.time() * 1000000)

    msg.command = rdi_control_t.RANGE
    msg.d = new_range

    lcmh.publish("RDI_CONTROL", msg.encode())


class AltitudeEstimator(object):
    def __init__(self):
        self.times = []
        self.altitudes = []

    def altitude_observations(self, channel, data):
        # for each message attempt the possible
        # feed-in types
        altitude = None
        utime = None
        try:
            ms_msg = micron_sounder_t.decode(data)

            # TODO: perform some validity checks

            altitude = ms_msg.altitude
            utime = ms_msg.utime
        except ValueError:
            pass

        try:
            pd5_msg = rdi_pd5_t.decode(data)

            # TODO: add some extra checks for valid range

            altitude = pd5_msg.pd4.altitude
            utime = pd5_msg.utime
        except ValueError:
            pass

        # stash altitude and time for later
        self.times.append(utime)
        self.altitudes.append(altitude)

    def compute_and_clear(self):
        try:
            average_altitude = sum(self.altitudes) / len(self.altitudes)
        except:
            return 100


        self.times = list()
        self.altitudes = list()

        return average_altitude


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--vehicle-name', type=str, default="DEFAULT")
    parser.add_argument('-c', '--channel', type=str, default=None)
    parser.add_argument('--max-range', type=float, default=60)

    args = parser.parse_args()

    vehicle_name = args.vehicle_name

    if args.channel is None:
        print "No channel supplied."
        quit()

    lcmh = lcm.LCM()

    altitude_estimator = AltitudeEstimator()

    def update_dvl_range(channel, data):
        alt = altitude_estimator.compute_and_clear()
        max_range = alt * 1.25
        binned_max_range = min([args.max_range, max([math.ceil(max_range / 5.0) * 5.0, 10])])
        publish_range_update(lcmh, binned_max_range)
        print alt, binned_max_range


    lcmh.subscribe(args.channel, altitude_estimator.altitude_observations)
    lcmh.subscribe("HEARTBEAT_1HZ", update_dvl_range)

    while True:
        lcmh.handle()
