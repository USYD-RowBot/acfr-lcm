#!/usr/bin/env python
import lcm
import sys


sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from senlcm import  acfr_psu_t


volts = {}
amps = {}
temp = {}

def handle_acfr_psu(channel_name, data):
    global volts, amps, temp
    psu = acfr_psu_t.decode(data)
    volts[psu.address] = psu.voltage
    amps[psu.address] = psu.current
    temp[psu.address] = psu.temperature

    out_str = ''
    total_power = 0
    for key in volts.keys():
        power = volts[key] * amps[key]
        total_power += power
        out_str += '{:2.2f}V\t{:3.2f}A\t{:3.2f}W\t{:2.2f}C\t\t'.format(volts[key], amps[key], power, temp[key])
    print out_str + '{:3.2f}W'.format(total_power)

if __name__ == '__main__':
    lc = lcm.LCM()
    lc.subscribe('NGA.PSU.*', handle_acfr_psu)
    while(1):
        lc.handle()

