#!/usr/bin/python


import lcm
import sys

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import usbl_fix_short_t
from acfrlcm import auv_status_short_t
from urioce_lcm import float_acomm_status_t

lat = None 
lon = None

def usblHandler(channel, data):
    global lat, lon
    msg = usbl_fix_short_t.decode(data)
    print 'Received usb_fix_short_t on channel ' + channel
    
    lat =  msg.latitude
    lon = msg.longitude


def pfloatHandler(channel, data):
    global lat, lon
    pfloatMsg = float_acomm_status_t.decode(data)
    print 'Received float_acomm_status_t on channel ' + channel

    if lat is not None and lon is not None:

        auvStatMsg = auv_status_short_t()
        auvStatMsg.altitude = pfloatMsg.altitude
        auvStatMsg.depth = pfloatMsg.depth
        auvStatMsg.charge = pfloatMsg.bat_charge
        auvStatMsg.waypoint = pfloatMsg.leg_num
        auvStatMsg.latitude = lat
        auvStatMsg.longitude = lon
        auvStatMsg.vel = 5

        lc.publish("PFLOAT.AUVSTAT", auvStatMsg.encode())
    else:
        print 'Received pfloat status but waiting for usbl fix'
    
lc = lcm.LCM()
lc.subscribe("FALKOR.USBL_FIX.PFLOAT.", usblHandler)
lc.subscribe("AFS.PFLOAT", pfloatHandler)

while True:
    lc.handle()
