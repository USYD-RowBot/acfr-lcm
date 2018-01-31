#!/usr/bin/python


import lcm
import sys
import math

LCMROOT='/home/auv/git/acfr-lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import usbl_fix_t
from acfrlcm import auv_status_short_t
from urioce_lcm import float_acomm_status_t

pfloatMsg = None

def usblHandler(channel, data):
    global pfloatMsg
    msg = usbl_fix_t.decode(data)
    print 'Received usb_fix_short_t on channel ' + channel
    
    lat = msg.latitude
    lon = msg.longitude
    if pfloatMsg is not None:

        auvStatMsg = auv_status_short_t()
        auvStatMsg.utime = pfloatMsg.timestamp * 1e6
        if pfloatMsg.altitude < 1250:
            auvStatMsg.altitude = pfloatMsg.altitude / 10
        elif pfloatMsg.altitude > 12500:
            auvStatMsg.altitude = -125
        else:
            auvStatMsg.altitude = -pfloatMsg.altitude / 100
        auvStatMsg.depth = pfloatMsg.depth / 10
        auvStatMsg.charge = pfloatMsg.bat_charge
        auvStatMsg.waypoint = pfloatMsg.leg_num
        auvStatMsg.latitude = lat*180/math.pi
        auvStatMsg.longitude = lon*180/math.pi
        auvStatMsg.vel = 5

        lc.publish("PFLOAT.AUVSTAT", auvStatMsg.encode())
    else:
        print 'Received usbl_fix status but waiting for pfloat message'
 

def pfloatHandler(channel, data):
    global pfloatMsg
    pfloatMsg = float_acomm_status_t.decode(data)
    print 'Received float_acomm_status_t on channel ' + channel

   
lc = lcm.LCM()
lc.subscribe("FALKOR.USBL_FIX.PFLOAT", usblHandler)
lc.subscribe("AFS.PFLOAT", pfloatHandler)

while True:
    lc.handle()
