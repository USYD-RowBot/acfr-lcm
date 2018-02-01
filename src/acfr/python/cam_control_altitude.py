#!/usr/bin/python

import lcm
import time
import sys

LCMROOT='/home/auv/git/acfr-lcm'
sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from acfrlcm.auv_camera_trigger_t import auv_camera_trigger_t
from senlcm import uvc_osi_t

min_height = 0.1
height_start = 4
height_stop = 6

SET_STATE = 4
TRIGGER_FLAG = 1


def uvc_osi_handler(channel, data):
    msg = uvc_osi_t.decode(data)
    alt = msg.altimeter

    msg = auv_camera_trigger_t()
    msg.utime = int(time.time() * 1000000)
    msg.command = SET_STATE

    if alt >= min_height and alt <= height_start:
        msg.enabled = 1
        TRIGGER_FLAG = 1
    elif alt > height_stop:
        msg.enabled = 0
        TRIGGER_FLAG = 0

    lc.publish("CAMERA_TRIGGER", msg.encode())
    
lc = lcm.LCM()
lc.subscribe("UVC_OSI", uvc_osi_handler)

while True:
    lc.handle()
