#!/usr/bin/env python


import lcm
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_camera_trigger_t import auv_camera_trigger_t

lc = lcm.LCM();

msg = auv_camera_trigger_t()
msg.command = auv_camera_trigger_t.SET_STATE
msg.enabled = int(float(sys.argv[1])) # 0=stop, 1=start
    
    
lc.publish('CAMERA_TRIGGER.SIRIUS', msg.encode())

