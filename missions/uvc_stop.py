#!/usr/bin/env python

import lcm
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
sys.path.append('/home/auv/git/acfr_lcm/src/acfr/python/')

from senlcm.uvc_omstop_t import uvc_omstop_t

lc = lcm.LCM();
msg = uvc_omstop_t()


lc.publish('UVC_OMSTOP.IVERAME', msg.encode())

