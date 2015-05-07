#!/usr/bin/env python

import lcm
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
sys.path.append('/home/auv/git/acfr_lcm/src/acfr/python/')

from senlcm.uvc_omstart_t import uvc_omstart_t

if (len(sys.argv) < 2):
	print "Usage: uvc_start [mission name] [use gps] <srp name> "
	exit

lc = lcm.LCM();
msg = uvc_omstart_t()

print sys.argv

msg.mission_name = sys.argv[1]
msg.msg_flag_gps = int(sys.argv[2])

if (len(sys.argv) == 4):	
	msg.srp_name = sys.argv[3]
	msg.mission_type = 0
else:
	msg.srp_name = ''
	msg.mission_type = 1

msg.msg_flag_sounder = 0;
msg.msg_flag_cal_pressure = 0;

lc.publish('UVC_OMSTART', msg.encode())

