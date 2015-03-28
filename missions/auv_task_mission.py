#!/usr/bin/env python


import lcm
import sys
import xml.etree.ElementTree as ET

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_global_planner_t import auv_global_planner_t

lc = lcm.LCM();

f = open(sys.argv[1],"r")
mission = f.readlines()
missionXML = ET.fromstringlist(mission)
missionString = ET.tostring(missionXML)

msg = auv_global_planner_t()
msg.command = auv_global_planner_t.GOTO
msg.str = missionString
    
    
lc.publish('TASK_PLANNER_COMMAND.SIRIUS', msg.encode())

