#!/usr/bin/env python

# send a new mission <argv[1]> to veicle <argv[2]>

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
msg.command = auv_global_planner_t.MISSION
msg.str = missionString
    
vehicle_name = sys.argv[2];
    
lc.publish(vehicle_name+'.TASK_PLANNER_COMMAND', msg.encode())

