#!/usr/bin/env python
import lcm
import sys
import os.path

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from bot_procman import printf_t
from acfrlcm import auv_path_command_t
from acfrlcm import auv_acfr_nav_t


if (len(sys.argv) < 2):
	exit
lcm_url = 'file://'+sys.argv[1]+'?speed=0'
print lcm_url
lc = lcm.LCM(lcm_url)

dd = {}


def handler(channel, data):
    msg = printf_t.decode(data)
    if msg.sheriff_id not in dd:
        dd[msg.sheriff_id] = []
        print msg.sheriff_id
    dd[msg.sheriff_id].append(msg)  
	
def gphandler(channel, data):
    msg = auv_path_command_t.decode(data)
    sheriff_id = 'mission waypoints'
    if sheriff_id not in dd:
        dd[sheriff_id] = []
        print sheriff_id
    dd[sheriff_id].append(msg)    
   
def navhandler(channel, data):
    msg = auv_acfr_nav_t.decode(data)
    sheriff_id = 'nav'
    if sheriff_id not in dd:
        dd[sheriff_id] = []
        print sheriff_id
    dd[sheriff_id].append(msg)  
    
lc.subscribe('PMD_PRINTF', handler)
lc.subscribe('NGA.PATH_COMMAND', gphandler)
lc.subscribe('NGA.ACFR_NAV', navhandler)


while True:
    try:
        lc.handle()
    except IOError:
        break
    
subdirectory = 'logs'
try:
    os.mkdir(subdirectory)
except Exception:
    pass
for key, value in dd.iteritems() :
    filename = '{}.txt'.format(key)
    write_header = True
    with open(os.path.join(subdirectory, filename), 'w') as f:
        for msg in value:
            if key == 'mission waypoints':
                if write_header:
                    f.write('utime,goal_id,waypoint\n')
                    write_header = False
                f.write('{},{},{}\n'.format(msg.utime, msg.goal_id, msg.waypoint))
            elif key == 'nav':
                if write_header:
                    f.write('utime,latitude,longitude,x,y,depth,roll,pitch,heading,vx,vy,vz\n')
                    write_header = False
                f.write('{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(msg.utime, msg.latitude, msg.longitude, msg.x, msg.y, msg.depth, msg.roll, msg.pitch, msg.heading, msg.vx, msg.vy, msg.vz))
            else:
                try:
                    f.write('{} {}\n'.format(msg.utime, msg.text))
                except:
                    pass
    f.close()

#overall logs - searching for keywords across all channels
log = open(os.path.join(subdirectory, 'log.txt'), 'w')
for key, value in dd.iteritems() :
    for msg in value:
        if (("abort" in msg.text) or ("ABORT:" in msg.text) or ("Sending camera trigger" in msg.text) or ("Sending way point" in msg.text) or ("waypoint reached" in msg.text)):
            log.write('{} {}\n'.format(msg.utime, msg.text))
log.close()


