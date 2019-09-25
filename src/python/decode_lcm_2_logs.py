#!/usr/bin/env python
import lcm
import sys
import os.path
import matplotlib.pyplot as plt

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
utime = []
depth = []
altitude = []
water_depth = []
zero_nav_time = []
distance_est = []
dist = 0

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
        print 'IOError'
        break
    
subdirectory = 'logs'
try:
    os.mkdir(subdirectory)
except Exception:
    print 'delete logs folder'
    pass

for key, value in dd.iteritems():
    filename = '{}.txt'.format(key)
    write_header = True
    plot_nav = True
    get_time_zero = True
    if 'mission waypoints' in filename:
        with open(os.path.join(subdirectory, filename), 'w+') as f:
            for msg in value:
                if write_header:
                    f.write('utime,goal_id,waypoint\n')
                    write_header = False
                f.write('{},{},{}\n'.format(msg.utime, msg.goal_id, msg.waypoint))
    elif 'nav' in filename:
        with open(os.path.join(subdirectory, filename), 'w+') as f:
            for msg in value:
                if plot_nav:
                    if get_time_zero:
                        zero_nav_time = msg.utime
                        get_time_zero = False
                    utime.append((msg.utime-zero_nav_time)/1000000.0)
                    dist = dist + 0.1*msg.vx
                    distance_est.append(dist)
                    depth.append(msg.depth)
                    altitude.append(msg.altitude)
                    water_depth.append(-1.0*(msg.depth+msg.altitude))
                if write_header:
                    f.write('utime,latitude,longitude,x,y,depth,altitude,roll,pitch,heading,vx,vy,vz,water_depth\n')
                    write_header = False
                f.write('{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(msg.utime, msg.latitude, msg.longitude, msg.x, msg.y, msg.depth, msg.altitude, msg.roll, msg.pitch, msg.heading, msg.vx, msg.vy, msg.vz,(msg.depth+msg.altitude)))


log =  open(os.path.join(subdirectory, 'log.txt'), 'w+')
for key, value in dd.iteritems():
    for msg in value:
        try:
            if (("abort" in msg.text) or ("ABORT:" in msg.text) or ("Sending camera trigger" in msg.text) or ("Sending way point" in msg.text) or ("waypoint reached" in msg.text)):
                log.write('{} {}\n'.format(msg.utime, msg.text))
        except:
            pass
log.close()

if plot_nav:  
    plt.figure(1)
    plt.subplot(311)
    plt.title('Nav Depth')
    plt.xlabel('Seconds')
    plt.ylabel('Metres')
    plt.grid(True)
    plt.plot(utime, depth, 'b--')

    plt.subplot(312)
    plt.title('Nav Altitude')
    plt.xlabel('Seconds')
    plt.ylabel('Metres')
    plt.grid(True)
    plt.plot(utime, altitude, 'r--')

    plt.subplot(313)
    plt.title('Depth + Altitude = sea floor profile')
    plt.xlabel('Seconds')
    plt.ylabel('Metres')
    plt.grid(True)
    plt.plot(utime, water_depth, 'g--')
    plt.show()


