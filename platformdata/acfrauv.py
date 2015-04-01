######################################################################
# This file handles all the platform pose updates.
#
# There are some TODOS listed throughout the file with instructions.
# See documentation below for more info.
#
# Contact me if you have any questions.
#
# Author:
#   Ariell Friedman
#   ariell.friedman@gmail.com
#   25 AUG 2014
#
######################################################################

from random import random, randint
import time
import threading

import sys
import math
import lcm
import ConfigParser
import select
import string
import os
import pyproj
import json
import requests




LCMROOT='/home/auv/git/acfr_lcm'
#SEABEDGUIROOT='/home/auv/git/seabed_gui'
sys.path.append('{0}/build/lib/python{1}.{2}/dist-packages/perls/lcmtypes/'.format(LCMROOT, sys.version_info[0], sys.version_info[1]))
#sys.path.append('{0}/build/lib/'.format(LCMROOT))
#sys.path.append('{0}/lib/'.format(SEABEDGUIROOT))


# TODO: remove try-except statement
try:
    import missionMP
except:
    print "Unable to load missionMP (Sirius)"
try:
    import missionXML
except:
    print "Unable to load missionXML (Iver)"


from acfrlcm import auv_status_short_t, ship_status_t
from senlcm import usbl_fix_t
#except:
#    pass

# This global dictionary stores all the platform information updates
platformdata = {}


######################################################################
# Start threads for platform updates
# These are currently fake threads that update the vehicle
# poses. TODO: Make them real or replace them with something similar!
######################################################################
def init_platformdata_threads():
    LcmThread().start()
    WaveGliderWGMSThread('WGMS.WGLIDER', 5*60).start()


def init_push_data(configfile):
    cfg = ConfigParser.ConfigParser()
    cfg.read(configfile)
    if (cfg.has_option('layers', 'remotepush')):
        remotesec = cfg.get('layers', 'remotepush')
        url = cfg.get(remotesec, 'url')
        targets = cfg.get(remotesec, 'targets').split(',')


        upddelay = float(cfg.get(remotesec, 'upddelay'))

        sendRemoteDataThread(upddelay, targets, url).start()

    return


######################################################################
# Get data for a specific platform
# The global variable platformdata is updated by another process/thread.
# This function simply reads the output for a specific platform
######################################################################
def get_platformdata(platform):
    data = platformdata[platform]  # get data
    data['curts'] = int(time.time())    # add curr ts
    if data['state'] == 'follow':
        data['pose'] = dict((k, platformdata[data['follow']]['pose'][k]) for k in ('lat', 'lon'))  #platformdata[data['follow']]['pose']
        data['msgts'] = platformdata[data['follow']]['msgts']
    elif (data['state'] == 'stationary'):
        data['msgts'] = data['curts']
    elif (data['curts']-data['msgts']) > 30:
        speed = data['pose']['speed'] if 'speed' in data['pose'] else 0.5
        data['pose']['uncertainty'] = (data['curts']-data['msgts'])*speed


    return data


def set_platformdata(platform=None, data={}):
    global platformdata
    if platform is None:
        platformdata = json.loads(data)
    else:
        platformdata[platform] = json.loads(data)
    return


def send_to_platform(args):
    for k in args:
        args[k] = args[k][0]
        #response += "<br>{}: {}".format(k, args[k][0])
    #{'auvstateplatform': u'Iver', 'auvstate': u'online', 'auvstatedesc': u'', 'auvstatefollow': u'FALKOR', 'auvstatelon': u'', 'auvstatelat': u''}
    platform = str(args['auvstateplatform'])
    data = {
        'state': str(args['auvstate']),
        'statemsg': str(args['auvstatemsg'])
    }

    if args['auvstate'] == 'follow':
        data['follow'] = str(args['auvstatefollow'])
    elif args['auvstate'] == 'stationary':
        data['pose'] = {
            'lat': float(args['auvstatelat']),
            'lon': float(args['auvstatelon']),
            'uncertainty': 0.2,
            'speed': 0
        }

    print data
    platformdata[platform] = data
    #set_platformdata(platform=platform, data=json.dumps(data))

    return 0
######################################################################
# Parse mission file
# This platform
# Outputs latlngs and origin from mission file
# TODO: parse actual mission file
######################################################################
def parse_mission (filepath, cfgorigin=[0, 0]):
    # TODO: parse mission file and origin and return in LAT/LON
    # filepath will contain the filepath to the mission file

    # from the filename we need to work out which mission parser to use
    try:
        fileName, fileExtension = os.path.splitext(filepath)
        if fileExtension.lower() == '.xml':
            mission = missionXML.Mission()
            startind = 0
        elif fileExtension.lower() == '.mp':
            mission = missionMP.Mission()
            startind = 2
        else:
            print 'Incorrect mission type'
            return

        mission.load(filepath)
        origin = [mission.getOriginLat(), mission.getOriginLon()]
        if origin[0] == 0 or origin[1] == 0:
            origin = cfgorigin

        waypoints = mission.dumpSimple()


        # convert the waypoints to latitude and longitude
        projStr = '+proj=tmerc +lon_0={} +lat_0={} +units=m'.format(origin[1], origin[0])
        p = pyproj.Proj(projStr)

        latlngs = []
        for wpt in waypoints:
            ll = p(wpt.y, wpt.x, inverse=True)
            latlng = [ll[1], ll[0]]
            latlngs.append(latlng)

        latlngs = latlngs[startind:]

    except:
        print "Unable to parse mission!!!"
        return
    return latlngs, origin




######################################################################
#
######################################################################

class LcmThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.exitFlag = False
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit


    def usblFixHandler(self, channel, data):
        msg = usbl_fix_t.decode(data)
        msgid = msg.utime
        platform = channel  # 'usbl{}'.format(msg.remote_id)
        # double target_x;
        # double target_y;
        # double target_z;
        # double latitude;
        # double longitude;
        # double depth;
        # double accuracy;
        #
        # double ship_latitude;
        # double ship_longitude;
        # float ship_roll;
        # float ship_pitch;
        # float ship_heading;
        platformdata[platform] = {
            'msgid': msgid,                                 # REQUIRED (number)
            'state': 'online',
            'msgts': int(time.time()),
            'pose': {
                'lat': round(math.degrees(msg.latitude), 8),          # REQUIRED (decimal degrees)
                'lon': round(math.degrees(msg.longitude), 8),          # REQUIRED (decimal degrees)
                'depth': round(msg.depth, 1),
                'XYZ': "{}, {}, {}".format(round(msg.target_x, 1), round(msg.target_y, 1), round(msg.target_z, 1)),
                'uncertainty': round(msg.accuracy, 2),
                'speed': 0.5
            }
        }



    def shipStatusHandler(self, channel, data):
        msg = ship_status_t.decode(data)
        msgid = msg.utime
        #platform = '{}'.format(msg.ship_id)
        platform = msg.name
        platformdata[platform] = {
            'msgid': msgid,                                 # REQUIRED (number)
            'state': 'online',
            'msgts': int(time.time()),
            'pose': {
                'lat': round(math.degrees(msg.latitude), 8),          # REQUIRED (decimal degrees)
                'lon': round(math.degrees(msg.longitude), 8),         # REQUIRED (decimal degrees)
                'heading': round(math.degrees(msg.heading), 2),        # REQUIRED (degrees)
                'roll': round(math.degrees(msg.roll), 2),        # REQUIRED (degrees)
                'pitch': round(math.degrees(msg.pitch), 2),        # REQUIRED (degrees)
                'speed': 1
            }
        }



    def auvStatusHandler(self, channel, data):
        print channel
        msg = auv_status_short_t.decode(data)
        print msg
        platform = channel #'auv{}'.format(msg.target_id)
        msgid = msg.utime
        platformdata[platform] = {
            'msgid': msgid,                                 # REQUIRED (number)
            'state': 'online',
            'msgts': int(time.time()),
            'pose': {
                'lat': round(math.degrees(msg.latitude), 8),                  # REQUIRED (decimal degrees)
                'lon': round(math.degrees(msg.longitude), 8),                 # REQUIRED (decimal degrees)
                'heading': round(msg.heading/10.0,1), # REQUIRED (degrees)
                'alt': float(msg.altitude)/10.0,                           # OPTIONAL (m)
                'depth': float(msg.depth)/10.0,                            # OPTIONAL (m)
                'roll': round(msg.roll/10.0,1),       # OPTIONAL / REQUIRED for dashboard (degrees)
                'pitch': round(msg.pitch/10.0,1),      # OPTIONAL / REQUIRED for dashboard (degrees)
                'speed': 0.5
                #'uncertainty': 1
            },
            'stat': {
                # You can add whatever custom status messages you like here
                # NB: 'bat' has a special display widget
                # Any others, just get displayed normally
                'bat': msg.charge,                     # REQUIRED for dashboard (int 0-100)
                'waypt': msg.waypoint,
                '#imgs': msg.img_count
                #'state': 'OK'
            },
            'alert': {
                # You can add whatever custom alerts you like here.
                # Must return 1 for error, 0, for OK.
                'dvl':  msg.status & (1 << 0),
                'dvl_bl':  msg.status & (1 << 1),
                'gps':  msg.status & (1 << 2),
                'depth':  msg.status & (1 << 3),
                'comp':  msg.status & (1 << 4),
                'imu':  msg.status & (1 << 5),
                'oas':  msg.status & (1 << 6),
                'nav':  msg.status & (1 << 7),
                'epuck':  msg.status & (1 << 8),
                'abort':  msg.status & (1 << 9),
                'press':  msg.status & (1 << 13),
                'leak':  msg.status & (1 << 15)
            }
        }
        # Alert if AUV > 6 m
        if platformdata[platform]['pose']['depth'] < 6:
            platformdata[platform]['alert']['DEP<6'] = 1

    def run(self):
        self.lc = lcm.LCM()
        self.lc.subscribe("AUVSTAT.*", self.auvStatusHandler)
        self.lc.subscribe("SHIP_STATUS.*", self.shipStatusHandler)
        self.lc.subscribe("USBL_FIX.*", self.usblFixHandler)

        timeout = 1  # amount of time to wait, in seconds
        while not self.exitFlag:
            rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
            if rfds:
                self.lc.handle()


class sendRemoteDataThread (threading.Thread):
    def __init__(self, delay, targets, destserver):
        threading.Thread.__init__(self)
        self.delay = delay
        self.targets = targets
        self.destserver = destserver
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit

    def run(self):

        while(1) :
            sendplatforms = {}
            for key in self.targets:
                key = key.strip()
                try:
                    print "Getting {}".format(key)
                    sendplatforms[key] = get_platformdata(key)

                except:
                    print "ERROR!!!   Unable to read {}".format(key)

            try:
                print "Sending data to {}".format(self.destserver)
                payload = {'platformdata': json.dumps(sendplatforms)}
                r = requests.post(self.destserver, data=payload)
            except:
                print "ERROR!!!   Unable to send data to {}".format(self.destserver)

            time.sleep(self.delay)


# For waveglider
import pycurl
import cStringIO

class WaveGliderWGMSThread (threading.Thread):
    #convert the following:
    #curl -k -H "Content-Type: text/xml; charset=utf-8" --dump-header headers -H "SOAPAction:" -d @Documents/waveglider/loginsoap.xml -X POST https://gliders.wgms.com/webservices/entityapi.asmx
    #curl -b headers "http://uh.wgms.com/pages/exportPage.aspx?viewid=36113&entitytype=42"
    def __init__(self, platform, delay):
        threading.Thread.__init__(self)
        self.platform = platform
        self.delay = delay
        self.daemon = True  # run in daemon mode to allow for ctrl+C exit
        # authentication info
        self.auth_url = 'https://gliders.wgms.com/webservices/entityapi.asmx'
        self.auth_xml = '/home/oshirojk/Documents/waveglider/loginsoap.xml'

        # data retrieve url
        self.export_url = 'http://uh.wgms.com/pages/exportPage.aspx?viewid=36113&entitytype=42'

    def run (self):
        while(1) :
            try:
                # get data
                buf = cStringIO.StringIO()
                c = pycurl.Curl()
                c.setopt(pycurl.URL, self.export_url)
                c.setopt(pycurl.COOKIEFILE, 'datatools/waveglider_header')
                c.setopt(pycurl.WRITEFUNCTION, buf.write)
                c.setopt(pycurl.VERBOSE, True)
                c.perform()
                curldata = buf.getvalue().split('\n')
                buf.close()


                # headers = curldata[0].split(',')
                data = curldata[1].split(',')
                # for i in range(len(headers)):
                # 	print '{}) {}: {}'.format(i, headers[i],data[i])

                platformdata[self.platform] = {
                    'msgid': data[0],
                    'state': 'online',
                    'msgts': int(time.time()),
                    'headings': 'F:{}, S:{}'.format(data[5],data[4]),
                    'pose': {
                        'lat': float(data[11]),
                        'lon': float(data[12]),
                        'heading': float(data[3]),
                        'speed': float(data[1])*0.514444444
                    },
                    'stat': {
                        'bat': round(float(data[8])/6.6,1),
                        'ftemp': int(data[6]),
                        'pressure': int(data[7])
                    }
                }
            except:
                print "ERROR!!! Unable to get WGMS data"


            time.sleep(self.delay)
