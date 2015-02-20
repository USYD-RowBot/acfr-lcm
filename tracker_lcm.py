import sys
from threading import Thread
import math
import lcm
import ConfigParser
import select
import string

LCMROOT='/home/auv/git/acfr_lcm'
sys.path.append('{0}/build/lib/python{1}.{2}/dist-packages/perls/lcmtypes'.format(LCMROOT, sys.version_info[0], sys.version_info[1]))

from senlcm import usbl_fix_t
from acfrlcm import auv_status_t, auv_acfr_nav_t, ship_status_t

# Flag to exit the thread
exitFlag = 0

# Using dictionaries to store data
ship = {}
iver = {}
sirius = {}
shipsize = {}
usbl = {}
auv_nav = {}
auv_status = {}
auv_alerts = {}
auv_sensors = {}

# Other globals
att_source = ''
targets = []


class LcmThread(Thread):

    def __init__(self):
        Thread.__init__(self)

    def novatelHandler(self, channel, data):
        global ship
        msg = novatel_t.decode(data)
        ship['lon'] =  math.degrees(msg.longitude)
        ship['lat'] =  math.degrees(msg.latitude)
        ship['roll'] =  math.degrees(msg.roll)
        ship['pitch'] =  math.degrees(msg.pitch)
        ship['hdg'] =  math.degrees(msg.heading)


    def rt3202Handler(self, channel, data):
        global ship
        msg = rt3202_t.decode(data)
        ship['lon'] =  math.degrees(msg.lon)
        ship['lat'] =  math.degrees(msg.lat)
        ship['roll'] =  math.degrees(msg.r)
        ship['pitch'] =  math.degrees(msg.p)
        ship['hdg'] =  math.degrees(msg.h)

    def auvStatusHandler(self, channel, data):
        global auv_status
        msg = auv_status_t.decode(data)
        platform = string.split(channel, '.')[1]
        auv_status[platform]['lon'] = math.degrees(msg.longitude)
        auv_status[platform]['lat'] = math.degrees(msg.latitude)
        auv_status[platform]['depth'] = msg.depth/10.0
        auv_status[platform]['alt'] = msg.altitude/10.0
        auv_status[platform]['roll'] = round(math.degrees(msg.roll/10.0),1)
        auv_status[platform]['pitch'] = round(math.degrees(msg.pitch/10.0),1)
        auv_status[platform]['hdg'] = round(math.degrees(msg.heading/10.0),1)
        auv_status[platform]['img'] = msg.img_count
        auv_status[platform]['wpt'] = msg.waypoint
        auv_status[platform]['status'] = msg.status
        auv_status[platform]['batt'] = msg.charge

        # decode the status word
        #    Status word, binary states
        #            bit 0       DVL
        #            bit 1       DVL BL
        #            bit 2       GPS
        #            bit 3       Depth
        #            bit 4       Compass
        #            bit 5       IMU
        #            bit 6       OAS
        #            bit 7       Nav
        #            bit 8       Ecopuck
        #            bit 9       Aborted
        #            bit 10
        #            bit 11
        #            bit 12

        #          Warning flags
        #            bit 13      Pressure
        #            bit 14      Temp
        #            bit 15      Leak

        auv_sensors[platform] = []
        auv_alerts[platform] = []
        d = msg.status
        if d & (1 << 0):
            auv_sensors[platform].append('DVL')

        if d & (1 << 1):
            auv_sensors[platform].append('DVL_BL')

        if d & (1 << 2):
            auv_sensors[platform].append('GPS')

        if d & (1 << 3):
            auv_sensors[platform].append('DEPTH')

        if d & (1 << 4):
            auv_sensors[platform].append('COMPASS')

        if d & (1 << 5):
            auv_sensors[platform].append('IMU')

        if d & (1 << 6):
            auv_sensors[platform].append('OAS')

        if d & (1 << 7):
            auv_sensors[platform].append('NAV')

        if d & (1 << 9):
            auv_alerts[platform].append('ABORT')

        if d & (1 << 13):
            auv_alerts[platform].append('PRESSURE')

        if d & (1 << 14):
            auv_alerts[platform].appemd('TEMP')

        if d & (1 << 15):
            auv_alerts[platform].append('LEAK')



    def acfrNavHandler(self, channel, data):
        global auv_nav
        msg = auv_acfr_nav_t.decode(data)
        # channel name must be ACFR_NAV.platform.TOP or ACFR_NAV.platform
        platform = string.split(channel, '.')[1]
        auv_nav[platform]['lat']= math.degrees(msg.latitude)
        auv_nav[platform]['lon']= math.degrees(msg.longitude)
        auv_nav[platform]['x']= msg.x
        auv_nav[platform]['y']= msg.y
        auv_nav[platform]['depth']= msg.depth
        auv_nav[platform]['alt']= msg.altitude
        auv_nav[platform]['roll']= math.degrees(msg.roll)
        auv_nav[platform]['pitch']= math.degrees(msg.pitch)
        auv_nav[platform]['heading']= math.degrees(msg.heading)

    def usblHandler(self, channel, data):
        msg = usbl_fix_t.decode(data)
        global usbl
        usbl['id'] = msg.remote_id
        usbl['lat'] = math.degrees(msg.latitude)
        usbl['lon'] = math.degrees(msg.longitude)
        usbl['depth'] = msg.depth
        usbl['accuracy'] = msg.accuracy

    def ahrsHandler(self, channel, data):
        global ship
        msg = ahrs_t.decode(data)
        ship['roll'] =  math.degrees(msg.roll)
        ship['pitch'] =  math.degrees(msg.pitch)
        ship['hdg'] =  math.degrees(msg.heading)

    def shipststatusHandler(self, channel, data):
        global ship
        msg = gpsd3_t.decode(data)
        if msg.status > 0:
            ship['lon'] =  math.degrees(msg.fix.longitude)
            ship['lat'] =  math.degrees(msg.fix.latitude)

    def run(self):

        self.lc = lcm.LCM()
        print 'att source {}'.format(att_source)
        if att_source == 'novatel':
            print 'Subscribing to NOVATEL'
            self.lc.subscribe("NOVATEL", self.novatelHandler)
        elif att_source == 'rt3202':
            self.lc.subscribe("RT3202", self.rt3202Handler)
        elif att_source == 'ahrs':
            self.lc.subscribe("AHRS", self.ahrsHandler)
            self.lc.subscribe("GPSD_CLIENT", self.gpsdHandler)
        self.lc.subscribe("NOVATEL", self.novatelHandler)

        # subscribe to the NAV and HEALTH stream for all the targets
        for tt in targets:
            ch_nav = 'ACFR_NAV.{}.TOP'.format(tt)
            ch_health = 'AUV_STATUS.{}'.format(tt)
            print ch_nav
            print ch_health
            self.lc.subscribe(ch_nav, self.acfrNavHandler)
            self.lc.subscribe(ch_health, self.auvStatusHandler)

        self.lc.subscribe("USBL_FIX", self.usblHandler)

        timeout = 1  # amount of time to wait, in seconds
        while not exitFlag:
            rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
            if rfds:
                self.lc.handle()

