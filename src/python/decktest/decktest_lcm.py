#!/usr/bin/env python

import lcm
import sys
import math
import socket
import time
import thread
import threading
import configargparse
import curses
import curses.ascii
import locale

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
locale.setlocale(locale.LC_ALL, '')    # set your locale

# Import required lcm message types
from bot_param import request_t    # for heartbeat
from senlcm import os_power_system_t # for batteries
from acfrlcm import auv_acfr_nav_t # for nav
        # for modem
from senlcm import tcm_t        # for tcm
from senlcm import gpsd3_t      # for gps
from acfrlcm import auv_iver_motor_command_t   # for tail thruster and fins
        # for strobe
        # for camera
from senlcm import rdi_pd5_t    # for dvl
from senlcm import ysi_t        # for ysi

# TODO - do we need these?
# from acfrlcm.auv_global_planner_t import auv_global_planner_t
# from acfrlcm.auv_iver_motor_command_t import auv_iver_motor_command_t
# from senlcm import os_compass_t, airmar_t
# from acfrlcm import auv_vis_rawlog_t

# create global dictionaries for incoming LCM messages
SUMMARYDATA = {}
MSGCOUNTDATA = {}
REQCOUNTDATA = {}
HBDATA = {}
BATTDATA = {}
NAVDATA = {}
MODEMDATA = {}
TCMDATA = {}
GPSDATA = {}
TAILDATA = {}  # ?
STROBEDATA = {}  # ?
CAMERADATA = {}
DVLDATA = {}
YSIDATA = {}

test_duration = 15  #seconds
test_started = False
test_complete = False
th_fail = False

# display colors (used by Curses)
c_BLUE = 1
c_RED = 2
c_YELLOW = 3
c_GREEN = 4

# test styles used for timing tests
SINGLE_RUN = 1
PERIODIC = 2

# display symbols
mu = "u'\u00B5'.encode('utf-8')"
deg = "u'\u00B0'.encode('utf-8')"

# layout globals - display box indents
column_width = 50
left_in = 2
left_2nd = 16
left_3rd = 40
title_box_height = 4
max_cols = 5  # max mumber of display box columns

# module specific selective colour definitions
min_satelites = 8  # TODO Number of satelites to colour green OK


class DisplayItem:
    def __init__(self, name, data):
        self.name = name
        self.data = data
        self.fields = [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ']

# used by the tests to selectively auto-colour the output when it is written to the boxes
    def sel_col_txt(self, ln_no, indent, write_data):
        if write_data == '-1':  # default values (no-data)
            self.box.addstr(ln_no, indent, write_data, curses.color_pair(c_YELLOW))
        elif self.name == 'GPS':  # add aditional elif after this section to add module specific behaviour
            if ln_no == 5:  # conditions per line item
                if write_data == '0':
                    self.box.addstr(ln_no, indent, 'NO FIX', curses.color_pair(c_RED))
                elif write_data == '1':
                    self.box.addstr(ln_no, indent, 'GPS FIX', curses.color_pair(c_GREEN))
                elif write_data == '2':
                    self.box.addstr(ln_no, indent, 'DGPS FIX', curses.color_pair(c_GREEN))
                else:
                    self.box.addstr(ln_no, indent, 'INVALID', curses.color_pair(c_RED))
            elif ln_no == 6:
                if int(write_data) < min_satelites:
                    self.box.addstr(ln_no, indent, write_data, curses.color_pair(c_RED))
                else:
                    self.box.addstr(ln_no, indent, write_data, curses.color_pair(c_GREEN))
            else:
                self.box.addstr(ln_no, indent, write_data, curses.color_pair(c_GREEN))
        # end of gps specific conditions
        else:
            self.box.addstr(ln_no, indent, write_data, curses.color_pair(c_GREEN))

# used by the summary to selectively auto-colour the message count outputs
    def sel_col_count(self, ln_no, indent, write_data, req_freq):
        if int(write_data) < ((req_freq * test_duration) * 0.95):  # message count too low
            self.box.addstr(ln_no, indent, write_data + ' (<' + str(req_freq * test_duration) + ')', curses.color_pair(c_RED))
        elif int(write_data) > ((req_freq * test_duration) * 1.05):  # count too high
            self.box.addstr(ln_no, indent, write_data + ' (>' + str(req_freq * test_duration) + ')', curses.color_pair(c_RED))
        else:   # count OK
            self.box.addstr(ln_no, indent, write_data, curses.color_pair(c_GREEN))

    # draw a display box for the test, at the location shown
    def create_disp_box(self, start_y, start_x):
        self.box = curses.newwin(len(self.fields) + 2, column_width, start_y, start_x)

    # write (or re-write) the tags and data to the display box using selective colour for data fields
    def write_disp_box(self):
        box_title = ' ' + self.name + ' '
        self.box.clear()
        self.box.border(0)
        # print labels
        line_no = 0
        self.box.addstr(line_no, ((column_width - len(box_title)) / 2), box_title)

        self.box.noutrefresh()


class Summary(DisplayItem):
    def __init__(self, name, data, countdata): # , test_list, active_test_list):
        DisplayItem.__init__(self, name, data)
        self.countdata = countdata
        #self.active_test_list = active_test_list

    def write_disp_box(self):
        box_title = ' ' + self.name + ' '
        self.box.clear()
        self.box.border(0)
        if test_complete:
            line_no = 0
            self.box.addstr(line_no, ((column_width - len(box_title)) / 2), box_title)
            line_no = 3
            self.box.addstr(line_no, left_in, 'Test Complete')
            self.sel_col_txt(4, left_in, 'Test started: ')
            self.sel_col_txt(4, left_2nd, str(test_started))
            self.sel_col_txt(5, left_in, 'Test complete: ')
            self.sel_col_txt(5, left_2nd, str(test_complete))
            self.sel_col_txt(6, left_in, 'Summary: ')
            self.sel_col_txt(6, left_2nd, str(len(MSGCOUNTDATA)))
            # self.sel_col_txt(6, left_in, 'Thread failed: ')
            # self.sel_col_txt(6, left_2nd, str(th_fail))
            line_no = 8
            self.box.addstr(line_no, left_in, 'Incoming Message Counts:  (test duration ' + str(test_duration) + 's)')
            line_no = 9
            for entry in MSGCOUNTDATA.keys():
                self.box.addstr(line_no, left_in, str(entry + ': '))
                self.sel_col_count(line_no, left_2nd, str(MSGCOUNTDATA[entry]), REQCOUNTDATA[entry])
                line_no = line_no + 1
        elif test_started:
            line_no = 0
            self.box.addstr(line_no, ((column_width - len(box_title)) / 2), box_title)
            line_no = 3
            self.sel_col_txt(line_no, left_in, 'Test In progress')
            self.sel_col_txt(4, left_in, 'Test started: ')
            self.sel_col_txt(4, left_2nd, str(test_started))
            self.sel_col_txt(5, left_in, 'Test complete: ')
            self.sel_col_txt(5, left_2nd, str(test_complete))
            self.sel_col_txt(6, left_in, 'Thread failed: ')
            self.sel_col_txt(6, left_2nd, str(th_fail))
        else:
            line_no = 0
            self.box.addstr(line_no, ((column_width - len(box_title)) / 2), box_title)
            line_no = 3
            self.sel_col_txt(line_no, left_in, 'Press \'r\' to start test')
            self.sel_col_txt(4, left_in, 'Test started: ')
            self.sel_col_txt(4, left_2nd, str(test_started))
            self.sel_col_txt(5, left_in, 'Test complete: ')
            self.sel_col_txt(5, left_2nd, str(test_complete))
            self.sel_col_txt(6, left_in, 'Thread failed: ')
            self.sel_col_txt(6, left_2nd, str(th_fail))

        self.box.noutrefresh()


class Test(DisplayItem):
    def __init__(self, name, data, channel, handler, data_type, req_freq, fields, field_tags, field_units):
        DisplayItem.__init__(self, name, data)
        # super(Test, self).__init__(name, data)
        self.channel = channel
        self.handler = handler
        self.data_type = data_type
        self.req_freq = req_freq
        self.fields = fields
        self.field_tags = field_tags
        self.field_units = field_units
        # initialse variables in case no data comes through
        for field in fields:
            data[field] = -1
        data['MSGCOUNT'] = 0

    # write (or re-write) the tags and data to the display box using selective colour for data fields
    def write_disp_box(self):
        box_title = ' ' + self.name + ' '
        self.box.clear()
        self.box.border(0)
        # print labels
        line_no = 0
        self.box.addstr(line_no, ((column_width - len(box_title)) / 2), box_title)
        for entry in self.field_tags:
            line_no = line_no + 1
            self.box.addstr(line_no, left_in, entry + ': ')
        # print values
        line_no = 1
        for entry in self.fields:
            self.sel_col_txt(line_no, left_2nd, repr(self.data[entry]))
            line_no = line_no + 1
        # print units
        line_no = 1
        for entry in self.field_units:
            self.box.addstr(line_no, left_3rd, entry)
            line_no = line_no + 1

        self.box.noutrefresh()


class ActiveTest(DisplayItem):
    def __init__(self, name, data, out_channel, publisher, out_type, test_style):
        DisplayItem.__init__(self, name, data)  #, channel, handler, data_type, fields, field_tags, field_units)
        self.out_channel = out_channel
        self.publisher = publisher
        self.out_type = out_type
        self.test_style = test_style  # periodic vs single run full duration


# class ActiveTestFeedback(Test):
#     def __init__(self, name, data, channel, handler, data_type, fields, field_tags, field_units, has_data_in, out_data, out_channel, publisher, out_type, out_fields):
#         Test.__init__(self, name, data, channel, handler, data_type, fields, field_tags, field_units)
#         self.has_data_in = has_data_in
#         self.out_data = out_data
#         self.out_channel = out_channel
#         self.publisher = publisher
#         self.out_type = out_type
#         self.out_fields = out_fields


# timer for test triggers
class TimerThread(threading.Thread):
    def __init__(self, thread_name, test_list, active_test_list):
        threading.Thread.__init__(self)
        self.thread_name = thread_name
        self.test_list = test_list
        self.active_test_list = active_test_list

    def run(self):
        global test_started
        global test_complete

        def reset_msg_counters():
            for test in self.test_list:
                test.data['MSGCOUNT'] = 0

        def store_msg_counters():
            for test in self.test_list:
                MSGCOUNTDATA[str(test.name)] = test.data['MSGCOUNT']
                REQCOUNTDATA[str(test.name)] = test.req_freq

        # zero message counts
        test_started = True
        test_complete = False

        # store start time TODO
        started_at = time.time()
        reset_msg_counters()
        finish_at = started_at + test_duration

        # setup and start single run tests
        for test in self.active_test_list:
            if test.style == SINGLE_RUN:
                # run some setup functions
                x = 0
                # start cameras/strobes

        # setup triggers etc for periodic tests
        for test in self.active_test_list:
            if test.style == PERIODIC:
                # run some setup functions
                x = 0
                # 100RPM max
                # cycle fins and motor
                # TODO - Active Test Timing

        # setup timers to run tests with periodic events
            # tail -

        while time.time() < finish_at:
            x = 0

        # timers complete
        # store message counts
        store_msg_counters()

        # stops and cleanups etc for tests
        for test in self.active_test_list:
            x = 0
            # stop cameras strobes TODO

        # post processing
        test_complete = True
        test_started = False
        # kill thread
        pass

# Generates and manages the set of tests required by options for the vehicle specified


class Test_Set:
    def __init__(self, options, vehicle):
        self.options = options
        self.vehicle = vehicle
        self.TEST_LIST = list()
        self.ACTIVE_TEST_LIST = list()

    def summary_setup(self):
        self.test_stats = Summary('SUMMARY', SUMMARYDATA, MSGCOUNTDATA)

    def hb_setup(self):
        hb_test = Test('HB', HBDATA, "HEARTBEAT_10HZ", 'hb_handler', request_t, 10,
                        ['CHANNEL', 'MSGCOUNT', 'TIME', 'DIFF'],
                        ['Channel', 'Msg Count', 'Time', 'Period'],
                        [' ', ' ', '(' + eval(mu) + 's)', '(' + eval(mu) + 's)'])
        self.TEST_LIST.append(hb_test)

    def nav_setup(self):
        nav_test = Test('NAV', NAVDATA, "ACFR_NAV.TOP", 'nav_handler', auv_acfr_nav_t, 1,
                        ['CHANNEL', 'MSGCOUNT', 'TIME', 'LAT', 'LONG', 'X', 'Y', 'DEPTH', 'ROLL', 'PITCH', 'HEADING', 'VX', 'VY', 'VZ', 'ROLLRATE', 'PITCHRATE', 'HEADINGRATE', 'ALTITUDE', 'FWD_OBST_DIST'],
                        ['Channel', 'Msg Count', 'Time', 'Latitude', 'Longitude', 'X', 'Y', 'Depth', 'Roll', 'Pitch', 'Heading', 'VX', 'VY', 'VZ', 'Roll Rate', 'Pitch Rate', 'Heading Rate', 'Altitude', 'Obstacle Fwd'],
                        [' ', ' ', '(' + eval(mu) + 's)','(' + eval(deg) + ')', '(' + eval(deg) + ')', '(m)', '(m)', '(m)', '(' + eval(deg) + ')', '(' + eval(deg) + ')', '(' + eval(deg) + ')', '(m/s)', '(m/s)', '(m/s)', '(' + eval(deg) + '/s)', '(' + eval(deg) + '/s)', '(' + eval(deg) + '/s)', '(m)', '(m)'])
        self.TEST_LIST.append(nav_test)

    def gps_setup(self):
        gps_test = Test('GPS', GPSDATA, "GPSD_CLIENT", 'gps_handler', gpsd3_t, 15,
                        ['CHANNEL', 'MSGCOUNT', 'TIME', 'ONLINE', 'STATUS', 'NUM_SATS', 'MODE', 'LAT', 'LONG'],
                        ['Channel', 'Msg Count', 'Time', 'Online', 'Status', 'Satellites', 'Mode', 'Latitude', 'Longitude'],
                        [' ', ' ', '(' + eval(mu) + 's)', '(' + eval(mu) + 's)', ' ', ' ', ' ', '(' + eval(deg) + ')', '(' + eval(deg) + ')'])
        self.TEST_LIST.append(gps_test)

    def modem_setup(self):  # TODO = LT still to write LCM structure
        pass
        # modem_test = Test('MODEM', MODEMDATA, "?", modem_handler, modem_t, ['CHANNEL', 'TIME'])
        # self.TEST_LIST.append(modem_test)

    def tcm_setup(self):
        tcm_test = Test('TCM', TCMDATA, "TCM", 'tcm_handler', tcm_t, 10,
                        ['CHANNEL', 'MSGCOUNT', 'TIME', 'HEADING', 'ROLL', 'PITCH', 'TEMP'],
                        ['Channel', 'Msg Count', 'Time', 'Heading', 'Roll', 'Pitch', 'Temp'],
                        [' ', ' ', '(' + eval(mu) + 's)', '(' + eval(deg) + ')', '(' + eval(deg) + ')', '(' + eval(deg) + ')', '(' + eval(deg) + 'c)'])
        self.TEST_LIST.append(tcm_test)

    def dvl_setup(self, has_compass):
        if has_compass:
            dvl_test = Test('DVL + COMPASS', DVLDATA, "RDI", 'dvl_handler', rdi_pd5_t, 4,
                        ['CHANNEL', 'MSGCOUNT', 'TIME', 'CONFIG', 'SALINITY', 'DEPTH', 'ROLL', 'PITCH', 'HEADING', 'ALTITUDE', 'SOUND_SPEED', 'HEAD_TEMP', 'RANGE0', 'RANGE1', 'RANGE2', 'RANGE3'],
                        ['Channel', 'Msg Count', 'Time', 'Config', 'Salinity', 'Depth', 'Roll', 'Pitch', 'Heading', 'Altitude', 'Sound Speed', 'Head Temp', 'Range 0', 'Range 1', 'Range 2', 'Range 3'],
                        [' ', ' ', '(' + eval(mu) + 's)',' ', '(mg/l)', '(m)', '(m)', '(m)', '(' + eval(deg) + ')', '(m)', '(m/s)', '(' + eval(deg) + 'c)', '(m)', '(m)', '(m)', '(m)'])
        else:
            dvl_test = Test('DVL', DVLDATA, "RDI", 'dvl_handler', rdi_pd5_t, 4,
                        ['CHANNEL', 'MSGCOUNT', 'TIME', 'CONFIG', 'SALINITY', 'DEPTH', 'ROLL', 'PITCH', 'HEADING', 'ALTITUDE', 'SOUND_SPEED', 'HEAD_TEMP', 'RANGE0', 'RANGE1', 'RANGE2', 'RANGE3'],
                        ['Channel', 'Msg Count', 'Time', 'Config', 'Salinity', 'Depth', 'Roll', 'Pitch', 'Heading', 'Altitude', 'Sound Speed', 'Head Temp', 'Range 0', 'Range 1', 'Range 2', 'Range 3'],
                        [' ', ' ', '(' + eval(mu) + 's)',' ', '(mg/l)', '(m)', '(m)', '(m)', '(' + eval(deg) + ')', '(m)', '(m/s)', '(' + eval(deg) + 'c)', '(m)', '(m)', '(m)', '(m)'])
        self.TEST_LIST.append(dvl_test)

    def ysi_setup(self):
         ysi_test = Test('YSI', YSIDATA, "YSI", 'ysi_handler', ysi_t, 1,
                         ['CHANNEL', 'MSGCOUNT', 'TIME', 'TEMP', 'DEPTH', 'TURBIDITY', 'CHLOROPHYL', 'CONDUCTIVITY', 'OXYGEN', 'BATTERY', 'SALINITY'],
                         ['Channel', 'Msg Count', 'Time', 'Temperature', 'Depth', 'Turbidity', 'Chlorophyl', 'Conductivity', 'Oxygen', 'Battery', 'Salinity'],
                         [' ', ' ', '(' + eval(mu) + 's)', '(' + eval(deg) + 'c)', '(m)', '(NTU)', '(mg/l)', '(S/m)', '(mg/l)', '(V)', '(mg/l)'])
         self.TEST_LIST.append(ysi_test)

    # def tail_setup(self):
    #     tail_test = ActiveTest('TAIL FINS + THRUSTERS', TAILDATA, tail_publisher())
    #     self.ACTIVE_TEST_LIST.append(tail_test)


    def generate_test_list(self):

        self.summary_setup() # and add the summary data for the active tests etc
        self.hb_setup()   # always add HB to test list

        self.POSS_TESTS = {
                      'nav': 'self.nav_setup()',
                      'modem': 'self.modem_setup()',
                      'tcm': 'self.tcm_setup()',
                      'gps': 'self.gps_setup()',
                      # 'tail': 'self.tail_setup()', # TODO - Active Tests - go here? probably not
                      # 'cam_strobe': self.camera_strobe_setup(),
                       'dvl': 'self.dvl_setup(False)',
                       'dvl_c': 'self.dvl_setup(True)',
                      'ysi': 'self.ysi_setup()'}

        self.POSS_ACTIVE_TESTS = {
            'tail': 'tail_timer_setup()',  # TODO - Active Tests
            # 'cam_strobe': self.camera_strobe_setup()
        }

        # Run through the list of tests (from configs) and initialise the corresponding class + add to list
        for test in self.POSS_TESTS.keys():
            if vars(options).has_key(test):
                repeats = int(vars(options)[test])
                while repeats > 0:
                #if int(vars(options)[test]) > 0:
                    eval(self.POSS_TESTS[test])
                    repeats = repeats -1
                # tester functions complete setup for testing





    def run_tests(self):
        global th_fail
        try:
            # create new thread for the timer to run in
            self.test_timer = TimerThread("thread-timer", self.TEST_LIST, self.ACTIVE_TEST_LIST)
            #self.reset_msg_counters()
            self.test_timer.start()
        except:
            print "Error: unable to start test timer thread"
            th_fail = True


# -------------------------- LCM Handlers ----------------------------#
# LCM Incoming Message handlers
# and LCM Outgoing message handlers


# HB Message IN
def hb_handler(channel, data):
    global HBDATA
    #print 'Handling HB ... '
    msg = request_t.decode(data)
    HBDATA['CHANNEL'] = channel
    HBDATA['MSGCOUNT'] = HBDATA['MSGCOUNT'] + 1
    HBDATA['PREV_TIME'] = HBDATA['TIME']
    HBDATA['TIME'] = msg.utime
    HBDATA['DIFF'] = HBDATA['TIME'] - HBDATA['PREV_TIME']

# BATT Message IN
def batt_handler(channel, data):
    global BATTDATA
    #print 'Handling BATT ... '
    msg = os_power_system_t.decode(data)
    BATTDATA['CHANNEL'] = channel
    BATTDATA['TIME'] = msg.utime
    BATTDATA['CURRENT'] = msg.current
    BATTDATA['POWER'] = msg.power
    BATTDATA['AVGCHARGE'] = msg.avg_charge
    BATTDATA['CAPACITY'] = msg.capacity
    BATTDATA['CAPACITYFULL'] = msg.capacity_full
    BATTDATA['MINUTESTEF'] = msg.minutes_tef
    BATTDATA['NUMCONTROLLERS'] = msg.num_controller
    for int i in 1:BATTDATA['NUMCONTROLLERS']
        BATTDATA['CONTROLLER_'i''] = msg.controller
        self.utime = 0
        self.current = 0.0
        self.power = 0.0
        self.avg_charge_p = 0
        self.capacity = 0.0
        self.capacity_full = 0.0
        self.minutes_tef = 0
        self.sys_message = ""
        self.battery_state = ""
        self.num_batteries = 0
        self.battery = []
        for int j in 1:BATTDATA['CONTROLLER_' + i]
            BATTDATA['CONROLLER_' i '_BATTERY_' + j + '_TEMPERATURE'] = msg.controller[i].msg.temperature
            self.temperature = 0.0
            self.voltage = 0.0
            self.current = 0.0
            self.avg_current = 0.0
            self.remaining_capacity = 0.0
            self.full_capacity = 0.0
            self.charge_state = 0
            self.avg_tte = 0
            self.avg_ttf = 0
            self.serial_num = 0
            self.cycles = 0



# NAV Message IN
def nav_handler(channel, data):
    global NAVDATA
    #print 'Handling NAV ... '
    msg = auv_acfr_nav_t.decode(data)
    NAVDATA['CHANNEL'] = channel
    NAVDATA['MSGCOUNT'] = NAVDATA['MSGCOUNT'] + 1
    NAVDATA['TIME'] = msg.utime
    NAVDATA['LAT'] = msg.latitude
    NAVDATA['LONG'] = msg.longitude
    NAVDATA['X'] = round(msg.x, 8)
    NAVDATA['Y'] = round(msg.y, 8)
    NAVDATA['DEPTH'] = round(msg.depth, 8)
    NAVDATA['ROLL'] = round(msg.roll * 180/math.pi, 6)
    NAVDATA['PITCH'] = round(msg.pitch * 180/math.pi, 6)
    NAVDATA['HEADING'] = round(msg.heading * 180/math.pi, 6)
    NAVDATA['VX'] = round(msg.vx, 5)
    NAVDATA['VY'] = round(msg.vy, 5)
    NAVDATA['VZ'] = round(msg.vz, 5)
    NAVDATA['ROLLRATE'] = round(msg.rollRate * 180/math.pi, 6)
    NAVDATA['PITCHRATE'] = round(msg.pitchRate * 180/math.pi, 6)
    NAVDATA['HEADINGRATE'] = round(msg.headingRate * 180/math.pi, 6)
    NAVDATA['ALTITUDE'] = round(msg.altitude, 5)
    NAVDATA['FWD_OBST_DIST'] = round(msg.fwd_obstacle_dist, 8)


# evologics Modem Message IN - TODO LT still to write
def modem_handler(channel, data):
    global MODEMDATA
    #print 'Handling Modem ... '
    #msg = tcm_t.decode(data)
    #MODEMDATA[''] =


# GPS Message IN
def gps_handler(channel, data):
    #print '\nHandling GPS ... ' + repr(channel)
    msg = gpsd3_t.decode(data)
    GPSDATA['CHANNEL'] = channel
    GPSDATA['MSGCOUNT'] = GPSDATA['MSGCOUNT'] + 1
    GPSDATA['TIME'] = msg.utime
    GPSDATA['ONLINE'] = msg.online
    GPSDATA['STATUS'] = msg.status # 0:No Fix, 1:Fix, 2:DGPS Fix
    GPSDATA['NUM_SATS'] = msg.satellites_visible
    GPSDATA['MODE'] = msg.fix.mode
    GPSDATA['LAT'] = msg.fix.latitude * 180/math.pi
    GPSDATA['LONG'] = msg.fix.longitude * 180/math.pi


# TCM Message IN
def tcm_handler(channel, data):
   # print '\nHandling TCM ... '
    msg = tcm_t.decode(data)
    TCMDATA['CHANNEL'] = channel
    TCMDATA['MSGCOUNT'] = TCMDATA['MSGCOUNT'] + 1
    TCMDATA['TIME'] = msg.utime
    TCMDATA['HEADING'] = msg.heading * 180/math.pi
    TCMDATA['ROLL'] = msg.roll * 180/math.pi
    TCMDATA['PITCH'] = msg.pitch * 180/math.pi
    TCMDATA['TEMP'] = msg.temperature


# Tail Fins and Thruster - OUT
def tail_publisher(lcm_inst, top_angle, bottom_angle, port_angle, starboard_angle, rpm):
    msg = auv_iver_motor_command_t()  # set outgoing msg type
    msg.source = auv_iver_motor_command_t.REMOTE  # for override priority
    # set values
    msg.top = top_angle
    msg.bottom = bottom_angle
    msg.port = port_angle
    msg.starboard = starboard_angle
    msg.main = rpm
    if(abs(msg.main) < 40):
         msg.main = 0;
    msg.utime = int(round(time.time() * 1000))  # convert s to us for consistency
    # and publish
    lcm_inst.publish('IVER_MOTOR', msg.encode())
    # out only, no feedback message at this point


    #    msg.point2_x = float(sys.argv[2])

    #    lc.publish('TASK_PLANNER_COMMAND.'+sys.argv[1], msg.encode())
    #    lc.publish('IVER_MOTOR', msg.encode())
    #    lc.publish('IVER_MOTOR.TOP', msg.encode())


# Strobe - OUT # TODO OUT
def strobe_tester():
    print 'testing strobe(s)'
# no feedback message at this point


# Camera - OUT # TODO OUT
def camera_tester():
    print 'testing camera(s)'


# Camera Message TODO
def camera_handler(channel, data):
    #global CAMERADATA
    #print '\nHandling Camera ... '
    #msg = XXXXXX_t.decode(data)
    #CAMERADATA[''] =
    CAMERADATA['CHANNEL'] = channel


# DVL Message TODO - what do we need from this - also - intermittant in perlspy
def dvl_handler(channel, data):
    # print '\nHandling DVL ... '
    msg = rdi_pd5_t.decode(data)
    DVLDATA['CHANNEL'] = channel
    DVLDATA['MSGCOUNT'] = DVLDATA['MSGCOUNT'] + 1
    DVLDATA['TIME'] = msg.utime
    DVLDATA['CONFIG'] = msg.pd4.system_config
    DVLDATA['SALINITY'] = msg.salinity
    DVLDATA['DEPTH'] = msg.depth
    DVLDATA['ROLL'] = msg.roll
    DVLDATA['PITCH'] = msg.pitch
    DVLDATA['HEADING'] = msg.heading
    DVLDATA['ALTITUDE'] = msg.pd4.altitude
    DVLDATA['SOUND_SPEED'] = msg.pd4.speed_of_sound
    DVLDATA['HEAD_TEMP'] = msg.pd4.xducer_head_temp
    DVLDATA['RANGE0'] = msg.pd4.range[0]
    DVLDATA['RANGE1'] = msg.pd4.range[1]
    DVLDATA['RANGE2'] = msg.pd4.range[2]
    DVLDATA['RANGE3'] = msg.pd4.range[3]

# YSI Message
def ysi_handler(channel, data):
    # print '\nHandling YSI ... '
    msg = ysi_t.decode(data)
    YSIDATA['CHANNEL'] = channel
    YSIDATA['MSGCOUNT'] = YSIDATA['MSGCOUNT'] + 1
    YSIDATA['TIME'] = msg.utime
    YSIDATA['TEMP'] = msg.temperature
    YSIDATA['DEPTH'] = msg.depth
    YSIDATA['TURBIDITY'] = msg.turbidity
    YSIDATA['CHLOROPHYL'] = msg.chlorophyl
    YSIDATA['CONDUCTIVITY'] = msg.conductivity
    YSIDATA['OXYGEN'] = msg.oxygen
    YSIDATA['BATTERY'] = msg.battery
    YSIDATA['SALINITY'] = msg.salinity


# lc.subscribe("CAMERA_TRIGGER.OUT", camera_handler) # need to publish to CAMERA_TRIGGER to get an image to be taken - camcontrol.py example then also need to trigger w camtigger.py
# lc.subscribe("ACFR_AUV_VIS_RAWLOG", rawlog_handler) #TODO - can't find this - do we need?
# lc.subscribe("BATTERY", battery_handler) #TODO - later

# TODO Do we need? they exist in pyspy
# AUVSTAT.IVERACFR
# PARAM_UPDATE
# PATH_RESPONSE task planned global planner dubbins path planner (circle)



# TODO - LCM Outgoing Message handlers - publishers example

    # Set msg type
    #    msg = auv_global_planner_t()


    #    msg.point2_x = float(sys.argv[2])

    #    lc.publish('TASK_PLANNER_COMMAND.'+sys.argv[1], msg.encode())
    #    lc.publish('IVER_MOTOR', msg.encode())
    #    lc.publish('IVER_MOTOR.TOP', msg.encode())


# -------------------------- End of Handlers ----------------------------#


# ------------------------------ Interface ------------------------------#

def decktestinterface(stdscr):

    global vehicle_name
    global column_width

    # create the test set as requested by options
    decktest = Test_Set(options, vehicle_name)
    # generate a test object for each test
    decktest.generate_test_list()
    # summary is created separately, and Hearbeat is automatically added to list

    # display box dimensions
    max_box_entries = 20
    extra_space = 40
    padsize_height = max_box_entries * len(vars(options).keys()) + extra_space

    # box layout calculation values
    gutter = 1
    edge = 1
    out_col_width = column_width + 2 * (gutter + edge)

    screen_height,screen_width = stdscr.getmaxyx()

    if screen_width < out_col_width:
        num_columns = 1
        padsize_width = out_col_width
    else:
        num_columns = screen_width // out_col_width
        if num_columns > max_cols:
            num_columns = max_cols
        padsize_width = out_col_width * num_columns

    pad = curses.newpad(padsize_height, padsize_width) # TODO


    # If we make it here, we should have some tests to run:

    # Setup LCM
    lc = lcm.LCM()

    def lcm_thread(threadName, delay):
        # LCM Message subscriptions
        # Subscribe and handle messages for tests chosen
        # in config file/command line
        # process all contents, but only display
        # values included in test initialization and

        for test in decktest.TEST_LIST:
            lc.subscribe(test.channel, eval(test.handler))

        # Loop forever 'handling'
        while True:
            lc.handle()

    # Create LCM Subscriber Thread
    try:
         thread.start_new_thread(lcm_thread, ('thread-lcm', 1))
    except:
        print "Error: unable to start thread"
        # TODO - What other tests need threads


    # Create two threads as follows
    # try:
    #     # thread.start_new_thread(check_reedswitch, ("thread-ctl", 0.05, 1))
    #     # thread.start_new_thread(get_imagecount, ("thread-img", 1))
    #     # thread.start_new_thread(get_sysinfo, ("thread-sys", 2))
    #     # #    thread.start_new_thread( get_lcmcompassinfo, ('thread-cmp', 5))
    #     # thread.start_new_thread(update_display, ("thread-disp", 0.5))
    # except:
    #     print "Error: unable to start thread"

    #----------------------------- Curses Interface -----------------------------#

    stdscr.clear()

    # curses colour configs
    curses.init_pair(c_BLUE, curses.COLOR_BLUE, curses.COLOR_BLACK)
    curses.init_pair(c_RED, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(c_YELLOW, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(c_GREEN, curses.COLOR_GREEN, curses.COLOR_BLACK)

    def drawbgd():
        stdscr.clear()

        # draw border
        stdscr.border(0)
        stdscr.noutrefresh()

        # draw title
        title = curses.newwin(title_box_height, (num_columns * column_width) + ((num_columns -1) * gutter), 1, 2)
        title.border(0)
        title1 = 'ACFR AUV - LCM Decktest'
        title2 = 'Vehicle: ' + vehicle_name
        title.addstr(1, ((((num_columns * column_width) + gutter) - len(title1)) / 2), title1, curses.color_pair(1))
        title.addstr(2, ((((num_columns * column_width) + gutter) - len(title2)) / 2), title2, curses.color_pair(1))
        title.addstr(1, 2, 'Quit: \'q\'', curses.color_pair(1))
        title.addstr(2, 2, 'Run: \'r\'', curses.color_pair(1))
        title.noutrefresh()

    # layout calculation values
    column_y_sums = [title_box_height + gutter, title_box_height + gutter, title_box_height + gutter, title_box_height + gutter, title_box_height + gutter]

    # draw test summary box in first column
    decktest.test_stats.create_disp_box(column_y_sums[0] , left_in)
    column_y_sums[0] = column_y_sums[0] + edge + len(decktest.test_stats.fields) + edge

    # print channels and handlers for troubleshooting
    # i=0
    # for test in decktest.TEST_LIST:
    #     stdscr.addstr(50+i, 10, '\"' + test.channel + '\"')
    #     stdscr.addstr(60+i, 10, test.handler)
    #     i = i+1

    # then for rest of tests get each test to write it's box, in the next shortest column
    for test in decktest.TEST_LIST:
        col_num = column_y_sums.index(min(column_y_sums[0:num_columns]))
        test.create_disp_box(column_y_sums[col_num], left_in + col_num * (column_width + gutter))
        column_y_sums[col_num] = column_y_sums[col_num] + edge + len(test.fields) + edge

    curses.curs_set(0)  # to make cursor invisible
    curses.halfdelay(5)  # set timeout on keyboard input to half a second, so that loop continues

    # Main display loop - wait (forever) so we can see the results
    while True:
        # update background, title etc
        drawbgd()

        decktest.test_stats.write_disp_box()

        # get each test to update its info without re-draw
        for test in decktest.TEST_LIST:
            test.write_disp_box()

        curses.doupdate() # then re-draw all to prevent flicker
        time.sleep(1)  # (s) delay to make readable

        # keep displaying values with a 1 second refresh until receive 'q' to quit
        c = stdscr.getch()
        if c == ord('r'):   #TODO - start the active tests
            if test_started == False:
                decktest.run_tests()

        elif c == ord('q'):
            break  # Exit the while loop

#--------------------------- End of Interface ---------------------------#


#----------------------------- MAIN PROGRAM -----------------------------#
#----------------- Command Line and File Option Management --------------#

# Run config and feedback, before call Curses wrapper for interface

# Command Line and config file args read using ConfigArgParse
# reads config file specific to vehicle_name
vehicle_name = socket.gethostname()

#config_path = str('/home/jmartin/GIT/acfr_lcm/scripts/Decktest/configs/decktest_config-' + vehicle_name + '.ini')   # TODO confirm actual path
p = configargparse.ArgParser(default_config_files=['configs/decktest_config-' + vehicle_name + '.ini'])
p.add('-nav', '--nav', help ='Set 0 or 1 acfr_nav module to test')
p.add('-modem', '--modem', help ='Set 0 or 1 Evologics Modem to test')
p.add('-tcm', '--tcm', help ='Set 0 or 1 TCM to test')
p.add('-gps', '--gps', help ='Set 0 or 1 GPS to test')
p.add('-tail', '--tail', help ='Set 0 or 1 Tail Thrusters + Fin sets to test')
p.add('-stb', '--strobe', help ='Set number of Strobes to test')
p.add('-cam', '--camera', help ='Set number of Cameras to test')
p.add('-dvl', '--dvl', help ='Set number of Doppler Velocity Loggers to test')
p.add('-dvl_c', '--dvl_c', help ='Set number of DVLs with Compass to test')
p.add('-ysi', '--ysi', help ='Set number of YSI to test')

options = p.parse_args()

# If no test options are called, (or all called w zero) no point running:
sum = 0
opt_vals = vars(options).values()
for i in opt_vals:
    if (i != None):
        if i.isdigit():
            sum += int(i)
if sum < 1:
    print '\n\n----------------------------------------------------------\n'
    print '          Starting Decktest for ' + vehicle_name + '\n'
    print '----------------------------------------------------------\n'
    print('No (non-zero) test configurations detected on command line or in config file - refer to usage instructions\n')
    print(p.format_help()) # TODO confirm actual path and add info
    exit(0)

# print(p.format_values()) # useful for logging where different settings came from

#---------------------------- Interface Wrap -------------------------------#

# Wrap Curses Interface to ensure terminal is returned to normal operation on exit
# calls the main program (decktestinterface)

try:
    curses.wrapper(decktestinterface)
except KeyboardInterrupt:
    print "Got KeyboardInterrupt exception. LCM Decktest now Exiting"
    exit(1)

#------------------------- Program Exit Message ----------------------------#

print 'Quit (q) Selected. LCM Decktest now Exiting'
exit(0)

#------------------------------- Program END -------------------------------#




# Every time we get a AUV_RAWLOG message we just increment the image counter
# def rawlog_handler(channel, data):
#    global IMGDATA
#    msg = auv_vis_rawlog_t.decode(data)
#    IMGDATA['#IMS'] += 1
#
## messages from the Oceam server compass
# def compass_handler(channel, data):
#    global NAVDATA
#    msg = os_compass_t.decode(data)
#    rph = [i * 180/math.pi for i in msg.rph] # convert to degrees
#    NAVDATA['ROLL'] = '{:0.1f}'.format(rph[0])
#    NAVDATA['PTCH'] = '{:0.1f}'.format(rph[1])
#    NAVDATA['HDNG'] = '{:0.1f}'.format(rph[2])
#    NAVDATA['DEPT'] = '{:0.3f}'.format(msg.depth)

