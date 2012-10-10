# Backseat-LCM interface for HAUV
# Michael Kaess
# based on example code by Mike Elkins (Bluefin)

import sys
import socket
import random
import getopt
import threading
import signal

import datetime
import time
import lcm
import util

# messages from backseat driver to payload
from perls.lcmtypes.hauv.bs_raw_t import bs_raw_t # any message in string format
from perls.lcmtypes.hauv.bs_nvg_t import bs_nvg_t # navigation update
from perls.lcmtypes.hauv.bs_nvr_t import bs_nvr_t # velocity and rate update
from perls.lcmtypes.hauv.bs_rcm_t import bs_rcm_t # raw compass data
from perls.lcmtypes.hauv.bs_rdp_t import bs_rdp_t # raw depth sensor data
from perls.lcmtypes.hauv.bs_imu_t import bs_imu_t # raw IMU data
from perls.lcmtypes.hauv.bs_dvl_t import bs_dvl_t # raw DVL data
from perls.lcmtypes.hauv.bs_dvl_2_t import bs_dvl_2_t # raw DVL data
from perls.lcmtypes.hauv.bs_rbs_t import bs_rbs_t # battery voltage
from perls.lcmtypes.hauv.bs_rnv_t import bs_rnv_t # relative navigation position
from perls.lcmtypes.hauv.bs_rnv_2_t import bs_rnv_2_t # 
from perls.lcmtypes.hauv.bs_pit_t import bs_pit_t # pitch servo positions
from perls.lcmtypes.hauv.bs_cnv_t import bs_cnv_t # 

# messages from payload to backseat driver
from perls.lcmtypes.hauv.pl_raw_t import pl_raw_t # any message in string format
from perls.lcmtypes.hauv.pl_san_t import pl_san_t # set sonar angle
from perls.lcmtypes.hauv.pl_ghp_t import pl_ghp_t # go to hull position
from perls.lcmtypes.hauv.pl_gbp_t import pl_gbp_t # go to bottom position
from perls.lcmtypes.hauv.pl_sus_t import pl_sus_t # suspend mission
from perls.lcmtypes.hauv.pl_res_t import pl_res_t # resume mission
from perls.lcmtypes.hauv.pl_rbo_t import pl_rbo_t # relative navigation bearing offset
from perls.lcmtypes.hauv.pl_rns_t import pl_rns_t # reset coordinate frame (according to Mike, using current x and y, it should behave exactly like RBO...)

lc = lcm.LCM()

# forwarding backseat driver messages for Seebyte
sem = threading.Semaphore()
conn_sem = None
def forward_listen(f):
    global conn_sem
    global sem
    while 1:
        time.sleep(0.01);
        sem.acquire();
        if conn_sem is None:
            sem.release();
            print 'Waiting for forwarding connection...'
            conn, addr = f.accept()
            print 'Forwarding connection established with', addr
            sem.acquire();
            conn_sem = conn
        else:
            # forward Seebyte messages
            try:
                data = conn_sem.recv(1024, socket.MSG_DONTWAIT)
                s.send(data)
            except socket.error as e:
                ()
        sem.release();

def backseat_handler(msg):
    time_received = util.timeNow()
    raw = bs_raw_t()
    raw.message = msg
    lc.publish('HAUV_BS_RAW', raw.encode())
    list = msg[3:-3].split(',')
    id = list[0][0:3]
    if msg[:3] == '$BF':
        if id == 'NVG':
            nvg = bs_nvg_t()
            nvg.time_received = time_received
            nvg.time = util.time_hauv2long(list[1])
            nvg.latitude = float(list[2])
            nvg.hemisphere_ns = list[3]
            nvg.longitude = float(list[4])
            nvg.hemisphere_ew = list[5]
            nvg.quality = float(list[6])
            nvg.altitude = float(list[7])
            nvg.depth = float(list[8])
            nvg.heading = util.deg2rad(float(list[9]))
            nvg.roll = util.deg2rad(float(list[10]))
            nvg.pitch = util.deg2rad(float(list[11]))
            nvg.time_compute = util.time_hauv2long(list[12])
            lc.publish('HAUV_BS_NVG', nvg.encode())
            print 'NVG',
        elif id == 'NVR':
            nvr = bs_nvr_t()
            nvr.time_received = time_received
            nvr.time = util.time_hauv2long(list[1])
            nvr.east_velocity = float(list[2])
            nvr.north_velocity = float(list[3])
            nvr.down_velocity = float(list[4])
            nvr.pitch_rate = float(list[5])
            nvr.roll_rate = float(list[6])
            nvr.yaw_rate = float(list[7])
            lc.publish('HAUV_BS_NVR', nvr.encode())
            print 'NVR',
        elif id == 'RCM':
            rcm = bs_rcm_t()
            rcm.time_received = time_received
            rcm.time = util.time_hauv2long(list[1])
            rcm.compass_number = integer(list[2])
            rcm.heading = float(list[3])
            rcm.pitch = float(list[4])
            rcm.roll = float(list[5])
            rcm.time_measured = util.time_hauv2long(list[6])
            lc.publish('HAUV_BS_RCM', rcm.encode())
            print 'RCM',
        elif id == 'RDP':
            rdp = bs_rdp_t()
            rdp.time_received = time_received
            rdp.time = util.time_hauv2long(list[1])
            rdp.pressure = float(list[2])
            lc.publish('HAUV_BS_RDP', rdp.encode())
            print 'RDP',
        elif id == 'IMU':
            imu = bs_imu_t()
            imu.time_received = time_received
            imu.time = util.time_hauv2long(list[1])
            imu.rate_x = util.deg2rad(float(list[2]))
            imu.rate_y = util.deg2rad(float(list[3]))
            imu.rate_z = util.deg2rad(float(list[4]))
            imu.acc_x = float(list[5])
            imu.acc_y = float(list[6])
            imu.acc_z = float(list[7])
            imu.time_measured = util.time_hauv2long(list[8])
            lc.publish('HAUV_BS_IMU', imu.encode())
            print 'IMU',
        elif id == 'DVL':
            dvl = bs_dvl_t()
            dvl.time_received = time_received
            dvl.time = util.time_hauv2long(list[1])
            if list[2]!="" and list[3]!="" and list[4]!="" and list[5]!="" and list[6]!=""  and list[7]!=""  and list[8]!="":
                dvl.x_velocity = float(list[2])
                dvl.y_velocity = float(list[3])
                dvl.z_velocity = float(list[4])
                dvl.range1 = float(list[5])
                dvl.range2 = float(list[6])
                dvl.range3 = float(list[7])
                dvl.range4 = float(list[8])
                dvl.temperature = float(list[9])
                dvl.time_measured = util.time_hauv2long(list[10])
                lc.publish('HAUV_BS_DVL', dvl.encode())
                print 'DVL',
        elif id == 'DV2':
            dvl = bs_dvl_2_t()
            dvl.time_received = time_received
            dvl.time = util.time_hauv2long(list[1])
            if list[2]!="" and list[3]!="" and list[4]!="" and list[5]!="" and list[6]!=""  and list[7]!=""  and list[8]!="":
                dvl.velocity1 = float(list[2])
                dvl.velocity2 = float(list[3])
                dvl.velocity3 = float(list[4])
                dvl.velocity4 = float(list[5])
                dvl.range1 = float(list[6])
                dvl.range2 = float(list[7])
                dvl.range3 = float(list[8])
                dvl.range4 = float(list[9])
                dvl.temperature = float(list[10])
                dvl.time_measured = util.time_hauv2long(list[11])
                lc.publish('HAUV_BS_DVL_2', dvl.encode())
                print 'DV2',
        elif id == 'RBS':
            if list[6]!="":
              rbs = bs_rbs_t()
              rbs.time_received = time_received
              rbs.time = util.time_hauv2long(list[1])
              rbs.battery_number = int(list[2])
              rbs.voltage = float(list[3])
              rbs.minimum_cell_voltage = float(list[4])
              rbs.maximum_cell_voltage = float(list[5])
              rbs.temperature = float(list[6])
              lc.publish('HAUV_BS_RBS', rbs.encode())
              print 'RBS'  # line break to avoid infinitly long lines with procman
        elif id == 'RNV':
            if len(list) == 6:
                rnv = bs_rnv_t()
                rnv.time_received = time_received
                rnv.time = util.time_hauv2long(list[1])
                rnv.horizontal = float(list[2])
                rnv.vertical = float(list[3])
                rnv.distance = float(list[4])
                rnv.heading = util.deg2rad(float(list[5]))
                lc.publish('HAUV_BS_RNV', rnv.encode())
                print 'RNV',
            else:
                rnv = bs_rnv_2_t()
                rnv.time_received = time_received
                rnv.time = util.time_hauv2long(list[1])
                rnv.horizontal = float(list[2])
                rnv.vertical = float(list[3])
                rnv.distance = float(list[4])
                rnv.heading = util.deg2rad(float(list[5]))
                rnv.depth = float(list[6])
                rnv.absheading = util.deg2rad(float(list[7]))
                rnv.absroll = util.deg2rad(float(list[8]))
                rnv.abspitch = util.deg2rad(float(list[9]))
                rnv.time_nav = long(1000000*float(list[10])) #util.time_hauv2long(list[10])
                lc.publish('HAUV_BS_RNV_2', rnv.encode())
                print 'RNV2',
        elif id == 'CNV':
            cnv = bs_cnv_t()
            cnv.time_received = time_received
            cnv.time_nav = long(1000000*float(list[1])) #util.time_hauv2long(list[1])
            cnv.x = float(list[2])
            cnv.y = float(list[3])
            cnv.z = float(list[4])
            cnv.heading = util.deg2rad(float(list[5]))
            lc.publish('HAUV_BS_CNV', cnv.encode())
            print 'CNV',
        elif id == 'PIT':
            pit = bs_pit_t()
            pit.time_received = time_received
            pit.time = util.time_hauv2long(list[1])
            pit.pitch_dvl = util.deg2rad(float(list[2]))
            pit.pitch_sonar = util.deg2rad(float(list[3]))
            lc.publish('HAUV_BS_PIT', pit.encode())
            print 'PIT',
        sys.stdout.flush()

def backseat_listen(s):
    global conn_sem
    global sem
    while 1:

        msg = ''
        while 1:
            while msg.find('*') == -1:
                chunk = s.recv(1024)
                if chunk == '':
                    print "socket connection broken"
                    time.sleep(0.1)

                # forward data if connection established
                sem.acquire()
                if conn_sem is not None:
                    try:
                        conn_sem.send(chunk)
                    except socket.error as e:
                        print 'Forwarding connection lost'
                        conn_sem = None
                sem.release()

                # todo: reconnect...
                msg = msg + chunk
            pos = msg.find('*')+3
            backseat_handler(msg[:pos])
            # keep any parts of next message
            msg = msg[pos+2:] # +2: skip endline as msg.rstrip('\r\n') does not work...

def parseAddr(hostAndPort):
    """Turn host:port into a tuple of (hostName,portNum)"""
    parts = hostAndPort.split(':')
    if len(parts) != 2:
        raise Exception("Expected address to be hostname:port")
    return (parts[0],int(parts[1]))

def checksum(text):
    # Just XOR the 8 bits, repeatedly.
    sum = 0
    for c in text:
        sum ^= ord(c) 
    return sum
    
def payloadCommand(s,cmd,*args):
    msg = '$BP'+cmd#+",000000.00"
    for a in args:
        msg += ',' + str(a)
    # Now add checksum
    msg += ("*%02x"%checksum(msg[1:]))#[2:] # Skip $, Skip 0x
    #   print msg
    s.send(msg+'\r\n')

def lcm_handler(channel, data):
    id = channel[8:11]
    if id == 'RAW':
        raw = pl_raw_t.decode(data)
        payloadCommand(s, 'RAW', raw.message)
        print '*RAW',
    elif id == 'SUS':
        sus = pl_sus_t.decode(data)
        payloadCommand(s, 'SUS', util.time_long2hauv(sus.time))
        print '*SUS',
    elif id == 'RES':
        res = pl_res_t.decode(data)
        payloadCommand(s, 'RES', util.time_long2hauv(res.time))
        print '*RES',
    elif id == 'SAN':
        san = pl_san_t.decode(data)
        payloadCommand(s, 'SAN', util.time_long2hauv(san.time), util.rad2deg(san.angle))
        print '*SAN',
    elif id == 'GHP':
        ghp = pl_ghp_t.decode(data)
        payloadCommand(s, 'GHP', util.time_long2hauv(ghp.time), ghp.horizontal, ghp.vertical, ghp.distance)
        print '*GHP',
    elif id == 'GBP':
        gbp = pl_gbp_t.decode(data)
        if gbp.is_depth:
            flag = 'D'
        else:
            flag = 'A'
        payloadCommand(s, 'GBP', util.time_long2hauv(gbp.time), gbp.x, gbp.y, flag, gbp.depth_altitude)
        # util.rad2deg(gbp.relative_bearing), - parameter was decomissioned by Bluefin, we'll leave it in LCM to stay compatible
        print '*GBP',
    elif id == 'RBO':
        rbo = pl_rbo_t.decode(data)
        payloadCommand(s, 'RBO', util.time_long2hauv(rbo.time), "%.1f" % util.rad2deg(rbo.angle_offset))
        print '*RBO',
    elif id == 'RNS':
        rns = pl_rns_t.decode(data)
        payloadCommand(s, 'RNS', util.time_long2hauv(rns.time), rns.x, rns.y, "%.1f" % util.rad2deg(rns.bearing_offset))
        print '*RNS',
    sys.stdout.flush()

def lcm_listen():
    subscription = lc.subscribe("HAUV_PL_RAW|HAUV_PL_SUS|HAUV_PL_RES|HAUV_PL_SAN|HAUV_PL_GHP|HAUV_PL_GBP|HAUV_PL_RBO|HAUV_PL_RNS", lcm_handler)

    while 1:
        lc.handle()

def nop():
    return

if __name__ == '__main__':
    # HAUV address and port
    serverAddr = ('10.0.1.112',29500)
#    serverAddr = ('10.0.3.96',29500)
    # forwarding service for Seebyte
    forwardAddr = ('',50126)
    # what log messages to receive
#    messages = ['NVG','NVR','RCM','RDP','RVL','IMU']
#    messages = ['RNV']
    messages = ['ALL','PIT','DV2']
    optlist,args = getopt.gnu_getopt(sys.argv[1:],'s:f:')
    if len(args) != 0:
        raise Exception("Unknown command line argument")

    for o in optlist:
        if o[0] == '-s':
            serverAddr = parseAddr(o[1])
        if o[0] == '-f':
            forwardAddr = parseAddr(o[1])

    print 'waiting for connection'

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect(serverAddr)
    for msg in messages:
        s.send("$BPLOG,%s,ON\r\n"%msg)

    f = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    f.bind(forwardAddr);
    f.listen(1)
    
    print 'connected'

    print 'creating threads'

    forwarder = threading.Thread(target=forward_listen,args=(f,))
    forwarder.start()

    listener = threading.Thread(target=backseat_listen,args=(s,))
    listener.start()

    lcm_listener = threading.Thread(target=lcm_listen,args=())
    lcm_listener.start()

    print 'created threads'

    try:
        while True:
            time.sleep(0.5);
    except KeyboardInterrupt:
        print 'exiting'
        exit()

#    cmd = { 'q':myexit }
#    while 1:
#        c = util.getch()
#        cmd.get(c,nop)
