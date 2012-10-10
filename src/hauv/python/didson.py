import sys
import socket
import random
import getopt
import threading
import array

import math
import datetime
import time
import lcm
import util

from perls.lcmtypes.hauv.didson_raw_t import didson_raw_t

lc = lcm.LCM()

def listen(s):
    fs = open('didson.log', 'w')
    fc = open('client.log', 'w')
    while 1:
        data, address = s.recvfrom(1024*10)
        time_received = util.timeNow()
        if data == '':
            print 'WARNING: empty message...'
        if address == ('10.0.1.117',700):
            print 'd',
            print len(data),
            if ord(data[1]) <> 0x5a:
                print 'WARNING: wrong header'
            didson = didson_raw_t()
            didson.time_received = time_received
            didson.length = len(data)
            didson.data = array.array('b',data)
            lc.publish('HAUV_DIDSON', didson.encode())
            fs.write(data)
            fs.flush
        else: # must be a command... / if address == ('10.0.3.104',700):
            print 'c',
            print len(data),
            if ord(data[1]) <> 0xa5:
                print 'WARNING: wrong header'
            didson = didson_raw_t()
            didson.time_received = time_received
            didson.length = len(data)
            didson.data = array.array('b',data)
            lc.publish('HAUV_DIDSON_CLIENT', didson.encode())
            fc.write(data)
            fc.flush

def myexit(s):
    exit()

def nop(s):
    return

if __name__ == '__main__':

    # UDP connection
    s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(('',700))

    listener = threading.Thread(target=listen,args=(s,))
    listener.start()
    
    cmd = { 'q':myexit }
    while 1:
        c = util.getch()
        cmd.get(c,nop)(s)
