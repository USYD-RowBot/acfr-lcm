#!/usr/bin/env python
import socket
import time

HOST = '172.16.154.169' 
PORT = 966
PORT_DATA = 4001
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
DATA = '\x21\x00'
s.send(DATA)
data = s.recv(4096)
d = data
print 'Received', repr(d)
time.sleep(0.1)
DATA = '\x22\x00'
s.send(DATA)
data = s.recv(4096)
s.close()
d = data
print 'Received', repr(d)

