import serial
import time
import io
import sys
import lcm
import string

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from senlcm.bk9115_t import bk9115_t
from perllcm.heartbeat_t import heartbeat_t

ser = serial.Serial(
    port='/dev/ttyUSB2',
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS)

lc = lcm.LCM()
    
def heartbeat_handler(channel, data):
	cmd = 'MEAS:SCAL:VOLT?\r\n'
	ser.write(cmd)
	voltage = ser.readline()

	cmd = 'MEAS:SCAL:CURR?\r\n'
	ser.write(cmd)
	current = ser.readline()
	
	msg = bk9115_t()
	msg.utime = int(time.time() * 1000000)
	msg.voltage = string.atof(voltage)
	msg.current = string.atof(current);
	lc.publish("BK9115", msg.encode())
	

lc.subscribe("HEARTBEAT_1HZ", heartbeat_handler)

while True:
    lc.handle()
	
    
    


