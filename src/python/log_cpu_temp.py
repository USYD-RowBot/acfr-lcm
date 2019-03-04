import time
import io
import sys
import lcm
import string

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from perllcm.heartbeat_t import heartbeat_t

lc = lcm.LCM()
    
def heartbeat_handler(channel, data):
	hb = heartbeat_t.decode(data)

	with open('/sys/class/hwmon/hwmon2/temp1_input', 'r') as temp_file:
		current_temp = temp_file.read()

	with open('temp_log.txt', 'a') as log_file:
		log_file.write('{},{}\n'.format(hb.utime, current_temp))

lc.subscribe("HEARTBEAT_1HZ", heartbeat_handler)

while True:
    lc.handle()
	
    
    


