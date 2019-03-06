import time
import io
import sys
import lcm
import string
import os, os.path

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from perllcm.heartbeat_t import heartbeat_t

lc = lcm.LCM()
# find the right dir that stores coretemp
DIR = '/sys/class/hwmon/'
available_dirs = os.listdir(DIR)
right_dir = list()
for name in available_dirs:
	try:
		with open('/sys/class/hwmon/'+name+'/name', 'r') as check_file:
			if 'temp' in check_file.read():
				right_dir.append(name)
				print 'coretemp stored here:/sys/class/hwmon/'+right_dir+'/temp1_input'
		with open('/sys/class/hwmon/'+name+'/device/name', 'r') as check_file:
			if 'temp' in check_file.read():
				right_dir.append(name)
				print 'coretemp stored here:/sys/class/hwmon/'+right_dir+'/device/temp1_input'
	except Exception:
		pass
    
def heartbeat_handler(channel, data):
	hb = heartbeat_t.decode(data)
	current_temp = list()
	for folder in right_dir:
		file_names = os.listdir('/sys/class/hwmon/'+ folder)
		for file_name in file_names:
			if 'input' in file_name:
				with open('/sys/class/hwmon/'+folder+'/'+file_name, 'r') as temp_file:
					current_temp.append(temp_file.read())


	with open('temp_log.txt', 'a') as log_file:
		log_file.write(str(hb.utime).rstrip('\n'))
		for value in current_temp:
			log_file.write(str(','+value[:-4]).rstrip('\n'))
		log_file.write('\n')

lc.subscribe("HEARTBEAT_1HZ", heartbeat_handler)

while True:
    lc.handle()
	
    
    


