import time
import io
import sys
import lcm
import string
import os, os.path

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from perllcm.heartbeat_t import heartbeat_t
from acfrlcm import auv_cpu_temp_monitor_t

lc = lcm.LCM()
#vehicle_name = sys.argv[1];
# find the right dir that stores coretemp
DIR = '/sys/class/hwmon/'
available_dirs = os.listdir(DIR)
right_dir = list()
global max_val
max_val = 0
for name in available_dirs:
	try:
		with open('/sys/class/hwmon/'+name+'/name', 'r') as check_file:
			if 'temp' in check_file.read():
				right_dir.append(name)
				print 'coretemp stored here:/sys/class/hwmon/'+name+'/temp1_input'
		with open('/sys/class/hwmon/'+name+'/device/name', 'r') as check_file:
			if 'temp' in check_file.read():
				right_dir.append(name)
				print 'coretemp stored here:/sys/class/hwmon/'+name+'/device/temp1_input'
	except Exception:
		pass
    
def heartbeat_handler(channel, data):
	hb = heartbeat_t.decode(data)
	tm = auv_cpu_temp_monitor_t()
	current_temp = list()
	total = 0
	global max_val
	for folder in right_dir:
		file_names = os.listdir('/sys/class/hwmon/'+ folder)
		for file_name in file_names:
			if 'input' in file_name:
				with open('/sys/class/hwmon/'+folder+'/'+file_name, 'r') as temp_file:
					str_val = temp_file.read()
					int_val = int(str_val[:-4])
					total += int_val
					current_temp.append(str_val[:-4])
					if int_val > max_val:
						max_val = int_val

	tm.utime = hb.utime
	tm.num_readings = len(current_temp)
	tm.temp_readings = list(map(int, current_temp))
	tm.all_time_max = max_val
	tm.current_avg = total/len(current_temp)
	lc.publish("CPU_TEMP", tm.encode())
	#lc.publish(vehicle_name+".CPU_TEMP", tm.encode())
	# with open('temp_log.txt', 'a') as log_file:
	# 	log_file.write(str(hb.utime).rstrip('\n'))
	# 	for value in current_temp:
	# 		log_file.write(str(','+value).rstrip('\n'))
	# 	log_file.write('\n')

lc.subscribe("HEARTBEAT_1HZ", heartbeat_handler)


while (1):
	lc.handle()


exit
    
    


