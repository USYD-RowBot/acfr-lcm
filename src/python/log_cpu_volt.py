#!/usr/bin/env python
import sensors
import time
import io
import sys
import lcm
import string
import os, os.path

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from perllcm.heartbeat_t import heartbeat_t
from acfrlcm import auv_cpu_voltage_monitor_t

lc = lcm.LCM()
sensors.init()

def heartbeat_handler(channel, data):
    hb = heartbeat_t.decode(data)
    vm = auv_cpu_temp_monitor_t()
    current_volt = list()
    name = list()

    for chip in sensors.iter_detected_chips():
        #print '%s at %s' % (chip, chip.adapter_name)
        for feature in chip:
            f_val = feature.get_value()
            #print '  %s: %.2f' % (feature.label, f_val)
            if ('V' in feature.label):
                current_volt.append(f_val)
                name.append(feature.label)
    vm.utime = hb.utime
    vm.num_readings = len(current_volt)
    vm.names = list(name)
    vm.volt_readings = list(current_volt)
    lc.publish("CPU_VOLT", vm.encode())        
        
lc.subscribe("HEARTBEAT_1HZ", heartbeat_handler)

while (1):
    lc.handle()

sensors.cleanup()
exit
