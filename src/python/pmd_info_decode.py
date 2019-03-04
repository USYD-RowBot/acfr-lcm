#!/usr/bin/env python
import lcm
import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
from bot_procman import info2_t

if (len(sys.argv) < 2):
	exit
lcm_url = 'file://'+sys.argv[1]+'?speed=0'
print lcm_url
lc = lcm.LCM(lcm_url)

dd = {} 

def handle_info(channel_name, raw_data):
    msg = info2_t.decode(raw_data)


    # and the deputy says what it is
    # this might be the one to extract program information from

    # go through all the commands and add the sid to the command name
    # this makes it easier to track across multiple instantiations
    if msg.host not in dd:
        dd[msg.host] = []
        print msg.host
    dd[msg.host].append(msg)  

    
lc.subscribe("PMD_INFO2", handle_info)

while True:
    try:
        lc.handle()
    except IOError:
        break

for key, value in dd.iteritems():
    filename = '{}.txt'.format(key)
    with open(filename, 'w') as f:
        for msg in value:
            try:
                f.write('{},{}\n'.format(msg.utime, msg.cpu_load))
            except:
                print 'bad data'


