#!/usr/bin/env python
import lcm
import sys

sys.path.append('/home/git/acfr_lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
from bot_procman import printf_t

if (len(sys.argv) < 2):
	exit
lcm_url = 'file://'+sys.argv[1]+'?speed=0'
print lcm_url
lc = lcm.LCM(lcm_url)

dd = {}


def handler(channel, data):
    msg = printf_t.decode(data)
    if msg.sheriff_id not in dd:
        dd[msg.sheriff_id] = []
        print msg.sheriff_id
    dd[msg.sheriff_id].append(msg)  
	
    	
    
    
lc.subscribe('PMD_PRINTF', handler)
while True:
    try:
        lc.handle()
    except IOError:
        break
        
for key, value in dd.iteritems() :
    filename = '{}.txt'.format(key)
    f = open(filename, 'w')
    for msg in value:
        try:
	    f.write('{} {}\n'.format(msg.utime, msg.text))
        except:
            print 'bad data'
    f.close()
        
        
    


