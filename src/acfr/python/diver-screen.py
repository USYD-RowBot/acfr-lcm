#!/usr/bin/python

import thread
import time
import os
import subprocess
import serial


import lcm
import sys

LCMROOT='/home/auv/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import os_power_system_t, os_compass_t

# TRY:
# sudo setfont /usr/share/consolefonts/Lat15-Terminus24x12.psf.gz

# Global dictionary containing data to display on screen
DATADICT = {}
STATES = {"STAT":0 , "TRIG":1 , "SHUTDOWN":2}
CURENTSTATE = STATES["STAT"]
SHUTDOWNDELAY = 10 #s
DATADICT['*CAMS'] = False

# Define a function for the thread
def get_imagecount( threadName, delay):
    global DATADICT
    #imgdir = subprocess.check_output("ls -trd /media/data/r*/i* | tail -n 1",shell=True).rstrip()
    while True:
        DATADICT['.IMDIR'] = subprocess.check_output("ls -trd /media/data/r*/i* | tail -n 1",shell=True).rstrip()
        DATADICT['#IMGS'] = subprocess.check_output("ls {} | wc -l".format(DATADICT['.IMDIR']), shell=True).rstrip()
        DATADICT['.IMG'] = subprocess.check_output("ls -tr {} | tail -n 1".format(DATADICT['.IMDIR']), shell=True).rstrip()
        time.sleep(delay)


def get_lcmbattinfo (threadName, delay):
    lc = lcm.LCM()
    def lcmbatt_handler(channel, data):
        global DATADICT
        msg = os_power_system_t.decode(data)
        charge=str(msg.avgChargeP)
        if str(msg.minutesTE) == "9999" :
            time="CHRGING"
        else :
            time=str(msg.minutesTE)+" mins"
        DATADICT['BATT'] = "{}%%, {}".format(charge, time)
    
    subscription = lc.subscribe("BATTERY", lcmbatt_handler)
    while True:
        lc.handle()
        time.sleep(delay)

def get_sysinfo (threadName, delay):
    global DATADICT
    #TMPCMD = "sensors coretemp-isa-* | grep Core* | awk -F ' ' '{print $2 $3}' | tr '\n' ' '"
    HDDCMD = "df -h | grep /dev/sda6 | awk -F ' ' '{print $3\"/\"$4,\"(\"$5\")\"}'"
    while True:
        t1 = int(open('/sys/class/hwmon/hwmon0/device/temp2_input','r').read())/1000
        t2 = int(open('/sys/class/hwmon/hwmon0/device/temp3_input','r').read())/1000
        DATADICT['CPU'] = "{}\xb0C, {}\xb0C".format(t1,t2)  #\xc2\xb0C
        DATADICT['HDD'] = subprocess.check_output(HDDCMD, shell=True).rstrip()
        time.sleep(delay)

def check_reedswitch(threadName, readdelay, debouncedelay) :
    global CURENTSTATE
    ser = serial.Serial(port='/dev/ttyUSB0')
    triggered = False
    count = 0
    maxcount = int(SHUTDOWNDELAY/debouncedelay)
    while True :
        curtrigger = not ser.getCTS()
        if triggered == True and curtrigger == True and count >= maxcount :
            CURENTSTATE = STATES["SHUTDOWN"]
            count = 0
            time.sleep(debouncedelay)
        elif triggered == True and curtrigger == True :
            count += 1
            time.sleep(debouncedelay)
        elif curtrigger == False and triggered == True :
            CURENTSTATE = STATES["TRIG"]
            count = 0
            time.sleep(debouncedelay)
        else :
            CURENTSTATE = STATES["STAT"]
            count = 0
            time.sleep(readdelay)
        triggered = curtrigger
        DATADICT['CTL'] = "{}/{}".format(count, maxcount)

def update_display(threadName, delay):
    while True:
        os.system('clear')
        if CURENTSTATE == STATES["SHUTDOWN"]:
            print "****************************\n***** SHUTTING DOWN !! *****\n****************************"
            os.system('bash {}/scripts/stop_mission_diver.sh'.format(LCMROOT))
            os.system('bash {}/scripts/stop_lcm.sh'.format(LCMROOT))
            os.system('sudo /sbin/halt')
            time.sleep(10)
        elif CURENTSTATE == STATES["TRIG"]:
            print "****************************\n***** TRIGGER DETECTED *****\n****************************"
            if not DATADICT['*CAMS']: 
                print "\n\n    STARTING cameras..."
                os.system('bash {}/scripts/start_mission_diver.sh'.format(LCMROOT))
                DATADICT['*CAMS'] = True
            else :
                print "\n\n    STOPPING cameras..."
                os.system('bash {}/scripts/stop_mission_diver.sh'.format(LCMROOT))
                DATADICT['*CAMS'] = False
            time.sleep(2)
        elif CURENTSTATE == STATES["STAT"]:
            dispstr = ""
            for key in sorted(DATADICT):
                if key[0] != ".":
                    dispstr += "{}:\t{}\n".format(key,DATADICT[key])
            #if '.IMG' in DATADICT.keys():
            #    os.system('convert {0}/{1} -size 800x -pointsize 30 -gravity NorthWest -stroke black -strokewidth 6 -annotate 0 "{2}" -stroke  none -fill white -annotate 0 "{2}"  result.jpg'.format(DATADICT['.IMDIR'],DATADICT['.IMG'], dispstr))
            #    os.system('sudo fbi -noverbose result.jpg')

            print dispstr
            time.sleep(delay)


# Create two threads as follows
try:
    thread.start_new_thread(check_reedswitch, ("thread-ctl", 0.1, 1))
    thread.start_new_thread( get_imagecount, ("thread-img", 2) )
    thread.start_new_thread( get_sysinfo, ("thread-sys", 0.5) )
    #thread.start_new_thread( get_lcmbattinfo, ("thread-bat", 5) )
    thread.start_new_thread( update_display, ("thread-disp", 0.1) )
except:
    print "Error: unable to start thread"

while 1:
    time.sleep(60)
    pass
   
   
