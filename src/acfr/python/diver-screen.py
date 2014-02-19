#!/usr/bin/python

import thread
import time
import os
import subprocess
import serial
import math


import lcm
import sys

LCMROOT='/home/auv/git/acfr_lcm'

sys.path.append('{}/build/lib/python2.7/dist-packages/perls/lcmtypes'.format(LCMROOT))
from senlcm import os_power_system_t, os_compass_t

# TRY:
# sudo setfont /usr/share/consolefonts/Lat15-Terminus24x12.psf.gz

# Global dictionary containing data to display on screen
SYSDATA = {}
IMGDATA = {}
NAVDATA = {}
HIDDATA = {}

STATES = {"STAT":0 , "TRIG":1 , "SHUTDOWN":2}
CURENTSTATE = STATES["STAT"]
SHUTDOWNDELAY = 10 #s
IMGDATA['CAMS'] = False
VERBOSE = False

# Define a function for the thread
def get_imagecount( threadName, delay):
    global HIDDATA, IMGDATA
    #imgdir = subprocess.check_output("ls -trd /media/data/r*/i* | tail -n 1",shell=True).rstrip()
    while True:
        HIDDATA['IMDIR'] = subprocess.check_output("ls -trd /media/data/r*/i* | tail -n 1",shell=True).rstrip()
        IMGDATA['#IMS'] = subprocess.check_output("ls {} | wc -l".format(HIDDATA['IMDIR']), shell=True).rstrip()
        IMGDATA['LIMG'] = subprocess.check_output("ls -tr {} | tail -n 1".format(HIDDATA['IMDIR']), shell=True).rstrip()
        time.sleep(delay)


def get_lcmcompassinfo (threadName, delay):
    global NAVDATA
    def lcmcompass_handler(channel, data):
        msg = os_compass_t.decode(data)
        rph = [i * 180/math.pi for i in msg.rph] # convert to degrees
        NAVDATA['ROLL'] = '{:0.1f}'.format(rph[0])
        NAVDATA['PTCH'] = '{:0.1f}'.format(rph[1])
        NAVDATA['HDNG'] = '{:0.1f}'.format(rph[2])
        NAVDATA['DEPT'] = '{:0.3f}'.format(msg.depth)
    lc = lcm.LCM()
    subscription = lc.subscribe("OS_COMPASS", lcmcompass_handler)
    while True:
        lc.handle()


def get_lcmbattinfo (threadName, delay):
    global SYSDATA
    def lcmbatt_handler(channel, data):
        msg = os_power_system_t.decode(data)
        charge=str(msg.avg_charge_p)
        if int(msg.current) > 0 :
            mins="CHARGING: "+str(msg.minutes_tef)+" mins"
        elif msg.minutes_tef == -1:
            mins="FULLY CHARGED"
        else :
            mins=str(msg.minutes_tef)+" mins"
        SYSDATA['BATT'] = "{}%, {}".format(charge, mins)

    lc = lcm.LCM()
    subscription = lc.subscribe("BATTERY", lcmbatt_handler)
    while True:
        #DATADICT['BATT']="----"
        lc.handle()
        #time.sleep(delay)

def get_sysinfo (threadName, delay):
    global SYSDATA
    #TMPCMD = "sensors coretemp-isa-* | grep Core* | awk -F ' ' '{print $2 $3}' | tr '\n' ' '"
    HDDCMD = "df -h | grep /dev/sda6 | awk -F ' ' '{print $3\"/\"$4,\"(\"$5\")\"}'"
    while True:
        t1 = int(open('/sys/class/hwmon/hwmon0/device/temp2_input','r').read())/1000
        t2 = int(open('/sys/class/hwmon/hwmon0/device/temp3_input','r').read())/1000
        SYSDATA['CPU'] = "{}\xb0C, {}\xb0C".format(t1,t2)  #\xc2\xb0C
        SYSDATA['HDD'] = subprocess.check_output(HDDCMD, shell=True).rstrip()
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
        IMGDATA['CTL'] = "{}/{}".format(count, maxcount)

def update_display(threadName, delay):
    while True:
        os.system('clear')
        if CURENTSTATE == STATES["SHUTDOWN"]:
            os.system('setterm -foreground white -background red')
            print "****************************\n***** SHUTTING DOWN !! *****\n****************************"
            os.system('bash {}/scripts/stop_mission_diver.sh'.format(LCMROOT))
            os.system('bash {}/scripts/stop_lcm.sh'.format(LCMROOT))
            os.system('sudo /sbin/halt')
            time.sleep(10)
        elif CURENTSTATE == STATES["TRIG"]:
            print "****************************\n***** TRIGGER DETECTED *****\n****************************"
            if not IMGDATA['CAMS']: 
                os.system('setterm -foreground green -background black')
                print "\n\n    STARTING cameras...\n\n"
                os.system('bash {}/scripts/start_mission_diver.sh'.format(LCMROOT))
                IMGDATA['CAMS'] = True
            else :
                os.system('setterm -foreground white -background black')
                print "\n\n    STOPPING cameras...\n\n"
                os.system('bash {}/scripts/stop_mission_diver.sh'.format(LCMROOT))
                IMGDATA['CAMS'] = False
            time.sleep(2)
        elif CURENTSTATE == STATES["STAT"]:
            imgstr = ""
            dispstr = ""
            dispstr += "IMAGE DATA *************************\n\n"
            for key in sorted(IMGDATA):
                dispstr += "  {}:\t{}\n".format(key,IMGDATA[key])
            #    imgstr += "  {}:\t{}\n".format(key,IMGDATA[key])
            #if IMGDATA['CAMS'] :
            #    dispstr += '\033[92m{}\033[0m'.format(imgstr)
            #else :
            #    dispstr += '\033[93m{}\033[0m'.format(imgstr)
            dispstr += "\n\nNAVIGATION DATA ********************\n\n"
            for key in sorted(NAVDATA):
                dispstr += "  {}:\t{}\n".format(key,NAVDATA[key])
            dispstr += "\n\nSYSTEM DATA ************************\n\n"
            for key in sorted(SYSDATA):
                dispstr += "  {}:\t{}\n".format(key,SYSDATA[key])
                
            if VERBOSE :
                dispstr += "\nVERBOSE DATA ***********************\n"
                for key in sorted(HIDDATA):
                    dispstr += "  {}:\t{}\n".format(key,HIDDATA[key])
            #if '.IMG' in DATADICT.keys():
            #    os.system('convert {0}/{1} -size 800x -pointsize 30 -gravity NorthWest -stroke black -strokewidth 6 -annotate 0 "{2}" -stroke  none -fill white -annotate 0 "{2}"  result.jpg'.format(DATADICT['.IMDIR'],DATADICT['.IMG'], dispstr))
            #    os.system('sudo fbi -noverbose result.jpg')

            print dispstr
            time.sleep(delay)


# Create two threads as follows
try:
    thread.start_new_thread(check_reedswitch, ("thread-ctl", 0.05, 1))
    thread.start_new_thread( get_imagecount, ("thread-img", 1) )
    thread.start_new_thread( get_sysinfo, ("thread-sys", 2) )
    thread.start_new_thread( get_lcmbattinfo, ("thread-bat", 5) )
    thread.start_new_thread( get_lcmcompassinfo, ('thread-cmp', 5))
    thread.start_new_thread( update_display, ("thread-disp", 0.5) )
except:
    print "Error: unable to start thread"

while 1:
    time.sleep(60)
    pass
   
   
