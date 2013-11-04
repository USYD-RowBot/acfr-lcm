#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
    Acquire for X mins using speclib
    method flags: '-n' use network serial server, '-h' use hardware serial
    3rd arguement gives a label on the log files
    """

import sys
import speclib_3 as sp
import datetime
import time
import lcdlib as lcd

# Input argument
if len(sys.argv) != 4 or sys.argv[1] == '--help':
    #print out help and exit
    print '\tPython Based Beagle-Bone STS spectrometer logger - Dark Spectra'
    print '\nUsage: ./specAcq.py METHOD LABEL INTTIME'
    print '\n\tMETHOD - This can be either -h for hardware serial or -n for network serial'
    print '\tLABEL - a label which will go in the header of the log file'
    print '\tINTTIME - the integration time in uSec you want to set it too'

    print '\n\tExample: ./specAcq.py -h \'A label for your file\' 100000'
    print '\nLog files are stored by date and time in ~/Logs/SpectraLogs/'
    print 'Acquisition will continue and collect 10 dark spectra'
    print 'This program is made to work on the original beaglebone, but could be modified to run on the beaglebone black, just need some changes to how it accesses the serial ports'
    print '\n(C) Daniel Bongiorno, 2013\n'
    sys.exit(0)


method = sys.argv[1]
label = sys.argv[2]
intTime = int(sys.argv[3])

if method == '-n':
    sp.initSpec('portserverts4', 2101)

elif method == '-h':
	sp.initPortBeagleBone()
    #     sp.initSpec(115200, '/dev/tty.PL2303-00001014')
	sp.initSpec(115200, '/dev/ttyO5')
        sp.ser.timeout = 3
#    sp.initSpec(115200, '/dev/ttyUSB0')

# Write the exit file - we do not want to exit straight away, we have to wait until the
# stopSpecAcq function is called.
open("exitFile","w").write("n")


lcd.initializeDisplay(2,16)
lcd.clearScreen()


starttime = datetime.datetime.now()


lcd.setCursorPos(1,1,True)
lcd.writeToDisplay('Cover Spectrometer')
sp.getSpectra()
s = sp.getDarkSpec()

lcd.setCursorPos(2,1,True)
lcd.writeToDisplay('Acquiring Dark')

sp.setIntTime(intTime)

time.sleep(0.5)
fnm = sp.logSpectra(label)

delta = 0
    
for i in range (10):
    sp.getSpectra()
    time.sleep(0.5)
    sp.appendSpectraToLog(fnm)
    print 'Spectra %u: %s, Elapsed:%.3f min, Mean:%.1f, Max:%u, IntT:%ums' % (i, sp.specData['spectraTimeStamp'].isoformat(),delta, sp.specData['latestSpecMean'], sp.specData['latestSpecMax'], int(sp.specData['intTime'])/1000 )
    
    lcd.setCursorPos(1,1,True)
    lcd.writeToDisplay('S:%u Mx:%u ' % (i, sp.specData['latestSpecMax']))
    lcd.setCursorPos(2,1,True)
    lcd.writeToDisplay('M:%.0f I:%u   ' % (sp.specData['latestSpecMean'], int(sp.specData['intTime'])/1000))
    
    now = datetime.datetime.now()
    deltaT = (now - starttime)
    delta = deltaT.seconds / 60.0


sp.ser.close()
print 'Finished Acquisition'
