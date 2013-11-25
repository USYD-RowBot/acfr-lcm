#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Acquire for X mins using speclib
method flags: '-n' use network serial server, '-h' use hardware serial
3rd arguement gives a label on the log files
    
    
running on my mac #!/opt/local/bin/python
"""

import sys
import speclib_3 as sp
import datetime
import time

# Input argument
if len(sys.argv) != 5 or sys.argv[1] == '--help':
#print out help and exit
    print '------------------------------------------------------------'
    print '\tPython Based Ocean Optics STS Spectrometer logger'
    print '------------------------------------------------------------'
    print '\nUsage: ./specAcq.py METHOD LABEL NUMBERFILE CHECKRATE'
    print '\n\tMETHOD - This can be either -h for hardware serial or -n for network serial'
    print '\tLABEL - a label which will go in the header of the log file'
    print '\tNUMBERFILE - number of samples per log file'
    print '\tCHECKRATE - number of samples it will acquire before re-checking the integration time'
    print '\n\tExample: ./specAcq.py -h \'A label for your file\' 100 10'
    print '\nLog files are stored by date and time in ~/Logs/SpectraLogs/'
    print 'Acquisition will continue until a \'y\' is written to ~/exitFile'
    print 'This can be done with:\n\techo \'y\' > ~/exitFile'
    print 'This program is made to work on the original beaglebone, but could be modified to run on the beaglebone black, just need some changes to how it accesses the serial ports'
    print '\n(C) Daniel Bongiorno, 2013\n'
    print '------------------------------------------------------------'
    sys.exit(0)


method = sys.argv[1]
label = sys.argv[2]
numPerFile = int(sys.argv[3])

thresholds = [16000, 1490, 6000, 250,400]

checkRate = int(sys.argv[4])

if method == '-n':
    sp.initSpec('portserverts4', 2101)
    
elif method == '-h':
    #	sp.initPortBeagleBone()
#    sp.initSpec(115200, '/dev/tty.PL2303-00001014')
    sp.initSpec(115200, '/dev/tps5')
#	sp.initSpec(115200, '/dev/ttyO5')
    sp.ser.timeout = 10
#    sp.initSpec(115200, '/dev/ttyUSB0')

# Write the exit file - we do not want to exit straight away, we have to wait until the 
# stopSpecAcq function is called.
open("exitFile","w").write("n")
starttime = datetime.datetime.now()


s = sp.getSpectra()
oldIntTime = int(sp.specData['intTime'])
newIntTime = sp.checkIntTime(s,thresholds)

sp.setIntTime(newIntTime)

time.sleep(0.01)
fnm = sp.logSpectra(label)

# loop for a fixed ammount of time
# every 100 samples acquire temperature
delta = 0

exitting = open("exitFile","r").read(1)
nn = 0

while (exitting == 'n'):
    for i in range (checkRate):
        sp.getSpectra()
        time.sleep(0.5)
        sp.appendSpectraToLog(fnm)
        nn += 1
        print 'Spec %u: %s, Elap:%.3f min, Mean:%.1f, Max:%u, IntT:%ums, BTemp:%.1f, DTemp:%.1f' % (nn, sp.specData['spectraTimeStamp'].isoformat(),delta, sp.specData['latestSpecMean'], sp.specData['latestSpecMax'], int(sp.specData['intTime'])/1000, sp.specData['boardTemp'], sp.specData['detectTemp'])

       
        now = datetime.datetime.now()
        deltaT = (now - starttime)
        delta = deltaT.seconds / 60.0  
              
	exitting = open("exitFile","r").read(1)
# 		Read exit file to see if it is time to quite


        if exitting == 'y':
            break
    if exitting == 'y':
        break
    
    # test to see if intTime is optimal
    sp.getTemp()
    oldIntTime = int(sp.specData['intTime'])
    newIntTime = sp.checkIntTime(sp.specData['latestSpectra'], thresholds)
    if abs(newIntTime - oldIntTime) > 10000:
        sp.setIntTime(newIntTime)
    else:
        newIntTime = oldIntTime
    
   
    if nn > numPerFile:
        sp.getTemp()
        sp.getSpectra()
        fnm = sp.logSpectra(label)
        nn = 0

sp.ser.close()
print 'Finished Acquisition'
