# -*- coding: utf-8 -*-
"""
STS Micro Library
This is a Python Library for using the Ocean Optics STS Microspectrometer over
a RS232 serial connection. This has be tested and built to work with Python 2.7.
First start by importing the library:
    import speclib
Using the import statement above each function call will need to be preceeded with
speclib.Function Call

@author: Daniel Bongiorno
"""

import math
import array
import serial
import socket
import datetime
import time
import struct
import os
from collections import namedtuple
#import matplotlib.pyplot as plt
#from construct import *
#import construct

ser = None


DEFAULTPORT = '/dev/ttyS0'
SERIALTIMEOUT = 10
#HOST = '127.0.0.1'
#PORT = 2101

specData = {'initTime':datetime.datetime.now(), 'intTime':100000, 'boxcar':0, 'averaging': 1,'baudRate':9600 }

def initSpec(baudRate, port):
    '''This will initialise the serial comms with the spectrometer and will 
    find what baud the spectrometer is currently set to and change it to the 
    desired baud rate. This will also acquire all the calibration coefficients,
    set averaging and boxcar, check temperature and acquire the serial number. 
    Possible values for baudrate are: 9600, 19200, 38400, 57600, 115200.
    
    This will now work for using a TCP serial port server, instead of baudrate
    you specify an IP or URL to the serial port server'''
    
#    sleepTime = 0.1
    if isinstance(baudRate, int):
#    Initialise Serial Port at Baud rate
        initSerial(baudRate, port)
#    time.sleep(sleepTime)
    elif isinstance(baudRate, str):
        initNetSerial(baudRate, port)
    
#    time.sleep(sleepTime)    
    
#    set Box Car Value
    setBoxCar(specData['boxcar'])
#    time.sleep(sleepTime)
    
#    Set Averaging
    setAveraging(specData['averaging'])
#    time.sleep(sleepTime)
    
#    Set Integration time
    setIntTime(specData['intTime'])
#    time.sleep(sleepTime)

    #set Trigger mode to default 0    
    setTriggerMode(0)    
    
#   Get nonlinear coeff
    getNonlinCoeff()
#    time.sleep(sleepTime)
    
#    Get Wavelength Coeff
    getWavelengthCoeff()
    makeWavelengths()
#    time.sleep(sleepTime)
    
    # Get Temperature
    getTemp()
#    time.sleep(sleepTime)
    
#    Get serial Number
    getSerialNum()
#    time.sleep(sleepTime)
    
#--------UNDER THE HOOD---------------
def initPortBeagleBone():
# this sets up the mux settings for using Serial port 5 on a BeagleBone
    RECEIVE_ENABLE = 32
    MUX_MODE = 4
    RX_MUX = 'lcd_data9'
    TX_MUX = 'lcd_data8'
	# set the RX pin for Mode 0 with receive bit on
    # - use %X formatter, since value written must be in hex (e.g. write "21" for mode 1 with receive enabled = 33)
    open('/sys/kernel/debug/omap_mux/' + RX_MUX, 'wb').write("%X" % (RECEIVE_ENABLE + MUX_MODE))
    # set the TX pin for Mode 0
    open('/sys/kernel/debug/omap_mux/' + TX_MUX, 'wb').write("%X" % MUX_MODE)
	# for port 5 should write 0x24 to lcd_data9 (Pin 38 - P8) and 0x04 to lcd_data8 (Pin 37 - P8)


def initSerial(baudRate, port):
    '''When the spectrometer is first turned on it will wake up at 9600 BAUD,
    then it will need to be set to the desired baud, which it will maintain 
    until power is removed from the device. So the program will have to see
    if it respondes at 9600 first, if not run through the options until it
    does and then change if necessary to the desired Baud rate.'''
    
#    Initialise Serial at baudrate
    global ser
#   Just set baud to 9600 and tell the spectrometer to set to 115200
    currentBaud = 9600

    #currentBaud = findCurrentBaud(port)
    #print 'Found baud to be %u' % currentBaud
#    Set serial to found baud
    if port == 0:
        ser = serial.Serial(DEFAULTPORT, currentBaud)
    else:
        ser = serial.Serial(port, currentBaud)
    ser.timeout = SERIALTIMEOUT
    ser.writeTimeout = SERIALTIMEOUT
    #time.sleep(1) # to allow for the creation of the serial port, could be reduced    
    
    writePack('setBaud', [baudRate])
    time.sleep(1)
    if port == 0:
        ser = serial.Serial(DEFAULTPORT, baudRate)
    else:
        ser = serial.Serial(port, baudRate)
    b = writeReadPack('getBaud', [0], 64, False)
    if len(b) > 0:    
        p = processPacket(b,'baud')
        if p.imData[0] == baudRate:
            print 'Baud set to %u' % (baudRate)
            specData['baudRate'] = baudRate
        else:
            print 'Error setting baud to %u' % (baudRate)
 
def initNetSerial(addr, port):
    '''This will assume the spectrometer started from the powered off state and
    so the baud is 9600, so it will start talking at 9600 and try switching the 
    spectrometer and the netSerial port server to 115200, the default TCP port for
    serial port 1 is 2101 (settings for using a Digi Serial Port Server TS4)
    '''
    global ser

    setNetSerialBaud(addr,1,9600)
    #connect using sockets to the serial port server, usually on 2101
    ser = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ser.settimeout(SERIALTIMEOUT)
    ser.connect((addr,port))
    writePack('setBaud', [115200])
    setNetSerialBaud(addr,1,115200)

    print 'Spectrometer and NetSerial set to 115200'
  
def findCurrentBaud(port):
    global ser
    outBaud = 0
    baudRange = [9600, 57600, 115200, 19200, 38400]
    for baud in baudRange:
        if port == 0:
            ser = serial.Serial(DEFAULTPORT, baud)
        else:
            ser = serial.Serial(port, baud)
        ser.timeout = 0.5
        specData['baudRate'] = baud
        time.sleep(0.4)
        b = writeReadPack('getBaud', [0], 64, False)

        if len(b) > 0:
            p = processPacket(b,'baud')
            if p.imData[0] == baud:
                outBaud = baud
                break
        
    return outBaud
    
def findCurrentNetBaud(addr,port):
    '''This is not currently working but is intended to search through the baud
    rates and find what the spectrometer is set to'''
    
    global ser
    serialPortNum = 1
    outBaud = 0
    baudRange = [9600, 57600, 115200, 19200, 38400]
    for baud in baudRange:
        try:
            setNetSerialBaud(addr,serialPortNum,baud)
            ser = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ser.settimeout(SERIALTIMEOUT)
            ser.connect((addr,port))
            
            b = writeReadPack('getBaud', [0], 64, False)
            ser.close()
            if len(b) > 0:
                p = processPacket(b,'baud')
                if p.imData[0] == baud:
                    outBaud = baud
                    break
        except:
            print 'error in finding baud'
        
    return outBaud
    
def setNetSerialBaud(addr,serialPortNum,baud):
    USERNAME = 'root\n'
    PASSWORD = 'dbps\n'    
    
    
    ps = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect to port server on telnet port 23 to change the baud rate    
    ps.connect((addr,23))
    ps.sendall(USERNAME)
    time.sleep(0.2)
    ps.sendall(PASSWORD)
    time.sleep(0.5)
#    print 'set line range=%u baud=%u' % (serialPortNum, baud)
    ps.sendall('set line range=%u baud=%u\n\n' % (serialPortNum, baud))
    time.sleep(0.2)
#    r = ps.recv(50)
#    print r
    ps.close()
        
#-----SET-----------------
def setAveraging(avgValue):
    '''This can be as large as 5000, spectrometer won’t return a spectra until all have been
sampled, ie it isn’t a running average. It is also written to specData[’averaging’].
'''
    global specData
    writePack('setAveraging', [avgValue])
    
    b = writeReadPack('getAveraging', [0], 64, False)
    if len(b) > 0:
        proc = processPacket(b,'averaging')    
         
        if proc.imData[0] == avgValue:
            print 'Averaging set to %u' % avgValue
            specData['averaging'] = avgValue
        else:
            print 'Problem setting Averaging'
            print b
    else:
        printError(1)
       
def setBoxCar(boxcarValue):
    '''This will smooth out spectra by averaging neighbouring values. The boxcarValue
defines the width of the averaging (this needs to be a odd number). It is also written
to specData[’boxcar’].'''

    global specData
    writePack('setBoxcar', [boxcarValue])

    b = writeReadPack('getBoxcar', [0], 64, False)
    if len(b) > 0:
        proc = processPacket(b,'regular')    
    
        if proc.imData[0:proc.imDataLen][0] == boxcarValue:
            specData['boxcar'] = boxcarValue
            print 'Boxcar set to %u' % boxcarValue
        else:
            print 'Problem setting Boxcar'   
    else:
        printError(1)
        
def setIntTime(intTime):
    '''This is time in μsec, it is also written to specData[’intTime’].'''
    global specData
    writePack('setIntTime', [intTime])
    print 'Integration time set to %umicroSec' % (intTime)
    specData['intTime'] = intTime

def setTriggerMode(mode):
    '''this will set the trigger mode on the spectrometer, mode 0 is default, 
    and integration begins as soon as possible after request. mode 1 integration 
    or trigger delay starts after a rising trigger edge. mode 2 integration
    is internally triggered to synchronise with continuous strobe'''
    global specData    
    writePack('setTriggerMode', [mode])
    print 'Trigger mode set to %u' % (mode)
    specData['triggerMode'] = mode

def simTriggerPulse():
    '''This will cause the spectrometer to react exactly as though an electrical 
    rising edge signal was applied to the external trigger pin'''
    
    writePack('simTriggerPulse',[])
    print 'Trigger Pulse sent'

def setOutputEn(outputPins):
    '''Output pins in a 2 element list where the first element is a decimal
    representation of a 32bit vector and the second element is a 32bit mask.
    So element who are set to 1 in the vector are set as outputs and 0 for 
    inputs but they will only change if there is a 1 on the mask as well.
    Example: setOutputEn([1, 1]) will set GPIO 1 as an output.'''
    
    writePack('setOutputEn', outputPins)
    print 'GPIO pins set to [%u,%u]' % (outputPins[0], outputPins[1])
          
def setGPIOstate(outputState):
    '''Similar method to setOutputEn with a vector in the first element and 
    a mask in second element.
    Example: setGPIO([1, 1]) will set GPIO-1 high and setGPIO([0, 1]) will
    set GPIO-1 low.'''
    writePack('setGPIO', outputState)
    print 'GPIO pins toggled to [%u,%u]' % (outputState[0], outputState[1])


#-----GET-----------------
def getNspectra(numSpec, label):
    '''Will acquire N spectra and log them to a log file with label 'label'.
    It would be best to acquire a dark spectra before running this function.
'''    
    getTemp()
    getSpectra()
    fnm = logSpectra(label)
    for i in range(numSpec):
        getSpectra()
        appendSpectraToLog(fnm)
        print 'Spectra %u acquired: %s' % (i, specData['spectraTimeStamp'].isoformat())
        
def getSpectraAutoIntTime(maxIntTime, threshold, numAboveThres, numAttempts, increment):
    '''This will aim to maximise the signal to noise by modifying the int time
    until it finds an optimal int time. Input is the maximum integration time
    you want and the function returns the optimal integration time in uSeconds.
    If the integration time is at a point where the signal is already good, 
    it won't try and improve on it. It will look to see if at least 'numAboveThres'
    pts are above an intensity of 'Threshold'. For the case of clipping, if 
    the signal is above 16000 then it will reduce the intTime to stop clipping. For
    each try it will increase by increment'''
    for attemp in range(numAttempts):
        #just try finding a new value for numAttempts attempts
        spec = getSpectra()
        aboveThresCount = 0
        clipped = False
        for wave in spec:
            if wave > threshold:
                aboveThresCount = aboveThresCount + 1
            if wave > 16000:
                clipped = True
                
        if aboveThresCount >= numAboveThres and not clipped:
            #Success
            print 'IntTime of %uuSec is found to be the best' % specData['intTime']
            break
        else:
            #the Integration time is too low try a larger value.
            curInt = specData['intTime']
            if (curInt + increment) > maxIntTime:
                print 'Hit max Int Time: Int time set to %uuSec' % curInt   
                break
            else:
                print 'Current IntTime: %uuSec is too low' % curInt
                setIntTime(curInt + increment)
            
        if clipped:
            print 'Current IntTime: %uuSec is too high, lowering intTime' % curInt
            curInt = specData['intTime']
            setIntTime(curInt - increment)
            
    return specData['intTime']
  
def getAutoIntTime(powerDensityThres, numAttempts, increment):
    ''' Will find the int time that delivers above the power density threshold,
    which is the integral of the response for each frequency above 1500 (dark DC).
    Will only look between 400nm and 680nm, where for underwater use most of the 
    energy will be. So this will be 610 frequency bins.
    
    '''
    for attemp in range(numAttempts):
        spec = getSpectra()
        clipped = False
        power = 0
        curInt = specData['intTime']
        for wave in spec[140:750]:
            if wave > 1500:
                power = power + (wave-1500)
            if wave > 16000:
                clipped = True
         
        if power >= powerDensityThres:
            #Success
            print 'IntTime of %uuSec is found to be the best' % specData['intTime']
            break
        else:
            print 'Current IntTime: %uuSec is too low' % curInt
            setIntTime(curInt + increment)
               
        if clipped:
            print 'Current IntTime: %uuSec is too high, lowering intTime' % curInt
            curInt = specData['intTime']
            setIntTime(curInt - increment)
            
            
    return specData['intTime']

def checkIntTime(spectra, thresholds):
    ''' This routine searches through a spectra array and determines if it has maxed out or the signal is too small, it then does a calculation to determine a better int time.
        Thresholds: [0] = Max threshold, [1] = Min Threshold, [2] = desired mean, [3] = start index, [4] = width '''
    
    curInt = int(specData['intTime'])
    maxS = max(spectra)

    mean = sum(spectra[thresholds[3]:thresholds[3] + thresholds[4]]) / float(thresholds[4])

    gain = (thresholds[2]-thresholds[1]) / (mean - thresholds[1])
    # gain = (max - min) / (mean - min)
    # min is the dark current

    if maxS >= thresholds[0]:
        saturated = True
        gain *= 0.9
        print 'Signal has Saturated: value = %uu' % maxS

    if maxS <= thresholds[1]:
        tooLow = True
        print 'Signal is too low: value = %uu' % maxS

    if gain > 20.0:
        gain = 10.0

    newIntTime = (int(curInt * gain) /1000) * 1000

    if newIntTime < 1000:
        newIntTime = 1000

    if newIntTime > 10000000:
        newIntTime = 10000000

    print 'CurIntTime: %u, NewIntTime: %u, mean: %.1f, gain: %.1f, max: %u' % (curInt, newIntTime, mean, gain, maxS)

    return newIntTime


def getSpectra():
    '''returns a list object of the spectra and also writes it to specData[’latestSpectra’]
and writes a timestamp to specData[’spectraTimeStamp’].'''

    global specData
    b = writeReadPack('getCorSpec', [0], 2112, True)
    if len(b) > 0:
        proc = processPacket(b,'spectra')
        
        specData['latestSpectra'] = proc.payload
        specData['spectraTimeStamp'] = datetime.datetime.now()
        specData['latestSpecMean'] = average(proc.payload[250:650])
        specData['latestSpecMax'] = max(proc.payload)
#        print 'Spectra Acquired'
        return proc.payload
    else:
        printError(1)

def average(values):
    """Computes the arithmetic mean of a list of numbers.
    """
    return sum(values, 0.0) / float(len(values))

def getDarkSpec():
    '''Writes dark spectra to specData[’darkSpec’] and writes a timestamp to
specData[’darkSpecTimeStamp’]. Remember to cover up aperature before send-
ing command.'''
    specData['darkSpec'] = getSpectra()
    specData['darkSpecTimeStamp'] = specData['spectraTimeStamp']
    print 'Dark Spectra Acquired'

def getRefSpec():
    '''Writes reference spectra to specData[’refSpec’] and writes a timestamp to
specData[’refSpecTimeStamp’]. Use this for calculating reflectance.'''

    specData['refSpec'] = getSpectra()
    specData['refSpecTimeStamp'] = specData['spectraTimeStamp']
    print 'Reference Spectra Acquired'

def getNonlinCoeff():
    global specData
    b = writeReadPack('getNonlinCoeffCount', [0], 64, False)
    proc = processPacket(b,'regular')
    coeff = []
#    print proc.imData[0]
    for i in range(proc.imData[0]):
        outPack = writeReadPack('getNonlinCoeffN',[i],64, False)
        proc = processPacket(outPack,'LFloat32')
        coeff.append(proc.imData[0])
#        time.sleep(0.05)
#        print 'b'
        
    specData['nonlinCoeff'] = coeff
    print 'Nonlinear Coefficients Acquired'
    return coeff
    
def getWavelengthCoeff():
    global specData
    b = writeReadPack('getWaveCoeffCount', [0], 64, False)
    proc = processPacket(b,'regular')
    coeff = []
#    print proc.imData[0]
    for i in range(proc.imData[0]):
        outPack = writeReadPack('getWaveCoeffN',[i],64, False)
        proc = processPacket(outPack,'LFloat32')
        coeff.append(proc.imData[0])
#        time.sleep(0.1)
#        print 'w'
        
    specData['waveCoeff'] = coeff
    print 'Wavelength Coefficients Acquired'
    return coeff
    
def getTemp():
    '''This will acquire the temperatures of the detector and the board, these are written
to specData[’detectTemp’] and specData[’boardTemp’] respectively. There is
also a timestamp that is written to specData[’tempTimeStamp’].'''

    global specData
    b = writeReadPack('getAllTemp', [0], 64, False)
    proc = processPacket(b,'LFloat32')
    coeff = []
    coeff.append(proc.imData[0]) # Detector Temperature
    coeff.append(proc.imData[2]) # Board Temperature
        
    specData['detectTemp'] = coeff[0]
    specData['boardTemp'] = coeff[1]
    specData['tempTimeStamp'] = datetime.datetime.now()
    print 'Temp Acquired, Board: %.1fdeg, Detector: %.1fdeg' % (specData['boardTemp'], specData['detectTemp'])
    return coeff

def getCorSpectra():
    '''This will acquire a new spectra and provided you have previously acquired a dark
spectra it will return the nonlinearity corrected spectra, this is also written to
specData[’corSpectra’].'''
    print 'Remember you will need a dark Spectra'
    correctSpectra(getSpectra(), specData['nonlinCoeff'], specData['darkSpec'])
    return specData['corSpectra']

def getSerialNum():
    global specData
    b = writeReadPack('getSerialNum', [0], 64, False)
    proc = processPacket(b,'string')
    specData['serialNum'] = proc.imData
    print 'Serial Acquired: %s' % (specData['serialNum'])
    return specData['serialNum']
 

#--------UTILITIES----------

#def acquireForNsamples(N):
#    getSpectra()    
#    plotSpectra()
#    plt.show()
#    for i in range(N):
#        getSpectra()
#        plotSpectra()
#        plt.draw()

def calcRGBmeans():
    #140:750 index which is 400 - 680nm
    
    specData['redMean'] = average(specData['latestSpectra'][547:750])
    specData['greenMean'] = average(specData['latestSpectra'][344:547])
    specData['blueMean'] = average(specData['latestSpectra'][140:344])


def plotSpectra():
    '''This uses matplotlib and will plot the latest spectra from
specData[’latestSpectra’] as well as the dark spectral and the reference spectra
on different figures.'''

   # import matplotlib.pyplot as plt
#this allows the library to be copied to the beaglebone and not need the matplotlib

        #or enable interactive mode
    #plt.ion()

    plt.figure(1)
#    plt.subplot(311)
    plt.plot(specData['wavelengths'],specData['latestSpectra'],'r-')
    plt.xlabel('Wavelength (nm)')
    plt.ylabel('Intensity (counts)')
    plt.title('Raw Spectra - %s' % specData['spectraTimeStamp'].isoformat())
    
    if specData.has_key('darkSpec'):
        correctSpectra(specData['latestSpectra'], specData['nonlinCoeff'], specData['darkSpec'])
        plt.figure(2)
#        plt.subplot(312)
        plt.plot(specData['wavelengths'],specData['corSpectra'],'b-')
        plt.xlabel('Wavelength (nm)')
        plt.ylabel('Intensity (counts)')
        plt.title('Nonlinearity Corrected Spectra')

        
    if specData.has_key('refSpec'):
        ref = specData['refSpec']
        plt.figure(3)
#        plt.subplot(313)
        plt.plot(specData['wavelengths'],ref,'g-')
        plt.xlabel('Wavelength (nm)')
        plt.ylabel('Intensity (counts)')
        plt.title('Reference Spectra')
        
    plt.draw()

        

def logSpectra(label):
    '''This will open a new file in ./Logs/SpectraLogs/filename where the filename is
year month day hour min sec - SpectraLog.txt. The timestamp of the filename is
based on the time of the log’s creation. The file is tab-delimited and contains the
spectra data as well as all the settings and calibration coefficients. This function
will return the filename so you can add extra spectra to the log and it is also written
to specData[’logFnm’]. The input \texttt{label} will print the label string at the top 
of the log file, this can be useful for specifying depth, location or various 
other pieces of information relevant to the data.'''

    if label == None:
        label = 'No Label'
    
    timeNow = datetime.datetime.now()
    timezone = time.strftime('%z', time.gmtime())

#    Check for the existance of the log file directory

    logDir = os.path.expanduser('~/spectralLogs/')
    if os.path.exists(logDir) == False:
        os.mkdir(logDir)

    fnm = '%s/%s-SpectraLog.txt' % (logDir,timeNow.strftime('%Y%m%d-%H%M%S'))
    fid = open(fnm, 'w')
    fid.write('RS232 STS MicroSpectrometer Spectra Log - Daniel Bongiorno, 2013\n')
    fid.write('Date:\t%s\tTime:\t%s %s\n' % (timeNow.strftime('%d/%m/%Y'),timeNow.strftime('%H:%M:%S'), timezone ))
    fid.write('Label: \t%s\n' % label)
    fid.write('Serial Number:\t%s\n' % (specData['serialNum']))  
    fid.write('Initial Integration Time: (microsec)\t%u\n' % (specData['intTime']))
    fid.write('Averaging:\t%u\tBoxcar:\t%u\n' % (specData['averaging'], specData['boxcar']))
    fid.write('Calibration Coefficients\tIntercept\t1st Coeff\t2nd Coeff\t3rd Coeff\t4th Coeff\t5th Coeff\t6th Coeff\t7th Coeff\n')
    coeff = specData['waveCoeff']    
    fid.write('Wavelength Calibration:\t%.15f\t%.15f\t%.15f\t%.15f\n' % (coeff[0],coeff[1],coeff[2],coeff[3]))
    coeff = specData['nonlinCoeff'] 
    fid.write('Nonlinear Calibration:\t%.25f\t%.25f\t%.25f\t%.25f\t%.25f\t%.25f\t%.25f\t%.25f\n' % (coeff[0],coeff[1],coeff[2],coeff[3],coeff[4],coeff[5],coeff[6],coeff[7])  )  
    fid.write('Board Temp:\t%f\tDetector Temp:\t%f\tTimestamp:\t%s\n' % (specData['boardTemp'], specData['detectTemp'], specData['tempTimeStamp'].strftime('%d/%m/%Y\t%H:%M:%S') ) )
    
    wave = specData['wavelengths']
    fid.write('\nType\tTimeStamp\t\tInt Time (microsec)\tWavelength\n\tDate:\tTime:\n\t\t\t\t')
    for band in wave:
        fid.write('%.4f\t' % band)
        
    fid.write('\n')
    
#    Test to see if there is a dark Spectra if yes paste dark
    if specData.has_key('darkSpec'):
        dark = specData['darkSpec']
        fid.write('Dark\t%s\t%s\t%u\t' % (specData['darkSpecTimeStamp'].strftime('%d/%m/%Y'),specData['darkSpecTimeStamp'].strftime('%H:%M:%S.%f'), specData['intTime']))
        for band in dark:
            fid.write('%.0f\t' % band)
        fid.write('\n')
            
    if specData.has_key('refSpec'):
        ref = specData['refSpec']
        fid.write('Reference\t%s\t%s\t%u\t' % (specData['refSpecTimeStamp'].strftime('%d/%m/%Y'),specData['refSpecTimeStamp'].strftime('%H:%M:%S.%f'), specData['intTime']))
        for band in ref:
            fid.write('%.0f\t' % band)
        fid.write('\n')
            
    
#    Now write spectral data
    if specData.has_key('latestSpectra'):
        spec = specData['latestSpectra']
        fid.write('Spectra\t%s\t%s\t%u\t' % (specData['spectraTimeStamp'].strftime('%d/%m/%Y'),specData['spectraTimeStamp'].strftime('%H:%M:%S.%f'), specData['intTime']))
        for band in spec:
            fid.write('%.0f\t' % band)
        fid.write('\n')

#close the file
    fid.close()
    print 'Logged Spectra written to %s' % fnm
    
    specData['logFnm'] = fnm
    return fnm

def appendSpectraToLog(fileName):
    '''This will add a new spectra to the log based on the inputed filename, it will pull
the new spectra from specData[’latestSpectra’] so please ensure that you call
getSpectra() before calling this function.'''

    fid = open(fileName, 'a')
	#add in next entry
 #    Now write spectral data
    if specData.has_key('latestSpectra'):
        spec = specData['latestSpectra']
        fid.write('Spectra\t%s\t%s\t%u\t' % (specData['spectraTimeStamp'].strftime('%d/%m/%Y'),specData['spectraTimeStamp'].strftime('%H:%M:%S.%f'), specData['intTime']))
        for band in spec:
            fid.write('%.0f\t' % band)
        fid.write('\n')
        
    fid.close()
    
def recordDarkSpectra():
    global specData
    specData['darkSpec'] = getSpectra()
    print 'recorded Dark Spectra'
    specData['darkSpecTimeStamp'] = datetime.datetime.now()
    return specData['darkSpec']

def correctSpectra(uncorSpec, coeff, darkSpec):
    global specData
    
    uncormindark = map(lambda x,y: x - y, uncorSpec, darkSpec)
    corSpec = []
    for band in uncormindark:
        corSpec.append(coeff[0] + coeff[1] * band + coeff[2] * (band **2) +
            coeff[3] * (band ** 3) + coeff[4]*(band ** 4) + coeff[5] * (band ** 5) 
            + coeff[6]*(band ** 6) + coeff[7]*(band ** 7))
    specData['corSpectra'] = corSpec
    return corSpec
    
def zeroFiller(numBytes):
    output = []
    if numBytes > 0:
        for i in range(numBytes):
            new = [0]
            output = output + list(new)
    
        return output

def split4BytePacket(time):
    #% time in us, output has to be LSB first
    outputPacket = [0,0,0,0]
    
    outputPacket[3] = int(math.floor(time/256**3))
    time = time - outputPacket[3]*256**3
    outputPacket[2] = int(math.floor(time/256**2))
    time = time - outputPacket[2]*256**2
    outputPacket[1] = int(math.floor(time/256))
    time = time - outputPacket[1]*256
    outputPacket[0] = int(math.floor(time))
    return outputPacket
    

#----------Serial interaction-------------
def flushNetSerial():
    '''Not currently working properly'''
    ser.setblocking(1)
    ser.settimeout(6)
#    read = ser.recv(100)
    read = '1'
    time.sleep(0.1)
    while len(read) > 0:
        try:
            read = ser.recv(1)
        except socket.error:
            read = '1'
            pass

# ----- from: http://code.activestate.com/recipes/408859-socketrecv-three-ways-to-turn-it-into-recvall/

def recv_basic(the_socket):
    total_data=[]
    while True:
        data = the_socket.recv(8192)
        if not data: break
        total_data.append(data)
    return ''.join(total_data)
    
def recv_timeout(the_socket,timeout=2):
    the_socket.setblocking(0)
    total_data=[];data='';begin=time.time()
    while 1:
        #if you got some data, then break after wait sec
        if total_data and time.time()-begin>timeout:
            break
        #if you got no data at all, wait a little longer
        elif time.time()-begin>timeout*2:
            break
        try:
            data=the_socket.recv(8192)
            if data:
                total_data.append(data)
                begin=time.time()
            else:
                time.sleep(0.1)
        except:
            pass
    return ''.join(total_data)

def recv_end(the_socket):
    End='\xc5\xc4\xc3\xc2'
    total_data=[];data=''
    while True:
            data=the_socket.recv(8192)
            if End in data:
                total_data.append(data[:data.find(End)])
                break
            total_data.append(data)
            if len(total_data)>1:
                #check if end_of_data was split
                last_pair=total_data[-2]+total_data[-1]
                if End in last_pair:
                    total_data[-2]=last_pair[:last_pair.find(End)]
                    total_data.pop()
                    break
    return ''.join(total_data) + End

def recv_size(the_socket):
    #data length is packed into 4 bytes
    total_len=0;total_data=[];size=sys.maxint
    size_data=sock_data='';recv_size=8192
    while total_len<size:
        sock_data=the_socket.recv(recv_size)
        if not total_data:
            if len(sock_data)>4:
                size_data+=sock_data
                size=struct.unpack('>i', size_data[:4])[0]
                recv_size=size
                if recv_size>524288:recv_size=524288
                total_data.append(size_data[4:])
            else:
                size_data+=sock_data
        else:
            total_data.append(sock_data)
        total_len=sum([len(i) for i in total_data ])
    return ''.join(total_data)
  
#-----------------------------------------
     
def writeReadPack(packet, variables,length,waitIntTime):
    global HOST, PORT, ser
    pack = makePacket(packet, variables)
    
    if type(ser) == socket._socketobject:

        ser.setblocking(1)
        ser.sendall(pack)
        time.sleep(0.05)
        read = recv_end(ser)
        return read

            
            
    else:
        ser.flushInput()
        ser.write(pack)
        #time.sleep((length*2)/specData['baudRate'])
        try:
            read = ser.read(length)
        except OSError:
            print 'Had an OS error trying again in 0.5sec'
            time.sleep(0.5)
            ser.flushInput()
            ser.write(pack)
            time.sleep((length*2)/specData['baudRate'])
            read = ser.read(length)
            pass
            
            
        # Retry on Time out
        if len(read) == 0:  
            for i in range(5):
    #            ser.flushInput()
                ser.write(pack)
                #time.sleep((length)/(specData['baudRate']/8))
                if waitIntTime:
                    time.sleep(specData['intTime'] * specData['averaging'])
                read = ser.read(length)

                try:
                    read = ser.read(length)
                except OSError:
                    print 'Had an OS error trying again in 1sec'
                    ser.write(pack)                    
                    time.sleep(1)
                    read = ser.read(length)
                    pass
                
                
                if len(read) > 0:
                    break
            if len(read) == 0:
                printError(1)
    #    Check it is a valid packet
        if validPacket(read):
            return read
        else:
            return []
            printError(2)
    
    
def writePack(packet, variables):
    global ser,HOST, PORT
    pack = makePacket(packet, variables)
    
    if type(ser) == socket._socketobject: 
        ser.sendall(pack)
    else:
        ser.write(pack)
        ser.flushInput()
    

#------Making stuff----------

def makeWavelengths():
    global specData
    pixnum = range(1024) #0 - 1023
    wavelength = []
    coeff = specData['waveCoeff']
    for i in pixnum:
        wavelength.append(coeff[0] + coeff[1]*i + coeff[2]*(i**2) + coeff[3] * (i**3)) 
    specData['wavelengths'] = wavelength

def makePacket(packetType, variables):
    #function [outputPacket] = makePacket(packetType, variables)
    #% Predefined Packets
    start = [193,192] #% 0xC1 0xC0
    protocol = [0,16] 
    flags = [0,0]
    error = [0,0]
    checksumType = [0]
    imLength = [4]
    bytesRemain = [20,0,0,0]
    checksum = zeroFiller(16)
    footer = [197,196,195,194]
    #% make a packet based on input choices
    
        
    if packetType == 'setIntTime':
        msgtype = [16, 0, 17, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables[0]) + zeroFiller(12) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getCorSpec':
        msgtype = [0, 16, 16, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + [0] + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getSerialNum':
        msgtype = [0, 1, 0, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + [0] + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'setBaud':
        msgtype = [16, 8, 0, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables[0]) + zeroFiller(12) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getBaud':
        msgtype = [0, 8, 0, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + [0] + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'setAveraging':
        msgtype = [16, 0, 18, 0]
        imLength = [2]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables[0]) + zeroFiller(12) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getAveraging':
        msgtype = [0, 0, 18, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + [0] + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'setBoxcar':
        msgtype = [16, 16, 18, 0]
        imLength = [1]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables[0]) + zeroFiller(12) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getBoxcar':
        msgtype = [0, 16, 18, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + [0] + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getAllTemp':
        msgtype = [2, 0, 64, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + [0] + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getWaveCoeffCount':
        msgtype = [0, 1, 24, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + [0] + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getWaveCoeffN':
        msgtype = [1, 1, 24, 0]
        imLength = [1]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables[0]) + zeroFiller(12) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getNonlinCoeffCount':
        msgtype = [0, 17, 24, 0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + [0] + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'getNonlinCoeffN':
        msgtype = [1, 17, 24, 0]
        imLength = [1]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables[0]) + zeroFiller(12) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'setOutputEn':
        msgtype = [16, 1, 32, 0]
        imLength = [8]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables[0]) + split4BytePacket(variables[1]) +  zeroFiller(8) +
            bytesRemain + checksum + footer)
        
        
    elif packetType == 'setGPIO':
        msgtype = [16, 3, 32, 0]
        imLength = [8]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables[0]) + split4BytePacket(variables[1]) +  zeroFiller(8) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'setSingleStrobePulse':
        msgtype = [17, 0, 48, 0]
        imLength = [4]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +
            checksumType + imLength + split4BytePacket(variables) + zeroFiller(16) +
            bytesRemain + checksum + footer)
        
    elif packetType == 'setSingleStrobeEnable':
        msgtype = [18, 0, 48, 0]
        imLength = [4]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +          
            checksumType + imLength + split4BytePacket(variables) + zeroFiller(16) +
            bytesRemain + checksum + footer)
            
    elif packetType == 'setTriggerMode':
        msgtype = [16, 1, 17, 0]
        imLength = [1]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +          
            checksumType + imLength + [variables[0]] + zeroFiller(15) +
            bytesRemain + checksum + footer)
            
    elif packetType == 'simTriggerPulse':
        msgtype = [32, 1, 17, 0]
        imLength = [0]
        outputPacket = (start + protocol + flags + error + msgtype + zeroFiller(10) +          
            checksumType + imLength + zeroFiller(16) +
            bytesRemain + checksum + footer)
            
    bytePack = array.array('B', outputPacket).tostring()
    return bytePack


#-----Processing Packets---------------

def processPacket(inPacketStr, packetType):
# take the packet as an input and output a struct of the packet parsed
    inPacket = []
    for x in range(len(inPacketStr)):
        inPacket.append( struct.unpack_from('B',inPacketStr,x)[0])
    
    
    p = namedtuple('packet', 'header,protocol,flags,error,msgtype,regarding,checksumType,imDataLen,imData,bytesRemain,payload,checksum,footer')
    
    #implement the header which will be the same for all packets
    p.header = convert2Num(inPacket,0,2)
    p.protocol = convert2Num(inPacket,2,2)
    p.flags = convert2Num(inPacket,4,2)
    p.error = convert2Num(inPacket,6,2)
    p.msgtype = convert2Num(inPacket,8,4)
    p.regarding = convert2Num(inPacket,12,4)
    p.checksumType = inPacket[22]
    p.imDataLen = inPacket[23]
    p.imData = inPacket[24:24 + p.imDataLen]
    p.bytesRemain = convert2Num(inPacket,40,4)
    p.payload = []

    if packetType == 'regular':
        p.checksum = inPacket[44:60]
        p.footer = convert2Num(inPacket,60,4)
    
    elif packetType == 'averaging':
        p.imData[0] = convert2Num(inPacket,24,2)
        p.checksum = inPacket[44:60]
        p.footer = convert2Num(inPacket,60,4)

    elif packetType == 'baud':
        p.imData[0] = convert2Num(inPacket,24,4)
        p.checksum = inPacket[44:60]
        p.footer = convert2Num(inPacket,60,4)
                    
    elif packetType == 'temperature':
        for i in range(4):
            p.imData[i] = convert2float(inPacket,24 + (i * 4))
        
        p.checksum = inPacket[44:60]
        p.footer = convert2Num(inPacket,60,4)

    elif packetType == 'spectra':
        for i in range(1024):
            p.payload.append(convert2Num(inPacket, 44 + (i * 2), 2))
                
        p.checksum = inPacket[44:60]
        p.footer = convert2Num(inPacket,60,4) 
                    
    elif packetType == 'string':
        p.imData = array.array('B', inPacket[24:24+p.imDataLen]).tostring()
        p.checksum = inPacket[44:60]
        p.footer = convert2Num(inPacket,60,4)

    elif packetType == 'LFloat32':
        for i in range(4):
            p.imData[i] = convert2float(inPacket,24 + (i * 4))
        
        p.checksum = inPacket[44:60]
        p.footer = convert2Num(inPacket,60,4)

    return p



def printError(errorNum):
    if errorNum == 1:
        print 'Error: Reading/Writing to device'

    elif errorNum == 2:
        print 'Error: Incomplete packet'

def validPacket(b):
# begining of packet must begin in C1C0 and the end C5C4C3
    if b[0:2] == '\xc1\xc0' and b[-4:-1] == '\xc5\xc4\xc3':
        return True
    else:
        return False

def convert2Num(packet, offset, numBytes):
    result = 0
    for i in range(numBytes):
        result += packet[offset + i] << (i * 8)

    return result

def convert2float(packet,offset):
    s = array.array('B', packet[offset : offset + 4]).tostring()
    h = struct.unpack('f',s)
    return h[0]


