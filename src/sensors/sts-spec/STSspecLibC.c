//
//  STSspecLibC.c
//  Library for interfacing with the STS microspectrometer from Ocean Optics
//
//  Created by Daniel Bongiorno on 21/08/13.
//  Copyright (c) 2013 Daniel Bongiorno. All rights reserved.
//

// THE PROCESS
/*
 Setup for LCM (initialisation)
 
 LCM fields
 - Spectrometer Serial Number
 - wavelength coeff.
 - nonlin coeff.
 
 Each Sample LCM:
 - time stamp
 - integration time
 - spectral data
 */

/*
 Process (loop):
 - take a sample
 - make a getSpectra packet
 - send out packet over serial to spectrometer
 - receive full packet back
 - process packet to get spectral data
 - record time stamp
 - if a certain number of pts in a sample are above a threshold (not just noise)
 - and pts are not over a threshold (thus not clipping)
 - change integration time to increase or decrease signal (put a limit on max integration time)
 - make setIntTime packet
 - send out packet over serial to spectrometer
 - send out LCM packet
 
 - repeat sampling process
 
 */
/*
 Sending data out and receiving in:
 - all data packets in and out
 - Start with: 0xC1, 0xC0
 - Finish with: 0xC5, 0xC4, 0xC3, 0xC2
 
 */


#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <time.h>


//#include "perls-common/serial.h"

#include <unistd.h>   // UNIX standard function definitions
#include "STSspecLibC.h"
#include <sys/select.h>

#define DEBUGGING
#define TIMEOUTS_ENABLED


_Bool rxTxInProgress = false;
int baudRateGlobal = 9600;
long intTimeGlobal = 500000;


// ----Get/Set From Spectrometer---




int getSpectra(int serialPort, unsigned short spectra[]){
    int error;
    char outputPacket[64];
    //float sitin[2];
    double timeout = 10000;
    
    char buffer[2112];
    char foundPacket[2112];
    
    makePacket(10, 0, outputPacket);
    error = sendPacket(serialPort, outputPacket);
    usleep(intTimeGlobal);
    error = receivePacket(buffer, 2112, serialPort, timeout, foundPacket);
    error = processPacket(foundPacket, spectra, NULL);
    return error;
}

int getWaveCoeff(int serialPort, float coeff[]){
    char outputPacket[64];
    unsigned short numC[1], standin[1];
    int error;
    double timeout = 2;
    char buffer[256];
    char foundPacket[64];
    float tempCoeff[1];
    
    makePacket(11, 0, outputPacket);
    //error = sendReceivePacket(serialPort, outputPacket, numC,NULL, timeout);
    error = sendPacket(serialPort, outputPacket);
    usleep(20000);
    error = receivePacket(buffer, 64, serialPort, timeout, foundPacket);
    //for (int i = 0; i < 64; i++) {
    //    printf("0x%02x\t",buffer[i] & 0xFF);
    //}
    error = processPacket(foundPacket, numC, NULL);
    printf("numC: %lu\n",numC[0]);
    
    for (int i = 0; i < numC[0]; i++) {
        
        makePacket(2, i, outputPacket);
        error = sendPacket(serialPort, outputPacket);
        usleep(20000);
        error = receivePacket(buffer, 64, serialPort, timeout, foundPacket);
        //for (int j = 0; j < 64; j++) {
        //    printf("0x%02x\t",buffer[j] & 0xFF);
        //}
        //printf("\n\n");
        error = processPacket(foundPacket, standin, tempCoeff);
        //printf("Coef2 %.9f\n", tempCoeff[0]);
        coeff[i] = tempCoeff[0];
    }
    return error;
}

int getNonlinCoeff(int serialPort, float coeff[]){
    char outputPacket[64];
    unsigned short numC[1];
    float coeffTemp[1];
    int error;
    double timeout = 2;
    float tempCoeff[1];
    char buffer[256];
    char foundPacket[64];
    
    makePacket(12, 0, outputPacket);
    error = sendPacket(serialPort, outputPacket);
    usleep(20000);
    error = receivePacket(buffer, 64, serialPort, timeout, foundPacket);
    error = processPacket(foundPacket, numC, NULL);
    printf("numC: %lu\n",numC[0]);
    for (int i = 0; i < numC[0]; i++) {
        
        makePacket(3, i, outputPacket);
        error = sendPacket(serialPort, outputPacket);
        usleep(40000);
        error = receivePacket(buffer, 64, serialPort, timeout, foundPacket);
        //for (int j = 0; j < 64; j++) {
        //    printf("0x%02x\t",buffer[j] & 0xFF);
        //}
        //printf("\n\n");
        error = processPacket(foundPacket, NULL, tempCoeff);
        //printf("Coef2 %.9f\n", tempCoeff[0]);
        coeff[i] = tempCoeff[0];
    }
    return error;
}

int getTemp(int serialPort, float temps[]){
    char outputPacket[64];
    unsigned long sitin[1];
    int error;
    double timeout = 2000;
    unsigned char buffer[64], foundPacket[64];
    makePacket(14, 0, outputPacket);
    error = sendPacket(serialPort, outputPacket);
    error = receivePacket(buffer, 64, serialPort, timeout, foundPacket);
    error = processPacket(foundPacket, NULL, temps);
    return error;
}

int getSerialNum(int serialPort, char serialNum[]){
    char outputPacket[64];
    int error;
    double timeout = 2000;
    unsigned char buffer[256], foundPacket[64];
    unsigned short serialNumLong[16];
    
    makePacket(13, 0, outputPacket);
    error = sendPacket(serialPort, outputPacket);    
    error = receivePacket(buffer, 64, serialPort, timeout, foundPacket);
    error = processPacket(foundPacket, serialNumLong, NULL);
    for (int i = 0; i < 16 ; i++) {
        serialNum[i] = (char)(serialNumLong[i]);
    }
    
    return error;
}


int setIntTime(int serialPort, long intTime){
    char outputPacket[64];
    int error;
    //    long returnData[2];
    intTimeGlobal = intTime;
    makePacket(1, intTime, outputPacket);
    error = sendPacket(serialPort, outputPacket);
    //    error = sendReceivePacket(serialPort, outputPacket, returnData, NULL, 1);
    
    return error;
    
}

int collectSpectra(int serialPort ) {
    /*
     char serialNum[16];
     float waveCoeff[8];
     float nonlinCoeff[8];
     int error;
     long spectra[1024];
     
     // get serial number
     getSerialNum(serialNum);
     // get wavelength Count & coeff
     getWaveCoeff(waveCoeff);
     // get nonlin coeff
     getNonlinCoeff(nonlinCoeff);
     // set intergration time to 10ms
     setIntTime(10000);
     
     // send coeffs and serial number to LCM
     //----------TO DO-------------
     
     //adjust the exit conditions
     while (1) {
     //get spectra
     error = getSpectra(serialPort, spectra);
     // send spectra out to LCM
     //----------TO DO-------------
     
     //check signal levels
     
     
     
     }
     */
     return 0;
}




// -------Serial Routines------

//int initSerial(char portAddress[],int baud){
//    int error;
//    error = serialport_init(portAddress, baud);
//    baudRateGlobal = baud;
//    return error;
//}

int sendReceivePacket(int serialPort, char packetToSend[],unsigned long returnData[], float returnFloat[], double timeout){
    int error, txError;
    char inPacket[3000];
    int numRxAttempts = 2;
    int numTxAttempts = 7;
    char foundPacket[2112];
    time_t startTime, curTime;
    time(&startTime);
    double timeDiff = 0;
    
    
    for (int j = 0; j < numTxAttempts; j++) {
        
        txError = sendPacket(serialPort, packetToSend);
        
        int sleeptime = timeDelayCalc(baudRateGlobal, 64) * 1000;
        //time delay calc returns in milli sec and usleep is in micro seconds
        usleep(sleeptime);
        
        usleep(intTimeGlobal);
        //sleep(1);
        
        sleeptime = timeDelayCalc(baudRateGlobal, 2112) * 1000;
        usleep(sleeptime * 1.05);
        
        //usleep(4000);
        //sleep based on baud rate and packet size & integration time
        
        for (int i = 0; i < numRxAttempts; i++) {
            
            //Make sure we are not still transmitting
            while (rxTxInProgress == true) {
                time(&curTime);
                timeDiff = difftime(curTime, startTime);
                usleep(1);
                if (timeDiff > timeout) {
#ifdef DEBUGGING
                    printf("TX in Progress: Timed out from waiting\n");
#endif
                    break;
                }
            }
            
            error = receivePacket(inPacket, 2112, serialPort, timeout, foundPacket);
            if (error == 0) {
                // it has collected a full packet
                error = processPacket(foundPacket, returnData, returnFloat);
                if (error == 0) {
                    return error;
                }
            }
#ifdef DEBUGGING
            printf("Error = %u, RxAttempt:%u, TxAttempt:%u\n",error,i+1,j+1);
            //usleep(1000);
            //wait a millisec
#endif
        }
        if (error == 0) {
            break;
        }
        
        //flushBuffer(serialPort);
    }
    
    if (error == 0) {
        return 0;
    }
    else {
        // it failed after numAttempts amount so return 1.
        return error;
    }
    return 1;
}


int sendPacket(int serialPort, char packetToSend[]) {
    int n = 64; //number of elements in the packet, hardcoded to 64 because sizeof wasn't working correctly
    //int i = 0;
    time_t startTime, curTime;
    time(&startTime);
    double timeDiff = 0;
    int timeout = 2;
    int error = 0;
    
#ifdef DEBUGGING
    if (rxTxInProgress == true) {
        printf("RxTX in Progress: waiting before sending\n");
    }
#endif
    
    //    check to make sure we are not currently receiving packets, implement a timeout here
    while (rxTxInProgress == true) {
        time(&curTime);
        timeDiff = difftime(curTime, startTime);
        usleep(1);
        if (timeDiff > timeout) {
#ifdef DEBUGGING
            printf("Rx in Progress: Timed out from waiting\n");
#endif
            break;
        }
    }
    
    
    
    rxTxInProgress = true;
    error = write(serialPort, packetToSend, n);
    rxTxInProgress = false;
    return error;
}


int receivePacket(unsigned char buf[], int bufferSize, int serialPort, double timeout, char foundPacket[]) {
    
    ssize_t n = 0;
    fd_set rfds;
    int error = 0;
    int offset = 0;
    unsigned char rawbuf[3000];
    
    struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
    
    
    memset(buf,0x00,bufferSize);
    while(1) {
        
        FD_ZERO(&rfds);
        FD_SET(serialPort, &rfds);
        
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        
        if(ret != 0 && FD_ISSET(serialPort, &rfds))
        {
            n = read(serialPort, rawbuf, bufferSize-offset);
            if(n < 0) {
                return -1;
            } else if (n == 0) {
                
            } else {
                memcpy(&buf[offset], rawbuf, n);
                offset += n;
            }
            
            
        }
        usleep(1000);
        timeout--;
        if (timeout < 0){
        printf("Timed out\n");
            break;
        }
        if (offset >= bufferSize) {
            break;
        }
        
    }
    
    error = findPacket(buf, bufferSize, foundPacket);
    return error;
    
}

//    //time_t startTime, curTime;
//    int error = 1;
//    //double timeDiff = 0;
//    int numBytesToRead = 3000;
//    int headCount = 0;
//    
//    memset(buffer, 0, bufferSize);
//    
//    //error = serialport_readNbytes(serialPort, buffer, numBytesToRead, timeout);
//    //New section
//    char b[1];  // read expects an array, so we give it a 1-byte array
//    int i=0;
//    rxTxInProgress = true;
//    do {
//        int n = read(serialPort, b, 1);  // read a char at a time
//        if( n==-1) return -1;    // couldn't read
//        if( n==0 ) {
//            usleep( 2 * 1000 );  // wait 1 msec try again
//            timeout--;
//            buffer[i] = b[0];
//            i++;
//            continue;
//        }
//        
//        
//        
//        //lets try to find the end of a packet and cut Rx at that pt
//        //find the end
//        /*
//        if (buffer[i] == '\xc5') {
//            headCount = 3;
//        }
//        else if (headCount == 3 && buffer[i] == '\xc4') {
//            headCount = 4;
//        }
//        else if (headCount == 4 && buffer[i] == '\xc3') {
//            headCount = 5;
//        }
//        else if (headCount == 5 && buffer[i] == '\xc2') {
//            headCount = 6;
//            
//#ifdef DEBUGGING
//            printf("Rx: found 0xc2, end of packet\n");
//#endif
//            //we have found the end of the packet
//            flushBuffer(serialPort);
//            break;
//        }
//        */
//        
//    } while(i < bufferSize && timeout>0 );
//    
//    if (timeout <= 0) {
//        printf("Timeout while receiving bytes\n");
//    }
//    
//    
//    rxTxInProgress = false;
//    error = findPacket(buffer, bufferSize, foundPacket);
//    return error;



int flushBuffer(int serialPort) {
    int error;
    char c[3000];
    //error = serialport_flush(serialPort);
//    for (int i = 0; i < 3000; i++) {
    error = read(serialPort, c, 3000);
//    }
#ifdef DEBUGGING
    printf("Flush %u\n",error);
#endif
    return error;
}


// ------Processing Routines------


int findPacket(char buffer[], int bufferSize, char foundPacket[]) {
    int headCount = 0;
    int indexes[2] = {0,0};
    _Bool foundStart = false;
    
    for (int i = 0; i < bufferSize; i++) {
        if (buffer[i] == '\xc1' && foundStart == false) {
            headCount = 1;
        }
        
        else if (headCount == 1 && buffer[i] == '\xc0' && foundStart == false) {
            foundStart = true;
            headCount = 2;
            indexes[0] = i-1;
        }
        
        //Can cut this because it is not done in Rx
        //find the end
        if (headCount == 2 && buffer[i] == '\xc5') {
            headCount = 3;
            indexes[1] = i;
        }
        else if (headCount == 3 && buffer[i] == '\xc4') {
            headCount = 4;
            indexes[1] = i;
        }
        else if (headCount == 4 && buffer[i] == '\xc3') {
            headCount = 5;
            indexes[1] = i;
        }
        else if (headCount == 5 && buffer[i] == '\xc2') {
            headCount = 6;
            indexes[1] = i;
            break;
        }
    }
    if (headCount == 6) {
        //we have found a complete packet
        int length = indexes[1] - indexes[0] + 1;
        for (int i = 0; i < length; i++ ){
            foundPacket[i] = buffer[i+indexes[0]];
        }
        return 0;
    }
#ifdef DEBUGGING
    printf("Find Pack: Fail, header Count: %u\n, strt Header: %u, end Header: %u, end: 0x%x, 0x%x, 0x%x, 0x%x\n",headCount, indexes[0],indexes[1], buffer[indexes[0] + 2111], buffer[indexes[0] + 2112], buffer[indexes[0] + 2113], buffer[indexes[0] + 2114]);
#endif
    return 1;
}


/*
 int receivePacketOLD(char buffer[], int serialPort, double timeout) {
 _Bool foundBegin = false;
 _Bool foundEnd = false;
 unsigned int index = 0;
 time_t startTime, curTime;
 double timeDiff = 0;
 
 
 time(&startTime);
 //record the beginning time
 
 //Have we finished pulling a packet off the serial device?
 
 unsigned int headerMarker = 0;
 //this is a count of how many header and trail characters have correctly been detected so you can determine the start and finish
 
 rxTxInProgress = true;
 //Try finding the beginning of the packet with 0xc1 and 0xc0
 while (foundBegin == false) {
 char c[1];
 //char *header[2] = {"\xc1","\xc0"};
 
 ssize_t bufSize = 1;
 
 int error = read(serialPort, c, bufSize);
 //current read character off serial device
 
 switch (headerMarker) {
 case 0:
 //we are looking for the first character
 if (strcmp(c,"\xc1") == 0) {
 headerMarker = 1;
 }
 break;
 
 case 1:
 if (strcmp(c, "\xc0\x01") == 0) {
 //we have found the beginning of a packet
 foundBegin = true;
 }
 break;
 
 default:
 headerMarker = 0;
 //we haven't found what we are looking for so keep going.
 //need a time out here
 break;
 }
 #ifdef TIMEOUTS_ENABLED
 //check for time out
 time(&curTime);
 timeDiff = difftime(curTime, startTime);
 
 if (timeDiff > timeout) {
 return 1;
 }
 //take out timeout for debugging
 #endif
 }
 
 buffer[0] = 0xc1;
 buffer[1] = 0xc0;
 index = 2;
 headerMarker = 0;
 int error = serialport_readNbytes(serialPort, buffer, 2112, timeout);
 
 //so we have the beginning we now need to write this out to buffer
 if (foundBegin == true) {
 while (foundEnd == false) {
 char c[1];
 
 ssize_t bufSize = 1;
 if (index > 2112) {
 //check we haven't gone off the edge of a packet
 return 2;
 
 } else {
 int error = read(serialPort, c, bufSize);
 //current read character off serial device
 }
 
 buffer[index] = c[0];
 //write to packet buffer
 
 index++;
 //increase index so we dont write over our buffer
 
 switch (headerMarker) {
 case 0:
 if (strcmp(c, "\xc5\x01") == 0) {
 //we have the 4th last header marker
 headerMarker = 1;
 }
 break;
 case 1:
 if (strcmp(c, "\xc4\x01") == 0) {
 //we have the 3rd last header marker
 headerMarker = 2;
 }
 break;
 case 2:
 if (strcmp(c, "\xc3\x01") == 0) {
 //we have the 2nd header marker
 headerMarker = 3;
 }
 break;
 case 3:
 if (strcmp(c, "\xc2\x01") == 0) {
 //we have the last header marker
 headerMarker = 4;
 foundEnd = true;
 rxTxInProgress = false;
 return 0;
 }
 break;
 
 default:
 headerMarker = 0;
 break;
 }
 //check for time out
 time(&curTime);
 timeDiff = difftime(curTime, startTime);
 
 // Take out timeout for debugging
 if (timeDiff > timeout) {
 return 1;
 }
 
 }
 }
 
 
 //flush serial buffer
 flushBuffer(serialPort);
 
 rxTxInProgress = false;
 return error;
 }
 */

int makePacket(int packetType, long variables, char outputPacket[]) {
    /*
     packetType is an integer:
     0 - Set Baud
     1 - Set Integration Time
     2 - Get wavelength coefficient N
     3 - Get nonlinear coefficient N
     
     10 - Get Spectral reading return packet is 2112 bytes
     11 - Get a count of the number of wavelength coefficients
     12 - Get a count of the number of nonlinear coefficients
     13 - Get serial number
     14 - Get all Temperatures
     */
    char start[] = {0xc1,0xc0,0x00,0x10,0x00,0x00,0x00,0x00};
    
    for (int i = 0; i < 8; i++) {
        outputPacket[i] = start[i];
    }
    
    //Message type
    if (packetType == 0) {
        //Set Baud
        char msgType[] = {0x10,0x08,0x00,0x00};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[i];
        }
    }
    else if (packetType == 1) {
        //Set intTime
        char msgType[] = {0x10,0x00,0x11,0x00};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[i];
        }
    }
    else if (packetType == 2){
        //Get waveCoeffN
        char msgType[] = {0x00,0x18,0x01,0x01};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[3 - i];
        }
    }
    else if (packetType == 3){
        //Get nonlinCoeffN
        char msgType[] = {0x00,0x18,0x11,0x01};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[3 - i];
        }
    }
    
    
    
    
    else if (packetType == 10){
        //Get Spec
        char msgType[] = {0x00,0x10,0x10,0x00};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[i];
        }
    }
    else if (packetType == 11){
        //Get waveCoeffCount
        char msgType[] = {0x00,0x18,0x01,0x00};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[3 - i];
        }
    }
    
    else if (packetType == 12){
        //Get nonlinCoeffCount
        char msgType[] = {0x00,0x18,0x11,0x00};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[3 - i];
        }
    }
    else if (packetType == 13){
        //Get Serial Number
        char msgType[] = {0x00,0x00,0x01,0x00};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[3 - i];
        }
    }
    else if (packetType == 14){
        //Get temps
        char msgType[] = {0x00,0x40,0x00,0x02};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[3 - i];
        }
    }
    
    else {
        //Get Spec
        char msgType[] = {0x00,0x10,0x10,0x00};
        for (int i = 0; i < 4; i ++) {
            outputPacket[i + 8] = msgType[i];
        }
    }
    
    //now zero filler for 10 bytes and checksum type (=0)
    
    for (int i = 0; i < 11; i++){
        outputPacket[i + 12] = 0x00;
    }
    
    zeroFiller(12, 11, outputPacket);
    
    // set command will have some user selective parameters
    if (packetType > 9) {
        //get command
        zeroFiller(23, 17, outputPacket);
    }
    else {
        //set command
        
        //split up variables in 4 byte packet LSB first
        char var[4];
        split4Byte(var, variables);
        
        outputPacket[23] = 0x04;
        for (int i = 0; i<4; i++) {
            outputPacket[24+i] = var[i];
        }
        zeroFiller(28, 12, outputPacket);
        
    }
    
    //Bytes remain
    outputPacket[40] = 0x14;
    zeroFiller(41, 3, outputPacket);
    
    //Checksum
    zeroFiller(44, 16, outputPacket);
    
    //Footer
    char footer[] = {0xc5, 0xc4, 0xc3, 0xc2};
    for (int i = 0; i < 4; i++) {
        outputPacket[i + 60] = footer[i];
    }
    
    return 0;
}

int processPacket(char inPacket[], unsigned short returnData[], float floatData[]) {
    //do some checks for errors on the packet
    short msb = (short)(inPacket[6]);
    short lsb = (short)(inPacket[7]);
    short error = (msb << 8) + lsb;
    if (error > 0) {
        return (int)(error);
    }
    
    
    //check message type
    unsigned long msgType = ((unsigned long)inPacket[8]) + (((unsigned long)inPacket[9]) << 8) + (((unsigned long)inPacket[10]) << 16) + (((unsigned long)inPacket[11]) << 24);
    
    if (msgType == 0x00101000) {
        //spectral packet
        //bytes 44 - 2091 will be the 2048 byte payload
        //should be LSB first
        for (int i = 0; i < 1024; i++) {
            unsigned int msb = (inPacket[2*i + 45]);
            unsigned char lsb = (inPacket[2*i + 44]);
            unsigned short d = (msb << 8) + lsb;
            returnData[i] = d;
        }
        return 0;
        
    }
    else if (msgType == 0x00400002) {
        //reading all temperature sensors
        
        float * f = (float *)(inPacket + 24);
        float detectTemp = *f;
        
        float * g = (float *)(inPacket + 32);
        float boardTemp = *g;
        
        floatData[0] = detectTemp;
        floatData[1] = boardTemp;
        //might have a problem here with casting, these are floats which we are making longs
        return 0;
    }
    
    else if (msgType == 0x00000101) {
        //serial number length
        char length = inPacket[24];
        returnData[0] = length;
        //need to figure out how to return this
        return 0;
    }
    
    else if (msgType == 0x00000100) {
        //serial number
        //char serialNum[16];
        for (int i = 0; i < 16; i++) {
            returnData[i] = inPacket[i+24];
        }
        //need to figure out how to return this
        return 0;
    }
    else if (msgType == 0x00180100) {
        //wavelength Coeff count
        returnData[0] = (unsigned short) inPacket[24];
        return 0;
    }
    else if (msgType == 0x00180101) {
        //wavelength Coeff number x
        //float coeff = (float)(inPacket[24]) + (float)(inPacket[25] << 8) + (float)(inPacket[26] << 16) + (float)(inPacket[27] << 24);
        float * f = (float *)(inPacket + 24);
        float coeff = *f;
        
        //printf("Coef %.9f\n", coeff);
        
        floatData[0] = coeff;
        return 0;
    }
    
    else if (msgType == 0x00181100) {
        //nonlin Coeff count
        returnData[0] = (unsigned short) inPacket[24];
        return 0;
    }
    else if (msgType == 0x00181101) {
        //nonlin Coeff number x
        //float coeff = (float)(inPacket[24]) + (float)(inPacket[25] << 8) + (float)(inPacket[26] << 16) + (float)(inPacket[27] << 24);

        
        float * f = (float *)(inPacket + 24);
        float coeff = *f;
        
        //printf("Coef %.9f\n", coeff);
        
        floatData[0] = coeff;
        return 0;
    }
    
    //if the msgtype didnt match anything return error 100.
    return 100;
}


int zeroFiller(int startIndex, int numZeros, char packet[]) {
    for (int i = startIndex; i < (startIndex + numZeros); i ++) {
        packet[i] = 0x00;
    }
    return 0;
}

int split4Byte(char packet[], long number){
    //want to take the number and split it into a 4 byte packet with LSB first
    packet[3] = (char)(floorl(number/(pow(256,3))));
    number = number - packet[3] * (pow(256,3));
    
    packet[2] = (char)(floorl(number/(pow(256,2))));
    number = number - packet[2] * (pow(256,2));
    
    packet[1] = (char)(floorl(number/(256)));
    number = number - packet[1] * (256);
    
    packet[0] = (char)(floorl(number));
    
    return 0;
}


int timeDelayCalc(int baud, int numbytes) {
    //This calculates how long a delay should be for RX TX of a packet
    //baud is in bits/sec, there is a stop and a start bit so divide by 10
    //result is in milli sec
    // + 1 at end for rounding down due to int maths
    return (numbytes * 1000) / (baud/10) + 1;
    
    
}


short findMax(unsigned short result[], unsigned short array[], int arraySize){
    //result[0] is the index of the max result
    //result[1] is the value of that max result.
    result[1] = array[0];
    result[0] = 0;
    for (int i = 0; i < arraySize; i++) {
        if (array[i] > result[1]) {
            //we have found a larger value
            //printf("found %lu, at %u\n",array[i], i);
            result[0] = i;
            result[1] = array[i];
        }
    }
    return result[0];
}

float findMean(unsigned short array[], int arraySize, int startIdx) {
    float mean = 0;
    float n = (float) arraySize;
    for (int i = startIdx; i < (startIdx + arraySize); i++) {
        //printf("%u\n", array[i]);
        mean += (array[i] / n);
    }
    return mean;
}

long checkIntTime(unsigned long specData[], int arraySize, long intTime, unsigned long thresholds[] ){
    //This routine searches through a spectra array and determines if it has maxed out or the signal is too small, it then does a calculation to determine a better int time.
    //Thresholds: [0] = Max threshold, [1] = Min Threshold, [2] = desired mean.
    unsigned short max[2];
    long newIntTime = intTime;
    float fnewIntTime;
    
    findMax(max, specData, arraySize);
    _Bool saturated = false;
    _Bool tooLow = false;
    
    //For 400-700nm on the USB2000+ index = 172 -> 1063
    float mean = findMean(specData, thresholds[4], thresholds[3]);
    float gain = thresholds[2] / mean;
    
    //Check for over the max threshold
    if (max[1] >= thresholds[0]) {
        saturated = true;
        gain = gain * 0.95;
        printf("Signal has saturated: value = %u\n",max[1]);
    }
    //check for less than min threshold
    if (max[1] <= thresholds[1]) {
        tooLow = true;
        printf("Signal is too low: value = %u\n",max[1]);
        if (gain > 20.0) {
            //it is going to ramp the gain up too much and it will bounce around, so put a threshold on it.
            gain = 5.0;
        }
    }
    
    fnewIntTime = intTime * gain;
    if (fnewIntTime < 1000) {
        fnewIntTime = 1000;
    }
    
    if (fnewIntTime > 8000000) {
        fnewIntTime = 8000000;
    }
    
    printf("intTime %u, mean: %.1f gain: %.1f newIntf %.1f max %u\n ", intTime, mean, gain, fnewIntTime,max[1]);
    
    newIntTime = (long) fnewIntTime;
    return newIntTime;
}




