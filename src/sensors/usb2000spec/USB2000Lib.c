//
//  USB2000Lib.c
//  Learning C
//
//  Created by Daniel Bongiorno on 4/09/13.
//  Copyright (c) 2013 Daniel Bongiorno. All rights reserved.
//

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <unistd.h>   // UNIX standard function definitions
//#include "arduino-serial-lib.h"
#include <time.h>
#include <sys/select.h>
#include "USB2000Lib.h"

//#define DEBUGGING
#define TIMEOUTS_ENABLED

_Bool rxTxInProgress = false;
int baudRateGlobal = 9600;
long intTimeGlobal = 1000;





// ----Get/Set From Spectrometer---
int getSpectra(int serialPort, unsigned long spectra[]) {
    int error = 0;
    int timeout = 1000;
    char packet[] = "S";
    unsigned char rawSpec[8194],rawSpecBuf[512];
    unsigned char bufp[6];
    unsigned long x,y,n;
    int numSpec = 0, numBytes = 0, numSbytes;
    
    error = sendPacket(serialPort, packet, 1);
    specHeader header;
    usleep((useconds_t)intTimeGlobal);
    //sleep(1);
    error = readHeaderPacket(&header, serialPort);
    #ifdef DEBUGGING
    printf("read header: error %u\n",error);
    #endif
    
    if (error == 0) {
        
    
        switch (header.pixelMode) {
            case 0:
                //read all 2048
                numSpec = 2048;
                break;
            case 3:
                //read from x to y every n
                receiveData(bufp, 6, serialPort, timeout);
                convertBytesToLong(&bufp[0], 2, &x);
                convertBytesToLong(&bufp[2], 2, &y);
                convertBytesToLong(&bufp[4], 2, &n);
                numSpec = floorl((y-x)/n);
                
                
            default:
                break;
        }
        if (header.dataSize == 0) {
            numBytes = numSpec * 2 + 2;
            numSbytes = 2;
        }
        else {
            numBytes = numSpec * 4 + 2;
            numSbytes = 4;
        }
    
        int offset = 0;
        while (1) {
        
            int n = receiveData(rawSpecBuf, 512, serialPort, timeout);

        
            memcpy(&rawSpec[offset], rawSpecBuf, n);
            offset += n;
            if (offset >= numBytes) {
                break;
            }
        
        }
    }
    else {
    #ifdef DEBUGGING
        printf("Flushing serial buffer\n");
    #endif
        flushBuffer(serialPort);
    }
    
    for (int i = 0; i < numSpec; i++) {
        convertBytesToLong(&rawSpec[i*numSbytes], numSbytes, &spectra[i]);
        //printf("%u %u %u %lu\n",i, rawSpec[i*2], rawSpec[i*2 + 1], spectra[i]);
        //printf("%lu\n", spectra[i]);
    }
    
    //flushBuffer(serialPort);
    
    
    return error;
}



int setIntTime(int serialPort, long intTime) {
    unsigned char intChar[4];
    int error = 0;
    split4Byte(intChar, intTime);
    //printf("split 0x%x 0x%x 0x%x 0x%x\n", intChar[0], intChar[1], intChar[2], intChar[3]);

    
    unsigned char outPacket[] = {'i',0x00,0x00,0x00,0x00};
    for (int i = 1; i < 5; i++) {
        outPacket[i] = intChar[i-1];
    }
    error = sendPacket(serialPort, outPacket,5); 
    //printf("sending 0x%x 0x%x 0x%x 0x%x 0x%x\n", outPacket[0], outPacket[1], outPacket[2], outPacket[3], outPacket[4]);
   
    
    
    
    unsigned char buf[1];
    error = receiveData(buf,1,serialPort,100);
    
    if (buf[0] == 0x06){
    
        intTimeGlobal = intTime;
        error = 0;
    }
    else {
    error = 1;
    }
    
    //send a query request to check intTime
    unsigned char out[] = {'?', 'i'};
    error = sendPacket(serialPort, out,2);
    
    unsigned char buf2[5];
    error = receiveData(buf2,5,serialPort,100);
    //printf("0x%x 0x%x 0x%x 0x%x 0x%x\n", buf2[0], buf2[1], buf2[2], buf2[3], buf2[4]);
    
    long intTimeCheck;
    convertBytesToLong(&buf2[1], 4, &intTimeCheck);
    
    //printf("int check %lu\n", intTimeCheck);
    
    if (intTimeCheck == intTime) {
        error = 0;
        }
    else {
        error = 2;
    }     
    
    
    return error;
}

int setTriggerMode(int serialPort, int triggerMode){
    int error = 0;
    char packet[] = {'T',0x00,0x00};
    if (triggerMode < 5) {
        packet[2] = (char)triggerMode;
        error = sendPacket(serialPort, packet,3);
    } else {
        error = 1;
    }
    unsigned char buf[1];
    error = receiveData(buf, 1, serialPort, 500);
    if (buf[0] == 0x06) {
        return 0;
        }
    else { 
        return error;
    }
    return error;
}

int setDataMode(int serialPort, _Bool binaryMode) {
    int error = 0;
    char ascii[2] = "Aa";
    char binary[2] ="Bb";
    if (binaryMode == true) {
        error = sendPacket(serialPort, binary,2);
    } else {
        error = sendPacket(serialPort, ascii,2);
    }
    unsigned char buf[1];
    error = receiveData(buf,1,serialPort,100);
    
    if (buf[0] == 0x06){
        error = 0;
    }
    else {
        error = 1;
    }
    
    return error;
}


//CHECK RXTX ROUTINES, WITH HARDWARE TRIGGER COULD BE SITTING THERE FOR A WHILE WAITING FOR RX SO EXTEND TIMEOUT TO SOMETHING LONG

// ----Serial Routines---
/*
int initSerial(char portAddress[],int baud){
    
    // Initialise serial port to portAddress eg: '/dev/tty.PL2303000' and baud, STS will take upto 115200 
    int error;
    error = serialport_init(portAddress, baud);
    baudRateGlobal = baud;
    return error;
}
*/

int sendReceivePacket(int serialPort, char packetToSend[], int outPacketSize, unsigned long returnData[], int timeout) {
    int error, txError;
    unsigned char inPacket[3000];
    int numRxAttempts = 2;
    int numTxAttempts = 7;
    char foundPacket[2112];
    time_t startTime, curTime;
    time(&startTime);
    double timeDiff = 0;
    
    
    for (int j = 0; j < numTxAttempts; j++) {
        
        txError = sendPacket(serialPort, packetToSend, outPacketSize);
        
        int sleeptime = timeDelayCalc(baudRateGlobal, 64) * 1000;
        //time delay calc returns in milli sec and usleep is in micro seconds
        //        usleep(sleeptime);
        //
        //        usleep(intTimeGlobal);
        //        //sleep(1);
        //
        //        sleeptime = timeDelayCalc(baudRateGlobal, 2112) * 1000;
        //        usleep(sleeptime * 1.05);
        
        //usleep(4000);
        //sleep based on baud rate and packet size & integration time
        
        for (int i = 0; i < numRxAttempts; i++) {
            
            //Make sure we are not still transmitting
            while (rxTxInProgress == true) {
                time(&curTime);
                timeDiff = difftime(curTime, startTime);
                usleep(1000);
                if (timeDiff > timeout) {
#ifdef DEBUGGING
                    printf("TX in Progress: Timed out from waiting\n");
#endif
                    break;
                }
            }
            
            error = receivePacket(inPacket, 4113, serialPort, timeout);
            if (error == 0) {
                // it has collected a full packet
                error = processPacket(foundPacket, returnData);
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

//----------

int sendReceiveChar(int serialPort, char packetToSend[], int outPacketSize, char returnChar[], int timeout){
    
    int i = 0;
    time_t startTime, curTime;
    time(&startTime);
    double timeDiff = 0;
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
    write(serialPort, packetToSend, outPacketSize);
//    for (i = 0; i < outPacketSize; i++) {
//        serialport_writebyte(serialPort, packetToSend[i]);
//        usleep(5);
//    }
    rxTxInProgress = false;
    
    
    error = receivePacket(returnChar, 5, serialPort, timeout);
    return error;
    
}


int sendPacket(int serialPort, char packetToSend[], int outPacketSize) {
    //check for ACK 0x06 or NAK 0x15
    int error = 0;
    int i = 0;
    time_t startTime, curTime;
    time(&startTime);
    double timeDiff = 0;
    int timeout = 2;
    int numAttempts = 5;
    
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
    
    unsigned char buf[1];
    
    rxTxInProgress = true;
    error = write(serialPort, packetToSend, outPacketSize);
//    for (i = 0; i < outPacketSize; i++) {        
//        serialport_writebyte(serialPort, packetToSend[i]);
//        usleep(5);
//    }
    //receiveData(buf, 1, serialPort, timeout);
    rxTxInProgress = false;
    
    //#ifdef DEBUGGING
    //printf("send error: %u\n", error);
    //#endif
    
    return error;
}

int readHeaderPacket(specHeader* head, int serialPort) {
    unsigned char buf[15];
    unsigned char rawbuf[15];
    int offset = 0;
    long timeout = 2000;
    long tmo = 2;
    time_t startTime, curTime;
    time(&startTime);
    double timeDiff = 0;
    
    while(1) {
        //printf(".");
        int n = receiveData(rawbuf, 15-offset, serialPort, timeout);
        #ifdef DEBUGGING
        printf("n=%u offset=%u\n",n,offset);
        #endif
        memcpy(&buf[offset], rawbuf, n);
        offset += n;
        if (offset >= 15) {
           break;
        }
        time(&curTime);
        timeDiff = difftime(curTime, startTime);
        if (timeDiff > tmo) {
            return 1;
        } 
       
   
    }        
            
    //receiveData(buf, 15, serialPort, 2000);
    
    if (buf[0] == 0x02) {
        head->acqSuccess = true;
    } else {
        head->acqSuccess = false;
        return 1;
    }
    if (buf[1] == 0xFF && buf[2] == 0xFF) {
        //we can continue
        unsigned long flag, numScans, intTime, pixMode,baseLineMSW,baseLineLSW;
        convertBytesToLong(&buf[3], 2, &flag);
        convertBytesToLong(&buf[5], 2, &numScans);
        convertBytesToLong(&buf[7], 2, &intTime);
        convertBytesToLong(&buf[9], 2, &baseLineMSW);
        convertBytesToLong(&buf[11], 2, &baseLineLSW);
        convertBytesToLong(&buf[13], 2, &pixMode);
        
        //for (int i = 0; i < 15; i ++) {
        //     printf("%u 0x%X\n", i,buf[i]);
        //}
        
        head->dataSize = (int)flag;
        head->numScans = (int)numScans;
        head->intTime = (long)intTime;
        head->baselineMSW = (int)baseLineMSW;
        head->baselineLSW = (int)baseLineLSW;
        head->pixelMode = (int)pixMode;
    }
    else {
        return 1;
    }
    return 0;
}

int receiveData(unsigned char buf[], int bufferSize, int serialPort, long timeout){
    ssize_t n = 0;
    fd_set rfds;
    
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
            n = read(serialPort, buf, bufferSize);
        
        
            if(n < 0) {
                return -1;
            } else if (n == 0) {

                #ifdef DEBUGGING
                printf(".");
                #endif
            } else {
                #ifdef DEBUGGING
                printf("%u char fnd\n",n);
                for (int i = 0; i < 10; i++) {
                    printf("%u\t0x%X\n",i, buf[i]);
                }
                #endif
            
                return (int)n;
            }
            

        }
        usleep(1000);
        timeout--;
        if (timeout < 0){
            break;
        }

    }
     #ifdef DEBUGGING

    printf("read error: %u\n", n);
    #endif
    return n;
}


int receivePacket(unsigned char buffer[], int bufferSize, int serialPort, long timeout){
    
    int error = 0;
    
    /*
     unsigned char b;  // read expects an array, so we give it a 1-byte array
     int i=0;
     rxTxInProgress = true;
     do {
     ssize_t n = read(serialPort, &b, 1);  // read a char at a time
     if( n==-1) return -1;    // couldn't read
     if( n==0 ) {
     usleep( 1 * 1000 );  // wait 1 msec try again
     timeout--;
     continue;
     }
     buffer[i] = b;
     i++;
     
     } while(i < bufferSize && timeout>0 );
     
     if (timeout <= 0) {
     printf("Timeout while receiving bytes\n");
     }
     
     rxTxInProgress = false;
     return error;
     */
    
    unsigned char buf[256];
    while(1) {
        ssize_t n = read(serialPort, buf, 256);
        if(n < 0) {
            break;
        } else if (n == 0) {
            usleep(1000);
            
        } else {
        
            printf("%d\n", n);
            for(int i = 0; i < n; i++) {
                printf("%d ", buf[i]);
            }
            printf("\n");
        }
        
    }
}



int flushBuffer(int serialPort){
    ssize_t reads;
    unsigned char c[512];
    
    while (1) {
        reads = receiveData(c, 512, serialPort, 2);
        if (reads == 0) {
            break;
        }
        else {
#ifdef DEBUGGING
            printf("Flush %u\n",(int)reads);
#endif
        }
    }
    return (int)reads;
}


// ---Processing Routines---
int processPacket(char inPacket[], unsigned long specData[]) {
    int error = 0;
    /*
     WORD 0xFFFF – start of spectrum
     WORD Data size flag (0=Data is WORD’s, 1=Data is DWORD’s)
     WORD Number of Scans Accumulated
     WORD Integration time in milliseconds
     WORD FPGA Established Baseline value (MSW)
     WORD FPGA Established Baseline value (MSW)
     WORD pixel mode
     WORDs if pixel mode not 0, indicates parameters passed to the Pixel Mode command (P) (D)WORDs spectral data depending on Data size flag
     WORD 0xFFFD – end of spectrum
     
     word : MSB first then LSB
     */
    
    /* Pixel Mode
     0 = all 2048 pixels
     1 = every nth pixel with no averaging
     2 = N/A
     3 = pixel x through y every n pixels
     4 = up to 10 randomly selected pixels between 0 and 2047 (denoted p1, p2, ... p10)
     */
    
    unsigned long flag, numScans, intTime, pixMode,baseLineMSW,baseLineLSW;
    convertBytesToLong(&inPacket[2], 2, &flag);
    convertBytesToLong(&inPacket[4], 2, &numScans);
    convertBytesToLong(&inPacket[6], 2, &intTime);
    convertBytesToLong(&inPacket[8], 2, &baseLineMSW);
    convertBytesToLong(&inPacket[10], 2, &baseLineLSW);
    convertBytesToLong(&inPacket[12], 2, &pixMode);
    
    int numBytes;
    if (flag == 0) {
        //We have WORD size data
        numBytes = 2;
    } else {
        //We have DWORD size spec data
        numBytes = 4;
    }
    if (pixMode == 0) {
        //This means spectral data starts at index 14 for numScans * datasize
        for (int i = 0; i < numScans; i++) {
            convertBytesToLong(&inPacket[i * numBytes + 14], numBytes, &specData[i]);
        }
    }
    
    
    
    return error;
}


int convertBytesToLong(unsigned char inPacket[], int numBytes, unsigned long *returnValue) {
    if (numBytes == 4) {
        //We have a DWORD - 32bit unsigned integer
        *returnValue = ((unsigned long)inPacket[3]) + (((unsigned long)inPacket[2]) << 8) + (((unsigned long)inPacket[1]) << 16) + (((unsigned long)inPacket[0]) << 24);
    } else {
        //We have a WORD - 16bit unsigned integer
        *returnValue = ((unsigned long)inPacket[1]) + (((unsigned long)inPacket[0]) << 8);
    }
    return 0;
}


int split4Byte(char packet[], long number){
    //want to take the number and split it into a 4 byte packet with LSB first
    packet[0] = (char)(floorl(number/(pow(256,3))));
    number = number - packet[0] * (pow(256,3));
    
    packet[1] = (char)(floorl(number/(pow(256,2))));
    number = number - packet[1] * (pow(256,2));
    
    packet[2] = (char)(floorl(number/(256)));
    number = number - packet[2] * (256);
    
    packet[3] = (char)(floorl(number));
    
    return 0;
}

int timeDelayCalc(int baud, int numbytes) {
    //This calculates how long a delay should be for RX TX of a packet
    //baud is in bits/sec, there is a stop and a start bit so divide by 10
    //result is in milli sec
    // + 1 at end for rounding down due to int maths
    return (numbytes * 1000) / (baud/10) + 1;
    
    
}

long findMax(unsigned long result[], unsigned long array[], int arraySize){
    //result[0] is the index of the max result
    //result[1] is the value of that max result.
    result[1] = array[0];
    result[0] = 0;
    for (int i = 0; i < arraySize; i++) {
        if (array[i] > result[1]) {
            //we have found a larger value
            result[0] = i;
            result[1] = array[i];
        }
    }
    return result[0];
}

float findMean(unsigned long array[], int arraySize, int startIdx) {
    float mean = 0;
    for (int i = startIdx; i < (startIdx + arraySize); i++) {
        mean += (array[i] / arraySize);
    }
    return mean;
}


long checkIntTime(unsigned long specData[], int arraySize, long intTime, unsigned long thresholds[] ){
    //This routine searches through a spectra array and determines if it has maxed out or the signal is too small, it then does a calculation to determine a better int time.
    //Thresholds: [0] = Max threshold, [1] = Min Threshold, [2] = desired mean.
    unsigned long max[2];
    unsigned long newIntTime = intTime;
    findMax(max, specData, arraySize);
    _Bool saturated = false;
    _Bool tooLow = false;
    
    //Check for over the max threshold
    if (max[1] >= thresholds[0]) {
        saturated = true;
        
        //For 400-700nm on the USB2000+ index = 172 -> 1063
        float mean = findMean(specData, 891, 172);
        float gain = thresholds[2] / mean;
        newIntTime = intTime * gain;
        
    }
    //check for less than min threshold
    if (max[1] <= thresholds[1]) {
        tooLow = true;
        //For 400-700nm on the USB2000+ index = 172 -> 1063
        float mean = findMean(specData, 891, 172);
        float gain = thresholds[2] / mean;
        if (gain > 20) {
            //it is going to ramp the gain up too much and it will bounce around, so put a threshold on it.
            gain = 5;
        }
        newIntTime = intTime * gain;
    }
    
    return newIntTime;
}
