//
//  USB2000Lib.h
//  Learning C
//
//  Created by Daniel Bongiorno on 4/09/13.
//  Copyright (c) 2013 Daniel Bongiorno. All rights reserved.
//

#ifndef Learning_C_USB2000Lib_h
#define Learning_C_USB2000Lib_h
#endif
typedef struct {
    int dataSize;
    int numScans;
    long intTime;
    int baselineMSW;
    int baselineLSW;
    int pixelMode;
    _Bool acqSuccess;
} specHeader;




// ----Get/Set From Spectrometer---
int getSpectra(int serialPort, unsigned long spectra[]);
int setBaud(int serialPort, char portAddress[], int baudChoice);
/* Set Baud Rate
 0  =   2400
 1  =   4800
 2  =   9600
 3  =   19200
 4  =   38400
 5  =   Not supported
 6  =   115200
 */

int setIntTime(int serialPort, long intTime);
int setTriggerMode(int serialPort, int triggerMode);
int setDataMode(int serialPort, _Bool binaryMode);


// ----Serial Routines---
int initSerial(char portAddress[],int baud);
/* Initialise serial port to portAddress eg: '/dev/tty.PL2303000' and baud, USB2000+ will take upto 115200 */

int sendReceivePacket(int serialPort, char packetToSend[], int outPacketSize, unsigned long returnData[], int timeout);
int sendReceiveChar(int serialPort, char packetToSend[], int outPacketSize, char returnChar[], int timeout);
int sendPacket(int serialPort, char packetToSend[], int outPacketSize);
int readHeaderPacket(specHeader* head, int serialPort);
int receiveData(unsigned char buf[], int bufferSize, int serialPort, long timeout);
int receivePacket(unsigned char buffer[], int bufferSize, int serialPort, long timeout);
int flushBuffer(int serialPort);


// ---Processing Routines---
int processPacket(char inPacket[], unsigned long specData[]);
int convertBytesToLong(unsigned char inPacket[], int numBytes, unsigned long *returnValue);
int split4Byte(char packet[], long number);
int timeDelayCalc(int baud, int numbytes);
unsigned long findMax(unsigned long result[], unsigned long array[], int arraySize);
float findMean(unsigned long array[], int arraySize, int startIdx);
long checkIntTime(unsigned long specData[], int arraySize, long intTime, unsigned long thresholds[] );
