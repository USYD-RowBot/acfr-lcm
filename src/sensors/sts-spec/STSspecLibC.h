// Library for interfacing with the STS microspectrometer from Ocean Optics
//  STSspecLibC.h
//  Written for C
//
//  Created by Daniel Bongiorno on 21/08/13.
//  Copyright (c) 2013 Daniel Bongiorno. All rights reserved.
//

#ifndef Learning_C_STSspecLibC_h
#define Learning_C_STSspecLibC_h



#endif

_Bool rxTxInProgress;


// ----Get/Set From Spectrometer---

int getSpectra(int serialPort, unsigned short spectra[]);
/* will return a 1024 long array of spectral readings, needs to be nonlinearity corrected */
int getWaveCoeff(int serialPort, float coeff[]);
int getNonlinCoeff(int serialPort, float coeff[]);
int getTemp(int serialPort, float temps[]);
/* Returns a 2 element float array of detector and board temperatures in celcius respectively. */
int getSerialNum(int serialPort, char serialNum[]);
int setIntTime(int serialPort, long intTime);
int collectSpectra(int serialPort );
// Do not use at the moment not working.

// ----Serial Routines---
//int initSerial(char portAddress[],int baud);
/* Initialise serial port to portAddress eg: '/dev/tty.PL2303000' and baud, STS will take upto 115200 */

int sendReceivePacket(int serialPort, char packetToSend[],unsigned long returnData[], float returnFloat[], double timeout);
int sendPacket(int serialPort, char packetToSend[]);
int receivePacket(unsigned char buffer[], int bufferSize, int serialPort, double timeout, char foundPacket[]);
int flushBuffer(int serialPort);

// ---Processing Routines---
int findPacket(char buffer[], int bufferSize, char foundPacket[]);
int makePacket(int packetType, long variables, char outputPacket[]);
int processPacket(char inPacket[], unsigned short returnData[], float floatData[]);
int zeroFiller(int startIndex, int numZeros, char packet[]);
int split4Byte(char packet[], long number);
int timeDelayCalc(int baud, int numbytes);
long findMax(unsigned long result[], unsigned long array[], int arraySize);
float findMean(unsigned long array[], int arraySize, int startIdx);
long checkIntTime(unsigned long specData[], int arraySize, long intTime, unsigned long thresholds[] );
