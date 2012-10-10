/*****************************************************************************
 *  Copyright (c) 2009, OpenJAUS.com
 *  All rights reserved.
 *  
 *  This file is part of OpenJAUS.  OpenJAUS is distributed under the BSD 
 *  license.  See the LICENSE file for details.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the University of Florida nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File Name: setSignalsMessage.c
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo (galluzzo AT gmail DOT com)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the functionality of a SetSignalsMessage

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jaus/jaus.h>
#include "setSignalsMessage.h"

static const int commandCode = JAUS_SET_SIGNALS;
static const int maxDataSizeBytes = 9;

static JausBoolean headerFromBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static JausBoolean headerToBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int headerToString(SetSignalsMessage message, char **buf);

static JausBoolean dataFromBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int dataToBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static void dataInitialize(SetSignalsMessage message);
static void dataDestroy(SetSignalsMessage message);
static unsigned int dataSize(SetSignalsMessage message);

// ************************************************************************************************************** //
//                                    USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

// Initializes the message-specific fields
static void dataInitialize(SetSignalsMessage message)
{
	// Set initial values of message fields
    message->version = newJausByte (0);
    message->presenceVector = newJausByte (JAUS_BYTE_PRESENCE_VECTOR_ALL_ON);

    message->turnSignalStatus = newJausByte (0);

    message->hornStatus = JAUS_FALSE;
    message->hornPeriodOn = newJausDouble (0);
    message->hornPeriodOff = newJausDouble (0);

    message->headlightsStatus = newJausByte (0);
    message->highbeamsStatus = JAUS_FALSE;
    message->foglightsStatus = JAUS_FALSE;
    message->lightsPeriodOn = newJausDouble (0);
    message->lightsPeriodOff = newJausDouble (0);
}

// Destructs the message-specific fields
static void dataDestroy(SetSignalsMessage message)
{
	// Free message fields
    // No dynamic
}

// Return boolean of success
static JausBoolean dataFromBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	int index = 0;
    JausByte tmp_byte;
	
    // Unpack Message Fields from Buffer
    // unpack according to presence vector
    
    // Version
    if (!jausByteFromBuffer (&message->version, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
    index += JAUS_BYTE_SIZE_BYTES;

    // Presence Vector
    if (!jausByteFromBuffer (&message->presenceVector, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
    index += JAUS_BYTE_SIZE_BYTES;

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_TURN_SIGNALS_BIT)) {
        // unpack

        if (!jausByteFromBuffer (&message->turnSignalStatus, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;

    }

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_HORN_BIT)) {
        // unpack
        // status
        if (!jausByteFromBuffer (&tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
        message->hornStatus = (tmp_byte == 1) ? JAUS_TRUE : JAUS_FALSE;

        // period on
        if (!jausByteFromBuffer (&tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
        message->hornPeriodOn = jausByteToDouble (tmp_byte, 0, 10);

        // period off
        if (!jausByteFromBuffer (&tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
        message->hornPeriodOff = jausByteToDouble (tmp_byte, 0, 10);

    }

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_HEADLIGHTS_BIT)) {
        // unpack
        // status
        if (!jausByteFromBuffer (&tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;

        message->headlightsStatus = newJausByte (0);
        if (jausByteIsBitSet (tmp_byte, 0))
            jausByteSetBit (&message->headlightsStatus, 0);
        if (jausByteIsBitSet (tmp_byte, 1))
            jausByteSetBit (&message->headlightsStatus, 1);
        if (jausByteIsBitSet (tmp_byte, 2))
            jausByteSetBit (&message->headlightsStatus, 2);

        message->highbeamsStatus = jausByteIsBitSet (tmp_byte, JAUS_SIGNALS_HEADLIGHTS_BF_HIGHBEAMS_BIT);
        message->foglightsStatus = jausByteIsBitSet (tmp_byte, JAUS_SIGNALS_HEADLIGHTS_BF_FOGLIGHTS_BIT);

        // period on
        if (!jausByteFromBuffer (&tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
        message->lightsPeriodOn = jausByteToDouble (tmp_byte, 0, 10);

        // period off
        if (!jausByteFromBuffer (&tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
        message->lightsPeriodOff = jausByteToDouble (tmp_byte, 0, 10);

    }


    return JAUS_TRUE;
}

// Returns number of bytes put into the buffer
static int dataToBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	int index = 0;
    JausByte tmp_byte;

    // Pack Message Fields to Buffer
    // pack according to presence vector

    // Version
    if (!jausByteToBuffer (message->version, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
    index += JAUS_BYTE_SIZE_BYTES;

    // Presence Vector
    if (!jausByteToBuffer (message->presenceVector, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
    index += JAUS_BYTE_SIZE_BYTES;

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_TURN_SIGNALS_BIT)) {
        if (!jausByteToBuffer (message->turnSignalStatus, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
    }

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_HORN_BIT)) {
        // status
        tmp_byte = (message->hornStatus) ? newJausByte (1) : newJausByte (0);

        if (!jausByteToBuffer (tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;

        // period on
        tmp_byte = jausByteFromDouble (message->hornPeriodOn, 0, 10);
        if (!jausByteToBuffer (tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;

        // period off
        tmp_byte = jausByteFromDouble (message->hornPeriodOff, 0, 10);
        if (!jausByteToBuffer (tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
    }

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_HEADLIGHTS_BIT)) {
        // status
        tmp_byte = message->headlightsStatus;
        if (message->highbeamsStatus) jausByteSetBit (&tmp_byte, JAUS_SIGNALS_HEADLIGHTS_BF_HIGHBEAMS_BIT);
        if (message->foglightsStatus) jausByteSetBit (&tmp_byte, JAUS_SIGNALS_HEADLIGHTS_BF_FOGLIGHTS_BIT);
        if (!jausByteToBuffer (tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;

        // period on
        tmp_byte = jausByteFromDouble (message->lightsPeriodOn, 0, 10);
        if (!jausByteToBuffer (tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;

        // period off
        tmp_byte = jausByteFromDouble (message->lightsPeriodOff, 0, 10);
        if (!jausByteToBuffer (tmp_byte, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
        index += JAUS_BYTE_SIZE_BYTES;
    }

	return index;
}

static int dataToString(SetSignalsMessage message, char **buf)
{
  //message already verified 

  //Setup temporary string buffer
  
  //Fill in maximum size of output string
  unsigned int bufSize = 800;
  (*buf) = (char*)malloc(sizeof(char)*bufSize);
  
  strcpy((*buf), "\nVersion: ");
  jausByteToHexString (message->version, (*buf)+strlen(*buf));

  strcpy((*buf), "\nPresence Vector: ");
  jausByteToHexString (message->presenceVector, (*buf)+strlen(*buf));

  strcpy((*buf), "\nTurn Signal Status: ");
  jausByteToString (message->turnSignalStatus, (*buf)+strlen(*buf));

  strcpy((*buf), "\nHorn Status: ");
  if (message->hornStatus) strcpy((*buf), "ON");
  else strcpy((*buf), "OFF");

  strcpy((*buf), "\nHorn Period On: ");
  jausDoubleToString (message->hornPeriodOn, (*buf)+strlen(*buf));

  strcpy((*buf), "\nHorn Period Off: ");
  jausDoubleToString (message->hornPeriodOff, (*buf)+strlen(*buf));

  strcpy((*buf), "\nHeadlights Status: ");
  jausByteToString (message->headlightsStatus, (*buf)+strlen(*buf));

  if (message->highbeamsStatus) strcpy((*buf), "ON");
  else strcpy((*buf), "OFF");

  strcpy((*buf), "\nFoglights Status: ");
  if (message->foglightsStatus) strcpy((*buf), "ON");
  else strcpy((*buf), "OFF");

  strcpy((*buf), "\nLights Period On: ");
  jausDoubleToString (message->lightsPeriodOn, (*buf)+strlen(*buf));

  strcpy((*buf), "\nLights Period Off: ");
  jausDoubleToString (message->lightsPeriodOff, (*buf)+strlen(*buf));
  
  return (int)strlen((*buf));
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(SetSignalsMessage message)
{
    int size = 0;
    size += 2 * JAUS_BYTE_SIZE_BYTES;

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_TURN_SIGNALS_BIT))
        size += JAUS_BYTE_SIZE_BYTES;

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_HORN_BIT))
        size += 3*JAUS_BYTE_SIZE_BYTES;

    if (jausByteIsBitSet (message->presenceVector, JAUS_SIGNALS_PV_HEADLIGHTS_BIT))
        size += 3*JAUS_BYTE_SIZE_BYTES;

    return size;
}

// ************************************************************************************************************** //
//                                    NON-USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

SetSignalsMessage setSignalsMessageCreate(void)
{
	SetSignalsMessage message;

	message = (SetSignalsMessage)malloc( sizeof(SetSignalsMessageStruct) );
	if(message == NULL)
	{
		return NULL;
	}
	
	// Initialize Values
	message->properties.priority = JAUS_DEFAULT_PRIORITY;
	message->properties.ackNak = JAUS_ACK_NAK_NOT_REQUIRED;
	message->properties.scFlag = JAUS_NOT_SERVICE_CONNECTION_MESSAGE;
	message->properties.expFlag = JAUS_NOT_EXPERIMENTAL_MESSAGE;
	message->properties.version = JAUS_VERSION_3_3;
	message->properties.reserved = 0;
	message->commandCode = commandCode;
	message->destination = jausAddressCreate();
	message->source = jausAddressCreate();
	message->dataFlag = JAUS_SINGLE_DATA_PACKET;
	message->dataSize = maxDataSizeBytes;
	message->sequenceNumber = 0;
	
	dataInitialize(message);
	message->dataSize = dataSize(message);
	
	return message;	
}

void setSignalsMessageDestroy(SetSignalsMessage message)
{
	dataDestroy(message);
	jausAddressDestroy(message->source);
	jausAddressDestroy(message->destination);
	free(message);
}

JausBoolean setSignalsMessageFromBuffer(SetSignalsMessage message, unsigned char* buffer, unsigned int bufferSizeBytes)
{
	int index = 0;
	
	if(headerFromBuffer(message, buffer+index, bufferSizeBytes-index))
	{
		index += JAUS_HEADER_SIZE_BYTES;
		if(dataFromBuffer(message, buffer+index, bufferSizeBytes-index))
		{
			return JAUS_TRUE;
		}
		else
		{
			return JAUS_FALSE;
		}
	}
	else
	{
		return JAUS_FALSE;
	}
}

JausBoolean setSignalsMessageToBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	if(bufferSizeBytes < setSignalsMessageSize(message))
	{
		return JAUS_FALSE; //improper size	
	}
	else
	{	
		message->dataSize = dataToBuffer(message, buffer+JAUS_HEADER_SIZE_BYTES, bufferSizeBytes - JAUS_HEADER_SIZE_BYTES);
		if(headerToBuffer(message, buffer, bufferSizeBytes))
		{
			return JAUS_TRUE;
		}
		else
		{
			return JAUS_FALSE; // headerToSetSignalsBuffer failed
		}
	}
}

SetSignalsMessage setSignalsMessageFromJausMessage(JausMessage jausMessage)
{
	SetSignalsMessage message;
	
	if(jausMessage->commandCode != commandCode)
	{
		return NULL; // Wrong message type
	}
	else
	{
		message = (SetSignalsMessage)malloc( sizeof(SetSignalsMessageStruct) );
		if(message == NULL)
		{
			return NULL;
		}
		
		message->properties.priority = jausMessage->properties.priority;
		message->properties.ackNak = jausMessage->properties.ackNak;
		message->properties.scFlag = jausMessage->properties.scFlag;
		message->properties.expFlag = jausMessage->properties.expFlag;
		message->properties.version = jausMessage->properties.version;
		message->properties.reserved = jausMessage->properties.reserved;
		message->commandCode = jausMessage->commandCode;
		message->destination = jausAddressCreate();
		*message->destination = *jausMessage->destination;
		message->source = jausAddressCreate();
		*message->source = *jausMessage->source;
		message->dataSize = jausMessage->dataSize;
		message->dataFlag = jausMessage->dataFlag;
		message->sequenceNumber = jausMessage->sequenceNumber;
		
		// Unpack jausMessage->data
		if(dataFromBuffer(message, jausMessage->data, jausMessage->dataSize))
		{
			return message;
		}
		else
		{
			return NULL;
		}
	}
}

JausMessage setSignalsMessageToJausMessage(SetSignalsMessage message)
{
	JausMessage jausMessage;
	
	jausMessage = (JausMessage)malloc( sizeof(struct JausMessageStruct) );
	if(jausMessage == NULL)
	{
		return NULL;
	}	
	
	jausMessage->properties.priority = message->properties.priority;
	jausMessage->properties.ackNak = message->properties.ackNak;
	jausMessage->properties.scFlag = message->properties.scFlag;
	jausMessage->properties.expFlag = message->properties.expFlag;
	jausMessage->properties.version = message->properties.version;
	jausMessage->properties.reserved = message->properties.reserved;
	jausMessage->commandCode = message->commandCode;
	jausMessage->destination = jausAddressCreate();
	*jausMessage->destination = *message->destination;
	jausMessage->source = jausAddressCreate();
	*jausMessage->source = *message->source;
	jausMessage->dataSize = dataSize(message);
	jausMessage->dataFlag = message->dataFlag;
	jausMessage->sequenceNumber = message->sequenceNumber;
	
	jausMessage->data = (unsigned char *)malloc(jausMessage->dataSize);
	jausMessage->dataSize = dataToBuffer(message, jausMessage->data, jausMessage->dataSize);
		
	return jausMessage;
}

unsigned int setSignalsMessageSize(SetSignalsMessage message)
{
	return (unsigned int)(dataSize(message) + JAUS_HEADER_SIZE_BYTES);
}

char* setSignalsMessageToString(SetSignalsMessage message)
{
  if(message)
  {
    char* buf1 = NULL;
    char* buf2 = NULL;
    
    int returnVal;
    
    //Print the message header to the string buffer
    returnVal = headerToString(message, &buf1);
    
    //Print the message data fields to the string buffer
    returnVal += dataToString(message, &buf2);
    
    char* buf;
    buf = (char*)malloc(strlen(buf1)+strlen(buf2));
    strcpy(buf, buf1);
    strcat(buf, buf2);
    
    free(buf1);
    free(buf2);
    
    return buf;
  }
  else
  {
    char* buf = "Invalid SetSignals Message";
    char* msg = (char*)malloc(strlen(buf)+1);
    strcpy(msg, buf);
    return msg;
  }
}
//********************* PRIVATE HEADER FUNCTIONS **********************//

static JausBoolean headerFromBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	if(bufferSizeBytes < JAUS_HEADER_SIZE_BYTES)
	{
		return JAUS_FALSE;
	}
	else
	{
		// unpack header
		message->properties.priority = (buffer[0] & 0x0F);
		message->properties.ackNak	 = ((buffer[0] >> 4) & 0x03);
		message->properties.scFlag	 = ((buffer[0] >> 6) & 0x01);
		message->properties.expFlag	 = ((buffer[0] >> 7) & 0x01);
		message->properties.version	 = (buffer[1] & 0x3F);
		message->properties.reserved = ((buffer[1] >> 6) & 0x03);
		
		message->commandCode = buffer[2] + (buffer[3] << 8);
	
		message->destination->instance = buffer[4];
		message->destination->component = buffer[5];
		message->destination->node = buffer[6];
		message->destination->subsystem = buffer[7];
	
		message->source->instance = buffer[8];
		message->source->component = buffer[9];
		message->source->node = buffer[10];
		message->source->subsystem = buffer[11];
		
		message->dataSize = buffer[12] + ((buffer[13] & 0x0F) << 8);

		message->dataFlag = ((buffer[13] >> 4) & 0x0F);

		message->sequenceNumber = buffer[14] + (buffer[15] << 8);
		
		return JAUS_TRUE;
	}
}

static JausBoolean headerToBuffer(SetSignalsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	JausUnsignedShort *propertiesPtr = (JausUnsignedShort*)&message->properties;
	
	if(bufferSizeBytes < JAUS_HEADER_SIZE_BYTES)
	{
		return JAUS_FALSE;
	}
	else
	{	
		buffer[0] = (unsigned char)(*propertiesPtr & 0xFF);
		buffer[1] = (unsigned char)((*propertiesPtr & 0xFF00) >> 8);

		buffer[2] = (unsigned char)(message->commandCode & 0xFF);
		buffer[3] = (unsigned char)((message->commandCode & 0xFF00) >> 8);

		buffer[4] = (unsigned char)(message->destination->instance & 0xFF);
		buffer[5] = (unsigned char)(message->destination->component & 0xFF);
		buffer[6] = (unsigned char)(message->destination->node & 0xFF);
		buffer[7] = (unsigned char)(message->destination->subsystem & 0xFF);

		buffer[8] = (unsigned char)(message->source->instance & 0xFF);
		buffer[9] = (unsigned char)(message->source->component & 0xFF);
		buffer[10] = (unsigned char)(message->source->node & 0xFF);
		buffer[11] = (unsigned char)(message->source->subsystem & 0xFF);
		
		buffer[12] = (unsigned char)(message->dataSize & 0xFF);
		buffer[13] = (unsigned char)((message->dataFlag & 0xFF) << 4) | (unsigned char)((message->dataSize & 0x0F00) >> 8);

		buffer[14] = (unsigned char)(message->sequenceNumber & 0xFF);
		buffer[15] = (unsigned char)((message->sequenceNumber & 0xFF00) >> 8);
		
		return JAUS_TRUE;
	}
}

static int headerToString(SetSignalsMessage message, char **buf)
{
  //message existance already verified 

  //Setup temporary string buffer
  
  unsigned int bufSize = 500;
  (*buf) = (char*)malloc(sizeof(char)*bufSize);
  
  strcpy((*buf), jausCommandCodeString(message->commandCode) );
  strcat((*buf), " (0x");
  sprintf((*buf)+strlen(*buf), "%04X", message->commandCode);

  strcat((*buf), ")\nReserved: ");
  jausUnsignedShortToString(message->properties.reserved, (*buf)+strlen(*buf));

  strcat((*buf), "\nVersion: ");
  switch(message->properties.version)
  {
    case 0:
      strcat((*buf), "2.0 and 2.1 compatible");
      break;
    case 1:
      strcat((*buf), "3.0 through 3.1 compatible");
      break;
    case 2:
      strcat((*buf), "3.2 and 3.3 compatible");
      break;
    default:
      strcat((*buf), "Reserved for Future: ");
      jausUnsignedShortToString(message->properties.version, (*buf)+strlen(*buf));
      break;
  }

  strcat((*buf), "\nExp. Flag: ");
  if(message->properties.expFlag == 0)
    strcat((*buf), "Not Experimental");
  else 
    strcat((*buf), "Experimental");
  
  strcat((*buf), "\nSC Flag: ");
  if(message->properties.scFlag == 1)
    strcat((*buf), "Service Connection");
  else
    strcat((*buf), "Not Service Connection");
  
  strcat((*buf), "\nACK/NAK: ");
  switch(message->properties.ackNak)
  {
  case 0:
    strcat((*buf), "None");
    break;
  case 1:
    strcat((*buf), "Request ack/nak");
    break;
  case 2:
    strcat((*buf), "nak response");
    break;
  case 3:
    strcat((*buf), "ack response");
    break;
  default:
    break;
  }
  
  strcat((*buf), "\nPriority: ");
  if(message->properties.priority < 12)
  {
    strcat((*buf), "Normal Priority ");
    jausUnsignedShortToString(message->properties.priority, (*buf)+strlen(*buf));
  }
  else
  {
    strcat((*buf), "Safety Critical Priority ");
    jausUnsignedShortToString(message->properties.priority, (*buf)+strlen(*buf));
  }
  
  strcat((*buf), "\nSource: ");
  jausAddressToString(message->source, (*buf)+strlen(*buf));
  
  strcat((*buf), "\nDestination: ");
  jausAddressToString(message->destination, (*buf)+strlen(*buf));
  
  strcat((*buf), "\nData Size: ");
  jausUnsignedIntegerToString(message->dataSize, (*buf)+strlen(*buf));
  
  strcat((*buf), "\nData Flag: ");
  jausUnsignedIntegerToString(message->dataFlag, (*buf)+strlen(*buf));
  switch(message->dataFlag)
  {
    case 0:
      strcat((*buf), " Only data packet in single-packet stream");
      break;
    case 1:
      strcat((*buf), " First data packet in muti-packet stream");
      break;
    case 2:
      strcat((*buf), " Normal data packet");
      break;
    case 4:
      strcat((*buf), " Retransmitted data packet");
      break;
    case 8:
      strcat((*buf), " Last data packet in stream");
      break;
    default:
      strcat((*buf), " Unrecognized data flag code");
      break;
  }
  
  strcat((*buf), "\nSequence Number: ");
  jausUnsignedShortToString(message->sequenceNumber, (*buf)+strlen(*buf));
  
  return strlen((*buf));
  

}
