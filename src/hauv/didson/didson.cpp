/**
 * @file didson.cpp
 * @brief DIDSON-LCM interface
 * @author Michael Kaess
 * @version $Id: didson.cpp 3737 2011-01-27 18:56:56Z ayoungk $
 */

#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <resolv.h>
#include <unistd.h>

#include <sys/time.h>
#include <string.h>
#include <inttypes.h>

#include "perls-lcmtypes/hauv_didson_t.h"
#include "perls-lcmtypes/hauv_didson_raw_t.h"

#include <assert.h>

// HULS3 uses different port and packet sizes than the HAUV1b
const bool HULS3 = true;


uint64_t timeNow() {
  struct timeval t;
  gettimeofday(&t, NULL);
  uint64_t res = (uint64_t)t.tv_sec * (uint64_t)1000000 + (uint64_t)t.tv_usec;
  return res;
}

lcm_t * lcm;

static void assemble_frames(const hauv_didson_raw_t* msg) {
  static hauv_didson_t frame;
  static int count = 0;
  static bool valid = false;
  static int frame_number = 0;

  assert(sizeof(unsigned short int)==2);
  unsigned short int *data = (unsigned short int *)msg->data;

  //  printf("size data %x\n", sizeof(msg->data));
  //  printf("%x %x %x %x\n", data[0], data[1], data[2], data[3]);

  bool processed = false;
  if (data[0] == 0x5a02) {

    if (!HULS3) {

      if (data[2] == 0x01) {
        if (count == 0) {
          count = 1;

          //        memcpy(&frame, &data[4], 1024);
          memcpy(&frame, &data[2], 1024); // image data seemed to have been shifted  5/12/2010

          memcpy(&frame.m_cData[0], &data[4+1024/2], 24576);
        } else {
          printf("-----WARNING: frame 0 expected\n");
        }
      } else if (data[2] == 0x22) {
        if (count == 1) {
          count = 0;
          memcpy(&frame.m_cData[24576], &data[4], 24576);
          hauv_didson_t_publish (lcm, "HAUV_DIDSON_FRAME", &frame);
          printf("sent frame %i\n", frame_number);
          frame_number++;
        } else {
          printf("-----WARNING: frame 1 expected\n");
        }
      }

    } else {

      if (data[2] == 0x10 || data[2] == 0x30) {
        // data packet
        if (data[3]!=count) {
          printf("----------------------WARNING: count wrong after %i\n", count);
          count = data[3];
          valid = false;
        }
        if (valid) {
          assert(msg->length == HULS3 ? 8200 : 1032);
          if (count>48) {
            printf("ERROR: count>48\n");
          } else {
            memcpy(&frame.m_cData[count*(HULS3 ? 8096 : 1024)], &data[4], HULS3 ? 8096 : 1024);
          }
        }
        count++;
        if (data[2] == 0x30) {
          // last packet
          if (valid) {
            hauv_didson_t_publish (lcm, "HAUV_DIDSON_FRAME", &frame); // todo: rename all...
            printf("sent frame (%i assembled)\n", count);
          }
        }
        processed = true;
      } else if (data[2] == 0x08 || data[2] == 0x48) {
        // frame header
//        memcpy(&frame, &data[4], 1024);
// Packet shifted
        memcpy(&frame, &data[2], 1024);
        // the header packet is used as indicator for when the data was requested
        frame.time_first_packet = msg->time_received;
        // reset counter
        count = 0;
        valid = true;
        processed = true;
      }
    }

  }
  if (!processed) {
    //    printf("%x %x %x %x\n", data[0], data[1], data[2], data[3]);
  }
}

int main() {
  lcm = lcm_create (NULL);
  //lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
  //lcm = lcm_create ("udpm://");

  if (!lcm)
    return 1;

  int sd;
  struct sockaddr_in addr;
  struct sockaddr from;
  unsigned char data[1024*30];

  sd = socket(PF_INET, SOCK_DGRAM, 0);
  bzero(&addr, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(HULS3 ? 50700 : 700);
  inet_aton("", &addr.sin_addr);
  printf("connecting...\n");

  int bol = 1;
  setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &bol, 4);
  setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &bol, 4);

  if (bind(sd, (struct sockaddr *)&addr,
           sizeof(struct sockaddr)) == -1) {
    perror("bind");
    exit(1);
  }

printf("connected\n");
  while(1) {
    socklen_t length = sizeof(from);
    ssize_t n = recvfrom(sd, data, sizeof(data), 0, &from, &length);
    uint64_t time_received = timeNow();

    hauv_didson_raw_t didson;
    didson.time_received = time_received;
    didson.length = n;
    didson.data = (uint8_t*)&data;
    // not needed anymore    hauv_didson_raw_t_publish (lcm, "HAUV_DIDSON_RAW", &didson);
    assemble_frames(&didson);
  }

  close(sd);
}

// from DIDSON code - included here for reference

typedef unsigned int UINT;
typedef unsigned char BYTE;
typedef unsigned long long CTime;
typedef bool BOOL;

const int DATA_SIZE = 49152;
const int FRAME_PAD_LENGTH = 604;

#pragma pack (1)
struct FrameHeader
{
	UINT m_nFrameNumber;
	CTime m_tFrameTime; // client PC time (8 bytes, was 4 bytes prior to V4.00)
	UINT m_nVersion; // keep unique version for frame data (sonar data format)
	// At this time not used. Any change in frame header
	// generates an increment in file version
	UINT m_nStatus; // feedback on client commands
	UINT m_nYear; // sonar CPU year ex: 2002
	UINT m_nMonth; // 1-12
	UINT m_nDay; // 1-31
	UINT m_nHour; // 0-23
	UINT m_nMinute; // 0-59
	UINT m_nSecond; // 0-59
	UINT m_nHSecond; // 0-99
	UINT m_nTransmitMode; // bit1 = ENABLE, bit0 = HF_MODE
	UINT m_nWindowStart; // 1-31 CW meters = N*Incr, Incr = .375 (HF) or 0.75 (LF)
	// XW meters = N*Incr, Incr = .42 (HF) or 0.84 (LF)
	UINT m_nWindowLength; // 0-3 CW (1.125, 2.25, 4.5, 9 m) HF or (4.5, 9, 18, 36 m) LF
	// Std XW (1.25, 2.5, 5, 10 m) HF or (5, 10, 20, 40 m) LF
	// LR XW (2.5, 5, 10, 20 m) HF or (10, 20, 40, 80 m) LF
	UINT m_nThreshold; // 0-80 dB for display only � does not affect raw data
	UINT m_nIntensity; // 10-90 dB for display only � does not affect raw data
	UINT m_nReceiverGain; // 0-40 dB normalized gain range
	UINT m_nDegC; // Power Supply Temp �C
	UINT m_nDegC2; // A/D Temp �C
	UINT m_nHumidity; // relative humidity 0-100
	UINT m_nFocus; // 0-255 Focus position code (range varies with sonar model)
	UINT m_nBattery; // Volts at sonar PS in tenths, e.g. 145 = 14.5 volts (Note 12)
	float m_fUserValue1; // User defined value for embedded applications (Note 13)
	float m_fUserValue2; // User defined value for embedded applications
	float m_fUserValue3; // User defined value for embedded applications
	float m_fUserValue4; // User defined value for embedded applications
	float m_fUserValue5; // User defined value for embedded applications
	float m_fUserValue6; // User defined value for embedded applications
	float m_fUserValue7; // User defined value for embedded applications
	float m_fUserValue8; // User defined value for embedded applications
	float m_fVelocity; // These are space holders for platform generated data
	float m_fDepth; // Placed in the header external to the sonar (except DH model)
	float m_fAltitude; // Placed in the header external to the sonar
	float m_fPitch; // Placed in the header external to the sonar
	float m_fPitchRate; // Placed in the header external to the sonar
	float m_fRoll; // Placed in the header external to the sonar
	float m_fRollRate; // Placed in the header external to the sonar
	float m_fHeading; // From sonar compass or external to the sonar
	float m_fHeadingRate; // Placed in the header external to the sonar
	float m_fCompassHeading; // From sonar compass when installed
	float m_fCompassPitch; // From sonar compass when installed
	float m_fCompassRoll; // From sonar compass when installed
	double m_dLatitude; // In degrees ddd.ddddddd
	double m_dLongitude; // In degrees ddd.ddddddd
	float m_fSonarPosition; // Placed in the header external to the sonar
	UINT m_nConfigFlags; // sonar configuration flags updated Dec 2006 (see Note 8)
	UINT m_nPrismTilt; // sonar split-body variable added 26 April 2004
	float m_fTargetRange; // added 14 September 2004 for topside software use
	float m_fTargetBearing; // added 14 September 2004 for topside software use
	BOOL m_bTargetPresent; // added 14 September 2004 for topside software use
	UINT m_nFirmwareRevision; // added 1 April 2005
	UINT m_nFlags; // added 1 August 2005 (see Note 9)
	UINT m_nSourceFrame; // added 16 Nov 2006 (record CSOT source frame)
	float m_fWaterTemp; // added 19 February 2007 to store NMEA MTW input
	UINT m_nTimerPeriod; // added 18 June 2007 for non-integral frame rates
	float m_fSonarX; // added 26 December 2007
	float m_fSonarY; // added 26 December 2007
	float m_fSonarZ; // added 6 March 2008 for DDF_04
	float m_fSonarPan; // from ROS/Sidus/SMC rotators
	float m_fSonarTilt; // from ROS/Sidus/SMC rotators
	float m_fSonarRoll; // from ROS/Sidus/SMC rotators
	float m_fPanPNNL; // legacy custom data from Battelle PNNL equipment
	float m_fTiltPNNL; // legacy custom data from Battelle PNNL equipment
	float m_fRollPNNL; // legacy custom data from Battelle PNNL equipment
	double m_dVehicleTime; // Bluefin HAUV time (seconds since 1 Jan 1970)
	float m_fTimeGGK; // time data from NMEA GGK string
	UINT m_nDateGGK; // time data from NMEA GGK string
	UINT m_nQualityGGK; // date data from NMEA GGK string
	UINT m_nNumSatsGGK; // number of satellites data from NMEA GGK string
	float m_fDOPGGK; // degree of precision data from NMEA GGK string
	float m_fEHTGGK; // data from NMEA GGK string
	float m_fHeaveTSS; // heave data from TSS aux input string
	UINT m_nYearGPS; // GPS input data - GPS time data
	UINT m_nMonthGPS; // GPS input data - GPS time data
	UINT m_nDayGPS; // GPS input data - GPS time data
	UINT m_nHourGPS; // GPS input data - GPS time data
	UINT m_nMinuteGPS; // GPS input data - GPS time data
	UINT m_nSecondGPS; // GPS input data - GPS time data
	UINT m_nHSecondGPS; // GPS input data - GPS time data
	float m_fSonarPanOffset; // sonar mount "pan 0" rotation about Z axis
	float m_fSonarTiltOffset; // sonar mount "tilt 0" rotation about local sonar Y axis
	float m_fSonarRollOffset; // sonar mount "roll 0" rotation about local sonar X axis
	float m_fSonarXOffset; // added 28 May 2008
	float m_fSonarYOffset; // added 28 May 2008
	float m_fSonarZOffset; // added 28 May 2008
	float m_fTMatrix[16]; // added 28 May 2008
	BYTE m_cRsvdData[FRAME_PAD_LENGTH]; // (604) pad frame header to 1024 bytes
        BYTE m_cData[DATA_SIZE]; // acoustic data length
	// DIDSON-Std 24576 (LF) or 49152 (HF)
	// DIDSON-LR 24576 (LF, HF)
};
#pragma pack ()
