#include <string.h>
#include <stdint.h>

#include "perls-lcmtypes/senlcm_ms_gx3_25_t.h"

#include "ms_3dm_gx3.h"

static void
ms_convertRawToDouble(uint8_t *rawData, double d[],
		      int numDoubles) {

  uint32_t liaison;
  int i;
  int size = sizeof(uint32_t);

  for(i = 0; i < numDoubles; i++) {

    liaison = ntohl(*(uint32_t*)(rawData + (i * size)));
    d[i] = *(float*)&liaison;
    
  }

}

uint16_t
ms_compute_checksum (const uint8_t *buf, int buflen)
{
  uint16_t checksum = 0;
  for (int i=0; i<buflen-2; i++) {
    uint8_t *val = (uint8_t *) &buf[i];
    checksum +=*val;
  }
  return checksum;
}

int
ms_verify_checksum (const uint8_t *buf, int buflen)
{
  uint16_t *val = (uint16_t *) &buf[buflen-2];
  uint16_t checksum = ntohs (*val);
  if (checksum == ms_compute_checksum (buf, buflen))
    return 1;
  else
    return 0;
}

int
ms_parseSettings (const uint8_t buf[], int buflen,
		  senlcm_ms_gx3_25_t *ms, uint8_t* commandBuf)
{
  if (buflen != LEN_SAMPLING_SETTINGS || !ms_verify_checksum (buf, buflen))
    return 0;

  struct rawpkt {
    uint8_t Header;
    uint8_t Data[LEN_SAMPLING_SETTINGS - 3];
    uint16_t Checksum;
  } __attribute__((packed)) data;

  memcpy (&data, buf, buflen);
  
  memcpy (commandBuf, data.Data, LEN_SAMPLING_SETTINGS - 3);

  return 1;

}

int
ms_parseEulerAng (const uint8_t buf[], int buflen,
		    senlcm_ms_gx3_25_t *ms) {
  
   if (buflen != LEN_EULER_ANG || !ms_verify_checksum (buf, buflen))
    return 0;
  
  struct rawpkt {
    uint8_t Header;
    uint8_t Euler[3 * sizeof(float)]; // 3 "raw" floats
    uint32_t TimerTicks;
    uint16_t Checksum;
  } __attribute__((packed)) data;

  memcpy (&data, buf, buflen);

  ms_convertRawToDouble(data.Euler, ms->Euler, 3);

  ms->TimerTicks = ntohl(data.TimerTicks);

  return 1;
  
}

int
ms_parseGyroVector (const uint8_t buf[], int buflen,
		    senlcm_ms_gx3_25_t *ms)
{
  if (buflen != LEN_GYRO_STAB_ACCEL_ANG_RATE_MAG || !ms_verify_checksum (buf, buflen))
    return 0;
  
  struct rawpkt {
    uint8_t Header;
    uint8_t StabAccel[3 * sizeof(float)]; // 3 "raw" floats
    uint8_t CompAngRate[3 * sizeof(float)]; // 3 "raw" floats
    uint8_t StabMagField[3 * sizeof(float)]; // 3 "raw" floats
    uint32_t TimerTicks;
    uint16_t Checksum;
  } __attribute__((packed)) data;

  memcpy (&data, buf, buflen);

  ms_convertRawToDouble(data.StabMagField, ms->MagField, 3);
  ms_convertRawToDouble(data.StabAccel,    ms->Accel, 3);
  ms_convertRawToDouble(data.CompAngRate,  ms->AngRate, 3);

  //convert to m/s^2
  int i;
  for (i = 0; i < 3; i++)
    ms_convert_accel(&ms->Accel[i]);

  ms->TimerTicks = ntohl(data.TimerTicks);

  return 1;
}

int
ms_parseInstVector (const uint8_t buf[], int buflen,
		    senlcm_ms_gx3_25_t *ms)
{
  if (buflen != LEN_ACCEL_ANG_RATE_MAG_VECTOR || !ms_verify_checksum (buf, buflen))
    return 0;

  struct rawpkt {
    uint8_t Header;
    uint8_t instAccel[3 * sizeof(float)]; // 3 "raw" floats
    uint8_t instAngRate[3 * sizeof(float)]; // 3 "raw" floats
    uint8_t instMagField[3 * sizeof(float)]; // 3 "raw" floats
    uint32_t TimerTicks;
    uint16_t Checksum;
  } __attribute__((packed)) data;

  memcpy (&data, buf, buflen);
  
  ms_convertRawToDouble(data.instMagField, ms->MagField, 3);
  ms_convertRawToDouble(data.instAccel,    ms->Accel, 3);
  ms_convertRawToDouble(data.instAngRate,  ms->AngRate, 3);

  //convert to m/s^2
  int i;
  for (i = 0; i < 3; i++)
    ms_convert_accel(&ms->Accel[i]);

  ms->TimerTicks = ntohl(data.TimerTicks);

  return 1;
}

//int
//ms_parseQuat (uint8_t buf[], int buflen,
//	      senlcm_ms_gx3_25_t *ms)
//{
//  if (buflen != LEN_QUATERNION || !ms_verify_checksum (buf, buflen))
//    return 0;
//
//  struct rawpkt {
//    uint8_t Header;
//    uint8_t StabQ[4 * sizeof(float)]; // 4 "raw" floats
//    uint32_t TimerTicks;
//    uint16_t Checksum;
//  } __attribute__((packed)) data;
//  memcpy(&data, buf, buflen);
//
//  ms_convertRawToDouble(data.StabQ, ms->sQ, 4);
//
//  ms->TimerTicks = ntohl(data.TimerTicks);
//
//  return 1;    
//}

int
ms_parseTemperature (uint8_t buf[], int buflen,
		     senlcm_ms_gx3_25_t *ms)
{
  if (buflen != LEN_TEMPERATURE || !ms_verify_checksum (buf, buflen))
    return 0;
  
  struct rawpkt {
    uint8_t Header;
    uint16_t TempMag;
    uint16_t TempGy;
    uint16_t TempGx;
    uint16_t TempGz;
    uint32_t TimerTicks;
    uint16_t Checksum;
  } __attribute__((packed)) data;
  memcpy (&data, buf, buflen);

  ms_convert_Temperature (data.TempMag, &ms->Temperature);
  ms->TimerTicks = ntohl(data.TimerTicks);
    
  return 1;
}

int
ms_parseReadEepromWithChecksum (uint8_t buf[], int buflen,
				uint16_t *Value,
                                senlcm_ms_gx3_25_t *ms)
{
  if (buflen != LEN_WRITE_WORD_TO_EEPROM || !ms_verify_checksum (buf, buflen))
    return 0;

  struct rawpkt {
    uint8_t Header;
    uint16_t Value;
    uint32_t TimerTicks;
    uint16_t Checksum;
  } __attribute__((packed)) data;
  memcpy (&data, buf, buflen);

  *Value = data.Value;
  ms->TimerTicks = ntohl(data.TimerTicks);

  return 1;
}
