#include <string.h>
#include <stdint.h>

#include "ms_3dm_gx1.h"


uint16_t
ms_compute_checksum (const char *buf, int buflen)
{
    uint16_t checksum = buf[0];
    for (int i=1; i<buflen-2; i+=2)
    {
        uint16_t *val = (uint16_t *) &buf[i];
        checksum += ntohs (*val);
    }
    return checksum;
}

int
ms_verify_checksum (const char *buf, int buflen)
{
    uint16_t *val = (uint16_t *) &buf[buflen-2];
    uint16_t checksum = ntohs (*val);
    if (checksum == ms_compute_checksum (buf, buflen))
        return 1;
    else
        return 0;
}

int
ms_parseGyroVector (char buf[], int buflen,
                    double StabMagField[], uint16_t MagGainScale,
                    double StabAccel[],    uint16_t AccelGainScale,
                    double CompAngRate[],  uint16_t GyroGainScale,
                    int16_t *TimerTicks)
{
    if (buflen != LEN_GYRO_VECTOR || !ms_verify_checksum (buf, buflen))
        return 0;

    struct rawpkt
    {
        uint8_t Header;
        int16_t StabMagField[3];
        int16_t StabAccel[3];
        int16_t CompAngRate[3];
        uint16_t TimerTicks;
        uint16_t Checksum;
    } __attribute__((packed)) data;
    memcpy (&data, buf, buflen);

    ms_convert_MagField (data.StabMagField, StabMagField, MagGainScale);
    ms_convert_Accel (data.StabAccel, StabAccel, AccelGainScale);
    ms_convert_AngRate (data.CompAngRate, CompAngRate, GyroGainScale);
    //ms_convert_TimerTicks (data.TimerTicks, TimerTicks);
    *TimerTicks = ntohs (data.TimerTicks);

    return 1;
}

int
ms_parseInstVector (char buf[], int buflen,
                    double MagField[], uint16_t MagGainScale,
                    double Accel[],    uint16_t AccelGainScale,
                    double AngRate[],  uint16_t GyroGainScale,
                    int16_t *TimerTicks)
{
    if (buflen != LEN_INST_VECTOR || !ms_verify_checksum (buf, buflen))
        return 0;

    struct rawpkt
    {
        uint8_t Header;
        int16_t MagField[3];
        int16_t Accel[3];
        int16_t AngRate[3];
        uint16_t TimerTicks;
        uint16_t Checksum;
    } __attribute__((packed)) data;
    memcpy (&data, buf, buflen);

    ms_convert_MagField (data.MagField, MagField, MagGainScale);
    ms_convert_Accel (data.Accel, Accel, AccelGainScale);
    ms_convert_AngRate (data.AngRate, AngRate, GyroGainScale);
    //ms_convert_TimerTicks (data.TimerTicks, TimerTicks);
    *TimerTicks = ntohs (data.TimerTicks);

    return 1;
}


int
ms_parseInstQuat (char buf[], int buflen,
                  double Q[],
                  int16_t *TimerTicks)
{
    if (buflen != LEN_INST_QUAT || !ms_verify_checksum (buf, buflen))
        return 0;

    struct rawpkt
    {
        uint8_t Header;
        int16_t Q[4];
        uint16_t TimerTicks;
        uint16_t Checksum;
    } __attribute__((packed)) data;
    memcpy (&data, buf, buflen);

    ms_convert_Q (data.Q, Q);
    //ms_convert_TimerTicks (data.TimerTicks, TimerTicks);
    *TimerTicks = ntohs (data.TimerTicks);

    return 1;

}


int
ms_parseGyroQuat (char buf[], int buflen,
                  double StabQ[],
                  int16_t *TimerTicks)
{
    if (buflen != LEN_GYRO_QUAT || !ms_verify_checksum (buf, buflen))
        return 0;

    struct rawpkt
    {
        uint8_t Header;
        int16_t StabQ[4];
        uint16_t TimerTicks;
        uint16_t Checksum;
    } __attribute__((packed)) data;
    memcpy (&data, buf, buflen);

    ms_convert_Q (data.StabQ, StabQ);
    //ms_convert_TimerTicks (data.TimerTicks, TimerTicks);
    *TimerTicks = ntohs (data.TimerTicks);

    return 1;
}

int
ms_parseGyroQuatVector (char buf[], int buflen,
                        double StabQ[],
                        double MagField[], uint16_t MagGainScale,
                        double Accel[],    uint16_t AccelGainScale,
                        double CompAngRate[], uint16_t GyroGainScale,
                        int16_t *TimerTicks)
{
    if (buflen != LEN_GYRO_QUAT_VECTOR || !ms_verify_checksum (buf, buflen))
        return 0;

    struct rawpkt
    {
        uint8_t Header;
        int16_t StabQ[4];
        int16_t MagField[3];
        int16_t Accel[3];
        int16_t CompAngRate[3];
        uint16_t TimerTicks;
        uint16_t Checksum;
    } __attribute__((packed)) data;
    memcpy (&data, buf, buflen);

    ms_convert_Q (data.StabQ, StabQ);
    ms_convert_MagField (data.MagField, MagField, MagGainScale);
    ms_convert_Accel (data.Accel, Accel, AccelGainScale);
    ms_convert_AngRate (data.CompAngRate, CompAngRate, GyroGainScale);
    //ms_convert_TimerTicks (data.TimerTicks, TimerTicks);
    *TimerTicks = ntohs (data.TimerTicks);

    return 1;
}

int
ms_parseTemperature (char buf[], int buflen,
                     double *Temp,
                     int16_t *TimerTicks)
{
    if (buflen != LEN_TEMPERATURE || !ms_verify_checksum (buf, buflen))
        return 0;

    struct rawpkt
    {
        uint8_t Header;
        int16_t Temp;
        uint16_t TimerTicks;
        uint16_t Checksum;
    } __attribute__((packed)) data;
    memcpy (&data, buf, buflen);

    ms_convert_Temperature (data.Temp, Temp);
    //ms_convert_TimerTicks (data.TimerTicks, TimerTicks);
    *TimerTicks = ntohs (data.TimerTicks);

    return 1;
}

int
ms_parseReadEepromWithChecksum (char buf[], int buflen,
                                uint16_t *Value,
                                int16_t *TimerTicks)
{
    if (buflen != LEN_READ_EEPROM_WITH_CHECKSUM || !ms_verify_checksum (buf, buflen))
        return 0;

    struct rawpkt
    {
        uint8_t Header;
        uint16_t Value;
        uint16_t TimerTicks;
        uint16_t Checksum;
    } __attribute__((packed)) data;
    memcpy (&data, buf, buflen);

    *Value = ntohs (data.Value);
    *TimerTicks = ntohs (data.TimerTicks);

    return 1;
}
