#ifndef __MICROSTRAIN_GX1_H__
#define __MICROSTRAIN_GX1_H__

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <arpa/inet.h>

#include "perls-common/units.h"


#ifdef __cplusplus
extern "C" {
#endif

#define MICROSTRAIN_MAX_LEN     128

/* commands */
#define CMD_RAW_SENSOR          0x01     
#define LEN_RAW_SENSOR          23


#define CMD_GYRO_VECTOR         0x02
#define LEN_GYRO_VECTOR         23
/* returns 0 fail, 1 success */
int
ms_parseGyroVector (char buf[], int buflen,
                    double StabMagField[], uint16_t MagGainScale,
                    double StabAccel[],    uint16_t AccelGainScale,
                    double CompAngRate[],  uint16_t GyroGainScale,
                    int16_t *TimerTicks);


#define CMD_INST_VECTOR      0x03
#define LEN_INST_VECTOR      23
/* returns 0 fail, 1 success */
int
ms_parseInstVector (char buf[], int buflen,
                    double MagField[], uint16_t MagGainScale,
                    double Accel[],    uint16_t AccelGainScale,
                    double AngRate[],  uint16_t GyroGainScale,
                    int16_t *TimerTicks);


#define CMD_INST_QUAT        0x04
#define LEN_INST_QUAT        13
/* returns 0 fail, 1 success */
int
ms_parseInstQuat (char buf[], int buflen,
                  double Q[],
                  int16_t *TimerTicks);


#define CMD_GYRO_QUAT           0x05
#define LEN_GYRO_QUAT           13
/* returns 0 fail, 1 success */
int
ms_parseGyroQuat (char buf[], int buflen,
                  double StabQ[],
                  int16_t *TimerTicks);


#define CMD_CAPTURE_GYRO_BIAS   0x06
#define LEN_CAPTURE_GYRO_BIAS   5


#define CMD_TEMPERATURE         0x07
#define LEN_TEMPERATURE         7
/* returns 0 fail, 1 success */
int
ms_parseTemperature (char buf[], int buflen,
                     double *Temp,
                     int16_t *TimerTicks);

#define CMD_READ_EEPROM         0x08 // 0x28 is preferred
#define LEN_READ_EEPROM         2


#define CMD_WRITE_EEPROM        0x09 // 0x29 is preferred
#define LEN_WRITE_EEPROM        2


#define CMD_INSTANT_OR_MATRIX   0x0A
#define LEN_INSTANT_OR_MATRIX   23


#define CMD_GYRO_OR_MATRIX      0x0B
#define LEN_GYRO_OR_MATRIX      23


#define CMD_GYRO_QUAT_VECTOR    0x0C
#define LEN_GYRO_QUAT_VECTOR    31
/* returns 0 fail, 1 success */
int
ms_parseGyroQuatVector (char buf[], int buflen,
                        double StabQ[], 
                        double MagField[], uint16_t MagGainScale,
                        double Accel[],    uint16_t AccelGainScale,
                        double CompAngRate[], uint16_t GyroGainScale,
                        int16_t *TimerTicks);


#define CMD_INST_EULER          0x0D
#define LEN_INST_EULER          11


#define CMD_GYRO_EULER          0x0E
#define LEN_GYRO_EULER          11


#define CMD_SET_CONTINUOUS      0x10
#define LEN_SET_CONTINUOUS      7

#define CMD_READ_EEPROM_WITH_CHECKSUM   0x28
#define LEN_READ_EEPROM_WITH_CHECKSUM   7
/* returns 0 fail, 1 success */
int
ms_parseReadEepromWithChecksum (char buf[], int buflen,
                                uint16_t *Value,
                                int16_t *TimerTicks);

#define CMD_WRITE_EEPROM_WITH_CHECKSUM  0x29
#define LEN_WRITE_EEPROM_WITH_CHECKSUM  7


#define CMD_GYRO_EULER_VECTOR   0x31
#define LEN_GYRO_EULER_VECTOR   23


#define CMD_FIRMWARE_VERSION    0xF0
#define LEN_FIRMWARE_VERSION    5


#define CMD_SERIAL_NUMBER       0xF1
#define LEN_SERIAL_NUMBER       5

#define CMD_INIT_HARD_IRON_FIELD_CALIB  0x40
#define LEN_INIT_HARD_IRON_FIELD_CALIB  5


static inline void
ms_convert_MagField (int16_t raw_magfield[], double magfield[], uint16_t MagGainScale)
{
    for (int i=0; i<3; i++)
        magfield[i] = ((int16_t) ntohs (raw_magfield[i])) / (32768000.0 / MagGainScale); // Gauss
}

static inline void
ms_convert_Accel (int16_t raw_accel[], double accel[], uint16_t AccelGainScale)
{
    for (int i=0; i<3; i++)
        accel[i] = ((int16_t) ntohs (raw_accel[i])) / (32768000.0 / AccelGainScale) * 9.81; // m/s^2
}

static inline void
ms_convert_AngRate (int16_t raw_angRate[], double angRate[], uint16_t GyroGainScale)
{
    for (int i=0; i<3; i++)
        angRate[i] = ((int16_t) ntohs (raw_angRate[i])) / (32768000.0 / GyroGainScale); // rad/s
}

static inline void
ms_convert_TimerTicks (int16_t raw_timerTicks, double *timerTicks)
{
    *timerTicks = ntohs (raw_timerTicks) * 0.0065536; // seconds
}

static inline void
ms_convert_M (int16_t raw_M[], double M[])
{
    for (int i=0; i<9; i++)
        M[i] = ((int16_t) ntohs (raw_M[i])) / 8192.0; // dimless
}

static inline void
ms_convert_Q (int16_t raw_Q[], double Q[])
{
    for (int i=0; i<4; i++)
        Q[i] =  ((int16_t) ntohs (raw_Q[i])) / 8192.0; // dimless
}

static inline void
ms_convert_Euler (int16_t raw_E[], double E[])
{
    for (int i=0; i<3; i++)
        E[i] = ((int16_t) ntohs (raw_E[i])) * (360.0/65536.0) * UNITS_DEGREE_TO_RADIAN; // rad
}

static inline void
ms_convert_Temperature (int16_t raw_T, double *T)
{
    *T = ((((int16_t) ntohs (raw_T)) * 5.0/65536.0)-0.5)*100.0; // C
}

static inline void
ms_convert_Checksum (uint16_t raw_checksum, uint16_t *checksum)
{
    *checksum = ntohs (raw_checksum);
}

static inline void
ms_Q_to_M (double Q[], double M[][3])
{
    double q0=Q[0], q1=Q[1], q2=Q[2], q3=Q[3];
    // 1st row
    M[0][0] = 2.0*(q0*q0 - 0.5 + q1*q1);
    M[0][1] = 2.0*(q1*q2 + q0*q3);
    M[0][2] = 2.0*(q1*q3 - q0*q2);
    // 2nd row
    M[1][0] = 2.0*(q1*q2 - q0*q3);
    M[1][1] = 2.0*(q0*q0 - 0.5 + q2*q2);
    M[1][2] = 2.0*(q2*q3 + q0*q1);
    // 3rd row
    M[2][0] = 2.0*(q1*q3 + q0*q2);
    M[2][1] = 2.0*(q2*q3 - q0*q1);
    M[2][2] = 2.0*(q0*q0 - 0.5 + q3*q3);
}

static inline void
ms_M_to_Q (double M[][3], double Q[])
{
    double test1 =  M[0][0] + M[1][1] + M[2][2];
    double test2 =  M[0][0] - M[1][1] - M[2][2];
    double test3 = -M[0][0] + M[1][1] - M[2][2];
    double test4 = -M[0][0] - M[1][1] + M[2][2];

    if (test1>test2 && test1>test3 && test1>test4) {
        double s = 2.0 * sqrt (1+test1);
        Q[0] = s/4.0;
        Q[1] = (M[1][2] - M[2][1])/s;
        Q[2] = (M[2][0] - M[0][2])/s;
        Q[3] = (M[0][1] - M[1][0])/s;
    }
    else if (test2>test1 && test2>test3 && test2>test4) {
        double s = 2.0 * sqrt (1+test2);
        Q[0] =  (M[2][1] - M[1][2])/s;
        Q[1] = -s/4.0;
        Q[2] = -(M[1][0] + M[0][1])/s;
        Q[3] = -(M[0][2] + M[2][0])/s;
    }
    else if (test3>test1 && test3>test2 && test3>test4) {
        double s = 2.0 * sqrt (1+test3);
        Q[0] =  (M[0][2] - M[2][0])/s;
        Q[1] = -(M[1][0] + M[0][1])/s;
        Q[2] = -s/4.0;
        Q[3] = -(M[2][1] + M[1][2])/s;
    }
    else {
        double s = 2.0 * sqrt (1+test4);
        Q[0] =  (M[1][0] - M[0][1])/s;
        Q[1] = -(M[0][2] + M[2][0])/s;
        Q[2] = -(M[2][1] + M[1][2])/s;
        Q[3] = -s/4;
    }
}

static inline void
ms_M_to_Euler (double M[][3], double Euler[])
{
    double h = atan2 (M[0][1], M[0][0]);
    double ch = cos (h), sh = sin (h);
    double p = atan2 (-M[0][2], M[0][0]*ch + M[0][1]*sh);
    double r = atan2 (M[2][0]*sh - M[2][1]*ch, -M[1][0]*sh + M[1][1]*ch);

    Euler[0] = r;
    Euler[1] = p;
    Euler[2] = h;
}

static inline void
ms_Euler_to_M (double Euler[], double M[][3])
{
    double cr = cos (Euler[0]), sr = sin (Euler[0]);
    double cp = cos (Euler[1]), sp = sin (Euler[1]);
    double ch = cos (Euler[2]), sh = sin (Euler[2]);
    
    // 1st row
    M[0][0] = ch*cp;
    M[0][1] = sh*cp;
    M[0][2] = -sp;
    // 2nd row
    M[1][0] = -sh*cr + ch*sp*sr;
    M[1][1] = ch*cr + sh*sp*sr;
    M[1][2] = cp*sr;
    // 3rd row
    M[2][0] = sh*sr + ch*sp*cr;
    M[2][1] = -ch*sr + sh*sp*cr;
    M[2][2] = cp*cr;
}

static inline void
ms_Q_to_Euler (double Q[], double Euler[])
{
    double M[3][3];
    ms_Q_to_M (Q, M);
    ms_M_to_Euler (M, Euler);
}

static inline void
ms_Euler_to_Q (double Euler[], double Q[])
{
    double M[3][3];
    ms_Euler_to_M (Euler, M);
    ms_M_to_Q (M, Q);
}

/* returns checksum */
uint16_t
ms_compute_checksum (const char *buf, int buflen);

/* returns 1 pass, 0 fail */
int
ms_verify_checksum (const char *buf, int buflen);


#ifdef __cplusplus
}
#endif

#endif //__MICROSTRAIN_GX1_H__
