#ifndef __MICROSTRAIN_GX3_H__
#define __MICROSTRAIN_GX3_H__

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <arpa/inet.h>

#include "perls-common/units.h"

#include "perls-lcmtypes/senlcm_ms_gx3_25_t.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MICROSTRAIN_MAX_LEN     128

/* commands */  

#define CMD_ACCEL_ANG_RATE 0xC2
#define LEN_ACCEL_ANG_RATE 31

#define CMD_DEL_ANG_DEL_VEL 0xC3
#define LEN_DEL_ANG_DEL_VEL 31

#define CMD_SET_CONT_MODE 0xC4
#define LEN_SET_CONT_MODE 8

#define CMD_ORIENT_MATRIX 0xC5
#define LEN_ORIENT_MATRIX 43

#define CMD_ORIENT_UPDATE_MATRIX 0xC6
#define LEN_ORIENT_UPDATE_MATRIX 43

#define CMD_MAG_VECTOR 0xC7
#define LEN_MAG_VECTOR 19

#define CMD_ACCEL_ANG_RATE_ORIENT_MATRIX 0xC8
#define LEN_ACCEL_ANG_RATE_ORIENT_MATRIX 67

#define CMD_WRITE_ACCEL_BIAS_COR 0xC9
#define LEN_WRITE_ACCEL_BIAS_COR 19

#define CMD_WRITE_GYRO_BIAS_COR 0xCA
#define LEN_WRITE_GYRO_BIAS_COR 19

#define CMD_ACCEL_ANG_RATE_MAG_VECTOR_ORIENT_MATRIX 0xCC
#define LEN_ACCEL_ANG_RATE_MAG_VECTOR_ORIENT_MATRIX 79

#define CMD_CAPTURE_GYRO_BIAS 0xCD
#define LEN_CAPTURE_GYRO_BIAS 19

#define CMD_EULER_ANG 0xCE
#define LEN_EULER_ANG 19
/* returns 0 for fail, 1 for success */
int
ms_parseEulerAng (const uint8_t buf[], int buflen,
		    senlcm_ms_gx3_25_t *ms);

#define CMD_EULER_ANG_AND_ANG_RATES 0xCF
#define LEN_EULER_ANG_AND_ANG_RATES 31

#define CMD_TRAN_QUANT_TO_NV_MEMORY 0xD0
#define LEN_TRAN_QUANT_TO_NV_MEMORY 9

#define CMD_DEL_ANG_DEL_VEL_MAG_VECTORS 0xD3
#define LEN_DEL_ANG_DEL_VEL_MAG_VECTORS 43

#define CMD_MODE 0xD4
#define LEN_MODE 4

#define CMD_MODE_PRESET 0xD5
#define LEN_MODE_PRESET 4

#define CMD_CONT_PRESET 0xD6
#define LEN_CONT_PRESET 4

#define CMD_TIMER 0xD7
#define LEN_TIMER 7

#define CMD_COMMS_SETTINGS 0xD9
#define LEN_COMMS_SETTINGS 10

#define CMD_STATIONARY_TEST 0xDA
#define LEN_STATIONARY_TEST 7

#define CMD_SAMPLING_SETTINGS 0xDB
#define LEN_SAMPLING_SETTINGS 19
int
ms_parseSettings (const uint8_t buf[], int buflen,
		  senlcm_ms_gx3_25_t *ms, uint8_t* commandBuf);

#define CMD_REALIGN_UP_AND_NORTH 0xDD
#define LEN_REALIGN_UP_AND_NORTH 7

#define CMD_READ_WORD_FROM_EEPROM 0xE5
#define LEN_READ_WORD_FROM_EEPROM 5

#define CMD_READ_FIRMWARE_VERSION_NUM 0xE9
#define LEN_READ_FIRMWARE_VERSION_NUM 7

#define CMD_READ_DEVICE_ID_STRING 0xEA
#define LEN_READ_DEVICE_ID_STRING 20

#define CMD_STOP_CONT_MODE_NO_REPLY 0xFA
#define LEN_STOP_CONT_MODE_NO_REPLY 0

#define CMD_FIRMWARE_UPDATE_NO_REPLY 0xFD
#define LEN_DEVICE_RESET_NO_REPLY 0

#define CMD_GYRO_STAB_ACCEL_ANG_RATE_MAG 0xD2
#define LEN_GYRO_STAB_ACCEL_ANG_RATE_MAG 43
/* returns 0 for fail, 1 for success */
int
ms_parseGyroVector (const uint8_t buf[], int buflen,
		    senlcm_ms_gx3_25_t *ms);

#define CMD_ACCEL_ANG_RATE_MAG_VECTOR 0xCB
#define LEN_ACCEL_ANG_RATE_MAG_VECTOR 43
/* returns 0 for fail, 1 for success */
int
ms_parseInstVector (const uint8_t buf[], int buflen,
		    senlcm_ms_gx3_25_t *ms);

#define CMD_QUATERNION 0xDF
#define LEN_QUATERNION 23
///* returns 0 for fail, 1 for success */
//int
//ms_parseQuat (uint8_t buf[], int buflen,
//	      senlcm_ms_gx3_25_t *ms);

#define CMD_TEMPERATURE 0xD1
#define LEN_TEMPERATURE 15
/* returns 0 for fail, 1 for success */
int
ms_parseTemperature (uint8_t buf[], int buflen,
		     senlcm_ms_gx3_25_t *ms);

#define CMD_WRITE_WORD_TO_EEPROM 0xE4
#define LEN_WRITE_WORD_TO_EEPROM 5
/* returns 0 for fail, 1 for success */
int
ms_parseReadEepromWithChecksum (uint8_t buf[], int buflen,
                                uint16_t *Value,
				senlcm_ms_gx3_25_t *ms);

static inline void
ms_convert_accel (double *accel)
{
  *accel = *accel * 9.81;
}

static inline void
ms_convert_TimerTicks (int32_t raw_timerTicks, double *timerTicks)
{
    *timerTicks = ntohl (raw_timerTicks) * 0.0065536; // seconds
}

/* returns checksum */
uint16_t
ms_compute_checksum (const uint8_t *buf, int buflen);

/* returns 1 pass, 0 fail */
int
ms_verify_checksum (const uint8_t *buf, int buflen);

static inline void
ms_convert_Temperature (uint16_t raw_T, double *T)
{
  uint16_t t = ntohs(raw_T);
  *T = -1481.96 + sqrt(2.1962e6 + (1.8639 - ((3 * (double)t) / 4096))/3.88e-6);
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
        E[i] = ((int16_t) ntohs (raw_E[i])) * (360.0/65536.0) 
	  * UNITS_DEGREE_TO_RADIAN; // rad
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

#ifdef __cplusplus
}
#endif

#endif //__MICROSTRAIN_GX3_H__
