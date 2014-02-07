#ifndef __PELICAN_H__
#define __PELICAN_H__

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <arpa/inet.h>
#include <time.h>

#include "perls-common/units.h"

#include "perls-lcmtypes/senlcm_pelican_t.h"

/**
 * @defgroup SensorDriversPelican Pelican Sensor Driver
 * @brief This is just sample documentation for a sensor driver.
 * Documenting drivers like this is not necessary
 * @ingroup SensorDrivers
 * @author Paul Ozog - paulozog@umich.edu
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define PELIC_MAX_LEN 128 //128 is probably overkill

/**
 * From documentation : used to poll telemetry
 */
#define PELIC_QUERY_STR ">*>p"

#define PELIC_IMURAWDATA_ID SENLCM_PELICAN_T_PD_IMURAWDATA
#define PELIC_LLSTATUS_ID SENLCM_PELICAN_T_PD_LLSTATUS
#define PELIC_IMUCALCDATA_ID SENLCM_PELICAN_T_PD_IMUCALCDATA
#define PELIC_HLSTATUS_ID SENLCM_PELICAN_T_PD_HLSTATUS
#define PELIC_DEBUGDATA_ID SENLCM_PELICAN_T_PD_DEBUGDATA
#define PELIC_CTRLOUT_ID SENLCM_PELICAN_T_PD_CTRLOUT
#define PELIC_FLIGHTPARAMS_ID SENLCM_PELICAN_T_PD_FLIGHTPARAMS
#define PELIC_CTRLCOMMANDS_ID SENLCM_PELICAN_T_PD_CTRLCOMMANDS
#define PELIC_CTRLINTERNAL_ID SENLCM_PELICAN_T_PD_CTRLINTERNAL
#define PELIC_RCDATA_ID SENLCM_PELICAN_T_PD_RCDATA
#define PELIC_CTRLSTATUS_ID SENLCM_PELICAN_T_PD_CTRLSTATUS
#define PELIC_CTRLINPUT_ID SENLCM_PELICAN_T_PD_CTRLINPUT
#define PELIC_CTRLFALCON_ID SENLCM_PELICAN_T_PD_CTRLFALCON
#define PELIC_WAYPOINT_ID SENLCM_PELICAN_T_PD_WAYPOINT
#define PELIC_CURRENTWAY_ID SENLCM_PELICAN_T_PD_CURRENTWAY
#define PELIC_NMEADATA_ID SENLCM_PELICAN_T_PD_NMEADATA
#define PELIC_GPSDATA_ID SENLCM_PELICAN_T_PD_GPSDATA
#define PELIC_SINGLEWAYPOINT_ID SENLCM_PELICAN_T_PD_SINGLEWAYPOINT
#define PELIC_GOTOCOMMAND_ID SENLCM_PELICAN_T_PD_GOTOCOMMAND
#define PELIC_LAUNCHCOMMAND_ID SENLCM_PELICAN_T_PD_LAUNCHCOMMAND
#define PELIC_LANDCOMMAND_ID SENLCM_PELICAN_T_PD_LANDCOMMAND
#define PELIC_HOMECOMMAND_ID SENLCM_PELICAN_T_PD_HOMECOMMAND
#define PELIC_GPSDATAADVANCED_ID SENLCM_PELICAN_T_PD_GPSDATAADVANCED

#define PELIC_LLSTATUS_CMD SENLCM_PELICAN_T_LL_STATUS_CMD
#define PELIC_IMU_RAWDATA_CMD SENLCM_PELICAN_T_IMU_RAWDATA_CMD
#define PELIC_IMU_CALCDATA_CMD SENLCM_PELICAN_T_IMU_CALCDATA_CMD
#define PELIC_RC_DATA_CMD SENLCM_PELICAN_T_RC_DATA_CMD
#define PELIC_CTRL_OUT_CMD SENLCM_PELICAN_T_CTRL_OUT_CMD
#define PELIC_GPS_DATA_CMD SENLCM_PELICAN_T_GPS_DATA_CMD
#define PELIC_CURRENT_WAY_CMD SENLCM_PELICAN_T_CURRENT_WAY_CMD
#define PELIC_GPS_DATA_ADVANCED_CMD SENLCM_PELICAN_T_GPS_DATA_ADVANCED_CMD
#define PELIC_CAM_DATA_CMD SENLCM_PELICAN_T_CAM_DATA_CMD

/**
 * Holds the preamble ">*>p" and type of struct from Asc Tech
 * documentation
 */
typedef struct pelic_query_t
{
    char preamble[4];
    uint16_t packetType;
} pelic_query_t;

/**
 * LowLevel (LL) Status - Taken strait from documentation
 */
typedef struct pelic_LlStatusStruct
{
  /**
   * battery voltages in mV
   */
  int16_t battery_voltage_1;

  /**
   * Not used for pelican
   */
  int16_t battery_voltage_2;

  /**
   * "Don't care" according to Asc Tech documentation
   */
  int16_t status;

  /**
   * Controller cycles per second (should be about 1000)
   */
  int16_t cpu_load;
  
  /**
   * is compass enabled?
   */
  int8_t compass_enabled;

  /**
   * checksum error?
   */
  int8_t chksum_error;

  /**
   * is the vehicle flying?
   */
  int8_t flying;

  /**
   * are the motors on?
   */
  int8_t motors_on;

  /**
   * Encoded flight mode.  This might be undocumented
   */

  int16_t flightMode;

  /**
   * Time motors are running (units unknown)
   */
  int16_t up_time;

} __attribute__((packed)) pelic_LlStatusStruct;

/**
 * IMU Calculated data - Taken strait from documentation
 */
typedef struct pelic_ImuCalcDataStruct
{
  /**
   * angles derived by integration of gyro_outputs, drift compensated by data
   * fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1
   * degree
   */
  int32_t angle_nick;
  /**
   * angles derived by integration of gyro_outputs, drift compensated by data
   * fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1
   * degree
   */
  int32_t angle_roll;
  /**
   * angles derived by integration of gyro_outputs, drift compensated by data
   * fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1
   * degree
   */
  int32_t angle_yaw;

  /**
   * angular velocities, raw values [16 bit] but bias free
   */
  int32_t angvel_nick;

  /**
   * angular velocities, raw values [16 bit] but bias free
   */
  int32_t angvel_roll;

  /**
   * angular velocities, raw values [16 bit] but bias free
   */
  int32_t angvel_yaw;

  /**
   * acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
   */
  short acc_x_calib;

  /**
   * acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
   */
  short acc_y_calib;

  /**
   * acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
   */
  short acc_z_calib;

  /**
   * horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
   */
  short acc_x;

  /**
   * horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
   */
  short acc_y;

  /**
   * horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
   */
  short acc_z;

  /**
   * reference angles derived by accelerations only: -90000..+90000; 1000 = 1
   * degree
   */
  int32_t acc_angle_nick;

  /**
   * reference angles derived by accelerations only: -90000..+90000; 1000 = 1
   * degree
   */
  int32_t acc_angle_roll;

  /**
   * total acceleration measured (10000 = 1g)
   */
  int32_t acc_absolute_value;
  
  /**
   * magnetic field sensors output, offset free and scaled; units not
   * determined, as only the direction of the field vector is taken into
   * account
   */
  int32_t Hx;

  /**
   * magnetic field sensors output, offset free and scaled; units not
   * determined, as only the direction of the field vector is taken into
   * account
   */
  int32_t Hy;

  /**
   * magnetic field sensors output, offset free and scaled; units not
   * determined, as only the direction of the field vector is taken into
   * account
   */
  int32_t Hz;

  /**
   * compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
   */
  int32_t mag_heading;

  /**
   * pseudo speed measurements: integrated accelerations, pulled towards zero;
   * units unknown; used for short-term position stabilization
   */
  int32_t speed_x;

  /**
   * pseudo speed measurements: integrated accelerations, pulled towards zero;
   * units unknown; used for short-term position stabilization
   */
  int32_t speed_y;

  /**
   * pseudo speed measurements: integrated accelerations, pulled towards zero;
   * units unknown; used for short-term position stabilization
   */
  int32_t speed_z;

  /**
   * height in mm (after data fusion)
   */
  int32_t height;

  /**
   * diff. height in mm/s (after data fusion)
   */
  int32_t dheight;

  /**
   * diff. height measured by the pressure sensor [mm/s]
   */
  int32_t dheight_reference;

  /**
   * height measured by the pressure sensor [mm]
   */
  int32_t height_reference;

} __attribute__((packed)) pelic_ImuCalcDataStruct;

/**
 * Remote Control (RC) Data : Taken strait from Asc Tech Documentation
 */
typedef struct pelic_RcDataStruct {

  /**
   * channels as read from R/C receiver
   */
  uint16_t channels_in[8];

  /**
   * channels bias free, remapped and scaled to 0..4095
   */
  uint16_t channels_out[8];

  /**
   * Indicator for valid R/C receiption
   */
  uint8_t lock;

} __attribute__((packed)) pelic_RcDataStruct;

/**
 * GPS Data : Taken strait from Asc Tech Documentation
 */
typedef struct pelic_GpsDataStruct
{
  /**
   * latitude in deg * 10ˆ7
   */
  int32_t latitude;

  /**
   * longitude in deg * 10ˆ7
   */
  int32_t longitude;

  /**
   * GPS height in mm
   */
  int32_t height;

  /**
   * speed in x (E/W) in mm/s
   */
  int32_t speed_x;

  /**
   * speed in y(N/S) in mm/s
   */
  int32_t speed_y;

  /**
   * GPS heading in deg * 1000
   */
  int32_t heading;

  /**
   * horizontal estimate in mm
   */
  uint32_t horizontal_accuracy;

  /**
   * vertical estimate in mm
   */
  uint32_t vertical_accuracy;

  /**
   * speed estimate mm/s
   */
  uint32_t speed_accuracy;

  /**
   * number of satellite vehicles used in NAV solution
   */
  uint32_t numSV;

  /**
   * GPS status information; 0x03 = valid GPS fix
   */
  int32_t status;
    
} __attribute__((packed)) pelic_GpsDataStruct;

/**
 * @brief Verify the checksum of received packet
 *
 * @param buf : packet
 * @param len : length of buf (bytes)
 *
 * @return 1 if checksum matches, 0 otherwise
 */
uint8_t
pelic_verifyChecksum (uint8_t *buf, int bufLen);

/**
 * @brief Handler for all received packets
 *
 * @param  buf : packet
 * @param  len : lenght of buf (bytes)
 * @param  pelic : pointer to senlcm_pelican_t
 *
 * @return None.  Calls the appropriate pelic_parseXXX function
 * depending on the packet descriptor (buf[5] : see manual)
 */
uint8_t
pelic_parsePacket (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic);

/**
 * Called if packet type is LL Status
 */
void
pelic_parseLlStatus (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic);

/**
 * Called if packet type is IMU Calc Data
 */
void
pelic_parseImuCalcData (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic);

/**
 * Called if packet type is RC Data
 */
void
pelic_parseRcData (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic);

/**
 * Called if packet type is GPS Data
 */
void
pelic_parseGpsData (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic);

/**
 * Taken from Asc Tech documentation : calculate the CRC
 */
uint16_t
pelic_crcUpdate (uint16_t crc, uint8_t data);
  
#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PELICAN_H__
