#ifndef __MICROSTRAIN_GX3_H__
#define __MICROSTRAIN_GX3_H__

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <arpa/inet.h>

#include "perls-common/units.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/getopt.h"
#include "perls-common/error.h"
#include "perls-common/units.h"
#include "perls-common/generic_sensor_driver.h"

#include "perls-lcmtypes/senlcm_ms_gx3_45_t.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MICROSTRAIN_MAX_LEN         1024
    
#define SYNC1                       0x75
#define SYNC2                       0x65
    
#define ACK_LEN                     10
#define CMD_OVERHEAD_LEN            6
#define FIELD_OVERHEAD_LEN          2

#define RATE_10HZ_b1                0x00
#define RATE_10HZ_b2                0x0A
#define RATE_20HZ_b1                0x00
#define RATE_20HZ_b2                0x05
#define RATE_100HZ_b1               0x00
#define RATE_100HZ_b2               0x01


// -----------------------------------------------------------------------------
// Base Command Set
// -----------------------------------------------------------------------------
#define BASE_CMD_SET                0x01
#define CMD_PING                    0x01    //!< Send a ping command
#define CMD_SET_TO_IDLE             0x02    //!< Place device into idle mode
#define CMD_SET_TO_IDLE_LEN         0
#define CMD_RESUME                  0x06    //!< Put in mode before idle was called (otherwise default)
#define CMD_GET_DEV_INFO            0x03    //!< Get device information (firmware, model, serial ect)
#define CMD_GED_DEV_DESC            0x04    //!< Get device descriptors 
#define CMD_BUILT_IN_TEST           0x05    //!< Run a bilt in test
#define CMD_RESET                   0x7E    //!< Reset device


// -----------------------------------------------------------------------------
// 3DM Command Set (common to MicroStrain Inertial Sensors with MIP packet protocol)
// -----------------------------------------------------------------------------
#define MS3DM_CMD_SET               0x0C
#define CMD_POLL_AHRS_DATA          0x01    //!< Poll AHRS data using given format       
#define CMD_POLL_GPS_DATA           0x02    //!< Poll GPS data
#define CMD_POLL_NAV_DATA           0x03    //!< Poll NAV data (output of Kalman filter)
#define CMD_GET_AHRS_DATA_RATE      0x06    //!< Get the decimation base for the AHRS Data rate calcs
#define CMD_GET_GPS_DATA_RATE       0x07    //!< Get the decimation base for the GPS Data rate calcs
#define CMD_GET_NAV_DATA_RATE       0x0B    //!< Get the decimation base for the NAV Data rate calcs
#define CMD_AHRS_MSG_FMT            0x08    //!< Get or set the format of the AHRS data packet
#define CMD_GPS_MSG_FMT             0x09    //!< Get or set the format of the GPS data packet
#define CMD_NAV_MSG_FMT             0x0A    //!< Get or set the format of the NAV data packet
#define CMD_ED_CONT_DATA_STREAM     0x11    //!< Enable / disable continous data stream
#define CMD_DEV_START_SET           0x30    //!< Save load or reset default values for startup settings
#define CMD_AHRS_SIG_COND_SET       0x35    //!< Set, read, or save AHRS signal condidioning params (inc disable magnometer)
#define CMD_UART_BAUD_RATE          0x40    //!< Set, read, or save uart baud rate
#define CMD_DEV_STATUS              0x64    //!< Get device specific status

// -----------------------------------------------------------------------------
// Navigation Filter Command Set (Specific to MS inertial navigation sensors)
// -----------------------------------------------------------------------------
#define NAV_CMD_SET                 0x0D
#define CMD_RESET_FILTER            0x01    //!< Reset filter (if you dont have auto-init enabled you will need to give an initial attitude or heading)
#define CMD_SET_INIT_ATT            0x02    //!< Set the initial attitude
#define CMD_SET_INIT_HEADING        0x03    //!< Set the initial heading angle
#define CMD_SET_INIT_ATT_AHRS       0x04    //!< Set the initial attitude from the AHRS (magnetometer must be on, with interference can cause filter to diverge)
#define CMD_SET_VEHICLE_DYNAMICS    0x10    //!< Set the vehicle dynamics mode (portable, automotive, and airborne)
#define CMD_ROT_SENSOR_VEHICLE      0x11    //!< Set the sensor to vehicle rotation (warning: effects many other commands)
#define CMD_TRANS_SENSOR_VEHICLE    0x12    //!< Set the sensor to vehicle translation in sensor frame (warning: effects many other commands)
#define CMD_ANTENNA_OFFSET          0x13    //!< Set the INS to GPS antenna translation (in sensor frame)
#define CMD_BIAS_EST_CTRL           0x14    //!< control calculation of sensor biases
#define CMD_GPS_SRC_CTRL            0x15    //!< control source of GPS information (can use external source)
#define CMD_EXT_GPS_UPDATE          0x16    //!< External GPS update
#define CMD_EXT_HEADING_UPDATE      0x17    //!< Trigger filter update with external heading (reference to NED)
#define CMD_HEADING_UPDATE_CTRL     0x18    //!< Set source for heading updates in KF (GPS velocity, Magnometer, external)
#define CMD_AUTO_INIT_CTRL          0x19    //!< Setup auto initialization for the filter
#define CMD_ACCEL_NOISE_STD         0x1A    //!< Set the accelerometer white noise standard deivation
#define CMD_GYRO_NOISE_STD          0x1B    //!< Set the gyroscope white noise standard deivation
#define CMD_GYRO_BIAS_MODEL_PARAM   0x1D    //!< Set the gyroscope bias model parameters


// -----------------------------------------------------------------------------
// System Command Set (advanced commands for devices with multiple intelligent internal sensor blocks)
// Most likely will not need to use this
// -----------------------------------------------------------------------------
#define SYS_CMD_SET                 0x7F
#define CMD_COMM_MODE               0x10    //!< set communications mode (can speak directly to AHRS and GPS, will act just like gx3-25)

// -----------------------------------------------------------------------------
// AHRS Data
// -----------------------------------------------------------------------------
#define DATA_AHRS_SET                0x80
#define DATA_SCALED_ACCEL            0x04    //!< scaled accelorometer vector
#define DATA_SCALED_GYRO             0x05    //!< scaled gyro vector
#define DATA_SCALED_MAG              0x05    //!< scaled magnetometer vector
#define DATA_DELTA_THETA             0x07    //!< time integral of angular rate
#define DATA_DELTA_VEL               0x08    //!< time integral of velocity
#define DATA_ORIENT_MAT              0x09    //!< orientation matrix
#define DATA_ORIENT_QUAT             0x0A    //!< orientation quarternion
#define DATA_EULER_ANGLES            0x0C    //!< euler Angles
#define DATA_AHRS_GPS_TIMESTAMP      0x12    //!< GPS correlation timestamp

// -----------------------------------------------------------------------------
// GPS Data
// -----------------------------------------------------------------------------
#define DATA_GPS_SET                 0x81
#define DATA_LLH_POS                 0x03    //!< Lat Lng Height positon data
#define DATA_NED_VEL                 0x05    //!< NED velocities
#define DATA_UTC_TIME                0x08    //!< UTC time
#define DATA_GPS_TIME                0x09    //!< GPS time
#define DATA_HWD_STATUS              0x0D    //!< Hardware status

// -----------------------------------------------------------------------------
// NAV Data
// -----------------------------------------------------------------------------
#define DATA_NAV_SET                 0x82
#define DATA_FILTER_STATUS           0x10    //!< Kalman Filter Status
#define DATA_NAV_GPS_TIMESTAMP       0x11    //!< GPS correlation timestamp
#define DATA_EST_LLH_POS             0x01    //!< Lat Lng Height estimated position
#define DATA_EST_NED_VEL             0x02    //!< estimated NED velocities
#define DATA_EST_ORIENT_QUAT         0x03    //!< estimated quarternion orientation
#define DATA_EST_ORIENT_MAT          0x04    //!< estimated orientation matrix
#define DATA_EST_EULER_ANGLES        0x05    //!< estimated orientation euler angles
#define DATA_EST_GYRO_BIAS           0x06    //!< estimated gyro biases
#define DATA_EST_LLH_POS_STD         0x08    //!< estimated position uncertianty 1 std
#define DATA_EST_NED_VEL_STD         0x09    //!< estimated velocity uncertianty 1 std
#define DATA_EST_EULER_STD           0x0A    //!< estimated euler angle uncertianty 1 std
#define DATA_EST_GYRO_BIAS_STD       0x0B    //!< estimated gyro bias
#define DATA_EST_LIN_ACCEL           0x0D    //!< estimated linear acceleration
#define DATA_EST_ANG_RATE            0x0E    //!< estimated angular rates
#define DATA_WGS84_LOCAL_GRAV_MAG    0x0F    //!< local gravity magnitude
#define DATA_EST_QUAT_STD            0x12    //!< estimated 
#define DATA_EST_GRAV_VEC            0x13    //!< estimated 
#define DATA_HEADING_UP_SRC_STATE    0x14    //!< heading update source state
#define DATA_MAG_MODEL_SOL           0x15    //!< magnetic model solution


/* returns checksum */
uint16_t
ms_compute_checksum (const uint8_t *buf, int buflen);


typedef struct _ms_mip_cmd_t ms_mip_cmd_t;
struct _ms_mip_cmd_t {
    uint8_t set_desc;
    uint8_t cmd_desc;
    uint8_t field_len;
    uint8_t *field_data;
};


int
ms_send_command (generic_sensor_driver_t *gsd, ms_mip_cmd_t cmd, int wait_ack);

void
ms_setup (generic_sensor_driver_t *gsd);

int
ms_verify_checksum (uint8_t *buf, int len);

static void
ms_raw_to_double(uint8_t *raw, double *d, int num_doubles);

static void
ms_raw_to_float(uint8_t *raw, float *d, int num_floats);

uint16_t
ms_raw_to_uint16_t (uint8_t *raw);

int
ms_parse_nav_packet (generic_sensor_driver_t *gsd, senlcm_ms_gx3_45_t* msg_out, uint8_t *buf, int len);



#ifdef __cplusplus
}
#endif

#endif //__MICROSTRAIN_GX3_H__
