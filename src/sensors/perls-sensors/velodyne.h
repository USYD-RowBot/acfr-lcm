#ifndef __PERLS_COMMON_VELODYNE_H__
#define __PERLS_COMMON_VELODYNE_H__

#include <stdio.h>
#include <stdint.h>
#include <glib.h>

#include "perls-math/fasttrig.h"
#include "perls-lcmtypes/perllcm_position_t.h"
#include "perls-lcmtypes/perllcm_velodyne_laser_return_collection_t.h"
#include "perls-lcmtypes/perllcm_velodyne_laser_return_t.h"
#include "perls-lcmtypes/perllcm_velodyne_laser_return_lite_t.h"

/**
 * @defgroup PerlsCommonVelodyne Velodyne Utility Functions
 * @brief Code to parse and motion compensate velodyne data packets 
 * @ingroup PerlsCommon
 * @include: perls-common/velodyne.h
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define VELODYNE_MIN_RANGE 1.0

#define VELODYNE_HDL_32E_MODEL_STR      "HDL_32E"
#define VELODYNE_HDL_64E_S1_MODEL_STR   "HDL_64E_S1"
#define VELODYNE_HDL_64E_S2_MODEL_STR   "HDL_64E_S2"
#define VELODYNE64_NUM_LASERS            64      
#define VELODYNE32_NUM_LASERS            32

#define VELODYNE_RADIANS_PER_LSB 0.00017453293
#define VELODYNE_METERS_PER_LSB  0.002
#define VELODYNE_SPIN_RATE       (10*360/1e6*UNITS_DEGREE_TO_RADIAN) //10 Hz = 3600/1e6*DTOR rads/usec
#define VELODYNE_INTRA_SHOT_USEC (4) //4 microseconds
#define VELODYNE_PACKETS_PER_REV 229

#define VELODYNE_POSITION_PORT          8308    //!< UDP port for data packet
#define VELODYNE_DATA_PORT              2368    //!< UDP port for position packet
#define VELODYNE_DATA_PACKET_LEN        1206    //!< UDP length packet lenght in bytes
#define VELODYNE_POSITION_PACKET_LEN    512     //!< UDP position packet lenght in bytes

#define VELODYNE_UPPER_START_IDENTIFIER 0xeeff
#define VELODYNE_LOWER_START_IDENTIFIER 0xddff

// defines for indicies and lengths in data packet
#define VELODYNE_DATA_TIMESTAMP_START         (1200)           //!< Index of the start of the timestamp
#define VELODYNE_DATA_FIRING_START(i_f)       (100*i_f)        //!< Index of the start of firing given the firing ind (i_f=0...11)
#define VELODYNE_DATA_LASER_START(i_f, i_l)   (100*i_f+4+i_l*3)//!< Index of the start of laser give the firing ind (i_f=0...11) and laser ind (i_l=0..31)
#define VELODYNE_FIRING_BLOCK_LEN              100             //!< Length of a single firing block data
#define VELODYNE_NUM_FIRING_PER_PACKET        (12)             //!< Number of firings per packet
#define VELODYNE_NUM_LASERS_PER_FIRING        (32)             //!< Number of laser returns per firing
#define VELODYNE_NUM_LASER_RETURNS_PER_PACKET (12*32)          //!< Number of laser returns in a single data packet

// defines for location of quantities ind position packet
#define VELODYNE_POSITION_TIMESTAMP_START     (14 + 24 + 160)   //!< Packet index of the start the timestamp  

/**
 * Get the timestamp in usec (since the top of the hour) from the data packet
 */
#define VELODYNE_GET_TIMESTAMP_USEC(data)  (                            \
        (data[VELODYNE_DATA_TIMESTAMP_START]<<0)  +                     \
        (data[VELODYNE_DATA_TIMESTAMP_START+1]<<8) +                    \
        (data[VELODYNE_DATA_TIMESTAMP_START+2]<<16) +                   \
        (data[VELODYNE_DATA_TIMESTAMP_START+3]<<24)                     \
        ) 

/**
 * Get start indentifier
 */
#define VELODYNE_GET_START_IDENTIFIER(data, start) (                    \
        data[start] + (data[start+1]<<8)                                \
        )

/**
 * Get the rotational position from firing start
 */
#define VELODYNE_GET_ROT_POS(data, start)   (                           \
        (data[start+2] + (data[start+3]<<8)) * VELODYNE_RADIANS_PER_LSB \
        )

/**
 * Get the raw range from laser data start (VELODYNE_DATA_LASER_START)
 */
#define VELODYNE_GET_RANGE(data, start) (                               \
        (data[start] + (data[start+1]<<8)) * VELODYNE_METERS_PER_LSB    \
        )

/**
 * Get the raw intensity from laser data start (VELODYNE_DATA_LASER_START)
 */
#define VELODYNE_GET_INTENSITY(data, start) (   \
        data[start+2]                           \
        )

/**
 * Get the offset, in microseconds, between a laser firing in the packet and
 * the last laser firing in the packet
 * given the firing index (i_f=0...11) and laser index (i_l=0..31)
 *
 * see velodyne 32 manual pg 24
 */
#define VELODYNE_32_LASER_FIRING_TIME_OFFSET(if, il) (  \
        -542.592 + (i_f*46.08) + (i_l*1.152)            \
        )

/**
 * @brief enum of different Velodyne sensor models
 *
 */

typedef enum _velodyne_sensor_type_t {
    VELODYNE_SENSOR_TYPE_HDL_32E = 0,
    VELODYNE_SENSOR_TYPE_HDL_64E_S1 = 1,
    VELODYNE_SENSOR_TYPE_HDL_64E_S2 = 2,
} velodyne_sensor_type_t;


/**
 * @brief Structure used for laser return collection.
 * This structure is used to produce full or partial laser scans from multiple
 * data packets.
 */
typedef struct _velodyne_laser_return_collector_t  velodyne_laser_return_collector_t;
struct _velodyne_laser_return_collector_t {
    
    // setup params
    double     start_angle;         //!< setup: collect returns starting at this angle ()
    double     end_angle;           //!< setup: stop collecting returns at this angle
    uint8_t    whole_scan;          //!< setup: bool, collect whole scan, not segment
    uint8_t    collection_ready;    //!< flag if collecton is ready
    double     x_vs[6];             //!< vehicle to sensor transform, set if providing motion compensation in robots frame
    
    // internal
    int64_t             utime_first_laser;   //!< utime of first laser in this collection
    uint8_t             first_laser_has_pose;//!< flag if we have a pose for the first laser
    double              first_laser_pose[6]; //!< flag if we have a pose for the first laser
    perllcm_position_t  state;               //!< most recent sensor state (pose and motion)
    uint32_t            num_lr;              //!< number of laser returns
    GArray             *laser_returns;       //!< array of laser returns
    int64_t             utime_collection;    //!< utime that the collection is motion compensated to based on time of start angle
    uint8_t             collecting;          //!< flag if inside angle collecting data
    int                 packet_cnt;          //!< number of packets collected thus far
};

/**
 * @brief enum collector push returns
 *
 */
typedef enum _velodyne_collector_push_return_t {
    VELODYNE_COLLECTION_READY = 0,
    VELODYNE_COLLECTION_PUSH_OK = 1,
    VELODYNE_COLLECTION_PUSH_ERROR = 2,
} velodyne_collector_push_return_t;


/**
 * @brief Velodyne laser intrinsic calibration structure structure
 * 
 */
typedef struct _velodyne_laser_calib_t velodyne_laser_calib_t;
struct _velodyne_laser_calib_t {
    double rcf;                //!< rotational/yaw offset (radians)
    double vcf;                //!< vertical offset (radians)
    double hcf;                //!< horizontal off-axis offset (meters)
    double range_offset;       //!< range offset (meters)
    double voffset;            //!< vertical offset (meters)
    double range_scale_offset; //!< (scalar)
};

/**
 * @brief Velodyne intrinsic calibration structure structure
 *
 * @details
 * physical laser numbers -> index based on firing order
 * logical laser numbers -> index based on increasing angle
 *
 * 
 * @see velodyne_laser_calib
 */
typedef struct _velodyne_calib_t velodyne_calib_t;
struct _velodyne_calib_t {
    velodyne_sensor_type_t sensor_type; //!< the type of sensor (32 or 64)
    int num_lasers;                     //!< the number of lasers for this sensor
    velodyne_laser_calib_t *lasers;     //!< laser calibrations (physical laser index)
    int *physical2logical;              //!< mapping physical laser index to logical laser index
    int *logical2physical;              //!< mapping logical laser index to physical laser index
    double *va_sin;                     //!< cached sin of vertical angle of laser (physical laser index)
    double *va_cos;                     //!< cached cos of vertical angle of laser (physical laser index)
    int **calibrated_intensity;         //!< intensity calibration lookup table
};

/**
 * @brief create the velodyne_calib_t structure
 */
velodyne_calib_t *
velodyne_calib_create (velodyne_sensor_type_t sensor_type, const char *db_xml_file_path);

/**
 * @brief free the velodyne_calib_t structure
 */
void
velodyne_calib_free (velodyne_calib_t *calib);

/**
 * @brief This function reads the intrinsic intensity calibration file
 *
 * @see velodyne_calib_t
 */
void
velodyne_read_calibrated_intensity (velodyne_calib_t* calib, const char *db_xml_file_path);


/**
 * @brief This function decodes a single data packet
 * returns an allocated velodyne_laser_return_collection_t structure
 * user needs to free when done with
 *
 * @see velodyne_free_laser_return_collection
 */
perllcm_velodyne_laser_return_collection_t *
velodyne_decode_data_packet (velodyne_calib_t* calib, uint8_t *data, int data_len, int64_t utime);

/**
 * @brief This function decodes a single data packet
 * returns an allocated velodyne_laser_return_collection_t structure
 * user needs to free when done with.
 *
 * Includes option for user to specify sub-sampeling percentage [0.0 ... 1.0]
 *
 * @see velodyne_free_laser_return_collection
 */
perllcm_velodyne_laser_return_collection_t *
velodyne_decode_data_packet_subsamp (velodyne_calib_t* calib, uint8_t *data, int data_len, int64_t utime, double subsamp_pct);

/**
 * @brief Callocs a velodyne_laser_return_collection_t structure
 *
 */
perllcm_velodyne_laser_return_collection_t *
velodyne_calloc_laser_return_collection (int num_samples, int num_lite_samples);


/**
 * @brief Frees a velodyne_laser_return_collection_t structure
 *
 */
void
velodyne_free_laser_return_collection (perllcm_velodyne_laser_return_collection_t *lrc);


/**
 * @brief create the velodyne_laser_return_collector_t structure
 *
 * @details set whole_scan=1 to collect 1 full revelution, if whole_scan=0 then use
 * start_angle, and end_angle to specify segment to collect
 * also need to specify x_vs if you want to provide motion compensation in the robot's frame instead of the sensors
 * 
 */
velodyne_laser_return_collector_t *
velodyne_laser_return_collector_create (uint8_t whole_scan, double start_angle, double end_angle, double x_vs[6]);

/**
 * @brief free the velodyne_laser_return_collector_t structure
 */
void
velodyne_laser_return_collector_free (velodyne_laser_return_collector_t *collector);

/**
 * @brief push new laser return data onto a collector
 *
 * @return returns velodyne_collector_push_return_t signaling if collection is
 * ready
 * 
 */
velodyne_collector_push_return_t
velodyne_collector_push_laser_returns (velodyne_laser_return_collector_t *collector,
                                       perllcm_velodyne_laser_return_collection_t *new_returns);

/**
 * @brief push state data (pose and motion in world frame) onto a collector to be used for motion compensation.
 *
 * @details The user can push on world frame state, pose, velocities, and rates
 * Set unknown quantities to zero for no motion compensation
 *
 * @return returns velodyne_collector_push_return_t signaling if collection is
 * ready
 * 
 */
velodyne_collector_push_return_t
velodyne_collector_push_state (velodyne_laser_return_collector_t *collector,
                               perllcm_position_t state);

/**
 * @brief pull a completed collection from the collector
 *
 * @returns a motion compensated collection of laser returns, returns null if not avaliable.
 * 
 */
perllcm_velodyne_laser_return_collection_t *
velodyne_collector_pull_collection (velodyne_laser_return_collector_t *collector);




/**
 * @brief convert physical to logical laser index
 */
static inline int
velodyne_physical_to_logical (velodyne_calib_t *calib, int phys)
{
    return calib->physical2logical[phys];
}

/**
 * @brief convert logical to physical laser index
 */
static inline int
velodyne_logical_to_physical (velodyne_calib_t *calib, int logical)
{
    return calib->logical2physical[logical];
}

/**
 * @brief print the calibration parameters
 *
 * @see velodyne_calib_t
 */
void
velodyne_calib_dump (velodyne_calib_t *calib);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */


#endif // __PERLS_COMMON_VELODYNE_H__
