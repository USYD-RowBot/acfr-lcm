#ifndef __LIBAUV_RDI_H__
#define __LIBAUV_RDI_H__

#include <stdint.h>

#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd0_t.h"
#include "perls-lcmtypes/senlcm_rdi_bathy_t.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _rdi_pd_mode rdi_pd_mode_t;
enum _rdi_pd_mode {
    RDI_PD0_MODE=0, 
    RDI_PD4_MODE=4, 
    RDI_PD5_MODE=5, 
    RDI_PD6_MODE=6
};

#define RDI_PD0_HEADER     0x7f
#define RDI_PD45_HEADER    0x7d
#define RDI_PD4_LEN        47
#define RDI_PD5_LEN        88
#define RDI_PD4_DATA_ID    0
#define RDI_PD5_DATA_ID    1

/* To auto append line continuation in Emacs: C-c C-\
*/
#define RDI_PD45_COMMON                                                 \
    /* Stores the ExplorerDVL (speed log) identification word (0x7d).   \
    */                                                                  \
    uint8_t header_id;                                                  \
                                                                        \
    /* Identifies which data pattern will follow based on the PD-command. \
       0 = PD4                                                          \
       1 = PD5                                                          \
    */                                                                  \
    uint8_t data_id;                                                    \
                                                                        \
    /* Contains the number of bytes sent in this data structure, not including \
       the final checksum.                                              \
    */                                                                  \
    int16_t nbytes;                                                     \
                                                                        \
    /* Defines the ExplorerDVL hardware/firmware configuration.  Convert to \
       binary and interpret as follows.                                 \
                                                                        \
       BIT 76543210                                                     \
           00xxxxxx  NO TRANSFORMATION (BEAM COORDINATES)               \
           01xxxxxx  INSTRUMENT COORDINATES                             \
           10xxxxxx  SHIP COORDINATES                                   \
           11xxxxxx  EARTH COORDINATES
           xx0xxxxx  TILT INFORMATION NOT USED IN CALCULATIONS          \
           xx1xxxxx  TILT INFORMATION USED IN CALCULATIONS              \
           xxx0xxxx  3-BEAM SOLUTIONS NOT COMPUTED                      \
           xxx1xxxx  3-BEAM SOLUTIONS COMPUTED                          \
           xxxxx010  300-kHz  ExplorerDVL                               \
           xxxxx011  600-kHz  ExplorerDVL                               \
           xxxxx100  1200-kHz ExplorerDVL                               \
    */                                                                  \
    uint8_t system_config;                                              \
                                                                        \
    /* These fields contain the velocity of the vessel in relation to the \
       bottom in mm/s.  Positive values indicate vessel motion to the east (X), \
       north (Y), and up (Z).  When a velocity is bad, the value is set to \
       -32768 (0x8000).                                                 \
                                                                        \
       LSD = 1 mm/s                                                     \
    */                                                                  \
    int16_t btv[4]; /* [x,y,z,error] */                                 \
                                                                        \
    /* These fields contain the vertical range from the ExplorerDVL to  \
       the bottom as determined by each beam.  This vertical range      \
       does not compensate for the effects of pitch and roll.  When a   \
       bottom detection is bad, the field is set to zero.               \
                                                                        \
       Scaling: LSD = 1 cm; Range = 0 to 65535 cm                       \
     */                                                                 \
    uint16_t altitude[4]; /* [b1,b2,b3,b4] */                           \
                                                                        \
    /* This field shows the status of bottom-referenced correlation and \
       echo amplitude data.  Convert to binary and interpret as follows. \
       A zero code indicates status is ok.                              \
                                                                        \
       BIT 76543210                                                     \
       1xxxxxxx  BEAM 4 LOW ECHO AMPLITUDE                              \
       x1xxxxxx  BEAM 4 LOW CORRELATION                                 \
       xx1xxxxx  BEAM 3 LOW ECHO AMPLITUDE                              \
       xxx1xxxx  BEAM 3 LOW CORRELATION                                 \
       xxxx1xxx  BEAM 2 LOW ECHO AMPLITUDE                              \
       xxxxx1xx  BEAM 2 LOW CORRELATION                                 \
       xxxxxx1x  BEAM 1 LOW ECHO AMPLITUDE                              \
       xxxxxxx1  BEAM 1 LOW CORRELATION                                 \
    */                                                                  \
    uint8_t btv_status;                                                 \
                                                                        \
    /* These fields contain the velocity of the vessel in relation to the \
       water-mass reference layer in mm/s.  Positive values indicate vessel \
       motion to east (X), north (Y), and up (Z).  When a velocity is bad, \
       the value is set to -32768 (0x8000).                             \
                                                                        \
       LSD = 1 mm/s                                                     \
    */                                                                  \
    int16_t wtv[4]; /* [x,y,z,error] */                                 \
                                                                        \
    /* These fields contain the starting boundary (near surface) and    \
       the ending boundary (near bottom) of the water-mass layer        \
       (BL-command).  If the minimum size field is zero, the ExplorerDVL \
       does not calculate water mass data.                              \
                                                                        \
       Scaling: LSD = 1 dm; Range = 0-9999 dm                           \
    */                                                                  \
    uint16_t wtv_layer_start;                                           \
    uint16_t wtv_layer_end;                                             \
                                                                        \
    /* This field shows the status of water mass depth and correlation  \
       data.  Convert to binary and interpret as follows.  A zero code  \
       indicates status is ok.                                          \
                                                                        \
       BIT 76543210                                                     \
           xxx1xxxx  ALTITUDE IS TOO SHALLOW                            \
           xxxx1xxx  BEAM 4 LOW CORRELATION                             \
           xxxxx1xx  BEAM 3 LOW CORRELATION                             \
           xxxxxx1x  BEAM 2 LOW CORRELATION                             \
           xxxxxxx1  BEAM 1 LOW CORRELATION                             \
    */                                                                  \
    uint8_t wtv_status;                                                 \
                                                                        \
    /* These fields contain the time of the first ping (TOFP) of the    \
       current ensemble.                                                \
    */                                                                  \
    int8_t tofp_hour;                                                   \
    int8_t tofp_minute;                                                 \
    int8_t tofp_second;                                                 \
    int8_t tofp_hundredth;                                              \
                                                                        \
    /* These fields contain the results of the ExplorerDVL's built-in   \
       test function.  A zero code indicates a successful BIT result.   \
                                                                        \
       BYTE 40  BYTE 41 (BYTE 41 RESERVED FOR FUTURE USE)               \
       1xxxxxxx xxxxxxxx = RESERVED                                     \
       x1xxxxxx xxxxxxxx = RESERVED                                     \
       xx1xxxxx xxxxxxxx = RESERVED                                     \
       xxx1xxxx xxxxxxxx = DEMOD 1 ERROR                                \
       xxxx1xxx xxxxxxxx = DEMOD 0 ERROR                                \
       xxxxx1xx xxxxxxxx = RESERVED                                     \
       xxxxxx1x xxxxxxxx = DSP ERROR                                    \
       xxxxxxx1 xxxxxxxx = RESERVED                                     \
    */                                                                  \
    uint16_t builtin_test;                                              \
                                                                        \
    /* Contains either manual or calculated speed of sound information  \
       (EC - Speed of Sound)                                            \
                                                                        \
       Scaling: LSD = 1 m/s;  Range = 1400-1600 m/s                     \
    */                                                                  \
    uint16_t speed_of_sound;                                            \
                                                                        \
    /* Contains the temperature of the water at the transducer head.    \
                                                                        \
       Scaling: LSD = 0.01 C; Range = -500 to +4000                     \
    */                                                                  \
    int16_t xducer_head_temp;


typedef struct _rdi_pd4 rdi_pd4_t;
struct _rdi_pd4 {
    RDI_PD45_COMMON;

    /* This field contains a modulo 65536 checksum.  The ExplorerDVL
       computes the checksum by summing all the bytes in the output
       buffer excluding the checksum.
    */
    uint16_t checksum;

} __attribute__ ((packed));


typedef struct _rdi_pd5 rdi_pd5_t;
struct _rdi_pd5 {
    RDI_PD45_COMMON;

    /* Contains the salinity value of the water at the transducer head.  This
       value may be a manual setting or a reading from a conductivity sensor

       Scaling: LSD = 1 part per thousand; Range = 0 to 40 ppt
    */
    int8_t salinity;

    /* Contains the depth of the transducer below the water surface
       (ED - Depth of Transducer).  This value may be a manual setting or
       a reading from a depth sensor

       Scaling: LSD = 1 dm; Range = 1 to 9999 dm
    */
    uint16_t depth;

    /* Contains the ExplorerDVL pitch angle (EP - Pitch and Roll Angles).
       This value may be a manual setting or a reading from a tilt sensor.
       Positive values mean that Beam #3 is spatially higher than Beam #4.

       Scaling: LSD = 0.01 degree; Range = -20.00 to +20.00 degrees
    */
    int16_t pitch;

    /* Contains the ExplorerDVL roll angle (ER - Roll Angle).  This value 
       may be a manual setting or a reading from a tilt sensor.  For up-facing
       ExplorerDVLs, positive values mean that Beam #2 is spatially higher than
       Beam #1.  For down-facing ExplorerDVLs, positive values mean that Beam #1
       is spatially higher than Beam #2.

       Scaling: LSD = 0.01 degree; Range = -20.00 to +20.00 degrees
    */
    int16_t roll;

    /* Contains the ExplorerDVL heading angle (EH - Heading).  This value may
       be a manual setting or a reading from a heading sensor.

       Scaling: LSD = 0.01 degree; Range = 000.00 to 359.99 degrees
    */
    uint16_t heading;

    /* Contains the Distance Made Good (DMG) over the bottom since the time
       of the first ping after initalization or <BREAK>.

       Scaling: LSD = 1 dm; Range = -10,000,000 to 10,000,000 dm
    */
    int32_t dmg_btv[4]; /* [East, North, Up, Error] */

    /* Contain the distance made good over the water-mass layer since the
       time ofthe first ping after initialization or <BREAK>.
       
       Scaling: LSD = 1 dm; Range = -10,000,000 to 10,000,000 dm
    */
    int32_t dmg_wtv[4]; /* [East, North, Up, Error] */

    /* This field contains a modulo 65536 checksum.  The ExplorerDVL
       computes the checksum by summing all the bytes in the output
       buffer excluding the checksum.
    */
    uint16_t checksum;

} __attribute__ ((packed));


// the PD0 message is dynamic in size so we need multiple structures for the different parts

// start of the header
typedef struct 
{
    uint8_t header_id;      
    uint8_t data_id;
    int16_t nbytes;        
    uint8_t spare;      
    uint8_t num_data_types;
    uint16_t *offsets;
} __attribute__ ((packed)) rdi_pd0_header_t;      

// PD0 fixed leader
typedef struct 
{
    uint16_t leader_id;
    uint8_t cpu_fw_ver;
    uint8_t cpu_fw_rev;
    uint16_t system_config;
    uint8_t sim_flag;
    uint8_t spare1;
    uint8_t num_beams;
    uint8_t num_cells;
    int16_t pings_per_ens;
    int16_t depth_cell_length;
    int16_t blank_after_xmit;   
    uint8_t profiling_mode;
    uint8_t low_corr_threshold;
    uint8_t num_code_reps;
    uint8_t gd_min;
    int16_t err_vel_max;
    uint8_t tpp_mins;
    uint8_t tpp_secs;
    uint8_t tpp_hund;
    uint8_t coord_trans;
    int16_t heading_align;
    int16_t heading_bias;
    uint8_t sensor_source;
    uint8_t sensors_avail;
    int16_t bin1_dist;
    int16_t xmit_pulse_len;
    int16_t wp_ref_avg;
    uint8_t false_target;
    uint8_t spare2;
    int16_t xmit_lag_dist;
    uint8_t cpu_serial[8];
    int16_t sys_bw;
    uint8_t spare3[6];
} __attribute__ ((packed)) rdi_pd0_fixed_leader_t;

// PD0 variable leader
typedef struct 
{
    uint16_t leader_id;
    int16_t ens_num;
    uint8_t rtc_year;
    uint8_t rtc_month;
    uint8_t rtc_day;
    uint8_t rtc_hour;
    uint8_t rtc_min;
    uint8_t rtc_sec;
    uint8_t rtc_hund;
    uint8_t ens_msb;
    uint16_t bit_result;
    uint16_t sos;
    uint16_t depth;
    uint16_t heading;
    uint16_t pitch;
    uint16_t roll;
    uint16_t salinity;
    uint16_t temp;
    uint8_t mpt_min;
    uint8_t mpt_sec;
    uint8_t mpt_hund;
    uint8_t head_std;
    uint8_t pitch_std;
    uint8_t roll_std;
    uint8_t adc[8];
    uint32_t error;
    uint16_t spare1;
    uint32_t pressure;
    uint32_t press_var;
    uint16_t spare2;
//    uint8_t rtc_2k_cent;
//    uint8_t rtc_2k_year;
//    uint8_t rtc_2k_mon;
//    uint8_t rtc_2k_day;
//    uint8_t rtc_2k_hour;
//    uint8_t rtc_2k_min;
//    uint8_t rtc_2k_sec;
//    uint8_t rtc_2k_hund;
} __attribute__ ((packed)) rdi_pd0_variable_leader_t;    

typedef struct 
{
    int16_t d[4];
} vel_cell_t;    

typedef struct 
{
    uint16_t id;
    int16_t *vel;
} __attribute__ ((packed)) rdi_pd0_velocity_t;

/*
typedef struct 
{
    uint16_t id;
    cell_t *cell;
} pd0_cep_t;
*/    
typedef struct 
{
    uint16_t bt_id;
    uint16_t pings_per_ens;
    uint16_t delay_before_reacq;
    uint8_t corr_mag_min;
    uint8_t eval_amp_min;
    uint8_t percent_good_min;
    uint8_t mode;
    uint16_t err_vel_max;
    uint32_t reserved1;
    uint16_t range[4];
    uint16_t vel[4];
    uint8_t corr[4];
    uint8_t eval_amp[4];
    uint8_t percent_good[4];
    uint16_t ref_layer_min;
    uint16_t ref_layer_near;
    uint16_t ref_layer_vel[4];
    uint8_t ref_corr[4];
    uint8_t ref_int[4];
    uint8_t ref_percent_good[4];
    uint16_t max_depth;
    uint8_t rssi_amp[4];
    uint8_t gain;
    uint8_t range_msb[4];
    uint8_t reserved2[4];
} rdi_pd0_bt_t;    



typedef struct
{
    rdi_pd0_header_t header;
    rdi_pd0_fixed_leader_t fixed;
    rdi_pd0_variable_leader_t variable;
    rdi_pd0_velocity_t velocity;
} rdi_pd0_t;    
    

/* Returns 0 pass, -1 error */
int
rdi_verify_checksum (const char *buf, int len);

/* Returns 0 pass, -1 error */
int
rdi_parse_pd4 (const char *buf, int len, rdi_pd4_t *pd4);

/* Returns 0 pass, -1 error */
int
rdi_parse_pd5 (const char *buf, int len, rdi_pd5_t *pd5);

/* Returns 0 pass, -1 error */
int
rdi_parse_pd0 (const char *buf, int len, rdi_pd0_t *pd0);

void free_rdi_pd0(rdi_pd0_t *pd0);

/*===============================================================================================
 * LCM
 *===============================================================================================*/

senlcm_rdi_pd4_t
rdi_pd4_to_lcm_pd4 (const rdi_pd4_t *pd4);

senlcm_rdi_pd5_t
rdi_pd5_to_lcm_pd5 (const rdi_pd5_t *pd5);

senlcm_rdi_pd0_t
rdi_pd0_to_lcm_pd0 (const rdi_pd0_t *pd0);


/* Computes RDI bathymetry in reference frame w
 * r1 to r4 - slant range as measured along beams 1 thru 4
 * x_ws     - 6x1 vector [x,y,z,r,p,h] of sensor pose w.r.t. frame w, 
              (set to all or zeros or NULL to report result w.r.t. sensor ref frame)
 */
senlcm_rdi_bathy_t
rdi_bathy (double r1, double r2, double r3, double r4, const double x_ws[6]);

#ifdef __cplusplus
}
#endif

#endif //__LIBAUV_RDI_H__
