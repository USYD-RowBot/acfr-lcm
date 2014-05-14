#ifndef __PERLS_NAVIGATOR_H__
#define __PERLS_NAVIGATOR_H__

#include <bot_core/bot_core.h>

#include "perls-est/est.h"

#include "perls-common/getopt.h"

#define INIT_SAMPLES_XY  5
#define INIT_SAMPLES_Z   5
#define INIT_SAMPLES_RPH 20
#define INIT_SAMPLES_UVW 5
#define INIT_SAMPLES_ABC 20

//#define INIT_SENSORKEY_XY_OLD "sensors.gpsd-client"
#define INIT_SENSORKEY_XY     "sensors.gpsd3-client"
#define INIT_SENSORKEY_Z      "sensors.dstar-ssp1"
#define INIT_SENSORKEY_RPH    "sensors.ms-gx1"
#define INIT_SENSORKEY_UVW    "sensors.rdi"
#define INIT_SENSORKEY_ABC    "sensors.ms-gx1"

#define DT_EPS_SEC 0.005 

//------------------------------------------------------------------------------
// define navigatior state structure
//------------------------------------------------------------------------------
typedef struct _state_t state_t;
struct _state_t {  
    
    lcm_t *lcm;
    char  *prefix;
    char  *channel;
    char  *channel_debug_meas;
    char  *channel_debug_pred;

    char *hb1_channel;
    char *hb5_channel;
    char *hb10_channel;
    
    est_estimator_t *estimator;
    
    int debug;  //flag if debugging is on
    
    int done;
    int is_daemon;
    
    //config params
    BotParam *param;       //config file handle
    getopt_t *gopt;         //getopt

    int max_pm_dt;          //maximum time in microseconds to wait for measurement before timeout and predict
    char *est_engine;       //string name of estimation engine
    BotGPSLinearize *llxy;  //struct containing gps origin used to convert between local xy
    
    // pointer to prediction structure
    est_pred_t *pred_t;
    perllcm_est_navigator_index_t *index_t;
    void (*pred_user_t_free) (void *user);
    
    // measurement to process
    est_meas_t *meas_t_next;
    
    // time of last update to filter
    int64_t utime_last_update;
    int64_t utime_last_heartbeat;
    
    // initilization
    // navigator <-> os_remotehelm interaction
    // navigator waits till it has enough data to init then signals with
    //      init_data_ready = 1;
    // os-remote then starts mission and signals mission start with
    //      mission_running = 1;
    // navigator then starts the filter and signals
    //      filter_running = 1
    
    int init_data_ready;
    int filter_running;
    int os_remotehelm_waiting_to_start;
    int os_remotehelm_mission_running;

    int init_xy_cnt;    
    double init_xy[2][INIT_SAMPLES_XY];

    int init_z_cnt;
    double init_z[INIT_SAMPLES_Z];

    int init_rph_cnt;
    double init_rph[3][INIT_SAMPLES_RPH];

    int init_uvw_cnt;
    double init_uvw[3][INIT_SAMPLES_UVW];

    int init_abc_cnt;
    double init_abc[3][INIT_SAMPLES_ABC];

    // for testing
    // int64_t test_last_utime;

};


// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

double
lat_to_earth_rate (const double latitude_rad);


#endif //__PERLS_NAVIGATOR_H__
