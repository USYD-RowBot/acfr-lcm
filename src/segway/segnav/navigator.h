#ifndef __PERLS_SEG_NAVIGATOR_H__
#define __PERLS_SEG_NAVIGATOR_H__

#include "perls-common/bot_util.h"
#include "perls-common/getopt.h"

#include "perls-est/est.h"
#include "perls-est/est_pmf/est_pmf.h"  //process model functions
#include "perls-est/est_omf/est_omf.h"  //observation model functions

#define DT_EPS_SEC 0.005          // dont predict unless at least this much time has elapsed (200 hz)
#define MAX_DELAYED_STATES  10     // number of delayed states to maintain 

#define SEGWAY_STATE_CHANNEL "SEGWAY_STATE"

#define INIT_SAMPLES_RPH 5
#define INIT_SAMPLES_ABC 5

#define INIT_SENSORKEY_RPH    "sensors.ms-gx3"
#define INIT_SENSORKEY_ABC    "sensors.ms-gx3"

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
    char  *channel_drop_ds;
    char  *channel_drop_ds_ack;

    char *hb1_channel;
    char *hb5_channel;
    char *hb10_channel;
    char *hb20_channel;
    char *hb30_channel;
    char *hb40_channel;
    char *hb50_channel;
    
    char *kvh_channel;
    
    est_estimator_t *estimator;
    
    int debug;  //flag if debugging is on
    
    int filter_running;
    int init_data_ready;
    
    int done;
    int is_daemon;
    
    // we won't be using gps but sometimes we need a rough idea of where we are
    // ie earth rate compensation for KVH
    double org_lat;
    double org_lon;
    
    //config params
    BotParam *param;    //config file handle
    getopt_t *gopt;     //getopt

    int max_pm_dt;      //maximum time in microseconds to wait for measurement before timeout and predict
    char *est_engine;   //string name of estimation engine
    
    // pointer to prediction structure
    est_pred_t *pred_t;
    perllcm_est_navigator_index_t *index_t;
    void (*pred_user_t_free) (void *user);
    
    // measurement to process
    est_meas_t *meas_t_next;
    
    // time of last update to filter
    int64_t utime_last_update;
    int64_t utime_last_heartbeat;
    int64_t utime_delayed_states[MAX_DELAYED_STATES+1];
    int idx_last_graph_node;
    
    // init information
    int init_rph_cnt;
    double init_rph[3][INIT_SAMPLES_RPH];
    int init_abc_cnt;
    double init_abc[3][INIT_SAMPLES_ABC];
    
    // control parameters
    double current_kvh_angle;       // most recent kvh angle
    double last_predict_kvh_angle;  // kvh angle at last predict
    double current_wl;              // most recent wheel velocity (left)
    double current_wr;              // most recent wheel velocity (right)
    
};


// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

double
lat_to_earth_rate (const double latitude_rad);

void
push_utime_delayed_states (int64_t *utime_ds, int64_t utime);

void
remove_utime_delayed_states (int64_t *utime_ds, int index);

#endif //__PERLS_SEG_NAVIGATOR_H__
