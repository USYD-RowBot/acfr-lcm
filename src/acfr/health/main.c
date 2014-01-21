#include <stdio.h>
#include <signal.h>
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/senlcm_tcm_t.h"
#include "perls-lcmtypes/senlcm_kvh1750_t.h"
#include "perls-lcmtypes/senlcm_gpsd3_t.h"
#include "perls-lcmtypes/senlcm_ecopuck_t.h"
#include "perls-lcmtypes/senlcm_micron_ping_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/senlcm_parosci_t.h"

#include "perls-lcmtypes/acfrlcm_auv_status_t.h"



#define COMPASS_TIMEOUT 1000000
#define GPS_TIMEOUT 1000000
#define ECOPUCK_TIMEOUT 1000000
#define NAV_TIMEOUT 1000000
#define IMU_TIMEOUT 1000000
#define DVL_TIMEOUT 1000000
#define DEPTH_TIMEOUT 1000000
#define OAS_TIMEOUT 1000000

#define COMPASS_BIT 0x0010
#define GPS_BIT 0x0004
#define ECOPUCK_BIT 0x0100
#define NAV_BIT 0x0080
#define IMU_BIT 0x0020
#define DVL_BIT 0x0001
#define DVL_BL_BIT 0x0002
#define DEPTH_BIT 0x0008
#define OAS_BIT 0x0040


typedef struct
{
    int64_t compass;
    int64_t gps;
    int64_t ecopuck;
    int64_t nav;
    int64_t imu;
    int64_t dvl;
    int dvl_bl;
    int64_t depth;
    int64_t oas;
    
    lcm_t *lcm; 

} state_t;

// Handlers

void handle_tcm(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_tcm_t *sensor, void *u)
{
    state_t *state = (state_t *)u;
    state->compass = sensor->utime;
}

void handle_imu(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_kvh1750_t *sensor, void *u)
{
    state_t *state = (state_t *)u;
    state->imu = sensor->utime;
}

void handle_gps(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_gpsd3_t *sensor, void *u)
{
    state_t *state = (state_t *)u;
    state->gps = sensor->utime;
}

void handle_ecopuck(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_ecopuck_t *sensor, void *u)
{
    state_t *state = (state_t *)u;
    state->ecopuck = sensor->utime;
}

void handle_micron(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_micron_ping_t *sensor, void *u)
{
    state_t *state = (state_t *)u;
    state->oas = sensor->utime;
}

void handle_rdi(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_rdi_pd5_t *sensor, void *u)
{
    state_t *state = (state_t *)u;
    state->dvl = sensor->utime;
}

void handle_parosci(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_parosci_t *sensor, void *u)
{
    state_t *state = (state_t *)u;
    state->depth = sensor->utime;
}

void handle_nav(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_acfr_nav_t *sensor, void *u)
{
    state_t *state = (state_t *)u;
    state->nav = sensor->utime;
}

void handle_heartbeat(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    acfrlcm_auv_status_t status;
    memset(&status, 0, sizeof(acfrlcm_auv_status_t));
    status.utime = timestamp_now();
    
    // check the age of the sensor data
    if((hb->utime - state->compass) < COMPASS_TIMEOUT)
        status.status = COMPASS_BIT;
    
    if((hb->utime - state->gps) < GPS_TIMEOUT)
        status.status |= GPS_BIT;

    if((hb->utime - state->ecopuck) < ECOPUCK_TIMEOUT)
        status.status |= ECOPUCK_BIT;

    if((hb->utime - state->nav) < NAV_TIMEOUT)
        status.status |= NAV_BIT;

    if((hb->utime - state->imu) < IMU_TIMEOUT)
        status.status |= IMU_BIT;

    if((hb->utime - state->dvl) < DVL_TIMEOUT)
        status.status |= DVL_BIT;

    if((hb->utime - state->depth) < DEPTH_TIMEOUT)
        status.status |= DEPTH_BIT;

    if((hb->utime - state->oas) < OAS_TIMEOUT)
        status.status |= OAS_BIT;


    acfrlcm_auv_status_t_publish(state->lcm, "AUV_HEALTH", &status);


}



int program_exit;
void signal_handler(int sigNum) 
{
    // do a safe exit
    program_exit = 1;
}

int main(int argc, char **argv)
{
    state_t state;
    memset(&state, 0, sizeof(state_t));
    state.lcm = lcm_create(NULL);
    
    // Subscribe to all the sensors we need to monitor
    senlcm_tcm_t_subscribe(state.lcm, "TCM", &handle_tcm, &state);
    senlcm_kvh1750_t_subscribe(state.lcm, "KVH1550", &handle_imu, &state); 
    senlcm_gpsd3_t_subscribe(state.lcm, "GPS", &handle_gps, &state); 
    senlcm_ecopuck_t_subscribe(state.lcm, "ECOPUCK", &handle_ecopuck, &state); 
    senlcm_micron_ping_t_subscribe(state.lcm, "MICRON", &handle_micron, &state); 
    senlcm_rdi_pd5_t_subscribe(state.lcm, "RDI", &handle_rdi, &state);  
    senlcm_parosci_t_subscribe(state.lcm, "PAROSCI", &handle_parosci, &state); 
    acfrlcm_auv_acfr_nav_t_subscribe(state.lcm, "ACFR_NAV", &handle_nav, &state); 
    
    // Subscribe to the heartbeat
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &handle_heartbeat, &state); 
        
    // Loop
    struct timeval tv;
    while(!program_exit)
    {
	    tv.tv_sec = 1;
	    tv.tv_usec = 0;
        lcmu_handle_timeout(state.lcm, &tv);    
    }
    
    return 0;
}
