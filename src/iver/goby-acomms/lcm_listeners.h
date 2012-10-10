#ifndef __LCM_LISTENERS_H__
#define __LCM_LISTENERS_H__

#include <iostream>
#include <glib.h>

#include "perls-lcmtypes/perllcm_auv_mission_status_t.h"
#include "perls-lcmtypes/perllcm_auv_os_conduit_opi_t.h"
#include "perls-lcmtypes/perllcm_auv_os_conduit_osi_t.h"
#include "perls-lcmtypes/perllcm_auv_os_conduit_t.h"


// acomms state struct
typedef struct _state_t state_t;
struct _state_t {
    lcm_t *lcm;

    GMutex *mutex;
    
    // lcm channel prefix
    char *prefix;

    // heartbeat channel
    char *hb_channel;

    // acommms channels
    char *acomms_channel;
    char *acomms_abort_channel;
    char *acomms_jump_channel;
    char *acomms_mini_channel;
    char *acomms_out_channel;
    char *acomms_owtt_channel;
    char *acomms_ping_channel;
    char *acomms_raw_channel;

    // os-remotehelm channels
    char *rh_abort_channel;
    char *rh_jump_channel;
    char *rh_mission_channel;

    // os-conduit
    char *osc_channel;
    char *osc_opi_channel;
    char *osc_osi_channel;
    char *osc_out_channel;
    
    perllcm_auv_os_conduit_osi_t osi;
    perllcm_auv_os_conduit_opi_t opi;

    double depth;
    double heading;

    bool mission_running;
    bool aborted_mission;
};


// lcm callback functions
void
mission_status_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                         const perllcm_auv_mission_status_t *msg, void *user);

void
osi_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_auv_os_conduit_osi_t *msg, void *user);

void
opi_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_auv_os_conduit_opi_t *msg, void *user);

void
os_conduit_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                     const perllcm_auv_os_conduit_t *msg, void *user);

#endif // __LCM_LISTENERS_H__
