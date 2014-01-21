/* ==================================================== 
** acomms.[ch]
**
** Support functions for goby-acomms
** ====================================================
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>

#include "perls-common/units.h"

#include "lcm_listeners.h"


void
mission_status_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                         const perllcm_auv_mission_status_t *msg, void *user)
{
    state_t *state = (state_t*) user;

    g_mutex_lock (state->mutex);
    state->mission_running = msg->mission_running; 
    g_mutex_unlock (state->mutex);
}

void
osi_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_auv_os_conduit_osi_t *msg, void *user)
{
    state_t *state = (state_t*) user;

    g_mutex_lock (state->mutex);
    state->osi = *msg;
    g_mutex_unlock (state->mutex);
}

void
opi_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_auv_os_conduit_opi_t *msg, void *user)
{
    state_t *state = (state_t*) user;

    g_mutex_lock (state->mutex);
    state->opi = *msg;
    g_mutex_unlock (state->mutex);
}

void
os_conduit_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                const perllcm_auv_os_conduit_t *msg, void *user)
{
    state_t *state = (state_t*) user;

    g_mutex_lock (state->mutex);
    double heading, pitch, roll, temp, depth = 0;
    int ret = sscanf (msg->compass.value, "$C%lfP%lfR%lfT%lfD%lf", &heading, &pitch, &roll, &temp, &depth);
    if (ret == 5) {
        if (depth > 255)
            state->depth = 255;
        state->depth   = depth * UNITS_FEET_TO_METER;
        state->heading = heading * UNITS_DEGREE_TO_RADIAN;
    }
    else {
        state->depth   = 0;
        state->heading = 0;
    }
    g_mutex_unlock (state->mutex);
}
