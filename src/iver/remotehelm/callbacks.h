#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "perls-lcmtypes/perllcm_auv_abort_t.h"
#include "perls-lcmtypes/perllcm_auv_jump_t.h"
#include "perls-lcmtypes/perllcm_auv_mission_status_t.h"
#include "perls-lcmtypes/perllcm_auv_safety_rules_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/perllcm_auv_remotehelm_cmd_t.h"

#include "perls-lcmtypes/senlcm_easydaq_t.h"
#include "perls-lcmtypes/senlcm_uvc_ojw_t.h"
#include "perls-lcmtypes/senlcm_uvc_omload_t.h"
#include "perls-lcmtypes/senlcm_uvc_omstart_t.h"
#include "perls-lcmtypes/senlcm_uvc_omstop_t.h"
#include "perls-lcmtypes/senlcm_uvc_osi_t.h"

void 
perllcm_auv_remotehelm_cmd_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                       const perllcm_auv_remotehelm_cmd_t *msg, void *user);

void
hb1_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_heartbeat_t *msg, void *user);

void
safety_rules_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                       const perllcm_auv_safety_rules_t *rules, void *user);

void 
osi_callback (const lcm_recv_buf_t *rbuf, const char * channel, 
              const senlcm_uvc_osi_t * msg, void * user);

void
abort_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                const perllcm_auv_abort_t *msg, void *user);

void
jump_callback (const lcm_recv_buf_t *rbuf, const char *channel,
               const perllcm_auv_jump_t *msg, void *user);


void
easydaq_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_easydaq_t *msg, void *user);



