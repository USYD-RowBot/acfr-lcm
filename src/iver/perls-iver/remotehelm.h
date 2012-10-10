#ifndef __PERLS_IVER_REMOTEHELM_H__
#define __PERLS_IVER_REMOTEHELM_H__

#include <stdbool.h>
#include <sys/time.h>

#include "perls-lcmtypes/senlcm_uvc_ojw_t.h"
#include "perls-lcmtypes/senlcm_uvc_omload_t.h"
#include "perls-lcmtypes/senlcm_uvc_omp_t.h"
#include "perls-lcmtypes/senlcm_uvc_omstart_t.h"
#include "perls-lcmtypes/senlcm_uvc_omstop_t.h"
#include "perls-lcmtypes/senlcm_uvc_omw_t.h"
#include "perls-lcmtypes/senlcm_uvc_opos_t.h"

#include <bot_param/param_client.h>

#ifdef __cplusplus
extern "C" {
#endif

/* publish cmd to appropriate lcm channel 
 * if timeout = NULL returns 0, 
 * else timeout > 0, waits timeout for ack, ret -1 undefined, 0 success, 1 cmd
 * recieved in error */
int
iver_rh_pub_ojw (lcm_t *lcm, const char *channel, senlcm_uvc_ojw_t *cmd, BotParam *param, struct timeval *timeout);

int
iver_rh_pub_omp (lcm_t *lcm, const char *channel, senlcm_uvc_omp_t *cmd, BotParam *param, struct timeval *timeout);

int
iver_rh_pub_omload (lcm_t *lcm, const char *channel, senlcm_uvc_omload_t *cmd, BotParam *param, struct timeval *timeout);

int
iver_rh_pub_omstart (lcm_t *lcm, const char *channel, senlcm_uvc_omstart_t *cmd, BotParam *param, struct timeval *timeout);

int
iver_rh_pub_omstop (lcm_t *lcm, const char *channel, senlcm_uvc_omstop_t *cmd, BotParam *param, struct timeval *timeout);

int
iver_rh_pub_omw (lcm_t *lcm, const char *channel, senlcm_uvc_omw_t *cmd, BotParam *param, struct timeval *timeout);

int
iver_rh_pub_opos (lcm_t *lcm, const char *channel, senlcm_uvc_opos_t *cmd, BotParam *param, struct timeval *timeout);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_IVER_REMOTEHELM_H__

