#ifndef __HANDLERS_BOT_CORE_H__
#define __HANDLERS_BOT_CORE_H__

#include "perls-lcmtypes/lcmtypes.h"

void
bot_core_image_sync_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                               const bot_core_image_sync_t *msg, void *user);

#endif //__HANDLERS_BOT_CORE_H__
