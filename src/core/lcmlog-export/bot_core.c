#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "perls-common/textread.h"

#include "lcmlog_export.h"
#include "bot_core.h"

void
bot_core_image_sync_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                               const bot_core_image_sync_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime", "%"PRId64,  msg->utime);
    textread_stop (tr);
}
