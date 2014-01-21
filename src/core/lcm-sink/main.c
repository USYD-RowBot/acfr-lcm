#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <glib.h>
#include <lcm/lcm.h>

#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/glib_util.h"
#include "perls-common/timestamp.h"

typedef struct value value_t;
struct value {
    int64_t utime;
    int64_t jitter;
};

static void 
message_handler (const lcm_recv_buf_t *rbuf, const char *channel, void *user)
{
    GHashTable *hash = user;

    value_t *ov = g_hash_table_lookup (hash, channel);

    int64_t utime = timestamp_now ();
    int64_t utime_last = ov ? ov->utime : 0;
    value_t nv = {
        .utime = utime,
        .jitter = utime - utime_last,
    };
    g_hash_table_insert (hash, strdup (channel), gu_dup (&nv, sizeof nv));

    printf ("jitter=%-16"PRId64"  Hz=%-12g  %s\n", nv.jitter, 1./nv.jitter*1E6, channel);
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "A generic LCM sink for testing channel traffic");
    getopt_add_help (gopt, NULL);
    getopt_add_string (gopt, 'c', "channels", ".*", "Comma seperated list of LCM regex channel names to monitor");
    getopt_add_example (gopt,
                        "Monitor EASYDAQ and all HEARTBEAT channels\n"
                        "%s --channels EASYDAQ,HEARTBEAT_.*HZ", argv[0]);

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=0) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) {
        ERROR ("lcm_create()");
        exit (EXIT_FAILURE);
    }

    GHashTable *hash = g_hash_table_new_full (&g_str_hash, &g_str_equal, &g_free, &g_free);
    char **channels = g_strsplit (getopt_get_string (gopt, "channels"), ",", -1);
    for (int i=0; channels[i]!=NULL; i++) {
        const char *channel = channels[i];
        printf ("Adding channel %s\n", channel);
        lcm_subscribe (lcm, channel, message_handler, hash);
    }

    while (1)
        lcm_handle (lcm);
}
