#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <glib.h>

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#define MAX_TIMERS 10

typedef struct user user_t;
struct user {
    lcm_t *lcm;
    char  *channel;
};

typedef struct timer_table timer_table_t;
struct timer_table {
    char   *channel;
    double hz;
};

int
timer_callback (void *_user)
{
    user_t *user = _user;
    perllcm_heartbeat_t hb = {.utime = timestamp_now ()};
    perllcm_heartbeat_t_publish (user->lcm, user->channel, &hb);
    return 1;
}


static void
add_timer_table_entry (timer_table_t *tt, int maxlen, const char *channel, double hz)
{
    // look for an empty slot
    int i = 0;
    for (i=0; i<maxlen && tt->channel; i++)
        tt++;

    if (i==maxlen) {
        ERROR ("Too many channels, increase MAX_TIMERS");
        abort ();
    }
    else {
        tt->channel = strdup (channel);
        tt->hz = hz;
    }
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // options
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, 
                            "Heartbeat process publishes LCM heartbeat_t messages at regular intervals.");
    getopt_add_bool    (gopt, 'h', "help",   0,  "Show this");
    getopt_add_bool    (gopt, 'D', "daemon", 0,  "Run as daemon?");
    getopt_add_string  (gopt, 'p', "prefix", "", "LCM channel prefix");
    getopt_add_string  (gopt, 'F', "freqs",   "1,5,10", "Comma separated list of frequencies to publish");
    getopt_add_example (gopt, 
                        "Use channel prefix TEST and  publish 7 and 8.5 Hz in addition to the 1,5,10 Hz defaults\n"
                        "%s --freq 1,5,10,7,8.5 --prefix TEST", argv[0]);
                        

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=0) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    // use a LCM channel prefix?
    char *prefix = NULL;
    if (getopt_has_flag (gopt, "prefix"))
        prefix = g_strconcat (getopt_get_string (gopt, "prefix"), NULL);
    else
        prefix = "";


    // construct heartbeat timer table
    int n_timers = 0;
    timer_table_t *tt = calloc (MAX_TIMERS, sizeof (*tt));
    char **freq = g_strsplit (getopt_get_string (gopt, "freqs"), ",", -1);
    for (int i=0; freq[i]!=NULL; i++) {
        if (i == MAX_TIMERS) {
            ERROR ("Exceeded max timers [%d]", MAX_TIMERS);
            exit (EXIT_FAILURE);
        }

        double hz = atof (freq[i]);
        char *channel = lcmu_channel_get_heartbeat (prefix, hz);
        add_timer_table_entry (tt, MAX_TIMERS, channel, hz);
        n_timers++;
        printf ("Channel:%-30sHz:%g\n", tt[i].channel, tt[i].hz);
        free (channel);
    }

    // daemon mode?
    bool daemon = getopt_get_bool (gopt, "daemon");
    if (daemon)
        daemon_fork ();

    // initialize lcm
    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) {
        ERROR ("lcm_create()");
        exit (EXIT_FAILURE);
    }

    // launch individual timer threads
    user_t user[n_timers];
    timeutil_timer_t t[n_timers];
    for (int i=0; i<n_timers; i++) {
        user[i].lcm = lcm;
        user[i].channel = tt[i].channel;
        struct timespec spec = timeutil_hz_to_timespec (tt[i].hz);
        t[i] = timeutil_timer_create (spec, &timer_callback, &user[i]);
    }

    for (size_t i=0; ; i++) {
        switch (i%=4) {
        case 0: printf ("thump.\n"); break;
        case 1: printf ("thump..\n"); break;
        case 2: printf ("thump...\n"); break;
        case 3: printf ("thump....\n"); break;
        };
        fflush (stdout);
        timeutil_sleep (1);
    }

    timeutil_timer_destroy (t);
}
