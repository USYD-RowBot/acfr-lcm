/* ================================================================
** main.c
**
** Driver for easyDAQ SERDIO8R relay card.
**
** 30 DEC 2008  ryan eustice  Created and written.
** ================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>  // umask()
#include <sys/stat.h>   // "
#include <unistd.h>     // fork()
#include <strings.h>

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "perls-lcmtypes/senlcm_easydaq_t.h"

#include "easydaq.h"



typedef struct _state_t state_t;
struct _state_t
{
    int fd;
    BotParam *param;
    getopt_t *gopt;
    lcm_t *lcm;
    char *channel;
    senlcm_easydaq_t easydaq;
    uint8_t bitmask;
};

static void 
add_commandline_options (state_t *state, int argc, char *argv[])
{
    getopt_add_description (state->gopt, "EasyDAQ SERDIO8R 8-channel relay-board driver.");
    getopt_add_help (state->gopt, NULL);
    getopt_add_bool (state->gopt, 'D', "daemon", 0, "Run as system daemon");
    getopt_add_bool (state->gopt, 'i', "ignore", 0, "Ignore init relay state request in .cfg");
    getopt_add_string (state->gopt, 'c', "channel", 
                       botu_param_get_str_or_default (state->param, "hotel.easydaq.channel", "EASYDAQ"),
                       "LCM channel name");
    getopt_add_string (state->gopt, '\0', "device", 
                       botu_param_get_str_or_default (state->param, "hotel.easydaq.device", "/dev/ttyS0"),
                       "Device to connect to, e.g. /dev/ttyS0");

    if (!getopt_parse (state->gopt, argc, argv, 1) || state->gopt->extraargs->len!=0) {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt, "help")) {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_SUCCESS);
    }
}

static int
update_state (state_t *state)
{
    const int MAX_ATTEMPS = 10;
    for (int i=0; i<MAX_ATTEMPS; i++) {
        easydaq_setdir (state->fd, 0x00);
        int err = easydaq_read (state->fd, &state->bitmask);
        if (!err) {
            for (int r=0; r<8; r++) {
                if (state->bitmask & (1 << r))
                    state->easydaq.relay[r].state = 1;
                else
                    state->easydaq.relay[r].state = 0;
            }
            return 0;
        } 
    }
    printf ("*Warning*: Easydaq not talking, are you on the correct serial port?\n");
    return -1;
}

static void
easydaq_t_callback (const lcm_recv_buf_t *rbuf, const char *channel, 
                    const senlcm_easydaq_t *easydaq, void *user)
{
    state_t *state = user;

    if (easydaq->self)
        return;
    
    // toggle relays
    for (int r=0; r<8; r++) {
        if (easydaq->relay[r].state)
            state->bitmask |= (1 << r);
        else
            state->bitmask &= ~(1 << r);
    }
    easydaq_setdir (state->fd, 0x00);
    easydaq_write (state->fd, state->bitmask);

    // publish current relay state
    if (0 == update_state (state)) {
        state->easydaq.self = 1;
        state->easydaq.utime = timestamp_now ();
        senlcm_easydaq_t_publish (state->lcm, state->channel, &state->easydaq);
    }
}

int
timer_callback (void *user)
{
    const senlcm_easydaq_t *easydaq = user;
    if (!easydaq->relay[0].label)
        return 1; // wait for initialiation to finish, don't print an empty struct

    static int up_sec=0, up_min=0, up_hour=0, up_days=0;
    if (++up_sec > 59) { up_sec = 0;
        if (++up_min > 59) { up_min = 0;
            if (++up_hour > 23) { up_hour = 0;
                ++up_days;
            }
        }
    }
    if (up_days)
        printf ("\n%02d:%02d:%02d up %d days\n", up_hour, up_min, up_sec, up_days);
    else
        printf ("\n%02d:%02d:%02d\n", up_hour, up_min, up_sec);
    easydaq_printf (easydaq);
    return 1;
}

int main (int argc, char *argv[]) 
{   
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // init state
    state_t *state = calloc (1, sizeof (*state));

    state->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!state->param) {
        ERROR ("Could not create config parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }

    state->gopt = getopt_create ();
    add_commandline_options (state, argc, argv);

    if (getopt_get_bool (state->gopt, "daemon"))
        daemon_fork ();
    else {
        // launch foreground timer thread
        struct timespec spec = timeutil_hz_to_timespec (1.0);
        timeutil_timer_create (spec, &timer_callback, &state->easydaq);
    }

    const char *device = getopt_get_string (state->gopt, "device");
    state->fd = easydaq_open (device);
    if (state->fd < 0) {
        ERROR ("easydaq_open");
        exit (EXIT_FAILURE);
    }

    // init lcm and subscribe
    state->lcm = lcm_create (NULL);
    if (!state->lcm) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }
    // easydaq channel subscribe
    state->channel = strdup (getopt_get_string (state->gopt, "channel"));
    senlcm_easydaq_t_subscribe (state->lcm, state->channel, &easydaq_t_callback, state);

    // parse config file
    state->easydaq.utime = timestamp_now ();
    for (int r=0; r<8; r++) {
        char key[128];
        
        sprintf (key, "hotel.easydaq.relay%d.label", r+1);
        char *label = botu_param_get_str_or_default (state->param, key, "unknown");

        sprintf (key, "hotel.easydaq.relay%d.group", r+1);
        char *group = botu_param_get_str_or_default (state->param, key, "unspecified");

        sprintf (key, "hotel.easydaq.relay%d.state", r+1);
        bool onoff = false;
	bot_param_get_boolean (state->param, key, (int*)&onoff);

        sprintf (key, "hotel.easydaq.relay%d.exclude_all", r+1);
        bool exclude_all = false;
	bot_param_get_boolean (state->param, key, (int*)&exclude_all);

        // assign our relays
        state->easydaq.relay[r].label = label;
        state->easydaq.relay[r].group = group;
        state->easydaq.relay[r].exclude_all = exclude_all;

        // queue up a relay request with our desired initial state
        if (!getopt_get_bool (state->gopt, "ignore"))
            state->easydaq.relay[r].state = onoff;
    }
    state->easydaq.self = 0;
    senlcm_easydaq_t_publish (state->lcm, state->channel, &state->easydaq);

    // main loop
    if (getopt_get_bool (state->gopt, "daemon"))
        close (STDERR_FILENO);
    while (1) {
        struct timeval timeout = {
            .tv_sec = 0,
            .tv_usec = 250000,
        };
        int ret = lcmu_handle_timeout (state->lcm, &timeout);

        if (ret==0 && 0==update_state (state)) { /* timeout */
            state->easydaq.self = 1;
            state->easydaq.utime = timestamp_now ();
            senlcm_easydaq_t_publish (state->lcm, state->channel, &state->easydaq);
        }
    }

    // clean up
    if (lockf (state->fd, F_ULOCK, 0) < 0) {
        PERROR ("lockf");
        return -1;
    }
    easydaq_close (state->fd);
    lcm_destroy (state->lcm);
    free (state);
    return 0;
}
