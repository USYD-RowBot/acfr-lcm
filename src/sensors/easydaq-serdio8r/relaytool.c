/**
 * @defgroup UtilitiesRelayTool relaytool
 * @brief A command line relay utility.
 * @ingroup Utilities
 *
 * @details
 * A command line utility for toggling relays in conjunction with the easydaq sensor driver.
 *
 * @see easydaq
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "perls-lcmtypes/senlcm_easydaq_t.h"

#include "easydaq.h"


static void
easydaq_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_easydaq_t *msg, void *user)
{
    senlcm_easydaq_t **easydaq = user;
    *easydaq = senlcm_easydaq_t_copy (msg);
}

static void
printf_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_easydaq_t *msg, void *user)
{
    bool *done = user;
    if (msg->self != 0)
    {
        *done = 1;
        easydaq_printf (msg);
        printf ("\n");
    }
}

static int
on_off (getopt_t *gopt, const char *lname, bool state)
{
    const char *req = getopt_get_string (gopt, lname);
    if (0==strcasecmp (req, "on"))
        return 1;
    else if (0==strcasecmp (req, "off"))
        return 0;
    else if (0==strcasecmp (req, "tog"))
        return !state;
    else
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
}

int main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // static getopt options
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "Command line tool for setting/querying EasyDAQ relay-card driver.");
    getopt_add_help (gopt, NULL);
    getopt_add_string (gopt, 'c', "channel",
                       botu_param_get_str_or_default (param, "hotel.easydaq.channel", "EASYDAQ"),
                       "LCM channel name");
    const char *channel = getopt_get_string (gopt, "channel");

    // create a lcm object and temporarily subscribe to easydaq channel
    lcm_t *lcm = lcm_create (NULL);
    senlcm_easydaq_t *easydaq = NULL;
    senlcm_easydaq_t_subscription_t *sub =
        senlcm_easydaq_t_subscribe (lcm, channel, &easydaq_callback, &easydaq);


    // observe the current easydaq state
    struct timeval timeout =
    {
        .tv_sec = 0,
        .tv_usec = 500000,
    };
    int ret = lcmu_handle_timeout (lcm, &timeout);
    senlcm_easydaq_t_unsubscribe (lcm, sub);
    if (ret == 0)
        ERROR ("Timeout: is easydaq daemon running on LCM channel [%s]?", channel);
    else if (easydaq)
    {
        // dynamic getopt options
        getopt_add_spacer (gopt, "");
        getopt_add_spacer (gopt, "The following OPTS accept ARG of [on|off|tog]");

        getopt_add_spacer (gopt, "------------------By relay------------------------");
        getopt_add_string (gopt, 'a',  "all",   "", "");
        for (int i=0; i<8; i++)
        {
            char relay[8];
            sprintf (relay, "%d", i+1);
            getopt_add_string (gopt, relay[0], relay, "", "");
        }

        getopt_add_spacer (gopt, "------------------By label-------------------------");
        for (int i=0; i<8; i++)
            getopt_add_string (gopt, '\0', easydaq->relay[i].label, "", "");

        getopt_add_spacer (gopt, "------------------By group-------------------------");
        for (int i=0; i<8; i++)
        {
            bool unique = 1;
            for (size_t j=0; j<i; j++)
                unique *= strcmp (easydaq->relay[j].group, easydaq->relay[i].group);
            if (unique)
                getopt_add_string (gopt, '\0', easydaq->relay[i].group, "", "");
        }
    }
    else
    {
        ERROR ("easydaq is NULL");
        exit (EXIT_FAILURE);
    }

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len != 0)
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }
    else if (ret == 0)
        exit (EXIT_FAILURE);


    if (argc == 1)
    {
        // just print our current relay state
        easydaq_printf (easydaq);
        printf ("\n");
    }
    else
    {
        // parse commandline args
        if (getopt_has_flag (gopt, "all"))
        {
            for (size_t i=0; i<8; i++)
            {
                if (!easydaq->relay[i].exclude_all)
                    easydaq->relay[i].state = on_off (gopt, "all", easydaq->relay[i].state);
            }
        }
        else
        {
            for (int i=0; i<8; i++)
            {
                char relaynum[8];
                sprintf (relaynum, "%d", i+1);
                if (getopt_has_flag (gopt, relaynum))
                    easydaq->relay[i].state = on_off (gopt, relaynum, easydaq->relay[i].state);
                else if (getopt_has_flag (gopt, easydaq->relay[i].label))
                    easydaq->relay[i].state = on_off (gopt, easydaq->relay[i].label, easydaq->relay[i].state);
                else if (getopt_has_flag (gopt, easydaq->relay[i].group))
                    easydaq->relay[i].state = on_off (gopt, easydaq->relay[i].group, easydaq->relay[i].state);
            }
        }

        // send over our relay request
        easydaq->self = 0;
        senlcm_easydaq_t_publish (lcm, channel, easydaq);

        // listen for and print out the observed relay state
        bool done = 0;
        sub = senlcm_easydaq_t_subscribe (lcm, channel, &printf_callback, &done);
        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        do
        {
            ret = lcmu_handle_timeout (lcm, &timeout);
        }
        while (!done && ret>0);
        senlcm_easydaq_t_unsubscribe (lcm, sub);
    }

    // clean up
    senlcm_easydaq_t_destroy (easydaq);
    lcm_destroy (lcm);

    exit (EXIT_SUCCESS);
}
