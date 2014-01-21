#ifndef __IVER_NAV_H__
#define __IVER_NAV_H__

#include <lcm/lcm.h>
#include <bot_param/param_client.h>

typedef struct
{
    lcm_t *lcm;
    BotParam *param;

    bool use_navigator;

    double rph[3];
    double dfs;
    double temp;

    char *os_conduit_write_chan;
} nav_state_t;

nav_state_t*
nav_init (lcm_t *lcm, BotParam *param);

void
nav_destroy (nav_state_t *state);

#endif //__IVER_NAV_H__
