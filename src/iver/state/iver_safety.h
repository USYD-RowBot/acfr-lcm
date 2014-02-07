#ifndef __IVER_SAFETY_H__
#define __IVER_SAFETY_H__

#include <lcm/lcm.h>
#include <bot_param/param_client.h>

typedef struct
{
    int64_t max_allowable_age;
    int64_t age;
    int64_t last_utime;
} safety_sensor_t;

typedef struct 
{
    lcm_t *lcm;

    // data age per sensor
    safety_sensor_t microstrain;
    safety_sensor_t os_compass;
    safety_sensor_t dstar;
    safety_sensor_t rdi;

    // safety rules active/violated bitmasks
    int64_t safety_rules_active;
    int64_t safety_rules_violated;

    // safety channel
    char *safety_channel;
} safety_t;

safety_t*
safety_init (lcm_t *lcm, BotParam *param);

void
safety_destroy (safety_t *state);

#endif //__IVER_SAFETY_H__
