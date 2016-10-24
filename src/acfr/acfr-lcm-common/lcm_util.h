
#include <sys/select.h>
#include <lcm/lcm.h>
#include <sys/time.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef LCM_UTIL_H
#define LCM_UTIL_H

int lcmu_handle_timeout (lcm_t *lcm, struct timeval *timeout);

#endif // LCM_UTIL_H
