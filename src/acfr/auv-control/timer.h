
#ifndef TIMER_H
#define TIMER_H

#include <time.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <stdio.h>

typedef struct 
{
	int timer_fd;
	unsigned long long wakeups_missed;
} periodic_info;

extern int make_periodic (unsigned int period, periodic_info *info);
extern void wait_period (periodic_info *info);

#endif
