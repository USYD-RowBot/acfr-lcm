#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h> // needed for PRId64 macros

#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

struct data {
    double hz;
    int64_t utime_prev;
};

int
callback (void *user)
{
    struct data *data = user;

    int64_t utime = timestamp_now ();
    printf ("Hz=%3lf: %8"PRId64" us\n",
            data->hz, utime - data->utime_prev);
    data->utime_prev = utime;

    return 1;
}

int
main (int argc, char *argv[])
{
    struct data data[] = {
        {.hz = 3},
        {.hz = 2},
        {.hz = 1},
    };

    size_t ntimers = sizeof (data) / sizeof (struct data);
    timeutil_timer_t timers[ntimers];
    for (size_t i=0; i<ntimers; i++) {
        struct timespec spec = timeutil_hz_to_timespec (data[i].hz);
        timers[i] = timeutil_timer_create (spec, callback, &data[i]);
    }

    for (size_t i=0; i<5; i++)
        sleep (1);

    timeutil_timer_destroy (&timers[0]);

    printf ("timeutil_timer_destroy (th[0])\n");    
    while (1)
        sleep (1);
}
