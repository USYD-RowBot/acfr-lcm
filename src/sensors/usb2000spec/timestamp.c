/* ================================================================
** timestamp.[ch]
**
** timestamp utilities.
**
** 23 DEC 2008  Ryan Eustice  Created from mitdgc-log-viewer/src/common/timestamp.[ch]
** 23 DEC 2008  RME           Added support for timestamp_double()
** ================================================================

MIT 2007 DARPA Urban Challenge Log File Viewer
Copyright (C) 2007-2008  Massachusetts Institute of Technology

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/select.h>
#include <glib.h>


#include "timestamp.h"

int64_t
timestamp_now (void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

int64_t
timestamp_seconds (int64_t v)
{
    return v / 1000000;
}

int64_t 
timestamp_useconds (int64_t v)
{
    return v % 1000000;
}

double 
timestamp_to_double (int64_t v)
{
    return v / 1000000.0;
}

void 
timestamp_to_timeval (int64_t v, struct timeval *tv)
{
    tv->tv_sec  = timestamp_seconds (v);
    tv->tv_usec = timestamp_useconds (v);
}

void 
timestamp_to_timespec (int64_t v, struct timespec *ts) 
{
    ts->tv_sec  = timestamp_seconds (v);
    ts->tv_nsec = timestamp_useconds (v) * 1000;
}

void
timestamp_to_gtimeval (int64_t v, GTimeVal *result)
{
    result->tv_sec  = timestamp_seconds (v);
    result->tv_usec = timestamp_useconds (v);
}

timestamp_sync_state_t*
timestamp_sync_init (double dev_ticks_per_second, int64_t dev_ticks_wraparound,
                     double rate)
{
    timestamp_sync_state_t *s;

    s = calloc (1, sizeof (*s));
    if (!s)
        return NULL;

    s->dev_ticks_per_second = dev_ticks_per_second;
    s->dev_ticks_wraparound = dev_ticks_wraparound;
    s->max_rate_error = rate;
    s->is_valid = 0;

    return s;
}

void
timestamp_sync_free (timestamp_sync_state_t *s)
{
    free (s);
}

int64_t
timestamp_sync (timestamp_sync_state_t *s, int64_t dev_ticks,
                int64_t host_utime)
{
    if (!s->is_valid) {
        /* The first sync has no history */
        s->is_valid = 1;

        s->sync_host_time = host_utime;
        s->last_dev_ticks = dev_ticks;
        s->dev_ticks_since_sync = 0;

        return host_utime;
    }


    // how many device ticks since the last invocation?
    int64_t dticks = dev_ticks - s->last_dev_ticks;
    s->last_dev_ticks = dev_ticks;
    if (dticks < 0)
        dticks += s->dev_ticks_wraparound;

    s->dev_ticks_since_sync += dticks;

    // overestimate device time by a factor of s->rate
    double rate = 1000000.0 / s->dev_ticks_per_second * s->max_rate_error;

    // estimate of the host's time corresponding to the device's time
    int64_t dev_utime = s->sync_host_time + (s->dev_ticks_since_sync * rate);

    int64_t time_err = host_utime - dev_utime;

    /* If time_err is very large, resynchronize, emitting a warning. if
     * it is negative, we're just adjusting our timebase (it means
     * we got a nice new low-latency measurement.) */
    if (time_err > 1000000000LL) { /* 1000 seconds */
        fprintf (stderr, "Warning: Time sync has drifted by more than 1000 seconds\n");
        s->sync_host_time = host_utime;
        s->dev_ticks_since_sync = 0;
        dev_utime = host_utime;
    }
    if (time_err < 0) {
        s->sync_host_time = host_utime;
        s->dev_ticks_since_sync = 0;
        dev_utime = host_utime;
    }
    //printf ("%lld  %lld  %lld  %lld\n", host_utime, dev_utime, time_err, dticks);

    return dev_utime;
}

size_t
timestamp_strftime (char *s, size_t max, const char *format, struct timeval *tv)
{
    int tmalloc = 0;
    if (tv == NULL) {
        tmalloc = 1;
        tv = malloc (sizeof (struct timeval));
        timestamp_to_timeval (timestamp_now (), tv);
    }

    struct tm tm;
    localtime_r (&tv->tv_sec, &tm);


    const char *formatend = format + strlen (format);
    char format2[1024], tmp[1024];
    
    // handle %i arg if present
    char *istr = strstr (format, "%i");
    if (istr != NULL) {
        if (istr > format) {
            memset (tmp, '\0', sizeof (tmp));
            strncpy (tmp, format, istr - format);
            sprintf (format2, "%s%06ld%s", tmp, tv->tv_usec, istr+2 < formatend ? istr+2 : "");
        }
        else {
            sprintf (format2, "%06ld%s", tv->tv_usec, format+2);
        }
    }
    else
        strcpy (format2, format);

    if (tmalloc)
        free (tv);

    return strftime (s, max, format2, &tm);
}

size_t
acfr_timestamp_strftime (char *s, size_t max, const char *format, struct timeval *tv)
{
    int tmalloc = 0;
    if (tv == NULL) {
        tmalloc = 1;
        tv = malloc (sizeof (struct timeval));
        timestamp_to_timeval (timestamp_now (), tv);
    }

    struct tm tm;
    localtime_r (&tv->tv_sec, &tm);


    const char *formatend = format + strlen (format);
    char format2[1024], tmp[1024];

    // handle %i arg if present
    char *istr = strstr (format, "%i");
    if (istr != NULL) {
        if (istr > format) {
            memset (tmp, '\0', sizeof (tmp));
            strncpy (tmp, format, istr - format);
            sprintf (format2, "%s%03ld%s", tmp, tv->tv_usec/1000, istr+2 < formatend ? istr+2 : "");
        }
        else {
            sprintf (format2, "%03ld%s", tv->tv_usec/1000, format+2);
        }
    }
    else
        strcpy (format2, format);

    if (tmalloc)
        free (tv);

    return strftime (s, max, format2, &tm);
}
