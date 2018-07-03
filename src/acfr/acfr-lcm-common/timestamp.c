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
#include <sys/time.h>
#include <sys/select.h>

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
    if (!s->is_valid)
    {
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
    if (time_err > 1000000000LL)   /* 1000 seconds */
    {
        fprintf (stderr, "Warning: Time sync has drifted by more than 1000 seconds\n");
        s->sync_host_time = host_utime;
        s->dev_ticks_since_sync = 0;
        dev_utime = host_utime;
    }
    if (time_err < 0)
    {
        s->sync_host_time = host_utime;
        s->dev_ticks_since_sync = 0;
        dev_utime = host_utime;
    }
#if 0
    printf ("%"PRId64"  %"PRId64"  %+6"PRId64"  %8"PRId64"  %"PRId64"\n",
            host_utime, dev_utime, time_err, dticks, dev_ticks);
#endif

    return dev_utime;
}


/* ----------------------------------------------------------------------

   sprintfs dsl data time string based on computer system clock

   MODIFICATION HISTORY
   DATE         WHO             WHAT
   -----------  --------------  ----------------------------
   14 APR 1999  Louis Whitcomb  Created and Written
   22 JAN 2001  SL              Renamed from rov_sprintf_dsl_time_string to
                                rovtime2dsltime_str, added optional
				time parameter, localtime->gmtime, 4-digit yr,
				2-digit decimal seconds. If rov_time is < 0,
				use current time
   ---------------------------------------------------------------------- */
int rovtime2dsltime_str(double rov_time, char *str)
{ int    num_chars;

  // for min, sec
  double total_secs;
  double secs_in_today;

  //double day;
  double hour;
  double min;
  double sec;

  // for date and hours
  struct tm  *tm;
  time_t      current_time;

  // read gettimeofday() clock and compute min, and
  // sec with microsecond precision. Use rov_time if
  // specified (rov_time >= 0)
  if (rov_time >= 0) 
    { total_secs = rov_time;
      current_time = (time_t)total_secs;
      tm = gmtime(&current_time);
    }
  else 
    {//use current time
      total_secs = ((double)(timestamp_now()))/1e6;
      current_time = time(NULL);
      tm = gmtime(&current_time);
    }
		
  secs_in_today = fmod(total_secs,86400.0); //86400=24.0*60.0*60.0
  hour = secs_in_today/3600.0;
  min  = fmod(secs_in_today/60.0,60.0);
  sec  = fmod(secs_in_today,60.0);

  // call time() and gmtime for hour and date 
  // current_time = time(NULL);
  // tm = gmtime(&current_time);

  num_chars = sprintf(str,"%04d/%02d/%02d %02d:%02d:%05.2f",
		      (int) tm->tm_year+1900,
		      (int) tm->tm_mon+1,
		      (int) tm->tm_mday,
		      (int) tm->tm_hour,
		      (int) min,
		      sec);

   
  return num_chars;
}



