#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/getopt.h"
#include "perls-common/units.h"
#include "perls-common/generic_sensor_driver.h"

#include "perls-lcmtypes/senlcm_kvh_dsp3000_t.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "KVH DSP3000 FOG sensor driver.");
    getopt_add_string (gsd->gopt, 'm', "mode", "rate", "Output mode {rate, delta, angle}");
    getopt_add_bool   (gsd->gopt, 'z', "zero", 0,      "Zero tare");
    return 0;
}

static void
kvh_init (generic_sensor_driver_t *gsd, int mode)
{
    printf ("\n");
    switch (mode)
    {
    case SENLCM_KVH_DSP3000_T_RATE_MODE:
        printf ("RATE MODE\n");
        gsd_write (gsd, "R", 1);
        break;
    case SENLCM_KVH_DSP3000_T_DELTA_MODE:
        printf ("DELTA MODE\n");
        gsd_write (gsd, "A", 1);
        break;
    case SENLCM_KVH_DSP3000_T_ANGLE_MODE:
        printf ("ANGLE MODE\n");
        gsd_write (gsd, "P", 1);
        break;
    default:
        ERROR ("This should never happen!");
        exit (EXIT_FAILURE);
    }

    if (getopt_get_bool (gsd->gopt, "zero"))
    {
        printf ("ZERO\n");
        gsd_write (gsd, "Z", 1);
    }
}

int
main (int argc, char *argv[])
{
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical (gsd, '\r','\n');
    gsd_launch (gsd);

    // parse kvh mode
    int mode;
    const char *mode_str = getopt_get_string (gsd->gopt, "mode");
    if (0==strcasecmp (mode_str, "rate"))
        mode = SENLCM_KVH_DSP3000_T_RATE_MODE;
    else if (0==strcasecmp (mode_str, "delta"))
        mode = SENLCM_KVH_DSP3000_T_DELTA_MODE;
    else if (0==strcasecmp (mode_str, "angle"))
        mode = SENLCM_KVH_DSP3000_T_ANGLE_MODE;
    else
    {
        ERROR ("unknown mode option");
        exit (EXIT_FAILURE);
    }

    int64_t utime=0, utime_prev=0;
    gsd_flush (gsd);
    gsd_reset_stats (gsd);
    while (1)
    {
        // read stream
        char buf[128];
        utime_prev = utime;
        gsd_read (gsd, buf, 128, &utime);

        // kvh initialization: check whether
        // a) the first time through, or
        // b) the sensor get powered on *after* the driver was already started
        const int64_t MAX_USEC = 1e6/100 * 50; // 50 misses @ 100Hz
        if ((utime - utime_prev) > MAX_USEC)
        {
            kvh_init (gsd, mode);
            continue;
        }

        double data;
        int valid;
        int ret = sscanf (buf, "%lf %d", &data, &valid);
        if (ret == 2 && valid == 1)
        {
            senlcm_kvh_dsp3000_t kvh;
            kvh.utime = utime;
            kvh.mode = mode;
            kvh.data = data * DTOR; // rad or rad/s depending on mode
            /* Earth rate in degrees = -15.04107 x sin(latitude) Note:
               Northern latitudes are positive and southern latitudes
               are negative.
            */

            senlcm_kvh_dsp3000_t_publish (gsd->lcm, gsd->channel, &kvh);
            gsd_update_stats (gsd, 1);
        }
        else
            gsd_update_stats (gsd, -1);
    } // while
}
