#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/getopt.h"
#include "perls-common/units.h"
#include "perls-common/generic_sensor_driver.h"

#include "perls-lcmtypes/senlcm_tritech_es_t.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

static int parse_buf (const char *buf, int len, senlcm_tritech_es_t *tritech)
{
    double range = -1;
    sscanf(buf, "%lfm", &range);
    if (range > -1)
    {
        tritech->range = range;
        return 1;
    }
    else
        return 0;
}

static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "TRITECH ECHOSOUNDER sensor driver.");
    return 0;
}

int main (int argc, char *argv[])
{
    // Generic Sensor Driver Stuff
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical (gsd, '\r','\n');
    gsd_launch (gsd);

    while (1)
    {
        char buf[15];
        int64_t timestamp;
        int len = gsd_read (gsd, buf, 128, &timestamp);
        senlcm_tritech_es_t tritech;
        if (parse_buf (buf, len, &tritech))
        {
            tritech.utime = timestamp;
            senlcm_tritech_es_t_publish (gsd->lcm, gsd->channel, &tritech);
            gsd_update_stats (gsd, 1);
        }
        else
            gsd_update_stats (gsd, -1);
    } // while
} //main
