#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/ascii.h"
#include "perls-common/bot_util.h"
#include "perls-common/dfs.h"
#include "perls-common/error.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-common/nmea.h"
#include "perls-common/units.h"

#include "perls-math/rphcorr.h"

#include "perls-lcmtypes/senlcm_os_compass_t.h"

#include "os5000.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

static int
parse_buf (const char *buf, int len, senlcm_os_compass_t *osc, bool freshwater)
{
    os5000_t os = os5000_parse_0x01 (buf, len);

    // rph
    if (os.bitmask & OS5000_BITMASK_R)
        osc->rph[0] = os.R*DTOR;
    else
        return 0;
    if (os.bitmask & OS5000_BITMASK_P)
        osc->rph[1] = os.P*DTOR;
    else
        return 0;
    if (os.bitmask & OS5000_BITMASK_H)
        osc->rph[2] = os.H*DTOR;
    else
        return 0;

    // temperature
    if (os.bitmask & OS5000_BITMASK_T)
        osc->T = os.T;
    else
        return 0;

    // pressure in pascals
    if (os.bitmask & OS5000_BITMASK_D)
        osc->p_volts = os.D;
    else
        return 0;
    osc->p_meas = os5000_volts_to_psi (os.D)*UNITS_PSI_TO_PASCAL;
    static int nsamples = 0;
    static double p_o = 0;
    if (nsamples++ < 10)
        p_o = ((nsamples-1.0)/nsamples)*p_o + (1.0/nsamples)*osc->p_meas; // running average
    osc->p_o = p_o;
    osc->p_gage = osc->p_meas - osc->p_o;

    // depth in meters
    if (freshwater)
        osc->depth = dfs_simple (DFS_RHO_FRESHWATER, osc->p_gage);
    else
        osc->depth = dfs_simple (DFS_RHO_SALTWATER, osc->p_gage);

    // magnetic field in Gauss
    if (os.bitmask & OS5000_BITMASK_M_XYZ)
    {
        osc->Mxyz[0] = os.Mxyz[0] * UNITS_MILLI_TO_ONE;
        osc->Mxyz[1] = os.Mxyz[1] * UNITS_MILLI_TO_ONE;
        osc->Mxyz[2] = os.Mxyz[2] * UNITS_MILLI_TO_ONE;
    }
    else
        return 0;

    // acceleration in G's
    if (os.bitmask & OS5000_BITMASK_A_XYZ)
    {
        osc->Gxyz[0] = os.Axyz[0];
        osc->Gxyz[1] = os.Axyz[1];
        osc->Gxyz[2] = os.Axyz[2];
    }
    else
        return 0;

    return 1;
}

static int
os_config (generic_sensor_driver_t *gsd)
{
#define SEND_TO_COMPASS(gsd, buf, len) gsd_write (gsd, buf, len); usleep (250e3)

    int len = 0;
    char buf[2048];

    printf ("configuring OS5000 for Iver use... ");

    /* raw volts output for depth */
    len = sprintf (buf, "%cM\r", ASCII_ESC);
    SEND_TO_COMPASS (gsd, buf, len);
    len = sprintf (buf, "0\r");
    SEND_TO_COMPASS (gsd, buf, len);

    /* select desired fields */
    const int desired =
        OS5000_BITMASK_H + OS5000_BITMASK_P + OS5000_BITMASK_R +
        OS5000_BITMASK_T + OS5000_BITMASK_D + OS5000_BITMASK_M_XYZ + OS5000_BITMASK_A_XYZ;
    len = sprintf (buf, "%cX\r", ASCII_ESC);
    SEND_TO_COMPASS (gsd, buf, len);
    len = sprintf (buf, "%d\r", desired);
    SEND_TO_COMPASS (gsd, buf, len);

    /* select desired output format */
    len = sprintf (buf, "%c*\r", ASCII_ESC);
    SEND_TO_COMPASS (gsd, buf, len);
    len = sprintf (buf, "1\r");
    SEND_TO_COMPASS (gsd, buf, len);

    printf ("ready!\n");

    return 1;
}


static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Ocean-Server OS5500-USG digital compass sensor driver.");
    return 0;
}

int
main (int argc, char *argv[])
{
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical (gsd, '\r','\n');
    gsd_launch (gsd);

    rphcorr_t *rphcorr = rphcorr_create (gsd->params, gsd->rootkey);
    if (!rphcorr)
    {
        ERROR ("rphcorr_create () failed!");
        exit (EXIT_FAILURE);
    }

    os_config (gsd);

    double salinity = bot_param_get_double_or_fail (gsd->params, "site.salinity");
    bool freshwater = salinity < 5 ? 1 : 0;

    gsd_flush (gsd);
    gsd_reset_stats (gsd);
    while (1)
    {
        // read stream
        char buf[1024];
        int64_t timestamp;
        int len = gsd_read (gsd, buf, 128, &timestamp);

        senlcm_os_compass_t os_compass;
        if (parse_buf (buf, len, &os_compass, freshwater))
        {
            os_compass.utime = timestamp;
            senlcm_os_compass_t_publish (gsd->lcm, gsd->channel, &os_compass);
            publish_rphcorr (gsd->lcm, rphcorr, timestamp, os_compass.rph);

            gsd_update_stats (gsd, 1);
        }
        else
            gsd_update_stats (gsd, -1);

    } // while

    rphcorr_destroy (rphcorr);
}
