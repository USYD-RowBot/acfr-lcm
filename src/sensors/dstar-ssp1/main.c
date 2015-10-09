#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/dfs.h"
#include "perls-common/error.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-common/nmea.h"
#include "perls-common/units.h"

#include "perls-lcmtypes/senlcm_dstar_ssp1_t.h"


/*
    Output Units:
	Send to sensor:
	  #G1 for mPSI
	  #G1000 for PSI
	  #G1458 for m_sw
	  #G444 for ft_sw
	  #G432 for feet_fresh_water    <---- !!
	  #G145 for kPa
	  #G14505 for bar
	  #G14696 for ATA
    Message Format
    Desert Star-Pressure Sensor:
      $YXDPT,##.###,0.0*CheckSum
      $YXTMP,##.###,0.0*CheckSum
*/


static int parse_buf (const char *buf, int len, senlcm_dstar_ssp1_t *dstar, bool freshwater)
{
    static double p_atm_avg = 0;
    static int nsamples = 0;

    if (!nmea_validate_checksum (buf))
        return 0;

    if (0==strncmp (buf, "$YXDPT", 6))
    {
        double pressure_psi;
        if (1 == sscanf (buf, "$YXDPT,%lf,0.0", &pressure_psi))
        {
            // on startup, init p_atm to ambient pressure
            if (nsamples <= 10)
            {
                nsamples++;

                double pressure_Pa = pressure_psi * UNITS_PSI_TO_PASCAL;
                double dfs = dfs_simple (DFS_RHO_FRESHWATER, pressure_Pa - DFS_P_ATM_NOMINAL);

                // Check that we are on the surface
                if (-1.5 < dfs && dfs < 1.5)
                    p_atm_avg = ((nsamples-1.0)/nsamples)*p_atm_avg + (1.0/nsamples)*pressure_Pa; // tare
                else
                    p_atm_avg = DFS_P_ATM_NOMINAL;
            }

            dstar->p_abs = pressure_psi * UNITS_PSI_TO_PASCAL;
            dstar->p_gage = dstar->p_abs - p_atm_avg;
            dstar->p_atm = p_atm_avg;
            if (freshwater)
                dstar->depth = dfs_simple (DFS_RHO_FRESHWATER, dstar->p_gage);
            else
                dstar->depth = dfs_simple (DFS_RHO_SALTWATER, dstar->p_gage);
            return 1;
        }
        else
            return 0;
    }
    else if (0==strncmp (buf, "$YXTMP", 6))
    {
        double T;
        if (1 == sscanf (buf, "$YXTMP,%lf,0.0", &T))
        {
            dstar->temperature = T;
            return 1;
        }
        else
            return 0;
    }
    else
        return 0;
}

static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Desert-Star SSP-1 sensor driver.");
    /*
    getopt_add_bool (gsd->gopt, '\0', "tare",     0, "Sets the current pressure as the zero point");
    getopt_add_bool (gsd->gopt, '\0', "untare",   0, "Removes zero compensation");
    */
    return 0;
}


int main (int argc, char *argv[])
{
    // Generic Sensor Driver Stuff
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical (gsd, '\r','\n');
    gsd_launch (gsd);

    double salinity = bot_param_get_double_or_fail (gsd->params, "site.salinity");
    bool freshwater = salinity < 5 ? 1 : 0;

    /*
    if (getopt_get_bool (gsd->gopt, "tare"))
        gsd_write (gsd, "#Z1\n", 4);

    if (getopt_get_bool (gsd->gopt, "untare"))
        gsd_write (gsd, "#Z0\n", 4);

    */

    gsd_flush (gsd);
    gsd_reset_stats (gsd);
    while (1)
    {
        char buf[128];
        int64_t timestamp;
        int len = gsd_read (gsd, buf, sizeof buf, &timestamp);

        senlcm_dstar_ssp1_t dstar;
        if (parse_buf (buf, len, &dstar, freshwater))
        {
            dstar.utime = timestamp;
            if (0==strncmp (buf, "$YXDPT", 6))  // publish on depth event
            {
                if (0.0 <= dstar.temperature && dstar.temperature <= 35.0)
                    senlcm_dstar_ssp1_t_publish (gsd->lcm, gsd->channel, &dstar);
                else
                    ERROR ("temperature out of specified calibration range [0-35C]: T=%f\n", dstar.temperature);
            }
            gsd_update_stats (gsd, 1);
        }
        else
            gsd_update_stats (gsd, -1);
    } // while
}
