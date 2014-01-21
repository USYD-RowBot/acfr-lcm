#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"

#include "perls-common/error.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/units.h"

#include "perls-sensors/rdi.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RDI_DEV_TICKS_PER_SECOND (100) //  1/tofp_hundredth
#define RDI_DEV_TICKS_WRAPAROUND (24*3600*100)

static rdi_pd_mode_t rdi_pd_mode = RDI_PD4_MODE;
static int rdi_pd_len = RDI_PD4_LEN;

static int64_t
rdi_timestamp_sync (timestamp_sync_state_t *tss, int64_t tofp_hours, int64_t tofp_minute,
                    int64_t tofp_second, int64_t tofp_hundredth, int64_t host_utime)
{
    int64_t dev_ticks = (tofp_hours*3600 + tofp_minute*60 + tofp_second)*100 + tofp_hundredth;
    return timestamp_sync (tss, dev_ticks, host_utime);
}

static void
program_dvl (generic_sensor_driver_t *gsd, const char *config)
{
    if (gsd->io == GSD_IO_PLAYBACK)
        return;

    printf ("Initializing DVL according to %s\n", config);
    FILE *fptr = fopen (config, "r");
    if (fptr == NULL) {
        PERROR ("fopen()");
        exit (EXIT_FAILURE);
    }

    /* block until we know dvl is alive */
    char buf[256];
    do {
        const char sw_break[] = "===\n";  // software break
        gsd_write (gsd, sw_break, strlen (sw_break));
    } while (!gsd_read_timeout (gsd, buf, sizeof buf, NULL, 100E3));
    printf ("\n");

    char line[256];
    while (gsd_read_timeout (gsd, line, sizeof line, NULL, 100E3)>0)
        printf ("%s", line);

    printf ("--------------------programming---------------------------------\n");
    while (fgets (line, sizeof line, fptr)) {
        if (line[0] == ';') continue; // comment

        printf ("%s", line);
        gsd_write (gsd, line, strlen (line));

        if (0==strncmp (line, "#PD4", 4)) {
            rdi_pd_mode = RDI_PD4_MODE;
            rdi_pd_len  = RDI_PD4_LEN;
        }
        if (0==strncmp (line, "#PD5", 4)) {
            rdi_pd_mode = RDI_PD5_MODE;
            rdi_pd_len  = RDI_PD5_LEN;
        }
        timeutil_usleep (100E3); // need to give DVL some time for command to take
    } // while
    fclose (fptr);
    printf ("--------------------done-----------------------------------------\n");
}


static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "RDI Workhorse & Explorer DVL sensor driver.");
    getopt_add_string (gsd->gopt, '\0', "config", "", "RDI initialization configuration script.");
    return 0;
}


int
main (int argc, char *argv[])
{
    timestamp_sync_state_t *tss = timestamp_sync_init (RDI_DEV_TICKS_PER_SECOND, RDI_DEV_TICKS_WRAPAROUND, 1.01);

    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, &myopts);
    gsd_launch (gsd);

    // initialize dvl
    char dvlconfig[PATH_MAX] = {'\0'};
    if (getopt_has_flag (gsd->gopt, "config"))
        strcpy (dvlconfig, getopt_get_string (gsd->gopt, "config"));
    else {
        char key[256];
        sprintf (key, "%s.%s", gsd->rootkey, "config");

        char *config;
        if (0==bot_param_get_str (gsd->params, key, &config))
            snprintf (dvlconfig, sizeof dvlconfig, "%s/%s", gsd->rootdir, config);
        else {
            ERROR ("bot_param_get_str(), unable to parse config key [%s]", key);
            exit (EXIT_FAILURE);
        }
    }
    program_dvl (gsd, dvlconfig);

    gsd_noncanonical (gsd, rdi_pd_len, 1);
    gsd_flush (gsd);
    gsd_reset_stats (gsd);
    while (1) {
        // read stream
        char buf[1024];
        int64_t timestamp;
        int len = gsd_read (gsd, buf, rdi_pd_len, &timestamp);

        // try to parse it
        switch (rdi_pd_mode) {   
        case RDI_PD4_MODE: {
            rdi_pd4_t pd4;
            if (0==rdi_parse_pd4 (buf, len, &pd4)) {
                senlcm_rdi_pd4_t lcm_pd4 = rdi_pd4_to_lcm_pd4 (&pd4);
                lcm_pd4.utime = rdi_timestamp_sync (tss, pd4.tofp_hour, pd4.tofp_minute, pd4.tofp_second, 
                                                    pd4.tofp_hundredth, timestamp);
                senlcm_rdi_pd4_t_publish (gsd->lcm, gsd->channel, &lcm_pd4);
                gsd_update_stats (gsd, true);
            }
            else
                gsd_update_stats (gsd, false);

            break;
        }
        case RDI_PD5_MODE: {
            rdi_pd5_t pd5;
            if (0==rdi_parse_pd5 (buf, len, &pd5)) {
                senlcm_rdi_pd5_t lcm_pd5 = rdi_pd5_to_lcm_pd5 (&pd5);
                lcm_pd5.utime = lcm_pd5.pd4.utime = rdi_timestamp_sync (tss, pd5.tofp_hour, pd5.tofp_minute, 
                                                                        pd5.tofp_second, pd5.tofp_hundredth, timestamp);
                senlcm_rdi_pd5_t_publish (gsd->lcm, gsd->channel, &lcm_pd5);
                gsd_update_stats (gsd, true);
            }
            else
                gsd_update_stats (gsd, false);

            break;
        }
        default:
            ERROR ("unsupported PD mode by driver");
            gsd_update_stats (gsd, -1);
        } // switch

    } // while

    timestamp_sync_free (tss);
}
