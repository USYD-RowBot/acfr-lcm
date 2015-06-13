#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <gps.h> // from debian package libgps-dev

#include <bot/bot_core.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/getopt.h"
#include "perls-common/nmea.h"
#include "perls-common/units.h"
#include "perls-common/generic_sensor_driver.h"

#include "perls-lcmtypes/senlcm_raw_t.h"
#include "perls-lcmtypes/senlcm_ppsboard_t.h"
#include "perls-lcmtypes/senlcm_gpsd_t.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

static generic_sensor_driver_t *gsd;

static senlcm_gpsd_t lcm_gps_data;
static senlcm_ppsboard_t lcm_ppsboard;

static void
init_lcm_gps_data (senlcm_gpsd_t *gd)
{
    const int max_sats = 20;
    gd->used = malloc (max_sats * sizeof (*gd->used));
    gd->PRN = malloc (max_sats * sizeof (*gd->PRN));
    gd->elevation = malloc (max_sats * sizeof (*gd->elevation));
    gd->azimuth = malloc (max_sats * sizeof (*gd->azimuth));
    gd->ss = malloc (max_sats * sizeof (*gd->ss));
}

static void
init_lcm_ppsboard (senlcm_ppsboard_t *pb)
{
    pb->ntp_time = malloc (8 * sizeof (*pb->ntp_time));
    pb->src_type = malloc (8 * sizeof (*pb->ntp_time));
    pb->sync_date = malloc (8 * sizeof (*pb->ntp_time));
    pb->sync_time = malloc (8 * sizeof (*pb->ntp_time));
}

static void
free_lcm_gps_data (senlcm_gpsd_t *gd)
{
    free (gd->used);
    free (gd->PRN);
    free (gd->elevation);
    free (gd->azimuth);
    free (gd->ss);
}

static void
free_lcm_ppsboard (senlcm_ppsboard_t *pb)
{
    free (pb->ntp_time);
    free (pb->src_type);
    free (pb->sync_date);
    free (pb->sync_time);
}

static int
parse_gpsd (const struct gps_data_t *ud, senlcm_gpsd_t *gd)
{
    gd->utime = timestamp_now ();
    gd->online    = (int64_t) (ud->online * 1.0E6);

    /* gps_fix */
    gd->fix_utime = ud->fix.time * 1E6;
    gd->mode      = ud->fix.mode;
    gd->ept       = ud->fix.ept;
    gd->latitude  = ud->fix.latitude * DTOR;
    gd->longitude = ud->fix.longitude * DTOR;
    gd->eph       = ud->fix.eph;
    gd->altitude  = ud->fix.altitude;
    gd->epv       = ud->fix.epv;
    gd->track     = ud->fix.track;
    gd->epd       = ud->fix.epd;
    gd->speed     = ud->fix.speed;
    gd->eps       = ud->fix.eps;
    gd->climb     = ud->fix.climb;
    gd->epc       = ud->fix.epc;

    gd->geoidal_separation = ud->separation;

    gd->satellites_used = ud->satellites_used;
    for (int i=0; i<ud->satellites_used; i++)
        gd->used[i] = ud->used[i];

    gd->status = ud->status;
    gd->pdop   = ud->pdop;
    gd->hdop   = ud->hdop;
    gd->tdop   = ud->tdop;
    gd->gdop   = ud->gdop;
    gd->epe    = ud->epe;

    gd->satellites_visible = ud->satellites;
    for (size_t i=0; i<gd->satellites_visible; i++)
    {
        gd->PRN[i] = ud->PRN[i];
        gd->elevation[i] = ud->elevation[i];
        gd->azimuth[i] = ud->azimuth[i];
        gd->ss[i] = ud->ss[i];
    }

    if (ud->gps_device != NULL)
        gd->gps_device = (char *) ud->gps_device;
    else
        gd->gps_device = "";

    if (ud->gps_id != NULL)
        gd->gps_id = (char *) ud->gps_id;
    else
        gd->gps_id = "";

    gd->ndevices = ud->ndevices;
    gd->devicelist = ud->devicelist;

    return 1;
}

static int
parse_ppsda (const char *buf, senlcm_ppsboard_t *ppsboard)
{
    ppsboard->utime = timestamp_now ();

    if (!nmea_arg (buf, 1, ppsboard->ntp_time))
        return -1;

    char ntp_status;
    if (!nmea_argc (buf, 2, &ntp_status))
        return -1;
    else
    {
        if (ntp_status == 'A')
            ppsboard->ntp_status = 1;
        else
            ppsboard->ntp_status = 0;
    }

    char src_type;
    if (!nmea_argc (buf, 4, &src_type))
        return -1;
    else
    {
        if (src_type == 'I')
            ppsboard->src_type = "IVER";
        else if (src_type == 'G')
            ppsboard->src_type = "GARMIN";
        else
            ppsboard->src_type = "UNKNOWN";
    }

    if (!nmea_argi (buf, 5, (int *) &ppsboard->src_pps))
        return -1;

    if (!nmea_argi (buf, 7, (int *) &ppsboard->sync_mode))
        return -1;

    if (!nmea_argi (buf, 8, (int *) &ppsboard->sync_num))
        return -1;

    if (!nmea_arg  (buf, 9, ppsboard->sync_date))
        *ppsboard->sync_date = '\0';

    if (!nmea_arg  (buf, 10, ppsboard->sync_time))
        *ppsboard->sync_time = '\0';

    if (!nmea_argi (buf, 12, (int *) &ppsboard->offset_counts))
        ppsboard->offset_counts = atoi ("NaN");

    if (!nmea_argf (buf, 13, &ppsboard->offset_usecs))
        ppsboard->offset_usecs = atof ("NaN");

    if (!nmea_argf (buf, 15, &ppsboard->temperature))
        return -1;

    return 1;
}

static void
on_clean_data (struct gps_data_t *ud, char *buf, size_t ulen, int level)
{
    if (buf[0] == '$')   // raw NMEA pkt
    {
        senlcm_raw_t raw;
        raw.utime = timestamp_now ();
        raw.length = ulen;
        raw.data = (uint8_t *) buf;

        if (0==strncmp (buf, "$PPSDA", 6))   // $PPSDA
        {
            static char ppsboard_channel[] = "PPSBOARD";
            static char *ppsboard_read_channel = NULL;
            if (!ppsboard_read_channel)
            {
                ppsboard_read_channel = malloc (LCM_MAX_CHANNEL_NAME_LENGTH);
                char *raw_delim = strtok (gsd->read_channel, gsd->channel);
                sprintf (ppsboard_read_channel, "%s%s", ppsboard_channel, raw_delim);
            }
            senlcm_raw_t_publish (gsd->lcm, ppsboard_read_channel, &raw);

            if (parse_ppsda (buf, &lcm_ppsboard))
            {
                senlcm_ppsboard_t_publish (gsd->lcm, ppsboard_channel, &lcm_ppsboard);
                gsd_update_stats (gsd, 1);
            }
            else
                gsd_update_stats (gsd, -1);
        }
        else  // generic NMEA packet
            senlcm_raw_t_publish (gsd->lcm, gsd->read_channel, &raw);
    }
    else   // gpsd packet
    {
        if (0==strncmp (buf, "GPSD,O=RMC", 10))
        {
            if (parse_gpsd (ud, &lcm_gps_data))
            {
                senlcm_gpsd_t_publish (gsd->lcm, gsd->channel, &lcm_gps_data);
                gsd_update_stats (gsd, 1);
            }
            else
                gsd_update_stats (gsd, -1);
        }
    }
}

static void
on_data (struct gps_data_t *ud, char *buf, size_t ulen, int level)
{
    if (buf[0] == '$' && strstr (buf, "GPSD,"))
    {
        //printf ("\nGPSD weirdness:\n");

        char mybuf[ulen+1];
        memcpy (mybuf, buf, ulen);
        mybuf[ulen] = '\0';

        char *tok = strchr (mybuf, '\n');
        if (tok)
        {
            mybuf[tok-mybuf] = '\0';
            //printf ("mybuf=%s\n", mybuf);
            on_clean_data (ud, mybuf, strlen (mybuf), level);
            tok++;
            //printf ("tok=%s\n", tok);
            on_clean_data (ud, tok, strlen (tok), level);

        }
    }
    else
        on_clean_data (ud, buf, ulen, level);
}

static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "gpsd client driver.");
    getopt_add_string (gsd->gopt, 's', "server", "127.0.0.1", "gpsd server");
    getopt_add_int    (gsd->gopt, 'p', "port",   "2947",      "gpsd port");
    getopt_add_string (gsd->gopt, '\0', "gpsddev", "",        "gpsd device");
    return 0;
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    gsd = gsd_create (argc, argv, NULL, myopts);

    //gsd_reset_stats (gsd);

    init_lcm_gps_data (&lcm_gps_data);
    init_lcm_ppsboard (&lcm_ppsboard);


    // what's our gps device?
    char *gpsddev;
    if (getopt_has_flag (gsd->gopt, "gpsddev"))
        gpsddev = (char *) getopt_get_string (gsd->gopt, "gpsddev");
    else
    {
        char key[256];
        sprintf (key, "%s.gpsddev", gsd->rootkey);
        if (bot_param_get_str (gsd->params, key, &gpsddev))
        {
            ERROR ("gpsddev not set");
            exit (EXIT_FAILURE);
        }
    }

    // open gpsd client
    struct gps_data_t *gpsdata;
    gpsdata = gps_open (NULL, NULL);
    if (gpsdata == NULL)
    {
        ERROR ("gps_open() failed.");
        exit (EXIT_FAILURE);
    }

    gps_set_raw_hook (gpsdata, on_data);
    char query[256];
    sprintf (query, "j=0 k r+ w+ f=%s\n", gpsddev);
    gps_query (gpsdata, query);


    while (!gsd->done)
        gps_poll (gpsdata);

    gps_close (gpsdata);
    free_lcm_gps_data (&lcm_gps_data);
    free_lcm_ppsboard (&lcm_ppsboard);

    return 0;
}
