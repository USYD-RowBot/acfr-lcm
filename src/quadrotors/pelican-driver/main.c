/**
 * @author Paul Ozog - paulozog@umich.edu
 *
 * @file pelican/main.c
 *
 * @brief Telemetry driver for Asc Tech Pelican UAV
 */
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/timestamp.h"
#include "perls-common/getopt.h"
#include "perls-common/error.h"
#include "perls-common/generic_sensor_driver.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/senlcm_pelican_t.h"

#include "pelican.h"

/**
 * @brief Send the command to poll for telemetry
 * @param gsd : the program's generic sensor driver
 *
 * @param len : the size of the struct you want to poll (ie: the size
 * of the LL_STATUS struct)
 *
 * @param msg : Should contain the preamble ">*>p" and the type of the
 * struct you wish to poll
 *
 * @param buf : Will store the data here
 *
 * @param utime : Needed for timeout
 *
 * @return length of buf
 */
static int
poll_data (generic_sensor_driver_t *gsd, 
           uint16_t len,
           pelic_query_t *msq, uint8_t buf[], 
           int64_t *utime)
{
    gsd_noncanonical (gsd, len, 0);

    int length = 0;

    while (length == 0) {
        gsd_write (gsd, (char*)&msq->preamble, 4);
        gsd_write (gsd, (char*)&msq->packetType, sizeof(uint16_t));
        length = gsd_read_timeout (gsd, (char*)buf, PELIC_MAX_LEN, utime, 100000);
    }

    return length;
}

/**
 * @brief Wrapper to poll_data, update GSD stats for "good" and "bad"
 * packets
 *
 * @param  gsd 
 *
 * @param packetType : The type of packet we wish to poll (see
 * documentation for Pelican)
 *
 * @param len : The length of the struct we are polling (see
 * documentation for Pelican)
 *
 * @param pelic : lcm struct for Pelican
 *
 * @return None
 */
static void 
pollPelic (generic_sensor_driver_t *gsd,
           uint16_t packetType,
           uint16_t len,
           senlcm_pelican_t      *pelic) {
  
    uint8_t buf[PELIC_MAX_LEN];
    int bufLen, success;

    pelic_query_t queryMsg = {
        .preamble	= PELIC_QUERY_STR,
        .packetType = packetType,
    };

    bufLen = poll_data (gsd, len, &queryMsg, buf, &pelic->utime);

    success = pelic_parsePacket (buf, bufLen, pelic);

    if (success > 0) {
        pelic->utime = timestamp_now ();
        senlcm_pelican_t_publish (gsd->lcm, gsd->channel, pelic);
        gsd_update_stats (gsd,1);
    }
    else
        gsd_update_stats (gsd, 0);
}

/**
 * @brief Extend generic_sensor_driver_t's command line arguments
 *
 * @param gsd
 *
 * @return 0 for sucess, ? for not sucess
 */
static int
getOpts (generic_sensor_driver_t* gsd)
{  
    getopt_add_description (gsd->gopt, "Pelican pelican sensor driver.");
    return 0;
}

/**
 * @brief Poll for desired data types.
 *
 * @details Get lowlevel (LL) status, calc IMU data, and GPS data
 * every iteration.  Get remote control (RC) data every 5 iterations
 */
int
main (int argc, char *argv[])
{  
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, getOpts);

    gsd_launch (gsd);

    senlcm_pelican_t pelic = {0};

    uint8_t iteration = 0;
    while (1) {
    
        pollPelic(gsd, PELIC_LLSTATUS_CMD,
                  sizeof (pelic_LlStatusStruct) + 11, &pelic);

        pollPelic(gsd, PELIC_IMU_CALCDATA_CMD,
                  sizeof (pelic_ImuCalcDataStruct) + 11, &pelic);

        pollPelic(gsd, PELIC_GPS_DATA_CMD,
                  sizeof (pelic_GpsDataStruct) + 11, &pelic);

        if (iteration % 5 == 0)
            pollPelic(gsd, PELIC_RC_DATA_CMD,
                      sizeof (pelic_RcDataStruct) + 11, &pelic);
    
        iteration++;
    }

    return 0;
}
