#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-common/getopt.h"
#include "perls-common/error.h"

#include "perls-math/rphcorr.h"

#include "perls-lcmtypes/senlcm_ms_gx3_25_t.h"

#include "ms_3dm_gx3.h"

#define INST_MAGFIELD SENLCM_MS_GX3_25_T_INST_MAGFIELD
#define INST_ACCEL    SENLCM_MS_GX3_25_T_INST_ACCEL
#define INST_ANGRATE  SENLCM_MS_GX3_25_T_INST_ANGRATE
#define STAB_MAGFIELD SENLCM_MS_GX3_25_T_STAB_MAGFIELD
#define STAB_ACCEL    SENLCM_MS_GX3_25_T_STAB_ACCEL
#define STAB_ANGRATE  SENLCM_MS_GX3_25_T_STAB_ANGRATE
#define EULER         SENLCM_MS_GX3_25_T_EULER

#define TEMPERATURE   SENLCM_MS_GX3_25_T_TEMPERATURE

#define DEV_TICKS_PER_SECOND (1.0/0.0065536)

uint16_t MagGainScale;
uint16_t AccelGainScale;
uint16_t GyroGainScale;

typedef struct _ms_query_t ms_query_t;
struct _ms_query_t
{
    uint8_t cmd_byte;
    uint8_t cmd_data[24];
    int cmd_data_len;
    int response_len;
};

static int
poll_data (generic_sensor_driver_t *gsd,
           ms_query_t *msq, uint8_t buf[],
           int64_t *utime)
{
    gsd_noncanonical (gsd, msq->response_len, 0);
    int ret = 0;
    while (ret == 0)
    {
        gsd_write (gsd, (char*)&msq->cmd_byte, 1);
        gsd_write (gsd, (char*)msq->cmd_data, msq->cmd_data_len);
        ret = gsd_read_timeout (gsd, (char*)buf, MICROSTRAIN_MAX_LEN, utime, 1000000);
    }
    return ret;
}

static void
pubEulerAng (generic_sensor_driver_t *gsd,
             timestamp_sync_state_t *tss,
             senlcm_ms_gx3_25_t *ms, rphcorr_t *rphcorr)
{
    uint8_t buf[MICROSTRAIN_MAX_LEN];

    ms_query_t msq =
    {
        .cmd_byte = CMD_EULER_ANG,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_EULER_ANG,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseEulerAng (buf, buflen, ms);

    if (ret > 0)
    {
        ms->bitmask = EULER;
        ms->utime = timestamp_sync (tss, ms->TimerTicks, ms->utime);

        senlcm_ms_gx3_25_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);

        publish_rphcorr (gsd->lcm, rphcorr, ms->utime, ms->Euler);
    }
    else
        gsd_update_stats (gsd, -1);
}



static void
pubStabVector (generic_sensor_driver_t *gsd,
               timestamp_sync_state_t *tss,
               senlcm_ms_gx3_25_t *ms)
{
    uint8_t buf[MICROSTRAIN_MAX_LEN];

    ms_query_t msq =
    {
        .cmd_byte = CMD_GYRO_STAB_ACCEL_ANG_RATE_MAG,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_GYRO_STAB_ACCEL_ANG_RATE_MAG,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseGyroVector (buf, buflen, ms);

    if (ret > 0)
    {
        ms->bitmask = STAB_MAGFIELD | STAB_ACCEL | STAB_ANGRATE;
        ms->utime = timestamp_sync (tss, ms->TimerTicks, ms->utime);

        senlcm_ms_gx3_25_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, -1);
}

static void
pubInstVector (generic_sensor_driver_t *gsd,
               timestamp_sync_state_t *tss,
               senlcm_ms_gx3_25_t *ms)
{
    uint8_t buf[MICROSTRAIN_MAX_LEN];

    ms_query_t msq =
    {
        .cmd_byte = CMD_ACCEL_ANG_RATE_MAG_VECTOR,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_ACCEL_ANG_RATE_MAG_VECTOR,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseInstVector (buf, buflen, ms);

    if (ret > 0)
    {
        ms->bitmask = INST_MAGFIELD | INST_ACCEL | INST_ANGRATE;
        ms->utime = timestamp_sync (tss, ms->TimerTicks, ms->utime);

        senlcm_ms_gx3_25_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, -1);
}

//static void
//pubGyroQuat (generic_sensor_driver_t *gsd,
//	     timestamp_sync_state_t *tss,
//	     senlcm_ms_gx3_25_t *ms)
//{
//    uint8_t buf[MICROSTRAIN_MAX_LEN];
//
//    ms_query_t msq = {
//        .cmd_byte = CMD_QUATERNION,
//        .cmd_data = "\0",
//        .cmd_data_len = 0,
//        .response_len = LEN_QUATERNION,
//    };
//    int buflen = poll_data (gsd, &msq, buf, &ms->utime);
//
//    int ret = ms_parseQuat (buf, buflen, ms);
//
//    if (ret > 0) {
//        ms->bitmask = STAB_Q;
//        ms->utime = timestamp_sync (tss, ms->TimerTicks, ms->utime);
//
//        ms_Q_to_M (ms->sQ, ms->sM);
//        ms->bitmask |= STAB_M;
//
//        ms_M_to_Euler (ms->sM, ms->sEuler);
//        ms->bitmask |= STAB_EULER;
//
//        senlcm_ms_gx3_25_t_publish (gsd->lcm, gsd->channel, ms);
//        gsd_update_stats (gsd, 1);
//    }
//    else
//        gsd_update_stats (gsd, -1);
//}

//static void
//pubEnableQuat (generic_sensor_driver_t *gsd,
//	       timestamp_sync_state_t *tss,
//	       senlcm_ms_gx3_25_t *ms) {
//
//  uint8_t buf[MICROSTRAIN_MAX_LEN];
//  uint8_t defaultBuf[LEN_SAMPLING_SETTINGS - 3];
//
//  ms_query_t querySettings = {
//    .cmd_byte = CMD_SAMPLING_SETTINGS,
//    .cmd_data = {0xA8, 0xB9, 0},
//    .cmd_data_len = 24, //Can't be 3 like gx1
//    .response_len = LEN_SAMPLING_SETTINGS,
//  };
//
//  int buflen = poll_data(gsd, &querySettings, buf, &ms->utime);
//
//  //get the "default settings" and put them in defaultBuf
//  ms_parseSettings(buf, buflen, ms, defaultBuf);
//
//  //We will use this command to turn on quat
//  ms_query_t turnOnQuat = {
//    .cmd_byte = CMD_SAMPLING_SETTINGS,
//    .cmd_data = {0xA8, 0xB9, 1},
//    .cmd_data_len = 24, //Can't be 3 like gx1
//    .response_len = LEN_SAMPLING_SETTINGS,
//  };
//
//  //Copy the corresponding default settings to turnOnQuat.
//  memcpy(turnOnQuat.cmd_data + 3, defaultBuf, LEN_SAMPLING_SETTINGS - 3);
//  //Enable this bit to turn on quat (see gx3 documentation)
//  turnOnQuat.cmd_data[5] = 0x10;
//
//  //Turn on quat, keep everything else default
//  buflen = poll_data(gsd, &turnOnQuat, buf, &ms->utime);
//
//}

static void
pubTemperature (generic_sensor_driver_t *gsd,
                timestamp_sync_state_t *tss,
                senlcm_ms_gx3_25_t *ms)
{
    uint8_t buf[MICROSTRAIN_MAX_LEN];
    ms_query_t msq =
    {
        .cmd_byte = CMD_TEMPERATURE,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_TEMPERATURE,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseTemperature (buf, buflen, ms);

    if (ret > 0)
    {
        ms->bitmask = TEMPERATURE;
        ms->utime = timestamp_sync (tss, (uint32_t) ms->TimerTicks, ms->utime);

        senlcm_ms_gx3_25_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, -1);
}

static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Microstrain 3DM-GX3-25 AHRS sensor driver.");
    getopt_add_bool (gsd->gopt, '\0', "stabVectors",
                     1, "Report stabilized vectors (default on)");
    getopt_add_bool (gsd->gopt, '\0', "instVectors",
                     0, "Report instantaneous vectors (default off)");
    getopt_add_bool (gsd->gopt, 'i', "ins",
                     0, "Run 3DM-GX3-45 in 3DM-GX3-25 mode");
    return 0;
}

int
main (int argc, char *argv[])
{
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_launch (gsd);
    timestamp_sync_state_t *tss = timestamp_sync_init (DEV_TICKS_PER_SECOND, 65536, 101.0/100.0);

    rphcorr_t *rphcorr = rphcorr_create (gsd->params, gsd->rootkey);
    if (!rphcorr)
    {
        ERROR ("rphcorr_create () failed!");
        exit (EXIT_FAILURE);
    }

    senlcm_ms_gx3_25_t ms;
    memset (&ms, 0, sizeof (ms));

    int success = 0;
    // put -45 in -25 mode
    if (getopt_get_bool (gsd->gopt, "ins"))
    {

        while (!success)
        {

            // might need to send pause command
            // hexadecimal 75 65 01 02 02 02 E1 c7
            printf ("Pausing ...\n");
            uint8_t buf_pause[8] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xE1, 0xc7};

            gsd_flush (gsd);
            gsd_write (gsd, (char *)buf_pause, 10);

            // wait for ack
            uint8_t ack_buf[10];
            gsd_noncanonical (gsd, 10, 0);
            int ret = gsd_read_timeout (gsd, (char *)ack_buf, 10, NULL, 1e6);

            // check correct length ACK
            // ack_buf[6] should be cmd desc echo
            // ack_buf[7] should be zero for no errors
            if (10 == ret && ack_buf[6] == 0x02 && ack_buf[7] == 0x00)
            {
                printf ("Success! \n");
                success = 1;
            }
            else
            {
                printf ("Failed! \n");
                for (int i=0; i<10; i++)
                    printf ("%02X ", ack_buf[i]);
                printf ("\n");
            }

            //hexadecimal 75 65 7f 04 04 10 01 02 74 bd
            printf ("Setting 3DM-GX3-45 in 3DM-GX3-25 mode ...\n");
            uint8_t buf[10] = {0x75, 0x65, 0x7F, 0x04, 0x04, 0x10, 0x01, 0x02, 0x00, 0x00};
            uint8_t byte1 = 0;
            uint8_t byte2 = 0;
            for (int i=0; i<8; i++)
            {
                byte1 += buf[i];
                byte2 += byte1;
            }
            buf[8] = byte1;
            buf[9] = byte2;

            gsd_flush (gsd);
            gsd_write (gsd, (char *)buf, 10);

            // wait for ack
            gsd_noncanonical (gsd, 10, 0);
            ret = gsd_read_timeout (gsd, (char *)ack_buf, 10, NULL, 1e6);

            // check correct length ACK
            // ack_buf[6] should be cmd desc echo
            // ack_buf[7] should be zero for no errors
            if (10 == ret && ack_buf[6] == 0x10 && ack_buf[7] == 0x00)
            {
                printf ("Success! \n");
                success &= 1;
            }
            else
            {
                printf ("Failed! \n");
                for (int i=0; i<10; i++)
                    printf ("%02X ", ack_buf[i]);
                printf ("\n");
                success = 0;
            }
        }
    }
    else
    {
        //pubEnableQuat(gsd, tss, &ms);
    }

    uint8_t iteration = 0;
    while (1)
    {

        if (getopt_get_bool (gsd->gopt, "stabVectors"))
            pubStabVector (gsd, tss, &ms);

        if (getopt_get_bool (gsd->gopt, "instVectors"))
            pubInstVector (gsd, tss, &ms);

        pubEulerAng (gsd, tss, &ms, rphcorr);

        if (iteration % 100 == 0)
            pubTemperature (gsd, tss, &ms);

        iteration++;
    }

    rphcorr_destroy (rphcorr);
}
