#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/timestamp.h"
#include "perls-common/getopt.h"
#include "perls-common/error.h"
#include "perls-common/generic_sensor_driver.h"

#include "perls-lcmtypes/senlcm_ms_gx1_t.h"

#include "ms_3dm_gx1.h"

#define INST_MAGFIELD SENLCM_MS_GX1_T_INST_MAGFIELD
#define INST_ACCEL    SENLCM_MS_GX1_T_INST_ACCEL
#define INST_ANGRATE  SENLCM_MS_GX1_T_INST_ANGRATE
#define INST_EULER    SENLCM_MS_GX1_T_INST_EULER
#define INST_M        SENLCM_MS_GX1_T_INST_M
#define INST_Q        SENLCM_MS_GX1_T_INST_Q
#define STAB_MAGFIELD SENLCM_MS_GX1_T_STAB_MAGFIELD
#define STAB_ACCEL    SENLCM_MS_GX1_T_STAB_ACCEL
#define STAB_ANGRATE  SENLCM_MS_GX1_T_STAB_ANGRATE
#define STAB_EULER    SENLCM_MS_GX1_T_STAB_EULER
#define STAB_M        SENLCM_MS_GX1_T_STAB_M
#define STAB_Q        SENLCM_MS_GX1_T_STAB_Q
#define TEMPERATURE   SENLCM_MS_GX1_T_TEMPERATURE


#define DEV_TICKS_PER_SECOND (1.0/0.0065536)

uint16_t MagGainScale;
uint16_t AccelGainScale;
uint16_t GyroGainScale;

typedef struct _ms_query_t ms_query_t;
struct _ms_query_t
{
    char cmd_byte;
    char cmd_data[24];
    int  cmd_data_len;
    int  response_len;
};

static int
poll_data (generic_sensor_driver_t *gsd, ms_query_t *msq, char buf[], int64_t *utime)
{
    gsd_noncanonical (gsd, msq->response_len, 0);
    int ret = 0;
    while (ret == 0)
    {
        gsd_write (gsd, &msq->cmd_byte, 1);
        gsd_write (gsd, msq->cmd_data, msq->cmd_data_len);
        ret = gsd_read_timeout (gsd, buf, MICROSTRAIN_MAX_LEN, utime, 1000000);
    }
    return ret;
}

static void
pubGyroVector (generic_sensor_driver_t *gsd, timestamp_sync_state_t *tss, senlcm_ms_gx1_t *ms)
{
    char buf[MICROSTRAIN_MAX_LEN];

    ms_query_t msq =
    {
        .cmd_byte = CMD_GYRO_VECTOR,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_GYRO_VECTOR,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseGyroVector (buf, buflen,
                                  ms->sMagField, MagGainScale,
                                  ms->sAccel,    AccelGainScale,
                                  ms->sAngRate,  GyroGainScale,
                                  &ms->TimerTicks);
    if (ret > 0)
    {
        ms->bitmask = STAB_MAGFIELD | STAB_ACCEL | STAB_ANGRATE;
        ms->utime = timestamp_sync (tss, (uint16_t) ms->TimerTicks, ms->utime);

        senlcm_ms_gx1_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, 0);
}

static void
pubInstVector (generic_sensor_driver_t *gsd, timestamp_sync_state_t *tss, senlcm_ms_gx1_t *ms)
{
    char buf[MICROSTRAIN_MAX_LEN];

    ms_query_t msq =
    {
        .cmd_byte = CMD_INST_VECTOR,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_INST_VECTOR,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseInstVector (buf, buflen,
                                  ms->iMagField, MagGainScale,
                                  ms->iAccel, AccelGainScale,
                                  ms->iAngRate, GyroGainScale,
                                  &ms->TimerTicks);
    if (ret > 0)
    {
        ms->bitmask = INST_MAGFIELD | INST_ACCEL | INST_ANGRATE;
        ms->utime = timestamp_sync (tss, (uint16_t) ms->TimerTicks, ms->utime);

        senlcm_ms_gx1_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, 0);
}

static void
pubInstQuat (generic_sensor_driver_t *gsd, timestamp_sync_state_t *tss, senlcm_ms_gx1_t *ms)
{
    char buf[MICROSTRAIN_MAX_LEN];

    ms_query_t msq =
    {
        .cmd_byte = CMD_INST_QUAT,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_INST_QUAT,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseInstQuat (buf, buflen, ms->iQ, &ms->TimerTicks);
    if (ret > 0)
    {
        ms->bitmask = INST_Q;
        ms->utime = timestamp_sync (tss, (uint16_t) ms->TimerTicks, ms->utime);

        ms_Q_to_M (ms->iQ, ms->iM);
        ms->bitmask |= INST_M;

        ms_M_to_Euler (ms->iM, ms->iEuler);
        ms->bitmask |= INST_EULER;

        senlcm_ms_gx1_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, 0);
}

static void
pubGyroQuat (generic_sensor_driver_t *gsd, timestamp_sync_state_t *tss, senlcm_ms_gx1_t *ms)
{
    char buf[MICROSTRAIN_MAX_LEN];

    ms_query_t msq =
    {
        .cmd_byte = CMD_GYRO_QUAT,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_GYRO_QUAT,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseGyroQuat (buf, buflen, ms->sQ, &ms->TimerTicks);
    if (ret > 0)
    {
        ms->bitmask = STAB_Q;
        ms->utime = timestamp_sync (tss, (uint16_t) ms->TimerTicks, ms->utime);

        ms_Q_to_M (ms->sQ, ms->sM);
        ms->bitmask |= STAB_M;

        ms_M_to_Euler (ms->sM, ms->sEuler);
        ms->bitmask |= STAB_EULER;

        senlcm_ms_gx1_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, 0);
}


static void
pubGyroQuatVector (generic_sensor_driver_t *gsd, timestamp_sync_state_t *tss, senlcm_ms_gx1_t *ms)
{
    char buf[MICROSTRAIN_MAX_LEN];

    ms_query_t msq =
    {
        .cmd_byte = CMD_GYRO_QUAT_VECTOR,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_GYRO_QUAT_VECTOR,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseGyroQuatVector (buf, buflen,
                                      ms->sQ,
                                      ms->iMagField, MagGainScale,
                                      ms->iAccel, AccelGainScale,
                                      ms->sAngRate, GyroGainScale,
                                      &ms->TimerTicks);
    if (ret > 0)
    {
        ms->bitmask = STAB_Q | INST_MAGFIELD | INST_ACCEL | STAB_ANGRATE;
        ms->utime = timestamp_sync (tss, (uint16_t) ms->TimerTicks, ms->utime);

        ms_Q_to_M (ms->sQ, ms->sM);
        ms->bitmask |= STAB_M;

        ms_M_to_Euler (ms->sM, ms->sEuler);
        ms->bitmask |= STAB_EULER;

        senlcm_ms_gx1_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, 0);
}

static void
pubTemperature (generic_sensor_driver_t *gsd, timestamp_sync_state_t *tss, senlcm_ms_gx1_t *ms)
{
    char buf[MICROSTRAIN_MAX_LEN];
    ms_query_t msq =
    {
        .cmd_byte = CMD_TEMPERATURE,
        .cmd_data = "\0",
        .cmd_data_len = 0,
        .response_len = LEN_TEMPERATURE,
    };
    int buflen = poll_data (gsd, &msq, buf, &ms->utime);

    int ret = ms_parseTemperature (buf, buflen, &ms->Temperature, &ms->TimerTicks);
    if (ret > 0)
    {
        ms->bitmask = TEMPERATURE;
        ms->utime = timestamp_sync (tss, (uint16_t) ms->TimerTicks, ms->utime);

        senlcm_ms_gx1_t_publish (gsd->lcm, gsd->channel, ms);
        gsd_update_stats (gsd, 1);
    }
    else
        gsd_update_stats (gsd, 0);
}

static uint16_t
getMagGainScale (generic_sensor_driver_t *gsd)
{
    char buf[MICROSTRAIN_MAX_LEN];
    ms_query_t msq =
    {
        .cmd_byte = CMD_READ_EEPROM_WITH_CHECKSUM,
        .cmd_data = {0, 232},
        .cmd_data_len = 2,
        .response_len = LEN_READ_EEPROM_WITH_CHECKSUM,
    };
    int64_t utime;
    int buflen = poll_data (gsd, &msq, buf, &utime);

    uint16_t MagGainScale;
    int16_t TimerTicks;
    int ret = ms_parseReadEepromWithChecksum (buf, buflen, &MagGainScale, &TimerTicks);
    if (ret > 0)
    {
        gsd_update_stats (gsd, 1);
        return MagGainScale;
    }
    else
    {
        gsd_update_stats (gsd, 0);
        ERROR("using default uncalibrated value MagGainScale=2000");
        return 2000;
    }
}

static uint16_t
getAccelGainScale (generic_sensor_driver_t *gsd)
{
    char buf[MICROSTRAIN_MAX_LEN];
    ms_query_t msq =
    {
        .cmd_byte = CMD_READ_EEPROM_WITH_CHECKSUM,
        .cmd_data = {0, 230},
        .cmd_data_len = 2,
        .response_len = LEN_READ_EEPROM_WITH_CHECKSUM,
    };
    int64_t utime;
    int buflen = poll_data (gsd, &msq, buf, &utime);

    uint16_t AccelGainScale;
    int16_t TimerTicks;
    int ret = ms_parseReadEepromWithChecksum (buf, buflen, &AccelGainScale, &TimerTicks);
    if (ret > 0)
    {
        gsd_update_stats (gsd, 1);
        return AccelGainScale;
    }
    else
    {
        gsd_update_stats (gsd, 0);
        ERROR("using default uncalibrated value AccelGainScale=7000");
        return 7000;
    }
}

static uint16_t
getGyroGainScale (generic_sensor_driver_t *gsd)
{
    char buf[MICROSTRAIN_MAX_LEN];
    ms_query_t msq =
    {
        .cmd_byte = CMD_READ_EEPROM_WITH_CHECKSUM,
        .cmd_data = {0, 130},
        .cmd_data_len = 2,
        .response_len = LEN_READ_EEPROM_WITH_CHECKSUM,
    };
    int64_t utime;
    int buflen = poll_data (gsd, &msq, buf, &utime);

    uint16_t GyroGainScale;
    int16_t TimerTicks;
    int ret = ms_parseReadEepromWithChecksum (buf, buflen, &GyroGainScale, &TimerTicks);
    if (ret > 0)
    {
        gsd_update_stats (gsd, 1);
        return GyroGainScale;
    }
    else
    {
        gsd_update_stats (gsd, 0);
        ERROR("using default uncalibrated value GyroGainScale=8500");
        return 8500;
    }
}


static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Microstrain 3DM-GX1 AHRS sensor driver.");
    getopt_add_bool (gsd->gopt, '\0', "stabAttitude", 0, "Report stabilized attitude (default)");
    getopt_add_bool (gsd->gopt, '\0', "stabVectors",  0, "Report stabilized vectors (default)");
    getopt_add_bool (gsd->gopt, '\0', "instAttitude", 0, "Report instantaneous attitude (default)");
    getopt_add_bool (gsd->gopt, '\0', "instVectors",  0, "Report instantaneous vectors (default)");
    return 0;
}


int
main (int argc, char *argv[])
{
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_launch (gsd);

    timestamp_sync_state_t *tss = timestamp_sync_init (DEV_TICKS_PER_SECOND, 65536, 101.0/100.0);

    senlcm_ms_gx1_t ms;
    memset (&ms, 0, sizeof (ms));

    int all = 1;
    if (getopt_has_flag (gsd->gopt, "stabAttitude") || getopt_has_flag (gsd->gopt, "stabVectors")  ||
            getopt_has_flag (gsd->gopt, "instAttitude") || getopt_has_flag (gsd->gopt, "instVectors"))
        all = 0;

    MagGainScale = ms.MagGainScale = getMagGainScale (gsd);
    AccelGainScale = ms.AccelGainScale = getAccelGainScale (gsd);
    GyroGainScale = ms.GyroGainScale = getGyroGainScale (gsd);

    uint8_t i=0;
    while (1)
    {
        if (all || getopt_get_bool (gsd->gopt, "stabAttitude"))
            pubGyroQuat (gsd, tss, &ms);

        if (all || getopt_get_bool (gsd->gopt, "stabVectors"))
            pubGyroVector (gsd, tss, &ms);

        if (all || getopt_get_bool (gsd->gopt, "instAttitude"))
            pubInstQuat (gsd, tss, &ms);

        if (all || getopt_get_bool (gsd->gopt, "instVectors"))
            pubInstVector (gsd, tss, &ms);

        if ( (i++ % 10)==0 )
            pubTemperature (gsd, tss, &ms);
    }
}
