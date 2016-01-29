#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "perls-common/textread.h"

#include "lcmlog_export.h"
#include "senlcm.h"

void
senlcm_acomms_range_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                               const senlcm_acomms_range_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",   "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "time_source", "%"PRId8, msg->time_source);

    TEXTREAD_ADD_FIELD (tr, "src",     "%"PRId8,   msg->src);

    TEXTREAD_ADD_FIELD (tr, "nowtt",   "%"PRId32,  msg->nowtt);
    for (int i = 0; i < 4; i++)
    {
        char buf[32];
        snprintf (buf, sizeof buf, "owtt(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%f", i < msg->nowtt ? msg->owtt[i] : 0);
    }

    TEXTREAD_ADD_FIELD (tr, "type",      "%"PRId8, msg->type);

    TEXTREAD_ADD_FIELD (tr, "sender_clk_mode",   "%"PRId8, msg->sender_clk_mode);
    TEXTREAD_ADD_FIELD (tr, "receiver_clk_mode", "%"PRId8, msg->receiver_clk_mode);

    TEXTREAD_ADD_CONST (tr, "TWO_WAY_PING",         SENLCM_ACOMMS_RANGE_T_TWO_WAY_PING);
    TEXTREAD_ADD_CONST (tr, "ONE_WAY_SYNC",         SENLCM_ACOMMS_RANGE_T_ONE_WAY_SYNCHRONOUS);
    TEXTREAD_ADD_CONST (tr, "REMUS_LBL",            SENLCM_ACOMMS_RANGE_T_REMUS_LBL);
    TEXTREAD_ADD_CONST (tr, "NARROWBAND_LBL",       SENLCM_ACOMMS_RANGE_T_NARROWBAND_LBL);

    TEXTREAD_ADD_CONST (tr, "CLK_MODE_NO_PPS_CLK_BAD",  SENLCM_ACOMMS_RANGE_T_NO_SYNC_TO_PPS_AND_CCCLK_BAD);
    TEXTREAD_ADD_CONST (tr, "CLK_MODE_NO_PPS_CLK_GOOD", SENLCM_ACOMMS_RANGE_T_NO_SYNC_TO_PPS_AND_CCCLK_GOOD);
    TEXTREAD_ADD_CONST (tr, "CLK_MODE_PPS_CLK_BAD",     SENLCM_ACOMMS_RANGE_T_SYNC_TO_PPS_AND_CCCLK_BAD);
    TEXTREAD_ADD_CONST (tr, "CLK_MODE_PPS_CLK_GOOD",    SENLCM_ACOMMS_RANGE_T_SYNC_TO_PPS_AND_CCCLK_GOOD);

    textread_stop (tr);
}

void
senlcm_dstar_ssp1_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const senlcm_dstar_ssp1_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",       "%"PRId64,msg->utime);
    TEXTREAD_ADD_FIELD (tr, "p_abs",       "%f",     msg->p_abs);
    TEXTREAD_ADD_FIELD (tr, "p_gage",      "%f",     msg->p_gage);
    TEXTREAD_ADD_FIELD (tr, "p_atm",       "%f",     msg->p_atm);
    TEXTREAD_ADD_FIELD (tr, "depth",       "%f",     msg->depth);
    TEXTREAD_ADD_FIELD (tr, "temperature", "%f",     msg->temperature);
    textread_stop (tr);

}


void
senlcm_gpsd3_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const senlcm_gpsd3_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",      "%"PRId64,   msg->utime);
    TEXTREAD_ADD_FIELD (tr, "online",     "%"PRId64,   msg->online);
    TEXTREAD_ADD_FIELD (tr, "fix_utime",  "%"PRId64,   msg->fix.utime);
    TEXTREAD_ADD_FIELD (tr, "mode",       "%"PRId8,    msg->fix.mode);
    TEXTREAD_ADD_FIELD (tr, "ept",        "%f",        msg->fix.ept);
    TEXTREAD_ADD_FIELD (tr, "latitude",   "%.15f",     msg->fix.latitude);
    TEXTREAD_ADD_FIELD (tr, "longitude",  "%.15f",     msg->fix.longitude);
    TEXTREAD_ADD_FIELD (tr, "altitude",   "%f",        msg->fix.altitude);
    TEXTREAD_ADD_FIELD (tr, "epx",        "%f",        msg->fix.epx);
    TEXTREAD_ADD_FIELD (tr, "epy",        "%f",        msg->fix.epy);
    TEXTREAD_ADD_FIELD (tr, "epv",        "%f",        msg->fix.epv);
    TEXTREAD_ADD_FIELD (tr, "track",      "%f",        msg->fix.track);
    TEXTREAD_ADD_FIELD (tr, "epd",        "%f",        msg->fix.epd);
    TEXTREAD_ADD_FIELD (tr, "speed",      "%f",        msg->fix.speed);
    TEXTREAD_ADD_FIELD (tr, "eps",        "%f",        msg->fix.eps);
    TEXTREAD_ADD_FIELD (tr, "climb",      "%f",        msg->fix.climb);
    TEXTREAD_ADD_FIELD (tr, "epc",        "%f",        msg->fix.epc);
    TEXTREAD_ADD_FIELD (tr, "geoidal_separation", "%f",msg->geoidal_separation);
    TEXTREAD_ADD_FIELD (tr, "status",             "%"PRId16,  msg->status);
    TEXTREAD_ADD_FIELD (tr, "satellites_used",    "%"PRId16,  msg->satellites_used);

    for (int i_used=0; i_used<12; i_used++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "used(:,%d)", i_used+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16, i_used < msg->satellites_used ? msg->used[i_used] : 0);
    }

    TEXTREAD_ADD_FIELD (tr, "xdop",          "%f",         msg->dop.xdop);
    TEXTREAD_ADD_FIELD (tr, "ydop",          "%f",         msg->dop.ydop);
    TEXTREAD_ADD_FIELD (tr, "pdop",          "%f",         msg->dop.pdop);
    TEXTREAD_ADD_FIELD (tr, "hdop",          "%f",         msg->dop.hdop);
    TEXTREAD_ADD_FIELD (tr, "vdop",          "%f",         msg->dop.vdop);
    TEXTREAD_ADD_FIELD (tr, "tdop",          "%f",         msg->dop.tdop);
    TEXTREAD_ADD_FIELD (tr, "gdop",          "%f",         msg->dop.gdop);
    TEXTREAD_ADD_FIELD (tr, "epe",           "%f",         msg->epe);
    TEXTREAD_ADD_FIELD (tr, "satellites_visible", "%"PRId16, msg->satellites_visible);

    for (int i_PRN=0; i_PRN<12; i_PRN++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "PRN(:,%d)", i_PRN+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16, i_PRN < msg->satellites_visible ? msg->PRN[i_PRN] : 0);
    }

    for (int i_elevation=0; i_elevation<12; i_elevation++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "elevation(:,%d)", i_elevation+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16,    i_elevation < msg->satellites_visible ? msg->elevation[i_elevation] : 0);
    }

    for (int i_azimuth=0; i_azimuth<12; i_azimuth++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "azimuth(:,%d)", i_azimuth+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16,    i_azimuth < msg->satellites_visible ? msg->azimuth[i_azimuth] : 0);
    }

    for (int i_ss=0; i_ss<12; i_ss++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "ss(:,%d)", i_ss+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16, i_ss < msg->satellites_visible ? msg->ss[i_ss] : 0);
    }

    TEXTREAD_ADD_FIELD (tr, "gps_path",      "%s",    msg->dev.path);
    TEXTREAD_ADD_FIELD (tr, "gps_flags",     "%d",    msg->dev.flags);
    TEXTREAD_ADD_FIELD (tr, "gps_driver",    "%s",    msg->dev.driver);
    TEXTREAD_ADD_FIELD (tr, "gps_subtype",   "%s",    msg->dev.subtype);
    TEXTREAD_ADD_FIELD (tr, "gps_activated", "%f",    msg->dev.activated);
    TEXTREAD_ADD_FIELD (tr, "baudrate",      "%d",    msg->dev.baudrate);
    TEXTREAD_ADD_FIELD (tr, "stopbits",      "%d",    msg->dev.stopbits);
    TEXTREAD_ADD_FIELD (tr, "cycle",         "%f",    msg->dev.cycle);
    TEXTREAD_ADD_FIELD (tr, "mincycle",      "%f",    msg->dev.mincycle);
    TEXTREAD_ADD_FIELD (tr, "driver_mode",   "%d",    msg->dev.driver_mode);


    TEXTREAD_ADD_CONST (tr, "MODE_NOT_SEEN", SENLCM_GPSD3_FIX_T_MODE_NOT_SEEN);
    TEXTREAD_ADD_CONST (tr, "MODE_NO_FIX", SENLCM_GPSD3_FIX_T_MODE_NO_FIX);
    TEXTREAD_ADD_CONST (tr, "MODE_2D", SENLCM_GPSD3_FIX_T_MODE_2D);
    TEXTREAD_ADD_CONST (tr, "MODE_3D", SENLCM_GPSD3_FIX_T_MODE_3D);

    TEXTREAD_ADD_CONST (tr, "FLAGS_SEEN_GPS", SENLCM_GPSD3_DEVCONFIG_T_FLAGS_SEEN_GPS);
    TEXTREAD_ADD_CONST (tr, "FLAGS_SEEN_RTCM2", SENLCM_GPSD3_DEVCONFIG_T_FLAGS_SEEN_RTCM2);
    TEXTREAD_ADD_CONST (tr, "FLAGS_SEEN_RTCM3", SENLCM_GPSD3_DEVCONFIG_T_FLAGS_SEEN_RTCM3);
    TEXTREAD_ADD_CONST (tr, "FLAGS_SEEN_AIS", SENLCM_GPSD3_DEVCONFIG_T_FLAGS_SEEN_AIS);

    textread_stop (tr);
}


void
senlcm_gpsd_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const senlcm_gpsd_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",      "%"PRId64,   msg->utime);
    TEXTREAD_ADD_FIELD (tr, "online",     "%"PRId64,   msg->online);
    TEXTREAD_ADD_FIELD (tr, "fix_utime",  "%"PRId64,   msg->fix_utime);
    TEXTREAD_ADD_FIELD (tr, "mode",       "%"PRId8,    msg->mode);
    TEXTREAD_ADD_FIELD (tr, "ept",        "%f",        msg->ept);
    TEXTREAD_ADD_FIELD (tr, "latitude",   "%.15f",       msg->latitude);
    TEXTREAD_ADD_FIELD (tr, "longitude",  "%.15f",       msg->longitude);
    TEXTREAD_ADD_FIELD (tr, "altitude",   "%f",        msg->altitude);
    TEXTREAD_ADD_FIELD (tr, "eph",        "%f",        msg->eph);
    TEXTREAD_ADD_FIELD (tr, "epv",        "%f",        msg->epv);
    TEXTREAD_ADD_FIELD (tr, "track",      "%f",        msg->track);
    TEXTREAD_ADD_FIELD (tr, "epd",        "%f",        msg->epd);
    TEXTREAD_ADD_FIELD (tr, "speed",      "%f",        msg->speed);
    TEXTREAD_ADD_FIELD (tr, "eps",        "%f",        msg->eps);
    TEXTREAD_ADD_FIELD (tr, "climb",      "%f",        msg->climb);
    TEXTREAD_ADD_FIELD (tr, "epc",        "%f",        msg->epc);
    TEXTREAD_ADD_FIELD (tr, "geoidal_separation", "%f",       msg->geoidal_separation);
    TEXTREAD_ADD_FIELD (tr, "status",             "%"PRId16,  msg->status);
    TEXTREAD_ADD_FIELD (tr, "satellites_used",    "%"PRId16,  msg->satellites_used);

    for (int i_used=0; i_used<12; i_used++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "used(:,%d)", i_used+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16, i_used < msg->satellites_used ? msg->used[i_used] : 0);
    }

    TEXTREAD_ADD_FIELD (tr, "pdop",          "%f",         msg->pdop);
    TEXTREAD_ADD_FIELD (tr, "hdop",          "%f",         msg->hdop);
    TEXTREAD_ADD_FIELD (tr, "tdop",          "%f",         msg->tdop);
    TEXTREAD_ADD_FIELD (tr, "gdop",          "%f",         msg->gdop);
    TEXTREAD_ADD_FIELD (tr, "epe",           "%f",         msg->epe);
    TEXTREAD_ADD_FIELD (tr, "satellites_visible", "%"PRId16, msg->satellites_visible);

    for (int i_PRN=0; i_PRN<12; i_PRN++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "PRN(:,%d)", i_PRN+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16, i_PRN < msg->satellites_visible ? msg->PRN[i_PRN] : 0);
    }

    for (int i_elevation=0; i_elevation<12; i_elevation++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "elevation(:,%d)", i_elevation+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16,    i_elevation < msg->satellites_visible ? msg->elevation[i_elevation] : 0);
    }

    for (int i_azimuth=0; i_azimuth<12; i_azimuth++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "azimuth(:,%d)", i_azimuth+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16,    i_azimuth < msg->satellites_visible ? msg->azimuth[i_azimuth] : 0);
    }

    for (int i_ss=0; i_ss<12; i_ss++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "ss(:,%d)", i_ss+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId16, i_ss < msg->satellites_visible ? msg->ss[i_ss] : 0);
    }

    TEXTREAD_ADD_FIELD (tr, "gps_device",    "%s",         msg->gps_device);
    TEXTREAD_ADD_FIELD (tr, "gps_id",        "%s",         msg->gps_id);
    TEXTREAD_ADD_FIELD (tr, "ndevices",      "%"PRId16,    msg->ndevices);
    for (int num_devices = 0; num_devices < msg->ndevices; num_devices++)
    {
        char buf[64];
        snprintf (buf, sizeof buf, "devicelist(:,%d)", num_devices+1);
        TEXTREAD_ADD_FIELD (tr, buf,    "%s",        msg->devicelist[num_devices]);
    }

    TEXTREAD_ADD_CONST (tr, "MODE_NOT_SEEN", SENLCM_GPSD_T_MODE_NOT_SEEN);
    TEXTREAD_ADD_CONST (tr, "MODE_NO_FIX", SENLCM_GPSD_T_MODE_NO_FIX);
    TEXTREAD_ADD_CONST (tr, "MODE_2D", SENLCM_GPSD_T_MODE_2D);
    TEXTREAD_ADD_CONST (tr, "MODE_3D", SENLCM_GPSD_T_MODE_3D);
    TEXTREAD_ADD_CONST (tr, "STATUS_NO_FIX", SENLCM_GPSD_T_STATUS_NO_FIX);
    TEXTREAD_ADD_CONST (tr, "STATUS_FIX", SENLCM_GPSD_T_STATUS_FIX);
    TEXTREAD_ADD_CONST (tr, "STATUS_DGPS_FIX", SENLCM_GPSD_T_STATUS_DGPS_FIX);

    textread_stop (tr);
}


void
senlcm_kvh_dsp3000_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                              const senlcm_kvh_dsp3000_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "mode",     "%"PRId8,  msg->mode);
    TEXTREAD_ADD_FIELD (tr, "data",     "%f",      msg->data);

    TEXTREAD_ADD_CONST (tr, "RATE_MODE",  SENLCM_KVH_DSP3000_T_RATE_MODE);
    TEXTREAD_ADD_CONST (tr, "DELTA_MODE", SENLCM_KVH_DSP3000_T_DELTA_MODE);
    TEXTREAD_ADD_CONST (tr, "ANGLE_MODE", SENLCM_KVH_DSP3000_T_ANGLE_MODE);
    textread_stop (tr);
}

void
senlcm_mocap_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const senlcm_mocap_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",           "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,1)",     "%f",  msg->xyzrph[0]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,2)",     "%f",  msg->xyzrph[1]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,3)",     "%f",  msg->xyzrph[2]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,4)",     "%f",  msg->xyzrph[3]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,5)",     "%f",  msg->xyzrph[4]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,6)",     "%f",  msg->xyzrph[5]);
    TEXTREAD_ADD_FIELD (tr, "residual",        "%f",  msg->residual);
    TEXTREAD_ADD_FIELD (tr, "valid",           "%"PRId8, msg->valid);
    textread_stop (tr);
}


void
senlcm_ms_gx1_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                         const senlcm_ms_gx1_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    // fields
    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",          "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "bitmask",        "%"PRId64,  msg->bitmask);
    TEXTREAD_ADD_FIELD (tr, "sMagField(:,1)", "%f",       msg->sMagField[0]);
    TEXTREAD_ADD_FIELD (tr, "sMagField(:,2)", "%f",       msg->sMagField[1]);
    TEXTREAD_ADD_FIELD (tr, "sMagField(:,3)", "%f",       msg->sMagField[2]);
    TEXTREAD_ADD_FIELD (tr, "sAccel(:,1)",    "%f",       msg->sAccel[0]);
    TEXTREAD_ADD_FIELD (tr, "sAccel(:,2)",    "%f",       msg->sAccel[1]);
    TEXTREAD_ADD_FIELD (tr, "sAccel(:,3)",    "%f",       msg->sAccel[2]);
    TEXTREAD_ADD_FIELD (tr, "sAngRate(:,1)",  "%f",       msg->sAngRate[0]);
    TEXTREAD_ADD_FIELD (tr, "sAngRate(:,2)",  "%f",       msg->sAngRate[1]);
    TEXTREAD_ADD_FIELD (tr, "sAngRate(:,3)",  "%f",       msg->sAngRate[2]);
    TEXTREAD_ADD_FIELD (tr, "sEuler(:,1)",    "%f",       msg->sEuler[0]);
    TEXTREAD_ADD_FIELD (tr, "sEuler(:,2)",    "%f",       msg->sEuler[1]);
    TEXTREAD_ADD_FIELD (tr, "sEuler(:,3)",    "%f",       msg->sEuler[2]);
    TEXTREAD_ADD_FIELD (tr, "sM1(:,1)",       "%f",       msg->sM[0][0]);
    TEXTREAD_ADD_FIELD (tr, "sM1(:,2)",       "%f",       msg->sM[0][1]);
    TEXTREAD_ADD_FIELD (tr, "sM1(:,3)",       "%f",       msg->sM[0][2]);
    TEXTREAD_ADD_FIELD (tr, "sM2(:,1)",       "%f",       msg->sM[1][0]);
    TEXTREAD_ADD_FIELD (tr, "sM2(:,2)",       "%f",       msg->sM[1][1]);
    TEXTREAD_ADD_FIELD (tr, "sM2(:,3)",       "%f",       msg->sM[1][2]);
    TEXTREAD_ADD_FIELD (tr, "sM3(:,1)",       "%f",       msg->sM[2][0]);
    TEXTREAD_ADD_FIELD (tr, "sM3(:,2)",       "%f",       msg->sM[2][1]);
    TEXTREAD_ADD_FIELD (tr, "sM3(:,3)",       "%f",       msg->sM[2][2]);
    TEXTREAD_ADD_FIELD (tr, "sQ(:,1)",        "%f",       msg->sQ[0]);
    TEXTREAD_ADD_FIELD (tr, "sQ(:,2)",        "%f",       msg->sQ[1]);
    TEXTREAD_ADD_FIELD (tr, "sQ(:,3)",        "%f",       msg->sQ[2]);
    TEXTREAD_ADD_FIELD (tr, "sQ(:,4)",        "%f",       msg->sQ[3]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,1)", "%f",       msg->iMagField[0]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,2)", "%f",       msg->iMagField[1]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,3)", "%f",       msg->iMagField[2]);
    TEXTREAD_ADD_FIELD (tr, "iAccel(:,1)",    "%f",       msg->iAccel[0]);
    TEXTREAD_ADD_FIELD (tr, "iAccel(:,2)",    "%f",       msg->iAccel[1]);
    TEXTREAD_ADD_FIELD (tr, "iAccel(:,3)",    "%f",       msg->iAccel[2]);
    TEXTREAD_ADD_FIELD (tr, "iAngRate(:,1)",  "%f",       msg->iAngRate[0]);
    TEXTREAD_ADD_FIELD (tr, "iAngRate(:,2)",  "%f",       msg->iAngRate[1]);
    TEXTREAD_ADD_FIELD (tr, "iAngRate(:,3)",  "%f",       msg->iAngRate[2]);
    TEXTREAD_ADD_FIELD (tr, "iEuler(:,1)",    "%f",       msg->iEuler[0]);
    TEXTREAD_ADD_FIELD (tr, "iEuler(:,2)",    "%f",       msg->iEuler[1]);
    TEXTREAD_ADD_FIELD (tr, "iEuler(:,3)",    "%f",       msg->iEuler[2]);
    TEXTREAD_ADD_FIELD (tr, "iM1(:,1)",       "%f",       msg->iM[0][0]);
    TEXTREAD_ADD_FIELD (tr, "iM1(:,2)",       "%f",       msg->iM[0][1]);
    TEXTREAD_ADD_FIELD (tr, "iM1(:,3)",       "%f",       msg->iM[0][2]);
    TEXTREAD_ADD_FIELD (tr, "iM2(:,1)",       "%f",       msg->iM[1][0]);
    TEXTREAD_ADD_FIELD (tr, "iM2(:,2)",       "%f",       msg->iM[1][1]);
    TEXTREAD_ADD_FIELD (tr, "iM2(:,3)",       "%f",       msg->iM[1][2]);
    TEXTREAD_ADD_FIELD (tr, "iM3(:,1)",       "%f",       msg->iM[2][0]);
    TEXTREAD_ADD_FIELD (tr, "iM3(:,2)",       "%f",       msg->iM[2][1]);
    TEXTREAD_ADD_FIELD (tr, "iM3(:,3)",       "%f",       msg->iM[2][2]);
    TEXTREAD_ADD_FIELD (tr, "iQ(:,1)",        "%f",       msg->iQ[0]);
    TEXTREAD_ADD_FIELD (tr, "iQ(:,2)",        "%f",       msg->iQ[1]);
    TEXTREAD_ADD_FIELD (tr, "iQ(:,3)",        "%f",       msg->iQ[2]);
    TEXTREAD_ADD_FIELD (tr, "iQ(:,4)",        "%f",       msg->iQ[3]);
    TEXTREAD_ADD_FIELD (tr, "Temperature",    "%f",       msg->Temperature);
    TEXTREAD_ADD_FIELD (tr, "TimerTicks",     "%"PRId16,  msg->TimerTicks);

    // constants
    TEXTREAD_ADD_CONST (tr, "INST_MAGFIELD", SENLCM_MS_GX1_T_INST_MAGFIELD);
    TEXTREAD_ADD_CONST (tr, "INST_ACCEL", SENLCM_MS_GX1_T_INST_ACCEL);
    TEXTREAD_ADD_CONST (tr, "INST_ANGRATE", SENLCM_MS_GX1_T_INST_ANGRATE);
    TEXTREAD_ADD_CONST (tr, "INST_EULER", SENLCM_MS_GX1_T_INST_EULER);
    TEXTREAD_ADD_CONST (tr, "INST_M", SENLCM_MS_GX1_T_INST_M);
    TEXTREAD_ADD_CONST (tr, "INST_Q", SENLCM_MS_GX1_T_INST_Q);
    TEXTREAD_ADD_CONST (tr, "STAB_MAGFIELD", SENLCM_MS_GX1_T_STAB_MAGFIELD);
    TEXTREAD_ADD_CONST (tr, "STAB_ACCEL", SENLCM_MS_GX1_T_STAB_ACCEL);
    TEXTREAD_ADD_CONST (tr, "STAB_ANGRATE", SENLCM_MS_GX1_T_STAB_ANGRATE);
    TEXTREAD_ADD_CONST (tr, "STAB_EULER", SENLCM_MS_GX1_T_STAB_EULER);
    TEXTREAD_ADD_CONST (tr, "STAB_M", SENLCM_MS_GX1_T_STAB_M);
    TEXTREAD_ADD_CONST (tr, "STAB_Q", SENLCM_MS_GX1_T_STAB_Q);
    TEXTREAD_ADD_CONST (tr, "TEMPERATURE", SENLCM_MS_GX1_T_TEMPERATURE);

    textread_stop (tr);
}

void
senlcm_ms_gx3_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                         const senlcm_ms_gx3_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    // fields
    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",          "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "bitmask",        "%"PRId64,  msg->bitmask);
    TEXTREAD_ADD_FIELD (tr, "sMagField(:,1)", "%f",       msg->sMagField[0]);
    TEXTREAD_ADD_FIELD (tr, "sMagField(:,2)", "%f",       msg->sMagField[1]);
    TEXTREAD_ADD_FIELD (tr, "sMagField(:,3)", "%f",       msg->sMagField[2]);
    TEXTREAD_ADD_FIELD (tr, "sAccel(:,1)",    "%f",       msg->sAccel[0]);
    TEXTREAD_ADD_FIELD (tr, "sAccel(:,2)",    "%f",       msg->sAccel[1]);
    TEXTREAD_ADD_FIELD (tr, "sAccel(:,3)",    "%f",       msg->sAccel[2]);
    TEXTREAD_ADD_FIELD (tr, "sAngRate(:,1)",  "%f",       msg->sAngRate[0]);
    TEXTREAD_ADD_FIELD (tr, "sAngRate(:,2)",  "%f",       msg->sAngRate[1]);
    TEXTREAD_ADD_FIELD (tr, "sAngRate(:,3)",  "%f",       msg->sAngRate[2]);
    TEXTREAD_ADD_FIELD (tr, "sEuler(:,1)",    "%f",       msg->sEuler[0]);
    TEXTREAD_ADD_FIELD (tr, "sEuler(:,2)",    "%f",       msg->sEuler[1]);
    TEXTREAD_ADD_FIELD (tr, "sEuler(:,3)",    "%f",       msg->sEuler[2]);
    TEXTREAD_ADD_FIELD (tr, "sM1(:,1)",       "%f",       msg->sM[0][0]);
    TEXTREAD_ADD_FIELD (tr, "sM1(:,2)",       "%f",       msg->sM[0][1]);
    TEXTREAD_ADD_FIELD (tr, "sM1(:,3)",       "%f",       msg->sM[0][2]);
    TEXTREAD_ADD_FIELD (tr, "sM2(:,1)",       "%f",       msg->sM[1][0]);
    TEXTREAD_ADD_FIELD (tr, "sM2(:,2)",       "%f",       msg->sM[1][1]);
    TEXTREAD_ADD_FIELD (tr, "sM2(:,3)",       "%f",       msg->sM[1][2]);
    TEXTREAD_ADD_FIELD (tr, "sM3(:,1)",       "%f",       msg->sM[2][0]);
    TEXTREAD_ADD_FIELD (tr, "sM3(:,2)",       "%f",       msg->sM[2][1]);
    TEXTREAD_ADD_FIELD (tr, "sM3(:,3)",       "%f",       msg->sM[2][2]);
    TEXTREAD_ADD_FIELD (tr, "sQ(:,1)",        "%f",       msg->sQ[0]);
    TEXTREAD_ADD_FIELD (tr, "sQ(:,2)",        "%f",       msg->sQ[1]);
    TEXTREAD_ADD_FIELD (tr, "sQ(:,3)",        "%f",       msg->sQ[2]);
    TEXTREAD_ADD_FIELD (tr, "sQ(:,4)",        "%f",       msg->sQ[3]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,1)", "%f",       msg->iMagField[0]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,2)", "%f",       msg->iMagField[1]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,3)", "%f",       msg->iMagField[2]);
    TEXTREAD_ADD_FIELD (tr, "iAccel(:,1)",    "%f",       msg->iAccel[0]);
    TEXTREAD_ADD_FIELD (tr, "iAccel(:,2)",    "%f",       msg->iAccel[1]);
    TEXTREAD_ADD_FIELD (tr, "iAccel(:,3)",    "%f",       msg->iAccel[2]);
    TEXTREAD_ADD_FIELD (tr, "iAngRate(:,1)",  "%f",       msg->iAngRate[0]);
    TEXTREAD_ADD_FIELD (tr, "iAngRate(:,2)",  "%f",       msg->iAngRate[1]);
    TEXTREAD_ADD_FIELD (tr, "iAngRate(:,3)",  "%f",       msg->iAngRate[2]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,1)", "%f",       msg->iMagField[0]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,2)", "%f",       msg->iMagField[1]);
    TEXTREAD_ADD_FIELD (tr, "iMagField(:,3)", "%f",       msg->iMagField[2]);
    TEXTREAD_ADD_FIELD (tr, "Temperature",    "%f",       msg->Temperature);
    TEXTREAD_ADD_FIELD (tr, "TimerTicks",     "%"PRId16,  msg->TimerTicks);

    // constants
    TEXTREAD_ADD_CONST (tr, "INST_MAGFIELD", SENLCM_MS_GX3_T_INST_MAGFIELD);
    TEXTREAD_ADD_CONST (tr, "INST_ACCEL", SENLCM_MS_GX3_T_INST_ACCEL);
    TEXTREAD_ADD_CONST (tr, "INST_ANGRATE", SENLCM_MS_GX3_T_INST_ANGRATE);
    TEXTREAD_ADD_CONST (tr, "STAB_MAGFIELD", SENLCM_MS_GX3_T_STAB_MAGFIELD);
    TEXTREAD_ADD_CONST (tr, "STAB_ACCEL", SENLCM_MS_GX3_T_STAB_ACCEL);
    TEXTREAD_ADD_CONST (tr, "STAB_ANGRATE", SENLCM_MS_GX3_T_STAB_ANGRATE);
    TEXTREAD_ADD_CONST (tr, "STAB_EULER", SENLCM_MS_GX3_T_STAB_EULER);
    TEXTREAD_ADD_CONST (tr, "STAB_M", SENLCM_MS_GX3_T_STAB_M);
    TEXTREAD_ADD_CONST (tr, "STAB_Q", SENLCM_MS_GX3_T_STAB_Q);
    TEXTREAD_ADD_CONST (tr, "TEMPERATURE", SENLCM_MS_GX3_T_TEMPERATURE);

    textread_stop (tr);
}


void
senlcm_ms_gx3_25_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                            const senlcm_ms_gx3_25_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    // fields
    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",          "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "bitmask",        "%"PRId64,  msg->bitmask);
    TEXTREAD_ADD_FIELD (tr, "MagField(:,1)",  "%f",       msg->MagField[0]);
    TEXTREAD_ADD_FIELD (tr, "MagField(:,2)",  "%f",       msg->MagField[1]);
    TEXTREAD_ADD_FIELD (tr, "MagField(:,3)",  "%f",       msg->MagField[2]);
    TEXTREAD_ADD_FIELD (tr, "Accel(:,1)",     "%f",       msg->Accel[0]);
    TEXTREAD_ADD_FIELD (tr, "Accel(:,2)",     "%f",       msg->Accel[1]);
    TEXTREAD_ADD_FIELD (tr, "Accel(:,3)",     "%f",       msg->Accel[2]);
    TEXTREAD_ADD_FIELD (tr, "AngRate(:,1)",   "%f",       msg->AngRate[0]);
    TEXTREAD_ADD_FIELD (tr, "AngRate(:,2)",   "%f",       msg->AngRate[1]);
    TEXTREAD_ADD_FIELD (tr, "AngRate(:,3)",   "%f",       msg->AngRate[2]);
    TEXTREAD_ADD_FIELD (tr, "Euler(:,1)",     "%f",       msg->Euler[0]);
    TEXTREAD_ADD_FIELD (tr, "Euler(:,2)",     "%f",       msg->Euler[1]);
    TEXTREAD_ADD_FIELD (tr, "Euler(:,3)",     "%f",       msg->Euler[2]);
    TEXTREAD_ADD_FIELD (tr, "Temperature",    "%f",       msg->Temperature);
    TEXTREAD_ADD_FIELD (tr, "TimerTicks",     "%"PRId16,  msg->TimerTicks);

    // constants
    TEXTREAD_ADD_CONST (tr, "INST_MAGFIELD",    SENLCM_MS_GX3_25_T_INST_MAGFIELD);
    TEXTREAD_ADD_CONST (tr, "INST_ACCEL",       SENLCM_MS_GX3_25_T_INST_ACCEL);
    TEXTREAD_ADD_CONST (tr, "INST_ANGRATE",     SENLCM_MS_GX3_25_T_INST_ANGRATE);
    TEXTREAD_ADD_CONST (tr, "EULER",            SENLCM_MS_GX3_25_T_EULER);
    TEXTREAD_ADD_CONST (tr, "STAB_MAGFIELD",    SENLCM_MS_GX3_25_T_STAB_MAGFIELD);
    TEXTREAD_ADD_CONST (tr, "STAB_ACCEL",       SENLCM_MS_GX3_25_T_STAB_ACCEL);
    TEXTREAD_ADD_CONST (tr, "STAB_ANGRATE",     SENLCM_MS_GX3_25_T_STAB_ANGRATE);
    TEXTREAD_ADD_CONST (tr, "TEMPERATURE",      SENLCM_MS_GX3_25_T_TEMPERATURE);

    textread_stop (tr);
}

void
senlcm_os_compass_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const senlcm_os_compass_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);
    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",     "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "rph(:,1)",  "%f",      msg->rph[0]);
    TEXTREAD_ADD_FIELD (tr, "rph(:,2)",  "%f",      msg->rph[1]);
    TEXTREAD_ADD_FIELD (tr, "rph(:,3)",  "%f",      msg->rph[2]);
    TEXTREAD_ADD_FIELD (tr, "T",         "%f",      msg->T);
    TEXTREAD_ADD_FIELD (tr, "depth",     "%f",      msg->depth);
    TEXTREAD_ADD_FIELD (tr, "p_volts",   "%f",      msg->p_volts);
    TEXTREAD_ADD_FIELD (tr, "p_meas",    "%f",      msg->p_meas);
    TEXTREAD_ADD_FIELD (tr, "p_gage",    "%f",      msg->p_gage);
    TEXTREAD_ADD_FIELD (tr, "p_o",       "%f",      msg->p_o);
    TEXTREAD_ADD_FIELD (tr, "Mxyz(:,1)", "%f",      msg->Mxyz[0]);
    TEXTREAD_ADD_FIELD (tr, "Mxyz(:,2)", "%f",      msg->Mxyz[1]);
    TEXTREAD_ADD_FIELD (tr, "Mxyz(:,3)", "%f",      msg->Mxyz[2]);
    TEXTREAD_ADD_FIELD (tr, "Gxyz(:,1)", "%f",      msg->Gxyz[0]);
    TEXTREAD_ADD_FIELD (tr, "Gxyz(:,2)", "%f",      msg->Gxyz[1]);
    TEXTREAD_ADD_FIELD (tr, "Gxyz(:,3)", "%f",      msg->Gxyz[2]);
    textread_stop (tr);
}



void
senlcm_ppsboard_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const senlcm_ppsboard_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",        "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "ntp_time",     "%s",      msg->ntp_time)
    TEXTREAD_ADD_FIELD (tr, "ntp_status",   "%"PRId8,  msg->ntp_status);
    TEXTREAD_ADD_FIELD (tr, "src_type",     "%s",      msg->src_type);
    TEXTREAD_ADD_FIELD (tr, "src_pps",      "%"PRId8,  msg->src_pps);
    TEXTREAD_ADD_FIELD (tr, "sync_mode",    "%"PRId16, msg->sync_mode);
    TEXTREAD_ADD_FIELD (tr, "sync_num",     "%"PRId16, msg->sync_num);
    TEXTREAD_ADD_FIELD (tr, "sync_date",    "%s",      msg->sync_date);
    TEXTREAD_ADD_FIELD (tr, "sync_time",    "%s",      msg->sync_time);
    TEXTREAD_ADD_FIELD (tr, "offset",       "%"PRId16, msg->offset_counts);
    TEXTREAD_ADD_FIELD (tr, "offset_usecs", "%f",      msg->offset_usecs);
    TEXTREAD_ADD_FIELD (tr, "temperature",  "%f",      msg->temperature);
    textread_stop (tr);
}


void
senlcm_prosilica_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                            const senlcm_prosilica_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime", "%"PRId64, msg->utime);
    for (int n = 0; n < msg->n_attributes; ++n)
    {
        const char *fieldd = NULL, *fields = NULL;
        if (0 == strcmp (msg->PvAttributes[n].label, "ExposureValue"))
            fieldd = "ExposureValue";
        else if (0 == strcmp (msg->PvAttributes[n].label, "FrameRate"))
            fieldd = "FrameRate";
        else if (0 == strcmp (msg->PvAttributes[n].label, "GainValue"))
            fieldd = "GainValue";

        if (fieldd)
            TEXTREAD_ADD_STRFIELD (tr, fieldd, "%d", msg->PvAttributes[n].value);
        if (fields)
            TEXTREAD_ADD_STRFIELD (tr, fields, "%s", msg->PvAttributes[n].value);
    }
    textread_stop (tr);
}


void
senlcm_rdi_pd4_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_rdi_pd4_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",             "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "system_config",     "%"PRIu8,  msg->system_config);
    TEXTREAD_ADD_FIELD (tr, "btv(:,1)",          "%f",      msg->btv[0]);
    TEXTREAD_ADD_FIELD (tr, "btv(:,2)",          "%f",      msg->btv[1]);
    TEXTREAD_ADD_FIELD (tr, "btv(:,3)",          "%f",      msg->btv[2]);
    TEXTREAD_ADD_FIELD (tr, "btv(:,4)",          "%f",      msg->btv[3]);
    TEXTREAD_ADD_FIELD (tr, "btv_status",        "%"PRIu8,  msg->btv_status);
    TEXTREAD_ADD_FIELD (tr, "wtv(:,1)",          "%f",      msg->wtv[0]);
    TEXTREAD_ADD_FIELD (tr, "wtv(:,2)",          "%f",      msg->wtv[1]);
    TEXTREAD_ADD_FIELD (tr, "wtv(:,3)",          "%f",      msg->wtv[2]);
    TEXTREAD_ADD_FIELD (tr, "wtv(:,4)",          "%f",      msg->wtv[3]);
    TEXTREAD_ADD_FIELD (tr, "wtv_status",        "%"PRIu8,  msg->wtv_status);
    TEXTREAD_ADD_FIELD (tr, "wtv_layer_start",   "%f",      msg->wtv_layer_start);
    TEXTREAD_ADD_FIELD (tr, "wtv_layer_end",     "%f",      msg->wtv_layer_end);
    TEXTREAD_ADD_FIELD (tr, "range(:,1)",        "%f",      msg->range[0]);
    TEXTREAD_ADD_FIELD (tr, "range(:,2)",        "%f",      msg->range[1]);
    TEXTREAD_ADD_FIELD (tr, "range(:,3)",        "%f",      msg->range[2]);
    TEXTREAD_ADD_FIELD (tr, "range(:,4)",        "%f",      msg->range[3]);
    TEXTREAD_ADD_FIELD (tr, "altitude",          "%f",      msg->altitude);
    TEXTREAD_ADD_FIELD (tr, "tofp_hour",         "%"PRIu8,  msg->tofp_hour);
    TEXTREAD_ADD_FIELD (tr, "tofp_minute",       "%"PRIu8,  msg->tofp_minute);
    TEXTREAD_ADD_FIELD (tr, "tofp_second",       "%"PRIu8,  msg->tofp_second);
    TEXTREAD_ADD_FIELD (tr, "tofp_hundredth",    "%"PRIu8,  msg->tofp_hundredth);
    TEXTREAD_ADD_FIELD (tr, "builtin_test(:,1)", "%"PRIu8,  msg->builtin_test[0]);
    TEXTREAD_ADD_FIELD (tr, "builtin_test(:,2)", "%"PRIu8,  msg->builtin_test[1]);
    TEXTREAD_ADD_FIELD (tr, "speed_of_sound",    "%f",      msg->speed_of_sound);
    TEXTREAD_ADD_FIELD (tr, "xducer_head_temp",  "%f",      msg->xducer_head_temp);

    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_MASK",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_MASK);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_BEAM",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_BEAM);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_INST",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_INST);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_SHIP",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_SHIP);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_EARTH", SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_EARTH);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_TILT_MASK",   SENLCM_RDI_PD4_T_SYSTEM_CONFIG_TILT_MASK);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_TILT_USED",   SENLCM_RDI_PD4_T_SYSTEM_CONFIG_TILT_USED);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_3BEAM_MASK",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_3BEAM_MASK);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_3BEAM_USED",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_3BEAM_USED);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_KHZ_MASK",    SENLCM_RDI_PD4_T_SYSTEM_CONFIG_KHZ_MASK);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_KHZ_300",     SENLCM_RDI_PD4_T_SYSTEM_CONFIG_KHZ_300);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_KHZ_600",     SENLCM_RDI_PD4_T_SYSTEM_CONFIG_KHZ_600);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_KHZ_1200",    SENLCM_RDI_PD4_T_SYSTEM_CONFIG_KHZ_1200);

    TEXTREAD_ADD_CONST (tr, "BTV_SENTINAL", SENLCM_RDI_PD4_T_BTV_SENTINAL);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM4_LOW_ECHO_AMPLITUDE", SENLCM_RDI_PD4_T_BTV_STATUS_BEAM4_LOW_ECHO_AMPLITUDE);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM4_LOW_CORRELATION",    SENLCM_RDI_PD4_T_BTV_STATUS_BEAM4_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM3_LOW_ECHO_AMPLITUDE", SENLCM_RDI_PD4_T_BTV_STATUS_BEAM3_LOW_ECHO_AMPLITUDE);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM3_LOW_CORRELATION",    SENLCM_RDI_PD4_T_BTV_STATUS_BEAM3_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM2_LOW_ECHO_AMPLITUDE", SENLCM_RDI_PD4_T_BTV_STATUS_BEAM2_LOW_ECHO_AMPLITUDE);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM2_LOW_CORRELATION",    SENLCM_RDI_PD4_T_BTV_STATUS_BEAM2_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM1_LOW_ECHO_AMPLITUDE", SENLCM_RDI_PD4_T_BTV_STATUS_BEAM1_LOW_ECHO_AMPLITUDE);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM1_LOW_CORRELATION",    SENLCM_RDI_PD4_T_BTV_STATUS_BEAM1_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_OK", SENLCM_RDI_PD4_T_BTV_STATUS_OK);

    TEXTREAD_ADD_CONST (tr, "WTV_SENTINAL", SENLCM_RDI_PD4_T_WTV_SENTINAL);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_BEAM4_LOW_CORRELATION", SENLCM_RDI_PD4_T_WTV_STATUS_BEAM4_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_BEAM3_LOW_CORRELATION", SENLCM_RDI_PD4_T_WTV_STATUS_BEAM3_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_BEAM2_LOW_CORRELATION", SENLCM_RDI_PD4_T_WTV_STATUS_BEAM2_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_BEAM1_LOW_CORRELATION", SENLCM_RDI_PD4_T_WTV_STATUS_BEAM1_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_OK", SENLCM_RDI_PD4_T_WTV_STATUS_OK);

    TEXTREAD_ADD_CONST (tr, "RANGE_SENTINAL", SENLCM_RDI_PD4_T_RANGE_SENTINAL);
    TEXTREAD_ADD_CONST (tr, "ALTITUDE_SENTINAL", SENLCM_RDI_PD4_T_ALTITUDE_SENTINAL);
    TEXTREAD_ADD_CONST (tr, "BUILTIN_TEST_OK", SENLCM_RDI_PD4_T_BUILTIN_TEST_OK);

    textread_stop (tr);
}

void
senlcm_tritech_es_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const senlcm_tritech_es_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",       "%"PRId64,msg->utime);
    TEXTREAD_ADD_FIELD (tr, "range",       "%f",     msg->range);
    textread_stop (tr);

}

void
senlcm_uvc_osi_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_uvc_osi_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",            "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "yaw_top",          "%d",       msg->yaw_top);
    TEXTREAD_ADD_FIELD (tr, "yaw_bot",          "%d",       msg->yaw_bot);
    TEXTREAD_ADD_FIELD (tr, "pitch_left",       "%d",       msg->pitch_left);
    TEXTREAD_ADD_FIELD (tr, "pitch_right",      "%d",       msg->pitch_right);
    TEXTREAD_ADD_FIELD (tr, "motor",            "%d",       msg->motor);
    TEXTREAD_ADD_FIELD (tr, "mode",             "%d",       msg->mode);
    TEXTREAD_ADD_FIELD (tr, "nextwp",           "%d",       msg->nextwp);
    TEXTREAD_ADD_FIELD (tr, "latitude",         "%f",       msg->latitude);
    TEXTREAD_ADD_FIELD (tr, "longitude",        "%f",       msg->longitude);
    TEXTREAD_ADD_FIELD (tr, "speed",            "%f",       msg->speed);
    TEXTREAD_ADD_FIELD (tr, "dist_to_nextwp",   "%f",       msg->dist_to_nextwp);
    TEXTREAD_ADD_FIELD (tr, "error",            "%d",       msg->error);
    TEXTREAD_ADD_FIELD (tr, "altimeter",        "%f",       msg->altimeter);
    TEXTREAD_ADD_FIELD (tr, "park_time",        "%d",       msg->park_time);
    TEXTREAD_ADD_FIELD (tr, "magnetic_dec",     "%f",       msg->magnetic_dec);
    textread_stop (tr);
}

void
senlcm_uvc_rphtd_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                            const senlcm_uvc_rphtd_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",            "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "roll",          "%f",       msg->rph[0]);
    TEXTREAD_ADD_FIELD (tr, "pitch",          "%f",       msg->rph[1]);
    TEXTREAD_ADD_FIELD (tr, "heading",          "%f",       msg->rph[2]);
    TEXTREAD_ADD_FIELD (tr, "temperature",          "%f",       msg->T);
    TEXTREAD_ADD_FIELD (tr, "depth",       "%f",       msg->depth);
    textread_stop (tr);
}

void
senlcm_uvc_dvl_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_uvc_dvl_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",            "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "vx",          "%f",       msg->vx);
    TEXTREAD_ADD_FIELD (tr, "vy",          "%f",       msg->vy);
	TEXTREAD_ADD_FIELD (tr, "vz",       	"%f",       msg->vz);
	TEXTREAD_ADD_FIELD (tr, "xdist",          "%f",       msg->xdist);
	TEXTREAD_ADD_FIELD (tr, "ydist",          "%f",       msg->ydist);
	TEXTREAD_ADD_FIELD (tr, "dfs",          "%f",       msg->dfs);
	TEXTREAD_ADD_FIELD (tr, "alt",          "%f",       msg->alt);
    textread_stop (tr);
}



////////////////////////////////////////////////////////////////////////////////
// Additional handlers added by ACFR
////////////////////////////////////////////////////////////////////////////////

void
senlcm_seabird_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_seabird_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "conductivity",    "%f", msg->conductivity);
    TEXTREAD_ADD_FIELD (tr, "temperature",    "%f", msg->temperature);
    TEXTREAD_ADD_FIELD (tr, "pressure",    "%f", msg->pressure);
    TEXTREAD_ADD_FIELD (tr, "salinity",    "%f", msg->salinity);
    TEXTREAD_ADD_FIELD (tr, "speed_of_sound",    "%f", msg->speed_of_sound);
    TEXTREAD_ADD_FIELD (tr, "pump",    "%d", msg->pump);

    textread_stop (tr);
}

// FIXME: What else do we need to extract for the processing scripts
void
senlcm_rdi_pd0_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_rdi_pd0_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "num_velocities",    "%d", msg->num_velocities);

    for (int num_vels = 0; num_vels < msg->num_velocities; num_vels++)
    {
        char buf[64];
        snprintf (buf, sizeof buf, "velocity(:,%d)", num_vels+1);
        TEXTREAD_ADD_FIELD (tr, buf,    "%f",        msg->velocity[num_vels]);
    }

    textread_stop (tr);
}

void
senlcm_os_power_system_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                  const senlcm_os_power_system_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);

    TEXTREAD_ADD_FIELD (tr, "power",    "%f", msg->power);
    TEXTREAD_ADD_FIELD (tr, "avg_charge_p",    "%d", msg->avg_charge_p);
    TEXTREAD_ADD_FIELD (tr, "capacity",    "%f", msg->capacity);
    TEXTREAD_ADD_FIELD (tr, "capacity_full",    "%f", msg->capacity_full);
    TEXTREAD_ADD_FIELD (tr, "minutes_tef",    "%d", msg->minutes_tef);

    TEXTREAD_ADD_FIELD (tr, "num_controllers",    "%d", msg->num_controllers);

    char buf[64];
    for(int i=0; i<msg->num_controllers; i++)
    {
        snprintf (buf, sizeof buf, "num_batteries(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, buf,    "%d", msg->controller[i].num_batteries);
    }

    int battery_number = 1;
    for(int i=0; i<msg->num_controllers; i++)
    {
        for(int j=0; j<msg->controller[i].num_batteries; j++)
        {
            // for each battery read the temp, voltage and current
            snprintf (buf, sizeof buf, "voltage(:,%d)", battery_number);
            TEXTREAD_ADD_FIELD (tr, buf,    "%f", msg->controller[i].battery[j].voltage);

            snprintf (buf, sizeof buf, "current(:,%d)", battery_number);
            TEXTREAD_ADD_FIELD (tr, buf,    "%f", msg->controller[i].battery[j].current);

            snprintf (buf, sizeof buf, "temperature(:,%d)", battery_number);
            TEXTREAD_ADD_FIELD (tr, buf,    "%f", msg->controller[i].battery[j].temperature);

            battery_number++;
        }
    }
    textread_stop (tr);
}

void
senlcm_ecopuck_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_ecopuck_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "chlorophyll","%f",      msg->chlorophyll);
    TEXTREAD_ADD_FIELD (tr, "turbidity","%f",      msg->turbidity);
    TEXTREAD_ADD_FIELD (tr, "temperature","%f",      msg->temperature);
    textread_stop (tr);
}

void
senlcm_oas_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                      const senlcm_oas_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "profRange","%f",      msg->profRange);
    TEXTREAD_ADD_FIELD (tr, "pseudoAlt","%f",      msg->pseudoAlt);
    TEXTREAD_ADD_FIELD (tr, "pseudoFwdDistance","%f",      msg->pseudoFwdDistance);
    textread_stop (tr);
}

void
senlcm_parosci_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_parosci_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "raw","%f",      msg->raw);
    TEXTREAD_ADD_FIELD (tr, "depth","%f",      msg->depth);
    textread_stop (tr);
}

void
senlcm_lq_modem_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const senlcm_lq_modem_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "time","%f",      msg->time);
    TEXTREAD_ADD_FIELD (tr, "lat","%f",      msg->lat);
    TEXTREAD_ADD_FIELD (tr, "lon","%f",      msg->lon);
    TEXTREAD_ADD_FIELD (tr, "heading","%f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "roll","%f",      msg->roll);
    TEXTREAD_ADD_FIELD (tr, "pitch","%f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "bearing","%f",      msg->bearing);
    TEXTREAD_ADD_FIELD (tr, "slantRange","%f",      msg->slantRange);
    TEXTREAD_ADD_FIELD (tr, "ping","%"PRId32,      msg->ping);
    TEXTREAD_ADD_FIELD (tr, "messageType","%"PRId8,      msg->messageType);

    textread_stop (tr);
}


void
senlcm_ysi_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                      const senlcm_ysi_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "temperature","%f",      msg->temperature);
    TEXTREAD_ADD_FIELD (tr, "depth","%f",      msg->depth);
    TEXTREAD_ADD_FIELD (tr, "turbidity","%f",      msg->turbidity);
    TEXTREAD_ADD_FIELD (tr, "chlorophyl","%f",      msg->chlorophyl);
    TEXTREAD_ADD_FIELD (tr, "conductivity","%f",      msg->conductivity);
    TEXTREAD_ADD_FIELD (tr, "oxygen","%f",      msg->oxygen);
    TEXTREAD_ADD_FIELD (tr, "battery","%f",      msg->battery);
    TEXTREAD_ADD_FIELD (tr, "salinity","%f",      msg->salinity);

    textread_stop (tr);
}

void
senlcm_micron_ping_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                              const senlcm_micron_ping_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "count","%d",      msg->count);
    TEXTREAD_ADD_FIELD (tr, "range","%f",      msg->range);
    TEXTREAD_ADD_FIELD (tr, "gain","%f",      msg->gain);
    TEXTREAD_ADD_FIELD (tr, "slope","%f",      msg->slope);
    TEXTREAD_ADD_FIELD (tr, "AD_interval","%f",      msg->AD_interval);
    TEXTREAD_ADD_FIELD (tr, "left_limit","%f",      msg->left_limit);
    TEXTREAD_ADD_FIELD (tr, "right_limit","%f",      msg->right_limit);
    TEXTREAD_ADD_FIELD (tr, "angle","%f",      msg->angle);
    TEXTREAD_ADD_FIELD (tr, "range_resolution","%f",      msg->range_resolution);

    TEXTREAD_ADD_FIELD (tr, "num_bins","%d",      msg->num_bins);

    char buf[64];
    for(int i=0; i<msg->num_bins; i++)
    {
        snprintf (buf, sizeof buf, "bins(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, buf,    "%d", msg->bins[i]);
    }

    textread_stop (tr);
}

void
senlcm_rdi_pd5_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_rdi_pd5_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",             "%"PRId64, msg->utime);

    TEXTREAD_ADD_FIELD (tr, "system_config",     "%"PRIu8,  msg->pd4.system_config);
    TEXTREAD_ADD_FIELD (tr, "btv(:,1)",          "%f",      msg->pd4.btv[0]);
    TEXTREAD_ADD_FIELD (tr, "btv(:,2)",          "%f",      msg->pd4.btv[1]);
    TEXTREAD_ADD_FIELD (tr, "btv(:,3)",          "%f",      msg->pd4.btv[2]);
    TEXTREAD_ADD_FIELD (tr, "btv(:,4)",          "%f",      msg->pd4.btv[3]);
    TEXTREAD_ADD_FIELD (tr, "btv_status",        "%"PRIu8,  msg->pd4.wtv_status);
    TEXTREAD_ADD_FIELD (tr, "wtv(:,1)",          "%f",      msg->pd4.wtv[0]);
    TEXTREAD_ADD_FIELD (tr, "wtv(:,2)",          "%f",      msg->pd4.wtv[1]);
    TEXTREAD_ADD_FIELD (tr, "wtv(:,3)",          "%f",      msg->pd4.wtv[2]);
    TEXTREAD_ADD_FIELD (tr, "wtv(:,4)",          "%f",      msg->pd4.wtv[3]);
    TEXTREAD_ADD_FIELD (tr, "wtv_status",        "%"PRIu8,  msg->pd4.wtv_status);
    TEXTREAD_ADD_FIELD (tr, "wtv_layer_start",   "%f",      msg->pd4.wtv_layer_start);
    TEXTREAD_ADD_FIELD (tr, "wtv_layer_end",     "%f",      msg->pd4.wtv_layer_end);
    TEXTREAD_ADD_FIELD (tr, "range(:,1)",        "%f",      msg->pd4.range[0]);
    TEXTREAD_ADD_FIELD (tr, "range(:,2)",        "%f",      msg->pd4.range[1]);
    TEXTREAD_ADD_FIELD (tr, "range(:,3)",        "%f",      msg->pd4.range[2]);
    TEXTREAD_ADD_FIELD (tr, "range(:,4)",        "%f",      msg->pd4.range[3]);
    TEXTREAD_ADD_FIELD (tr, "altitude",          "%f",      msg->pd4.altitude);
    TEXTREAD_ADD_FIELD (tr, "tofp_hour",         "%"PRIu8,  msg->pd4.tofp_hour);
    TEXTREAD_ADD_FIELD (tr, "tofp_minute",       "%"PRIu8,  msg->pd4.tofp_minute);
    TEXTREAD_ADD_FIELD (tr, "tofp_second",       "%"PRIu8,  msg->pd4.tofp_second);
    TEXTREAD_ADD_FIELD (tr, "tofp_hundredth",    "%"PRIu8,  msg->pd4.tofp_hundredth);
    TEXTREAD_ADD_FIELD (tr, "builtin_test(:,1)", "%"PRIu8,  msg->pd4.builtin_test[0]);
    TEXTREAD_ADD_FIELD (tr, "builtin_test(:,2)", "%"PRIu8,  msg->pd4.builtin_test[1]);
    TEXTREAD_ADD_FIELD (tr, "speed_of_sound",    "%f",      msg->pd4.speed_of_sound);
    TEXTREAD_ADD_FIELD (tr, "xducer_head_temp",  "%f",      msg->pd4.xducer_head_temp);

    TEXTREAD_ADD_FIELD (tr, "salinity",          "%f",      msg->salinity);
    TEXTREAD_ADD_FIELD (tr, "depth",          "%f",      msg->depth);
    TEXTREAD_ADD_FIELD (tr, "pitch",          "%f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "roll",          "%f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "dmg_btv(:,1)",          "%f",      msg->dmg_btv[0]);
    TEXTREAD_ADD_FIELD (tr, "dmg_btv(:,2)",          "%f",      msg->dmg_btv[1]);
    TEXTREAD_ADD_FIELD (tr, "dmg_btv(:,3)",          "%f",      msg->dmg_btv[2]);
    TEXTREAD_ADD_FIELD (tr, "dmg_btv(:,4)",          "%f",      msg->dmg_btv[3]);
    TEXTREAD_ADD_FIELD (tr, "dmg_wtv(:,1)",          "%f",      msg->dmg_wtv[0]);
    TEXTREAD_ADD_FIELD (tr, "dmg_wtv(:,2)",          "%f",      msg->dmg_wtv[1]);
    TEXTREAD_ADD_FIELD (tr, "dmg_wtv(:,3)",          "%f",      msg->dmg_wtv[2]);
    TEXTREAD_ADD_FIELD (tr, "dmg_wtv(:,4)",          "%f",      msg->dmg_wtv[3]);

    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_MASK",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_MASK);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_BEAM",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_BEAM);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_INST",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_INST);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_SHIP",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_SHIP);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_COORD_EARTH", SENLCM_RDI_PD4_T_SYSTEM_CONFIG_COORD_EARTH);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_TILT_MASK",   SENLCM_RDI_PD4_T_SYSTEM_CONFIG_TILT_MASK);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_TILT_USED",   SENLCM_RDI_PD4_T_SYSTEM_CONFIG_TILT_USED);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_3BEAM_MASK",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_3BEAM_MASK);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_3BEAM_USED",  SENLCM_RDI_PD4_T_SYSTEM_CONFIG_3BEAM_USED);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_KHZ_MASK",    SENLCM_RDI_PD4_T_SYSTEM_CONFIG_KHZ_MASK);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_KHZ_300",     SENLCM_RDI_PD4_T_SYSTEM_CONFIG_KHZ_300);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_KHZ_600",     SENLCM_RDI_PD4_T_SYSTEM_CONFIG_KHZ_600);
    TEXTREAD_ADD_CONST (tr, "SYSTEM_CONFIG_KHZ_1200",    SENLCM_RDI_PD4_T_SYSTEM_CONFIG_KHZ_1200);

    TEXTREAD_ADD_CONST (tr, "BTV_SENTINAL", SENLCM_RDI_PD4_T_BTV_SENTINAL);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM4_LOW_ECHO_AMPLITUDE", SENLCM_RDI_PD4_T_BTV_STATUS_BEAM4_LOW_ECHO_AMPLITUDE);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM4_LOW_CORRELATION",    SENLCM_RDI_PD4_T_BTV_STATUS_BEAM4_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM3_LOW_ECHO_AMPLITUDE", SENLCM_RDI_PD4_T_BTV_STATUS_BEAM3_LOW_ECHO_AMPLITUDE);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM3_LOW_CORRELATION",    SENLCM_RDI_PD4_T_BTV_STATUS_BEAM3_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM2_LOW_ECHO_AMPLITUDE", SENLCM_RDI_PD4_T_BTV_STATUS_BEAM2_LOW_ECHO_AMPLITUDE);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM2_LOW_CORRELATION",    SENLCM_RDI_PD4_T_BTV_STATUS_BEAM2_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM1_LOW_ECHO_AMPLITUDE", SENLCM_RDI_PD4_T_BTV_STATUS_BEAM1_LOW_ECHO_AMPLITUDE);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_BEAM1_LOW_CORRELATION",    SENLCM_RDI_PD4_T_BTV_STATUS_BEAM1_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "BTV_STATUS_OK", SENLCM_RDI_PD4_T_BTV_STATUS_OK);

    TEXTREAD_ADD_CONST (tr, "WTV_SENTINAL", SENLCM_RDI_PD4_T_WTV_SENTINAL);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_BEAM4_LOW_CORRELATION", SENLCM_RDI_PD4_T_WTV_STATUS_BEAM4_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_BEAM3_LOW_CORRELATION", SENLCM_RDI_PD4_T_WTV_STATUS_BEAM3_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_BEAM2_LOW_CORRELATION", SENLCM_RDI_PD4_T_WTV_STATUS_BEAM2_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_BEAM1_LOW_CORRELATION", SENLCM_RDI_PD4_T_WTV_STATUS_BEAM1_LOW_CORRELATION);
    TEXTREAD_ADD_CONST (tr, "WTV_STATUS_OK", SENLCM_RDI_PD4_T_WTV_STATUS_OK);

    TEXTREAD_ADD_CONST (tr, "RANGE_SENTINAL", SENLCM_RDI_PD4_T_RANGE_SENTINAL);
    TEXTREAD_ADD_CONST (tr, "ALTITUDE_SENTINAL", SENLCM_RDI_PD4_T_ALTITUDE_SENTINAL);
    TEXTREAD_ADD_CONST (tr, "BUILTIN_TEST_OK", SENLCM_RDI_PD4_T_BUILTIN_TEST_OK);

    textread_stop (tr);
}

void
senlcm_tcm_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                      const senlcm_tcm_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "heading","%f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "roll","%f",      msg->roll);
    TEXTREAD_ADD_FIELD (tr, "pitch","%f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "temperature","%f",      msg->temperature);
//    TEXTREAD_ADD_FIELD (tr, "mag_x","%f",      msg->mag_x);
//    TEXTREAD_ADD_FIELD (tr, "mag_y","%f",      msg->mag_y);
//    TEXTREAD_ADD_FIELD (tr, "mag_z","%f",      msg->mag_z);

    textread_stop (tr);
}

void
senlcm_kvh1750_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_kvh1750_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "angular(:,1)","%f",      msg->angular[0]);
    TEXTREAD_ADD_FIELD (tr, "angular(:,2)","%f",      msg->angular[1]);
    TEXTREAD_ADD_FIELD (tr, "angular(:,3)","%f",      msg->angular[2]);
    TEXTREAD_ADD_FIELD (tr, "linear(:,1)","%f",      msg->linear[0]);
    TEXTREAD_ADD_FIELD (tr, "linear(:,2)","%f",      msg->linear[1]);
    TEXTREAD_ADD_FIELD (tr, "linear(:,3)","%f",      msg->linear[2]);
    TEXTREAD_ADD_FIELD (tr, "temperature","%f",      msg->temperature);
    TEXTREAD_ADD_FIELD (tr, "status", "%"PRIu8,      msg->status);
    TEXTREAD_ADD_FIELD (tr, "sequence", "%"PRIu8,      msg->sequence);

    textread_stop (tr);
}

void
senlcm_usb2000_spec_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                               const senlcm_usb2000_spec_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "timestamp",   "%"PRId64,  msg->timestamp);
    TEXTREAD_ADD_FIELD (tr, "numSamples", "%"PRId16, msg->numSamples);
    TEXTREAD_ADD_FIELD (tr, "id",     "%s",   msg->id);

    for (int i_spec=0; i_spec<2048; i_spec++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "specData(:,%d)", i_spec+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId32,    i_spec < msg->numSamples ? msg->specData[i_spec] : 0);
    }
    TEXTREAD_ADD_FIELD (tr, "sixteenBitData",     "%"PRId8,   msg->sixteenBitData);
    TEXTREAD_ADD_FIELD (tr, "startEndIdx1",     "%"PRId8,   msg->startEndIdx);
    TEXTREAD_ADD_FIELD (tr, "startEndIdx2",     "%"PRId8,   msg->startEndIdx);
    TEXTREAD_ADD_FIELD (tr, "intTime",     "%"PRId16,   msg->intTime);

    textread_stop (tr);
}



void
senlcm_sts_spec_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const senlcm_sts_spec_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "timestamp",   "%"PRId64,  msg->timestamp);
    TEXTREAD_ADD_FIELD (tr, "numSamples", "%"PRId16, msg->numSamples);
    TEXTREAD_ADD_FIELD (tr, "id",     "%s",   msg->id);

    for (int i_spec=0; i_spec<1024; i_spec++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "specData(:,%d)", i_spec+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRId32,    i_spec < msg->numSamples ? msg->specData[i_spec] : 0);
    }
    TEXTREAD_ADD_FIELD (tr, "intTime",     "%"PRId16,   msg->intTime);
    TEXTREAD_ADD_FIELD (tr, "boardTemp", "%f", msg->boardTemp);
    TEXTREAD_ADD_FIELD (tr, "detectTemp", "%f", msg->detectTemp);
    TEXTREAD_ADD_FIELD (tr, "newTemps", "%"PRId8, msg->newTemps);


    textread_stop (tr);
}


void
senlcm_usbl_fix_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const senlcm_usbl_fix_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",   "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "remopte_id",   "%d",  msg->remote_id);
    TEXTREAD_ADD_FIELD (tr, "latitude","%15.12f",      msg->latitude);
    TEXTREAD_ADD_FIELD (tr, "longitude","%15.12f",      msg->longitude);
    TEXTREAD_ADD_FIELD (tr, "depth","%6.3f",      msg->depth);
    TEXTREAD_ADD_FIELD (tr, "accuracy","%f",      msg->accuracy);
    TEXTREAD_ADD_FIELD (tr, "ship_latitude","%15.12f",      msg->ship_latitude);
    TEXTREAD_ADD_FIELD (tr, "ship_longitude","%15.12f",      msg->ship_longitude);
    TEXTREAD_ADD_FIELD (tr, "ship_roll","%6.4f",      msg->ship_roll);
    TEXTREAD_ADD_FIELD (tr, "ship_pitch","%6.4f",      msg->ship_pitch);
    TEXTREAD_ADD_FIELD (tr, "ship_heading","%6.4f",      msg->ship_heading);
    TEXTREAD_ADD_FIELD (tr, "target_x","%6.4f",      msg->target_x);
    TEXTREAD_ADD_FIELD (tr, "target_y","%6.4f",      msg->target_y);
    TEXTREAD_ADD_FIELD (tr, "target_z","%6.4f",      msg->target_z);

    textread_stop (tr);
}


void
senlcm_novatel_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_novatel_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",   "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "gps_time",   "%"PRId64,  msg->gps_time);
    TEXTREAD_ADD_FIELD (tr, "latitude","%15.12f",      msg->latitude);
    TEXTREAD_ADD_FIELD (tr, "longitude","%15.12f",      msg->longitude);
    TEXTREAD_ADD_FIELD (tr, "latitude_sd","%15.12f",      msg->latitude_sd);
    TEXTREAD_ADD_FIELD (tr, "longitude_sd","%15.12f",      msg->longitude_sd);
    TEXTREAD_ADD_FIELD (tr, "roll","%6.3f",      msg->roll);
    TEXTREAD_ADD_FIELD (tr, "pitch","%6.3f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "heading","%6.3f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "height","%6.3f",      msg->height);
    TEXTREAD_ADD_FIELD (tr, "east_velocity","%6.3f",      msg->east_velocity);
    TEXTREAD_ADD_FIELD (tr, "north_velocity","%6.3f",      msg->north_velocity);
    TEXTREAD_ADD_FIELD (tr, "up_velocity","%6.3f",      msg->up_velocity);

    textread_stop (tr);
}

void
senlcm_evologics_usbl_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const senlcm_evologics_usbl_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",   "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "mtime",   "%"PRId64,  msg->mtime);
    TEXTREAD_ADD_FIELD (tr, "ctime",   "%"PRId64,  msg->ctime);
    TEXTREAD_ADD_FIELD (tr, "remopte_id",   "%d",  msg->remote_id)
    TEXTREAD_ADD_FIELD (tr, "x","%f",      msg->x);
    TEXTREAD_ADD_FIELD (tr, "y","%f",      msg->y);
    TEXTREAD_ADD_FIELD (tr, "z","%f",      msg->z);
    TEXTREAD_ADD_FIELD (tr, "e","%f",      msg->e);
    TEXTREAD_ADD_FIELD (tr, "n","%f",      msg->n);
    TEXTREAD_ADD_FIELD (tr, "u","%f",      msg->u);
    TEXTREAD_ADD_FIELD (tr, "r","%f",      msg->r);
    TEXTREAD_ADD_FIELD (tr, "p","%f",      msg->p);
    TEXTREAD_ADD_FIELD (tr, "h","%f",      msg->h);
    TEXTREAD_ADD_FIELD (tr, "prop_time","%f",      msg->prop_time);
    TEXTREAD_ADD_FIELD (tr, "rssi","%d",      msg->rssi);
    TEXTREAD_ADD_FIELD (tr, "accuracy","%f",      msg->accuracy);
    TEXTREAD_ADD_FIELD (tr, "integrity","%d",      msg->integrity);
    TEXTREAD_ADD_FIELD (tr, "depth","%f",      msg->depth);

    textread_stop (tr);
}

void
senlcm_ahrs_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const senlcm_ahrs_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",   "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "roll","%f",      msg->roll);
    TEXTREAD_ADD_FIELD (tr, "pitch","%f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "heading","%f",      msg->heading);

    textread_stop (tr);
}

