#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <glib.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/error.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-common/lcm_util.h"
#include "perls-common/nmea.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-lcmtypes/senlcm_uvc_ack_t.h"
#include "perls-lcmtypes/senlcm_uvc_ojw_t.h"
#include "perls-lcmtypes/senlcm_uvc_omload_t.h"
#include "perls-lcmtypes/senlcm_uvc_omp_t.h"
#include "perls-lcmtypes/senlcm_uvc_omstart_t.h"
#include "perls-lcmtypes/senlcm_uvc_omstop_t.h"
#include "perls-lcmtypes/senlcm_uvc_opi_t.h"
#include "perls-lcmtypes/senlcm_uvc_opos_t.h"
#include "perls-lcmtypes/senlcm_uvc_osd_t.h"
#include "perls-lcmtypes/senlcm_uvc_osi_t.h"
#include "perls-lcmtypes/senlcm_uvc_rphtd_t.h"

#include "perls-lcmtypes/perllcm_auv_os_conduit_raw_t.h"
#include "perls-lcmtypes/perllcm_auv_os_conduit_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define PUBLISH_OS_CONDUIT_RAW  1

typedef struct _state_t state_t;
struct _state_t
{
    // publish channels
    char *lcm_channel_out;

    // lcm channel prefix
    char *prefix;

    // os-conduit channels
    char *osc_ack_channel;
    char *osc_ojw_channel;
    char *osc_omp_channel;
    char *osc_opi_channel;
    char *osc_osd_channel;
    char *osc_osi_channel;
    char *osc_out_channel;
    char *osc_omstart_channel;
    char *osc_omstop_channel;
    char *osc_omload_channel;
    char *osc_opos_channel;
    char *osc_rphtd_channel;

    // heartbeat channels
    char *hb1_channel;
    char *hb5_channel;
    char *hb10_channel;

    // uvc serial parameters
    perllcm_auv_os_conduit_t os_conduit;
    senlcm_uvc_osd_t osd;
    generic_sensor_driver_t *gsd;
};

static void
os_conduit_init (perllcm_auv_os_conduit_t *os_conduit)
{
    memset (os_conduit, 0, sizeof (*os_conduit));
    os_conduit->ojw.value = strdup ("");
    os_conduit->omload.value = strdup ("");
    os_conduit->omp.value = strdup ("");
    os_conduit->omstart.value = strdup ("");
    os_conduit->omstop.value = strdup ("");
    os_conduit->opos.value = strdup ("");
    os_conduit->osd.value = strdup ("");
}

static void
osd_init (senlcm_uvc_osd_t *osd, BotParam *param)
{
    osd->gps     = bot_param_get_int_or_fail (param, "os-conduit.osd.gps");
    osd->compass = bot_param_get_int_or_fail (param, "os-conduit.osd.compass");
    osd->state   = bot_param_get_int_or_fail (param, "os-conduit.osd.state");
    osd->power   = bot_param_get_int_or_fail (param, "os-conduit.osd.power");
    osd->ysi     = bot_param_get_int_or_fail (param, "os-conduit.osd.ysi");
}

static void
publish (state_t *state, perllcm_auv_os_conduit_raw_t *sensor, 
         const int64_t utime, const char msg[])
{
#if PUBLISH_OS_CONDUIT_RAW
    state->os_conduit.utime = utime;
    sensor->age = (utime - sensor->utime) / 1000;
    sensor->utime = utime;
    free (sensor->value);
    sensor->value = strdup (msg);
    //printf ("%s\n", state->osc_out_channel);
    perllcm_auv_os_conduit_t_publish (state->gsd->lcm, state->osc_out_channel, &state->os_conduit);
#endif
}

/* returns 1 for successfully classifying and publishing uvc to
 * backseat driver messages */
static int
publish_from_uvc (state_t *state, const char *msg, const int64_t timestamp)
{
    // $OSI -- vehicle state data
    if (0==strncmp (msg, "$OSI", 4)) {
        char hex[128] = {'\0'};
        if (!nmea_arg (msg, 1, hex))
            ERROR ("unable to extract OSI hex");

        int yt=-1, yb=-1, pl=-1, pr=-1, mot=-1;
        if (5!=sscanf (hex, "%02x%02x%02x%02x%02x", &yt, &yb, &pl, &pr, &mot))
            ERROR ("unable to parse OSI yt, yb, pl, pr, mot");

        char mode_str[128];
        if (!nmea_arg (msg, 2, mode_str))
            ERROR ("unable to parse OSI mode_str");

        int mode = -1;
        if (0==strncmp (mode_str, "N", 1))
            mode = SENLCM_UVC_OSI_T_MODE_NORMAL;
        else if (0==strncmp (mode_str, "S", 1))
            mode = SENLCM_UVC_OSI_T_MODE_STOPPED;
        else if (0==strncmp (mode_str, "P", 1))
            mode = SENLCM_UVC_OSI_T_MODE_PARK;
        else if (0==strncmp (mode_str, "Mp", 2))
            mode = SENLCM_UVC_OSI_T_MODE_MANUAL_PARK;
        else if (0==strncmp (mode_str, "M", 1))
            mode = SENLCM_UVC_OSI_T_MODE_MANUAL_OVERRIDE;
        else if (0==strncmp (mode_str, "A", 1))
            mode = SENLCM_UVC_OSI_T_MODE_SERVO;
        else if (0==strncmp (mode_str, "W", 1))
            mode = SENLCM_UVC_OSI_T_MODE_MISSION;
        else
            ERROR ("unable to parse OSI mode");

        int nextwp = 0;
        if (!nmea_argi (msg, 3, &nextwp))
            ERROR ("unable to parse OSI nextwp");

        double latitude = 0;
        if (!nmea_argf (msg, 4, &latitude))
            ERROR ("unable to parse OSI latitude");

        double longitude = 0;
        if (!nmea_argf (msg, 5, &longitude))
            ERROR ("unable to parse OSI longitude");

        double speed = 0;
        if (!nmea_argf (msg, 6, &speed))
            ERROR ("unable to parse OSI speed");

        double dist_to_nextwp = 0;
        if (!nmea_argf (msg, 7, &dist_to_nextwp))
            ERROR ("unable to parse OSI dist_to_nextwp");

        char error[128] = {'\0'};
        if (!nmea_arg (msg, 8, error))
            ERROR ("unable to parse OSI error");

        int err_state = -1;
        if (0==strncmp (error, "N", 1))
            err_state = SENLCM_UVC_OSI_T_ERR_NONE;
        else if (0==strncmp (error, "Sop", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_OVER_PITCH;
        else if (0==strncmp (error, "Stl", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_EXCEED_TIME;
        else if (0==strncmp (error, "Sle", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_LEAK;
        else if (0==strncmp (error, "Sfp", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_NO_FORWARD_PROG;
        else if (0==strncmp (error, "Sed", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_EXCEED_MAX_DEPTH;
        else if (0==strncmp (error, "Sup", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_NO_UPWARD_PROG;
        else if (0==strncmp (error, "Stf", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_TOW_FLOAT_ENGAGED;
        else if (0==strncmp (error, "Srp", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_SAFETY_RET_PATH;
        else if (0==strncmp (error, "Snd", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_DFS_UNCHANGED;
        else if (0==strncmp (error, "Snc", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_COMPASS_STOPPED;
        else if (0==strncmp (error, "Spw", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_EXCEEDED_MIN_REQ_PWR;
        else if (0==strncmp (error, "Sti", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_STOP_AND_TRANSMIT_IRIDIUM;
        else
            ERROR ("unable to parse OSI err_state");

        double altimeter = 0;
        if (!nmea_argf (msg, 9, &altimeter))
            ERROR ("unable to parse OSI altimeter");

        char park[128]; double park_time = 0;
        if (!nmea_arg (msg, 10, park))
            ERROR ("unable to parse OSI park");
        else
            park_time = atoi (park+1);

        double magnetic_dec = 0;
        if (!nmea_argf (msg, 11, &magnetic_dec))
            ERROR ("unable to parse OSI magnetic_dec");

        senlcm_uvc_osi_t data = {
            .utime          = timestamp,
            .yaw_top        = yt,
            .yaw_bot        = yb,
            .pitch_left     = pl,
            .pitch_right    = pr,
            .motor          = mot, 
            .mode           = mode,
            .nextwp         = nextwp + 1,  // to match match vector map indexing
            .latitude       = DTOR * latitude,
            .longitude      = DTOR * longitude,
            .speed          = UNITS_KNOT_TO_METER_PER_SEC * speed,
            .dist_to_nextwp = dist_to_nextwp,
            .error          = err_state,
            .altimeter      = UNITS_FEET_TO_METER * altimeter,
            .park_time      = park_time,
            .magnetic_dec   = magnetic_dec,
        };
        senlcm_uvc_osi_t_publish (state->gsd->lcm, state->osc_osi_channel, &data); 
        return 1;
    }

    // $OPI -- vehicle power data
    else if (0==strncmp (msg,"$OPI",4)) {
        double percent = 0;
        if (!nmea_argf (msg, 1, &percent))
            ERROR ("unable to parse OPI percent");

        double remaining_cap = 0;
        if (!nmea_argf (msg, 2, &remaining_cap))
            ERROR ("unable to parse OPI remaining_cap");

        double pwr = 0;
        if (!nmea_argf (msg, 3, &pwr))
            ERROR ("unable to parse OPI pwr");

        double volts = 0;
        if (!nmea_argf (msg, 4, &volts))
            ERROR ("unable to parse OPI volts");

        double current = 0;
        if (!nmea_argf (msg, 5, &current))
            ERROR ("unable to parse OPI current");

        double time_til = 0;
        if (!nmea_argf (msg, 6, &time_til))
            ERROR ("unable to parse OPI time_til");

        char bs = '\0';
        if (!nmea_argc (msg, 7, &bs))
            ERROR ("unable to parse OPI bs");

        int batt_state = 0;
        switch (bs) {
            case 'C':
                batt_state = SENLCM_UVC_OPI_T_BS_CHARGING;
                break;
            case 'D':
                batt_state = SENLCM_UVC_OPI_T_BS_DISCHARGING;
                break;
            case 'F':
                batt_state = SENLCM_UVC_OPI_T_BS_FAULT;
                break;
            default:
                ERROR ("unable to parse OPI batt_state");
        }

        int leak = 0;
        if (!nmea_argi (msg, 8, &leak))
            ERROR ("unable to parse OPI leak");

        senlcm_uvc_opi_t data = {
            .utime         = timestamp,
            .percent       = percent,
            .remaining_cap = remaining_cap,
            .pwr           = pwr,
            .volts         = volts,
            .current       = current,
            .time_til      = time_til,	    
            .batt_state    = batt_state,
            .leak          = leak,
        };
        senlcm_uvc_opi_t_publish (state->gsd->lcm, state->osc_opi_channel, &data);
        return 1;
    }  

    // $C#P#R#T#D# -- uvc to host compass string
    else if (0==strncmp (msg, "$C", 2)) {
        double heading, pitch, roll, temp, depth = 0;
        if (5!=sscanf (msg, "$C%lfP%lfR%lfT%lfD%lf", &heading, &pitch, &roll, &temp, &depth)) {
            ERROR ("unable to parse UVC Compass string!");
        }
        
        senlcm_uvc_rphtd_t data = {
            .utime         = timestamp,
            .rph           = {roll*DTOR, pitch*DTOR, heading*DTOR},
            .T             = temp,
            .depth         = depth * UNITS_FEET_TO_METER,
        };
        senlcm_uvc_rphtd_t_publish (state->gsd->lcm, state->osc_rphtd_channel, &data);
        return 1;
    }

    // $ACK -- uvc ack of backseat driver cmd
    else if (0==strncmp (msg,"$ACK",4)) {
        int msg_type = 0;
        if (!nmea_argi (msg, 1, &msg_type))
            ERROR ("unable to parse ACK msg_type");

        int status = 0;
        if (!nmea_argi (msg, 2, &status))
            ERROR ("unable to parse ACK status");

        int err_num = 0;
        if (!nmea_argi (msg, 3, &err_num))
            ERROR ("unable to parse ACK err_num");

        char usrset[128] = {'\0'};
        int usrnum = 0;
        char usrval[128] = {'\0'};
        if (msg_type == SENLCM_UVC_ACK_T_ACK_ORWSET) {
            if (!nmea_arg (msg, 4, usrset))
                ERROR ("unable to parse ACK usrset");

            if (!nmea_argi (msg, 5, &usrnum))
                ERROR ("unable to parse ACK usrnum");

            if (!nmea_arg (msg,6,usrval))
                ERROR ("unable to parse ACK usrval");
        }

        senlcm_uvc_ack_t data = {
            .utime    = timestamp,
            .msg_type = msg_type,
            .status   = status,
            .err_num  = err_num,
            .usrset   = usrset,
            .usrnum   = usrnum,
            .usrval   = usrval,
        };
        senlcm_uvc_ack_t_publish (state->gsd->lcm, state->osc_ack_channel, &data); 
        return 1;
    }

    return 0;
}


static void
compass_query_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_heartbeat_t *beat, void *user)
{
    state_t *state = user;

    char msg[256];
    nmea_sprintf (msg, "$OSD,%s%s%s%s%s,*", ",", "C", ",", ",", ",");

    int64_t osd_utime = timestamp_now ();
    gsd_write (state->gsd, msg, strlen (msg));
    publish (state, &state->os_conduit.osd, osd_utime, msg);
}

static void
osd_query_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_heartbeat_t *beat, void *user)
{
    state_t *state = user;

    int g = state->osd.gps, c = state->osd.compass, s = state->osd.state,
    p = state->osd.power, y = state->osd.ysi;

    char msg[256];

    nmea_sprintf (msg, "$OSD,%s%s%s%s%s,*",
                        g == 0 ? "," : "G,", 
                        c == 0 ? "," : "C,", 
                        s == 0 ? "," : "S,",
                        p == 0 ? "," : "P,", 
                        y == 0 ? "," : "Y,");

    int64_t osd_utime = timestamp_now ();
    gsd_write (state->gsd, msg, strlen (msg));
    publish (state, &state->os_conduit.osd, osd_utime, msg);
}

static void
osd_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_uvc_osd_t *new_osd, void *user)
{
    state_t *state = user;
    state->osd.gps = new_osd->gps;
    state->osd.compass = new_osd->compass;
    state->osd.state = new_osd->state;
    state->osd.power = new_osd->power;
    state->osd.ysi = new_osd->ysi;
}

static void
omp_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_uvc_omp_t *cmd, void *user)
{
    state_t *state = user;

    int fyt = cmd->yaw_top, fyb = cmd->yaw_bot, fpl = cmd->pitch_left, fpr = cmd->pitch_right;
    int ms = cmd->motor, to = cmd->timeout;

    char msg[256];
    nmea_sprintf  (msg, "$OMP,%02x%02x%02x%02x%02x,00,%d*",
                   fyt, fyb, fpl, fpr, ms, to);
    
    int64_t omp_utime = timestamp_now ();
    gsd_write (state->gsd, msg, strlen (msg));
    publish (state, &state->os_conduit.omp, omp_utime, msg);
}

static void
omstart_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_uvc_omstart_t *cmd, void *user)
{
    state_t *state = user;
    char msg[256];
    nmea_sprintf (msg, "$OMSTART,%d,%d,%d,%d,%s*",
                  cmd->msg_flag_gps,
                  cmd->msg_flag_sounder,
                  cmd->msg_flag_cal_pressure,
                  cmd->mission_type,
                  cmd->mission_name);

    int64_t omstart_utime = timestamp_now ();
    gsd_write (state->gsd, msg, strlen (msg));
    publish (state, &state->os_conduit.omstart, omstart_utime, msg);
}

static void
omstop_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_uvc_omstop_t *cmd, void *user)
{
    state_t *state = user;
    
    char msg[256];
    nmea_sprintf (msg, "$OMSTOP,%d*", cmd->msg_flag);
    
    int64_t omstop_utime = timestamp_now ();
    gsd_write (state->gsd, msg, strlen (msg));
    publish (state, &state->os_conduit.omstop, omstop_utime, msg);
}

static void
omload_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_uvc_omload_t *cmd, void *user)
{
    state_t *state = user;

    char msg[256];
    nmea_sprintf (msg, "$OMLOAD,%s,%d*", cmd->directory_name, cmd->msg_flag);
    
    int64_t omload_utime = timestamp_now ();
    gsd_write (state->gsd, msg, strlen (msg));
    publish (state, &state->os_conduit.omload, omload_utime, msg);
}

static void
ojw_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_uvc_ojw_t *cmd, void *user)
{
    state_t *state = user;

    char msg[256];
    nmea_sprintf (msg, "$OJW,%d*", cmd->waypoint);
    
    int64_t ojw_utime = timestamp_now ();
    gsd_write (state->gsd, msg, strlen (msg));
    publish (state, &state->os_conduit.ojw, ojw_utime, msg);
}

static void
opos_callback (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_uvc_opos_t *cmd, void *user)
{
    state_t *state = user;

    char msg[256];
    //nmea_sprintf (msg, "$OPOS,%f,%f,%f,%f,%f*", 
    //        cmd->latitude, cmd->longitude, cmd->speed, cmd->conductivity, cmd->temperature);

    // just pack lat/lon/speed for now JMW 08/12 HACK
    nmea_sprintf (msg, "$OPOS,%f,%f,%f,,*", 
            cmd->latitude*UNITS_RADIAN_TO_DEGREE, 
            cmd->longitude*UNITS_RADIAN_TO_DEGREE, 
            cmd->speed*UNITS_METER_PER_SEC_TO_KNOT);

    int64_t opos_utime = timestamp_now ();
    gsd_write (state->gsd, msg, strlen (msg));
    publish (state, &state->os_conduit.opos, opos_utime, msg);
}

static void *
lcm_thread (void *context)
{
    generic_sensor_driver_t *gsd = context;
    while (!gsd->done)
        lcm_handle (gsd->lcm);
    
    return 0;
}

static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Ocean-Server Remote Helm I/O driver.");
    return 0;
}

int
main (int argc, char *argv[])
{
    state_t *state = calloc (1, sizeof (*state));
    os_conduit_init (&state->os_conduit);

    // generic sensor driver
    state->gsd = gsd_create (argc, argv, "os-conduit", &myopts);
    gsd_canonical (state->gsd, '\r','\n');
    gsd_launch (state->gsd);
        
    osd_init (&state->osd, state->gsd->params);

    // parse lcm channel names
    state->osc_ack_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_ACK);
    state->osc_ojw_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OJW);
    state->osc_omp_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OMP);
    state->osc_opi_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OPI);
    state->osc_opos_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OPOS);
    state->osc_osd_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OSD);
    state->osc_osi_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OSI);
    state->osc_out_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OUT);
    state->osc_rphtd_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_RPHTD);

    state->osc_omstart_channel = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OMSTART);
    state->osc_omstop_channel  = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OMSTOP);
    state->osc_omload_channel  = lcmu_channel_get_os_conduit (state->gsd->params, LCMU_CHANNEL_OS_CONDUIT_OMLOAD);

    state->prefix = bot_param_get_str_or_fail (state->gsd->params, "vehicle.lcm_channel_prefix");
    state->hb1_channel  = lcmu_channel_get_heartbeat (state->prefix, 1);
    state->hb5_channel  = lcmu_channel_get_heartbeat (state->prefix, 5);
    state->hb10_channel = lcmu_channel_get_heartbeat (state->prefix, 10);

    // subscribe to os-conduit commmand channels
    senlcm_uvc_osd_t_subscribe (state->gsd->lcm, state->osc_osd_channel, &osd_callback, state);    
    senlcm_uvc_omp_t_subscribe (state->gsd->lcm, state->osc_omp_channel, &omp_callback, state);
    senlcm_uvc_ojw_t_subscribe (state->gsd->lcm, state->osc_ojw_channel, &ojw_callback, state);
    senlcm_uvc_opos_t_subscribe (state->gsd->lcm, state->osc_opos_channel, &opos_callback, state);

    senlcm_uvc_omstart_t_subscribe (state->gsd->lcm, state->osc_omstart_channel, &omstart_callback, state);
    senlcm_uvc_omstop_t_subscribe (state->gsd->lcm, state->osc_omstop_channel, &omstop_callback, state);
    senlcm_uvc_omload_t_subscribe (state->gsd->lcm, state->osc_omload_channel, &omload_callback, state);

    perllcm_heartbeat_t_subscribe (state->gsd->lcm, state->hb1_channel, &osd_query_callback, state);

    // query compass at 10Hz to reverse engineer ocean-server soft-iron compass calibration
/*     perllcm_heartbeat_t_subscribe (state->gsd->lcm, state->hb10_channel, &compass_query_callback, state); */

    // launch lcm_thread
    pthread_t tid;
    pthread_create (&tid, NULL, lcm_thread, state->gsd);
    pthread_detach (tid);

    gsd_flush (state->gsd);
    gsd_reset_stats (state->gsd);
    while (!state->gsd->done) {
        // read stream
        char buf[1024];
        int64_t timestamp;
        int len = gsd_read (state->gsd, buf, sizeof buf, &timestamp);
        if (len > 0) {
            if (publish_from_uvc (state, buf, timestamp)) {
                gsd_update_stats (state->gsd, 1);
            }
            else {
                gsd_update_stats (state->gsd, -1);
            }
        }
        else {
            gsd_update_stats (state->gsd, -1);
        }
    }
}
