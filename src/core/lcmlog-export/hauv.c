#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "perls-common/textread.h"

#include "lcmlog_export.h"
#include "hauv.h"

void
hauv_bs_cnv_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_cnv_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",   "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time_nav",        "%"PRId64, msg->time_nav);
    TEXTREAD_ADD_FIELD (tr, "x",               "%f",      msg->x);
    TEXTREAD_ADD_FIELD (tr, "y",               "%f",      msg->y);
    TEXTREAD_ADD_FIELD (tr, "z",               "%f",      msg->z);
    TEXTREAD_ADD_FIELD (tr, "heading",         "%f",      msg->heading);
    textread_stop (tr);
}


void
hauv_bs_dvl_2_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                         const hauv_bs_dvl_2_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "velocity1",        "%f",      msg->velocity1);
    TEXTREAD_ADD_FIELD (tr, "velocity2",        "%f",      msg->velocity2);
    TEXTREAD_ADD_FIELD (tr, "velocity3",        "%f",      msg->velocity3);
    TEXTREAD_ADD_FIELD (tr, "velocity4",        "%f",      msg->velocity4);
    TEXTREAD_ADD_FIELD (tr, "range1",           "%f",      msg->range1);
    TEXTREAD_ADD_FIELD (tr, "range2",           "%f",      msg->range2);
    TEXTREAD_ADD_FIELD (tr, "range3",           "%f",      msg->range3);
    TEXTREAD_ADD_FIELD (tr, "range4",           "%f",      msg->range4);
    TEXTREAD_ADD_FIELD (tr, "temperature",      "%f",      msg->temperature);
    TEXTREAD_ADD_FIELD (tr, "time_measured",    "%"PRId64, msg->time_measured);
    textread_stop (tr);
}


void
hauv_bs_dvl_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_dvl_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "x_velocity",       "%f",      msg->x_velocity);
    TEXTREAD_ADD_FIELD (tr, "y_velocity",       "%f",      msg->y_velocity);
    TEXTREAD_ADD_FIELD (tr, "z_velocity",       "%f",      msg->z_velocity);
    TEXTREAD_ADD_FIELD (tr, "range1",           "%f",      msg->range1);
    TEXTREAD_ADD_FIELD (tr, "range2",           "%f",      msg->range2);
    TEXTREAD_ADD_FIELD (tr, "range3",           "%f",      msg->range3);
    TEXTREAD_ADD_FIELD (tr, "range4",           "%f",      msg->range4);
    TEXTREAD_ADD_FIELD (tr, "temperature",      "%f",      msg->temperature);
    TEXTREAD_ADD_FIELD (tr, "time_measured",    "%"PRId64, msg->time_measured);
    textread_stop (tr);
}


void
hauv_bs_imu_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_imu_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "rate_x",           "%f",      msg->rate_x);
    TEXTREAD_ADD_FIELD (tr, "rate_y",           "%f",      msg->rate_y);
    TEXTREAD_ADD_FIELD (tr, "rate_z",           "%f",      msg->rate_z);
    TEXTREAD_ADD_FIELD (tr, "acc_x",            "%f",      msg->acc_x);
    TEXTREAD_ADD_FIELD (tr, "acc_y",            "%f",      msg->acc_y);
    TEXTREAD_ADD_FIELD (tr, "acc_z",            "%f",      msg->acc_z);
    TEXTREAD_ADD_FIELD (tr, "time_measured",    "%"PRId64, msg->time_measured);
    textread_stop (tr);
}


void
hauv_bs_nvg_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_nvg_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "latitude",         "%.f",     msg->latitude);
    TEXTREAD_ADD_FIELD (tr, "hemisphere_ns",    "%s",      msg->hemisphere_ns);
    TEXTREAD_ADD_FIELD (tr, "longitude",        "%.f",     msg->longitude);
    TEXTREAD_ADD_FIELD (tr, "hemisphere_ew",    "%s",      msg->hemisphere_ew);
    TEXTREAD_ADD_FIELD (tr, "quality",          "%"PRId8,  msg->quality);
    TEXTREAD_ADD_FIELD (tr, "altitude",         "%f",      msg->altitude);
    TEXTREAD_ADD_FIELD (tr, "depth",            "%f",      msg->depth);
    TEXTREAD_ADD_FIELD (tr, "heading",          "%f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "roll",             "%f",      msg->roll);
    TEXTREAD_ADD_FIELD (tr, "pitch",            "%f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "time_compute",     "%"PRId64, msg->time_compute);
    textread_stop (tr);
}


void
hauv_bs_nvr_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_nvr_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "east_velocity",    "%f",      msg->east_velocity);
    TEXTREAD_ADD_FIELD (tr, "north_velocity",   "%f",      msg->north_velocity);
    TEXTREAD_ADD_FIELD (tr, "down_velocity",    "%f",      msg->down_velocity);
    TEXTREAD_ADD_FIELD (tr, "pitch_rate",       "%f",      msg->pitch_rate);
    TEXTREAD_ADD_FIELD (tr, "roll_rate",        "%f",      msg->roll_rate);
    TEXTREAD_ADD_FIELD (tr, "yaw_rate",         "%f",      msg->yaw_rate);
    textread_stop (tr);
}


void
hauv_bs_pit_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_pit_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "pitch_dvl",        "%f",      msg->pitch_dvl);
    TEXTREAD_ADD_FIELD (tr, "pitch_sonar",      "%f",      msg->pitch_sonar);
    textread_stop (tr);
}


void
hauv_bs_raw_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_raw_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "message", "%s", msg->message);
    textread_stop (tr);
}


void
hauv_bs_rbs_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_rbs_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",        "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",                 "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "battery_number",       "%"PRId32, msg->battery_number);
    TEXTREAD_ADD_FIELD (tr, "voltage",              "%f",      msg->voltage);
    TEXTREAD_ADD_FIELD (tr, "minimum_cell_voltage", "%f",      msg->minimum_cell_voltage);
    TEXTREAD_ADD_FIELD (tr, "maximum_cell_voltage", "%f",      msg->maximum_cell_voltage);
    TEXTREAD_ADD_FIELD (tr, "temperature",          "%f",      msg->temperature);
    textread_stop (tr);
}


void
hauv_bs_rcm_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_rcm_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "compass_number",   "%"PRId32, msg->compass_number);
    TEXTREAD_ADD_FIELD (tr, "heading",          "%f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "pitch",            "%f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "roll",             "%f",      msg->roll);
    TEXTREAD_ADD_FIELD (tr, "time_measured",    "%"PRId64, msg->time_measured);
    textread_stop (tr);
}


void
hauv_bs_rdp_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_rdp_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "pressure",         "%f",      msg->pressure);
    textread_stop (tr);
}


void
hauv_bs_rnv_2_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                         const hauv_bs_rnv_2_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "horizontal",       "%f",      msg->horizontal);
    TEXTREAD_ADD_FIELD (tr, "vertical",         "%f",      msg->vertical);
    TEXTREAD_ADD_FIELD (tr, "distance",         "%f",      msg->distance);
    TEXTREAD_ADD_FIELD (tr, "heading",          "%f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "depth",            "%f",      msg->depth);
    TEXTREAD_ADD_FIELD (tr, "absheading",       "%f",      msg->absheading);
    TEXTREAD_ADD_FIELD (tr, "absroll",          "%f",      msg->absroll);
    TEXTREAD_ADD_FIELD (tr, "abspitch",         "%f",      msg->abspitch);
    TEXTREAD_ADD_FIELD (tr, "time_nav",         "%"PRId64, msg->time_nav);
    textread_stop (tr);
}


void
hauv_bs_rnv_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_rnv_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "horizontal",       "%f",      msg->horizontal);
    TEXTREAD_ADD_FIELD (tr, "vertical",         "%f",      msg->vertical);
    TEXTREAD_ADD_FIELD (tr, "distance",         "%f",      msg->distance);
    TEXTREAD_ADD_FIELD (tr, "heading",          "%f",      msg->heading);
    textread_stop (tr);
}


void
hauv_didson_raw_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const hauv_didson_raw_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time_received",    "%"PRId64, msg->time_received);
    TEXTREAD_ADD_FIELD (tr, "length",           "%"PRId32, msg->length);
    for (int i=0; i<12; i++)
    {
        char buf[20];
        snprintf (buf, sizeof buf, "data(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, buf, "%"PRIu8, i < msg->length ? msg->data[i] : 0);
    }
    textread_stop (tr);
}


void
hauv_didson_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_didson_t *msg, void *user)
{
    return;
}


void
hauv_pl_gbp_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_gbp_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time",               "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "x",                  "%f",      msg->x);
    TEXTREAD_ADD_FIELD (tr, "y",                  "%f",      msg->y);
    TEXTREAD_ADD_FIELD (tr, "relative_bearing",   "%f",      msg->relative_bearing);
    TEXTREAD_ADD_FIELD (tr, "is_depth",           "%"PRId8,  msg->is_depth);
    TEXTREAD_ADD_FIELD (tr, "altitude",           "%f",      msg->depth_altitude);
    textread_stop (tr);
}


void
hauv_pl_ghp_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_ghp_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time",             "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "horizontal",       "%f",      msg->horizontal);
    TEXTREAD_ADD_FIELD (tr, "vertical",         "%f",      msg->vertical);
    TEXTREAD_ADD_FIELD (tr, "distance",         "%f",      msg->distance);
    textread_stop (tr);
}


void
hauv_pl_raw_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_raw_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "message", "%s", msg->message);
    textread_stop (tr);
}


void
hauv_pl_san_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_san_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time",  "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "angle", "%f",      msg->angle);
    textread_stop (tr);
}


void
hauv_pl_sus_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_sus_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time", "%"PRId64, msg->time);
    textread_stop (tr);
}


void
hauv_vehicle_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                              const hauv_vehicle_state_t *msg, void * user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "time",       "%"PRId64, msg->time);
    TEXTREAD_ADD_FIELD (tr, "x",          "%f",      msg->x);
    TEXTREAD_ADD_FIELD (tr, "y",          "%f",      msg->y);
    TEXTREAD_ADD_FIELD (tr, "z",          "%f",      msg->z);
    TEXTREAD_ADD_FIELD (tr, "roll",       "%f",      msg->roll);
    TEXTREAD_ADD_FIELD (tr, "pitch",      "%f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "heading",    "%f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "altitude",   "%f",      msg->altitude);
    TEXTREAD_ADD_FIELD (tr, "dvlAngle",   "%f",      msg->dvlAngle);
    TEXTREAD_ADD_FIELD (tr, "sonarAngle", "%f",      msg->sonarAngle);
    TEXTREAD_ADD_FIELD (tr, "rmin",       "%f",      msg->rmin);
    TEXTREAD_ADD_FIELD (tr, "rmax",       "%f",      msg->rmax);
    TEXTREAD_ADD_FIELD (tr, "sonarX",     "%f",      msg->sonarX);
    TEXTREAD_ADD_FIELD (tr, "sonarY",     "%f",      msg->sonarY);
    TEXTREAD_ADD_FIELD (tr, "sonarZ",     "%f",      msg->sonarZ);
    textread_stop (tr);
}
