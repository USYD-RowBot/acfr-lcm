#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "perls-common/textread.h"

#include "lcmlog_export.h"
#include "perllcm.h"

void
perllcm_est_navigator_debug_meas_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                            const perllcm_est_navigator_debug_meas_t *msg, void *user) 
{
    lcmlog_export_t *lle = user;
    
    char *channel_id = g_strconcat (channel, "_", msg->id_str, NULL);
    textread_t *tr = lle_get_textread (lle, channel_id);
    g_free (channel_id);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",              "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "id_str",             "%s",       msg->id_str);
    TEXTREAD_ADD_FIELD (tr, "meas_len",           "%d",       msg->meas_len);
    TEXTREAD_ADD_FIELD (tr, "nis",                "%.15f",    msg->nis);
    TEXTREAD_ADD_FIELD (tr, "mahal_innov_passed", "%.d",      msg->mahal_innov_passed);
    
    int sl = msg->meas_len;
    for (int i = 0; i < sl; i++) {
        char z_str[128],  eta_str[128];
        snprintf (z_str, sizeof z_str, "z(:,%d)", i+1);
        snprintf (eta_str, sizeof eta_str, "nu(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, z_str,   "%.15f", msg->z[i]);
        TEXTREAD_ADD_FIELD (tr, eta_str, "%.15f", msg->nu[i]);
    }
    textread_stop (tr);
}


void
perllcm_est_navigator_debug_pred_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                                            const perllcm_est_navigator_debug_pred_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",            "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "dt",               "%.15f",    msg->dt);
    textread_stop (tr);
}


void
perllcm_auv_navigator_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                                 const perllcm_auv_navigator_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",            "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "est_method",       "%d",       msg->est_method);
    TEXTREAD_ADD_FIELD (tr, "init_data_ready",  "%d",       msg->init_data_ready);
    TEXTREAD_ADD_FIELD (tr, "filter_running",   "%d",       msg->filter_running);
    TEXTREAD_ADD_FIELD (tr, "mu_len",           "%d",       msg->mu_len);
    TEXTREAD_ADD_FIELD (tr, "Sigma_len",        "%d",       msg->Sigma_len);

    int sl = msg->state_len;
    for (int i = 0; i < sl; i++) {
        char tmp[128];
        snprintf (tmp, sizeof tmp, "mu(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", msg->mu_len==sl ? msg->mu[i] : 0.0);
    }

    int sl2 = sl*sl;
    for (int i = 0; i < sl2 ; i++) {
        char tmp[128];
        snprintf (tmp, sizeof tmp, "Sigma(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", msg->Sigma_len==sl2 ? msg->Sigma[i] : 0.0);
    }
    
    TEXTREAD_ADD_FIELD (tr, "mu_latitude",      "%.15f",    msg->mu_latitude);
    TEXTREAD_ADD_FIELD (tr, "mu_longitude",     "%.15f",    msg->mu_longitude);
    TEXTREAD_ADD_FIELD (tr, "org_latitude",     "%.15f",    msg->org_latitude);
    TEXTREAD_ADD_FIELD (tr, "org_longitude",    "%.15f",    msg->org_longitude);
    
    // index fields
    TEXTREAD_ADD_FIELD (tr, "index_proc_state_len", "%d",   msg->index.proc_state_len);
    TEXTREAD_ADD_FIELD (tr, "index_u_len",      "%d",       msg->index.u_len);
    TEXTREAD_ADD_FIELD (tr, "index_x",          "%d",       msg->index.x);
    TEXTREAD_ADD_FIELD (tr, "index_y",          "%d",       msg->index.y);
    TEXTREAD_ADD_FIELD (tr, "index_z",          "%d",       msg->index.z);
    TEXTREAD_ADD_FIELD (tr, "index_r",          "%d",       msg->index.r);
    TEXTREAD_ADD_FIELD (tr, "index_p",          "%d",       msg->index.p);
    TEXTREAD_ADD_FIELD (tr, "index_h",          "%d",       msg->index.h);
    TEXTREAD_ADD_FIELD (tr, "index_u",          "%d",       msg->index.u);
    TEXTREAD_ADD_FIELD (tr, "index_v",          "%d",       msg->index.v);
    TEXTREAD_ADD_FIELD (tr, "index_w",          "%d",       msg->index.w);
    TEXTREAD_ADD_FIELD (tr, "index_a",          "%d",       msg->index.a);
    TEXTREAD_ADD_FIELD (tr, "index_b",          "%d",       msg->index.b);
    TEXTREAD_ADD_FIELD (tr, "index_c",          "%d",       msg->index.c);
    TEXTREAD_ADD_FIELD (tr, "index_x_dot",      "%d",       msg->index.x_dot);
    TEXTREAD_ADD_FIELD (tr, "index_y_dot",      "%d",       msg->index.y_dot);
    TEXTREAD_ADD_FIELD (tr, "index_z_dot",      "%d",       msg->index.z_dot);
    TEXTREAD_ADD_FIELD (tr, "index_theta_dot",  "%d",       msg->index.r_dot);
    TEXTREAD_ADD_FIELD (tr, "index_phi_dot",    "%d",       msg->index.p_dot);
    TEXTREAD_ADD_FIELD (tr, "index_psi_dot",    "%d",       msg->index.h_dot);
    textread_stop (tr);
}


void
perllcm_segway_navigator_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                                 const perllcm_segway_navigator_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",                "%"PRId64,  msg->utime);
    TEXTREAD_ADD_FIELD (tr, "est_method",           "%d",       msg->est_method);
    TEXTREAD_ADD_FIELD (tr, "init_data_ready",      "%d",       msg->init_data_ready);
    TEXTREAD_ADD_FIELD (tr, "filter_running",       "%d",       msg->filter_running);
    TEXTREAD_ADD_FIELD (tr, "mu_len",               "%d",       msg->mu_len);
    TEXTREAD_ADD_FIELD (tr, "Sigma_len",            "%d",       msg->Sigma_len);
    TEXTREAD_ADD_FIELD (tr, "state_len",            "%d",       msg->state_len);
    TEXTREAD_ADD_FIELD (tr, "delayed_state_len",    "%d",       msg->delayed_state_len);
    TEXTREAD_ADD_FIELD (tr, "max_delayed_states",   "%d",       msg->max_delayed_states);
    TEXTREAD_ADD_FIELD (tr, "delayed_states_cnt",   "%d",       msg->delayed_states_cnt);

    if (-1 == msg->max_delayed_states) {// no limit on the maximum state lenght, just do one delayed state worth
        
        int sl = msg->delayed_state_len;
        for (int i = 0; i < sl; i++) {
            char tmp[128];
            snprintf (tmp, sizeof tmp, "mu(:,%d)", i+1);
            TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", msg->mu_len >= sl ? msg->mu[i] : 0.0);
        }
    
        int sl2 = sl*sl;
        for (int i = 0; i < sl ; i++) {
            for (int j = 0; j < sl ; j++) {
                char tmp[128];
                snprintf (tmp, sizeof tmp, "Sigma(:,%d)", i*sl+j+1);
                TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", msg->Sigma_len>=sl2 ? msg->Sigma[i*sl+j] : 0.0);
            }
        }
        
    } else { // known maximum delayed state length
        
        int sl = msg->state_len;
        int ml = msg->delayed_state_len * msg->max_delayed_states;
        for (int i = 0; i < ml; i++) {
            char tmp[128];
            snprintf (tmp, sizeof tmp, "mu(:,%d)", i+1);
            TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", (i < sl) ? msg->mu[i] : 0.0);
        }
        for (int i = 0; i < ml ; i++) {
            for (int j = 0; j < ml ; j++) {
                char tmp[128];
                snprintf (tmp, sizeof tmp, "Sigma(:,%d)", i*ml+j+1);
                TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", (i < sl && j < sl) ? msg->Sigma[i*sl+j] : 0.0);
            }
        }
    }
    
    
    for (int i = 0; i < msg->max_delayed_states ; i++) {
        char tmp[128];
        snprintf (tmp, sizeof tmp, "delayed_states_utime(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, tmp, "%.15f",
            (i < msg->delayed_states_cnt) ? msg->delayed_states_utime[i] : 0.0);
    }
    
    
    TEXTREAD_ADD_FIELD (tr, "org_latitude",     "%.15f",    msg->org_latitude);
    TEXTREAD_ADD_FIELD (tr, "org_longitude",    "%.15f",    msg->org_longitude);
    
    // index fields
    TEXTREAD_ADD_FIELD (tr, "index_proc_state_len", "%d",   msg->index.proc_state_len);
    TEXTREAD_ADD_FIELD (tr, "index_u_len",      "%d",       msg->index.u_len);
    TEXTREAD_ADD_FIELD (tr, "index_x",          "%d",       msg->index.x);
    TEXTREAD_ADD_FIELD (tr, "index_y",          "%d",       msg->index.y);
    TEXTREAD_ADD_FIELD (tr, "index_z",          "%d",       msg->index.z);
    TEXTREAD_ADD_FIELD (tr, "index_r",          "%d",       msg->index.r);
    TEXTREAD_ADD_FIELD (tr, "index_p",          "%d",       msg->index.p);
    TEXTREAD_ADD_FIELD (tr, "index_h",          "%d",       msg->index.h);
    TEXTREAD_ADD_FIELD (tr, "index_u",          "%d",       msg->index.u);
    TEXTREAD_ADD_FIELD (tr, "index_v",          "%d",       msg->index.v);
    TEXTREAD_ADD_FIELD (tr, "index_w",          "%d",       msg->index.w);
    TEXTREAD_ADD_FIELD (tr, "index_a",          "%d",       msg->index.a);
    TEXTREAD_ADD_FIELD (tr, "index_b",          "%d",       msg->index.b);
    TEXTREAD_ADD_FIELD (tr, "index_c",          "%d",       msg->index.c);
    TEXTREAD_ADD_FIELD (tr, "index_x_dot",      "%d",       msg->index.x_dot);
    TEXTREAD_ADD_FIELD (tr, "index_y_dot",      "%d",       msg->index.y_dot);
    TEXTREAD_ADD_FIELD (tr, "index_z_dot",      "%d",       msg->index.z_dot);
    TEXTREAD_ADD_FIELD (tr, "index_theta_dot",  "%d",       msg->index.r_dot);
    TEXTREAD_ADD_FIELD (tr, "index_phi_dot",    "%d",       msg->index.p_dot);
    TEXTREAD_ADD_FIELD (tr, "index_psi_dot",    "%d",       msg->index.h_dot);
    textread_stop (tr);
}

void
perllcm_segway_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                                const perllcm_segway_state_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",                    "%"PRId64,  msg->utime);    
    TEXTREAD_ADD_FIELD (tr, "pitch_angle",              "%.15f",    msg->pitch_angle);
    TEXTREAD_ADD_FIELD (tr, "pitch_rate",               "%.15f",    msg->pitch_rate);
    TEXTREAD_ADD_FIELD (tr, "roll_angle",               "%.15f",    msg->roll_angle);
    TEXTREAD_ADD_FIELD (tr, "roll_rate",                "%.15f",    msg->roll_rate);
    TEXTREAD_ADD_FIELD (tr, "left_wheel_velocity",      "%.15f",    msg->left_wheel_velocity);
    TEXTREAD_ADD_FIELD (tr, "right_wheel_velocity",     "%.15f",    msg->right_wheel_velocity);
    TEXTREAD_ADD_FIELD (tr, "yaw_rate",                 "%.15f",    msg->yaw_rate);
    TEXTREAD_ADD_FIELD (tr, "servo_frames",             "%.15f",    msg->servo_frames);
    TEXTREAD_ADD_FIELD (tr, "left_wheel_displacement",  "%.15f",    msg->left_wheel_displacement);
    TEXTREAD_ADD_FIELD (tr, "right_wheel_displacement", "%.15f",    msg->right_wheel_displacement);
    TEXTREAD_ADD_FIELD (tr, "forward_displacement",     "%.15f",    msg->forward_displacement);
    TEXTREAD_ADD_FIELD (tr, "yaw_displacement",         "%.15f",    msg->yaw_displacement);
    TEXTREAD_ADD_FIELD (tr, "left_motor_torque",        "%.15f",    msg->left_motor_torque);
    TEXTREAD_ADD_FIELD (tr, "right_motor_torque",       "%.15f",    msg->right_motor_torque);
    TEXTREAD_ADD_FIELD (tr, "user_voltage",             "%.15f",    msg->user_voltage);
    TEXTREAD_ADD_FIELD (tr, "powerbase_voltage",        "%.15f",    msg->powerbase_voltage);
    textread_stop (tr);
}


void
perllcm_ardrone_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                                const perllcm_ardrone_state_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",              "%"PRId64,  msg->utime);    
    TEXTREAD_ADD_FIELD (tr, "pitch",              "%.15f",    msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "yaw",                "%.15f",    msg->yaw);
    TEXTREAD_ADD_FIELD (tr, "roll",               "%.15f",    msg->roll);
    TEXTREAD_ADD_FIELD (tr, "altitude",           "%.15f",    msg->altitude);
    TEXTREAD_ADD_FIELD (tr, "vx",                 "%.15f",    msg->vx);
    TEXTREAD_ADD_FIELD (tr, "vy",                 "%.15f",    msg->vy);
    TEXTREAD_ADD_FIELD (tr, "vz",                 "%.15f",    msg->vz);
    TEXTREAD_ADD_FIELD (tr, "battery",            "%.15f",    msg->battery);
    TEXTREAD_ADD_FIELD (tr, "flying",             "%d",       msg->flying);
    textread_stop (tr);
}

void
perllcm_ardrone_drive_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                                const perllcm_ardrone_drive_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",              "%"PRId64,  msg->utime);    
    TEXTREAD_ADD_FIELD (tr, "vx",                 "%.15f",    msg->vx);
    TEXTREAD_ADD_FIELD (tr, "vy",                 "%.15f",    msg->vy);
    TEXTREAD_ADD_FIELD (tr, "vz",                 "%.15f",    msg->vz);
    TEXTREAD_ADD_FIELD (tr, "vr",                 "%.15f",    msg->vr);
    textread_stop (tr);
}

void
perllcm_position_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                                const perllcm_position_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",              "%"PRId64,  msg->utime);    
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,1)",        "%.15f",    msg->xyzrph[0]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,2)",        "%.15f",    msg->xyzrph[1]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,3)",        "%.15f",    msg->xyzrph[2]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,4)",        "%.15f",    msg->xyzrph[3]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,5)",        "%.15f",    msg->xyzrph[4]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph(:,6)",        "%.15f",    msg->xyzrph[5]);
    
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,1)",        "%.15f",    msg->xyzrph_cov[0]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,2)",        "%.15f",    msg->xyzrph_cov[1]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,3)",        "%.15f",    msg->xyzrph_cov[2]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,4)",        "%.15f",    msg->xyzrph_cov[3]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,5)",        "%.15f",    msg->xyzrph_cov[4]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,6)",        "%.15f",    msg->xyzrph_cov[5]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,7)",        "%.15f",    msg->xyzrph_cov[6]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,8)",        "%.15f",    msg->xyzrph_cov[7]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,9)",        "%.15f",    msg->xyzrph_cov[8]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,10)",        "%.15f",    msg->xyzrph_cov[9]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,11)",        "%.15f",    msg->xyzrph_cov[10]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,12)",        "%.15f",    msg->xyzrph_cov[11]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,13)",        "%.15f",    msg->xyzrph_cov[12]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,14)",        "%.15f",    msg->xyzrph_cov[13]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,15)",        "%.15f",    msg->xyzrph_cov[14]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,16)",        "%.15f",    msg->xyzrph_cov[15]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,17)",        "%.15f",    msg->xyzrph_cov[16]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,18)",        "%.15f",    msg->xyzrph_cov[17]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,19)",        "%.15f",    msg->xyzrph_cov[18]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,20)",        "%.15f",    msg->xyzrph_cov[19]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,21)",        "%.15f",    msg->xyzrph_cov[20]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,22)",        "%.15f",    msg->xyzrph_cov[21]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,23)",        "%.15f",    msg->xyzrph_cov[22]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,24)",        "%.15f",    msg->xyzrph_cov[23]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,25)",        "%.15f",    msg->xyzrph_cov[24]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,26)",        "%.15f",    msg->xyzrph_cov[25]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,27)",        "%.15f",    msg->xyzrph_cov[26]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,28)",        "%.15f",    msg->xyzrph_cov[27]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,29)",        "%.15f",    msg->xyzrph_cov[28]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,30)",        "%.15f",    msg->xyzrph_cov[29]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,31)",        "%.15f",    msg->xyzrph_cov[30]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,32)",        "%.15f",    msg->xyzrph_cov[31]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,33)",        "%.15f",    msg->xyzrph_cov[32]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,34)",        "%.15f",    msg->xyzrph_cov[33]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,35)",        "%.15f",    msg->xyzrph_cov[34]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_cov(:,36)",        "%.15f",    msg->xyzrph_cov[35]);
    
    TEXTREAD_ADD_FIELD (tr, "xyzrph_dot(:,1)",        "%.15f",    msg->xyzrph_dot[0]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_dot(:,2)",        "%.15f",    msg->xyzrph_dot[1]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_dot(:,3)",        "%.15f",    msg->xyzrph_dot[2]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_dot(:,4)",        "%.15f",    msg->xyzrph_dot[3]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_dot(:,5)",        "%.15f",    msg->xyzrph_dot[4]);
    TEXTREAD_ADD_FIELD (tr, "xyzrph_dot(:,6)",        "%.15f",    msg->xyzrph_dot[5]);
    textread_stop (tr);
}

void
perllcm_van_vlink_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const perllcm_van_vlink_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime_i",       "%"PRId64,  msg->utime_i);
    TEXTREAD_ADD_FIELD (tr, "utime_j",       "%"PRId64,  msg->utime_j);

    TEXTREAD_ADD_FIELD (tr, "z(:,1)",  "%.15f",      msg->z[0]);
    TEXTREAD_ADD_FIELD (tr, "z(:,2)",  "%.15f",      msg->z[1]);
    TEXTREAD_ADD_FIELD (tr, "z(:,3)",  "%.15f",      msg->z[2]);
    TEXTREAD_ADD_FIELD (tr, "z(:,4)",  "%.15f",      msg->z[3]);
    TEXTREAD_ADD_FIELD (tr, "z(:,5)",  "%.15f",      msg->z[4]);

    TEXTREAD_ADD_FIELD (tr, "R(:,1)",  "%.15f",      msg->R[0]);
    TEXTREAD_ADD_FIELD (tr, "R(:,2)",  "%.15f",      msg->R[1]);
    TEXTREAD_ADD_FIELD (tr, "R(:,3)",  "%.15f",      msg->R[2]);
    TEXTREAD_ADD_FIELD (tr, "R(:,4)",  "%.15f",      msg->R[3]);
    TEXTREAD_ADD_FIELD (tr, "R(:,5)",  "%.15f",      msg->R[4]);
    TEXTREAD_ADD_FIELD (tr, "R(:,6)",  "%.15f",      msg->R[5]);
    TEXTREAD_ADD_FIELD (tr, "R(:,7)",  "%.15f",      msg->R[6]);
    TEXTREAD_ADD_FIELD (tr, "R(:,8)",  "%.15f",      msg->R[7]);
    TEXTREAD_ADD_FIELD (tr, "R(:,9)",  "%.15f",      msg->R[8]);
    TEXTREAD_ADD_FIELD (tr, "R(:,10)",  "%.15f",      msg->R[9]);
    TEXTREAD_ADD_FIELD (tr, "R(:,11)",  "%.15f",      msg->R[10]);
    TEXTREAD_ADD_FIELD (tr, "R(:,12)",  "%.15f",      msg->R[11]);
    TEXTREAD_ADD_FIELD (tr, "R(:,13)",  "%.15f",      msg->R[12]);
    TEXTREAD_ADD_FIELD (tr, "R(:,14)",  "%.15f",      msg->R[13]);
    TEXTREAD_ADD_FIELD (tr, "R(:,15)",  "%.15f",      msg->R[14]);
    TEXTREAD_ADD_FIELD (tr, "R(:,16)",  "%.15f",      msg->R[15]);
    TEXTREAD_ADD_FIELD (tr, "R(:,17)",  "%.15f",      msg->R[16]);
    TEXTREAD_ADD_FIELD (tr, "R(:,18)",  "%.15f",      msg->R[17]);
    TEXTREAD_ADD_FIELD (tr, "R(:,19)",  "%.15f",      msg->R[18]);
    TEXTREAD_ADD_FIELD (tr, "R(:,20)",  "%.15f",      msg->R[19]);
    TEXTREAD_ADD_FIELD (tr, "R(:,21)",  "%.15f",      msg->R[20]);
    TEXTREAD_ADD_FIELD (tr, "R(:,22)",  "%.15f",      msg->R[21]);
    TEXTREAD_ADD_FIELD (tr, "R(:,23)",  "%.15f",      msg->R[22]);
    TEXTREAD_ADD_FIELD (tr, "R(:,24)",  "%.15f",      msg->R[23]);
    TEXTREAD_ADD_FIELD (tr, "R(:,25)",  "%.15f",      msg->R[24]);
    textread_stop (tr);
}

void
perllcm_van_plink_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const perllcm_van_plink_t *msg, void *user)
{

    /* int64_t    utime_i;
    int64_t    utime_j;
    int8_t     prior;
    perllcm_pose3d_t x_ji;
    int32_t    link_id;
    double     Ig;
    double     S_L;*/

    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime_i",       "%"PRId64,  msg->utime_i);
    TEXTREAD_ADD_FIELD (tr, "utime_j",       "%"PRId64,  msg->utime_j);
    TEXTREAD_ADD_FIELD (tr, "Ig",        "%.15f",        msg->Ig);
    TEXTREAD_ADD_FIELD (tr, "S_L",        "%.15f",        msg->S_L);
    
    textread_stop (tr);
}
