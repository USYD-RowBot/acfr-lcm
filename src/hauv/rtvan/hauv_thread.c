#include <stdio.h>
#include <stdlib.h>

// external linking req'd
#include <glib.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/hauv_bs_cnv_t.h"
#include "perls-lcmtypes/hauv_bs_dvl_t.h"
#include "perls-lcmtypes/hauv_bs_pit_t.h"
#include "perls-lcmtypes/hauv_bs_rnv_2_t.h"

#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_rdi_bathy_t.h"
#include "perls-lcmtypes/perllcm_van_rdi_bathy_collection_t.h"

#include "perls-lcmtypes/perllcm_isam_cmd_t.h"

#include "perls-common/error.h"
#include "perls-common/lcm_util.h"
#include "perls-common/units.h"

#include "perls-math/ssc.h"

#include "perls-sensors/rdi.h"

#include "shared_memory.h"
#include "hauv_thread.h"
#include "van_util.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define SARATOGA 0

#define printf(format, ...)                             \
    printf ("%-12s " format, "[hauv]", ## __VA_ARGS__)

typedef struct hauv hauv_t;
struct hauv
{
    perllcm_pose3d_t x_lv;  // vehicle w.r.t. local-level (dynamic)

    perllcm_pose3d_t x_vs1; // servo1 (pitch_dvl) w.r.t. vehicle (dynamic)
    perllcm_pose3d_t x_vs2; // servo2 (pitch_sonar) w.r.t. vehicle (dynamic)

    perllcm_pose3d_t x_s1c; // camera w.r.t. servo1  (static)
    perllcm_pose3d_t x_s1d; // dvl w.r.t. servo1     (static)

    hauv_bs_cnv_t   bs_cnv;
    hauv_bs_dvl_t   bs_dvl;
    hauv_bs_pit_t   bs_pit;
    hauv_bs_rnv_2_t bs_rnv_2;

    /* bool periscope_mode; */
    perllcm_pose3d_t x_vcUw;  // vehicle to underwater camera
    perllcm_pose3d_t x_vcPeri;  // vehicle to periscope camera

    // to post process mosaic with dvl points
    GSList         *dvlpts_list;
    bool            save_dvlpts;
};

static void
glist_destroyer (gpointer data, gpointer user)
{
    free (data);
}

static perllcm_pose3d_t
get_vehicle_pose (const hauv_t *hauv)
{
    return hauv->x_lv;
}

static perllcm_pose3d_t
get_servo1_pose (const hauv_t *hauv)
{
    // x_ls1 = x_lv + x_vs1
    perllcm_pose3d_t x_ls1 = {0};
    x_ls1.utime = hauv->x_lv.utime;
    ssc_head2tail (x_ls1.mu, NULL, hauv->x_lv.mu, hauv->x_vs1.mu);
    return x_ls1;
}

static perllcm_pose3d_t
get_servo2_pose (const hauv_t *hauv)
{
    // x_ls2 = x_lv + x_vs2
    perllcm_pose3d_t x_ls2 = {0};
    x_ls2.utime = hauv->x_lv.utime;
    ssc_head2tail (x_ls2.mu, NULL, hauv->x_lv.mu, hauv->x_vs2.mu);
    return x_ls2;
}

static perllcm_pose3d_t
get_camera_pose_uw (const hauv_t *hauv)
{
    perllcm_pose3d_t x_lc = {0};

    // x_lc = x_ls1 + x_s1c
    perllcm_pose3d_t x_ls1 = get_servo1_pose (hauv);
    x_lc.utime = x_ls1.utime;
    ssc_head2tail (x_lc.mu, NULL, x_ls1.mu, hauv->x_s1c.mu);

    return x_lc;
}

static perllcm_pose3d_t
get_camera_pose_peri (const hauv_t *hauv)
{
    perllcm_pose3d_t x_lc = {0};

    x_lc.utime = hauv->x_lv.utime;
    ssc_head2tail (x_lc.mu, NULL, hauv->x_lv.mu, hauv->x_vcPeri.mu);

    return x_lc;
}

static perllcm_pose3d_t
get_camera_pose_saratoga (const hauv_t *hauv)
{
    perllcm_pose3d_t x_lc = {0};

    // x_lc = x_ls1 + x_s1c
    perllcm_pose3d_t x_ls2 = get_servo2_pose (hauv);
    x_lc.utime = x_ls2.utime;
    ssc_head2tail (x_lc.mu, NULL, x_ls2.mu, hauv->x_s1c.mu);

    return x_lc;
}

static perllcm_pose3d_t
get_dvl_pose (const hauv_t *hauv)
{
    // x_ld = x_ls1 + x_s1d
    perllcm_pose3d_t x_ld = {0}, x_ls1 = get_servo1_pose (hauv);
    x_ld.utime = x_ls1.utime;
    ssc_head2tail (x_ld.mu, NULL, x_ls1.mu, hauv->x_s1d.mu);
    return x_ld;
}

static void
hauv_bs_cnv_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const hauv_bs_cnv_t *msg, void *user)
{
    hauv_t *hauv = user;
    hauv->bs_cnv = *msg;

    // full state of x_lv is comprised of CNV and RNV_2 message
    if (hauv->bs_cnv.time_nav == hauv->bs_rnv_2.time_nav)
    {
        hauv->x_lv.utime = msg->time_received;
        hauv->x_lv.mu[SSC_DOF_X] = hauv->bs_cnv.x;
        hauv->x_lv.mu[SSC_DOF_Y] = hauv->bs_cnv.y;
        hauv->x_lv.mu[SSC_DOF_Z] = hauv->bs_cnv.z;
        hauv->x_lv.mu[SSC_DOF_R] = hauv->bs_rnv_2.absroll;
        hauv->x_lv.mu[SSC_DOF_P] = hauv->bs_rnv_2.abspitch;
        hauv->x_lv.mu[SSC_DOF_H] = hauv->bs_rnv_2.absheading;

        // publish camera pose

#if SARATOGA
        perllcm_pose3d_t x_lcUw = get_camera_pose_saratoga (hauv);
#else
        perllcm_pose3d_t x_lcUw = get_camera_pose_uw (hauv);
#endif
        perllcm_pose3d_t_publish (shm->lcm, VAN_CAMERA_POSE_UW_CHANNEL, &x_lcUw);
        perllcm_pose3d_t x_lcPeri = get_camera_pose_peri (hauv);
        perllcm_pose3d_t_publish (shm->lcm, VAN_CAMERA_POSE_PERI_CHANNEL, &x_lcPeri);

        // publish vehicle to camera sensor xform
        perllcm_pose3d_t_publish (shm->lcm, VAN_V2C_POSE_UW_CHANNEL, &hauv->x_vcUw);
        perllcm_pose3d_t_publish (shm->lcm, VAN_V2C_POSE_PERI_CHANNEL, &hauv->x_vcPeri);
    }
}

static void
hauv_bs_rnv_2_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                          const hauv_bs_rnv_2_t *msg, void *user)
{
    hauv_t *hauv = user;
    hauv->bs_rnv_2 = *msg;

#define OCEANUS_HACK 0
#if OCEANUS_HACK
    static bool warn = 1;
    if (warn)
    {
        printf ("==============================================================\n");
        printf ("*******************OCEANUS_HACK*******************************\n");
        printf ("==============================================================\n");
        warn = 0;
    }
    hauv->bs_rnv_2.absheading +=  -270*DTOR;
#endif

    // full state of x_lv is comprised of CNV and RNV_2 message
    if (hauv->bs_cnv.time_nav == hauv->bs_rnv_2.time_nav)
    {
        hauv->x_lv.utime = msg->time_received;
        hauv->x_lv.mu[SSC_DOF_X] = hauv->bs_cnv.x;
        hauv->x_lv.mu[SSC_DOF_Y] = hauv->bs_cnv.y;
        hauv->x_lv.mu[SSC_DOF_Z] = hauv->bs_cnv.z;
        hauv->x_lv.mu[SSC_DOF_R] = hauv->bs_rnv_2.absroll;
        hauv->x_lv.mu[SSC_DOF_P] = hauv->bs_rnv_2.abspitch;
        hauv->x_lv.mu[SSC_DOF_H] = hauv->bs_rnv_2.absheading;

        // publish camera pose
        perllcm_pose3d_t x_lcUw = get_camera_pose_uw (hauv);
        perllcm_pose3d_t_publish (shm->lcm, VAN_CAMERA_POSE_UW_CHANNEL, &x_lcUw);
        perllcm_pose3d_t x_lcPeri = get_camera_pose_peri (hauv);
        perllcm_pose3d_t_publish (shm->lcm, VAN_CAMERA_POSE_PERI_CHANNEL, &x_lcPeri);

        // publish vehicle to camera sensor xform
        perllcm_pose3d_t_publish (shm->lcm, VAN_V2C_POSE_UW_CHANNEL, &hauv->x_vcUw);
        perllcm_pose3d_t_publish (shm->lcm, VAN_V2C_POSE_PERI_CHANNEL, &hauv->x_vcPeri);
    }
}

static void
hauv_bs_pit_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const hauv_bs_pit_t *msg, void *user)
{
    hauv_t *hauv = user;
    hauv->bs_pit = *msg;

    hauv->x_vs1.utime = hauv->x_vs2.utime = msg->time_received;
    hauv->x_vs1.mu[SSC_DOF_P] = msg->pitch_dvl;

    // servo2 is shifted from DVL by {0.045, 0.22,0,0,pitch,0)
    hauv->x_vs2.mu[SSC_DOF_X] = 0.045;
    hauv->x_vs2.mu[SSC_DOF_Y] = 0.22;
    hauv->x_vs2.mu[SSC_DOF_P] = msg->pitch_sonar;

    ssc_head2tail (hauv->x_vcUw.mu, NULL, hauv->x_vs1.mu, hauv->x_s1c.mu);
}

static void
hauv_bs_dvl_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const hauv_bs_dvl_t *msg, void *user)
{
    hauv_t *hauv = user;
    hauv->bs_dvl = *msg;

    /* The DVL ranges reported by the HAUV are the ranges as they come out of the
     * DVL. You need to apply the 1/cos(30) correction to get the slant ranges as
     * measured along each beam direction.
     */
    const double cos30 = cos (30*DTOR);
    perllcm_pose3d_t x_ld = get_dvl_pose (hauv);
    perllcm_rdi_bathy_t bathy_l = rdi_bathy_janus30 (msg->range1/cos30, msg->range2/cos30,
                                  msg->range3/cos30, msg->range4/cos30, x_ld.mu);
    bathy_l.utime = msg->time_received;
    perllcm_rdi_bathy_t_publish (shm->lcm, VAN_DVL_BATHY_CHANNEL, &bathy_l);

    // save dvl point for post processing
    if (hauv->save_dvlpts)
    {

        perllcm_van_rdi_bathy_collection_t *bathy_list = calloc (1, sizeof (*bathy_list));
        perllcm_pose3d_t x_vd = {};
        ssc_head2tail (x_vd.mu, NULL, hauv->x_vs1.mu, hauv->x_s1d.mu);
        perllcm_pose3d_t x_lc = get_camera_pose_uw (hauv);
        double depth = x_lc.mu[2];

        // build bathy_list with one bathy_rdi_t
        bathy_list->npts = 1;
        bathy_list->bathy_v = malloc (sizeof (*bathy_list->bathy_v));
        bathy_list->x_vc = malloc (sizeof (*bathy_list->x_vc));
        bathy_list->calib_list = malloc (sizeof (*bathy_list->calib_list));

        bathy_list->bathy_v[0] = rdi_bathy_janus30 (msg->range1/cos30, msg->range2/cos30,
                                 msg->range3/cos30, msg->range4/cos30, x_vd.mu);
        if (depth > 0.25)
        {
            bathy_list->calib_list[0] = CALIB_UW_WATER;
            bathy_list->x_vc[0] = hauv->x_vcUw;
        }
        else
        {
            bathy_list->calib_list[0] = CALIB_PERI_AIR;
            bathy_list->x_vc[0] = hauv->x_vcPeri;
        }

        hauv->dvlpts_list = g_slist_append (hauv->dvlpts_list, bathy_list);
    }
}

static void
perllcm_isam_cmd_t_cmd_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_isam_cmd_t *msg, void *user )
{
    // cmd channel comes directly from viewer (ack channel comes from seserver)
    hauv_t *hauv = user;

    // on save button from viewer
    if (msg->mode == PERLLCM_ISAM_CMD_T_MODE_SAVE && hauv->save_dvlpts)
    {
        if (msg->savepath)
        {
            perllcm_van_calib_t *calibUw_water = &shm->waterUw.calib;
            perllcm_van_calib_t *calibUw_air = &shm->airUw.calib;
            perllcm_van_calib_t *calibPeri_water = &shm->waterPeri.calib;
            perllcm_van_calib_t *calibPeri_air = &shm->airPeri.calib;
            vanu_dvlpts_save2disk (msg->savepath, hauv->dvlpts_list,
                                   calibUw_water, calibUw_air, calibPeri_water, calibPeri_air);
        }
    }
}

gpointer
hauv_thread (gpointer user)
{
    printf ("Spawning\n");

    // slam_thread state
    hauv_t *hauv = calloc (1, sizeof (*hauv));

    /* hauv->periscope_mode = 0; */
    /* botu_param_get_boolean_to_bool (shm->param, "rtvan.hauv_thread.periscope_mode", &hauv->periscope_mode); */

    hauv->save_dvlpts = 0;
    botu_param_get_boolean_to_bool (shm->param, "rtvan.post_process.save_bathy", &hauv->save_dvlpts);

    char camcfg[256];
    snprintf (camcfg, sizeof camcfg, "%s.x_s1c", shm->cameraUw_rootkey);
    if (6 != bot_param_get_double_array (shm->param, camcfg, hauv->x_s1c.mu, 6))
    {
        ERROR ("error parsing x_s1c for underwater camera");
        exit (-1);
    }
    snprintf (camcfg, sizeof camcfg, "%s.x_vs", shm->cameraPeri_rootkey);
    if (6 != bot_param_get_double_array (shm->param, camcfg, hauv->x_vcPeri.mu, 6))
    {
        ERROR ("error parsing x_vs for periscope camera");
        exit (-1);
    }

    // config file
    if (6 != bot_param_get_double_array (shm->param, "dvl.x_vs1", hauv->x_vs1.mu, 6))
    {
        ERROR ("error parsing rtvan.hauv_thread.x_vs1");
        exit (-1);
    }
    if (6 != bot_param_get_double_array (shm->param, "sonar.x_vs2", hauv->x_vs2.mu, 6))
    {
        ERROR ("error parsing rtvan.hauv_thread.x_vs2");
        exit (-1);
    }
    if (6 != bot_param_get_double_array (shm->param, "dvl.x_s1d", hauv->x_s1d.mu, 6))
    {
        ERROR ("error parsing rtvan.hauv_thread.x_s1d");
        exit (-1);
    }
    for (size_t i=3; i<6; i++)
    {
        hauv->x_vs1.mu[i] *= DTOR;
        hauv->x_vs2.mu[i] *= DTOR;
        hauv->x_s1d.mu[i] *= DTOR;
        hauv->x_s1c.mu[i] *= DTOR;
        hauv->x_vcUw.mu[i]  *= DTOR;
        hauv->x_vcPeri.mu[i]  *= DTOR;
    }

    // lcm subscriptions
    hauv_bs_cnv_t_subscription_t *hauv_bs_cnv_t_sub =
        hauv_bs_cnv_t_subscribe (shm->lcm, "HAUV_BS_CNV", &hauv_bs_cnv_t_callback, hauv);

    hauv_bs_rnv_2_t_subscription_t *hauv_bs_rnv_2_t_sub =
        hauv_bs_rnv_2_t_subscribe (shm->lcm, "HAUV_BS_RNV_2", &hauv_bs_rnv_2_t_callback, hauv);

    hauv_bs_pit_t_subscription_t *hauv_bs_pit_t_sub =
        hauv_bs_pit_t_subscribe (shm->lcm, "HAUV_BS_PIT", &hauv_bs_pit_t_callback, hauv);

    hauv_bs_dvl_t_subscription_t *hauv_bs_dvl_t_sub =
        hauv_bs_dvl_t_subscribe (shm->lcm, "HAUV_BS_DVL", &hauv_bs_dvl_t_callback, hauv);

    perllcm_isam_cmd_t_subscription_t *perllcm_isam_cmd_t_cmd_sub =
        perllcm_isam_cmd_t_subscribe (shm->lcm, SE_OPTION_CMD_CHANNEL, &perllcm_isam_cmd_t_cmd_callback, hauv);

    while (!shm->done)
    {
        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        lcmu_handle_timeout (shm->lcm, &timeout);
    }

    // clean up
    hauv_bs_cnv_t_unsubscribe (shm->lcm, hauv_bs_cnv_t_sub);
    hauv_bs_rnv_2_t_unsubscribe (shm->lcm, hauv_bs_rnv_2_t_sub);
    hauv_bs_pit_t_unsubscribe (shm->lcm, hauv_bs_pit_t_sub);
    hauv_bs_dvl_t_unsubscribe (shm->lcm, hauv_bs_dvl_t_sub);
    perllcm_isam_cmd_t_unsubscribe (shm->lcm, perllcm_isam_cmd_t_cmd_sub);
    g_slist_foreach (hauv->dvlpts_list, &glist_destroyer, NULL);
    g_slist_free (hauv->dvlpts_list);
    free (hauv);

    printf ("Exiting\n");
    g_thread_exit (0);
    return NULL;
}
