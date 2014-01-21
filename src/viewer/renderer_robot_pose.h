#ifndef __VIEWER_RENDERERS_ROBOT_POSE_H__
#define __VIEWER_RENDERERS_ROBOT_POSE_H__

#include <stdint.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_vis/bot_vis.h>

#include <GL/glu.h>
#include <GL/gl.h>

#include <glib.h>
#include <gtk/gtk.h>

#include "perls-lcmtypes/perllcm_position_t.h"


typedef struct _robot_pose_data_t robot_pose_data_t;
struct _robot_pose_data_t {
    perllcm_position_t *pose;
    double *x_wl;
    int x_wl_adjust;
};

typedef struct _RendererRobotPose RendererRobotPose;
struct _RendererRobotPose {
    BotRenderer        renderer;
    BotViewer          *viewer;
    BotGtkParamWidget  *pw;

    lcm_t    *lcm;
    BotParam *param;

    // model generic
    BotTrans model_btrans;
    double   model_scale;
    double   model_max_dim;

    // wavefront model
    BotWavefrontModel *wavefront_model;
    int                wavefront_display_lists_ready;
    GLuint             wavefront_dl;
    
    // rwx model
    BotRwxModel *rwx_model;
    int          rwx_display_lists_ready;
    GLuint       rwx_dl;
    
    //struct containing gps origin used to convert between gps and local xy
    BotGPSLinearize *llxy;
    double org_alt;
    uint8_t use_alt;
    double meters_per_grid;

    // historic poses
    int64_t  pose_age;
    int64_t  pose_age_timer;
    int64_t  pose_utime_last; // time when we last added a pose
    GList   *pose_history;
    int      pose_history_len;
    
    // current pose
    robot_pose_data_t pose_data;
    int x_wl_adjust;
    double x_wl[6];
    perllcm_position_t pose;

    int show_z_scale;

    // current statuses
    double altitude;
    int    next_waypoint;   
    double dist_to_next_waypoint;
    double batt_percent;
    int    abort;
    int    error;
    double speed;
    
    double base_color[3];

    char line_type;
    char rootkey[256];
    GtkWidget *status_txt;
    
    // MISC (where data sources need additional state items)
    int modem_id;
};

void
update_history (int64_t utime, RendererRobotPose *self);

#endif // __VIEWER_RENDERERS_ROBOT_POSE_H__
