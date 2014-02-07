#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glib.h>
#include <gdk/gdkkeysyms.h>
#include <GL/gl.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/units.h"
#include "perls-math/ssc.h"


#include "pose_data_sources.h"
#include "renderers.h"

#define DTOR UNITS_DEGREE_TO_RADIAN
#define RTOD UNITS_RADIAN_TO_DEGREE
#define PARAM_X_WX_X "x_wl_x"
#define PARAM_X_WX_Y "x_wl_y"
#define PARAM_X_WX_Z "x_wl_z"
#define PARAM_X_WX_R "x_wl_r"
#define PARAM_X_WX_P "x_wl_p"
#define PARAM_X_WX_H "x_wl_h"

#define RENDERER_NAME "Planar Target"

typedef struct _RendererPlanarTarget RendererPlanarTarget;
struct _RendererPlanarTarget {
    BotRenderer renderer;
    perllcm_position_t *pose;
    perllcm_position_t *poseTarget;
    double x_vs[6];
    lcm_t *lcm;
    BotParam *param;
    BotViewer         *viewer;
    BotGtkParamWidget *pw;
};



static void
perllcm_position_t_cb (const lcm_recv_buf_t *rbuf, const char *channel, const perllcm_position_t *msg, void *user)
{    
    RendererPlanarTarget *self = (RendererPlanarTarget*) user;
    
    //Puts XYZRPH data of Targets location into pose
    if (self->poseTarget)
        perllcm_position_t_destroy (self->poseTarget);
    self->poseTarget = perllcm_position_t_copy (msg);
    
    bot_viewer_request_redraw (self->viewer);
}


static void
_renderer_free (BotRenderer *super)
{
    RendererPlanarTarget *self = (RendererPlanarTarget*) super->user;
    free (self);
}

static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{

    RendererPlanarTarget *self = (RendererPlanarTarget*) super->user;
    
    glEnable (GL_DEPTH_TEST);

    //glPushMatrix ();
    
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0, 1.0, 0.0, 0.0);

    //Initailizes Variables for the location of Robot
    double dx = self->pose->xyzrph[0];
    double dy = self->pose->xyzrph[1];
    double dz = self->pose->xyzrph[2];
    double rot_x = self->pose->xyzrph[3] * RTOD;
    double rot_y = self->pose->xyzrph[4] * RTOD;
    double rot_z = self->pose->xyzrph[5] * RTOD;
    
    //Moves/Rotates Coordinate system of robot
    glTranslated (dx, dy, dz);
    glRotatef (rot_z, 0, 0, 1);
    glRotatef (rot_y, 1, 1, 0);
    glRotatef (rot_x, 1, 0, 0);
   
    //size of target
    double width = .5;
    double height = .25;
    
    /*
    //For using the widgets on the toolbar /////////////////////////////////////////
    dx = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_X);
    dy = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_Y);
    dz = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_Z);
    rot_x = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_R);
    rot_y = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_P);
    rot_z = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_H);
    /////////////////////////////////////////////////////////////////////////////////
    */
    
    // move into camera frame
    double tmp[6] = {0,0,-.1, M_PI/2.0 ,0 ,M_PI/2.0}; // forward camera
    //double tmp[6] = {0,0,0,0,0,M_PI/2}; // downlooking camera
    memcpy (self->x_vs, tmp, 6*sizeof (double));
    
    dx = self->x_vs[0];
    dy = self->x_vs[1];
    dz = self->x_vs[2];
    rot_x = self->x_vs[3]*RTOD;
    rot_y = self->x_vs[4]*RTOD;
    rot_z = self->x_vs[5]*RTOD;
    //Move to frame of reference of Target
    glTranslated (dx, dy, dz);
    glRotatef (rot_z, 0, 0, 1);
    glRotatef (rot_y, 0, 1, 0);
    glRotatef (rot_x, 1, 0, 0);
    
    draw_axis (1);

    if (self->poseTarget) {
    
        double inv_pose_Target[6] = {0};
    
        ssc_inverse (inv_pose_Target, NULL, self->poseTarget->xyzrph);
    
        dx = inv_pose_Target[0];
        dy = inv_pose_Target[1];
        dz = inv_pose_Target[2];
        rot_x = inv_pose_Target[3]*RTOD;
        rot_y = inv_pose_Target[4]*RTOD;
        rot_z = inv_pose_Target[5]*RTOD;
        //Move to frame of reference of Target
        glTranslated (dx, dy, dz);
        glRotatef (rot_z, 0, 0, 1);
        glRotatef (rot_y, 0, 1, 0);
        glRotatef (rot_x, 1, 0, 0);
        
        //dx = self->poseTarget->xyzrph[0];
        //dy = self->poseTarget->xyzrph[1];
        //dz = self->poseTarget->xyzrph[2];
        //rot_x = self->poseTarget->xyzrph[3]*RTOD;
        //rot_y = self->poseTarget->xyzrph[4]*RTOD;
        //rot_z = self->poseTarget->xyzrph[5]*RTOD;
        ////Move to frame of reference of Target
        //glTranslated (dx, dy, dz);
        //glRotatef (rot_z, 0, 0, 1);
        //glRotatef (rot_y, 0, 1, 0);
        //glRotatef (rot_x, 1, 0, 0);
    }
    
    draw_axis (10);
    
    //Plots Target
    glBegin(GL_QUADS);
        glColor3f(0,0,1.0);
        glVertex3f(0, 0, 0); glVertex3f(width, 0, 0); glVertex3f(width, height, 0); glVertex3f(0, height, 0);
    glEnd();
}

//static void
//on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
//{
//    RendererPlanarTarget *self = (RendererPlanarTarget*) user;
//    bot_viewer_request_redraw (self->viewer);
//}
//static void
//on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
//{
//    RendererPlanarTarget *self = (RendererPlanarTarget*) user_data;
//    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
//}
//
//static void
//on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
//{
//    RendererPlanarTarget *self = (RendererPlanarTarget*) user_data;
//    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
//}

static RendererPlanarTarget *
new_renderer_planar_target (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data)
{    
    RendererPlanarTarget *self = (RendererPlanarTarget*) calloc (1, sizeof (*self));
    // load from config file
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    snprintf (key, sizeof key, "%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);

    snprintf (key, sizeof key, "%s.channel", rootkey);
    char *lcm_channel = bot_param_get_str_or_fail (self->param, key);

    BotRenderer *renderer = &self->renderer;

    renderer->draw = _renderer_draw;
    renderer->destroy = _renderer_free;
    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    printf ("%s \n", renderer->name);

    self->pose = pose_data->pose;
    self->viewer = viewer;

    self->poseTarget = NULL;
   

 
    //Subscribe to lcm channel
    self->lcm = bot_lcm_get_global (NULL);
    perllcm_position_t_subscribe(self->lcm, lcm_channel, &perllcm_position_t_cb, self);
    
    
    
    //g_signal_connect (G_OBJECT (viewer), "load-preferences", 
    //    G_CALLBACK (on_load_preferences), self);
    //g_signal_connect (G_OBJECT (viewer), "save-preferences",
    //    G_CALLBACK (on_save_preferences), self);
    
    return self;
}

void
setup_renderer_planar_target (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data, int priority)
{
    RendererPlanarTarget *self = new_renderer_planar_target (viewer, rootkey, pose_data);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
