#include <stdio.h>
#include <stdlib.h>

// external linking req'd
#include <math.h>
#include <glib.h>

#include "perls-common/bot_util.h"
#include "perls-common/units.h"

#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"

#include "renderers.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RENDERER_NAME "DVL Beams"

typedef struct _RendererDVLBeams RendererDVLBeams;
struct _RendererDVLBeams {
    BotRenderer renderer;
    perllcm_position_t *pose;
    
    lcm_t *lcm;
    BotParam *param;
    
    double dvl_range[4];
    int have_dvl;
    
    BotViewer         *viewer;
    BotGtkParamWidget *pw;   
};


static void
rdi_cb (const lcm_recv_buf_t *rbuf, const char *channel,
        const senlcm_rdi_pd4_t *msg, void *user)
{    
    RendererDVLBeams *self = (RendererDVLBeams*) user;
    
    self->dvl_range[0] = msg->range[0];
    self->dvl_range[1] = msg->range[1];
    self->dvl_range[2] = msg->range[2];
    self->dvl_range[3] = msg->range[3];
    
    self->have_dvl = 1;
}

static void
_renderer_free (BotRenderer *super)
{
    RendererDVLBeams *self = (RendererDVLBeams*) super->user;
    free (self);
}

static void 
dvl_beams_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererDVLBeams *self = (RendererDVLBeams*) super->user;
    if (!self->have_dvl)
        return;
    
    glEnable (GL_DEPTH_TEST);

    //glPushMatrix ();
    
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0, 1.0, 0.0, 0.0);

    double dx = self->pose->xyzrph[0];
    double dy = self->pose->xyzrph[1];
    double dz = self->pose->xyzrph[2];
    glTranslated (dx, dy, dz);
    
    double rot_x = self->pose->xyzrph[3] * RTOD;
    double rot_y = self->pose->xyzrph[4] * RTOD;
    double rot_z = self->pose->xyzrph[5] * RTOD;
    glRotatef (rot_z, 0, 0, 1);
    glRotatef (rot_y, 0, 1, 0);
    glRotatef (rot_x, 1, 0, 0);
    
    // test just render as a single cone
    double height = (self->dvl_range[0] + self->dvl_range[1] +
                     self->dvl_range[2] +self->dvl_range[3])/4;
    
    GLUquadricObj *quadObj = gluNewQuadric ();
    glColor4f (0.0, 1.0, 1.0, 0.5);
    gluCylinder (quadObj, 0, 2*height*sin(0.8), height, 20, 20);
    
    //glPopMatrix ();
}

//static void
//on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
//{
//    RendererDVLBeams *self = (RendererDVLBeams*) user;
//    bot_viewer_request_redraw (self->viewer);
//}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererDVLBeams *self = (RendererDVLBeams*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererDVLBeams *self = (RendererDVLBeams*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererDVLBeams *
new_renderer_dvl_beams (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data)
{    
    RendererDVLBeams *self = (RendererDVLBeams*) calloc (1, sizeof (*self));

    // load from config file
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    snprintf (key, sizeof key, "%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);

    snprintf (key, sizeof key, "%s.channel", rootkey);
    char *lcm_channel = bot_param_get_str_or_fail (self->param, key);

    BotRenderer *renderer = &self->renderer;

    renderer->draw = dvl_beams_renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = NULL; //bot_gtk_param_widget_new();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    self->pose = pose_data->pose;
    self->viewer = viewer;
    
    //self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);
    //gtk_widget_show_all(renderer->widget);
    //g_signal_connect (G_OBJECT (self->pw), "changed", 
    //                  G_CALLBACK (on_param_widget_changed), self);

    self->lcm = bot_lcm_get_global (NULL);
    senlcm_rdi_pd4_t_subscribe (self->lcm, lcm_channel, &rdi_cb, self);
    
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
            G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
            G_CALLBACK (on_save_preferences), self);
    
    return self;
}

void
setup_renderer_dvl_beams (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data, int priority)
{
    RendererDVLBeams *self = new_renderer_dvl_beams (viewer, rootkey, pose_data);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
