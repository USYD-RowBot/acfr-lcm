#include <stdio.h>
#include <stdlib.h>

#include "perls-lcmtypes/bot_core_planar_lidar_t.h"
#include "perls-lcmtypes/perllcm_position_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/units.h"

#include "perls-math/fasttrig.h"

#include "renderers.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RENDERER_NAME "Planar Laser"

#define PARAM_ENABLE_DISPLAY "Enable Display"
#define PARAM_HISTORY_LENGTH "Scan Hist. Len."
#define PARAM_HISTORY_SPACING "Scan Hist. Spc."
#define HIST_SPACE_TO_USEC (1e5)

#define LASER_DATA_CIRC_SIZE 20
#define POSE_DATA_CIRC_SIZE 5000 

typedef struct _RendererPlanarLaser RendererPlanarLaser;
struct _RendererPlanarLaser
{
    BotRenderer renderer;
    perllcm_position_t *pose;
    
    double x_vs[6];
    double color[3];
    
    lcm_t *lcm;
    BotParam *param;
    
    BotPtrCircular   *laser_data_circ;
    int64_t           last_laser_utime;
    BotPtrCircular   *pose_data_circ;
    int64_t           last_pose_utime;
    
    double            theta_offset;
    
    BotViewer         *viewer;
    BotGtkParamWidget *pw;   
};

void
circ_free_laser_data(void *user, void *p)
{       
    bot_core_planar_lidar_t *pl = (bot_core_planar_lidar_t*) p;
    bot_core_planar_lidar_t_destroy (pl);
}

void
circ_free_position_data(void *user, void *p)
{
    perllcm_position_t *pose = (perllcm_position_t*) p;
    perllcm_position_t_destroy (pose);
}


static void
bot_core_planar_lidar_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                          const bot_core_planar_lidar_t *msg, void *user)
{    
    RendererPlanarLaser *self = (RendererPlanarLaser*) user;
    
    int hist_spc = bot_gtk_param_widget_get_int (self->pw, PARAM_HISTORY_SPACING);
    
    // if enough time has elapsed since the last scan push it onto the circular buffer
    if (!hist_spc || abs (msg->utime - self->last_laser_utime) > (int64_t)(hist_spc*HIST_SPACE_TO_USEC)) {
        bot_ptr_circular_add (self->laser_data_circ, bot_core_planar_lidar_t_copy (msg));
        self->last_laser_utime = msg->utime;
    }
    
    if (0 == bot_ptr_circular_size(self->pose_data_circ)
        || self->pose->utime != self->last_pose_utime) {
        bot_ptr_circular_add (self->pose_data_circ, perllcm_position_t_copy (self->pose));
        self->last_pose_utime = self->pose->utime;
    }
    
}

static void
_renderer_free (BotRenderer *super)
{
    RendererPlanarLaser *self = (RendererPlanarLaser*) super->user;
    free (self);
}

int
find_closest_laser_pose (RendererPlanarLaser *self, bot_core_planar_lidar_t *pl)
{    
    int idx_closest = 0; 
    int64_t utime_dist_min = 1e6; // one second
    int len = bot_ptr_circular_size (self->pose_data_circ);

    if (0 == len)
        return -1;

    for (int cidx = 0; cidx < len; cidx++) {
        perllcm_position_t *pose_data = (perllcm_position_t*) bot_ptr_circular_index (self->pose_data_circ, cidx);
        int64_t utime_dist = abs (pose_data->utime - pl->utime);
        if (utime_dist < utime_dist_min) {
            utime_dist_min = utime_dist;
            idx_closest = cidx;
        }        
    }
    
    if (utime_dist_min == (int64_t) 1e6) {
        ERROR ("ERROR: No poses within one second for planar laser data!");
        return -1;
    }
    else
        return idx_closest;
    
    return -1;
}

static void 
planar_laser_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererPlanarLaser *self = (RendererPlanarLaser*) super->user;
    
    if (!bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_DISPLAY))
        return;
    
    glEnable (GL_DEPTH_TEST);
    
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0, 1.0, 0.0, 0.0);
    
    // draw laser points
    unsigned int hist_len = bot_gtk_param_widget_get_int (self->pw, PARAM_HISTORY_LENGTH);
    
    for (unsigned int cidx = 0; cidx < bot_ptr_circular_size (self->laser_data_circ) && cidx < hist_len; cidx++) {
        
        bot_core_planar_lidar_t *pl = (bot_core_planar_lidar_t*) bot_ptr_circular_index(self->laser_data_circ, cidx);
    
        glPushMatrix ();
    
        double dx, dy, dz;
        double rot_x, rot_y, rot_z;

        // find pose with utime closest to this laser return collection
        int idx_closest = find_closest_laser_pose (self, pl);
        if (-1 != idx_closest) {
            perllcm_position_t *pose = (perllcm_position_t*) bot_ptr_circular_index (self->pose_data_circ, idx_closest);
            
            // move into robot frame
            dx = pose->xyzrph[0];
            dy = pose->xyzrph[1];
            dz = pose->xyzrph[2];
            glTranslated (dx, dy, dz);
            rot_x = pose->xyzrph[3] * RTOD;
            rot_y = pose->xyzrph[4] * RTOD;
            rot_z = pose->xyzrph[5] * RTOD;
            glRotatef (rot_z, 0, 0, 1);
            glRotatef (rot_y, 0, 1, 0);
            glRotatef (rot_x, 1, 0, 0);
        }
        
        // move into sensor frame
        dx = self->x_vs[0];
        dy = self->x_vs[1];
        dz = self->x_vs[2];
        glTranslated (dx, dy, dz);
        rot_x = self->x_vs[3] * RTOD;
        rot_y = self->x_vs[4] * RTOD;
        rot_z = self->x_vs[5] * RTOD;
        glRotatef (rot_z, 0, 0, 1);
        glRotatef (rot_y, 0, 1, 0);
        glRotatef (rot_x, 1, 0, 0);
        
        //draw_axis (3);
        
        glColor3d (self->color[0], self->color[1], self->color[2]);
        glPointSize(5);
        glBegin (GL_POINTS);
        
        double theta = pl->rad0 - self->theta_offset;
        for (int i=0; i < pl->nranges; i++) {
            double st, ct;
            fsincos (theta, &st, &ct);
            glVertex3d(pl->ranges[i]*ct, pl->ranges[i]*st, 0);
            
            theta += pl->radstep;
        }
        glEnd (); 
        glPopMatrix();
    }   
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererPlanarLaser *self = (RendererPlanarLaser*) user;
    bot_viewer_request_redraw (self->viewer);
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPlanarLaser *self = (RendererPlanarLaser*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPlanarLaser *self = (RendererPlanarLaser*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererPlanarLaser *
new_renderer_planar_laser (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data)
{    
    RendererPlanarLaser *self = (RendererPlanarLaser*) calloc (1, sizeof (*self));

    // load from config file
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    snprintf (key, sizeof key, "%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);

    snprintf (key, sizeof key, "%s.channel", rootkey);
    char *lcm_channel = bot_param_get_str_or_fail (self->param, key);
    
    self->color[0] = 0.0;
    self->color[1] = 1.0;
    self->color[2] = 0.0;
    snprintf (key, sizeof key, "%s.color", rootkey);
    bot_param_get_double_array (self->param, key, self->color, 3);
    
    memset (self->x_vs, 0, 6*sizeof (double));
    snprintf (key, sizeof key, "%s.sensor_key", rootkey);
    char *sensor_key;
    if (!bot_param_get_str (self->param, key, &sensor_key)) {
        snprintf (key, sizeof key, "sensors.%s.x_vs", sensor_key);
        if (6 == bot_param_get_double_array (self->param, key, self->x_vs, 6) ) {
            self->x_vs[3] =  self->x_vs[3] * DTOR;
            self->x_vs[4] =  self->x_vs[4] * DTOR;
            self->x_vs[5] =  self->x_vs[5] * DTOR;
        }
        free (sensor_key);
    }
    
    self->theta_offset = 0;
    snprintf (key, sizeof key, "%s.theta_offset", rootkey);
    bot_param_get_double (self->param, key, &self->theta_offset);
    self->theta_offset *= DTOR;

    BotRenderer *renderer = &self->renderer;

    renderer->draw = planar_laser_renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new ();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    self->pose = pose_data->pose;
    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);
    
    
    self->laser_data_circ = bot_ptr_circular_new (LASER_DATA_CIRC_SIZE,
                                                  circ_free_laser_data, self);
    self->pose_data_circ = bot_ptr_circular_new (POSE_DATA_CIRC_SIZE,
                                                 circ_free_position_data, self);
    
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE_DISPLAY, 1, NULL);
    
    bot_gtk_param_widget_add_int (self->pw, PARAM_HISTORY_LENGTH, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 0,
                                  LASER_DATA_CIRC_SIZE, 1,
                                  1);


    bot_gtk_param_widget_add_int (self->pw, PARAM_HISTORY_SPACING, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 0,
                                  50, 1,
                                  1);
    
    gtk_widget_show_all (renderer->widget);
    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);

    self->lcm = bot_lcm_get_global (NULL);
    bot_core_planar_lidar_t_subscribe (self->lcm, lcm_channel, &bot_core_planar_lidar_t_cb, self);
    
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);
    
    return self;
}

void
setup_renderer_planar_laser (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data, int priority)
{
    RendererPlanarLaser *self = new_renderer_planar_laser (viewer, rootkey, pose_data);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
