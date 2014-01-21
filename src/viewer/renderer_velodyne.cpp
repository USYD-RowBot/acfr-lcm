#include <stdio.h>
#include <stdlib.h>

#include "perls-lcmtypes/senlcm_velodyne_t.h"
#include "perls-lcmtypes/senlcm_ford_velodyne_t.h"

#include "perls-lcmtypes/perllcm_velodyne_laser_return_t.h"
#include "perls-lcmtypes/perllcm_velodyne_laser_return_collection_t.h"
#include "perls-lcmtypes/perllcm_position_t.h"

#include "perls-common/cinttypes.h"
#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-math/ssc.h"

#include "renderers.h"

#ifdef __PERLS_SENSORS__
#include "perls-sensors/velodyne.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define MAX_REFERSH_RATE_USEC 30000 // about 1/30 of a second

#define RENDERER_NAME "Velodyne"
#define VELODYNE_DATA_CIRC_SIZE 10
#define POSE_DATA_CIRC_SIZE 2500 

#define PARAM_COLOR_MENU "Color"
#define PARAM_HISTORY_LENGTH "Scan Hist. Len."
#define PARAM_HISTORY_SPACING "Scan Hist. Spc."
#define PARAM_WRITE_NEXT_SCAN "Write Next Scan"
#define HIST_SPACE_TO_USEC (1e5)
#define PARAM_SUBSAMPLE "Sub-Sample %" 
#define PARAM_POINT_SIZE "Point Size" 
#define PARAM_POINT_ALPHA "Point Alpha" 

enum {
    COLOR_Z,
    COLOR_INTENSITY,
    COLOR_NONE,
};

typedef struct _pose_data_t pose_data_t;
struct _pose_data_t {
    double pose[6];
    double motion[6];
    int64_t utime;
};

typedef struct _RendererVelodyne RendererVelodyne;
struct _RendererVelodyne
{
    BotRenderer renderer;
    perllcm_position_t *pose;
    int x_wl_adjust;
    double *x_wl;
    
    lcm_t *lcm;
    BotParam *param;
    
    int have_data;
    
    velodyne_calib_t *calib;
    velodyne_laser_return_collector_t *collector;
    
    BotPtrCircular   *velodyne_data_circ;
    int64_t 	      last_velodyne_data_utime;
    BotPtrCircular   *pose_data_circ;
    int64_t           last_pose_utime;
    
    BotViewer         *viewer;
    BotGtkParamWidget *pw;   
};

void
_velodyne_cb (RendererVelodyne *self, uint8_t *data, int datalen, int64_t utime) {
   
    int do_push_motion = 0; // only push motion data if we are starting a new collection or there is a new pose
    int hist_spc = bot_gtk_param_widget_get_int (self->pw, PARAM_HISTORY_SPACING);
    
    static int64_t last_redraw_utime = 0;
    int64_t now = timestamp_now();
       
    //perllcm_velodyne_laser_return_collection_t *lrc =
    //velodyne_decode_data_packet(self->calib, data, datalen, utime);
	
    perllcm_velodyne_laser_return_collection_t *lrc =
	velodyne_decode_data_packet_subsamp(self->calib, data, datalen, utime,
					    bot_gtk_param_widget_get_double (self->pw, PARAM_SUBSAMPLE));
	
    int ret = velodyne_collector_push_laser_returns (self->collector, lrc);
	
    velodyne_free_laser_return_collection (lrc);
    
    if (VELODYNE_COLLECTION_READY == ret) {
    
	perllcm_velodyne_laser_return_collection_t *lrc =
	    velodyne_collector_pull_collection (self->collector);
	
	
	// if enough time has elapsed since the last scan push it onto the circular buffer
	if (!hist_spc || abs (lrc->utime - self->last_velodyne_data_utime) > (int64_t)(hist_spc*HIST_SPACE_TO_USEC)) {
	    bot_ptr_circular_add (self->velodyne_data_circ, lrc);
	    self->last_velodyne_data_utime = lrc->utime;
	}
	else {
	    // memory leak city if this isnt here as soon as you increase the history spacing
	    velodyne_free_laser_return_collection (lrc);
	}
	
	if ((now - last_redraw_utime) > MAX_REFERSH_RATE_USEC) {
	    bot_viewer_request_redraw (self->viewer);
	    last_redraw_utime = now;
	}
	
	//starting a new collection
	do_push_motion = 1;
    }
    
    
    // check if we have a new pose and add it to a circular buffer
    if (0 == bot_ptr_circular_size(self->pose_data_circ)
        || self->pose->utime != self->last_pose_utime) {
        
        pose_data_t *p = (pose_data_t*) calloc (1, sizeof (*p));
        p->utime = self->pose->utime;
        p->pose[0] = self->pose->xyzrph[0];
        p->pose[1] = self->pose->xyzrph[1];
        p->pose[2] = self->pose->xyzrph[2];
        p->pose[3] = self->pose->xyzrph[3];
        p->pose[4] = self->pose->xyzrph[4];
        p->pose[5] = self->pose->xyzrph[5];
        bot_ptr_circular_add (self->pose_data_circ, p);
        
        // new pose
        do_push_motion = 1;
        
        self->last_pose_utime = self->pose->utime;
    }
    
    if (do_push_motion) {
	velodyne_collector_push_state (self->collector, *(self->pose));
	do_push_motion = 0;	
    }
    
}


static void
velodyne_cb (const lcm_recv_buf_t *rbuf, const char *channel,
             const senlcm_velodyne_t *msg, void *user)
{
    RendererVelodyne *self = (RendererVelodyne*) user;
    
    if (msg->packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) {
	
	_velodyne_cb (self, msg->data, msg->datalen, msg->utime);
    }
}

static void
velodyne_ford_cb (const lcm_recv_buf_t *rbuf, const char *channel,
             const senlcm_ford_velodyne_t *msg, void *user)
{
    RendererVelodyne *self = (RendererVelodyne*) user;
    
    _velodyne_cb (self, msg->data, msg->datalen, msg->utime);
    
}
    
void
circ_free_velodyne_data(void *user, void *p)
{       
    perllcm_velodyne_laser_return_collection_t *lrc = (perllcm_velodyne_laser_return_collection_t*) p;
    velodyne_free_laser_return_collection (lrc);
}

void
circ_free_pose_data(void *user, void *p)
{
    pose_data_t *pose = (pose_data_t*) p;
    free (pose);  
}

int
find_closest_pose (RendererVelodyne *self, perllcm_velodyne_laser_return_collection_t *lrc)
{    
    int idx_closest = 0; 
    int64_t utime_dist_min = 1e6; // one second
    int len = bot_ptr_circular_size (self->pose_data_circ);

    if (0 == len)
        return -1;
    
    if (lrc->has_pose)
        return 0;

    for (int cidx = 0; cidx < len; cidx++) {
        pose_data_t *pose_data = (pose_data_t*) bot_ptr_circular_index (self->pose_data_circ, cidx);
        int64_t utime_dist = abs (pose_data->utime - lrc->utime);
        if (utime_dist < utime_dist_min) {
            utime_dist_min = utime_dist;
            idx_closest = cidx;
        }        
    }
    
    if (utime_dist_min == (int64_t) 1e6) {
        ERROR ("ERROR: No poses within one second for velodyne data!");
        return -1;
    }
    else {
        pose_data_t *pose_data = (pose_data_t*) bot_ptr_circular_index (self->pose_data_circ, idx_closest);
        memcpy (lrc->pose, pose_data->pose, 6*sizeof (double));
        lrc->has_pose = 1;
        return idx_closest;
    }
    
    return -1;
}

static void
_renderer_free (BotRenderer *super)
{
    RendererVelodyne *self = (RendererVelodyne*) super->user;
    bot_ptr_circular_destroy (self->velodyne_data_circ);
    velodyne_calib_free (self->calib);
    velodyne_laser_return_collector_free (self->collector);
    free (self);
}

static void 
velodyne_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererVelodyne *self = (RendererVelodyne*) super->user;
    
    int colormode = bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU);
    if (colormode == COLOR_NONE)
	return;

    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0, 1.0, 0.0, 0.0);
    
    if (self->x_wl_adjust) {
        double dx = self->x_wl[0];
        double dy = self->x_wl[1];
        double dz = self->x_wl[2];
        glTranslated (dx, dy, dz);
        double rot_x = self->x_wl[3]*RTOD;
        double rot_y = self->x_wl[4]*RTOD;
        double rot_z = self->x_wl[5]*RTOD;
        glRotatef (rot_z, 0, 0, 1);
        glRotatef (rot_y, 0, 1, 0);
        glRotatef (rot_x, 1, 0, 0);
    }
    
    // draw the velodyne point cloud
    unsigned int hist_len = bot_gtk_param_widget_get_int (self->pw, PARAM_HISTORY_LENGTH);
    
    for (unsigned int cidx = 0; cidx < bot_ptr_circular_size (self->velodyne_data_circ) && cidx < hist_len; cidx++) {
    
	perllcm_velodyne_laser_return_collection_t *lrc = 
        (perllcm_velodyne_laser_return_collection_t*) bot_ptr_circular_index (self->velodyne_data_circ, cidx);
	
	// kind of a hack 
	double sp = lrc->x_vs[4]; //sensor pitch
	double cos_sp = cos(sp);
	double sin_sp = sin(sp);
        
        int write_next = 0;
        FILE *f = NULL;
        if (bot_gtk_param_widget_get_bool (self->pw, PARAM_WRITE_NEXT_SCAN) ) {
            write_next = 1;
            bot_gtk_param_widget_set_bool (self->pw, PARAM_WRITE_NEXT_SCAN, 0);
            char fname[128] = {0};
            sprintf (fname, "%"cPRId64".vel", lrc->utime);
            f = fopen (fname, "w");
        }
	
        glPushMatrix ();

        double dx, dy, dz;
        double rot_x, rot_y, rot_z;

        // find pose with utime closest to this laser return collection
        if (-1 != find_closest_pose (self, lrc)) {
            // move into robot frame
            dx = lrc->pose[0];
            dy = lrc->pose[1];
            dz = lrc->pose[2];
            glTranslated (dx, dy, dz);
            rot_x = lrc->pose[3] * RTOD;
            rot_y = lrc->pose[4] * RTOD;
            rot_z = lrc->pose[5] * RTOD;
            glRotatef (rot_z, 0, 0, 1);
            glRotatef (rot_y, 0, 1, 0);
            glRotatef (rot_x, 1, 0, 0);
        }
        
        // move into sensor frame
        dx = lrc->x_vs[0];
        dy = lrc->x_vs[1];
        dz = lrc->x_vs[2];
        glTranslated (dx, dy, dz);
        rot_x = lrc->x_vs[3] * RTOD;
        rot_y = lrc->x_vs[4] * RTOD;
        rot_z = lrc->x_vs[5] * RTOD;
        glRotatef (rot_z, 0, 0, 1);
        glRotatef (rot_y, 0, 1, 0);
        glRotatef (rot_x, 1, 0, 0);
        
        if (write_next) {
            double x_ls[6]= {0};
            ssc_head2tail (x_ls, NULL, lrc->pose, lrc->x_vs);
            fprintf (f, "%lf, %lf, %lf, %lf, %lf, %lf\n",
                     x_ls[0], x_ls[1], x_ls[2], x_ls[3], x_ls[4], x_ls[5]);
        }
        
        //draw_axis (5);
        
        glEnable (GL_DEPTH_TEST);
        glEnable (GL_BLEND);
        glPointSize (bot_gtk_param_widget_get_double(self->pw, PARAM_POINT_SIZE));
        glBegin (GL_POINTS);
        double alpha = bot_gtk_param_widget_get_double (self->pw, PARAM_POINT_ALPHA);

        for (int s = 0; s < lrc->num_lr; s++) {
            
            perllcm_velodyne_laser_return_t *lr = &(lrc->laser_returns[s]);
            
            switch (colormode)  {
               case COLOR_INTENSITY: {
                   double v = (lr->intensity)/255.0; // normalize the intensity values
                   glColor4d (v, v, v, alpha);
                   break;
               }
               case COLOR_Z: {
                   // pitch into correct frame
                   double z = sin_sp*lr->xyz[0] + cos_sp*lr->xyz[2];
                   z = z - lrc->x_vs[2];
                   double Z_MIN = 0, Z_MAX = 3;
                   double z_norm = (z - Z_MIN) / (Z_MAX - Z_MIN);
                   glColor3fv (bot_color_util_jet (z_norm));
                   break;
               }
               default:
                   break;
            }
            
            glVertex3d (lr->xyz[0],lr->xyz[1],lr->xyz[2]);
            
            if (write_next) {
                fprintf (f, "%lf, %lf, %lf\n",
                         lr->xyz[0], lr->xyz[1], lr->xyz[2]);
            }
        }
        
        if (write_next)
            fclose (f);
        
        glEnd ();
        glPopMatrix ();
    }
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererVelodyne *self = (RendererVelodyne*) user;
    bot_viewer_request_redraw (self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererVelodyne *self = (RendererVelodyne*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererVelodyne *self = (RendererVelodyne*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererVelodyne *
new_renderer_velodyne (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data)
{    
    RendererVelodyne *self = (RendererVelodyne*) calloc (1, sizeof (*self));

    // load from config file
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    snprintf (key, sizeof key, "%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);

    snprintf (key, sizeof key, "%s.channel", rootkey);
    char *lcm_channel = bot_param_get_str_or_fail (self->param, key);

    double x_vs[6];
    if (6 == bot_param_get_double_array (self->param, "sensors.velodyne.x_vs", x_vs, 6) ) {
        x_vs[3] =  x_vs[3] * DTOR;
        x_vs[4] =  x_vs[4] * DTOR;
        x_vs[5] =  x_vs[5] * DTOR;
    }

    char *velodyne_model = bot_param_get_str_or_fail (self->param, "sensors.velodyne.model");
    char *calib_file = bot_param_get_str_or_fail (self->param, "sensors.velodyne.intrinsic_calib_file");

    if (0 == strcmp (velodyne_model, VELODYNE_HDL_32E_MODEL_STR))
        self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_32E, calib_file);
    else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR))
        self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S1, calib_file);
    else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S2_MODEL_STR))
        self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S2, calib_file);    
    else
        ERROR ("ERROR: Unknown Velodyne model \'%s\'", velodyne_model);

    free (calib_file);
    
    self->collector = velodyne_laser_return_collector_create (1, 0, 0, x_vs); // full scan
    //self->collector = velodyne_laser_return_collector_create (0, 2*M_PI-0.5, 0.5, x_vs); // good for oa
    //self->collector = velodyne_laser_return_collector_create (0, M_PI/4.0, 7.0*M_PI/4.0, x_vs);
    //self->collector = velodyne_laser_return_collector_create (0, M_PI/2.0, 3.0*M_PI/2.0, x_vs);
    
    
    // print to debug calibration parsing
    // velodyne_calib_dump (self->calib);

    BotRenderer *renderer = &self->renderer;

    renderer->draw = velodyne_renderer_draw;
    renderer->destroy = _renderer_free;

    
    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    self->pose = pose_data->pose;
    self->x_wl_adjust = pose_data->x_wl_adjust;
    self->x_wl = pose_data->x_wl;
    self->viewer = viewer;
    
    self->velodyne_data_circ = bot_ptr_circular_new (VELODYNE_DATA_CIRC_SIZE,
                                                     circ_free_velodyne_data, self);
    self->pose_data_circ = bot_ptr_circular_new (POSE_DATA_CIRC_SIZE,
                                                 circ_free_pose_data, self);
    
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);
    
    gtk_widget_show_all (renderer->widget);
    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);
    
    bot_gtk_param_widget_add_enum (self->pw, PARAM_COLOR_MENU, BOT_GTK_PARAM_WIDGET_MENU, COLOR_Z,
                                   "Height", COLOR_Z,
                                   "Intensity", COLOR_INTENSITY,
                                   "Do Not Render", COLOR_NONE,
                                   NULL);
    
    bot_gtk_param_widget_add_int (self->pw, PARAM_HISTORY_LENGTH, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 0,
                                  VELODYNE_DATA_CIRC_SIZE, 1,
                                  1);


    bot_gtk_param_widget_add_int (self->pw, PARAM_HISTORY_SPACING, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 0,
                                  50, 1,
                                  1);
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_SUBSAMPLE, 
                                     BOT_GTK_PARAM_WIDGET_SLIDER, 0.01,
                                     1.0, 0.01,
                                     1.0);
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_POINT_SIZE, 
                                     BOT_GTK_PARAM_WIDGET_SLIDER, 0,
                                     10, 0.5,
                                     2.0);

    bot_gtk_param_widget_add_double (self->pw, PARAM_POINT_ALPHA, 
                                     BOT_GTK_PARAM_WIDGET_SLIDER, 0,
                                     1, 0.5,
                                     1.0);


    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_WRITE_NEXT_SCAN, 0, NULL);


    self->lcm = bot_lcm_get_global (NULL);
    if (0 == strcmp (velodyne_model, VELODYNE_HDL_32E_MODEL_STR))
        senlcm_velodyne_t_subscribe (self->lcm, lcm_channel, &velodyne_cb, self);
    else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR))
        senlcm_ford_velodyne_t_subscribe (self->lcm, lcm_channel, &velodyne_ford_cb, self);
    else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S2_MODEL_STR))
        senlcm_velodyne_t_subscribe (self->lcm, lcm_channel, &velodyne_cb, self);
    else
        ERROR ("ERROR: Unknown Velodyne model \'%s\'", velodyne_model);

    free (velodyne_model);
    free (lcm_channel);
    
    return self;
}
#endif //__PERLS_SENSORS__

void
setup_renderer_velodyne (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data, int priority)
{
#ifdef __PERLS_SENSORS__
    RendererVelodyne *self = new_renderer_velodyne (viewer, rootkey, pose_data);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
#else
    ERROR ("perls-sensors required!");
#endif
}
