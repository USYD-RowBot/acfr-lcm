#include <stdio.h>
#include <stdlib.h>

#include "perls-common/cinttypes.h"
#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/units.h"

#include "perls-math/gsl_util.h"
#include "perls-math/ssc.h"

#include "pose_data_sources.h"
#include "renderers.h"


#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RENDERER_NAME "Robot Pose"

#define PARAM_ENABLE_DISPLAY "Enable Display"
#define PARAM_CLEAR_HISTORY "Clear History"
#define PARAM_HISTORY_LENGTH "History Length"
#define PARAM_AUTO_SCALE "Auto Scale Robot"
#define PARAM_FOLLOW "Follow"
#define PARAM_FOLLOW_POSE "Follow Pose"
#define PARAM_FOLLOW_YAW "Follow Yaw"
#define PARAM_FIND_ROBOT "Find Robot"
#define PARAM_X_WX_X "x_wl_x"
#define PARAM_X_WX_Y "x_wl_y"
#define PARAM_X_WX_Z "x_wl_z"
#define PARAM_X_WX_R "x_wl_r"
#define PARAM_X_WX_P "x_wl_p"
#define PARAM_X_WX_H "x_wl_h"
#define PARAM_Z_SCALE   "Z-axis Scale Factor"
#define MAX_Z_SCALE     100
#define Z_SCALE_DEFAULT 1

#define ADD_NEW_HIST_POSE_USEC 100000 // 0.1s
#define MAX_HISTORY_LENGTH     360000 // 10 hours
#define HISTORY_LENGTH_DEFAULT 60     // 1 min

// scale defines all relative to self->meters_per_grid
#define SCALE_MODEL_MAXIMA 1.5

void
update_history (int64_t utime, RendererRobotPose *self)
{
    if ((utime - self->pose_utime_last) >= ADD_NEW_HIST_POSE_USEC) {
        perllcm_position_t *tmp = (perllcm_position_t*) calloc (1, sizeof (*tmp));
        memcpy (tmp, &self->pose, sizeof (*tmp));
        
        // add a new pose to the history
        while (g_list_length (self->pose_history) >= MAX_HISTORY_LENGTH ) {
            GList *first = g_list_first (self->pose_history);
            self->pose_history = g_list_remove (self->pose_history, first->data);
        }        
        self->pose_history = g_list_append (self->pose_history, tmp);
        self->pose_age = utime - self->pose_utime_last;
        self->pose_age_timer = 0;
        self->pose_utime_last = utime;
    }
    
    self->pose_history_len = g_list_length (self->pose_history);
    int cur_value = bot_gtk_param_widget_get_int (self->pw, PARAM_HISTORY_LENGTH);
    if (cur_value >= self->pose_history_len-1)
        cur_value = self->pose_history_len;
    int step = floor (self->pose_history_len/100.0);
    
    bot_gtk_param_widget_modify_int (self->pw, PARAM_HISTORY_LENGTH, 0, self->pose_history_len, step, cur_value);
    bot_viewer_request_redraw (self->viewer);
}

static void
clear_pose_and_status (RendererRobotPose *self)
{
    memset(self->pose.xyzrph, 0, sizeof (double) * 6);
    memset(self->pose.xyzrph_dot, 0, sizeof (double) * 6);
    memset(self->pose.xyzrph_cov, 0, sizeof (double) * 6*6);
    self->pose_age = -1;
    self->pose_age_timer = -1;
    self->altitude = -1;
    self->next_waypoint = -1;   
    self->dist_to_next_waypoint = -1;
    self->batt_percent = -1;
    self->abort = -1;
    self->error = -1;
    self->speed = -1;
}

static void
print_status (RendererRobotPose *self, char *out)
{    
    // @TODO: changed PRId64 to lu. should check this!
    // wrap circular pose elements
    self->pose.xyzrph[3] = gslu_math_minimized_angle (self->pose.xyzrph[3]);
    self->pose.xyzrph[4] = gslu_math_minimized_angle (self->pose.xyzrph[4]);
    self->pose.xyzrph[5] = gslu_math_minimized_angle (self->pose.xyzrph[5]);

    sprintf (out, "xyz = %0.1lf, %0.1lf, %0.1lf NED\n",
             self->pose.xyzrph[0], self->pose.xyzrph[1], self->pose.xyzrph[2]);
    sprintf (out, "%srph = %0.1lf, %0.1lf, %0.1lf deg\n", out,
             self->pose.xyzrph[3]*RTOD,
             self->pose.xyzrph[4]*RTOD,
             self->pose.xyzrph[5]*RTOD);
    if (-1 == self->pose_age) 
        sprintf (out, "%sAge = NEVER\n", out);
    else {
        if (self->pose_age_timer > 1999999) {
            int64_t s = timestamp_seconds (self->pose_age_timer);
            int64_t m = s/60;
            if (m)
                sprintf (out, "%sAge = %"cPRId64":%02"cPRId64" min (cpu)\n", out, m, s-m*60);
            else
                sprintf (out, "%sAge = %"cPRId64" s (cpu)\n", out, s);
        }
        else {
            sprintf (out, "%sAge = %.02f s\n", out, timestamp_to_double (self->pose_age));
        }
    }

    // iver attributes
    if (-1 < self->altitude)
        sprintf (out, "%sAltitude = %0.1lf m\n", out, self->altitude);
    if (-1 < self->next_waypoint)
        sprintf (out, "%sNext waypoint = %d\n", out, self->next_waypoint);
    if (-1 < self->dist_to_next_waypoint)
        sprintf (out, "%sDist. to next waypt = %0.1lf m\n", out, self->dist_to_next_waypoint);
    if (-1 < self->speed)
        sprintf (out, "%sSpeed = %0.1lf m/s\n", out, self->speed);
    if (-1 < self->batt_percent)
        sprintf (out, "%sbatt_percent = %0.1lf %%\n", out, self->batt_percent);
    if (-1 < self->error)
        sprintf (out, "%serror = %d \n", out, self->error);
    if (-1 < self->abort)
        sprintf (out, "%sabort = %d \n", out, self->abort);
}


static void
_renderer_free (BotRenderer *super)
{
    RendererRobotPose *self = (RendererRobotPose*) super->user;
    if (self->wavefront_model)
        bot_wavefront_model_destroy (self->wavefront_model);
    if (self->rwx_model)
        bot_rwx_model_destroy (self->rwx_model);
    free (self);
}

static void
draw_wavefront_model (RendererRobotPose *self)
{
    glPushMatrix ();
    glEnable (GL_BLEND);
    glEnable (GL_RESCALE_NORMAL);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);
    glEnable (GL_COLOR_MATERIAL);
    glCallList (self->wavefront_dl);
    glPopMatrix ();
}

static GLuint
compile_wavefront_display_list (RendererRobotPose *self, BotWavefrontModel *model)
{
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);

    glEnable (GL_LIGHTING);
    glEnable (GL_COLOR_MATERIAL);
    bot_wavefront_model_gl_draw (model);
    glDisable (GL_COLOR_MATERIAL);
    glDisable (GL_LIGHTING);

    glEndList ();
    return dl;
}

static void
draw_rwx_model (RendererRobotPose *self)
{
    glPushMatrix ();
    glEnable (GL_BLEND);
    glEnable (GL_RESCALE_NORMAL);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);
    glCallList (self->rwx_dl);
    glPopMatrix ();
}

static GLuint
compile_rwx_display_list (RendererRobotPose *self, BotRwxModel *model)
{
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);

    glEnable (GL_LIGHTING);
    bot_rwx_model_gl_draw (model);
    glDisable (GL_LIGHTING);

    glEndList ();
    return dl;
}

static void
scale_rwx_model (BotRwxModel *model, double s)
{
    GList *citer;
    for (citer = model->clumps; citer != NULL; citer = citer->next) {
        BotRwxClump *clump = (BotRwxClump*) citer->data;
        for (int i = 0; i < clump->nvertices; i++) {
            BotRwxVertex *v = clump->vertices + i;
            for (int j=0; j<3; j++)
                v->pos[j] *= s;
        }
    }
}


static void 
robot_pose_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererRobotPose *self = (RendererRobotPose*) super->user;
    
    g_assert (self->viewer == viewer);  
    
    if (!bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_DISPLAY))
        return;
    
    BotViewHandler *vhandler = self->viewer->view_handler;
    
    int z_scale = 1;
    if (self->show_z_scale) {
        z_scale = bot_gtk_param_widget_get_int (self->pw, PARAM_Z_SCALE);
    }

    if (is_view_handler_owner (self->rootkey)) {
        vhandler->follow_mode = 0;
        double rph[3] = {0};
        double quat[4] = {0};
        double pose[3] = {0};
        // account for the difference between (N-W-UP) rotate to our prefered (N-E-D)
        pose[0] = self->pose.xyzrph[0];
        pose[1] = -self->pose.xyzrph[1];
        pose[2] = -self->pose.xyzrph[2]*z_scale;
        rph[0] = 0; //self->pose.r;
        rph[1] = 0; //self->pose.p;
        rph[2] = -self->pose.xyzrph[5];
        
        bot_roll_pitch_yaw_to_quat (rph, quat);
        vhandler->follow_mode = bot_gtk_param_widget_get_enum (self->pw, PARAM_FOLLOW);
            
        if (vhandler && vhandler->update_follow_target && vhandler->follow_mode != 0)
            vhandler->update_follow_target (vhandler, pose, quat);
    }
    else
        bot_gtk_param_widget_set_enum (self->pw, PARAM_FOLLOW, 0);
    
    // get a sense of scale so that as we zoom we can keep the mission visable
    // set relative to grid when automatic spacing is on
    double eye[3], look[3], up[3];
    vhandler->get_eye_look (viewer->view_handler, eye, look, up);
    // when looking directly down at the world, about how many grids should appear across the screen?
    double grids_per_screen = 10;
    double eye_dist = bot_vector_dist_3d (eye, look);
    self->meters_per_grid = eye_dist/grids_per_screen;    

    char status_txt[1024];
    print_status (self, status_txt);
    gtk_label_set_text (GTK_LABEL (self->status_txt), status_txt);

    glEnable (GL_DEPTH_TEST);

    // setup wavefront if its there
    if (!self->wavefront_display_lists_ready) {
        if (self->wavefront_model)
            self->wavefront_dl = compile_wavefront_display_list (self, self->wavefront_model);
        self->wavefront_display_lists_ready = 1;
    }
    // setup rwx if its there (rwx has second priority)
    else if (!self->rwx_display_lists_ready) {
        if (self->rwx_model)
            self->rwx_dl = compile_rwx_display_list (self, self->rwx_model);
        self->rwx_display_lists_ready = 1;
    }

    //glPushMatrix ();   

    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0,1.0,0.0,0.0);
    glTranslatef (0.0,0.0,0.0);
    draw_axis (10);
    
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
    
    // draw pose history trajectory (in reverse order)
    int hist_len = bot_gtk_param_widget_get_int (self->pw, PARAM_HISTORY_LENGTH);
    int count = 0;
    if (2 <= g_list_length (self->pose_history)) {
        GList *ph_list = g_list_last (self->pose_history);
        perllcm_position_t *ph_prev = (perllcm_position_t*) ph_list->data;
        perllcm_position_t *ph = NULL;
        
        glColor3f (self->base_color[0], self->base_color[1], self->base_color[2]);
        glPointSize(3);
        
        while ((ph_list = g_list_previous (ph_list)) && count < hist_len) {
            
            ph = (perllcm_position_t*) ph_list->data;
            
            if (self->line_type == '-') {
                //draw a line to previous
                glBegin (GL_LINES);
                glVertex3f (ph_prev->xyzrph[0], ph_prev->xyzrph[1], ph_prev->xyzrph[2]*z_scale);
                glVertex3f (ph->xyzrph[0], ph->xyzrph[1], ph->xyzrph[2]*z_scale); 
                glEnd ();
            } else if (self->line_type == '.') {
                glBegin (GL_POINTS);
                glVertex3f (ph_prev->xyzrph[0], ph_prev->xyzrph[1], ph_prev->xyzrph[2]*z_scale);
                glEnd ();
            }
    
            ph_prev = ph;
            count++;
        }
    }
    
    double dx = self->pose.xyzrph[0];
    double dy = self->pose.xyzrph[1];
    double dz = self->pose.xyzrph[2]*z_scale;
    glTranslated (dx, dy, dz);
    
    // plot spatial uncertianty
    glColor3f (self->base_color[0], self->base_color[1], self->base_color[2]);
    
    double eigv[4];
    double eig[2];
    double theta;
    double xy_cov[4] = {self->pose.xyzrph_cov[0], self->pose.xyzrph_cov[1],
                        self->pose.xyzrph_cov[6], self->pose.xyzrph_cov[7]};
    matrix_eigen_symm_2x2 (xy_cov, eigv, eig, &theta);
    bot_gl_draw_ellipse (3*sqrt(eig[0]), 3*sqrt(eig[1]), theta, 20);
    
    // draw the model
    double rot_x = self->pose.xyzrph[3] * UNITS_RADIAN_TO_DEGREE;
    double rot_y = self->pose.xyzrph[4] * UNITS_RADIAN_TO_DEGREE;
    double rot_z = self->pose.xyzrph[5] * UNITS_RADIAN_TO_DEGREE;
    glRotatef (rot_z, 0, 0, 1);
    glRotatef (rot_y, 0, 1, 0);
    glRotatef (rot_x, 1, 0, 0);
    
    // draw wavefront
    if (self->wavefront_display_lists_ready && self->wavefront_dl) {
        if (bot_gtk_param_widget_get_bool (self->pw, PARAM_AUTO_SCALE)) {
            double scale = (self->meters_per_grid * SCALE_MODEL_MAXIMA)/self->model_max_dim;
            glScalef (scale, scale, scale);
        }        
        draw_wavefront_model (self);
    } 
    else if (self->rwx_display_lists_ready && self->rwx_dl) {
        if (bot_gtk_param_widget_get_bool (self->pw, PARAM_AUTO_SCALE)) {
            double scale = (self->meters_per_grid * SCALE_MODEL_MAXIMA)/self->model_max_dim;
            glScalef (scale, scale, scale);
        }     
        draw_rwx_model (self);    
    }
    else
        draw_axis (self->meters_per_grid);

    //glPopMatrix ();
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    if (self->x_wl_adjust) {
        self->x_wl[0] = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_X);
        self->x_wl[1] = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_Y);
        self->x_wl[2] = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_Z);
        self->x_wl[3] = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_R)*DTOR;
        self->x_wl[4] = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_P)*DTOR;
        self->x_wl[5] = bot_gtk_param_widget_get_double(self->pw, PARAM_X_WX_H)*DTOR;
    }
    
    if (bot_gtk_param_widget_get_enum (self->pw, PARAM_FOLLOW))
        own_view_handler (self->rootkey);
    bot_viewer_request_redraw(self->viewer);
}

static void 
on_find_robot (GtkWidget *button, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    BotViewHandler *vhandler = self->viewer->view_handler;

    int duration_ms = 500;  //milliseconds
    
    double pose[6] = {0};
    if (self->x_wl_adjust) {
        ssc_head2tail (pose, NULL, self->x_wl, self->pose.xyzrph);
    } else {
        memcpy (pose, self->pose.xyzrph, 6*sizeof (double));
    }
    
    double eye[3] = {pose[0], -pose[1], -pose[2]+20};
    double lookat[3] = {pose[0], -pose[1] ,-pose[2]};
    double up[3] = {1, 0, 0};
    
    vhandler->set_look_at_smooth (vhandler, eye, lookat, up, duration_ms);
}

static void 
on_clear_history (GtkWidget *button, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    g_list_foreach (self->pose_history, (GFunc)g_free, NULL);
    g_list_free (self->pose_history);
    self->pose_history = NULL;
    self->pose_utime_last = 0;
    bot_viewer_request_redraw (self->viewer);
}

static void
load_model_btrans (RendererRobotPose *self, const char *model_fname)
{
    bot_trans_set_identity (&self->model_btrans);
    self->model_scale = 1.0;

    if (!model_fname)
        return;

    char *btrans_fname = g_strdup (model_fname);
    btrans_fname[strlen (btrans_fname)-3] = 'x';
    btrans_fname[strlen (btrans_fname)-2] = 'f';
    btrans_fname[strlen (btrans_fname)-1] = 'm';
    if (!g_file_test (btrans_fname, G_FILE_TEST_EXISTS)) {
        free (btrans_fname);
        return; // btrans file not given
    }

    BotParam *param = bot_param_new_from_file (btrans_fname);
    if (!param) {
        ERROR ("Unable to open accompanying btrans file [%s]", btrans_fname);
        free (btrans_fname);
        return;
    }

    double xyz[3] = {0, 0, 0};
    bot_param_get_double_array (param, "xyz", xyz, 3);

    double rph[3] = {0, 0, 0};
    bot_param_get_double_array (param, "rph", rph, 3);
    for (int i=0; i<3; i++)
        rph[i] *= DTOR;

    double quat[4];
    bot_roll_pitch_yaw_to_quat (rph, quat);
    bot_trans_set_from_quat_trans (&self->model_btrans, quat, xyz);

    double defaultLen = 1.0, desiredLen = 1.0;
    bot_param_get_double (param, "default", &defaultLen);
    bot_param_get_double (param, "desired", &desiredLen);
    self->model_scale = desiredLen / defaultLen;

    free (btrans_fname);
    bot_param_destroy (param);
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererRobotPose *self = (RendererRobotPose*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererRobotPose *self = (RendererRobotPose*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

RendererRobotPose *
renderer_robot_pose_new (BotViewer *viewer, char *rootkey)
{
    RendererRobotPose *self = (RendererRobotPose*) calloc (1, sizeof (*self));
    
    // load config -------------------------------------------------------------
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    strcpy (self->rootkey, rootkey);

    // lat lon origin
    double ll_deg[2];
    bot_param_get_double_array_or_fail (self->param, "site.orglatlon", ll_deg, 2);
    self->llxy = (BotGPSLinearize*) calloc (1, sizeof (BotGPSLinearize));
    bot_gps_linearize_init (self->llxy, ll_deg);
    self->org_alt = 0;
    self->use_alt = 0;
    if (0 == bot_param_get_double (self->param, "site.orgalt", &self->org_alt))
        self->use_alt = 1;

    char key[256] = {0};
    sprintf (key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);
    
    // parse model
    char *wavefront_fname = NULL;
    snprintf (key, sizeof key, "viewer.renderers.%s.obj_file", rootkey);
    bot_param_get_str (self->param, key, &wavefront_fname);


    char *rwx_fname = NULL;
    snprintf (key, sizeof key, "viewer.renderers.%s.rwx_file", rootkey);
    bot_param_get_str (self->param, key, &rwx_fname);

    if (wavefront_fname && rwx_fname) {
        ERROR ("only one obj or rwx model can be specified, not both! [viewer.renderers.%s]", rootkey);
        exit (EXIT_FAILURE);
    }

    snprintf (key, sizeof key, "viewer.renderers.%s.base_color", rootkey);
    if (3 != bot_param_get_double_array (self->param, key, self->base_color, 3)) {
	printf ("No base_color entry for \"viewer.renderers.%s\" using default (blue) \n", rootkey);
        self->base_color[0] = 1;
        self->base_color[1] = 0;
        self->base_color[2] = 0;
    }
    
    self->x_wl_adjust = 0;
    snprintf (key, sizeof key, "viewer.renderers.%s.x_wl_adjust", rootkey);
    bot_param_get_int (self->param, key, &self->x_wl_adjust);
    
    self->show_z_scale = 0;
    snprintf (key, sizeof key, "viewer.renderers.%s.show_z_scale", rootkey);
    bot_param_get_int (self->param, key, &self->show_z_scale);
    
    char *line_type = NULL;
    self->line_type = '-';
    snprintf (key, sizeof key, "viewer.renderers.%s.line_type", rootkey);
    bot_param_get_str (self->param, key, &line_type);
    if (line_type) {
        self->line_type = line_type[0];
        free (line_type);
    }

    clear_pose_and_status (self);
    self->lcm = bot_lcm_get_global (NULL);
   
    // load data sources
    sprintf (key, "viewer.renderers.%s.data_sources", rootkey);
    int num_data_sources = bot_param_get_num_subkeys (self->param, key);
    printf("\tNumber of Robot Pose data sources %d \n", num_data_sources);
    char **data_source_keys;
    data_source_keys = bot_param_get_subkeys (self->param, key);
    // loop over each data source
    for (int i=0 ; i<num_data_sources ; i++) {
        char *lcm_chan;
        sprintf (key, "viewer.renderers.%s.data_sources.%s", rootkey, data_source_keys[i]);
        lcm_chan = bot_param_get_str_or_fail (self->param, key);
        pose_data_source_subscribe (data_source_keys[i], self, lcm_chan);
        free (lcm_chan);
    }
        
    //--------------------------------------------------------------------------
    BotRenderer *renderer = &self->renderer;
    renderer->draw = robot_pose_renderer_draw;
    renderer->destroy = _renderer_free;
    renderer->widget = bot_gtk_param_widget_new ();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);

    if (wavefront_fname) { 
        self->wavefront_model = bot_wavefront_model_create (wavefront_fname);

        double m[16];
        load_model_btrans (self, wavefront_fname);
        bot_trans_get_mat_4x4 (&self->model_btrans, m);
        //bot_wavefront_model_apply_transform (self->rwx_model, m);
        //botu_wavefront_model_apply_scale (self->rwx_model, self->model_scale);

        double minv[3];
        double maxv[3];
        bot_wavefront_model_get_extrema (self->wavefront_model, minv, maxv);

        self->model_max_dim = -1;
        for (int i=0; i<2; i++) {
            double d = maxv[i] - minv[i];
            if (d > self->model_max_dim)
                self->model_max_dim = d;
        }
        free (wavefront_fname);
    }
    else if (rwx_fname) {    
        self->rwx_model = bot_rwx_model_create (rwx_fname);

        double m[16];
        load_model_btrans (self, rwx_fname);
        bot_trans_get_mat_4x4 (&self->model_btrans, m);
        bot_rwx_model_apply_transform (self->rwx_model, m);
        scale_rwx_model (self->rwx_model, self->model_scale);

        double minv[3];
        double maxv[3];
        bot_rwx_model_get_extrema (self->rwx_model, minv, maxv);

        self->model_max_dim = -1;
        for (int i=0; i<2; i++) {
            double d = maxv[i] - minv[i];
            if (d > self->model_max_dim)
                self->model_max_dim = d;
        }

        printf ("\tRWX file:%s\n", rwx_fname);
        printf ("\tModel max dim:%g\n", self->model_max_dim);
        for (int i=0; i<3; i++) {
            printf ("\tminv[%d]=%10g   maxv[%d]=%g\n", i, minv[i], i, maxv[i]);
        }
        free (rwx_fname);
    }

    // layout sidebar widgets --------------------------------------------------  
    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw),FALSE, TRUE, 0);
    gtk_widget_show (GTK_WIDGET (self->pw));

    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE_DISPLAY, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_AUTO_SCALE, 1, NULL);
    
    bot_gtk_param_widget_add_enum (self->pw, PARAM_FOLLOW, (BotGtkParamWidgetUIHint) 0, 0,
                                    "NONE", 0,
                                    PARAM_FOLLOW_POSE, BOT_FOLLOW_POS,
                                    PARAM_FOLLOW_YAW, BOT_FOLLOW_YAW, NULL);

    // find robot button
    GtkWidget *find_robot_button = gtk_button_new_with_label (PARAM_FIND_ROBOT);
    gtk_box_pack_start (GTK_BOX (vbox), find_robot_button, FALSE, FALSE, 0);
    g_signal_connect (G_OBJECT (find_robot_button), "clicked", G_CALLBACK (on_find_robot), self);

    // history length slider
    bot_gtk_param_widget_add_int(self->pw, PARAM_HISTORY_LENGTH, 
                                 BOT_GTK_PARAM_WIDGET_SLIDER, 0, MAX_HISTORY_LENGTH, 1, HISTORY_LENGTH_DEFAULT);
    
    // clear history btn
    GtkWidget *clear_history_button = gtk_button_new_with_label (PARAM_CLEAR_HISTORY);
    gtk_box_pack_start (GTK_BOX (vbox), clear_history_button, FALSE, FALSE, 0);
    g_signal_connect (G_OBJECT (clear_history_button), "clicked", G_CALLBACK (on_clear_history), self);
    
    // z_scale length slider
    if (self->show_z_scale) {
        bot_gtk_param_widget_add_int(self->pw, PARAM_Z_SCALE, 
                                     BOT_GTK_PARAM_WIDGET_SLIDER, 1, MAX_Z_SCALE, 1, Z_SCALE_DEFAULT);
    }
    
    // info textbox
    self->status_txt = gtk_label_new ("Status:");
    gtk_box_pack_start (GTK_BOX (vbox),  self->status_txt, FALSE, FALSE, 0);
    gtk_label_set_justify (GTK_LABEL (self->status_txt), GTK_JUSTIFY_LEFT);
    
    if (self->x_wl_adjust) {
        bot_gtk_param_widget_add_double(self->pw, PARAM_X_WX_X, 
            BOT_GTK_PARAM_WIDGET_SPINBOX, -9999999, 99999999, 0.01, 0);
        bot_gtk_param_widget_add_double(self->pw, PARAM_X_WX_Y, 
                BOT_GTK_PARAM_WIDGET_SPINBOX, -9999999, 99999999, 0.01, 0);
        bot_gtk_param_widget_add_double(self->pw, PARAM_X_WX_Z, 
                BOT_GTK_PARAM_WIDGET_SPINBOX, -9999999, 99999999, 0.01, 0);
    
        bot_gtk_param_widget_add_double(self->pw, PARAM_X_WX_R, 
                BOT_GTK_PARAM_WIDGET_SPINBOX, -180, 180, 0.1, 0);
        bot_gtk_param_widget_add_double(self->pw, PARAM_X_WX_P, 
                BOT_GTK_PARAM_WIDGET_SPINBOX, -180, 180, 0.1, 0);
        bot_gtk_param_widget_add_double(self->pw, PARAM_X_WX_H, 
                BOT_GTK_PARAM_WIDGET_SPINBOX, -180, 180, 0.1, 0);        
    }
    
    // END layout sidebar widgets ----------------------------------------------
    
    gtk_widget_show_all (renderer->widget);

    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
            G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
            G_CALLBACK (on_save_preferences), self);

    return self;
}

int
timer_callback (void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    if (-1 < self->pose_age_timer)
        self->pose_age_timer += 1000000;
    return 1;
}


robot_pose_data_t *
setup_renderer_robot_pose (BotViewer *viewer, char *rootkey, int render_priority)
{
    RendererRobotPose *self = renderer_robot_pose_new (viewer, rootkey);   
    bot_viewer_add_renderer (viewer, &self->renderer, render_priority);
    //bot_viewer_add_event_handler (viewer, &(self->ehandler), render_priority);

    struct timespec spec = timeutil_hz_to_timespec (1);
    timeutil_timer_create (spec, &timer_callback, self);
    
    self->pose_data.x_wl_adjust = self->x_wl_adjust;
    self->pose_data.x_wl = self->x_wl;
    self->pose_data.pose = &self->pose;
    //self->pose_data.show_z_scale = self->show_z_scale;

    return &self->pose_data;
}
