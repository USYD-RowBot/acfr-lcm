#include <stdio.h>
#include <stdlib.h>

#include "perls-lcmtypes/senlcm_acomms_range_t.h"
#include "perls-lcmtypes/senlcm_gpsd3_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/units.h"

#include "renderers.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RENDERER_NAME "Range Circles"

#define MAX_N_RANGES  1

typedef struct _range_circle_t range_circle_t;
struct _range_circle_t {
    double x;
    double y;
    double radius;
};

typedef struct _RendererRangeCircles RendererRangeCircles;
struct _RendererRangeCircles {
    BotRenderer renderer;
    perllcm_position_t *pose;

    lcm_t    *lcm;
    BotParam *param;

    int   n_ranges;
    GList *ranges;

    double speed_of_sound;

    int src;
    int owtt_index;

    BotViewer *viewer;
    BotGtkParamWidget *pw;
};

static void
range_circle_free (BotRenderer *super)
{
    RendererRangeCircles *self = (RendererRangeCircles*) super->user;
    free (self);
}

static void
range_circle_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererRangeCircles *self = (RendererRangeCircles*) renderer->user;

    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // for yaw just rotate about y
    glRotatef (180.0, 1.0, 0.0, 0.0);
    glTranslatef (0.0, 0.0, 0.0);

    // Draw the range size and orientation --------------
    glDisable (GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glPushMatrix ();

    int count = self->n_ranges;
    GList *circles_list = g_list_last (self->ranges);
    while ((count--) > 0 && circles_list) {
        range_circle_t *circle = (range_circle_t*) circles_list->data;
        GLUquadricObj *q = gluNewQuadric ();

        // draw the range circles
        if (count == (self->n_ranges-1))
            glColor3f (0.0, 1.0, 0.0);
        else
            glColor3f (0.0, 0.0, 1.0);
        glTranslatef (circle->x, circle->y, 0);
        gluDisk (q, circle->radius-0.5, circle->radius, 50, 10);
        gluDeleteQuadric (q);

        circles_list = g_list_previous (circles_list);
        glTranslatef (-circle->x, -circle->y, 0.0);
    }

    glDisable (GL_BLEND);
    glPopMatrix ();
}

static void
range_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                const senlcm_acomms_range_t *msg, void *user)
{
    RendererRangeCircles *self = (RendererRangeCircles*) user;

    if (msg->src != self->src|| !(msg->owtt[self->owtt_index] > 0))
        return;

    range_circle_t *circle = (range_circle_t*) calloc (1, sizeof (*circle));
    circle->x      = self->pose->xyzrph[0];
    circle->y      = self->pose->xyzrph[1];
    circle->radius = self->speed_of_sound * msg->owtt[self->owtt_index];

    // add a new pose to the history
    while (g_list_length (self->ranges) >= MAX_N_RANGES ) {
        GList *first = g_list_first (self->ranges);
        self->ranges = g_list_remove (self->ranges, first->data);
    }

    self->ranges = g_list_append (self->ranges, circle);
    self->n_ranges = g_list_length (self->ranges);
}

// static void
// on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
// {
//     RendererRangeCircles *self = (RendererRangeCircles*) user_data;
//     printf ("%s\n", self->renderer.name);
//     bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
// }

// static void
// on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
// {
//     RendererRangeCircles *self = (RendererRangeCircles*) user_data;
//     bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
// }


static RendererRangeCircles *
new_renderer_range_circles (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data)
{
    RendererRangeCircles *self = (RendererRangeCircles*) calloc (1, sizeof (*self));

    // load from config file
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);

    char key[256] = {0};
    snprintf (key, sizeof key, "%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);

    snprintf (key, sizeof key, "%s.channel", rootkey);
    char *lcm_channel = bot_param_get_str_or_fail (self->param, key);

    self->speed_of_sound = 1500;
    bot_param_get_double (self->param, "site.speed_of_sound", &self->speed_of_sound);

    snprintf (key, sizeof key, "%s.owtt_src", rootkey);
    bot_param_get_int (self->param, key, &self->src);
    printf ("owtt_src = %d\n", self->src);
    
    snprintf (key, sizeof key, "%s.owtt_index", rootkey);
    bot_param_get_int (self->param, key, &self->owtt_index);
    printf ("owtt_ind = %d\n", self->owtt_index);

    BotRenderer *renderer = &self->renderer;
    renderer->draw = range_circle_draw;
    renderer->destroy = range_circle_free;

    renderer->widget = NULL;
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    self->pose = pose_data->pose;
    self->viewer = viewer;

    self->lcm = bot_lcm_get_global (NULL);
    senlcm_acomms_range_t_subscribe (self->lcm, lcm_channel, &range_callback, self);
    
    //g_signal_connect (G_OBJECT (viewer), "load-preferences", 
    //        G_CALLBACK (on_load_preferences), self);
    //g_signal_connect (G_OBJECT (viewer), "save-preferences",
    //        G_CALLBACK (on_save_preferences), self);

    return self;
}

void
setup_renderer_range_circles (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data, int priority)
{
    RendererRangeCircles *self = new_renderer_range_circles (viewer, rootkey, pose_data);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
