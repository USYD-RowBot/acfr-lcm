#ifndef __VIEWER_RENDERERS_H__
#define __VIEWER_RENDERERS_H__

#include <math.h>
#include <glib.h>

#include <bot_vis/bot_vis.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include "renderer_robot_pose.h"

#ifdef __cplusplus
extern "C" {
#endif

// stand alone renderers
robot_pose_data_t *
setup_renderer_robot_pose (BotViewer *viewer, char *rootkey, int render_priority);

void
setup_renderer_image (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_vectormap_mission (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_status_text (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_lcmgl (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_isam_graph (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_pvn_map (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_sat_map (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_compass (BotViewer *viewer, int priority);

// renderers that attach to a robot pose
void
setup_renderer_dvl_beams (BotViewer *viewer, char *cfg_path, robot_pose_data_t *pose_data, int priority);

void
setup_renderer_planar_laser (BotViewer *viewer, char *cfg_path, robot_pose_data_t *pose_data, int priority);

void
setup_renderer_velodyne (BotViewer *viewer, char *cfg_path, robot_pose_data_t *pose_data, int priority);

void
setup_renderer_range_circles (BotViewer *viewer, char *cfg_path, robot_pose_data_t *pose_data, int priority);

void
setup_renderer_planar_target (BotViewer *viewer, char *rootkey, robot_pose_data_t *pose_data, int priority);

void
setup_renderer_hauv (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_vanctrl (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_hauv_pose (BotViewer *viewer, char *cfg_key, int priority);

void
setup_renderer_collections (BotViewer *viewer, int render_priority);

void
setup_renderer_camctrl (BotViewer *viewer, char *cfg_key, int render_priority);

void
setup_renderer_plot (BotViewer *viewer, char *cfg_key, int render_priority);

// misc functions
int
is_view_handler_owner (char *id_key);

void
own_view_handler (char *id_key);

static inline void 
draw_axis (double meters_per_grid)
{
    double s = meters_per_grid*0.5;
    glLineWidth (fmax (meters_per_grid*0.05,  2)); 
    glBegin (GL_LINES);
      glColor3f (1.0,0.0,0.0); glVertex3f (0.0,0.0,0.0); glVertex3f (s,0.0,0.0);
      glColor3f (0.0,1.0,0.0); glVertex3f (0.0,0.0,0.0); glVertex3f (0.0,s,0.0);
      glColor3f (0.0,0.0,1.0); glVertex3f (0.0,0.0,0.0); glVertex3f (0.0,0.0,s);
    glEnd ();
}

static inline void
matrix_eigen_symm_2x2 (const double m[4], double v[4], double e[2], double *theta)
{
    //assert (m[1] == m[2]);
    
    double phi = 0.5 * atan2 (-2 * m[2], m[3] - m[0]);
    v[0] = -sin (phi);
    v[1] = cos (phi);
    v[2] = -v[1];
    v[3] = v[0];

    if (fabs (v[0]) < 1e-10)
        e[0] = (m[2]*v[0] + m[3]*v[1]) / v[1];
    else
        e[0] = (m[0]*v[0] + m[1]*v[1]) / v[0];
    if (fabs (v[2]) < 1e-10)
        e[1] = (m[2]*v[2] + m[3]*v[3]) / v[3];
    else
        e[1] = (m[0]*v[2] + m[1]*v[3]) / v[2];

    *theta = phi;
}

#ifdef __cplusplus
}
#endif

#endif // __VIEWER_RENDERERS_H__
