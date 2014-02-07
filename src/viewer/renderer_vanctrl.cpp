/*
 * renders a van ctrl
 * Author: Ayoung Kim (ayoungk@umich.edu)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <vector>
#include <map>
#include <iostream>
#include <stdexcept>

#include <GL/gl.h>
#include <GL/glu.h>

#include "perls-common/cinttypes.h"
#include "perls-common/timestamp.h"

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include "perls-lcmtypes/perllcm_isam_cmd_t.h"
#include "perls-lcmtypes/perllcm_van_options_t.h"
#include "perls-lcmtypes/senlcm_prosilica_t.h"
#include "perls-lcmtypes/senlcm_prosilica_attribute_t.h"
#include "perls-lcmtypes/bot_core_image_t.h"

#include "perls-common/bot_util.h"

#include "renderers.h"

// strings for parameter widget
#define PARAM_VAN_PLOT_MASTER           "Master mode on"
#define PARAM_VAN_PLOT_FEATURES         "Plot features"
#define PARAM_VAN_PLOT_SP               "Plot scene prior"
#define PARAM_VAN_PLOT_PUTCORR          "Plot put corr"
#define PARAM_VAN_PLOT_ELLIPSES         "Plot search bound"
#define PARAM_VAN_PLOT_IN_OUT           "Plot in+out"
#define PARAM_VAN_PLOT_IN               "Plot inliers"
#define PARAM_VAN_PLOT_RELPOSE          "Plot pose estim"
#define PARAM_VAN_PLOT_3DPTS            "Plot 3dpts estim"
#define PARAM_VAN_PLOT_RELPOSE_3DPTS    "Plot pose+3dpts estim"
#define PARAM_VAN_PLOT_SUMMARY          "Plot summary"
#define PARAM_VAN_PLOT_WAITKEY          "Plot hold on"
#define PARAM_VAN_NPLINKS               "nlinks"
#define PARAM_VAN_USE_SALIENCY          "Use saliency"
#define PARAM_CAMERA_ON_OFF             "Camera on/off"
#define PARAM_SE_OPTION_LUT             "LUT mission"
//#define PARAM_NSAMPLE_IMGS              "Img#"
//#define PARAM_NEED_VERIFICATION         "Manual corr check?"
//#define PARAM_USE_MDIST_PLINK           "Mdist plink?"
//#define PARAM_CONNECT_COV               "Conn cov"

// parameters
#define MAX_NPLINKS     100
#define TN_WIDTH        0.35
#define TN_SPACING      0.02
#define TN_MAX_DISPLAY  10
#define GROUND_LEVEL 0.0
#define TNLIST_HEIGHT   0

/* plot relative pose estimation and/or 3d points*/
enum {
    MODE_OFF,
    MODE_POSE,  
    MODE_PTS,
    MODE_BOTH
};

typedef struct _RendererVanctrl RendererVanctrl;

struct _RendererVanctrl {
    BotRenderer renderer;

    BotViewer *viewer;

    BotGtkParamWidget  *pw;
    GtkWidget          *label;
    lcm_t* lcm;
    perllcm_van_options_t *van_opts;
    senlcm_prosilica_attribute_t* cam_attr;

    int8_t master_mode;

    int viewer_mode;

    int imageWidth, imageHeight;
    int imageBytesPerPixel;
    unsigned char imgdata[340*256*6];

    std::vector<bot_core_image_t*> m_imglist;
    std::vector<int64_t> m_imgutimelist;
    std::vector<double> x_pos;
    std::vector<double> y_pos;
    std::vector<double> z_pos;

    // render image with aspect ratio
    int glarea_width;
    int glarea_height;

    // loading graph and selecting utime to connect from sample images
    bool load_complete;
    int cur_sampleimg_idx;
    int64_t utime_conn;
    int n_loaded_nodes;
    double map_merging_cov;

    // channel names
    BotParam *m_param;
    char *m_vanopt_channel;
    char *m_img_uw_channel;
    char *m_img_peri_channel;
    char *m_isam_cmd_channel;
};

static void
draw_thumbnail (int n, int cur_idx, int w, int h, float ratio,
                BotRenderer *renderer, GLuint texName)
{
    float tn_width = TN_WIDTH;
    float space = TN_SPACING;

    RendererVanctrl *self = (RendererVanctrl*) renderer->user;

    if ( (1.0 - 0.5*tn_width - space*abs(cur_idx) - tn_width*abs(cur_idx)) < 0 ||
        n + cur_idx < 0 || n + cur_idx > self->n_loaded_nodes -1 )
        return;

    int gl_format = (self->imageBytesPerPixel==3) ? GL_RGB : GL_INTENSITY16;
    int gl_size = (self->imageBytesPerPixel==1) ? GL_UNSIGNED_BYTE : GL_UNSIGNED_SHORT;
    int gl_internalformat = (self->imageBytesPerPixel==3) ? GL_RGB : GL_LUMINANCE;

    // center image
    bot_core_image_t *botimg = self->m_imglist.at(n + cur_idx);
    memcpy(self->imgdata, botimg->data, botimg->width * botimg->height * self->imageBytesPerPixel);
    glTexImage2D (GL_TEXTURE_2D, 0, gl_format, w, h, 0, gl_internalformat, gl_size, self->imgdata);
    
    float x_left = -0.5*tn_width + tn_width*cur_idx + space*cur_idx;
    float x_right = 0.5*tn_width + tn_width*cur_idx + space*cur_idx;

    float tnlist_height = TNLIST_HEIGHT;
    glEnable (GL_TEXTURE_2D);
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture (GL_TEXTURE_2D, texName);
    glColor3f (1.0,1.0,1.0);
    glBegin (GL_QUADS);
      glTexCoord2f (0.0, 0.0); glVertex3f (x_left, tnlist_height+tn_width*ratio, 0);
      glTexCoord2f (1.0, 0.0); glVertex3f (x_right,tnlist_height+tn_width*ratio, 0);
      glTexCoord2f (1.0, 1.0); glVertex3f (x_right,tnlist_height,   0);
      glTexCoord2f (0.0, 1.0); glVertex3f (x_left, tnlist_height,   0);
    glEnd ();
    glDisable (GL_TEXTURE_2D);

    // add utime on the top
    char utime_str[256];
    int64_t utime = self->m_imgutimelist.at(n + cur_idx);
    snprintf (utime_str, sizeof utime_str, "%"cPRId64, utime);
    glPushMatrix ();
      glTranslated (0.5*(x_left+x_right), tnlist_height+tn_width*ratio+0.02, 0);
      double font_pos[3]={0.0 ,0.0, 0.0};
      if (cur_idx == 0) {
          glColor4f (1.0f, 0.0f, 0.0f, 1.0f);
          bot_gl_draw_text (font_pos, GLUT_BITMAP_HELVETICA_12, utime_str, 10);
      }
      else {
          glColor4f (0.0f, 0.0f, 0.0f, 1.0f);
          bot_gl_draw_text (font_pos, GLUT_BITMAP_HELVETICA_10, utime_str, 10);
      }

    glPopMatrix ();
}

static void
render_sampleimgs (BotViewer *viewer, BotRenderer *renderer)
{
    RendererVanctrl *self = (RendererVanctrl*) renderer->user;
    g_assert (self->viewer == viewer);

    GLuint texName;

    glMatrixMode (GL_PROJECTION);
    glPushMatrix ();
    glLoadIdentity ();

    glOrtho (-1, 1, -1, 1, -1, 1);

    glMatrixMode (GL_MODELVIEW);
    glPushMatrix ();
    glLoadIdentity ();

    glPushAttrib (GL_DEPTH_BUFFER_BIT);
    glDisable (GL_DEPTH_TEST);

    // keep image original apsect ratio on viewer
    float glarea_ratio = (float) self->glarea_width / (float) self->glarea_height;
    float image_ratio = (float) self->imageHeight / (float) self->imageWidth;
    float ratio = glarea_ratio * image_ratio;

    // Setup texture parameters
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glGenTextures (1, &texName);
    glBindTexture (GL_TEXTURE_2D, texName);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    int width = self->imageWidth;
    int height = self->imageHeight;

    int center_n = self->cur_sampleimg_idx;
    int max_display = TN_MAX_DISPLAY;

    if (center_n >= self->n_loaded_nodes)
        center_n = self->n_loaded_nodes-1;

    // draw thumbnails
    for (int i=-max_display; i<max_display; i++) {
        draw_thumbnail (center_n, i, width, height, ratio, renderer, texName);
    }

    glPopAttrib ();

    glMatrixMode (GL_PROJECTION);
    glPopMatrix ();

    glMatrixMode (GL_MODELVIEW);
    glPopMatrix ();

    glDeleteTextures (1, &texName);

}

static void
highlight_node (BotViewer *viewer, BotRenderer *renderer)
{
    RendererVanctrl *self = (RendererVanctrl*) renderer->user;
    g_assert (self->viewer == viewer);

    int center_n = self->cur_sampleimg_idx;
    if (center_n >= self->n_loaded_nodes)
        center_n = self->n_loaded_nodes-1;

    //int64_t utime = self->m_imgutimelist.at(center_n);

    double x = self->x_pos.at(center_n);
    double y = self->y_pos.at(center_n);
    double z = self->z_pos.at(center_n);

    //printf ("(%g,%g,%g)\n", x, y, z);

    GLUquadricObj *quadric = gluNewQuadric();  
    glPushMatrix ();
      glRotatef(180.0,1.0,0.0,0.0);
      glTranslatef(0.,0., GROUND_LEVEL);
      glTranslated (x,y,z);
      glColor4f (1.0f, 0.0f, 0.0f, 0.2f);
      gluQuadricDrawStyle(quadric, GLU_FILL);
      gluQuadricNormals(quadric, GLU_SMOOTH);
      gluSphere(quadric, 0.5, 10, 10);
    glPopMatrix ();
}

static void
vanctrl_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererVanctrl *self = (RendererVanctrl*) renderer->user;
    g_assert (self->viewer == viewer);

    glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT);
    glEnable (GL_DEPTH_TEST);
    glDepthFunc (GL_LESS);
    glDisable (GL_LIGHTING);
    glLineWidth (1);
    glPopAttrib ();

    // map merging related functions (sample images and highlighed node)
    if (self->load_complete && self->viewer_mode != PERLLCM_ISAM_CMD_T_MODE_START) {
      render_sampleimgs (viewer, renderer);
      highlight_node (viewer, renderer);

      int center_n = self->cur_sampleimg_idx;
      if (center_n >= self->n_loaded_nodes)
        center_n = self->n_loaded_nodes-1;

      self->utime_conn = self->m_imgutimelist.at(center_n);
    }
}

static void
vanctrl_free (BotRenderer *renderer) 
{
    if (!renderer)
        return;

    RendererVanctrl *self = (RendererVanctrl*) renderer->user;
 
    if (!self)
        return;

    free (self);
}

static void 
on_van_opts (const lcm_recv_buf_t *rbuf, const char *channel,
             const perllcm_van_options_t *msg, void *user_data)
{
    //printf("receiving plot option...");
    RendererVanctrl *self = (RendererVanctrl*) user_data;

    memcpy(self->van_opts, msg, sizeof(*msg));

    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_FEATURES, msg->vis_plot_features);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_SP, msg->vis_plot_scene_prior);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_PUTCORR, msg->vis_plot_put_corr);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_ELLIPSES, msg->vis_plot_search_ellipses);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_IN_OUT, msg->vis_plot_in_and_out);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_IN, msg->vis_plot_inliers);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_RELPOSE, msg->vis_plot_relpose);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_3DPTS, msg->vis_plot_3pts);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_RELPOSE_3DPTS, msg->vis_plot_relpose_3pts);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_SUMMARY, msg->vis_plot_summary);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_WAITKEY, msg->vis_plot_waitkey);
    bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_USE_SALIENCY, msg->vis_use_saliency);
    //bot_gtk_param_widget_set_bool (self->pw, PARAM_NEED_VERIFICATION, msg->manual_corr);
    bot_gtk_param_widget_set_int (self->pw, PARAM_VAN_NPLINKS, msg->n_plinks);
}


static void 
on_image_t (const lcm_recv_buf_t *rbuf, const char *channel,
            const bot_core_image_t *msg, void *user_data)
{
  RendererVanctrl *self = (RendererVanctrl*) user_data;
  bot_viewer_request_redraw (self->viewer);

  if (self->master_mode && self->viewer_mode == PERLLCM_ISAM_CMD_T_MODE_START) {
    // publish van plot options at frame rate
    perllcm_van_options_t van_opts = {0};
    van_opts.self = 0;
    van_opts.vis_plot_features = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_FEATURES);
    van_opts.vis_plot_scene_prior = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_SP);
    van_opts.vis_plot_put_corr = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_PUTCORR);
    van_opts.vis_plot_search_ellipses = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_ELLIPSES);
    van_opts.vis_plot_in_and_out = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_IN_OUT);
    van_opts.vis_plot_inliers = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_IN);
    van_opts.vis_plot_relpose = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_RELPOSE);
    van_opts.vis_plot_3pts = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_3DPTS);
    van_opts.vis_plot_relpose_3pts = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_RELPOSE_3DPTS);
    van_opts.vis_plot_summary = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_SUMMARY);
    van_opts.vis_plot_waitkey = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_WAITKEY);
    van_opts.vis_use_saliency = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_USE_SALIENCY);
    //van_opts.manual_corr = bot_gtk_param_widget_get_bool (self->pw, PARAM_NEED_VERIFICATION);
    //van_opts.mdist_plink = bot_gtk_param_widget_get_bool (self->pw, PARAM_USE_MDIST_PLINK);
    van_opts.n_plinks = bot_gtk_param_widget_get_int (self->pw, PARAM_VAN_NPLINKS);

    perllcm_van_options_t_publish (self->lcm, self->m_vanopt_channel, &van_opts);
  }
  else if (self->viewer_mode == PERLLCM_ISAM_CMD_T_MODE_LOAD) {

    // TODO: this is a hack to get only thumbnal 
    bool isthumbnail = false;
    if (msg->width < 500) isthumbnail = true;

    if (isthumbnail) {
      self->imageWidth = msg->width;
      self->imageHeight = msg->height;

      if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
        self->imageBytesPerPixel = 1;
      } else if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16) {
        self->imageBytesPerPixel = 2;
      } else if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {
        self->imageBytesPerPixel = 3;
      } else {
        printf("ERROR: unknown image format\n");
        exit(1);
      }
    
      self->m_imgutimelist.push_back (msg->utime);
      bot_core_image_t *botimg_dup = bot_core_image_t_copy (msg);
      self->m_imglist.push_back (botimg_dup);     
      //printf ("added %ld (%dx%d) rs=%d size=%d pixel=%d\n", msg->utime, msg->width, msg->height, msg->row_stride, msg->size, self->imageBytesPerPixel);
    }
  }
}

static void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *param, void *user_data)
{
    RendererVanctrl *self = (RendererVanctrl*) user_data;
    bot_viewer_request_redraw (self->viewer);

    self->master_mode = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_MASTER);
    if (self->master_mode) {
        perllcm_van_options_t van_opts = {0};
        van_opts.self = 0;
        van_opts.vis_plot_features = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_FEATURES),
        van_opts.vis_plot_scene_prior = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_SP);
        van_opts.vis_plot_put_corr = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_PUTCORR);
        van_opts.vis_plot_search_ellipses = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_ELLIPSES);
        van_opts.vis_plot_in_and_out = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_IN_OUT);
        van_opts.vis_plot_inliers = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_IN);
        van_opts.vis_plot_relpose = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_RELPOSE);
        van_opts.vis_plot_3pts = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_3DPTS);
        van_opts.vis_plot_relpose_3pts = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_RELPOSE_3DPTS);
        van_opts.vis_plot_summary = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_SUMMARY);
        van_opts.vis_plot_waitkey = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_PLOT_WAITKEY);
        van_opts.vis_use_saliency = bot_gtk_param_widget_get_bool (self->pw, PARAM_VAN_USE_SALIENCY);
        van_opts.n_plinks = bot_gtk_param_widget_get_int (self->pw, PARAM_VAN_NPLINKS);
        //van_opts.manual_corr = bot_gtk_param_widget_get_bool (self->pw, PARAM_NEED_VERIFICATION);
        //van_opts.mdist_plink = bot_gtk_param_widget_get_bool (self->pw, PARAM_USE_MDIST_PLINK);

        perllcm_van_options_t_publish (self->lcm, self->m_vanopt_channel, &van_opts);
        // camera on / off
        int camera_enable = bot_gtk_param_widget_get_bool (self->pw, PARAM_CAMERA_ON_OFF);
        if (camera_enable) {
            senlcm_prosilica_attribute_t attr = {0};
            attr.label = (char*)"AcquisitionStart";
            attr.value = (char*)"";

            senlcm_prosilica_t prosilica = {0};
            prosilica.utime = timestamp_now ();
            prosilica.self = 0;
            prosilica.n_attributes = 1;
            prosilica.PvAttributes = &attr;

            senlcm_prosilica_t_publish (self->lcm, "PROSILICA_M.ATTRIBUTES", &prosilica);
        }
        else {
            senlcm_prosilica_attribute_t attr = {0};
            attr.label = (char*)"AcquisitionStop";
            attr.value = (char*)"";

            senlcm_prosilica_t prosilica = {0};
            prosilica.utime = timestamp_now ();
            prosilica.self = 0;
            prosilica.n_attributes = 1;
            prosilica.PvAttributes = &attr;

            senlcm_prosilica_t_publish (self->lcm, "PROSILICA_M.ATTRIBUTES", &prosilica);
        }
    }

    // sample thumbnail
    //self->cur_sampleimg_idx = bot_gtk_param_widget_get_int (self->pw, PARAM_NSAMPLE_IMGS);
    //self->map_merging_cov = (double) bot_gtk_param_widget_get_int (self->pw, PARAM_CONNECT_COV);
}

/* when gl area resized, report it back to keep aspect ratio
 */
static void 
on_glarea_changed (GtkWidget *widget, GtkAllocation *allocation, void *user_data) 
{
    RendererVanctrl *self = (RendererVanctrl*) user_data;

    // get screen width and height for glarea
    self->glarea_width = allocation->width;
    self->glarea_height = allocation->height;
}

static void
_send_start_cmd (RendererVanctrl *self)
{
    //if (!self->master_mode) return;

    char savepath[] = "";
    char loadpath[] = "";
    perllcm_isam_cmd_t p = {0};
    p.mode = PERLLCM_ISAM_CMD_T_MODE_START;
    p.savepath = savepath;
    p.graphfile = loadpath;
    p.load_done = self->load_complete;
    p.utime_conn = self->utime_conn;
    p.cov_conn = self->map_merging_cov;
    perllcm_isam_cmd_t_publish (self->lcm, self->m_isam_cmd_channel, &p);

    self->viewer_mode = PERLLCM_ISAM_CMD_T_MODE_START;
}

static void
on_start_button (GtkWidget *button, void *user_data)
{
  RendererVanctrl *self = (RendererVanctrl*) user_data;
  _send_start_cmd (self);
}

static void
_send_stop_cmd (RendererVanctrl *self)
{
  //if (!self->master_mode) return;

  char savepath[] = "";
  char loadpath[] = "";
  perllcm_isam_cmd_t p = {0};
  p.mode = PERLLCM_ISAM_CMD_T_MODE_WAIT;
  p.savepath = savepath;
  p.graphfile = loadpath;
  perllcm_isam_cmd_t_publish (self->lcm, self->m_isam_cmd_channel, &p);

  self->viewer_mode = PERLLCM_ISAM_CMD_T_MODE_WAIT;

}

static void
on_stop_button (GtkWidget *button, void *user_data)
{
  RendererVanctrl *self = (RendererVanctrl*) user_data;

  _send_stop_cmd (self);
}

/* @todo: change this to isam_state_return
static void
on_se_return_state_t (const lcm_recv_buf_t *buf, const char *channel,
                      const se_return_state_t *msg, void *user_data)
{
    RendererVanctrl *self = (RendererVanctrl*) user_data;
    if (self->viewer_mode == PERLLCM_ISAM_CMD_T_MODE_LOAD
        && msg->state_type & SE_REQUEST_STATE_T_COV_BLOCK) {
        // it was loading a graph and receive the state request
        // which means rtvan has completed its last job to load the graph

        //self->viewer_mode == PERLLCM_ISAM_CMD_T_MODE_WAIT;
        self->load_complete = true;
        int n_nodes = msg->m / 36;
        self->n_loaded_nodes = n_nodes;
        printf ("Done with loading thumbnails for %d nodes\n", n_nodes);

        // automatically check required field
        //bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_ELLIPSES, 1);
        //bot_gtk_param_widget_set_bool (self->pw, PARAM_VAN_PLOT_MASTER, 1);
        //bot_gtk_param_widget_set_bool (self->pw, PARAM_NEED_VERIFICATION, 1);
        bot_gtk_param_widget_set_int (self->pw, PARAM_VAN_NPLINKS, 1);
        bot_viewer_request_redraw (self->viewer);
    }
}
*/

static int 
start_stop_handler (BotViewer *viewer, BotEventHandler *ehandler, 
                    const GdkEventKey *event)
{
    int keyval = event->keyval;

    RendererVanctrl *self = (RendererVanctrl*) g_object_get_data (G_OBJECT(viewer), "viewer:vanctrl");

    switch (keyval)
    {
    case 'S':
    case 's':
        printf ("START\n");
        _send_start_cmd (self);
        break;
    case 'T':
    case 't':
        printf ("STOP\n");
        _send_stop_cmd (self);
        break;
    default:
        return 0;
    }

    return 1;
}

RendererVanctrl *
renderer_vanctrl_new (BotViewer *viewer, char *rootkey)
{
    RendererVanctrl *self = (RendererVanctrl*) calloc (1, sizeof (*self));
    g_assert (self);

    // initialize
    self->viewer = viewer;
    self->renderer.draw = vanctrl_draw;
    self->renderer.destroy = vanctrl_free;
    self->renderer.name = (char*) "vanctrl";
    self->renderer.user = self;
    self->renderer.enabled = 1;
    self->lcm =  bot_lcm_get_global(NULL);
    self->van_opts = (perllcm_van_options_t*) malloc (sizeof (*(self->van_opts)));
    self->cam_attr = (senlcm_prosilica_attribute_t*) malloc (sizeof (*(self->cam_attr)));
    self->master_mode = 0;
    self->glarea_width = 1;
    self->glarea_height = 1;
    self->load_complete = false;
    self->cur_sampleimg_idx = 0;
    self->utime_conn = 0;
    self->n_loaded_nodes = 0;
    self->map_merging_cov = 5.0;

    // read channel names from config
    self->m_param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);

    char camkey[1024];
    char *camera_rootkey = bot_param_get_str_or_fail (self->m_param, "rtvan.cameraUw");
    snprintf (camkey, sizeof camkey, "%s.channel", camera_rootkey);
    if (0!=bot_param_get_str (self->m_param, camkey, &self->m_img_uw_channel))
        throw std::runtime_error("1st camera channel not found in config file");

    int dualcam = botu_param_get_boolean_or_default (self->m_param, "rtvan.dualcam", 0);

    if (dualcam) {
        char *camera_peri_rootkey = botu_param_get_str_or_default (self->m_param, "rtvan.cameraPeri", NULL);
        snprintf (camkey, sizeof camkey, "%s.channel", camera_peri_rootkey);
        if (0!=bot_param_get_str (self->m_param, camkey, &self->m_img_peri_channel))
            std::cout << "2nd camera channel not found in config file";
    }

    if (0!=bot_param_get_str (self->m_param, "rtvan.channel.VAN_OPTIONS_CHANNEL", &self->m_vanopt_channel))
        throw std::runtime_error("rtvan.channel.VAN_OPTIONS_CHANNEL not found in config file");

    if (0!=bot_param_get_str (self->m_param, "isamServer.lcm_channels.CMD_CHANNEL", &self->m_isam_cmd_channel))
        throw std::runtime_error("isamServer.lcm_channels.CMD_CHANNEL not found in config file");

    // subscribe to LCM streams 
    perllcm_van_options_t_subscribe (self->lcm, self->m_vanopt_channel, on_van_opts, self);
    bot_core_image_t_subscribe (self->lcm, self->m_img_uw_channel, on_image_t, self);
    if (dualcam)
        bot_core_image_t_subscribe (self->lcm, self->m_img_peri_channel, on_image_t, self);

    // TODO: load from isam file
    //int n_sample_imgs = 1000;

    if (viewer) {
        //self->renderer.widget = gtk_vbox_new (FALSE, 0);
        self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
        self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
        GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
        gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
        gtk_widget_show (vbox);

        gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw), 
                            FALSE, TRUE, 0);
        gtk_widget_show (GTK_WIDGET (self->pw));

        // check boxes
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_MASTER, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_FEATURES, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_SP, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_PUTCORR, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_ELLIPSES, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_IN_OUT, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_IN, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_RELPOSE, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_3DPTS, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_RELPOSE_3DPTS, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_SUMMARY, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_PLOT_WAITKEY, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_VAN_USE_SALIENCY, 0, NULL);

        // send lcm messeage to start the camera (using PvAttr)
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_CAMERA_ON_OFF, 1, NULL);

        //--------------------------------------------------------------------- 
        bot_gtk_param_widget_add_separator (self->pw, "");
        // slider (min, max, step, default)
        bot_gtk_param_widget_add_int (self->pw, PARAM_VAN_NPLINKS, BOT_GTK_PARAM_WIDGET_SLIDER, 0, MAX_NPLINKS, 1, 3);      // link proposal #
        // bot_gtk_param_widget_add_int (self->pw, PARAM_NSAMPLE_IMGS, BOT_GTK_PARAM_WIDGET_SLIDER, 0, n_sample_imgs, 1, 0);   // sample img #
        // bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_NEED_VERIFICATION, 0, NULL);      // need visual verification?
        // bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_USE_MDIST_PLINK, 0, NULL);        // use mdist to propose links
        // bot_gtk_param_widget_add_int (self->pw, PARAM_CONNECT_COV, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 20, 1, 5);               // covariance to connect maps

        // START | STOP
        //--------------------------------------------------------------------- 
        bot_gtk_param_widget_add_separator (self->pw, "");
        // send start/stop (lut option) command to seserver and rtvan
        GtkWidget *start_button = gtk_button_new_with_label ("Start");
        gtk_box_pack_start (GTK_BOX (vbox), start_button, FALSE, FALSE, 0);
        g_signal_connect (G_OBJECT (start_button), "clicked", G_CALLBACK (on_start_button), self);
        //bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_SE_OPTION_LUT, 0, NULL);

        GtkWidget *stop_button = gtk_button_new_with_label ("Stop");
        gtk_box_pack_start (GTK_BOX (vbox), stop_button, FALSE, FALSE, 0);
        g_signal_connect (G_OBJECT (stop_button), "clicked", G_CALLBACK (on_stop_button), self);

        //--------------------------------------------------------------------- 
        gtk_widget_show_all (self->renderer.widget);

        g_signal_connect(G_OBJECT (&(self->viewer->gl_area->area.widget)), 
                                   "size-allocate", G_CALLBACK(on_glarea_changed), self);

        g_signal_connect (G_OBJECT (self->pw), "changed",
                          G_CALLBACK (on_param_widget_changed), self);

        // logplayer controls
        BotEventHandler* ehandler = (BotEventHandler*) calloc (1, sizeof(*ehandler));
        ehandler->name = (char*)"START/STOP";
        ehandler->enabled = 1;
        ehandler->key_press = start_stop_handler;
        bot_viewer_add_event_handler (viewer, ehandler, 0);

        g_object_set_data (G_OBJECT(viewer), "viewer:vanctrl", self);
    }

    return self;
}

void
setup_renderer_vanctrl (BotViewer *viewer, char *rootkey, int render_priority)
{
    RendererVanctrl *self = renderer_vanctrl_new (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &(self->renderer), render_priority);
}

