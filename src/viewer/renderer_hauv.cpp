/*
 * renders a hauv trajectory
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <vector>
#include <map>
#include <string>
#include <stdint.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_vis/bot_vis.h>

#include <GL/glu.h>
#include <GL/gl.h>

// lcmtypes
#include "perls-lcmtypes/hauv_vehicle_state_t.h"
#include "perls-lcmtypes/hauv_bs_rnv_t.h"
#include "perls-lcmtypes/hauv_bs_rnv_2_t.h"
#include "perls-lcmtypes/hauv_bs_cnv_t.h"
#include "perls-lcmtypes/hauv_bs_nvg_t.h"
#include "perls-lcmtypes/hauv_pl_gbp_t.h"
#include "perls-lcmtypes/hauv_didson_t.h"

#include "perls-lcmtypes/hauv_wp_goto_t.h"
#include "perls-lcmtypes/hauv_wp_goto_request_t.h"
#include "perls-lcmtypes/hauv_wp_save_t.h"

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/senlcm_prosilica_t.h"
#include "perls-lcmtypes/perllcm_isam_cmd_t.h"
#include "perls-lcmtypes/perllcm_isam_info_t.h"

#include "perls-lcmtypes/perllcm_isam_graph_vis_t.h" // replaces vehicle state.h
#include "perls-lcmtypes/perllcm_isam_info_t.h"

// perls lib
#include "perls-common/units.h"
#include "perls-common/error.h"
#include "perls-common/bot_util.h"

#include "renderers.h"

// Position of ship model
const char* PARAM_SHIP_X = "Ship - X";
const double PARAM_SHIP_X_MIN = -100.;
const double PARAM_SHIP_X_MAX = +100.;
const double PARAM_SHIP_X_DELTA = 0.01;
const double PARAM_SHIP_X_DEFAULT = 0.;

const char* PARAM_SHIP_Y = "Ship - Y";
const double PARAM_SHIP_Y_MIN = -100.;
const double PARAM_SHIP_Y_MAX = +100.;
const double PARAM_SHIP_Y_DELTA = 0.01;
const double PARAM_SHIP_Y_DEFAULT = 0.;

const char* PARAM_SHIP_Z = "Ship - Z";
const double PARAM_SHIP_Z_MIN = -50.;
const double PARAM_SHIP_Z_MAX = +50.;
const double PARAM_SHIP_Z_DELTA = 0.01;
const double PARAM_SHIP_Z_DEFAULT = 0.;

const char* PARAM_SHIP_HEADING = "Ship - Heading";
const double PARAM_SHIP_HEADING_MIN = -180.;
const double PARAM_SHIP_HEADING_MAX = +180.;
const double PARAM_SHIP_HEADING_DELTA = 0.1;
const double PARAM_SHIP_HEADING_DEFAULT = 0.;

const char* PARAM_SHIP_S = "Ship - Scale";
const double PARAM_SHIP_S_MIN = 0.01;
const double PARAM_SHIP_S_MAX = 1000.;
const double PARAM_SHIP_S_DELTA = 0.01;
const double PARAM_SHIP_S_DEFAULT = 1;

#define DRAW_COORDINATE_AXIS 0
#define GROUND_LEVEL 0.0

// params
#define PARAM_SHOW_DIDSON           "DIDSON Frame"
#define PARAM_SHOW_IMAGE            "Camera Frame"
#define PARAM_SHOW_INFO             "iSam Info"
#define PARAM_SHOW_DVL_TRAJECTORY   "DR trajectory"
#define PARAM_SHOW_DVL_BOTTOM       "DVL bottom"
#define PARAM_TRAJ_CNV              "6dof cartesian? (or hull ref)"
#define PARAM_SHOW_SHIP             "Ship Model"

// lcm channels
#define ISAM_INFO_CHANNEL            "ISAM_INFO"
#define VAN_PLOT_CAM_TARGET          "VAN_PLOT_CAM_TARGET"        // target image via bot image


// @todo: read these from config
#define WP_GOTO_RQ_CHANNEL           "WP_GOTO_RQ"
#define WP_SAVE_CMD_CHANNEL          "SAVE_WP"

#define RENDERER_NAME "Hauv"

typedef struct _RendererHauv RendererHauv;

struct _RendererHauv {
    BotRenderer        renderer;
    BotViewer          *viewer;
    BotGtkParamWidget  *pw;
    GtkWidget          *label;
    lcm_t              *lcm;
    BotParam           *param;

    // wavefront model
    BotWavefrontModel *wfm_ship;
    int                wavefront_dl_ready_ship;
    GLuint             wavefront_dl_ship;
    int                display_dl_ready;
    GLuint             list_dl;

    // lcm messages
    hauv_bs_rnv_2_t           *rnv_data;
    hauv_didson_t             *didson_data;
    perllcm_isam_graph_vis_t  *igv;
    hauv_pl_gbp_t             gbp_history;


    // camera image rendering
    int imageWidth, imageHeight;
    int imageBytesPerPixel;
    unsigned char imageData[1360*1024*6];
    int targetimgWidth, targetimgHeight;
    int targetimgBytesPerPixel;
    unsigned char targetimgData[1360*1024*6];

    // camera attributes rendering
    double cam_fps;         // frame per second in string
    int cam_gain;           // camera gain in string
    int cam_exp;            // camera exposure time [ms] in string
    double camoffset; 
    int glarea_width;       // render image with aspect ratio
    int glarea_height;


    // @todo remove these stat related members
    hauv_vehicle_state_t vehicleState;
    hauv_vehicle_state_t vehicleStateHistory[50000];
    int vehicleStateCount;
    int stateCount;
    hauv_bs_rnv_2_t rnv_history[50000];
    int rnv_count;

    // pertaining to dead reckoning and height from bottom (from dvl)
    int hasNewXyz;
    int hasNewAlt;
    double currentX;
    double currentY;
    double currentZ;
    double currentAlt;

    // @todo fix this save/load related viewer mode
    int viewer_mode;

    // waypoint navigation user interface
    // waypoint ID, valid during execution of goto
    // Connects the radio buttons to waypoint id's
    std::map<GtkWidget*, int>   waypoints;
    int     waypoint_max_id;
    int64_t resume_waypoint;
    int     waypoint_goto_id;
    int     waypoint_selected_id;
    int     goto_mode; // goto mode on?

    std::map<std::string, std::string> info;

    // ship drawing
    double shipTransform[6]; // x,y,z,yaw,pitch,roll
    GLuint shiplist;         // SHIP model
    double param_ship_x;
    double param_ship_y;
    double param_ship_z;
    double param_ship_heading;
    double param_ship_s;

    //Stack of xyz, altitude used for drawing dead reckoning and height from bottom
    std::vector<double *> xyzHfb;

    // waypoint status
    std::string status_text;
};


struct WaypointButton
{
    WaypointButton(RendererHauv* renderer, int32_t waypoint_id)
                  : renderer(renderer), waypoint_id(waypoint_id) {}

    RendererHauv* renderer;
    int32_t waypoint_id;
};

// Image type - contains height, width, and data
struct Image {
    unsigned long sizeX;
    unsigned long sizeY;
    char *data;
};
typedef struct Image Image;

static void
draw_wavefront_model (GLuint wf_dl)
{
    glPushMatrix ();
    glEnable (GL_BLEND);
    glEnable (GL_RESCALE_NORMAL);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);
    glEnable (GL_COLOR_MATERIAL);
    glCallList (wf_dl);
    glPopMatrix ();
}

static GLuint
compile_wavefront_display_list (RendererHauv *self, BotWavefrontModel *model)
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

// display info and status
static void
render_info(BotViewer *viewer, BotRenderer *renderer)
{
    RendererHauv *self = (RendererHauv*) renderer->user;
    g_assert (self->viewer == viewer);

    std::string text;
    for (std::map<std::string, std::string>::iterator it = self->info.begin(); it != self->info.end(); it++) {
      text += it->first;
      text += ": ";
      text += it->second;
      text += "\n";
    }

    glColor3f (1.0,0.0,0.0);
    double font_pos[3] = {0., 0.98, 0};
    bot_gl_draw_text (font_pos, GLUT_BITMAP_HELVETICA_12, text.c_str(), BOT_GL_DRAW_TEXT_ANCHOR_HCENTER | BOT_GL_DRAW_TEXT_ANCHOR_TOP);//16);
}

static void
render_status(BotViewer *viewer, BotRenderer *renderer)
{
    RendererHauv *self = (RendererHauv*) renderer->user;
    g_assert (self->viewer == viewer);

    glColor3f (1.0,0.0,0.0);
    double font_pos[3] = {0., -0.95, 0};
    bot_gl_draw_text (font_pos, GLUT_BITMAP_HELVETICA_18, self->status_text.c_str(), BOT_GL_DRAW_TEXT_JUSTIFY_CENTER | BOT_GL_DRAW_TEXT_ANCHOR_HCENTER | BOT_GL_DRAW_TEXT_ANCHOR_BOTTOM);
}

// draw components

static void
draw_ship (RendererHauv *self)
{
    if (!self)
        return;

    glPushMatrix();
    glTranslatef(0.,0., GROUND_LEVEL);  // everything w.r.t. the ground level
    self->shipTransform[0] = self->param_ship_x;
    self->shipTransform[1] = self->param_ship_y;
    self->shipTransform[2] = self->param_ship_z;

    // Draw vehicle model
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_SHIP)) {
        glPushMatrix();
          // adjust by slide bar
          glTranslatef(self->shipTransform[0], self->shipTransform[1], self->shipTransform[2]);
          glRotatef(self->param_ship_heading,0.0,0.0,1.0);

          if (!self->wavefront_dl_ready_ship) { // setup wavefront if its there
              if (self->wfm_ship)
                self->wavefront_dl_ship = compile_wavefront_display_list (self, self->wfm_ship);
              self->wavefront_dl_ready_ship = 1;
          }

          glScalef(1.0/self->param_ship_s, 1.0/self->param_ship_s, 1.0/self->param_ship_s);
          // glScalef(100.0*0.3048,100.0*0.3048,100.0*0.3048); // Red-cloud model
          // glScalef(0.001,0.001,0.001); // Cutter model
          glColor3f(0.5,0.5,0.5);     // draw HAUV in gray

          glEnable(GL_LIGHTING);
          if (self->wavefront_dl_ready_ship && self->wavefront_dl_ship) // draw wavefront
              draw_wavefront_model (self->wavefront_dl_ship);
          glDisable(GL_LIGHTING);
        glPopMatrix();
    }
    glPopMatrix();
}

static void
draw_dead_reckoning (RendererHauv *self)
{

    if (!self)
        return;

    glPushMatrix();
    glRotatef(180.0,1.0,0.0,0.0);
    glTranslatef(0.,0., GROUND_LEVEL);

    // Draw dead-reckoning trajectory
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_DVL_TRAJECTORY)) {
        glEnableClientState(GL_VERTEX_ARRAY);
        float *path;
        int *pathIdx;
        path = (float*)malloc (sizeof(float)*3*self->xyzHfb.size());
        pathIdx = (int*)malloc (sizeof(int)*self->xyzHfb.size());
        for (uint32_t i=0; i<self->xyzHfb.size(); i++) {
            path[3*i + 0] = (float)self->xyzHfb[i][0];
            path[3*i + 1] = (float)self->xyzHfb[i][1];
            path[3*i + 2] = (float)self->xyzHfb[i][2];
            pathIdx[i] = i;
            //printf("%f\n", self->xyzHfb[i][3]);
        }
        glVertexPointer(3, GL_FLOAT, 0, path);

        glColor3f(0.0,1.0,1.0);
        glDrawElements(GL_LINE_STRIP, self->xyzHfb.size(), GL_UNSIGNED_INT, pathIdx);

        free(path);
        free(pathIdx);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
  
    glPopMatrix ();
}

static void
draw_dvl_bottom (RendererHauv *self)
{
    if (!self)
        return;

    glPushMatrix();
    glRotatef(180.0,1.0,0.0,0.0);
    glTranslatef(0.,0., GROUND_LEVEL);

    // Draw dead-reckoning trajectory
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_DVL_BOTTOM)) {
        glEnableClientState(GL_VERTEX_ARRAY);
        float *path;
        int *pathIdx;
        path = (float*)malloc (sizeof(float)*3*self->xyzHfb.size());
        pathIdx = (int*)malloc (sizeof(int)*self->xyzHfb.size());
        for (uint32_t i=0; i<self->xyzHfb.size(); i++) {
            path[3*i + 0] = (float)self->xyzHfb[i][0];
            path[3*i + 1] = (float)self->xyzHfb[i][1];
            path[3*i + 2] = (float)self->xyzHfb[i][2] + (float)self->xyzHfb[i][3];
            pathIdx[i] = i;
        }
        glVertexPointer(3, GL_FLOAT, 0, path);

        glColor3f(0.75,0.75,0.75);
        glDrawElements(GL_LINE_STRIP, self->xyzHfb.size(), GL_UNSIGNED_INT, pathIdx);

        free(path);
        free(pathIdx);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
  
    glPopMatrix ();
}

static void
render_image (BotViewer *viewer, BotRenderer *renderer)
{
    GLuint texName;

    RendererHauv *self = (RendererHauv*) renderer->user;
    g_assert (self->viewer == viewer);

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
    int gl_format = (self->imageBytesPerPixel==3) ? GL_RGB : GL_INTENSITY16;
    int gl_size = (self->imageBytesPerPixel==1) ? GL_UNSIGNED_BYTE : GL_UNSIGNED_SHORT;
    int gl_internalformat = (self->imageBytesPerPixel==3) ? GL_RGB : GL_LUMINANCE;
    glTexImage2D (GL_TEXTURE_2D, 0, gl_format, self->imageWidth, self->imageHeight,
                  0, gl_internalformat, gl_size, self->imageData);
    
    glEnable (GL_TEXTURE_2D);
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture (GL_TEXTURE_2D, texName);

    glColor3f (1.0,1.0,1.0);
    glBegin (GL_QUADS);
      glTexCoord2f (0.0, 0.0); glVertex3f (0.5, -1+0.5*ratio, 0);
      glTexCoord2f (1.0, 0.0); glVertex3f (1,   -1+0.5*ratio, 0);
      glTexCoord2f (1.0, 1.0); glVertex3f (1,   -1,   0);
      glTexCoord2f (0.0, 1.0); glVertex3f (0.5, -1,   0);
    glEnd ();
    glDisable (GL_TEXTURE_2D);

    // plot camera current params on image
    char camparam_str[256];
    snprintf (camparam_str, sizeof camparam_str, 
              "fps=%g, exp=%d, gain=%d, offset=%g", 
              self->cam_fps, self->cam_exp, self->cam_gain, self->camoffset);
    glPushMatrix ();
      glTranslated (0.52,-1+0.48*ratio,0);
      double font_pos[3]={0.0 ,0.0, 0.0};
      glColor4f (1.0f, 0.0f, 0.0f, 1.0f);
      bot_gl_draw_text (font_pos, GLUT_BITMAP_HELVETICA_12, camparam_str, 16);
    glPopMatrix ();

    glDeleteTextures (1, &texName);
}

static void
render_target_img (BotViewer *viewer, BotRenderer *renderer)
{
    GLuint texName;

    RendererHauv *self = (RendererHauv*) renderer->user;
    g_assert (self->viewer == viewer);

    // keep image original apsect ratio on viewer
    float glarea_ratio = (float) self->glarea_width / (float) self->glarea_height;
    float image_ratio = (float) self->targetimgHeight / (float) self->targetimgWidth;
    float ratio = glarea_ratio * image_ratio;

    // Setup texture parameters
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glGenTextures (1, &texName);
    glBindTexture (GL_TEXTURE_2D, texName);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    int gl_format = (self->targetimgBytesPerPixel==3) ? GL_RGB : GL_INTENSITY16;
    int gl_size = (self->targetimgBytesPerPixel==1) ? GL_UNSIGNED_BYTE : GL_UNSIGNED_SHORT;
    int gl_internalformat = (self->targetimgBytesPerPixel==3) ? GL_RGB : GL_LUMINANCE;
    glTexImage2D (GL_TEXTURE_2D, 0, gl_format, self->targetimgWidth, self->targetimgHeight,
                  0, gl_internalformat, gl_size, self->targetimgData);
    
    glEnable (GL_TEXTURE_2D);
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture (GL_TEXTURE_2D, texName);

    glColor3f (1.0,1.0,1.0);
    glBegin (GL_QUADS);
      glTexCoord2f (0.0, 0.0); glVertex3f (-1, -1+0.5*ratio, 0);
      glTexCoord2f (1.0, 0.0); glVertex3f (-0.5,   -1+0.5*ratio, 0);
      glTexCoord2f (1.0, 1.0); glVertex3f (-0.5,   -1,   0);
      glTexCoord2f (0.0, 1.0); glVertex3f (-1, -1,   0);
    glEnd ();
    glDisable (GL_TEXTURE_2D);

    glDeleteTextures (1, &texName);
}

static void
render_didson(BotViewer *viewer, BotRenderer *renderer)
{
    GLuint texName;
    int sonarWidth, sonarHeight;
    unsigned char* sonarData;

    RendererHauv *self = (RendererHauv*) renderer->user;
    g_assert (self->viewer == viewer);

    //printf("%d %d %d %d\n", viewport[0], viewport[1], viewport[2], viewport[3]); 

    sonarWidth = 96;
    sonarHeight = 512;
    sonarData = self->didson_data->m_cData;

    // Setup texture parameters
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &texName);
    glBindTexture(GL_TEXTURE_2D, texName);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_INTENSITY8, sonarWidth, sonarHeight,
        0, GL_LUMINANCE, GL_UNSIGNED_BYTE, sonarData);

    glEnable(GL_TEXTURE_2D);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture(GL_TEXTURE_2D, texName);

    glColor3f(1.0,1.0,1.0);
    glBegin(GL_QUADS);
      glTexCoord2f(0.0, 0.0); glVertex3f(0.5,0.25,0);
      glTexCoord2f(1.0, 0.0); glVertex3f(1,  0.25,0);
      glTexCoord2f(1.0, 1.0); glVertex3f(1,  1,   0);
      glTexCoord2f(0.0, 1.0); glVertex3f(0.5,1,   0);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    glDeleteTextures (1, &texName);
}

static void
hauv_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererHauv *self = (RendererHauv*) renderer->user;
    g_assert (self->viewer == viewer);

    // draw 3D scene
    glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT);
    glEnable (GL_DEPTH_TEST);
    glDepthFunc (GL_LESS);
    glDisable (GL_LIGHTING);
    glLineWidth (1);
    draw_ship (self);
    draw_dead_reckoning (self);
    draw_dvl_bottom (self);
    glPopAttrib ();

    // draw 2D stuff
    GLint viewport[4];
    glGetIntegerv (GL_VIEWPORT, viewport);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1, 1, -1, 1, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glPushAttrib (GL_DEPTH_BUFFER_BIT);
    glDisable (GL_DEPTH_TEST);

    // render current DIDSON frame
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_DIDSON))
        render_didson(viewer, renderer);

    // render current camera frame
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_IMAGE) && self->imageWidth>0)
        render_image(viewer, renderer);

    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_INFO))
        render_info(viewer, renderer);

    if (self->goto_mode == HAUV_WP_GOTO_T_MODE_GOTO)
        render_target_img(viewer, renderer);

    if (self->goto_mode == HAUV_WP_GOTO_T_MODE_GOTO 
        || self->goto_mode == HAUV_WP_GOTO_T_MODE_GOTO_REL
        || self->goto_mode == HAUV_WP_GOTO_T_MODE_RESUME)
        render_status(viewer, renderer);

    glPopAttrib();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

static void
hauv_free (BotRenderer *renderer)
{

    if (!renderer) return;
    RendererHauv *self = (RendererHauv*) renderer->user;
    if (!self) return;

    if (self->wfm_ship)
        bot_wavefront_model_destroy (self->wfm_ship);

    delete self;
}

// full size live image stream
static void 
on_IMAGE_data (const lcm_recv_buf_t *rbuf, const char *channel, 
               const bot_core_image_t *msg, void *user_data ) 
{
    RendererHauv *self = (RendererHauv*)user_data;

    //if (self->viewer_mode != SE_OPTION_T_MODE_LOAD) {
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_IMAGE)
       && msg->width > 500) 
    {
        self->imageWidth = msg->width;
        self->imageHeight = msg->height;
        if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
            self->imageBytesPerPixel = 1;
        } 
        else if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16) {
            self->imageBytesPerPixel = 2;
        } 
        else if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {
            self->imageBytesPerPixel = 3;
        } 
        else {
            printf("ERROR: unknown image format\n");
            exit(1);
        }
        memcpy(self->imageData, msg->data, msg->width*msg->height*self->imageBytesPerPixel);
    }
}

// thumbnail for map merging
static void 
on_TARGET_IMG_data (const lcm_recv_buf_t *rbuf, const char *channel, 
                    const bot_core_image_t *msg, void *user_data ) 
{
    RendererHauv *self = (RendererHauv*)user_data;

    self->targetimgWidth = msg->width;
    self->targetimgHeight = msg->height;
    if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
       self->targetimgBytesPerPixel = 1;
    } 
    else if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16) {
       self->targetimgBytesPerPixel = 2;
    } 
    else if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {
       self->targetimgBytesPerPixel = 3;
    } 
    else {
        printf("ERROR: unknown image format\n");
        exit(1);
    }
    memcpy(self->targetimgData, msg->data, msg->width*msg->height*self->targetimgBytesPerPixel);
}

// camera attribute
static void 
on_camatt_data (const lcm_recv_buf_t *rbuf, const char *channel, 
                const senlcm_prosilica_t *msg, void *user_data ) 
{
    RendererHauv *self = (RendererHauv*)user_data;
    for (int i=0; i<msg->n_attributes; i++) {
        const char *label = msg->PvAttributes[i].label;
        const char *value = msg->PvAttributes[i].value;
        if (0==strcmp (label, "StatFrameRate"))
            self->cam_fps = atof (value);
        else if (0==strcmp (label, "ExposureValue"))
            self->cam_exp = atoi (value);
        else if (0==strcmp (label, "GainValue"))
            self->cam_gain = atoi (value);
    }
}

// didson data
static void 
on_DIDSON_data (const lcm_recv_buf_t *rbuf, const char *channel, 
                const hauv_didson_t *msg, void *user_data ) 
{
    RendererHauv *self = (RendererHauv*) user_data;
    memcpy(self->didson_data, msg, sizeof(*msg));

    // float m_fSonarX; // added 26 December 2007
    // float m_fSonarY; // added 26 December 2007
    // float m_fSonarZ; // added 6 March 2008 for DDF_04
    // int32_t m_nWindowStart; // 1-31 CW meters = N*Incr, Incr = .375 (HF) or 0.75 (LF)
    // XW meters = N*Incr, Incr = .42 (HF) or 0.84 (LF)
    // int32_t m_nWindowLength; // 0-3 CW (1.125, 2.25, 4.5, 9 m) HF or (4.5, 9, 18, 36 m) LF
}

// rnv2 data callback
static void 
on_RNV_2_data (const lcm_recv_buf_t *rbuf, const char *channel, 
               const hauv_bs_rnv_2_t *msg, void *user_data ) 
{
    RendererHauv *self = (RendererHauv*) user_data;

    // need offset for seserver mode & dvl mode
    self->camoffset = msg->distance;  

    /*if (bot_gtk_param_widget_get_bool(self->pw, PARAM_TRAJ_CNV)) {
      self->state.heading = msg->absheading;
      self->state.roll = msg->absroll;
      self->state.pitch = msg->abspitch;
    }
    else {
      self->state.x = -msg->vertical;
      self->state.y =  msg->horizontal;
      self->state.heading =  msg->heading;
    }

    memcpy(&(self->rnv_history[self->rnv_count]), msg, sizeof(*msg));
    self->rnv_count++;
    if (self->rnv_count>49999) self->rnv_count = 0;*/

    bot_viewer_request_redraw (self->viewer);
}

static void 
on_RNV_data (const lcm_recv_buf_t *rbuf, const char *channel, 
             const hauv_bs_rnv_t *msg, void *user_data )
{
    //RendererHauv *self = (RendererHauv*) user_data;

    // For support for older log files we convert RNV into RNV2 as far as that is possible
    hauv_bs_rnv_2_t msg2;
    msg2.time_received = msg->time_received;
    msg2.time = msg->time;
    msg2.horizontal = msg->horizontal;
    msg2.vertical = msg->vertical;
    msg2.distance = msg->distance;
    msg2.heading = msg->heading;
    //msg2.depth = self->state.z;        // Should be from NVG msg
    msg2.absheading = msg->heading;    // Should be NVG heading
    //msg2.absroll = self->state.roll;   // Should be from NVG msg
    //msg2.abspitch = self->state.pitch; // Should be from NVG msg
    msg2.time_nav = msg->time;

    on_RNV_2_data (rbuf, channel, &msg2, user_data);
}

static void 
on_CNV_data (const lcm_recv_buf_t *rbuf, const char *channel, 
             const hauv_bs_cnv_t *msg, void *user_data )
{
    RendererHauv *self = (RendererHauv*) user_data;

    if (!self->hasNewXyz) {
        self->currentX = msg->x;
        self->currentY = msg->y;
        self->currentZ = msg->z;
        self->hasNewXyz = true;
    }

    if (self->hasNewAlt) {
        double *d = new double[4];
        d[0] = self->currentX;
        d[1] = self->currentY;
        d[2] = self->currentZ;
        d[3] = self->currentAlt;
        self->xyzHfb.push_back(d);
        self->hasNewXyz = false;
        self->hasNewAlt = false;
        bot_viewer_request_redraw (self->viewer);
    }
}

//@Todo: why is msg->altitude always 0?
static void
on_NVG_data (const lcm_recv_buf_t *rbug, const char *channel,
             const hauv_bs_nvg_t *msg, void *user_data)
{
    RendererHauv *self = (RendererHauv*) user_data;

    if (!self->hasNewAlt) {
        self->currentAlt = msg->altitude;
        self->hasNewAlt = true;
    }

    if (self->hasNewXyz) {
        double *d = new double[4];
        d[0] = self->currentX;
        d[1] = self->currentY;
        d[2] = self->currentZ;
        d[3] = self->currentAlt;
        self->xyzHfb.push_back(d);
        self->hasNewXyz = false;
        self->hasNewAlt = false;
        bot_viewer_request_redraw (self->viewer);
    }
}

static void 
on_GBP_data (const lcm_recv_buf_t *rbuf, const char *channel, 
             const hauv_pl_gbp_t *msg, void *user_data ) 
{
    RendererHauv *self = (RendererHauv*) user_data;
    memcpy(&(self->gbp_history), msg, sizeof(*msg));
}

// wp related functions
static void
on_isam_info(const lcm_recv_buf_t *rbuf, const char *channel,
             const perllcm_isam_info_t *msg, void *user_data)
{
    RendererHauv *self = (RendererHauv*) user_data;
    //std::cout <<"got info == " << msg->data << std::endl;
    self->status_text = msg->data;
}

static void
on_wp_goto_t (const lcm_recv_buf_t *rbuf, const char *channel,
              const hauv_wp_goto_t *msg, void *user_data)
{
    RendererHauv *self = (RendererHauv*) user_data;

    self->goto_mode = msg->mode;
}

// GUI inputs
static void
on_waypoint_save_button(GtkWidget *button, RendererHauv *self)
{
    int64_t recent_time = self->igv->node_id[self->igv->nnodes-1];

    // save the current utime
    hauv_wp_save_t s;
    s.utime = recent_time;
    s.waypoint_id = self->waypoint_selected_id;
    hauv_wp_save_t_publish (self->lcm, WP_SAVE_CMD_CHANNEL, &s);

    /*
    self->saved_waypoints[self->waypoint_next_id] = self->state.utime;

    hauv_wp_save_t s;
    s.utime = self->state.utime;
    s.waypoint_id = self->waypoint_next_id;
    hauv_wp_save_t_publish (self->lcm, "SE_SAVE_WAYPOINT", &s);

    self->waypoint_next_id = (self->waypoint_next_id + 1) % self->waypoint_max_id;
    */
  
}

static void
on_waypoint_goto_button(GtkWidget *button, RendererHauv *self)
{
    self->waypoint_goto_id = self->waypoint_selected_id;

    // request goto
    hauv_wp_goto_request_t rq_g;
    rq_g.mode = HAUV_WP_GOTO_T_MODE_GOTO;
    rq_g.waypoint_id = self->waypoint_selected_id;
    hauv_wp_goto_request_t_publish (self->lcm, WP_GOTO_RQ_CHANNEL, &rq_g);
}

static void
on_waypoint_goto_relative_button(GtkWidget *button, RendererHauv *self)
{
    self->waypoint_goto_id = self->waypoint_selected_id;

    hauv_wp_goto_request_t rq_g;
    rq_g.mode = HAUV_WP_GOTO_T_MODE_GOTO_REL;
    rq_g.waypoint_id = self->waypoint_selected_id;
    hauv_wp_goto_request_t_publish (self->lcm, WP_GOTO_RQ_CHANNEL, &rq_g);
}

static void
on_waypoint_resume_button(GtkWidget *button, RendererHauv *self)
{
    hauv_wp_goto_request_t rq_g;
    rq_g.mode = HAUV_WP_GOTO_T_MODE_RESUME;
    rq_g.waypoint_id = -1; //self->resume_waypoint;
    hauv_wp_goto_request_t_publish (self->lcm, WP_GOTO_RQ_CHANNEL, &rq_g);
}

static void
on_waypoint_delete_resume_button(GtkWidget *button, RendererHauv *self)
{
    // delete resume point from viewer
    //self->resume_waypoint = 0;
    hauv_wp_save_t s;
    s.utime = 0;
    s.waypoint_id = -1;
    hauv_wp_save_t_publish (self->lcm, WP_SAVE_CMD_CHANNEL  , &s);
}

static void
on_waypoint_off_button(GtkWidget *button, RendererHauv *self)
{
    hauv_wp_goto_request_t rq_g;
    rq_g.mode = HAUV_WP_GOTO_T_MODE_OFF;
    rq_g.waypoint_id = 0;
    hauv_wp_goto_request_t_publish (self->lcm, WP_GOTO_RQ_CHANNEL, &rq_g);
}


static void
on_waypoint_button_click(GtkWidget *button, RendererHauv *self)
{
    if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (button)))
    {
        self->waypoint_selected_id = self->waypoints[button];
    }
}

static void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *param,
                         void *user_data)
{
    RendererHauv *self = (RendererHauv*) user_data;

    // camera slider history
    self->param_ship_x = bot_gtk_param_widget_get_double(self->pw, PARAM_SHIP_X);
    self->param_ship_y = bot_gtk_param_widget_get_double(self->pw, PARAM_SHIP_Y);
    self->param_ship_z = bot_gtk_param_widget_get_double(self->pw, PARAM_SHIP_Z);
    self->param_ship_heading = bot_gtk_param_widget_get_double(self->pw, PARAM_SHIP_HEADING);
    self->param_ship_s = bot_gtk_param_widget_get_double(self->pw, PARAM_SHIP_S);

    bot_viewer_request_redraw (self->viewer);
}

// when gl area resized, report it back to keep aspect ratio
static void 
on_glarea_changed (GtkWidget *widget, GtkAllocation *allocation, void *user_data) 
{
    RendererHauv *self = (RendererHauv*) user_data;

    // get screen width and height for glarea
    self->glarea_width = allocation->width;
    self->glarea_height = allocation->height;
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererHauv *self = (RendererHauv*)user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererHauv *self = (RendererHauv*)user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
graph_cb (const lcm_recv_buf_t *rbuf, const char *channel,
          const perllcm_isam_graph_vis_t *msg, void *user)
{    
    RendererHauv *self = (RendererHauv*) user;
    
    if (self->igv)
        perllcm_isam_graph_vis_t_destroy (self->igv);
    
    self->igv = perllcm_isam_graph_vis_t_copy (msg);
}


RendererHauv *
renderer_hauv_new (BotViewer *viewer, char *rootkey)
{

    RendererHauv *self = new RendererHauv ();
    g_assert (self);

    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    self->viewer = viewer;
    self->renderer.draw = hauv_draw;
    self->renderer.destroy = hauv_free;
    self->renderer.name = (char*) "Hauv";
    self->renderer.user = self;
    self->renderer.enabled = 1;
    self->display_dl_ready = 0;
    self->rnv_count = 0;
    self->stateCount = 0;
    self->lcm =  bot_lcm_get_global(NULL);

    self->rnv_data = (hauv_bs_rnv_2_t*)malloc(sizeof(*(self->rnv_data)));
    self->didson_data = (hauv_didson_t*)malloc(sizeof(*(self->didson_data)));
    self->vehicleStateCount = 0;
    
    // parse model
    self->param_ship_x = PARAM_SHIP_X_DEFAULT;
    self->param_ship_y = PARAM_SHIP_Y_DEFAULT;
    self->param_ship_z = PARAM_SHIP_Z_DEFAULT;
    self->param_ship_heading = PARAM_SHIP_HEADING_DEFAULT;
    self->param_ship_s = PARAM_SHIP_S_DEFAULT;

    char key_ship[256] = {0};
    char *wavefront_fname_ship = NULL;
    snprintf (key_ship, sizeof key_ship, "viewer.renderers.%s.ship_obj_file", rootkey);
    bot_param_get_str (self->param, key_ship, &wavefront_fname_ship);

    if (wavefront_fname_ship) { 
        self->wfm_ship = bot_wavefront_model_create (wavefront_fname_ship);
        free (wavefront_fname_ship);
    }

    // initialize for camera plot
    self->imageWidth = 0;
    self->imageHeight = 0;
    self->glarea_width = 1;
    self->glarea_height = 1;
    self->goto_mode = HAUV_WP_GOTO_T_MODE_OFF;

    //For DVL dead reckoning
    self->hasNewAlt = 0;
    self->hasNewXyz = 0;

    self->waypoint_selected_id = 0;
    self->waypoint_goto_id = -1;
    self->waypoint_max_id = 10;
    self->resume_waypoint = 0;

    /* subscribe to LCM streams */
    hauv_bs_rnv_t_subscribe(self->lcm, "HAUV_BS_RNV", on_RNV_data, self);
    hauv_bs_rnv_2_t_subscribe(self->lcm, "HAUV_BS_RNV_2", on_RNV_2_data, self);
    hauv_pl_gbp_t_subscribe(self->lcm, "HAUV_PL_GBP", on_GBP_data, self);
    hauv_didson_t_subscribe(self->lcm, "HAUV_DIDSON_FRAME", on_DIDSON_data, self);
    bot_core_image_t_subscribe(self->lcm, "PROSILICA_PERI", on_IMAGE_data, self);
    bot_core_image_t_subscribe(self->lcm, "PROSILICA_UW", on_IMAGE_data, self);
    bot_core_image_t_subscribe(self->lcm, "PROSILICA_M", on_IMAGE_data, self);
    bot_core_image_t_subscribe(self->lcm, VAN_PLOT_CAM_TARGET, on_TARGET_IMG_data, self);
    senlcm_prosilica_t_subscribe(self->lcm, "PROSILICA_UW.ATTRIBUTES", on_camatt_data, self);
    senlcm_prosilica_t_subscribe(self->lcm, "PROSILICA_PERI.ATTRIBUTES", on_camatt_data, self);
    hauv_bs_cnv_t_subscribe(self->lcm, "HAUV_BS_CNV", on_CNV_data, self);
    hauv_bs_nvg_t_subscribe(self->lcm, "HAUV_BS_NVG", on_NVG_data, self);

    perllcm_isam_info_t_subscribe(self->lcm, "INFO", on_isam_info, self);
    perllcm_isam_graph_vis_t_subscribe (self->lcm, "HAUV_ISAM_VIS", graph_cb, self);
    hauv_wp_goto_t_subscribe(self->lcm, "WP_GOTO", on_wp_goto_t, self);

    if (viewer) {
        self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
        self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
        GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
        gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
        gtk_widget_show (vbox);

        gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw), 
                            FALSE, TRUE, 0);
        gtk_widget_show (GTK_WIDGET (self->pw));

        // checkboxes
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_SHOW_DIDSON, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_SHOW_IMAGE,  1, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_SHOW_INFO,   0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_SHOW_DVL_TRAJECTORY, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_SHOW_DVL_BOTTOM,     0, NULL);

        // plot whether hull reference dvl traj for sonar or 6dof CNV based dvl traj for camera
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_TRAJ_CNV, 1, NULL);

        //------------------------------------------------------------------

        // Buttons for waypoint control
        GtkWidget *wp_save_button = gtk_button_new_with_label("Save waypoint");
        gtk_box_pack_start(GTK_BOX(self->pw), wp_save_button, FALSE, FALSE, 0);
        g_signal_connect(G_OBJECT(wp_save_button), "clicked", G_CALLBACK(on_waypoint_save_button), self);

        GtkWidget *wp_goto_button = gtk_button_new_with_label("Goto waypoint");
        gtk_box_pack_start(GTK_BOX(self->pw), wp_goto_button, FALSE, FALSE, 0);
        g_signal_connect(G_OBJECT(wp_goto_button), "clicked", G_CALLBACK(on_waypoint_goto_button), self);

        GtkWidget *wp_goto_rel_button = gtk_button_new_with_label("Goto waypoint (rel)");
        gtk_box_pack_start(GTK_BOX(self->pw), wp_goto_rel_button, FALSE, FALSE, 0);
        g_signal_connect(G_OBJECT(wp_goto_rel_button), "clicked", G_CALLBACK(on_waypoint_goto_relative_button), self);

        GtkWidget *wp_resume_button = gtk_button_new_with_label("Resume");
        gtk_box_pack_start(GTK_BOX(self->pw), wp_resume_button, FALSE, FALSE, 0);
        g_signal_connect(G_OBJECT(wp_resume_button), "clicked", G_CALLBACK(on_waypoint_resume_button), self);

        GtkWidget *wp_delete_resume_button = gtk_button_new_with_label("Delete Resume");
        gtk_box_pack_start(GTK_BOX(self->pw), wp_delete_resume_button, FALSE, FALSE, 0);
        g_signal_connect(G_OBJECT(wp_delete_resume_button), "clicked", G_CALLBACK(on_waypoint_delete_resume_button), self);

        GtkWidget *wp_off_button = gtk_button_new_with_label("Off");
        gtk_box_pack_start(GTK_BOX(self->pw), wp_off_button, FALSE, FALSE, 0);
        g_signal_connect(G_OBJECT(wp_off_button), "clicked", G_CALLBACK(on_waypoint_off_button), self);

        // Add waypoint radio buttons
        char button_name[100];
        GtkWidget* waypoint_button = NULL;
        GSList* group = NULL;

        for (int i=0; i<10; ++i)
        {
            sprintf(button_name, "Waypoint %d", (i+1));
            waypoint_button = gtk_radio_button_new_with_label(group, button_name);

            gtk_box_pack_start(GTK_BOX(self->pw), waypoint_button, TRUE, TRUE, 0);
            g_signal_connect(waypoint_button, "clicked",
              G_CALLBACK(on_waypoint_button_click), self);

            gtk_widget_show(waypoint_button);
            group = gtk_radio_button_get_group(GTK_RADIO_BUTTON(waypoint_button));

            self->waypoints[waypoint_button] = i;
        }

        //------------------------------------------------------------------

        GtkWidget *separator;
        separator = gtk_hseparator_new();
        gtk_box_pack_start(GTK_BOX(self->pw), separator, FALSE, TRUE, 0);

        // ship model ------------------
        bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_SHOW_SHIP,   0, NULL);
        bot_gtk_param_widget_add_double(self->pw, PARAM_SHIP_X,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_SHIP_X_MIN, PARAM_SHIP_X_MAX,
                                    PARAM_SHIP_X_DELTA,
                                    PARAM_SHIP_X_DEFAULT);
        bot_gtk_param_widget_add_double(self->pw, PARAM_SHIP_Y,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_SHIP_Y_MIN, PARAM_SHIP_Y_MAX,
                                    PARAM_SHIP_Y_DELTA,
                                    PARAM_SHIP_Y_DEFAULT);
        bot_gtk_param_widget_add_double(self->pw, PARAM_SHIP_Z,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_SHIP_Z_MIN, PARAM_SHIP_Z_MAX,
                                    PARAM_SHIP_Z_DELTA,
                                    PARAM_SHIP_Z_DEFAULT);
        bot_gtk_param_widget_add_double(self->pw, PARAM_SHIP_HEADING,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_SHIP_HEADING_MIN, PARAM_SHIP_HEADING_MAX,
                                    PARAM_SHIP_HEADING_DELTA,
                                    PARAM_SHIP_HEADING_DEFAULT);
        bot_gtk_param_widget_add_double(self->pw, PARAM_SHIP_S,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_SHIP_S_MIN, PARAM_SHIP_S_MAX,
                                    PARAM_SHIP_S_DELTA,
                                    PARAM_SHIP_S_DEFAULT);


        gtk_widget_show_all (GTK_WIDGET (self->pw));

        g_signal_connect(G_OBJECT (&(self->viewer->gl_area->area.widget)), 
                                   "size-allocate", G_CALLBACK(on_glarea_changed), self);

        g_signal_connect (G_OBJECT (self->pw), "changed",
                          G_CALLBACK (on_param_widget_changed), self);

        // mfallon, save widget modes:
        g_signal_connect (G_OBJECT (viewer), "load-preferences",
                G_CALLBACK (on_load_preferences), self);
        g_signal_connect (G_OBJECT (viewer), "save-preferences",
                G_CALLBACK (on_save_preferences), self);	

    }

    return self;
}

void
setup_renderer_hauv (BotViewer *viewer, char *rootkey, int render_priority)
{
    RendererHauv *self = renderer_hauv_new (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &(self->renderer), render_priority);
}

