#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <map>

#include "perls-common/bot_util.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"

#include "perls-math/ssc.h"

#include "perls-sensors/rdi.h"

#include "perls-lcmtypes/perllcm_isam_graph_vis_t.h"
#include "perls-lcmtypes/perllcm_rdi_bathy_t.h"

#include "perls-lcmtypes/hauv_bs_pit_t.h"
#include "perls-lcmtypes/hauv_bs_dvl_t.h"
#include "perls-lcmtypes/hauv_bs_rnv_2_t.h"
#include "perls-lcmtypes/hauv_bs_cnv_t.h"
#include "perls-lcmtypes/hauv_didson_t.h"
#include "perls-lcmtypes/hauv_wp_list_t.h"

#include "renderers.h"

#define RENDERER_NAME "HAUV Pose Attachments"

#define CAM_FOV_HORZ                  40
#define CAM_FOV_VERT                  30
#define PERISCOPE_ANGLE_DEG           60
#define MAX_CAMERA_NFRAMES            5000

#define PARAM_SHOW_DVL              "DVL Path"
#define PARAM_SHOW_CAMERA_FOOTPRINT "Camera Footprint"
#define PARAM_SHOW_CAMERA_PATH      "Camera Path"
#define PARAM_SHOW_SONAR_CONE       "Sonar Cone"
#define PARAM_SHOW_WP               "Waypoints"
#define PARAM_SHOW_HAUV_MODEL       "Show HAUV Model"
#define PARAM_CAMERA_NFRAMES        "Camera Frame History"
#define PARAM_SHOW_HAUV_AXIS        "Show HAUV axis"
#define PARAM_FIND_ROBOT            "Find Robot"

#define PARAM_USE_DR_POSE           "Use DR for pose"       // @TODO: instead of using isam 

// camera, sonar configuration
#define PARAM_MODE_PERISCOPE        "Camera Periscope"
#define PARAM_MODE_SPLIT            "Sonar Split"
#define PARAM_MODE_HALFSPLIT        "Sonar Half-split"
#define PARAM_MODE_3DFLS            "Sonar 3DFLS"

#define DVL_POINT_SIZE 3.0

using namespace std;

typedef enum _sonar_mode_t
{
    MODE_SONAR_IMAGING = 0,
    MODE_SONAR_SPLIT = 1,
    MODE_SONAR_HALFSPLIT = 2,
    MODE_SONAR_3DFLS = 3,
} sonar_mode_t;

typedef struct _State {
    
    double dvlAngle;
    double sonarAngle;
    double camOffset;
    double sonarX;
    double sonarY;
    double sonarZ;
    double sonarRMin;
    double sonarRMax;

} State;

typedef struct _RendererHauvPose {
    BotRenderer renderer;
    lcm_t *lcm;
    BotParam *param;

    perllcm_isam_graph_vis_t *graph;

    BotViewer         *viewer;
    BotGtkParamWidget *pw;   

    map<uint64_t, perllcm_rdi_bathy_t *> nodeToDvlData;
    map<uint64_t, double> nodeToCamDistData;
    map<uint64_t, double> nodeToDvlAngle;

    State state;

    double x_sd[6];

    //booleans
    int showDvlPath;
    int showCamera;
    int showCameraPath;
    int showSonarCone;
    int periscopeMode;
    int showWP;
    int showHauvModel;
    int showAxis;

    // sonar mode
    int m_sonar_mode;

    // cp of waypoint list
    hauv_wp_list_t *wp_list;

    //other settings
    uint32_t maxCameraNFrames;

    // wavefront model
    BotWavefrontModel *wfm_robot;
    int                wavefront_dl_ready_robot;
    GLuint             wavefront_dl_robot;
    BotWavefrontModel *wfm_ship;
    int                wavefront_dl_ready_ship;
    GLuint             wavefront_dl_ship;

    double xyzrph[6];

} RendererHauvPose;

static void 
on_find_robot (GtkWidget *button, void *user)
{
    RendererHauvPose *self = (RendererHauvPose*) user;
    BotViewHandler *vhandler = self->viewer->view_handler;

    if (!self->graph)
        return;

    int duration_ms = 500;  //milliseconds
    
    double pose[6] = {0};
    memcpy (pose, self->graph->mu[self->graph->nnodes - 1], 6*sizeof (double));
    
    double eye[3] = {pose[0], -pose[1], -pose[2]+20};
    double lookat[3] = {pose[0], -pose[1] ,-pose[2]};
    double up[3] = {1, 0, 0};
    
    vhandler->set_look_at_smooth (vhandler, eye, lookat, up, duration_ms);
}

static GLuint
compile_wavefront_display_list (RendererHauvPose *self, BotWavefrontModel *model)
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
draw_hauv_axis()
{
    glLineWidth(3);
    glBegin(GL_LINES);
    glColor3f(1.0,0.0,0.0); glVertex3f(0.0,0.0,0.0); glVertex3f(1.0,0.0,0.0);
    glColor3f(0.0,1.0,0.0); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,1.0,0.0);
    glColor3f(0.0,0.0,1.0); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,0.0,1.0);
    glEnd();
}

static void 
draw_square (double x, double y, double z, double theta, double size) 
{
    glPushMatrix();
      glTranslatef(x, y, z);
      glRotatef(bot_to_degrees(theta),0.0,0.0,1.0);
      glBegin(GL_LINE_LOOP);
        glVertex3f(size,size,0.0);
        glVertex3f(-size,size,0.0);
        glVertex3f(-size,-size,0.0);
        glVertex3f(size,-size,0.0);
      glEnd();
    glPopMatrix();
}


static void
mapDvlToGraphNode (int64_t utime, const perllcm_rdi_bathy_t &dvlDataVolit, RendererHauvPose *self)
{
    if (!self->graph)
        return;

    perllcm_rdi_bathy_t *dvlData = perllcm_rdi_bathy_t_copy (&dvlDataVolit);

    int i=self->graph->nnodes - 1;

    if (self->nodeToDvlAngle.count(self->graph->node_id[i]) == 0) {
        self->nodeToDvlData[self->graph->node_id[i]] = dvlData;
        self->nodeToDvlAngle[self->graph->node_id[i]] = self->state.dvlAngle;
        return;
    }

    perllcm_rdi_bathy_t_destroy(dvlData);

}

static void
mapCamDistToGraphNode (const hauv_bs_rnv_2_t *rnvData, RendererHauvPose *self)
{
    if (!self->graph)
        return;

    int i=self->graph->nnodes - 1;

    if (self->nodeToCamDistData.count(self->graph->node_id[i]) == 0) {
        self->nodeToCamDistData[self->graph->node_id[i]] = rnvData->distance;
        return;
    }

}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererHauvPose *self = (RendererHauvPose*) user;
    self->showDvlPath      = bot_gtk_param_widget_get_bool (self->pw, PARAM_SHOW_DVL);
    self->showCamera       = bot_gtk_param_widget_get_bool (self->pw, PARAM_SHOW_CAMERA_FOOTPRINT);
    self->showCameraPath   = bot_gtk_param_widget_get_bool (self->pw, PARAM_SHOW_CAMERA_PATH);
    self->showSonarCone    = bot_gtk_param_widget_get_bool (self->pw, PARAM_SHOW_SONAR_CONE);
    self->showWP           = bot_gtk_param_widget_get_bool (self->pw, PARAM_SHOW_WP);
    self->showHauvModel    = bot_gtk_param_widget_get_bool (self->pw, PARAM_SHOW_HAUV_MODEL);
    self->showAxis         = bot_gtk_param_widget_get_bool (self->pw, PARAM_SHOW_HAUV_AXIS);
    self->maxCameraNFrames = bot_gtk_param_widget_get_int (self->pw, PARAM_CAMERA_NFRAMES);
    self->periscopeMode    = bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_PERISCOPE);

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_SPLIT)) {
        if (self->m_sonar_mode == MODE_SONAR_HALFSPLIT 
            && bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_HALFSPLIT) == 1)
            bot_gtk_param_widget_set_bool (self->pw, PARAM_MODE_HALFSPLIT, 0);
        if (self->m_sonar_mode == MODE_SONAR_3DFLS 
            && bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_3DFLS) == 1)
            bot_gtk_param_widget_set_bool (self->pw, PARAM_MODE_3DFLS, 0);
        self->m_sonar_mode = MODE_SONAR_SPLIT;
    }
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_HALFSPLIT)) {
        if (self->m_sonar_mode == MODE_SONAR_SPLIT
            && bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_SPLIT) == 1)
            bot_gtk_param_widget_set_bool (self->pw, PARAM_MODE_SPLIT, 0);
        if (self->m_sonar_mode == MODE_SONAR_3DFLS 
            && bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_3DFLS) == 1)
            bot_gtk_param_widget_set_bool (self->pw, PARAM_MODE_3DFLS, 0);
        self->m_sonar_mode = MODE_SONAR_HALFSPLIT;
    }
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_3DFLS)) {
        if (self->m_sonar_mode == MODE_SONAR_SPLIT
            && bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_SPLIT) == 1)
            bot_gtk_param_widget_set_bool (self->pw, PARAM_MODE_SPLIT, 0);
        if (self->m_sonar_mode == MODE_SONAR_HALFSPLIT 
            && bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_HALFSPLIT) == 1)
            bot_gtk_param_widget_set_bool (self->pw, PARAM_MODE_HALFSPLIT, 0);
        self->m_sonar_mode = MODE_SONAR_3DFLS;
    }

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_SPLIT) == 0 
        && bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_HALFSPLIT) == 0
        && bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_3DFLS) ==0 )
        self->m_sonar_mode = MODE_SONAR_IMAGING;

    bot_viewer_request_redraw (self->viewer);
}

static void
perllcm_isam_graph_vis_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                   const perllcm_isam_graph_vis_t *msg, void *user)
{

    RendererHauvPose *self = (RendererHauvPose *)user;

    if (self->graph)
        perllcm_isam_graph_vis_t_destroy(self->graph);

    self->graph = perllcm_isam_graph_vis_t_copy(msg);

}

static void
hauv_bs_dvl_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const hauv_bs_dvl_t *msg, void *user)
{
    RendererHauvPose *self = (RendererHauvPose *)user;

    //vehicle to servo
    double x_vs[6] = {0., 0., 0., 0., self->state.dvlAngle, 0.0};

    //vehicle to dvl
    double *x_vd = (double *)malloc(6 * sizeof(*x_vd));
    ssc_head2tail(x_vd, NULL, x_vs, self->x_sd);

    const double cos30 = cos (30*UNITS_DEGREE_TO_RADIAN);
    perllcm_rdi_bathy_t tmp = rdi_bathy_janus30 (msg->range1/cos30, msg->range2/cos30, 
                                                 msg->range3/cos30, msg->range4/cos30, x_vd);

    //Map the utime of the dvl to the utime of the corresponding node in the graph (using
    //some sort of closest time)
    mapDvlToGraphNode(msg->time, tmp, self);

    //clean up
    free(x_vd);

}

static void
hauv_bs_pit_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const hauv_bs_pit_t *msg, void *user)
{
    RendererHauvPose *self = (RendererHauvPose *)user;
    self->state.dvlAngle = msg->pitch_dvl;
    self->state.sonarAngle = msg->pitch_sonar;
}

static void
hauv_bs_rnv_2_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                          const hauv_bs_rnv_2_t *msg, void *user)
{
    RendererHauvPose *self = (RendererHauvPose *)user;
    self->state.camOffset = msg->distance;
    mapCamDistToGraphNode(msg, self);

    //Assing the DR orientation
    self->xyzrph[3] = msg->absroll;
    self->xyzrph[4] = msg->abspitch;
    self->xyzrph[5] = msg->absheading;
}

static void
hauv_didson_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const hauv_didson_t *msg, void *user)
{
    RendererHauvPose *self = (RendererHauvPose *)user;
    self->state.sonarX = msg->m_fSonarX;
    self->state.sonarY = msg->m_fSonarY;
    self->state.sonarZ = msg->m_fSonarZ;

    self->state.sonarRMin = msg->m_nWindowStart * 0.375;
    double windowLength[4] = {1.125, 2.25, 4.5, 9.0};
    self->state.sonarRMax = self->state.sonarRMin + windowLength[msg->m_nWindowLength];
}

static void
changeToVehicleFrame (double *pose)
{
    double tX   = pose[0];
    double tY   = pose[1];
    double tZ   = pose[2];
    double rotX = bot_to_degrees(pose[3]);
    double rotY = bot_to_degrees(pose[4]);
    double rotZ = bot_to_degrees(pose[5]);

    //Change to the vehicle's frame
    glPushMatrix();
    glTranslated(tX, tY, tZ);
    glRotated(rotZ, 0.0, 0.0, 1.0);
    glRotated(rotY, 0.0, 1.0, 0.0);
    glRotated(rotX, 1.0, 0.0, 0.0);
}

static void
hauv_wp_list_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                         const hauv_wp_list_t *msg, void *user)
{
    RendererHauvPose *self = (RendererHauvPose *)user;

    if (self->wp_list)
        hauv_wp_list_t_destroy (self->wp_list);

    self->wp_list = hauv_wp_list_t_copy (msg);
}

static void 
hauv_bs_cnv_t_callback (const lcm_recv_buf_t *rbuf, const char *channel, 
                           const hauv_bs_cnv_t *msg, void *user_data )
{
    RendererHauvPose *self = (RendererHauvPose*) user_data;

    //Assing the DR position
    self->xyzrph[0] = msg->x;
    self->xyzrph[1] = msg->y;
    self->xyzrph[2] = msg->z;
}

static void
renderDvlPath (BotViewer *viewer, BotRenderer *super)
{
    RendererHauvPose *self = (RendererHauvPose*) super->user;

    if (!self->showDvlPath)
        return;

    glEnable (GL_DEPTH_TEST);

    //For every pose
    for (int i=0; i<self->graph->nnodes; i++) {

        if (self->nodeToDvlData.count(self->graph->node_id[i]) == 0)
            continue;

        double *currentPose = self->graph->mu[i];
        changeToVehicleFrame(currentPose);

        perllcm_rdi_bathy_t *dvlData = self->nodeToDvlData[self->graph->node_id[i]];

        glPointSize(DVL_POINT_SIZE);
        glColor3f(0., 0., 1.0);
        for (int pointIdx=0; pointIdx<4; pointIdx++) {
 
            glBegin (GL_POINTS);
            glVertex3d(dvlData->xyz[pointIdx][0], dvlData->xyz[pointIdx][1], dvlData->xyz[pointIdx][2]);
            glEnd();

        }
        
        glPopMatrix(); //Go back to original frame

    }
}

static void
renderCameraFoot (BotViewer *viewer, BotRenderer *super)
{
    RendererHauvPose *self = (RendererHauvPose*) super->user;

    if (!self->showCamera)
        return;

    double *currentPose = NULL;
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_USE_DR_POSE))
        currentPose = self->xyzrph;
    else if (self->graph)
        currentPose = self->graph->mu[self->graph->nnodes - 1];
    else
        return;
    changeToVehicleFrame(currentPose);

    glPushMatrix();
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_PERISCOPE)) {
        glRotatef(90,0.0,0.0,1.0);
        glRotatef(90+PERISCOPE_ANGLE_DEG,1.0,0.0,0.0);
    }
    else {
        glRotatef(bot_to_degrees(self->state.dvlAngle),0.0,1.0,0.0);
        glRotatef(90,0.0,0.0,1.0);
        glRotatef(90,1.0,0.0,0.0);
    }
    //draw_hauv_axis();

    glColor3f(1.0,0.0,0.0);   // red cone for camera cone
    //glLineWidth (3);

    double origx,origy,origz,camx,camy,camz;
    origx = 0.0;
    origy = 0.0;
    origz = 0.0;
    camz = self->state.camOffset;
    camx = camz*tan(bot_to_radians(0.5*CAM_FOV_HORZ));
    camy = camz*tan(bot_to_radians(0.5*CAM_FOV_VERT));

    // Near side
    glBegin(GL_LINE_STRIP);
    glVertex3f( -origx,  origy,  origz);
    glVertex3f(  origx,  origy,  origz);
    glVertex3f(  origx, -origy,  origz);
    glVertex3f( -origx, -origy,  origz);
    glVertex3f( -origx,  origy,  origz);
    glEnd();
    // Far side
    glBegin(GL_LINE_STRIP);
    glVertex3f( -camx, camy, camz);
    glVertex3f( camx, camy,  camz);
    glVertex3f( camx,-camy,  camz);
    glVertex3f( -camx,-camy, camz);
    glVertex3f( -camx, camy, camz);
    glEnd();

    // Bottom
    glBegin(GL_LINE_STRIP);
    glVertex3f( camx, -camy,  camz);
    glVertex3f(-camx, -camy,  camz);
    glVertex3f(-origx,-origy, origz);
    glVertex3f( origx,-origy, origz);
    glVertex3f( camx, -camy,  camz);
    glEnd();

    // Top
    glBegin(GL_LINE_STRIP);
    glVertex3f( camx,  camy,  camz);
    glVertex3f(-camx,  camy,  camz);
    glVertex3f(-origx, origy, origz);
    glVertex3f( origx, origy, origz);
    glVertex3f( camx,  camy,  camz);
    glEnd();

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.9,0.1,0.1,0.2);
    glBegin(GL_QUADS);
    glVertex3f( camx,  camy, camz);
    glVertex3f( camx, -camy, camz);
    glVertex3f( -camx,-camy, camz);
    glVertex3f( -camx, camy, camz);
    glEnd();
    glPopAttrib ();
    glPopMatrix(); //Back to vehicle frame
    glPopMatrix(); //Back to local frame
}

static void
renderCameraPath (BotViewer *viewer, BotRenderer *super)
{

    RendererHauvPose *self = (RendererHauvPose *)super->user;

    if (!self->showCameraPath)
        return;

    int upperLim = 0;
    int numDrawnFrames = 0;;
    if (self->maxCameraNFrames < self->nodeToCamDistData.size())
        upperLim = self->maxCameraNFrames;
    else
        upperLim = self->nodeToCamDistData.size();

    for (int i=self->graph->nnodes - 1; i>=0; i--) {

        if (self->nodeToCamDistData.count(self->graph->node_id[i]) == 0)
            continue;

        if (self->nodeToDvlAngle.count(self->graph->node_id[i]) == 0)
            continue;

        double *currentPose = self->graph->mu[i];
        changeToVehicleFrame(currentPose);

        glColor3f(0.75,0.5,0.25);        
        glPushMatrix();
        if (bot_gtk_param_widget_get_bool (self->pw, PARAM_MODE_PERISCOPE)) {
            glRotatef(90,0.0,0.0,1.0);
            glRotatef(90+PERISCOPE_ANGLE_DEG,1.0,0.0,0.0);
        }
        else {
            glRotatef(bot_to_degrees(self->nodeToDvlAngle[self->graph->node_id[i]]),0.0,1.0,0.0);
            glRotatef(90,0.0,0.0,1.0);
            glRotatef(90,1.0,0.0,0.0);
        }

        double camx,camy,camz,alt;
        alt = self->nodeToCamDistData[self->graph->node_id[i]];
        camz = alt;
        camx = alt*tan(bot_to_radians(0.5*CAM_FOV_HORZ));
        camy = alt*tan(bot_to_radians(0.5*CAM_FOV_VERT));
        if (numDrawnFrames < upperLim) {
            glBegin(GL_LINE_STRIP);
            glVertex3f( camx,   camy,  camz);
            glVertex3f( -camx,  camy,  camz);
            glVertex3f( -camx, -camy,  camz);
            glVertex3f( camx,  -camy,  camz);
            glVertex3f( camx,   camy,  camz);
            glEnd();
            numDrawnFrames++;
        }
        glPopMatrix(); //Back to vehicle frame
        glPopMatrix(); //Back to local frame

    }

}

static void
_draw_cone (double xnear, double ynear, double znear, double xfar, double yfar, double zfar)
{
    // Near side
    glBegin(GL_LINE_STRIP);
    glVertex3f( xnear,  ynear,  znear);
    glVertex3f( xnear,  ynear, -znear);
    glVertex3f( xnear, -ynear, -znear);
    glVertex3f( xnear, -ynear,  znear);
    glVertex3f( xnear,  ynear,  znear);
    glEnd();
    // Far side
    glBegin(GL_LINE_STRIP);
    glVertex3f( xfar, yfar, zfar);
    glVertex3f( xfar, yfar,-zfar);
    glVertex3f( xfar,-yfar,-zfar);
    glVertex3f( xfar,-yfar, zfar);
    glVertex3f( xfar, yfar, zfar);
    glEnd();

    // Bottom
    glBegin(GL_LINE_STRIP);
    glVertex3f( xfar, yfar,-zfar);
    glVertex3f( xfar,-yfar,-zfar);
    glVertex3f( xnear,-ynear,-znear);
    glVertex3f( xnear, ynear,-znear);
    glVertex3f( xfar, yfar, -zfar);
    glEnd();

    // Top
    glBegin(GL_LINE_STRIP);
    glVertex3f( xfar, yfar,zfar);
    glVertex3f( xfar,-yfar,zfar);
    glVertex3f( xnear,-ynear,znear);
    glVertex3f( xnear, ynear,znear);
    glVertex3f( xfar, yfar, zfar);
    glEnd();
}

static void
_sonar_3dfls_mode (RendererHauvPose *self)
{
}

static void
renderSonarCone (BotViewer *viewer, BotRenderer *super)
{
    RendererHauvPose *self = (RendererHauvPose *)super->user;

    if (!self->showSonarCone)
        return;

    glColor3f(0.0,0.0,1.0);

    double *currentPose = NULL;
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_USE_DR_POSE))
        currentPose = self->xyzrph;
    else if (self->graph)
        currentPose = self->graph->mu[self->graph->nnodes - 1];
    else
        return;
    changeToVehicleFrame(currentPose);

    //change to the sonar's frame
    if (self->m_sonar_mode == MODE_SONAR_3DFLS) {
        glPushMatrix();
        glTranslatef(0.0,0.2,0.0);       // HULS3
        _sonar_3dfls_mode (self);
        glPopMatrix(); //go back to vehicle's frame
    }
    else {
        glPushMatrix();
        glTranslatef(0.0,0.2,0.0);       // HULS3

        double sonar_beam_angle = 7.0;        

        if (self->m_sonar_mode == MODE_SONAR_SPLIT) {
            sonar_beam_angle = 1.0;
            glRotatef(-90.0, 1.0, 0.0, 0.0);
        }

        // Rotate the sonar on it's own axis
        glRotatef(-bot_to_degrees(self->state.sonarAngle),0.0,0.0,1.0);
        // Put sonar sideways and rotate with DVL frame
        glRotatef(bot_to_degrees(self->state.dvlAngle)+90,0.0,1.0,0.0);
        // Face sonar to the right
        glRotatef(90.0, 0.0, 0.0, 1.0);
        
        double xnear,ynear,znear,xfar,yfar,zfar;
        xnear = self->state.sonarRMin*cos(bot_to_radians(14))*cos(bot_to_radians(sonar_beam_angle));
        ynear = self->state.sonarRMin*sin(bot_to_radians(14))*cos(bot_to_radians(sonar_beam_angle));
        znear = self->state.sonarRMin*sin(bot_to_radians(sonar_beam_angle));
        xfar = self->state.sonarRMax*cos(bot_to_radians(14))*cos(bot_to_radians(sonar_beam_angle));
        yfar = self->state.sonarRMax*sin(bot_to_radians(14))*cos(bot_to_radians(sonar_beam_angle));
        zfar = self->state.sonarRMax*sin(bot_to_radians(sonar_beam_angle));

        _draw_cone (xnear, ynear, znear, xfar, yfar, zfar);

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.1,0.1,0.9,0.2);
        glBegin(GL_QUADS);
        glVertex3f( xfar, yfar,zfar);
        glVertex3f( xfar,-yfar,zfar);
        glVertex3f( xnear,-ynear,znear);
        glVertex3f( xnear, ynear,znear);
        glEnd();
        glPopAttrib ();

        glPopMatrix(); //go back to vehicle's frame
    }

    glPopMatrix(); //go back to local frame

}

static void
renderWP (BotViewer *viewer, BotRenderer *super)
{
    RendererHauvPose *self = (RendererHauvPose *)super->user;

    if (!self->wp_list || !self->showWP) return;

    hauv_wp_list_t *wp_list = self->wp_list;
    //std::cout << "draw some waypoints" << std::endl;

    // @todo can we generalize this for cam, dvl and wp?
    GHashTable *ht_id2node = g_hash_table_new (g_int64_hash, g_int64_equal);
    for (int i=0; i<self->graph->nnodes; i++) {
        g_hash_table_insert (ht_id2node, &(self->graph->node_id[i]), &(self->graph->mu[i][0]));
    }
    

    for (int i=0; i < wp_list->n; i++) {
        double *mu;
        mu = (double*) g_hash_table_lookup (ht_id2node, &(wp_list->utime[i]));

        double box_size = 0.2;
        if (mu) {
            double pos[3]; pos[0] = mu[0]; pos[1] = mu[1]; pos[2] = mu[2];

            glColor3f(0.0,1.0,1.0); // cyan
            glLineWidth(3);
            draw_square (pos[0], pos[1], pos[2], 0.0, box_size);

            // draw... "wp #" or "res" with square boxes
            char wp_str[256];
            if (wp_list->id[i] < 0) {
                snprintf (wp_str, sizeof wp_str, "res");
            }
            else {
                snprintf (wp_str, sizeof wp_str, "wp %d", wp_list->id[i]+1);
            }
                
            glColor3f(1,1,1);
            bot_gl_draw_text(pos, GLUT_BITMAP_HELVETICA_12, wp_str, BOT_GL_DRAW_TEXT_DROP_SHADOW);
        }
    }

    // clean up
    g_hash_table_destroy (ht_id2node);
}

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

static void
renderHauvModel (BotViewer *viewer, BotRenderer *super)
{
    RendererHauvPose *self = (RendererHauvPose *)super->user;

    double *currentPose = NULL;
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_USE_DR_POSE))
        currentPose = self->xyzrph;
    else if (self->graph)
        currentPose = self->graph->mu[self->graph->nnodes - 1];
    else
        return;
    changeToVehicleFrame(currentPose);

    if (self->showAxis)
        draw_hauv_axis();

    // Draw vehicle model
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_HAUV_MODEL)) {
        glColor3f(0.5,0.5,0.5);     // draw HAUV in gray

        if (!self->wavefront_dl_ready_robot) { // setup wavefront if its there
            if (self->wfm_robot)
                self->wavefront_dl_robot = compile_wavefront_display_list (self, self->wfm_robot);
            self->wavefront_dl_ready_robot = 1;
        }

        glPushMatrix();
        // trans/rot/scale for HULS3 model
        glTranslatef(-1.0,0.7,0.1);   // to locate vehicle's coord at the dvl
        glRotatef(-90.0, 0.0, 0.0, 1.0);
        glRotatef(-90.0, 1.0, 0.0, 0.0);
        glScalef(0.001,0.001,0.001);

        glEnable(GL_LIGHTING);
        glPushMatrix();
        if (self->wavefront_dl_ready_robot && self->wavefront_dl_robot) // draw wavefront
            draw_wavefront_model (self->wavefront_dl_robot);
        glPopMatrix();
        glDisable(GL_LIGHTING);
        glPopMatrix();

    }

    glPopMatrix(); //pop-off of vehicle's frame
    
}

static void 
hauv_pose_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererHauvPose *self = (RendererHauvPose *)super->user;

    glRotatef (180.0, 1.0, 0.0, 0.0);

    //Draw HAUV model
    renderHauvModel(viewer, super);
    renderCameraFoot(viewer, super);
    renderSonarCone(viewer, super);

    if (self->graph) {
        renderDvlPath(viewer, super);
        renderCameraPath(viewer, super);
        renderWP(viewer, super);
    }
}

static void
_renderer_free (BotRenderer *super)
{
    RendererHauvPose *self = (RendererHauvPose*) super->user;
    perllcm_isam_graph_vis_t_destroy (self->graph);
    delete self;
}

static RendererHauvPose *
new_renderer_hauv_pose (BotViewer *viewer, char *rootkey)
{
    //In case we use the std, we should use 'new'
    RendererHauvPose *self = new RendererHauvPose();
    
    self->graph          = NULL;
    self->wp_list        = NULL;
    self->state.dvlAngle = -1;
    self->m_sonar_mode = MODE_SONAR_IMAGING;

    //Create the BotParam
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    self->lcm = bot_lcm_get_global (NULL);

    //Config file entires
    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);
    bot_param_get_double_array_or_fail(self->param, "dvl.x_s1d", self->x_sd, 6);
    char *vis_channel = botu_param_get_str_or_default (self->param, 
                                                       "isamServer.lcm_channels.VIS_CHANNEL",
                                                       "HAUV_ISAM_VIS");

    //Convert pose from servo1 to dvl to radians
    for (int i=0; i<6; i++)
        self->x_sd[i] *= UNITS_DEGREE_TO_RADIAN;

    //channel subscriptions
    perllcm_isam_graph_vis_t_subscribe (self->lcm, vis_channel, &perllcm_isam_graph_vis_t_callback, self);
    hauv_bs_pit_t_subscribe (self->lcm, "HAUV_BS_PIT", &hauv_bs_pit_t_callback, self);
    hauv_bs_dvl_t_subscribe (self->lcm, "HAUV_BS_DVL", &hauv_bs_dvl_t_callback, self);
    hauv_bs_rnv_2_t_subscribe (self->lcm, "HAUV_BS_RNV_2", &hauv_bs_rnv_2_t_callback, self);
    hauv_bs_cnv_t_subscribe (self->lcm, "HAUV_BS_CNV", &hauv_bs_cnv_t_callback, self);
    hauv_didson_t_subscribe (self->lcm, "HAUV_DIDSON_FRAME", &hauv_didson_t_callback, self);
    hauv_wp_list_t_subscribe (self->lcm, "WP_LIST", &hauv_wp_list_t_callback, self);

    //3D models (HAUV, etc.)
    char *wavefront_fname_robot = NULL;
    snprintf (key, sizeof key, "viewer.renderers.%s.robot_obj_file", rootkey);
    bot_param_get_str (self->param, key, &wavefront_fname_robot);

    if (wavefront_fname_robot) { 
        self->wfm_robot = bot_wavefront_model_create (wavefront_fname_robot);
        free (wavefront_fname_robot);
    }

    //BotRenderer
    BotRenderer *renderer = &self->renderer;
    renderer->draw = hauv_pose_renderer_draw;
    renderer->destroy = _renderer_free;
    renderer->widget = bot_gtk_param_widget_new ();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);

    //Checkboxes
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_USE_DR_POSE, 0, NULL);
    bot_gtk_param_widget_add_separator (self->pw, "");

    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_SHOW_DVL, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_SHOW_CAMERA_FOOTPRINT, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_SHOW_CAMERA_PATH, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_SHOW_SONAR_CONE, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_SHOW_WP, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_SHOW_HAUV_MODEL, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_SHOW_HAUV_AXIS, 1, NULL);

    //Slider bar
    bot_gtk_param_widget_add_int (self->pw, PARAM_CAMERA_NFRAMES, BOT_GTK_PARAM_WIDGET_SLIDER, 0, MAX_CAMERA_NFRAMES, 1, 500);

    bot_gtk_param_widget_add_separator (self->pw, "");
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_MODE_PERISCOPE, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_MODE_SPLIT, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_MODE_HALFSPLIT, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_MODE_3DFLS, 0, NULL);

    //Buttons
    GtkWidget *find_robot_button = gtk_button_new_with_label (PARAM_FIND_ROBOT);
    g_signal_connect (G_OBJECT (find_robot_button), "clicked", G_CALLBACK (on_find_robot), self);

    //Call the widget callback manually
    on_param_widget_changed (self->pw, NULL, self);

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);

    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw),FALSE, TRUE, 0);
    gtk_box_pack_start (GTK_BOX (vbox), find_robot_button, FALSE, FALSE, 0);
    gtk_widget_show (GTK_WIDGET (self->pw));

    gtk_widget_show_all (renderer->widget);

    return self;
}

void
setup_renderer_hauv_pose (BotViewer *viewer, char *rootkey, int priority)
{
    RendererHauvPose *self = new_renderer_hauv_pose (viewer, rootkey);
    for (int i=0; i<6; i++)
        self->xyzrph[i] = 0;
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
