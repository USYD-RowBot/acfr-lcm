#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>

// external linking req'd
#include "perls-lcmtypes/perllcm_isam_graph_vis_t.h"
#include "perls-lcmtypes/perllcm_isam_vlink_t.h"
#include "perls-lcmtypes/perllcm_isam_plink_t.h"
#include "perls-lcmtypes/perllcm_isam_plink_collection_t.h"
#include "perls-lcmtypes/perllcm_isam_cmd_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"

#include "renderer_util.h"
#include "renderers.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RENDERER_NAME "ISAM Graph"
#define CMD_CHANNEL   "CMD"

#define PARAM_DRAW_TRI "Draw Pose Triangles"
#define PARAM_DRAW_COV "Draw Covariance"

// time elevation
#define PARAM_TIME_ELEV "Time Elevation"
#define TIME_SCALE   10.0

typedef struct _RendererISAMGraph RendererISAMGraph;
struct _RendererISAMGraph {
    BotRenderer renderer;
    
    lcm_t *lcm;
    BotParam *param;
    
    perllcm_isam_graph_vis_t *igv;

    // @TODO: maybe use std vector or degueue?
    GList *gl_plink;
    GList *gl_vlink;
    
    GtkWidget *status_txt;
    GtkWidget* vbox; 
    
    BotViewer         *viewer;
    BotGtkParamWidget *pw;  

    // param
    bool param_time_elev;

    // renderer util to add dynamic colored checkbox
    DynCheckbox *dyncb;
    LinepltProp *lineprop;
};

static void
graph_cb (const lcm_recv_buf_t *rbuf, const char *channel,
          const perllcm_isam_graph_vis_t *msg, void *user)
{    
    RendererISAMGraph *self = (RendererISAMGraph*) user;
    
    if (self->igv)
        perllcm_isam_graph_vis_t_destroy (self->igv);
    
    self->igv = perllcm_isam_graph_vis_t_copy (msg);
    
    char status_txt[1024];
    int end = self->igv->nnodes-1; 
    sprintf (status_txt, "xyz = %0.1lf, %0.1lf, %0.1lf NED\n",
             self->igv->mu[end][0], self->igv->mu[end][1], self->igv->mu[end][2]);
    sprintf (status_txt, "%srph = %0.1lf, %0.1lf, %0.1lf deg\n", status_txt,
             self->igv->mu[end][3]*RTOD,
             self->igv->mu[end][4]*RTOD,
             self->igv->mu[end][5]*RTOD);
    
    gtk_label_set_text (GTK_LABEL (self->status_txt), status_txt);
    
    bot_viewer_request_redraw (self->viewer);
}

static void
plink_t_cb (const lcm_recv_buf_t*rbuf, const char *channel,
            const perllcm_isam_plink_collection_t *msg, void *user)
{
    RendererISAMGraph *self = (RendererISAMGraph*) user;
    for (int i=0; i<msg->nlinks; i++) {
        perllcm_isam_plink_t *plink = perllcm_isam_plink_t_copy (&(msg->plink[i]));   
        self->gl_plink = g_list_append (self->gl_plink, plink); 
    }
}

static void
vlink_t_cb (const lcm_recv_buf_t*rbuf, const char *channel,
            const perllcm_isam_vlink_t *msg, void *user)
{
    RendererISAMGraph *self = (RendererISAMGraph*) user;
    perllcm_isam_vlink_t *vlink = perllcm_isam_vlink_t_copy (msg);   
    self->gl_vlink = g_list_append (self->gl_vlink, vlink); 
}

static void
_renderer_free (BotRenderer *super)
{
    RendererISAMGraph *self = (RendererISAMGraph*) super->user;

    delete self;
}

static void
draw_items (RendererISAMGraph* self, dyncb_vis_item_t *item)
{

    std::vector<dyncb_vis_data_t>::iterator it;
    switch (item->m_type) {
        case TYPE_NODE: // draw nodes
            if (bot_gtk_param_widget_get_bool (self->pw, PARAM_DRAW_TRI)) {
                glPushMatrix ();
                glEnable (GL_LIGHTING);
                glEnable (GL_LIGHT0);
                glEnable (GL_COLOR_MATERIAL); // uses glcolor* to set colors while still providing lighting
            
                // lighting setup for shading   
                GLfloat light_position[4]={100.0, 0.0, -100.0, 0.0};
                glLightfv (GL_LIGHT0, GL_POSITION, light_position);
                GLfloat light_ambient[4]= {0.4, 0.4, 0.4, 1.0};
                glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
                GLfloat light_diffuse[4]= {0.4, 0.4, 0.4, 1.0};
                glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
                GLfloat light_specular[4]= {0.4, 0.4, 0.4, 1.0};
                glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
            
                // material stuff for shading
                glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
                GLfloat mat_specular[4]={1.0, 1.0, 1.0, 1.0};
                glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
                GLfloat mat_emission[4]={0.0, 0.0, 0.0, 1.0};
                glMaterialfv (GL_FRONT_AND_BACK, GL_EMISSION, mat_emission);
                glColor4f(0.0, 0.0, 1.0, 1.0);

                for (it=item->m_dyncb_vis_data.begin(); it<item->m_dyncb_vis_data.end(); it++) {
                    int32_t _cb_id = item->m_type + it->sensor_id;
                    if (self->dyncb->checkbox_on[_cb_id]) {
                        glPushMatrix ();
                        double *_mu = it->data;
                        double dx = _mu[0];
                        double dy = _mu[1];
                        double dz = _mu[2];
                        glTranslated (dx, dy, dz);
                        double rot_x = _mu[3] * RTOD;
                        double rot_y = _mu[4] * RTOD;
                        double rot_z = _mu[5] * RTOD;
                        glRotatef (rot_z, 0, 0, 1);
                        glRotatef (rot_y, 0, 1, 0);
                        glRotatef (rot_x, 1, 0, 0);
                        glRotatef (90.0, 0, 1, 0); // so that cone points along x not z
                            
                        glutSolidCone(0.2, 0.4, 3, 3);
                        
                        glPopMatrix ();
                    }
                }
                glPopMatrix ();
            } else {
                self->lineprop->apply_line_style (0, item->m_type);
                for (it=item->m_dyncb_vis_data.begin(); it < item->m_dyncb_vis_data.end(); it++) {
                    int32_t _cb_id = item->m_type + it->sensor_id;
                    if (self->dyncb->checkbox_on[_cb_id]) {
                        float *_color = self->lineprop->get_color(it->sensor_id, item->m_type);
                        glColor4f(_color[0], _color[1], _color[2], 1.0);
                        double *_mu = it->data;
                        glVertex3d (_mu[0], _mu[1], _mu[2]);
                    }
                }
                glEnd ();
            } // end of case draw nodes
            break;
        case TYPE_VLINK: // draw vlinks
            for (it=item->m_dyncb_vis_data.begin(); it<item->m_dyncb_vis_data.end(); it++) {
                int32_t _cb_id = item->m_type + it->sensor_id;
                if (self->dyncb->checkbox_on[_cb_id]) {
                    self->lineprop->apply_line_style (it->sensor_id, item->m_type);
                    double *_mu = it->data;
                    double xi[3], xj[3];
                    xi[0] = _mu[0]; xi[1] = _mu[1]; xi[2] = _mu[2];
                    xj[0] = _mu[3]; xj[1] = _mu[4]; xj[2] = _mu[5];
                    glBegin (GL_LINES);
                        glVertex3d (xi[0], xi[1], xi[2]);  glVertex3f(xj[0], xj[1], xj[2]); 
                    glEnd ();
                }
            }
            break;
        case TYPE_PLINK: // draw plinks
            for (it=item->m_dyncb_vis_data.begin(); it<item->m_dyncb_vis_data.end(); it++) {
                int32_t _cb_id = item->m_type + it->sensor_id;
                if (self->dyncb->checkbox_on[_cb_id]) {
                    self->lineprop->apply_line_style (it->sensor_id, item->m_type);
                    double *_mu = it->data;
                    double xi[3], xj[3];
                    xi[0] = _mu[0]; xi[1] = _mu[1]; xi[2] = _mu[2];
                    xj[0] = _mu[3]; xj[1] = _mu[4]; xj[2] = _mu[5];
                    glBegin (GL_LINES);
                        glVertex3d (xi[0], xi[1], xi[2]);  glVertex3f(xj[0], xj[1], xj[2]); 
                    glEnd ();
                }
            }
            break;
        default:
            printf ("Unknown item type to draw. ignored\n");
    }

}

static double
get_time_elev_z (int64_t t_min, int64_t t, double scale)
{
    double new_z = (double)(t-t_min) * scale;

    return new_z;
}

static void 
isam_graph_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererISAMGraph *self = (RendererISAMGraph*) super->user;
    
    glEnable (GL_DEPTH_TEST);

    if (!self->igv)
        return;

    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0, 1.0, 0.0, 0.0);

    // time elevation
    int64_t t_min = self->igv->node_id[0];
    int64_t t_max = self->igv->node_id[self->igv->nnodes-1];
    double dt_scale = TIME_SCALE / double (t_max - t_min) ;

    // vis nodes
    dyncb_vis_item_t *vis_nodes = new dyncb_vis_item_t ();
    vis_nodes->m_type = TYPE_NODE;

    // hash table for links
    GHashTable *ht_id2node = g_hash_table_new (g_int64_hash, g_int64_equal);
    for (int i=0; i<self->igv->nnodes; i++) {
        int32_t sensor_id = 0;      // not implemented yet. node's color by sensor
        dyncb_vis_data_t vdata;
        std::copy (self->igv->mu[i], self->igv->mu[i]+6, vdata.data);
        vdata.sensor_id = sensor_id;
        if (self->param_time_elev) vdata.data[2] = get_time_elev_z (t_min, self->igv->node_id[i], dt_scale);
        vis_nodes->m_dyncb_vis_data.push_back (vdata);

        g_hash_table_insert (ht_id2node, &(self->igv->node_id[i]), &(self->igv->mu[i][0]));

        self->dyncb->add_checkbox ("POSES", sensor_id, vis_nodes->m_type);
    }
    draw_items (self, vis_nodes);

    // plot spatial uncertianty -----------------------------------------------
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_DRAW_COV)) {
        glPushMatrix ();
        
        double dx = self->igv->mu[self->igv->nnodes-1][0];
        double dy = self->igv->mu[self->igv->nnodes-1][1];
        double dz = self->igv->mu[self->igv->nnodes-1][2];

        if (self->param_time_elev) dz = TIME_SCALE;

        glTranslated (dx, dy, dz);
        
        glColor3f (0, 1.0, 1.0);
        double eigv[4];
        double eig[2];
        double theta;
        double xy_cov[4] = {self->igv->covariance[0], self->igv->covariance[1],
                            self->igv->covariance[6], self->igv->covariance[7]};
        double zy_cov[4] = {self->igv->covariance[14], self->igv->covariance[8],
                            self->igv->covariance[13], self->igv->covariance[7]};
        double zx_cov[4] = {self->igv->covariance[14], self->igv->covariance[2],
                            self->igv->covariance[12], self->igv->covariance[0]};
        
        matrix_eigen_symm_2x2 (xy_cov, eigv, eig, &theta);
        bot_gl_draw_ellipse (3*sqrt(eig[0]), 3*sqrt(eig[1]), theta, 20);
        
        glRotatef (90, 0, 1, 0); // now xy plane is zy
        matrix_eigen_symm_2x2 (zy_cov, eigv, eig, &theta);
        bot_gl_draw_ellipse (3*sqrt(eig[0]), 3*sqrt(eig[1]), theta, 20);
        
        glRotatef (90, 1, 0, 0); // now xy plane is zx
        matrix_eigen_symm_2x2 (zx_cov, eigv, eig, &theta);
        bot_gl_draw_ellipse (3*sqrt(eig[0]), 3*sqrt(eig[1]), theta, 20);
        
        glPopMatrix ();
    }
    // -------------------------------------------------------------------------

    // loop over vlinks
    dyncb_vis_item_t *vis_vlinks = new dyncb_vis_item_t ();
    vis_vlinks->m_type = TYPE_VLINK;
    for (int i=0; i<self->igv->nlinks; i++) {

        double *xi;
        double *xj;

        // find associated nodes
        // @TODO: now it is cpp, shall we use std::map?
        xi = (double*) g_hash_table_lookup (ht_id2node, &(self->igv->links_i[i]));
        xj = (double*) g_hash_table_lookup (ht_id2node, &(self->igv->links_j[i]));

        if (xi && xj) {
            dyncb_vis_data_t vdata;
            vdata.data[0] = xi[0]; vdata.data[1] = xi[1]; vdata.data[2] = xi[2];
            vdata.data[3] = xj[0]; vdata.data[4] = xj[1]; vdata.data[5] = xj[2];
            vdata.sensor_id = self->igv->link_sensor_id[i];
            if (self->param_time_elev) {
                vdata.data[2] = get_time_elev_z (t_min, self->igv->links_i[i], dt_scale);
                vdata.data[5] = get_time_elev_z (t_min, self->igv->links_j[i], dt_scale);
            }
            vis_vlinks->m_dyncb_vis_data.push_back (vdata);
            
            switch (self->igv->link_sensor_id[i]) {
                case PERLLCM_ISAM_VLINK_T_SENSOR_ODOMETRY:
                    self->dyncb->add_checkbox ("VLINK - ODO", self->igv->link_sensor_id[i], vis_vlinks->m_type);
                    break;
                case PERLLCM_ISAM_VLINK_T_SENSOR_CAMERA:
                    self->dyncb->add_checkbox ("VLINK - CAM", self->igv->link_sensor_id[i], vis_vlinks->m_type);
                    break;
                case PERLLCM_ISAM_VLINK_T_SENSOR_LASER:
                    self->dyncb->add_checkbox ("VLINK - LASER", self->igv->link_sensor_id[i], vis_vlinks->m_type);
                    break;
                case PERLLCM_ISAM_VLINK_T_SENSOR_SONAR:
                    self->dyncb->add_checkbox ("VLINK - SONAR", self->igv->link_sensor_id[i], vis_vlinks->m_type);
                    break;
                case PERLLCM_ISAM_VLINK_T_SENSOR_GLC:
                    self->dyncb->add_checkbox ("VLINK - GLC", self->igv->link_sensor_id[i], vis_vlinks->m_type);
                    break;
                default :
                    break;
            }
        }
    }
    draw_items (self, vis_vlinks); 
    
    // loop over plinks
    dyncb_vis_item_t *vis_plinks = new dyncb_vis_item_t ();
    vis_plinks->m_type = TYPE_PLINK;
    if (self->gl_plink) {
        GList *plinks_list = g_list_first (self->gl_plink);
        perllcm_isam_plink_t *plink = NULL;
        do {
            plink = (perllcm_isam_plink_t*) plinks_list->data;
            
            double *xi;
            double *xj;
    
            // find associated nodes
            xi = (double*) g_hash_table_lookup (ht_id2node, &(plink->utime_i));
            xj = (double*) g_hash_table_lookup (ht_id2node, &(plink->utime_j));
        
            if (xi && xj) {
                dyncb_vis_data_t vdata;
                vdata.data[0] = xi[0]; vdata.data[1] = xi[1]; vdata.data[2] = xi[2];
                vdata.data[3] = xj[0]; vdata.data[4] = xj[1]; vdata.data[5] = xj[2];
                vdata.sensor_id = plink->sensor_id;
                if (self->param_time_elev) {
                    vdata.data[2] = get_time_elev_z (t_min, plink->utime_i, dt_scale);
                    vdata.data[5] = get_time_elev_z (t_min, plink->utime_j, dt_scale);
                }
                vis_plinks->m_dyncb_vis_data.push_back (vdata);

                switch (plink->sensor_id) {
                    case PERLLCM_ISAM_VLINK_T_SENSOR_ODOMETRY:
                        self->dyncb->add_checkbox ("PLINK - ODO", plink->sensor_id, vis_plinks->m_type);
                        break;
                    case PERLLCM_ISAM_VLINK_T_SENSOR_CAMERA:
                        self->dyncb->add_checkbox ("PLINK - CAM", plink->sensor_id, vis_plinks->m_type);
                        break;
                    case PERLLCM_ISAM_VLINK_T_SENSOR_LASER:
                        self->dyncb->add_checkbox ("PLINK - LASER", plink->sensor_id, vis_plinks->m_type);
                        break;
                    case PERLLCM_ISAM_VLINK_T_SENSOR_SONAR:
                        self->dyncb->add_checkbox ("PLINK - SONAR", plink->sensor_id, vis_plinks->m_type);
                        break;
                    case PERLLCM_ISAM_VLINK_T_SENSOR_GLC:
                        self->dyncb->add_checkbox ("PLINK - GLC", plink->sensor_id, vis_plinks->m_type);
                        break;
                    default :
                        break;
                }
            } 
                
        } while ((plinks_list = g_list_next (plinks_list)));
    }
    
    if (self->gl_vlink) {
        GList *vlinks_list = g_list_first (self->gl_vlink);
        perllcm_isam_vlink_t *vlink = NULL;
        do {
            vlink = (perllcm_isam_vlink_t*) vlinks_list->data;
            
            if (vlink->accept) continue;

            double *xi;
            double *xj;
    
            // find associated nodes
            xi = (double*) g_hash_table_lookup (ht_id2node, &(vlink->id1));
            xj = (double*) g_hash_table_lookup (ht_id2node, &(vlink->id2));
        
            if (xi && xj) {
                dyncb_vis_data_t vdata;
                vdata.data[0] = xi[0]; vdata.data[1] = xi[1]; vdata.data[2] = xi[2];
                vdata.data[3] = xj[0]; vdata.data[4] = xj[1]; vdata.data[5] = xj[2];
                vdata.sensor_id = vlink->sensor_id;
                if (self->param_time_elev) {
                    vdata.data[2] = get_time_elev_z (t_min, vlink->id1, dt_scale);
                    vdata.data[5] = get_time_elev_z (t_min, vlink->id2, dt_scale);
                }
                vis_plinks->m_dyncb_vis_data.push_back (vdata);

                switch (vlink->sensor_id) {
                    case PERLLCM_ISAM_VLINK_T_SENSOR_ODOMETRY:
                        self->dyncb->add_checkbox ("PLINK - ODO", vlink->sensor_id, vis_plinks->m_type);
                        break;
                    case PERLLCM_ISAM_VLINK_T_SENSOR_CAMERA:
                        self->dyncb->add_checkbox ("PLINK - CAM", vlink->sensor_id, vis_plinks->m_type);
                        break;
                    case PERLLCM_ISAM_VLINK_T_SENSOR_LASER:
                        self->dyncb->add_checkbox ("PLINK - LASER", vlink->sensor_id, vis_plinks->m_type);
                        break;
                    case PERLLCM_ISAM_VLINK_T_SENSOR_SONAR:
                        self->dyncb->add_checkbox ("PLINK - SONAR", vlink->sensor_id, vis_plinks->m_type);
                        break;
                    case PERLLCM_ISAM_VLINK_T_SENSOR_GLC:
                        self->dyncb->add_checkbox ("PLINK - GLC", vlink->sensor_id, vis_plinks->m_type);
                        break;
                    default :
                        break;
                }
            }
                
        } while ((vlinks_list = g_list_next (vlinks_list)));
    }
    draw_items (self, vis_plinks);

    // clean up
    g_hash_table_destroy (ht_id2node);
    delete vis_nodes;
    delete vis_plinks;
    delete vis_vlinks;
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererISAMGraph *self = (RendererISAMGraph*) user;

    self->param_time_elev = bot_gtk_param_widget_get_bool (self->pw, PARAM_TIME_ELEV);
    bot_viewer_request_redraw (self->viewer);
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererISAMGraph *self = (RendererISAMGraph*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererISAMGraph *self = (RendererISAMGraph*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static void 
on_save_graph (GtkWidget *button, void *user_data)
{
    RendererISAMGraph *self = (RendererISAMGraph*) user_data;

    // choose the file
    GtkWidget *dialog;
    dialog = gtk_file_chooser_dialog_new ("Select directory to save the graph",
                                          NULL,
                                          GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER,
                                          GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
                                          GTK_STOCK_SAVE, GTK_RESPONSE_ACCEPT,
                                          NULL);

    char *savepath = NULL;
    if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT)
        savepath = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));

    gtk_widget_destroy (dialog);

    // publish to seserver
    if (savepath) {
        perllcm_isam_cmd_t cmd = {0};
        cmd.utime = timestamp_now();
        cmd.mode = PERLLCM_ISAM_CMD_T_MODE_SAVE;
        cmd.savepath = savepath;
        cmd.graphfile = (char*) "";
        cmd.load_done = 0;
        cmd.utime_conn = 0;
        cmd.cov_conn = 0.0;
        perllcm_isam_cmd_t_publish (self->lcm, CMD_CHANNEL, &cmd);
    }

    g_free (savepath);
}

static void 
on_load_graph (GtkWidget *button, void *user_data)
{
    //RendererVanctrl *self = (RendererVanctrl*) user_data;

    // choose the file
    GtkWidget *dialog;
    dialog = gtk_file_chooser_dialog_new ("Select directory for existing graph",
            NULL,
            GTK_FILE_CHOOSER_ACTION_OPEN,
            GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
            GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT,
            NULL);

    char *loadpath = NULL;
    if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT)
        loadpath = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));

    gtk_widget_destroy (dialog);

    // publish to seserver
    char savepath[] = "";
    if (loadpath) {
        RendererISAMGraph *self = (RendererISAMGraph*) user_data;
        perllcm_isam_cmd_t cmd = {0};
        cmd.mode = PERLLCM_ISAM_CMD_T_MODE_LOAD;
        cmd.savepath = savepath;
        cmd.graphfile = loadpath;
        cmd.load_done = 0;
        cmd.utime_conn = 0;
        cmd.cov_conn = 0.0;
        perllcm_isam_cmd_t_publish (self->lcm, CMD_CHANNEL, &cmd);
    }

    g_free (loadpath);
}

static void
on_batch_button (GtkWidget *button, void *user_data)
{
    RendererISAMGraph *self = (RendererISAMGraph*) user_data;

    perllcm_isam_cmd_t cmd = {0};
    cmd.utime = timestamp_now();
    cmd.mode = PERLLCM_ISAM_CMD_T_MODE_BATCH;
    cmd.savepath = (char*) "";
    cmd.graphfile = (char*) "";
    cmd.load_done = 0;
    cmd.utime_conn = 0;
    cmd.cov_conn = 0.0;
    perllcm_isam_cmd_t_publish (self->lcm, CMD_CHANNEL, &cmd);
}

static void
on_toggle_button (GtkWidget *button, void *user_data)
{
    RendererISAMGraph *self = (RendererISAMGraph *)user_data;
    GtkToggleButton *firstButton = NULL;

    if (self->dyncb->cb_ptr.size())
        firstButton = (GtkToggleButton *)self->dyncb->cb_ptr[0];

    if (gtk_toggle_button_get_active(firstButton))
        for (size_t i=0; i<self->dyncb->cb_ptr.size(); i++)
            gtk_toggle_button_set_active((GtkToggleButton *)self->dyncb->cb_ptr[i], FALSE);
    else
        for (size_t i=0; i<self->dyncb->cb_ptr.size(); i++)
            gtk_toggle_button_set_active((GtkToggleButton *)self->dyncb->cb_ptr[i], TRUE);
        
}

static RendererISAMGraph *
new_renderer_isam_graph (BotViewer *viewer, char *rootkey)
{    
    RendererISAMGraph *self = new RendererISAMGraph ();
    
    self->lcm = bot_lcm_get_global (NULL);

    // load from config file
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);

    snprintf (key, sizeof key, "viewer.renderers.%s.channel", rootkey);
    char *lcm_channel = bot_param_get_str_or_fail (self->param, key);

    // display link proposal from plink channel
    snprintf (key, sizeof key, "viewer.renderers.%s.plink_channels", rootkey);
    int n_plink_ch = bot_param_get_array_len (self->param, key);
    if (n_plink_ch > 0) {
        char **plink_channels = bot_param_get_str_array_alloc (self->param, key);
        for (int i=0; i<n_plink_ch; i++)
            perllcm_isam_plink_collection_t_subscribe (self->lcm, plink_channels[i], &plink_t_cb, self);

        bot_param_str_array_free (plink_channels);
    }

    // display link proposal from vlink channel
    snprintf (key, sizeof key, "viewer.renderers.%s.vlink_channels", rootkey);
    int n_vlink_ch = bot_param_get_array_len (self->param, key);
    if (n_vlink_ch > 0) {
        char **vlink_channels = bot_param_get_str_array_alloc (self->param, key);
        for (int i=0; i<n_vlink_ch; i++)
            perllcm_isam_vlink_t_subscribe (self->lcm, vlink_channels[i], &vlink_t_cb, self);

        bot_param_str_array_free (vlink_channels);
    }

    BotRenderer *renderer = &self->renderer;
    renderer->draw = isam_graph_renderer_draw;
    renderer->destroy = _renderer_free;
    renderer->widget = bot_gtk_param_widget_new ();
    renderer->name = renderer_label;
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);
    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw),FALSE, TRUE, 0);
    gtk_widget_show (GTK_WIDGET (self->pw));
    
    // check boxes
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_DRAW_TRI, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_DRAW_COV, 1, NULL);

    // time elevation
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_TIME_ELEV, 0, NULL);

    // buttons
    GtkWidget *batch_button = gtk_button_new_with_label ("Batch");
    gtk_box_pack_start (GTK_BOX (vbox), batch_button, FALSE, FALSE, 0);
    g_signal_connect (G_OBJECT (batch_button), "clicked", G_CALLBACK (on_batch_button), self);

    GtkWidget *toggle_button = gtk_button_new_with_label("Toggle All");
    gtk_box_pack_start(GTK_BOX(vbox), toggle_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(toggle_button), "clicked", G_CALLBACK(on_toggle_button), self);

    self->status_txt = gtk_label_new ("Status:");
    gtk_box_pack_start (GTK_BOX (vbox),  self->status_txt, FALSE, FALSE, 0);
    gtk_label_set_justify (GTK_LABEL (self->status_txt), GTK_JUSTIFY_LEFT);

    // menu: save | load
    GtkWidget *save_graph_fmenu = gtk_menu_item_new_with_mnemonic ("_Save graph");
    gtk_menu_append (GTK_MENU(viewer->file_menu), save_graph_fmenu);
    gtk_widget_show (save_graph_fmenu);
    g_signal_connect (G_OBJECT (save_graph_fmenu), "activate",
                      G_CALLBACK (on_save_graph), self);

    GtkWidget *load_graph_fmenu = gtk_menu_item_new_with_mnemonic ("_Load existing graph...");
    gtk_menu_append (GTK_MENU(viewer->file_menu), load_graph_fmenu);
    gtk_widget_show (load_graph_fmenu);
    g_signal_connect (G_OBJECT (load_graph_fmenu), "activate",
            G_CALLBACK (on_load_graph), self);

    // END layout sidebar widgets ----------------------------------------------

    gtk_widget_show_all (renderer->widget);

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);

    perllcm_isam_graph_vis_t_subscribe (self->lcm, lcm_channel, &graph_cb, self);
    
    self->igv = NULL;
    self->gl_plink = NULL;
    self->gl_vlink = NULL;

    // line property
    self->lineprop = new LinepltProp (self->param, rootkey);

    // dynamic checkbox
    self->dyncb = new DynCheckbox (viewer, vbox, self->lineprop);

    return self;
}

void
setup_renderer_isam_graph (BotViewer *viewer, char *rootkey, int priority)
{
    RendererISAMGraph *self = new_renderer_isam_graph (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
