#include <stdio.h>
#include <stdlib.h>

#include "perls-lcmtypes/perllcm_pvn_laser_map_t.h"
#include "perls-lcmtypes/perllcm_pvn_vis_map_t.h"
#include "perls-lcmtypes/perllcm_pvn_eview_map_t.h"
#include "perls-lcmtypes/perllcm_pvn_eview_map_neighborhood_t.h"

#include "perls-common/error.h"
#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/units.h"

#include "renderers.h"

#ifdef __PERLS_PVN__
#include "perls-pvn/pvn_util.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RENDERER_NAME "PVN-Map"

#define PARAM_ENABLE "Enable"
#define PARAM_COLOR_MENU "Color"

#define EXEMPLAR_OFFSET 7
#define PARAM_EXEMPLAR_OFFSET "Exemplar Offset"

enum {
    COLOR_SINGLE,
    COLOR_PER_SCAN,
    COLOR_Z,
    COLOR_INTENSITY,
    COLOR_RGB,
};


typedef struct _RendererPVNMap RendererPVNMap;
struct _RendererPVNMap {
    BotRenderer renderer;
    
    lcm_t *lcm;
    BotParam *param;
    
    perllcm_pvn_laser_map_t *lmap;
    perllcm_pvn_vis_map_t *vmap;
    perllcm_pvn_eview_map_t *evmap;
    
    GLuint  map_dl; // map display list, otherwise rendering is very slow
    int map_dl_ready;
    
    BotViewer         *viewer;
    BotGtkParamWidget *pw;   
};

static void
_renderer_free (BotRenderer *super)
{
    RendererPVNMap *self = (RendererPVNMap *)super->user;
    free (self);
}



GLuint
compile_lmap_display_list (RendererPVNMap *self) {
    
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glAlphaFunc (GL_GREATER, 0.5);
    glEnable (GL_ALPHA_TEST);
    glPointSize(3.0);
    
    for (int i=0; i<self->lmap->num_lrcs; i++) {   
	perllcm_velodyne_laser_return_collection_t *lrc = &(self->lmap->lrcs[i]);
	
	glPushMatrix();
	
	double dx, dy, dz;
        double rot_x, rot_y, rot_z;
 
        // move into sensor frame
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
	
	if (COLOR_PER_SCAN == bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU)) {
            float rcolor[4] = {0};
            bot_color_util_rand_color (rcolor, 1.0, 0.1);
	    glColor3fv (rcolor);
        }
        if (COLOR_SINGLE == bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU))
            glColor3f (0.0, 1.0, 1.0);

	glBegin(GL_POINTS);
	for (int s = 0; s < (lrc->num_lr+lrc->num_lrl); s++) {
	    
	    uint8_t intensity=1;
	    double r=0, b=0, g=0;
	    double x=1, y=1, z=1;
	    if (s < lrc->num_lr) {
		perllcm_velodyne_laser_return_t *lr = &(lrc->laser_returns[s]);
		intensity = lr->intensity;
		x = lr->xyz[0];
		y = lr->xyz[1];
		z = lr->xyz[2];
	    } else {
		perllcm_velodyne_laser_return_lite_t *lr = &(lrc->laser_returns_lite[s - lrc->num_lr]);
		intensity = lr->intensity;
		x = lr->xyz[0];
		y = lr->xyz[1];
		z = lr->xyz[2];
	    }
	    if (self->lmap->num_rgbcs > 0) {
		r = self->lmap->rgbcs[i].r[s]/255.0;
		g = self->lmap->rgbcs[i].g[s]/255.0;
		b = self->lmap->rgbcs[i].b[s]/255.0;
	    }

	    
	    switch (bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU))  {
                case COLOR_INTENSITY: {
                    double v = (intensity)/255.0; // normalize the intensity values
                    glColor3d (v, v, v);
                    break;
                }
                case COLOR_Z: {
                    double z_norm = (z - self->lmap->xyz_min[2]) / (self->lmap->xyz_max[2] - self->lmap->xyz_min[2]);
		    //z_norm = (z_norm - 0.5)*1.5 + z_norm; // otherwise all in the middle
                    glColor3fv (bot_color_util_jet(z_norm));
                    break;
                }
		case COLOR_RGB: {
                    glColor3d (r, g, b);
                    break;
                }
		
		
                break;
            }
	    glVertex3f(x,y,z);

	    
	}
	glEnd();
	
	
	glPopMatrix();
    }
    
    glEndList ();
    
    //free (pData);
    
    return dl;
}


GLuint
compile_vmap_display_list (RendererPVNMap *self) {
   
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glAlphaFunc (GL_GREATER, 0.5);
    glEnable (GL_ALPHA_TEST);

    for (int i=0; i<self->vmap->nclusters; i++) {   
	perllcm_pvn_vis_map_cluster_t *vmc = &(self->vmap->clusters[i]);
	
	glPushMatrix();
	
	glColor3f (1.0, 0.0, 0.0);
	if (COLOR_PER_SCAN == bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU)) {
            float rcolor[4] = {0};
            bot_color_util_rand_color (rcolor, 1.0, 0.1);
	    glColor3fv (rcolor);
        }
        if (COLOR_SINGLE == bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU))
            glColor3f (0.0, 1.0, 1.0);
	    
	    
	glPointSize(10.0);
	glBegin(GL_POINTS);
	    glVertex3f(vmc->mean_xyz[0],
                       vmc->mean_xyz[1],
                       vmc->mean_xyz[2]);
	glEnd();
	glBegin(GL_LINES);
	    glVertex3f(vmc->mean_xyz[0],
                       vmc->mean_xyz[1],
                       vmc->mean_xyz[2]);
	    // plot towards camera so sin and cos flipped
	    glVertex3f(vmc->mean_xyz[0] + 1*sin(vmc->mean_view_aiz),
                       vmc->mean_xyz[1] + 1*cos(vmc->mean_view_aiz),
                       vmc->mean_xyz[2]);
	glEnd();

	glPointSize(2.0);
	glBegin(GL_POINTS);
	for (int j = 0; j<vmc->npts; j++) {
	
	    switch (bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU))  {
		case COLOR_INTENSITY: // no intensity for vmap
                case COLOR_Z: {
                    double z_norm = (vmc->z[j] - self->vmap->xyz_min[2]) / (self->vmap->xyz_max[2] - self->vmap->xyz_min[2]);
		    z_norm = (z_norm - 0.5)*1.5 + z_norm; // otherwise all in the middle
                    glColor3fv (bot_color_util_jet(z_norm));
                    break;
                }
                break;
            }
	    glVertex3f(vmc->x[j],vmc->y[j],vmc->z[j]);
	}
	glEnd();
	
	glPopMatrix();
    }
    
    glEndList ();
    
    //free (pData);
    
    return dl;
}


GLuint
compile_evmap_display_list (RendererPVNMap *self) {
   
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glAlphaFunc (GL_GREATER, 0.5);
    glEnable (GL_ALPHA_TEST);
/*
    for (int i=0; i<self->evmap->nn; i++) {   
	perllcm_pvn_eview_map_neighborhood_t *mn = &(self->evmap->neighborhoods[i]);
	
	glPushMatrix();
	
	glColor3f (1.0, 0.0, 0.0);
	if (COLOR_PER_SCAN == bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU)) {
            float rcolor[4] = {0};
            bot_color_util_rand_color (rcolor, 1.0, 0.1);
	    glColor3fv (rcolor);
        }
        if (COLOR_SINGLE == bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU))
            glColor3f (0.0, 1.0, 1.0);
	    
	    
	glPointSize(10.0);
	glBegin(GL_POINTS);
	    glVertex3f(mn->pose.mu[0],
                       mn->pose.mu[1],
                       mn->pose.mu[2]);
	glEnd();
	
	// move into neighborhood frame
	glTranslated (mn->pose.mu[0], mn->pose.mu[1], mn->pose.mu[2]);
	glRotatef (mn->pose.mu[5] * RTOD, 0, 0, 1);
	glRotatef (mn->pose.mu[4] * RTOD, 0, 1, 0);
	glRotatef (mn->pose.mu[3] * RTOD, 1, 0, 0);    
	
	// for each neighboorhood
	for (int j=0; j<mn->ne; j++) {
	    
	    perllcm_pvn_eview_map_exemplar_t *mne = &(mn->exemplars[j]);
	    
	    double z_off = 0;
	    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_EXEMPLAR_OFFSET))
		z_off = EXEMPLAR_OFFSET*j;
		
	    
	    glPointSize(10.0);
	    glBegin(GL_POINTS);
		glVertex3f(mne->x_n_e.mu[0], mne->x_n_e.mu[1], mne->x_n_e.mu[2]-z_off);
	    glEnd();
	    glBegin(GL_LINES);
		glVertex3f(mne->x_n_e.mu[0], mne->x_n_e.mu[1], mne->x_n_e.mu[2]-z_off);
		glVertex3f(0.0, 0.0, 0.0);
	    glEnd();
	    
	    glPushMatrix();
	    
	    // move into exemplar frame
	    glTranslated (mne->x_n_e.mu[0], mne->x_n_e.mu[1], mne->x_n_e.mu[2]-z_off);
	    glRotatef (mne->x_n_e.mu[5] * RTOD, 0, 0, 1);
	    glRotatef (mne->x_n_e.mu[4] * RTOD, 0, 1, 0);
	    glRotatef (mne->x_n_e.mu[3] * RTOD, 1, 0, 0);    
	    
	    glPointSize(2.0);
	    glBegin(GL_POINTS);
	    for (int k=0; k<mne->npts; k++) {
	    
		switch (bot_gtk_param_widget_get_enum (self->pw, PARAM_COLOR_MENU))  {
		    case COLOR_INTENSITY: // no intensity for vmap
		    case COLOR_Z: {
			double z_norm = (mne->z[k] - mne->min_xyz[2]) / (mne->max_xyz[2] - mne->min_xyz[2]);
			z_norm = (z_norm - 0.5)*1.5 + z_norm; // otherwise all in the middle
			glColor3fv (bot_color_util_jet(z_norm));
			break;
		    }
		    break;
		}
		glVertex3f(mne->x[k], mne->y[k], mne->z[k]);
	    }
	    glEnd();
	    
	    
	    glPopMatrix();    
	}
	
	glPopMatrix();
    }
*/    
    glEndList ();
        
    return dl;
}

GLuint
compile_map_display_list (RendererPVNMap *self)
{
    
    if (self->lmap != NULL) {
	return compile_lmap_display_list (self);
    } else if (self->vmap != NULL) {
	return compile_vmap_display_list (self);
    } else if (self->evmap != NULL) {
	return compile_evmap_display_list (self);
    }
    
    ERROR ("No map to build display list from");
    return 0;
}

static void 
pvn_map_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererPVNMap *self = (RendererPVNMap *)super->user;
    
    glEnable (GL_DEPTH_TEST);

    if (!bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE))
        return;

    if (!self->map_dl_ready && (self->lmap != NULL ||
				self->vmap != NULL ||
				self->evmap != NULL)) {
        self->map_dl = compile_map_display_list (self);
        self->map_dl_ready = 1;
    }
    
    if (!self->map_dl_ready) {
	return;
    }
    
    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0, 1.0, 0.0, 0.0);
    
    glPushMatrix ();
    glEnable (GL_BLEND);
    glCallList (self->map_dl);
    glPopMatrix ();    
    
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererPVNMap *self = (RendererPVNMap *)user;
    
    if (strcmp (name, PARAM_ENABLE) &&
       !(strcmp (name, PARAM_EXEMPLAR_OFFSET) == 0 && self->evmap == NULL) ) { // if any of the param widgets except enabled have changed 
	
	self->map_dl_ready = 0;
    }
    
    bot_viewer_request_redraw (self->viewer);
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPVNMap *self = (RendererPVNMap *)user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPVNMap *self = (RendererPVNMap *)user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static void 
on_load_map (GtkWidget *button, void *user_data)
{
    RendererPVNMap *self = (RendererPVNMap *)user_data;

    self->map_dl_ready = 0;

    // choose the file
    GtkWidget *dialog;
    dialog = gtk_file_chooser_dialog_new ("Select Laser Map File",
                                          NULL,
                                          GTK_FILE_CHOOSER_ACTION_OPEN,
                                          GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
                                          GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT,
                                          NULL);

    char *file = NULL;
    if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT){
        file  = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
	gtk_widget_destroy (dialog);
    } else {
	gtk_widget_destroy (dialog);
	return;
    }
    
    if (NULL != self->lmap) {
        perllcm_pvn_laser_map_t_destroy (self->lmap);
        self->lmap = NULL;
    }
    if (NULL != self->vmap) {
        perllcm_pvn_vis_map_t_destroy (self->vmap);
        self->vmap = NULL;
    }
    if (NULL != self->vmap) {
        perllcm_pvn_eview_map_t_destroy (self->evmap);
        self->evmap = NULL;
    }
    
    int end = strlen (file);
    if (0 == strcmp (&file[end-5], ".lmap")) {
	int32_t ret = LCMU_FREAD (file, &(self->lmap), perllcm_pvn_laser_map_t);
	if (ret < 0)
	    ERROR ("couldn't read %s from disk!", file);
    } else if (0 == strcmp (&file[end-5], ".vmap")) {
	int32_t ret = LCMU_FREAD (file, &(self->vmap), perllcm_pvn_vis_map_t);
	if (ret < 0)
	    ERROR ("couldn't read %s from disk!", file);
    } else if (0 == strcmp (&file[end-6], ".evmap")) {
	int32_t ret = LCMU_FREAD (file, &(self->evmap), perllcm_pvn_eview_map_t);
	if (ret < 0)
	    ERROR ("couldn't read %s from disk!", file);
    } else {
	ERROR ("Unknown file type %s. Must be .lmap, .vmap or .evmap", &file[end-5]);
    }
    
    bot_viewer_request_redraw (self->viewer);
}

static RendererPVNMap *
new_renderer_pvn_map (BotViewer *viewer, char *rootkey)
{    
    RendererPVNMap *self = (RendererPVNMap *)calloc (1, sizeof (*self));
    
    self->lcm = bot_lcm_get_global (NULL);

    // load from config file
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);
    
    self->lmap = NULL;
    self->vmap = NULL;
    self->evmap = NULL;
    self->map_dl_ready = 0;

    BotRenderer *renderer = &self->renderer;
    renderer->draw = pvn_map_renderer_draw;
    renderer->destroy = _renderer_free;
    renderer->widget = bot_gtk_param_widget_new();
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
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint)0, PARAM_ENABLE, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint)0, PARAM_EXEMPLAR_OFFSET, 1, NULL);
    
    // buttons
    GtkWidget *load_button = gtk_button_new_with_label ("Load Map");
    gtk_box_pack_start (GTK_BOX (vbox), load_button, FALSE, FALSE, 0);
    g_signal_connect (G_OBJECT (load_button), "clicked", G_CALLBACK (on_load_map), self);

    // menu
    GtkWidget *load_fmenu = gtk_menu_item_new_with_mnemonic ("Load Laser Map");
    gtk_menu_append (GTK_MENU(viewer->file_menu), load_fmenu);
    gtk_widget_show (load_fmenu);
    g_signal_connect (G_OBJECT (load_fmenu), "activate", G_CALLBACK (on_load_map), self);

    bot_gtk_param_widget_add_enum (self->pw, PARAM_COLOR_MENU, BOT_GTK_PARAM_WIDGET_MENU, COLOR_Z,
                                   "Single Color", COLOR_SINGLE,
                                   "Height", COLOR_Z,
                                   "Intensity", COLOR_INTENSITY,
				   "RGB", COLOR_RGB,
                                   "Per Scan/Cluster", COLOR_PER_SCAN,
                                   NULL);

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
#endif //__PERLS_PVN__

void
setup_renderer_pvn_map (BotViewer *viewer, char *rootkey, int priority)
{
#ifdef __PERLS_PVN__
    RendererPVNMap *self = new_renderer_pvn_map (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
#else
    ERROR ("perls-pvn required!");
#endif //__PERLS_PVN__
}
