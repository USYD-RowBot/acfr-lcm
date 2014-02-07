#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include "perls-common/bot_util.h"

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_viewer_image_pccs_t.h"

#include "renderers.h"

#define RENDERER_NAME "Renderer Image"
#define PARAM_ENABLE_DISPLAY "Enable Image Display"
#define PARAM_ENABLE_FEAT_DISPLAY "Display Features"
#define PARAM_ENABLE_SCENE_PRIOR_DISPLAY "Display Scene Prior"
#define PARAM_ENABLE_PUTATIVE_DISPLAY "Display Putative Corrs"
#define PARAM_ENABLE_INLIER_DISPLAY "Display Inlier Corrs"
#define PARAM_DISPLAY_SCALE "Display Scale"

#define NUM_RCOLORS 200

typedef struct _RendererImage RendererImage;
struct _RendererImage {
    //variables for to extend viewer "class"
    BotRenderer       renderer;
    BotViewer         *viewer;
    BotGtkParamWidget *pw;
    
    //lcm holders
    lcm_t *lcm;
    bot_core_image_t *last_image;
    perllcm_van_feature_collection_t *last_feature_collection;
    perllcm_van_scene_prior_t *last_scene_prior;
    perllcm_viewer_image_pccs_t *last_pccs_putative;
    perllcm_viewer_image_pccs_t *last_pccs_inlier;
    
    int r90cw;
    int r90ccw;
    
    float rcolors[NUM_RCOLORS][3];

    BotParam *param;

    // variables for sidebar widget controls
    double scale; //scale for plotting, controlled by slider
};

static void
on_feature_collection_t (const lcm_recv_buf_t *rbuf, const char *channel,
                         const perllcm_van_feature_collection_t *msg, void *user)
{
    RendererImage *self = (RendererImage*) user;
    
    if (self->last_feature_collection)
        perllcm_van_feature_collection_t_destroy (self->last_feature_collection);
    
    self->last_feature_collection = perllcm_van_feature_collection_t_copy (msg); 
    
    if (self->last_feature_collection->scene_prior.npts > 0) {
	if (self->last_scene_prior)
	    perllcm_van_scene_prior_t_destroy (self->last_scene_prior);
	
	self->last_scene_prior = perllcm_van_scene_prior_t_copy (&self->last_feature_collection->scene_prior);
    }
    
    bot_viewer_request_redraw (self->viewer);   
}

// LCM image_t callback
static void
on_bot_core_image_t (const lcm_recv_buf_t *rbuf, const char *channel,
                     const bot_core_image_t *msg, void *user)
{
    RendererImage *self = (RendererImage*) user;
    
    if(self->last_image)
        bot_core_image_t_destroy (self->last_image);
    
    self->last_image = bot_core_image_t_copy (msg); //destory is in image_free()
    bot_viewer_request_redraw (self->viewer);   
}

static void
on_perllcm_van_scene_prior_t (const lcm_recv_buf_t *rbuf, const char *channel,
			      const perllcm_van_scene_prior_t *msg, void *user)
{
    RendererImage *self = (RendererImage*) user;
    
    if (self->last_scene_prior)
	    perllcm_van_scene_prior_t_destroy (self->last_scene_prior);
	
    self->last_scene_prior = perllcm_van_scene_prior_t_copy (msg);
    bot_viewer_request_redraw (self->viewer);   
}

static void
on_perllcm_viewer_image_pccs_t (const lcm_recv_buf_t *rbuf, const char *channel,
			      const perllcm_viewer_image_pccs_t *msg, void *user)
{
    RendererImage *self = (RendererImage*) user;

    if (msg->type == PERLLCM_VIEWER_IMAGE_PCCS_T_TYPE_PUTATIVE) {
	if (self->last_pccs_putative)
	    perllcm_viewer_image_pccs_t_destroy (self->last_pccs_putative);
    
	self->last_pccs_putative = perllcm_viewer_image_pccs_t_copy (msg);
	
    } else if (msg->type == PERLLCM_VIEWER_IMAGE_PCCS_T_TYPE_INLIER) {
	if (self->last_pccs_inlier)
	    perllcm_viewer_image_pccs_t_destroy (self->last_pccs_inlier);
		
	self->last_pccs_inlier = perllcm_viewer_image_pccs_t_copy (msg);
    }
    
    bot_viewer_request_redraw (self->viewer);   
}

static void
image_draw (BotViewer *viewer, BotRenderer *renderer)
{
    GLuint texName;
    GLint viewport[4];
    uint8_t *img_data = NULL;
        
    RendererImage *self = (RendererImage*) renderer->user;
    g_assert (self->viewer == viewer);  
    //dont try to access image data if we havent received any lcm image messages yet
    if(!self->last_image || !bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_DISPLAY))
        return;
    
    int img_width = self->last_image->width;
    int img_height = self->last_image->height;
    img_data = self->last_image->data;
      
    // get the window viewport [x_pos y_pos width height]
    glGetIntegerv (GL_VIEWPORT, viewport);
    
    glMatrixMode (GL_PROJECTION); // Applies subsequent matrix operations to the projection matrix stack.
    glPushMatrix (); //pushes  the  current  matrix  stack  down by one, duplicating the current matrix.
    glLoadIdentity (); // replaces  the  current matrix in stack with the identity matrix
    //glOrtho poduces a parallel projection.  The current matrix is multiplied  by  this  matrix  and the result replaces the current matrix
    //void glOrtho( GLdouble left,GLdouble right,GLdouble bottom,GLdouble top,GLdouble near_val,GLdouble far_val)
    glOrtho(0.0f, viewport[2], viewport[3], 0.0f, 0.0f, 1.0f);
  
    glMatrixMode (GL_MODELVIEW);
    glPushMatrix ();
    glLoadIdentity ();
    
    glScalef (self->scale, self->scale, 1);
    if (self->r90cw) {
	glTranslated (img_height, 0.0, 0.0);
	glRotatef (90, 0, 0, 1);
    } else if (self->r90ccw) {
	glTranslated (0.0, img_width, 0.0);
	glRotatef (-90, 0.0, 0.0, 1);
    }
    //printf("%d %d %d %d\n", viewport[0], viewport[1], viewport[2], viewport[3]); 
  
    glPushAttrib (GL_DEPTH_BUFFER_BIT); 
    glDisable (GL_DEPTH_TEST);
    

    // Setup texture parameters
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1); //set storage mode for pixel operations (byte alignment)
    glGenTextures (1, &texName); //generate one texture name
    glBindTexture (GL_TEXTURE_2D, texName); //assoc a new 2d texture with this name
    //assign texture parameters
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); //repeat in s direction
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); //repeat in t direction
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); //zoom out sampling by nearst pixel
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); //zoom in sampling by nearst pixel
    
    //map image data on to graphical primitive
    glTexImage2D (GL_TEXTURE_2D, 0, GL_INTENSITY8, img_width, img_height,
                  0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img_data);

   
    glEnable (GL_TEXTURE_2D); //enable 2D texture mapping
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //define how texture data is interprated (replace existing)
    glBindTexture (GL_TEXTURE_2D, texName);
    
    glColor3f (1.0, 1.0, 1.0); //set current color to white
    //start a 4 vertice quadrilateral.
    glBegin (GL_QUADS);
      //maps 2d point to 3d location
      glTexCoord2f (0.0, 1.0); glVertex3f(0, img_height, 0);   //lower left
      glTexCoord2f (1.0, 1.0); glVertex3f(img_width, img_height, 0);   //lower right
      glTexCoord2f (1.0, 0.0); glVertex3f(img_width, 0, 0);   //upper right
      glTexCoord2f (0.0, 0.0); glVertex3f(0, 0, 0);  //upper left
    glEnd ();
    
    glDisable (GL_TEXTURE_2D);


    //plot features
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_FEAT_DISPLAY) &&
	self->last_feature_collection &&
       (self->last_feature_collection->utime==self->last_image->utime)) {
		
	glPointSize (3.0);
	glBegin (GL_POINTS);
	glColor3f (0.0, 0.0, 1.0);
	for (int n=0; n < self->last_feature_collection->ntypes; n++) {	
	    //perllcm_van_feature_t *fi = &pdata->fci->f[n];
	    perllcm_van_feature_t *f = &self->last_feature_collection->f[n];
	    for (int i=1; i < f->npts; i++) {
		glVertex3f (f->u[i], f->v[i], 0);
	    }
	}
	glEnd ();
    }
    
    // plot scene prior
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_SCENE_PRIOR_DISPLAY) &&
	self->last_scene_prior &&
       (self->last_scene_prior->utime == self->last_image->utime)) {
	
	double zmin = self->last_scene_prior->Zmean - 2*self->last_scene_prior->Zstd;
	double zmax = self->last_scene_prior->Zmean + 2*self->last_scene_prior->Zstd;
	
	glPointSize (2.0);
	glBegin (GL_POINTS);
	for (int n=0; n < self->last_scene_prior->npts; n++) {	
	    
	    double z_norm = (self->last_scene_prior->Z[n] - zmin) / (zmax - zmin);
	    //double z_norm = (self->last_scene_prior->X[n] + 2) / (2);  
	
	    glColor3fv (bot_color_util_jet (z_norm));
	    glVertex3f (self->last_scene_prior->u[n],
			self->last_scene_prior->v[n], 0);
	}
	glEnd ();	
    }
    
    // plot putative
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_PUTATIVE_DISPLAY) &&
	self->last_pccs_putative &&
       (self->last_pccs_putative->utime == self->last_image->utime)) {
	
	glPointSize (10.0);
	for (int n=0; n < self->last_pccs_putative->num_corrs; n++) {    
	    glColor3fv (self->rcolors[n%NUM_RCOLORS]);	    
	    // plot the image point
	    glBegin (GL_POINTS);
		glVertex3f (self->last_pccs_putative->u[n], self->last_pccs_putative->v[n], 0);
		glVertex3f (self->last_pccs_putative->up[n], self->last_pccs_putative->vp[n], 0);
	    glEnd ();
	    glBegin (GL_LINES);
		glVertex3f (self->last_pccs_putative->u[n], self->last_pccs_putative->v[n], 0);
		glVertex3f (self->last_pccs_putative->up[n], self->last_pccs_putative->vp[n], 0);
	    glEnd ();
	}
	
	for (int n=0; n < self->last_pccs_putative->num_corrs; n++) {
	    
	    // plot the projected point and its uncertianty
	    glMatrixMode (GL_MODELVIEW);
	    glPushMatrix ();
	    
	    glColor3fv (self->rcolors[n%NUM_RCOLORS]);
	    
	    glTranslatef (self->last_pccs_putative->up[n],
			  self->last_pccs_putative->vp[n], 0);
	    
	    if (n < self->last_pccs_putative->num_covp) {
		double eigv[4];
		double eig[2];
		double theta;
		double uv_cov[4] = {self->last_pccs_putative->covp[n][0],
				    self->last_pccs_putative->covp[n][1],
				    self->last_pccs_putative->covp[n][2],
				    self->last_pccs_putative->covp[n][3]};
		    
		matrix_eigen_symm_2x2 (uv_cov, eigv, eig, &theta);
		bot_gl_draw_ellipse (3*sqrt(eig[0]), 3*sqrt(eig[1]), theta, 20);
	    }
	
	    glMatrixMode (GL_MODELVIEW);
	    glPopMatrix ();
	}
    }
    
    
    // plot inliers
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_INLIER_DISPLAY) &&
	self->last_pccs_inlier &&
       (self->last_pccs_inlier->utime == self->last_image->utime)) {
	
	glPointSize (10.0);
	for (int n=0; n < self->last_pccs_inlier->num_corrs; n++) {
	    
	    glColor3fv (self->rcolors[n%NUM_RCOLORS]);
	    
	    glBegin (GL_POINTS);
		glVertex3f (self->last_pccs_inlier->u[n], self->last_pccs_inlier->v[n], 0);
		glVertex3f (self->last_pccs_inlier->up[n], self->last_pccs_inlier->vp[n], 0);
	    glEnd ();
	    glBegin (GL_LINES);
		glVertex3f (self->last_pccs_inlier->u[n], self->last_pccs_inlier->v[n], 0);
		glVertex3f (self->last_pccs_inlier->up[n], self->last_pccs_inlier->vp[n], 0);
	    glEnd ();
	}
    }
  
    //clean up
    glPopAttrib ();
    
    glMatrixMode (GL_PROJECTION);
    glPopMatrix ();
    
    glMatrixMode (GL_MODELVIEW);
    glPopMatrix ();
    
    glDeleteTextures (1, &texName);
}


static void
image_free (BotRenderer *super)
{
    RendererImage *self = (RendererImage*) super->user;
    
    if (self->last_image)
        bot_core_image_t_destroy (self->last_image);
    if (self->last_feature_collection)
        perllcm_van_feature_collection_t_destroy (self->last_feature_collection);
    if (self->last_scene_prior)
        perllcm_van_scene_prior_t_destroy (self->last_scene_prior);
    if (self->last_pccs_putative)
	perllcm_viewer_image_pccs_t_destroy (self->last_pccs_putative);
    if (self->last_pccs_inlier)
	perllcm_viewer_image_pccs_t_destroy (self->last_pccs_inlier);
	
    free (self);
}

static void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *param, void *user)
{
    RendererImage *self = (RendererImage*) user;
    //printf("ENABLED: %d\n",bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_DISPLAY));
    self->scale = bot_gtk_param_widget_get_double (self->pw, PARAM_DISPLAY_SCALE);
    
    bot_viewer_request_redraw (self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user)
{
    RendererImage *self = (RendererImage*) user;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user)
{
    RendererImage *self = (RendererImage*) user;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

RendererImage *
renderer_image_new (BotViewer *viewer, char *rootkey)
{
    //setup structure variables
    RendererImage *self = (RendererImage*) calloc (1, sizeof (*self));
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};
    
    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);

    snprintf (key, sizeof key, "viewer.renderers.%s.img_channel", rootkey);
    char *img_channel = bot_param_get_str_or_fail (self->param, key);

    snprintf (key, sizeof key, "viewer.renderers.%s.fc_channel", rootkey);
    char *fc_channel = NULL;
    bot_param_get_str (self->param, key, &fc_channel);
    
    snprintf (key, sizeof key, "viewer.renderers.%s.sp_channel", rootkey);
    char *sp_channel = NULL;
    bot_param_get_str (self->param, key, &sp_channel);

    snprintf (key, sizeof key, "viewer.renderers.%s.pccs_channel", rootkey);
    char *pccs_channel = NULL;
    bot_param_get_str (self->param, key, &pccs_channel);
    
    
    snprintf (key, sizeof key, "viewer.renderers.%s.r90cw", rootkey);
    self->r90cw = 0;
    bot_param_get_int (self->param, key, &self->r90cw);
    snprintf (key, sizeof key, "viewer.renderers.%s.r90ccw", rootkey);
    self->r90ccw = 0;
    bot_param_get_int (self->param, key, &self->r90ccw);

    self->viewer = viewer;
    
    self->renderer.draw = &image_draw;
    self->renderer.destroy = &image_free;
    self->renderer.name = renderer_label;
    self->renderer.user = self;
    self->renderer.enabled = 1;

    //subscribe to image LCM
    self->lcm = bot_lcm_get_global (NULL);
    bot_core_image_t_subscribe (self->lcm, img_channel, on_bot_core_image_t, self);
    if (fc_channel) {
        perllcm_van_feature_collection_t_subscribe (self->lcm, fc_channel, on_feature_collection_t, self);
	free (fc_channel);
    }
	
    if (sp_channel) {
	perllcm_van_scene_prior_t_subscribe (self->lcm, sp_channel, on_perllcm_van_scene_prior_t, self);
	free (sp_channel);
    }
    
    if (pccs_channel) {
	perllcm_viewer_image_pccs_t_subscribe (self->lcm, pccs_channel, on_perllcm_viewer_image_pccs_t, self);
	free (pccs_channel);
    }
    
    
    //layout sidebar widgets
    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw),FALSE, TRUE, 0);
    gtk_widget_show (GTK_WIDGET (self->pw));

    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE_DISPLAY, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE_FEAT_DISPLAY, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE_SCENE_PRIOR_DISPLAY, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE_PUTATIVE_DISPLAY, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE_INLIER_DISPLAY, 1, NULL);
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_DISPLAY_SCALE, BOT_GTK_PARAM_WIDGET_SLIDER, 0.1, 1.5, 0.1, 0.8);

    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);

    self->last_image = NULL;
    self->last_feature_collection = NULL;
    self->last_scene_prior = NULL;
    self->last_pccs_putative = NULL;
    self->last_pccs_inlier = NULL;


    float rcolor[4] = {0};
    for (int i=0; i<NUM_RCOLORS; i++) {

        bot_color_util_rand_color (rcolor, 1.0, 0.1);	
	memcpy (self->rcolors[i], rcolor, 3*sizeof (float));
    
    }

    return self;
}

void
setup_renderer_image (BotViewer *viewer, char *rootkey, int render_priority)
{
    RendererImage *self = renderer_image_new (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &self->renderer, render_priority);
}
