#include <stdio.h>
#include <stdlib.h>

#include "perls-common/bot_util.h"
#include "perls-common/units.h"
#include "perls-common/error.h"

#include "renderers.h"

#ifdef __PERLS_IVER__
#include "perls-iver/vectormap.h"

#define DEBUG 0

#define RENDERER_NAME "Vector-Map-Mission"
#define PARAM_ENABLE_DISPLAY "Enable Display"

// scale defines all relative to self->meters_per_grid
#define SCALE_WAYPOINT_RADIUS 0.1

// color defines
#define COLOR_WAYPOINT_LINE 	1.0, 0.0, 0.0
#define COLOR_WAYPOINT_DEFAULT 	0.0, 0.0, 1.0
#define COLOR_WAYPOINT_SELECTED 1.0, 1.0, 0.0
#define COLOR_WAYPOINT_NEXT 	0.0, 1.0, 1.0
#define COLOR_WAYPOINT_START    0.0, 1.0, 0.0
#define COLOR_WAYPOINT_END      1.0, 0.0, 0.0
#define COLOR_BUOY_DEFAULT 	1.0, 0.0, 1.0
#define COLOR_BUOY_SELECTED     0.0, 1.0, 1.0

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

typedef struct _RendererVectorMapMission RendererVectorMapMission;
struct _RendererVectorMapMission {
    
    //variables for extending viewer "class"
    BotRenderer renderer;
    BotGtkParamWidget *pw;
    BotViewer *viewer;
    
    //lcm holders
    lcm_t *lcm;
    
    // bot config
    BotParam *param;
    
    //struct containing gps origin used to convert between gps and local xy
    BotGPSLinearize *llxy;  
    
    // waypoint list parsed from mission file
    int n_waypoints;
    GList *waypoints;
    int n_buoys;
    GList *buoys;
    
    BotEventHandler ehandler;
    
    // to get a sense of scale
    double meters_per_grid;

    // variables for sidebar widget controls
    int enabled;
    
    GtkWidget *waypoint_combobox;
    GtkWidget *waypoint_info;
    int selected_waypoint;
    GtkWidget *buoy_combobox;
    GtkWidget *buoy_info;
    int selected_buoy;
};

static void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *param, void *user)
{
    RendererVectorMapMission *self = (RendererVectorMapMission *) user;
    self->enabled = bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE_DISPLAY);
    bot_viewer_request_redraw (self->viewer);
}

static void 
on_load_mission (GtkWidget *button, void *user)
{
    RendererVectorMapMission *self = (RendererVectorMapMission *) user;

    // choose the file
    GtkWidget *dialog;
    dialog = gtk_file_chooser_dialog_new ("Select Vector Map mission file",
                                          NULL,
                                          GTK_FILE_CHOOSER_ACTION_OPEN,
                                          GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
                                          GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT,
                                          NULL);

    char *file = NULL;
    if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT)
        file  = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));

    gtk_widget_destroy (dialog);

    // clear out previsouly loaded mission if there
    if (NULL != self->waypoints) {
	iver_vectormap_free_waypoints (self->waypoints);
	self->waypoints = NULL;
        // clear combobox
	for (int i=0; i<self->n_waypoints; i++ ) {
	    gtk_combo_box_remove_text (GTK_COMBO_BOX (self->waypoint_combobox), 0);
	}
        self->n_waypoints = 0;
    }    
    if (NULL != self->buoys) {
	iver_vectormap_free_buoys (self->buoys);
	self->buoys = NULL;
        // clear combobox
	for (int i=0; i<self->n_buoys ; i++ ) {
	    gtk_combo_box_remove_text (GTK_COMBO_BOX (self->buoy_combobox), 0);
        }
        self->n_buoys = 0;
    }   

    // load the waypoints
    self->waypoints = iver_vectormap_parse_mission (file);
    self->n_waypoints = g_list_length (self->waypoints);
    self->buoys = iver_vectormap_parse_buoy (file);
    self->n_buoys = g_list_length (self->buoys);
    

    // fill in the waypoint combobox
    char tmp[32] = {0};
    for (int i=1; i<=self->n_waypoints ; i++ ) {
	snprintf (tmp, sizeof tmp, "Waypoint %d", i);
	gtk_combo_box_append_text (GTK_COMBO_BOX (self->waypoint_combobox), tmp);
    }
    for (int i=1; i<=self->n_buoys ; i++ ) {
	snprintf (tmp, sizeof tmp, "Buoy %d", i);
	gtk_combo_box_append_text (GTK_COMBO_BOX (self->buoy_combobox), tmp);
    }

    bot_gtk_param_widget_set_bool (self->pw, PARAM_ENABLE_DISPLAY, 1);
    on_param_widget_changed (self->pw, PARAM_ENABLE_DISPLAY, self);
    bot_viewer_request_redraw (self->viewer);

    g_free (file);
}

static void
draw_mission (BotViewer *viewer, RendererVectorMapMission *self)
{     
    // plot waypoints
    if (NULL == self->waypoints)
	return;

    GList *wp_list = g_list_first (self->waypoints);
    iver_vectormap_waypoint_t *wp_prev = NULL;
    iver_vectormap_waypoint_t *wp = (iver_vectormap_waypoint_t *) wp_list->data;
    double ll_deg[2];
    double yx[2], yx_prev[2];
    
    do {
	wp = (iver_vectormap_waypoint_t *) wp_list->data;
	
	// find location
	ll_deg[0] = wp->latitude * RTOD;
        ll_deg[1] = wp->longitude * RTOD;
	// NOTE returns yx -> returns in ENU not NED
        bot_gps_linearize_to_xy (self->llxy, ll_deg, yx);

	//draw a box
	glPushMatrix ();
	glTranslatef (yx[1],yx[0],0.0); // move to location
	
	if (self->selected_waypoint != wp->num) {
	    if (1 == wp->num)
		glColor3f (COLOR_WAYPOINT_START);
	    else if (self->n_waypoints == wp->num)
		glColor3f (COLOR_WAYPOINT_END);
	    else
		glColor3f (COLOR_WAYPOINT_DEFAULT);
	}
        else
	    glColor3f (COLOR_WAYPOINT_SELECTED);
    
	glutSolidSphere (self->meters_per_grid*SCALE_WAYPOINT_RADIUS, 10, 10);
	glPopMatrix ();
	
	//draw a line to previous
	if (NULL != wp_prev) {
	    glColor3f (COLOR_WAYPOINT_LINE);
	    glBegin (GL_LINES);
	    glVertex3f (yx_prev[1], yx_prev[0], 0.0);  glVertex3f(yx[1], yx[0], 0.0); 
	    glEnd ();
	}

	yx_prev[0] = yx[0];
	yx_prev[1] = yx[1];
	wp_prev = wp;
    } while ((wp_list = g_list_next (wp_list)));

    // plot buoys    
    if (NULL == self->buoys)
	return;

    GList *b_list = g_list_first (self->buoys);
    iver_vectormap_buoy_t *b = (iver_vectormap_buoy_t *) b_list->data;    
    do {
	b = (iver_vectormap_buoy_t *) b_list->data;
	
	// find location
	ll_deg[0] = b->latitude * RTOD;
        ll_deg[1] = b->longitude * RTOD;
	// NOTE returns yx -> returns in ENU not NED
        bot_gps_linearize_to_xy (self->llxy, ll_deg, yx);

	//draw a box
	glPushMatrix ();
	glTranslatef (yx[1],yx[0],0.0); // move to location
	
	if (self->selected_buoy != b->num) 
		glColor3f (COLOR_BUOY_DEFAULT);
        else
	    glColor3f (COLOR_BUOY_SELECTED);
    
	glutSolidSphere (self->meters_per_grid*SCALE_WAYPOINT_RADIUS, 10, 10);
	glPopMatrix ();

    } while ((b_list = g_list_next (b_list)));
}

static void
vectormap_mission_draw (BotViewer *viewer, BotRenderer *renderer)
{
    
        
    RendererVectorMapMission *self = (RendererVectorMapMission *) renderer->user;
    g_assert (self->viewer == viewer);  
    
    // wait till we have something to draw
    if (!self->waypoints || !self->enabled)
        return;
    
    // get a sense of scale so that as we zoom we can keep the mission visable
    // set relative to grid when automatic spacing is on
    double eye[3], look[3], up[3];
    viewer->view_handler->get_eye_look (viewer->view_handler, eye, look, up);
    // when looking directly down at the world, about how many grids should appear across the screen?
    double grids_per_screen = 10;
    double eye_dist = bot_vector_dist_3d (eye, look);
    self->meters_per_grid = eye_dist/grids_per_screen;    
 
    // setup 3D drawing
    glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT);
    glEnable (GL_DEPTH_TEST);
    
    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0, 1.0, 0.0, 0.0);

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

    // draw 3D stuff
    draw_mission (viewer, self);
    
    // done
    glPopAttrib ();
 
#if DEBUG
    g_list_foreach (self->waypoints, (GFunc) &iver_vectormap_print_waypoint, NULL);
#endif
}

//// can use to take priority in getting the on_mouse_press events
//// return dist is distance to nearest element that you might pick
//// closest of all calbacks in all renders wins the right to get the event first
//// we dont care when we get the event because we are just gonna change the color
//// and display some info in the sidebar, so just use on mouse press event
//// you know you've won if ehandler->picking == 1 in on_mouse_press callback
//double
//pick_query (BotViewer *viewer, BotEventHandler *ehandler, 
//            const double ray_start[3], const double ray_dir[3]) {
//    
//    RendererVectorMapMission *self = (RendererVectorMapMission *) ehandler->user;
//    
//    // distance to nearest waypoint
//    double dist = 1e6;
//    
//    printf ("pick_query ray_start {%f %f %f} ray_dir {%f %f %f} \n",
//	    ray_start[0], ray_start[1], ray_start[2],
//	    ray_dir[0], ray_dir[1], ray_dir[2]);
//    
//    // return distance to waypoint bubble
//    return dist;
//    // return indicating nothing is close
//    return -1;
//}

static int 
on_mouse_press (BotViewer *viewer, BotEventHandler *ehandler,
                const double ray_start[3], const double ray_dir[3], 
                const GdkEventButton *event)
{    
    RendererVectorMapMission *self = (RendererVectorMapMission *) ehandler->user;

    switch (event->type) {
    case GDK_BUTTON_PRESS: {
        
        if (NULL == self->waypoints)
            return 0;
	    
        double xyz_inter[3];
	
        // all waypoints lie on z=0 plane (water surface)
        // => finds rays intersection with the z=0 plane
        double plane_pt[3] = { 0, 0, 0};		// origin
        double plane_normal[3] = { 0, 0, 1};	// z axis
        double lambda1 = ray_dir[0] * plane_normal[0] + 
                         ray_dir[1] * plane_normal[1] + 
                         ray_dir[2] * plane_normal[2];

        // check for where ray is parallel to plane
        if (fabs (lambda1) < 1e-9)
            return 0;

        double lambda2 = (plane_pt[0] - ray_start[0]) * plane_normal[0] +
                         (plane_pt[1] - ray_start[1]) * plane_normal[1] +
                         (plane_pt[2] - ray_start[2]) * plane_normal[2];
        double v = lambda2 / lambda1;
        xyz_inter[0] = ray_start[0] + v * ray_dir[0];
        xyz_inter[1] = ray_start[1] + v * ray_dir[1];
        xyz_inter[2] = ray_start[2] + v * ray_dir[2];
        // intersection point
        // printf("INTERSECTION POINT XYZ: %f %f %f\n", xyz_inter[0], xyz_inter[1], xyz_inter[2]);
	
        // FingCTnS remember NWU vs NED (rotate 180 about x)
        xyz_inter[1] = -xyz_inter[1];
        xyz_inter[2] = -xyz_inter[2];
	
        // loop through the waypoints to see if we've clicked on one
        double yx[2], ll_deg[2];
        GList *wp_list = g_list_first (self->waypoints);
        iver_vectormap_waypoint_t *wp = NULL;
        do {
            wp = (iver_vectormap_waypoint_t *) wp_list->data;
            ll_deg[0] = wp->latitude * RTOD;
            ll_deg[1] = wp->longitude * RTOD;
            // NOTE returns yx -> returns in ENU not NED
            bot_gps_linearize_to_xy(self->llxy, ll_deg, yx);
            double wp_radius = self->meters_per_grid*SCALE_WAYPOINT_RADIUS;
            double dist = sqrt ( (xyz_inter[0]-yx[1])*(xyz_inter[0]-yx[1])
                               + (xyz_inter[1]-yx[0])*(xyz_inter[1]-yx[0]) );
            if (dist <= wp_radius) {
                gtk_combo_box_set_active (GTK_COMBO_BOX (self->waypoint_combobox), wp->num-1);
            }
		
        } while ((wp_list = g_list_next (wp_list)));
        
        if (NULL != self->buoys) {
        
            // loop through the buoys to see if we've clicked on one
            GList *b_list = g_list_first (self->buoys);
            iver_vectormap_buoy_t *b = NULL;
            do {
                b = (iver_vectormap_buoy_t *) b_list->data;
                ll_deg[0] = b->latitude * RTOD;
                ll_deg[1] = b->longitude * RTOD;
                // NOTE returns yx -> returns in ENU not NED
                bot_gps_linearize_to_xy(self->llxy, ll_deg, yx);
                double b_radius = self->meters_per_grid*SCALE_WAYPOINT_RADIUS;
                double dist = sqrt ( (xyz_inter[0]-yx[1])*(xyz_inter[0]-yx[1])
                                   + (xyz_inter[1]-yx[0])*(xyz_inter[1]-yx[0]) );
                if (dist <= b_radius) {
                    gtk_combo_box_set_active (GTK_COMBO_BOX (self->buoy_combobox), b->num-1);
                }
                    
            } while ((b_list = g_list_next (b_list)));
            
        }
	    
        bot_viewer_request_redraw (self->viewer);

        break;
    }
    case GDK_BUTTON_RELEASE:    
        break;
    default:
        break;
    }

    return 1;
}

// called when waypoint combobox changes
static void
on_waypoint_change (GtkComboBox *cb, void *user)
{    
    RendererVectorMapMission *self = (RendererVectorMapMission *) user;
    
    if (NULL == self->waypoints)
	return;
    
    if (1 == sscanf (gtk_combo_box_get_active_text (cb), "Waypoint %d", &self->selected_waypoint)) {
        
        char wp_info[1024] = {0};
        iver_vectormap_print_waypoint ((iver_vectormap_waypoint_t *) g_list_nth_data (self->waypoints, self->selected_waypoint-1), wp_info);
        gtk_label_set_text (GTK_LABEL (self->waypoint_info), wp_info);
    }
    
    bot_viewer_request_redraw (self->viewer);
}

static void
on_buoy_change (GtkComboBox *cb, void *user)
{    
    RendererVectorMapMission *self = (RendererVectorMapMission *) user;
    
    if (NULL == self->buoys)
	return;
    
    if (1 == sscanf (gtk_combo_box_get_active_text (cb), "Buoy %d", &self->selected_buoy)) {
        
        char buoy_info[1024] = {0};
        iver_vectormap_print_buoy ((iver_vectormap_buoy_t *) g_list_nth_data (self->buoys, self->selected_buoy-1), buoy_info);
        gtk_label_set_text (GTK_LABEL (self->buoy_info), buoy_info);
    }
    
    bot_viewer_request_redraw (self->viewer);
}

static void
vectormap_mission_free (BotRenderer *super) 
{
    RendererVectorMapMission *self = (RendererVectorMapMission *) super->user;
    
    // all clean up goes here
    iver_vectormap_free_waypoints (self->waypoints);
    free(self->llxy);
    free (self);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererVectorMapMission *self = (RendererVectorMapMission *) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererVectorMapMission *self = (RendererVectorMapMission *) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

RendererVectorMapMission *
renderer_vectormap_mission_new (BotViewer *viewer, char *rootkey)
{
    RendererVectorMapMission *self = (RendererVectorMapMission *) calloc (1, sizeof (*self));
    
    self->lcm = bot_lcm_get_global (NULL);
    
    // load config -------------------------------------------------------------
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    double orglatlon[2];
    bot_param_get_double_array_or_fail (self->param, "site.orglatlon", orglatlon, 2);

    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);
    //--------------------------------------------------------------------------

    // lat lon origin
    self->llxy = (BotGPSLinearize *) calloc (1, sizeof (BotGPSLinearize));
    bot_gps_linearize_init (self->llxy, orglatlon);
    
    // setup structure variables for renderer
    self->viewer = viewer;
    self->renderer.draw = vectormap_mission_draw;
    self->renderer.destroy = vectormap_mission_free;
    self->renderer.name = renderer_label;
    self->renderer.user = self;
    self->renderer.enabled = 1;
    self->lcm = bot_lcm_get_global(NULL);
    
    // setup event callbacks
    BotEventHandler *ehandler = &self->ehandler;
    memset (ehandler, 0x00, sizeof (*ehandler));
    ehandler->name = (char *) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->mouse_press = on_mouse_press;
    ehandler->mouse_motion = NULL;
    ehandler->mouse_release = NULL;
    ehandler->mouse_scroll = NULL;
    //ehandler->pick_query = pick_query;
    ehandler->pick_query = NULL;
    ehandler->hover_query = NULL;
    ehandler->key_press = NULL;
    ehandler->user = self;
    
    self->waypoints = NULL;

    // layout sidebar widgets --------------------------------------------------
    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw),FALSE, TRUE, 0);
    gtk_widget_show (GTK_WIDGET (self->pw));

    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE_DISPLAY, 1, NULL);

    // waypoint combobox
    self->waypoint_combobox = gtk_combo_box_new_text ();
    gtk_box_pack_start (GTK_BOX (vbox), self->waypoint_combobox, FALSE, FALSE, 0);
    g_signal_connect (G_OBJECT (self->waypoint_combobox), "changed", G_CALLBACK (on_waypoint_change), self);
    // waypoint info
    self->waypoint_info = gtk_label_new ("Waypoint Info:");
    gtk_box_pack_start (GTK_BOX (vbox),  self->waypoint_info, FALSE, FALSE, 0);
    gtk_label_set_justify (GTK_LABEL (self->waypoint_info), GTK_JUSTIFY_LEFT);
    // buoy combobox
    self->buoy_combobox = gtk_combo_box_new_text ();
    gtk_box_pack_start (GTK_BOX (vbox), self->buoy_combobox, FALSE, FALSE, 0);
    g_signal_connect (G_OBJECT (self->buoy_combobox), "changed", G_CALLBACK (on_buoy_change), self);
    // buoy info
    self->buoy_info = gtk_label_new ("Buoy Info:");
    gtk_box_pack_start (GTK_BOX (vbox),  self->buoy_info, FALSE, FALSE, 0);
    gtk_label_set_justify (GTK_LABEL (self->buoy_info), GTK_JUSTIFY_LEFT);
    
    // load existing map (entry in file menu)
    GtkWidget *load_mission_fmenu = gtk_menu_item_new_with_mnemonic ("Load Vector Map _Mission ");
    gtk_menu_append (GTK_MENU(viewer->file_menu), load_mission_fmenu);
    gtk_widget_show (load_mission_fmenu);
    g_signal_connect (G_OBJECT (load_mission_fmenu), "activate",
                      G_CALLBACK (on_load_mission), self);
    // load existing map (button in sidebar)
    GtkWidget *load_mission_button = gtk_button_new_with_label ("Load Vector Map Mis.");
    gtk_box_pack_start (GTK_BOX (vbox), load_mission_button, FALSE, FALSE, 0);
    g_signal_connect (G_OBJECT (load_mission_button), "clicked", G_CALLBACK (on_load_mission), self);
    
    gtk_widget_show_all (self->renderer.widget);
    //--------------------------------------------------------------------------

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);

    return self;
}
#endif //__PERLS_IVER__

void
setup_renderer_vectormap_mission (BotViewer *viewer, char *rootkey, int render_priority)
{
#ifdef __PERLS_IVER__
    RendererVectorMapMission *self = renderer_vectormap_mission_new (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &(self->renderer), render_priority);
    bot_viewer_add_event_handler (viewer, &(self->ehandler), render_priority);
#else
    ERROR ("perls-iver required for render_vectormap_mission!");
#endif //__PERLS_IVER__
}
