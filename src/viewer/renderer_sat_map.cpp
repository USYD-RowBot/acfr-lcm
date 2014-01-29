#include <stdio.h>
#include <stdlib.h>

// external linking req'd
#include <curl/curl.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/timeutil.h"
#include "perls-common/units.h"

#include "renderers.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RENDERER_NAME "Sat-Map"

#define PARAM_ENABLE "Enable"
#define PARAM_MAP_TYPE_MENU "Map Type"
#define PARAM_ALPHA "Alpha"

#define IMAGE_WIDTH_HEIGHT 640
#define BINGMAPS_KEY "AkDJvkaRuayNYBgAvBbi_-Egd_2g2DtsLKc4dyNji_VNTGlB56G5MVc-YS8trDEC"

enum {
    TYPE_ROADMAP,
    TYPE_SATELLITE,
    TYPE_HYBRID,
};


typedef struct _RendererSatMap RendererSatMap;
struct _RendererSatMap {
    BotRenderer renderer;
    
    lcm_t *lcm;
    BotParam *param;
    
    CURL *curl;
    CURLcode res;
    
    BotViewer         *viewer;
    BotGtkParamWidget *pw;
    
    GLuint  map_dl; 
    int map_dl_ready;
    
    int zoom_level;
    int lat_tiles[2];
    int lon_tiles[2];
    int use_google;
    
    BotGPSLinearize *llxy;
    double orglat;
    double orglon;
    
    char *cache_dir;
    
    double z_offset;
    int alpha;
    
};

static void
_renderer_free (BotRenderer *super)
{
    RendererSatMap *self = (RendererSatMap*) super->user;
    curl_easy_cleanup (self->curl);
    free (self);
}


static size_t
push_data (char *ptr, size_t size, size_t nmemb, void *udata) 
{  
    fwrite (ptr, size, nmemb, (FILE *) udata);
    return size*nmemb;
}

// 
// see http://en.wikipedia.org/wiki/Mercator_projection
double
px2latrad (double lat, int zoom_lvl, int pixel)
{    
    //pixel /= 2; //divided by 2 because scale=2
    
    // lat = atan (sinh (pixel/R))
    int cp = 256*pow (2, zoom_lvl)-1; // circumfrence of the earth in pixels
    double R = (double) cp / (2.0*M_PI);         // radius of the earth in pixels
    // dlat/dpixel = R/cosh (lat/R)
    //return (double) pixel * R / cosh (lat / R); // i calculated this
    return (double) pixel * cos (lat)/R; // but this seems to be right, maybe sin, at our latitude they are both the same :)
}

double
px2lonrad (int zoom_lvl, int pixel)
{
    //pixel /= 2; //divided by 2 because scale=2
    
    // lon = pixel/R
    double cp = (double)(256*pow (2, zoom_lvl)-1); // circumfrence of the earth in pixels
    double R =  cp / (2.0*M_PI);   // radius of the earth in pixels
    // dlon/dpixel = 1/R
    return (double) pixel/R;
}

//void
//ll2xy_mercator (double orglat, double orglon, double lat, double lon, double *x_out, double *y_out) {
//    
//    double er = 6356750.0;//6378100; // earth's radius in meters;
//    double y_o = (orglon - M_PI)*er;
//    double x_o = log ((1 + sin (orglat)/ cos (orglat)))*er;
//    double y = (lon - M_PI)*er;
//    double x = log ((1 + sin (lat)/ cos (lat)))*er;
//    
//printf ("x_o=%lf, y_o=%lf, x=%lf, y=%lf \n", x_o, y_o, x, y);
//
//    *x_out = x - x_o;
//    *y_out = y - y_o;
//    
//}


void
plot_tile (RendererSatMap *self, double clat, double clon)
{    
    // file name of this tile
    char file[1024] = {0};
    int map_type = bot_gtk_param_widget_get_enum (self->pw, PARAM_MAP_TYPE_MENU);
    self->alpha = bot_gtk_param_widget_get_double (self->pw, PARAM_ALPHA);
    sprintf (file, "%s/%d_%d_%d_%lf_%lf.jpg", self->cache_dir, self->use_google, map_type, self->zoom_level, clat, clon);
    
    // try to load file from cache
    IplImage* img = NULL;
    img = cvLoadImage (file, CV_LOAD_IMAGE_COLOR);
    
    if (!img) { // no image in cache download new
        //write url
        char url[1024] = {0};
        
        if (self->use_google) {
            sprintf (url, "http://maps.googleapis.com/maps/api/staticmap?format=jpg&center=");
            sprintf (url, "%s%lf,%lf", url, clat*RTOD, clon*RTOD);
            sprintf (url, "%s&zoom=%d&size=%dx%d&sensor=false&scale=1", url, self->zoom_level, IMAGE_WIDTH_HEIGHT, IMAGE_WIDTH_HEIGHT);
            switch (map_type) {
                case TYPE_ROADMAP:
                    sprintf (url, "%s&maptype=%s", url, "roadmap");    
                    break;
                case TYPE_SATELLITE:
                    sprintf (url, "%s&maptype=%s", url, "satellite");    
                    break;
                case TYPE_HYBRID:
                    sprintf (url, "%s&maptype=%s", url, "hybrid");    
                    break;                
            }
        }
        else {
            sprintf (url, "http://dev.virtualearth.net/REST/v1/Imagery/Map/");
            switch (map_type) {
                case TYPE_ROADMAP:
                    sprintf (url, "%s%s", url, "Road/");    
                    break;
                case TYPE_SATELLITE:
                    sprintf (url, "%s%s", url, "Aerial/");    
                    break;
                case TYPE_HYBRID:
                    sprintf (url, "%s%s", url, "AerialWithLabels/");    
                    break;
            }
            sprintf (url, "%s%lf,%lf/", url, clat*RTOD, clon*RTOD);
            sprintf (url, "%s%d?mapSize=%d,%d&format=jpeg", url, self->zoom_level, IMAGE_WIDTH_HEIGHT, IMAGE_WIDTH_HEIGHT);
            sprintf (url, "%s&key=%s", url, BINGMAPS_KEY);
        }
    
        // set url to querry
        curl_easy_setopt (self->curl, CURLOPT_URL, url);
        // set data callback function
        curl_easy_setopt (self->curl, CURLOPT_WRITEFUNCTION, push_data);
        
        FILE *fout = fopen (file, "wb");
        if (!fout)
            ERROR ("Could not open %s for writing", file);

        // set data callback userdata
        curl_easy_setopt (self->curl, CURLOPT_WRITEDATA, fout); 
        self->res = curl_easy_perform (self->curl);
        fclose (fout);
        if (self->res)
            ERROR ("HTTP request failed, %s", curl_easy_strerror (self->res));
    
        img = cvLoadImage (file, CV_LOAD_IMAGE_COLOR);
    
        if (!img)
            return;
    }
    
    //cvNamedWindow ("TMP", CV_WINDOW_AUTOSIZE);
    //cvShowImage ("TMP", img);
    //cvWaitKey(0);
    
    int img_width = img->width;
    int img_height = img->height;

    // Setup texture parameters
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1); //set storage mode for pixel operations (byte alignment)
    GLuint texName;
    glGenTextures (1, &texName); //generate one texture name
    glBindTexture (GL_TEXTURE_2D, texName); //assoc a new 2d texture with this name
    //assign texture parameters
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); //repeat in s direction
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); //repeat in t direction
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); //zoom out sampling by nearst pixel
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); //zoom in sampling by nearst pixel
    
    // set texture alpha value
    int new_size = img_width*img_height*4;
    uint8_t *tmp_buf = (uint8_t *) calloc (1, new_size*sizeof (*tmp_buf));
    for (int i=0; i<img_height; i++) {
        for (int j=0; j<img_width; j++) {
            tmp_buf[i*(img_width*4) + j*4 + 0] = img->imageData[i*(img_width*3) + j*3 + 2];
            tmp_buf[i*(img_width*4) + j*4 + 1] = img->imageData[i*(img_width*3) + j*3 + 1];
            tmp_buf[i*(img_width*4) + j*4 + 2] = img->imageData[i*(img_width*3) + j*3 + 0];
            tmp_buf[i*(img_width*4) + j*4 + 3] = self->alpha;
        }
    }
    
    //map image data on to graphical primative
    glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA, img_width, img_height,
                  0, GL_RGBA, GL_UNSIGNED_BYTE, tmp_buf);
    free (tmp_buf);
    cvReleaseImage (&img);

    glEnable (GL_TEXTURE_2D); //enable 2D texture mapping
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //define how texture data is interprated (replace existing)
    glBindTexture (GL_TEXTURE_2D, texName);
    
    double ll_deg_4[2] = {(clat + px2latrad (clat, self->zoom_level, -img_width/2.0)) *UNITS_RADIAN_TO_DEGREE,
                          (clon + px2lonrad (self->zoom_level, -img_height/2.0))*UNITS_RADIAN_TO_DEGREE};
    double ll_deg_3[2] = {(clat + px2latrad (clat, self->zoom_level, -img_width/2.0)) *UNITS_RADIAN_TO_DEGREE,
                          (clon + px2lonrad (self->zoom_level,  img_height/2.0))*UNITS_RADIAN_TO_DEGREE};
    double ll_deg_2[2] = {(clat + px2latrad (clat, self->zoom_level,  img_width/2.0)) *UNITS_RADIAN_TO_DEGREE,
                          (clon + px2lonrad (self->zoom_level,  img_height/2.0))*UNITS_RADIAN_TO_DEGREE};
    double ll_deg_1[2] = {(clat + px2latrad (clat, self->zoom_level,  img_width/2.0)) *UNITS_RADIAN_TO_DEGREE,
                          (clon + px2lonrad (self->zoom_level, -img_height/2.0))*UNITS_RADIAN_TO_DEGREE};
    double yx_1[2], yx_2[2], yx_3[2], yx_4[2];
    bot_gps_linearize_to_xy (self->llxy, ll_deg_1, yx_1);
    bot_gps_linearize_to_xy (self->llxy, ll_deg_2, yx_2);
    bot_gps_linearize_to_xy (self->llxy, ll_deg_3, yx_3);
    bot_gps_linearize_to_xy (self->llxy, ll_deg_4, yx_4);
    //ll2xy_mercator (self->orglat, self->orglon, ll_deg_1[0]*DTOR, ll_deg_1[1]*DTOR, &yx_1[1], &yx_1[0]);
    //ll2xy_mercator (self->orglat, self->orglon, ll_deg_2[0]*DTOR, ll_deg_2[1]*DTOR, &yx_2[1], &yx_2[0]);
    //ll2xy_mercator (self->orglat, self->orglon, ll_deg_3[0]*DTOR, ll_deg_3[1]*DTOR, &yx_3[1], &yx_3[0]);
    //ll2xy_mercator (self->orglat, self->orglon, ll_deg_4[0]*DTOR, ll_deg_4[1]*DTOR, &yx_4[1], &yx_4[0]);
    
    glColor3f (1.0, 1.0, 1.0); 
    glBegin (GL_QUADS);
      glTexCoord2f (0.0, 0.0); glVertex3f(yx_1[1], yx_1[0], self->z_offset);   // lower left
      glTexCoord2f (1.0, 0.0); glVertex3f(yx_2[1], yx_2[0], self->z_offset);   // lower right  
      glTexCoord2f (1.0, 1.0); glVertex3f(yx_3[1], yx_3[0], self->z_offset);   // upper right
      glTexCoord2f (0.0, 1.0); glVertex3f(yx_4[1], yx_4[0], self->z_offset);   // upper left
    glEnd ();
    
    
    glDisable (GL_TEXTURE_2D);
}


GLuint
build_map_dl (RendererSatMap *self)
{    
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);
    glEnable (GL_BLEND);
    glDisable (GL_DEPTH_TEST);
    
    double dlat = px2latrad (self->orglat, self->zoom_level, IMAGE_WIDTH_HEIGHT);
    double dlon = px2lonrad (self->zoom_level, IMAGE_WIDTH_HEIGHT);
    
    
    for (int i=self->lat_tiles[0]; i<=self->lat_tiles[1]; i++) {
        for (int j=self->lon_tiles[0]; j<=self->lon_tiles[1]; j++) {
            printf ("\tLoading Tile \t[%d, %d] \n", i, j);
            plot_tile (self, self->orglat+dlat*i, self->orglon+dlon*j);
            
            if (self->use_google)
                timeutil_usleep (2e6);
        }
    }
    
    //plot_tile (self, self->orglat, self->orglon);
    glEndList ();
    
    self->map_dl_ready = 1;
    
    return dl;
}

static void 
sat_map_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
    RendererSatMap *self = (RendererSatMap*) super->user;

    if (!bot_gtk_param_widget_get_bool (self->pw, PARAM_ENABLE))
        return;
    
    if (!self->map_dl_ready ) {
        if (0 != self->map_dl) {
            glDeleteLists(self->map_dl, 1);
        }
        self->map_dl = build_map_dl (self);
        self->map_dl_ready = 1;
    }
        
    glPushMatrix ();
    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // rotate 180 around x-axis
    glRotatef (180.0, 1.0, 0.0, 0.0);
    glCallList (self->map_dl);
    glPopMatrix ();
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererSatMap *self = (RendererSatMap*) user;
    self->map_dl_ready = 0;
    bot_viewer_request_redraw (self->viewer);
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererSatMap *self = (RendererSatMap*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererSatMap *self = (RendererSatMap*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererSatMap *
new_renderer_sat_map (BotViewer *viewer, char *rootkey)
{    
    RendererSatMap *self = (RendererSatMap*) calloc (1, sizeof (*self));
    
    self->lcm = bot_lcm_get_global (NULL);

    // load from config file
    self->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    char key[256] = {'\0'};

    snprintf (key, sizeof key, "viewer.renderers.%s.label", rootkey);
    char *renderer_label = botu_param_get_str_or_default (self->param, key, RENDERER_NAME);
    
    snprintf (key, sizeof key, "viewer.renderers.%s.cache_dir", rootkey);
    self->cache_dir = botu_param_get_str_or_default (self->param, key, "/tmp");
    
    double ll_deg[2];
    bot_param_get_double_array_or_fail (self->param, "site.orglatlon", ll_deg, 2);
    self->llxy = (BotGPSLinearize*) calloc (1, sizeof (BotGPSLinearize));
    bot_gps_linearize_init (self->llxy, ll_deg);
    self->orglat = ll_deg[0] * DTOR;
    self->orglon = ll_deg[1] * DTOR;
    
    self->zoom_level = 19;
    snprintf (key, sizeof key, "viewer.renderers.%s.zoom_level", rootkey);
    bot_param_get_int (self->param, key, &self->zoom_level);
    snprintf (key, sizeof key, "viewer.renderers.%s.lat_tiles", rootkey);
    bot_param_get_int_array_or_fail (self->param, key, self->lat_tiles, 2);
    snprintf (key, sizeof key, "viewer.renderers.%s.lon_tiles", rootkey);
    bot_param_get_int_array_or_fail (self->param, key, self->lon_tiles, 2);
    self->use_google = 0;
    snprintf (key, sizeof key, "viewer.renderers.%s.use_google", rootkey);
    bot_param_get_int (self->param, key, &self->use_google);

    BotRenderer *renderer = &self->renderer;
    renderer->draw = sat_map_renderer_draw;
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
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, PARAM_ENABLE, 1, NULL);

    bot_gtk_param_widget_add_enum(self->pw, PARAM_MAP_TYPE_MENU, BOT_GTK_PARAM_WIDGET_MENU, TYPE_SATELLITE,
                                  "Roadmap", TYPE_ROADMAP,
                                  "Satellite", TYPE_SATELLITE,
                                  "Hybrid", TYPE_HYBRID,
                                  NULL);
    
    bot_gtk_param_widget_add_int (self->pw, PARAM_ALPHA, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER,
                                  15, 255, 15, 255);
    // END layout sidebar widgets ----------------------------------------------

    gtk_widget_show_all (renderer->widget);

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);
    
    
    // curl
    curl_global_init (CURL_GLOBAL_ALL);
    
    self->curl = curl_easy_init ();
    if (!self->curl)
        ERROR ("Could not init curl.");
    
    self->map_dl_ready = 0;
    self->z_offset = 0.0;
    self->alpha = 255;
    
    return self;
}

void
setup_renderer_sat_map (BotViewer *viewer, char *rootkey, int priority)
{
    RendererSatMap *self = new_renderer_sat_map (viewer, rootkey);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
