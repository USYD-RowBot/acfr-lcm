#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h> // needed for PRId64 macros
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <glib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-math/gsl_util.h"
#include "perls-math/ssc.h"
#include "perls-vision/botimage.h"
#include "perls-vision/calib.h"
#include "perls-vision/clahs.h"
#include "perls-vision/distortion.h"
#include "perls-vision/feature.h"
#include "perls-vision/opencv_util.h"

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_position_t.h"
#include "perls-lcmtypes/perllcm_rdi_bathy_t.h"
#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_vis_cvu_map_t.h"

#define POOL_THREADS_MAX    10
#define FEATURE_POINTS_MAX  1000

#define BATHY_TIME_STEPS    200
#define BATHY_BEAMS         4
#define BATHY_POINTS_MAX    (BATHY_BEAMS*BATHY_TIME_STEPS)

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define IMGCACHE_MAX 100

typedef struct _camera_params_t camera_params_t;
struct _camera_params_t {
    perllcm_van_calib_t calib;
    vis_cvu_map_t *map;
    IplImage      *mask;
    IplImage      *mask_siftgpu;
};


typedef struct _state_t state_t;
struct _state_t {
    GThreadPool     *pool;

    double  x_lc[6];    // current camera pose (can be used for invariant feature extraction)
    int     ftypes;     // bitmask of feature types to extract

    // position sources
    uint8_t use_position;
    char *position_channel;

    // scene prior sources
    uint8_t          use_dvl_depth_prior;
    char            *dvl_channel;
    gsl_matrix      *bathy_l;    // [4 x BATHY_POINTS_MAX] array in local-level
    size_t           bathy_i;    // current column index of bathy_l array (modulo number of cols)

    // harris feature patch
    gsl_matrix      *featpatch_sampler;

    // process every Nth frame
    int              imgstep;
    size_t           imgcounter;
    
    // lcm channels
    char *image_channel;
    char *feature_channel;
    
    char *cache_dir;
    
    int pub_undist_img;
    
    // img_warp list
    cache_t     *imgcache;
    
    // image/camera settings
    char *config_key;
    char *camera_config_key;
    char *camera_calib_key;
    double x_vs[6];
    camera_params_t camera_params;
    int have_camera_params;
    int use_clahs;

    // siftgpu server information
    char *siftgpu_server;
    
    int done;
    lcm_t *lcm;
    BotParam *param;
};


typedef struct pool_data pool_data_t;
struct pool_data {
    int64_t utime;
    IplImage *img_gray;
    double x_lc[6];
    gsl_matrix *bathy_l;
};

void
pooldata_free (pool_data_t *pdata)
{
    if (pdata->img_gray)
        cvReleaseImage (&pdata->img_gray);
    if (pdata->bathy_l)
        gsl_matrix_free (pdata->bathy_l);
    free (pdata);
}

static void
imgcache_value_destroy (void *value)
{
    IplImage *img = value;
    cvReleaseImage (&img);
}

static void *
imgcache_value_copy (const void *value)
{
    const IplImage *img = value;
    return cvCloneImage (img);
}

static void
perllcm_position_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                             const perllcm_position_t *msg, void *user)
{
    state_t *state = user;
    ssc_head2tail (state->x_lc, NULL, msg->xyzrph, state->x_vs);
}

static void
perllcm_rdi_bathy_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                              const perllcm_rdi_bathy_t *msg, void *user)
{
    state_t *state = user;

    for (size_t b=0; b<4; b++, state->bathy_i++) {
        const double r = msg->range[b];
        const double x = msg->xyz[b][0];
        const double y = msg->xyz[b][1];
        const double z = msg->xyz[b][2];
        if (r > PERLLCM_RDI_BATHY_T_RANGE_SENTINAL) {
            GSLU_VECTOR_VIEW (X, 4, {x, y, z, b+1});

            state->bathy_i %= state->bathy_l->size2;
            gsl_matrix_set_col (state->bathy_l, state->bathy_i, &X.vector);
        }
    }
}

static void
bot_core_image_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                           const bot_core_image_t *msg, void *user)
{
    
    state_t *state = user;

    // process only every nth frame
    if ((state->imgcounter++ % state->imgstep)) {
        printf ("bot_core_image_t event %"PRId64" - skipping feat ext\n", msg->utime);
        return;
    }

    // drop if number of concurrent pool threads is unable to keep up
    guint ntasks = g_thread_pool_unprocessed (state->pool);
    if (ntasks >= g_thread_pool_get_max_threads (state->pool)) {
        printf ("Warning: number of unprocessed tasks (%u) exceeds pool (%u)\n",
                ntasks, g_thread_pool_get_max_threads (state->pool));
        printf ("bot_core_image_t event %"PRId64" - dropping\n", msg->utime);
        return;
    }

    // stuff pool thread data and launch
    printf ("bot_core_image_t event %"PRId64" - processing\n", msg->utime);
    pool_data_t *pdata = malloc (sizeof (*pdata));
    pdata->utime = msg->utime;
    if (state->use_position) 
        memcpy (pdata->x_lc, state->x_lc, 6*sizeof (double));
    else
        memset (pdata->x_lc, 0, 6*sizeof (double));
    if (state->use_dvl_depth_prior) 
        pdata->bathy_l = gslu_matrix_clone (state->bathy_l);
    else
        pdata->bathy_l = NULL;
    
    pdata->img_gray = vis_botimage_to_iplimage_convert (msg, VIS_BOT2CVGRAY);
    g_thread_pool_push (state->pool, pdata, NULL);
}

static void
feature_pool_thread (gpointer pooldata, gpointer user)
{

    pool_data_t *pdata = pooldata;
    state_t *state = user;
    
    // TODO changing calibs on the fly, should listen to lcm channel
    perllcm_van_calib_t *calib = NULL;
    vis_cvu_map_t *map = NULL;
    IplImage *mask = NULL;
    IplImage *mask_siftgpu = NULL;
    vis_calib_view_gsl_t calib_gsl;
    if (state->have_camera_params) {
        calib = &state->camera_params.calib;
        map = state->camera_params.map;
        mask = state->camera_params.mask;
        mask_siftgpu = state->camera_params.mask_siftgpu;
        calib_gsl = vis_calib_view_gsl (calib);
    } 

    // adaptive histogram equalization...
    // -------------------------------------------------------------- //
    if (state->use_clahs) {
        vis_clahs_opts_t opts = {
            .tiles = {8, 10},
            .cliplimit = 0.0075,
            .bins = pdata->img_gray->depth > 8 ? 1024 : 256,
            .range = {0, 0},
            .dist = VIS_CLAHS_DIST_RAYLEIGH,
            .alpha = 0.4,
        };
        if (vis_clahs (pdata->img_gray->imageData, pdata->img_gray->width, 
                       pdata->img_gray->height, pdata->img_gray->depth, &opts) < 0) {
            ERROR ("vis_clahs() failed");
            pooldata_free (pdata);
            return;
        }
    }

    // converting to 8 bit...
    // -------------------------------------------------------------- //
    IplImage *img_8bit;
    if (pdata->img_gray->depth > 8) {
        img_8bit = cvCreateImage (cvGetSize (pdata->img_gray), IPL_DEPTH_8U, 1);
        cvConvertImage (pdata->img_gray, img_8bit, 0);
    } else
        img_8bit = pdata->img_gray;

    // undistorting image...
    // -------------------------------------------------------------- //
    IplImage *img_warp;
    if (map != NULL) {
        img_warp = cvCreateImage (cvGetSize (img_8bit), IPL_DEPTH_8U, 1);
        vis_cvu_warp_image (img_8bit, img_warp, map);
    } else {
        img_warp = cvCloneImage (img_8bit);
    }
    cache_push (state->imgcache, pdata->utime, cvCloneImage (img_warp));
    
    

    // extracting features...
    // -------------------------------------------------------------- //
    perllcm_van_feature_collection_t *fc = vis_feature_collection_alloc (pdata->utime, state->image_channel);

    if (state->ftypes & PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF) {
        int64_t t0 = timestamp_now ();
        CvSURFParams params = cvSURFParams (500, 1);
        perllcm_van_feature_t *f = vis_feature_cvsurf (img_warp, mask, params, FEATURE_POINTS_MAX);
        int64_t dt = timestamp_now () - t0;
        if (f) {
            vis_feature_collection_add (fc, f);
            printf ("cvsurf\t\tnpts=%4d\tdt=%"PRId64"\n", f->npts, dt);
        }
        else
            ERROR ("vis_feature_cvsurf(): returned NULL");
    }

    if (state->ftypes & PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS) {
        vis_feature_harris_params_t harris_params = {
            .qualityLevel = 0.01,   // 0.01 of best feature
            .minDistance = 10.0,     // euclidean distance in pixel
            .blockSize = 3,         // average block
            .k = 0.04,
        };
        int64_t t0 = timestamp_now ();
        perllcm_pose3d_t p3d;
        if (state->use_position) {
            memcpy (p3d.mu, pdata->x_lc, 6*sizeof (double));
        } else {
            memset (p3d.mu, 0, 6*sizeof (double));
        }
        perllcm_van_feature_t *f = vis_feature_cvharris (img_warp, mask, &harris_params, state->featpatch_sampler,
                                                         p3d, &calib_gsl.K.matrix, FEATURE_POINTS_MAX);
        int64_t dt = timestamp_now () - t0;
        if (f) {
            vis_feature_collection_add (fc, f);
            printf ("cvharris\t\tnpts=%4d\tdt=%"PRId64"\n", f->npts, dt);
        }
        else
            ERROR ("vis_feature_cvharris(): returned NULL");
    }
    
    if (state->ftypes & PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU) {
        int64_t t0 = timestamp_now ();
        perllcm_van_feature_t *f = vis_feature_siftgpu (img_warp, mask_siftgpu, state->siftgpu_server, -1, FEATURE_POINTS_MAX);
        int64_t dt = timestamp_now () - t0;
        if (f) {
            vis_feature_collection_add (fc, f);
            printf ("siftgpu\t\tnpts=%4d\tdt=%"PRId64"\n", f->npts, dt);
        }
        else
            ERROR ("vis_feature_siftgpu(): returned NULL");
    }

    if (fc->ntypes > 0) {
        // add our calibration
        // -------------------------------------------------------------- //
        if (calib != NULL)
            fc->calib = *calib;

        if (state->use_dvl_depth_prior) {
            // compute scene depth prior from bathymetry
            // -------------------------------------------------------------- //
            int64_t t0 = timestamp_now ();
            gsl_vector_view x_lc;
            if (state->use_position) {
                x_lc = gsl_vector_view_array (pdata->x_lc, 6);
            } else {
                double tmp[6] = {0};
                x_lc = gsl_vector_view_array (tmp, 6);
            }    
            gsl_matrix_view XYZ_l = gsl_matrix_submatrix (pdata->bathy_l, 0, 0, 3, pdata->bathy_l->size2);
            gsl_vector_view bid = gsl_matrix_row (pdata->bathy_l, 3);
            vis_feature_collection_scene_prior (fc, &XYZ_l.matrix, &bid.vector, &x_lc.vector,  calib);
            int64_t dt = timestamp_now () - t0;
            printf ("scene_prior\tnpts=%4d\tdt=%"PRId64"\n", fc->scene_prior.npts, dt);
            fc->scene_prior.utime = fc->utime;
    
            if (fc->scene_prior.npts == 0) {
                printf ("************************************************************\n");
                printf ("Warning: No DVL points project into bbox of image %"PRId64"!\n", pdata->utime);
                printf ("************************************************************\n");
            }
        } else {
            perllcm_van_scene_prior_t sp0 = {0};
            fc->scene_prior = sp0;
            fc->scene_prior.npts = 0;
        }

        // publish feature collection
        perllcm_van_feature_collection_t_publish (state->lcm, state->feature_channel, fc);
        
        // save features
        char filename[PATH_MAX];
        snprintf (filename, sizeof filename, "%s/%"PRId64".feat", state->cache_dir, pdata->utime);
        int32_t ret = LCMU_FWRITE (filename, fc, perllcm_van_feature_collection_t);
        if (ret < 0)
            ERROR ("couldn't write %s to disk!", filename);
    }
    else {
        printf ("Warning: no features detected in image %"PRId64"\n", pdata->utime);
    }

    if (state->pub_undist_img) {
        
        bot_core_image_t img = vis_iplimage_to_botimage_view (img_warp);
        img.utime = pdata->utime;
        char tmp[1024] = {0};
        sprintf (tmp, "%s_UNDIST", state->image_channel);
        bot_core_image_t_publish (state->lcm, tmp, &img);
    }

    // save unwarped image...
    // -------------------------------------------------------------- //
    vis_cvu_iplimg_save (state->cache_dir, pdata->utime, img_warp);
    
    // clean up
    // -------------------------------------------------------------- //
    perllcm_van_feature_collection_t_destroy (fc);
    if (img_8bit != pdata->img_gray)
        cvReleaseImage (&img_8bit);
    cvReleaseImage (&img_warp);
    pooldata_free (pdata);
    
}

static void
camera_params (camera_params_t *camera, BotParam *param, const char *cfgkey)
{
    camera->calib = vis_calib_load_config (param, cfgkey);
    
    // idealy would add this functionality to distortion.c/h and calib.c/h but
    // would need change of lcmdef to includes pointers which breaks current
    // rtvan implementation, also changes def of feature collection, and bathy collection
    if (camera->calib.kc_model == PERLLCM_VAN_CALIB_T_KC_MODEL_FULL_MAP)  {
    
        vis_cvu_map_t *map = malloc (sizeof (*map));
        map->mapu = cvCreateMat (camera->calib.height, camera->calib.width, CV_32FC1);
        map->mapv = cvCreateMat (camera->calib.height, camera->calib.width, CV_32FC1);
    
        char key[1024];
        snprintf (key, sizeof key, "%s.undist_map_path", cfgkey);
        char *undist_map_path = botu_param_get_str_or_default (param, key, "");
        //snprintf (key, sizeof mykey, "%s.dist_map_path", cfgkey);
        //char *dist_map_path = botu_param_get_str_or_default (param, key, "");
        
        perllcm_vis_cvu_map_t *lcm_map;
	LCMU_FREAD (undist_map_path, &lcm_map, perllcm_vis_cvu_map_t);

            
        for (int i=0; i<lcm_map->height; i++) {
            for (int j=0; j<lcm_map->width; j++) {
                cvmSet (map->mapu, i, j, lcm_map->mapu[i*lcm_map->width + j]);
                cvmSet (map->mapv, i, j, lcm_map->mapv[i*lcm_map->width + j]);    
            }
	}
                
        vis_calib_const_view_cv_t cv = vis_calib_const_view_cv (&(camera->calib));
        IplImage *Imask = cvCreateImage (cv.imageSize, IPL_DEPTH_8U, 1);
        IplImage *I = cvCreateImage (cv.imageSize, IPL_DEPTH_8U, 1);
    
        cvSet (I, cvScalarAll (255), NULL);
        cvRemap (I, Imask, map->mapu, map->mapv, 
                 CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));
        
        cvReleaseImage (&I);
        
        camera->map = map;
        camera->mask = Imask;
        
    } else {    
        camera->map = vis_undistort_map (&camera->calib);
        camera->mask = vis_undistort_mask (&camera->calib);
    }
    
    // enlarged mask for siftgpu
    int pad = 25;
    IplConvKernel *strel = cvCreateStructuringElementEx (pad, pad, (pad-1)/2, (pad-1)/2, CV_SHAPE_ELLIPSE, NULL);
    camera->mask_siftgpu = cvCloneImage (camera->mask);
    cvErode (camera->mask, camera->mask_siftgpu, strel, 1);
    cvReleaseStructuringElement (&strel);
}

//----------------------------------------------------------------------------------
// Called when program shuts down 
//----------------------------------------------------------------------------------
state_t *state_g = NULL;

static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\nShutting down ...\n");
    if (state_g->done) {
        printf ("Goodbye\n");
        exit (-1);
    } else {
        state_g->done = 1;
    }
}

int
main (int argc, char **argv)
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    printf ("%s ... \n", argv[0]);

    // feature_thread state
    state_t *state = calloc (1, sizeof (*state));
    state_g = state;
    
    // install custom signal handler
    struct sigaction act = {
        .sa_sigaction = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    // Read in the command line config key -------------------------------------
    // needs all options or it will break
    getopt_t *gopt_cfgkey = getopt_create ();
    getopt_add_description (gopt_cfgkey, "Extracts features from a stream of images");
    getopt_add_bool        (gopt_cfgkey, 'h', "help",             0,  "Display Help");
    getopt_add_string      (gopt_cfgkey, 'k', "configkey",        "", "Feature config file key");
    getopt_add_string      (gopt_cfgkey, 'i', "image_channel",    "", "Image channel");
    getopt_add_string      (gopt_cfgkey, 'f', "feature_channel",  "", "Feature channel");
    getopt_add_string      (gopt_cfgkey, 'd', "cache_dir",        "", "Cache directory");
    getopt_add_string      (gopt_cfgkey, 't', "feature_types",    "", "Feature types seperated by colons (cvsurf:siftgpu:harris)");
    getopt_add_string      (gopt_cfgkey, 's', "siftgpu_server",   "", "IP address of siftgpu server, blank for localhost");
    getopt_add_string      (gopt_cfgkey, 'c', "cam_config_key",   "", "Camera key in config file");
    getopt_add_string      (gopt_cfgkey, 'x', "cam_calib_key",    "", "Camera calib key in config file");
    getopt_add_string      (gopt_cfgkey, 'p', "position_channel", "", "Position channel");                                   
    getopt_add_bool        (gopt_cfgkey, 'u', "pub_undist",       0,  "Publish undistorted images");
    getopt_add_bool        (gopt_cfgkey, 'c', "use_clahs",        0,  "Perform contrast limited adaptive histogram specification");

    if (!getopt_parse (gopt_cfgkey, argc, argv, 1)) {    
        getopt_do_usage (gopt_cfgkey, "");
        return EXIT_FAILURE;
    }
    
    state->config_key = (char *) getopt_get_string (gopt_cfgkey, "configkey");  
    
    char cfg_key_str[1024] = {0};
    char ftypes_tmp[1024] = {0};
    char *camera_calib_key_tmp  = NULL;
    char *camera_config_key_tmp = NULL;
    char *position_channel_tmp  = NULL;
    char *image_channel_tmp     = NULL;
    char *feature_channel_tmp   = NULL;
    char *cache_dir_tmp         = NULL;
    char *siftgpu_server_tmp    = NULL;
    // parse config file -------------------------------------------------------
    state->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (! state->param) {
      ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
      exit (EXIT_FAILURE);
    }
    
    sprintf (cfg_key_str, "%s.ftypes", state->config_key);
    int n_ftypes = bot_param_get_array_len (state->param, cfg_key_str);
    if (n_ftypes > 0) {
        char **ftypes = bot_param_get_str_array_alloc (state->param, cfg_key_str);
        for (size_t n=0; n<n_ftypes; n++) {
            strcat (ftypes_tmp, ftypes[n]);
            if (n<n_ftypes-1)
                strcat (ftypes_tmp, ":");
        }
    } else {
        sprintf (ftypes_tmp, "siftgpu");
    }
    
    sprintf (cfg_key_str, "%s.siftgpu_server", state->config_key);
    siftgpu_server_tmp = botu_param_get_str_or_default (state->param, cfg_key_str, "");
    
    sprintf (cfg_key_str, "%s.pub_undist_img", state->config_key);
    state->pub_undist_img = 0;
    bot_param_get_int (state->param, cfg_key_str, &state->pub_undist_img);
    
    sprintf (cfg_key_str, "%s.use_clahs", state->config_key);
    state->use_clahs = 0;
    bot_param_get_int (state->param, cfg_key_str, &state->use_clahs);
    
    sprintf (cfg_key_str, "%s.camera_calib_key", state->config_key);
    camera_calib_key_tmp = botu_param_get_str_or_default (state->param, cfg_key_str, "");
    
    // use position
    sprintf (cfg_key_str, "%s.camera_config_key", state->config_key);
    camera_config_key_tmp = botu_param_get_str_or_default (state->param, cfg_key_str, "");
    
    sprintf (cfg_key_str, "%s.position_channel", state->config_key);
    position_channel_tmp = botu_param_get_str_or_default (state->param, cfg_key_str, "");
    
    sprintf (cfg_key_str, "%s.image_channel", state->config_key);    
    image_channel_tmp = botu_param_get_str_or_default (state->param, cfg_key_str, "IMAGE");
    
    sprintf (cfg_key_str, "%s.feature_channel", state->config_key);
    feature_channel_tmp = botu_param_get_str_or_default (state->param, cfg_key_str, "FEATURES");
    
    sprintf (cfg_key_str, "%s.cache_dir", state->config_key);
    cache_dir_tmp = botu_param_get_str_or_default (state->param, cfg_key_str, ".");
    
    sprintf (cfg_key_str, "%s.dvl_scene_depth_prior", state->config_key);
    state->use_dvl_depth_prior = bot_param_has_key (state->param, cfg_key_str);
    if (state->use_dvl_depth_prior)
        bot_param_get_str(state->param, cfg_key_str, &state->dvl_channel);
    
    // patch parameters (circular patch around a feature point as a descriptor)
    sprintf (cfg_key_str, "%s.feat_patch_w", state->config_key);
    size_t w = 5;
    bot_param_get_int (state->param, cfg_key_str, (int*)&w);
    state->featpatch_sampler = vis_feature_patch_sampler_alloc (w);
        
       
    // Read in the command line options ----------------------------------------
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "Extracts features from a stream of images");
    getopt_add_bool        (gopt, 'h', "help",             0,                     "Display Help");
    getopt_add_string      (gopt, 'k', "configkey",        "",                    "Feature config file key");
    getopt_add_string      (gopt, 'i', "image_channel",    image_channel_tmp,     "Image channel");
    getopt_add_string      (gopt, 'f', "feature_channel",  feature_channel_tmp,   "Feature channel");
    getopt_add_string      (gopt, 'd', "cache_dir",        cache_dir_tmp,         "Cache directory");
    getopt_add_string      (gopt, 't', "feature_types",    ftypes_tmp,            "Feature types seperated by colons (cvsurf:siftgpu:harris)");
    getopt_add_string      (gopt, 's', "siftgpu_server",   siftgpu_server_tmp,    "IP address of siftgpu server, blank for localhost");
    getopt_add_string      (gopt, 'c', "cam_config_key",   camera_config_key_tmp, "Camera key in config file");
    getopt_add_string      (gopt, 'x', "cam_calib_key",    camera_calib_key_tmp,  "Camera calib key in config file");
    getopt_add_string      (gopt, 'p', "position_channel", position_channel_tmp,  "Position channel");                                   
    getopt_add_bool        (gopt, 'u', "pub_undist_img",   state->pub_undist_img, "Publish undistorted images");
    getopt_add_bool        (gopt, 'c', "use_clahs",        state->use_clahs,      "Perform contrast limited adaptive histogram specification");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        return EXIT_FAILURE;
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        return EXIT_SUCCESS;
    }
    
    // set config options ------------------------------------------------------
    char *ftypes_opts = strdup (getopt_get_string (gopt, "feature_types"));
    char *result = NULL;
    result = strtok (ftypes_opts, ":");
    while( result != NULL ) {
        if (0==strcasecmp (result, "cvsurf"))
            state->ftypes |= PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF;
        else if (0==strcasecmp (result, "harris")) 
            state->ftypes |= PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS;
        else if (0==strcasecmp (result, "siftgpu"))
            state->ftypes |= PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU;
        result = strtok (NULL, ":");
    }
    free (ftypes_opts);
    
    if (0 == strcmp (getopt_get_string (gopt, "siftgpu_server"), "")) {
        if(state->siftgpu_server != NULL) {
            free (siftgpu_server_tmp);
            state->siftgpu_server = NULL;
        }
    } else {
        state->siftgpu_server = strdup (getopt_get_string (gopt, "siftgpu_server"));
    }
    if(siftgpu_server_tmp != NULL)
        free (siftgpu_server_tmp);
        
    state->image_channel = strdup (getopt_get_string (gopt, "image_channel"));
    if (image_channel_tmp != NULL)
        free(image_channel_tmp);
    state->feature_channel = strdup (getopt_get_string (gopt, "feature_channel"));
    if (feature_channel_tmp != NULL)
        free(feature_channel_tmp);
    state->cache_dir = strdup (getopt_get_string (gopt, "cache_dir"));
    if (cache_dir_tmp != NULL)
        free(cache_dir_tmp);    
    
    if (getopt_has_flag (gopt, "cam_calib_key")) {
        state->camera_calib_key = strdup (getopt_get_string (gopt, "cam_calib_key"));
        camera_params (&state->camera_params, state->param, state->camera_calib_key);
        if (camera_calib_key_tmp != NULL)
            free (camera_calib_key_tmp);
            
        state->have_camera_params = 1;
    } else {
        state->have_camera_params = 0;
    }
    
    state->use_position = bot_param_has_key (state->param, cfg_key_str);
    if (state->use_position) {
        state->position_channel = strdup (getopt_get_string (gopt, "position_channel"));
        state->camera_config_key = strdup (getopt_get_string (gopt, "camera_config_key"));
        sprintf (cfg_key_str, "%s.x_vs", state->camera_config_key);
        bot_param_get_double_array_or_fail (state->param, cfg_key_str, state->x_vs, 6);
        state->x_vs[3] = state->x_vs[3] * DTOR;
        state->x_vs[4] = state->x_vs[4] * DTOR;
        state->x_vs[5] = state->x_vs[5] * DTOR;
        
        if (camera_config_key_tmp != NULL)
            free (camera_config_key_tmp);
        if (position_channel_tmp != NULL)
            free (position_channel_tmp);
    }
    
    
    state->pub_undist_img = getopt_get_bool (gopt, "pub_undist_img");
    state->use_clahs = getopt_get_bool (gopt, "use_clahs");
    
    // image process step
    state->imgcounter = 0;
    state->imgstep = 1;
    sprintf (cfg_key_str, "%s.imgstep", state->config_key);
    bot_param_get_int(state->param, cfg_key_str, &state->imgstep);

    state->imgcache = cache_new (IMGCACHE_MAX, &imgcache_value_copy, &imgcache_value_destroy);

    // helper thread pool
    state->pool = g_thread_pool_new (feature_pool_thread, state, POOL_THREADS_MAX, 1, NULL);

    // initialize lcm 
    state->lcm = lcm_create (NULL);
    if (!state->lcm) {
        printf ("ERROR: lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }
    
    // lcm subscriptions
    bot_core_image_t_subscription_t *bot_core_image_t_sub = 
        bot_core_image_t_subscribe (state->lcm, state->image_channel, 
                                    &bot_core_image_t_callback, state);

    perllcm_position_t_subscription_t *perllcm_position_t_sub = NULL;
    if (state->use_position) {
        perllcm_position_t_sub =  perllcm_position_t_subscribe (state->lcm, state->position_channel, 
                                                                &perllcm_position_t_callback, state);
    }
    
    perllcm_rdi_bathy_t_subscription_t *perllcm_rdi_bathy_t_sub = NULL;
    if (state->use_dvl_depth_prior) {
        state->bathy_l = gsl_matrix_alloc (4, BATHY_POINTS_MAX);
        gsl_matrix_set_all (state->bathy_l, GSL_NEGINF);
        perllcm_rdi_bathy_t_sub = perllcm_rdi_bathy_t_subscribe (state->lcm, state->dvl_channel, 
                                                                 &perllcm_rdi_bathy_t_callback, state);
    }

    while (!state->done) {
        struct timeval timeout = {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        lcmu_handle_timeout (state->lcm, &timeout);
    }
    cvDestroyAllWindows ();

    // wait for thread pool to expire
    g_thread_pool_free (state->pool, 1, 1);

    // clean up
    bot_core_image_t_unsubscribe (state->lcm, bot_core_image_t_sub);
    if (state->use_position)
        perllcm_position_t_unsubscribe (state->lcm, perllcm_position_t_sub);    
    
    if (state->use_dvl_depth_prior)
        perllcm_rdi_bathy_t_unsubscribe (state->lcm, perllcm_rdi_bathy_t_sub);

    lcm_destroy (state->lcm);
    cache_destroy (state->imgcache);

    gslu_matrix_free (state->featpatch_sampler);
    free (state);
    
    printf ("Exiting. \n");
    
    return 0;
}
