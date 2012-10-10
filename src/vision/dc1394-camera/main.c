#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <glib.h>
#include <signal.h>
#include <fcntl.h>

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#include "perls_dc1394_camera.h"


typedef struct _setting_t setting_t;
struct _setting_t
{
    int manual;     // manual = 1, automatic = 0
    double value;
};

// config structure loaded from cfg file
typedef struct _cam_config_t cam_config_t;
struct _cam_config_t 
{
    char cam_id[256];
    dc1394video_mode_t dc1394_video_mode;
    dc1394color_coding_t dc1394_color_coding;
    dc1394color_filter_t dc1394_color_filter; // bayer pattern
    int bot_pixelformat;

    // standard
    setting_t framerate;
    setting_t speed1394b; 

    // format 7
    setting_t width;
    setting_t height;
    setting_t startx;
    setting_t starty;

    // regular settings
    setting_t brightness;
    setting_t exposure;     // has auto
    setting_t gamma;
    setting_t shutter;  // has auto
    setting_t gain; // has auto
    setting_t whitebalance_blue; // has auto
    setting_t whitebalance_red; // has auto

    char image_channel[256];
};

typedef struct _config_t config_t;
struct _config_t
{
    int num_cameras;
    cam_config_t *camera_configs;
};

// define state structure
typedef struct _state_t state_t;
struct _state_t {
    int done;
    int is_daemon;
    lcm_t *lcm;
    timestamp_sync_state_t **tss;
};

//Init the state structure
state_t state = {0};

void
read_option (BotParam *param, const char *key, config_t *config, int config_offset)
{
    int n = bot_param_get_array_len (param, key);
    setting_t *setting;

    if (n > 0) {
        double *tmp_arr = calloc (n, sizeof (double));
        bot_param_get_double_array (param, key, tmp_arr, n);

        for (int i=0; i < n; ++i) {
            setting = (setting_t*) ((intptr_t)&config->camera_configs[i] + config_offset);
            setting->manual = 1;
            setting->value = tmp_arr[i];
        }
        for (int i=1; n == 1 && i < config->num_cameras; ++i) {
            setting = (setting_t*) ((intptr_t)&config->camera_configs[i] + config_offset);
            setting->manual = 1;
            setting->value = tmp_arr[0];
        }

        free (tmp_arr);
    }
}



void
perls_dc1394_camera_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }

    int n;
    char **tmp;

    // read in camera ID's
    config->num_cameras = bot_param_get_array_len (param, "dc1394-camera.cam_id");

    if (config-> num_cameras < 1) {
        ERROR ("Must specify at least one camera ID\n");
        exit (EXIT_FAILURE);
    }

    config->camera_configs = calloc (config->num_cameras, sizeof (cam_config_t));

    tmp = bot_param_get_str_array_alloc (param, "dc1394-camera.cam_id");
    for (int i=0; i < config->num_cameras; ++i)
        strcpy (config->camera_configs[i].cam_id, tmp[i]);

    bot_param_str_array_free (tmp);



    // read in video modes
    n = bot_param_get_array_len (param, "dc1394-camera.video_mode");

    if (n > 0) {
        tmp = bot_param_get_str_array_alloc (param, "dc1394-camera.video_mode");

        for (int i=0; i < n && i < config->num_cameras; ++i)
            config->camera_configs[i].dc1394_video_mode = dc1394_videomode_string_to_int (tmp[i]);
        for (int i=1; n == 1 && i < config->num_cameras; ++i)
            config->camera_configs[i].dc1394_video_mode = config->camera_configs[0].dc1394_video_mode;

        bot_param_str_array_free (tmp);
    }



    // read in color codes
    n = bot_param_get_array_len (param, "dc1394-camera.color_code");

    if (n > 0) {
        tmp = bot_param_get_str_array_alloc (param, "dc1394-camera.color_code");

        for (int i=0; i < n && i < config->num_cameras; ++i)
            config->camera_configs[i].dc1394_color_coding = dc1394_colorcode_string_to_int (tmp[i]);
        for (int i=1; n == 1 && i < config->num_cameras; ++i)
            config->camera_configs[i].dc1394_color_coding = config->camera_configs[0].dc1394_color_coding;

        bot_param_str_array_free (tmp);
    }



    // read in lcm channels
    n = bot_param_get_array_len (param, "dc1394-camera.lcm_channel");

    if (n > 0) {
        tmp = bot_param_get_str_array_alloc (param, "dc1394-camera.lcm_channel");

        for (int i=0; i < n && i < config->num_cameras; ++i)
            strcpy (config->camera_configs[i].image_channel, tmp[i]);
        for (int i=0; n == 1 && i < config->num_cameras; ++i)
            snprintf (config->camera_configs[i].image_channel, 256, "%s%d", tmp[0], i);

        bot_param_str_array_free (tmp);
    }
    else {
        for (int i=0; i < config->num_cameras; ++i)
            snprintf (config->camera_configs[i].image_channel, 256, "IMAGE%d", i);
    }


    // get all other camera settings
    read_option (param, "dc1394-camera.framerate", config, offsetof (cam_config_t, framerate));
    read_option (param, "dc1394-camera.speed1394b", config, offsetof (cam_config_t, speed1394b));
    read_option (param, "dc1394-camera.brightness", config, offsetof (cam_config_t, brightness));
    read_option (param, "dc1394-camera.exposure", config, offsetof (cam_config_t, exposure));
    read_option (param, "dc1394-camera.gamma", config, offsetof (cam_config_t, gamma));
    read_option (param, "dc1394-camera.shutter", config, offsetof (cam_config_t, shutter));
    read_option (param, "dc1394-camera.gain", config, offsetof (cam_config_t, gain));
    read_option (param, "dc1394-camera.whitebalance_blue", config, offsetof (cam_config_t, whitebalance_blue));
    read_option (param, "dc1394-camera.whitebalance_red", config, offsetof (cam_config_t, whitebalance_red));
    read_option (param, "dc1394-camera.width", config, offsetof (cam_config_t, width));
    read_option (param, "dc1394-camera.height", config, offsetof (cam_config_t, height));
    read_option (param, "dc1394-camera.startx", config, offsetof (cam_config_t, startx));
    read_option (param, "dc1394-camera.starty", config, offsetof (cam_config_t, starty));
}

int
pixelformat_dc1394_to_bot (dc1394color_coding_t dc1394_color, dc1394color_filter_t filter)
{
    int pix = -1;
    switch (dc1394_color) {
        case DC1394_COLOR_CODING_MONO8:     pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY; break;
        case DC1394_COLOR_CODING_YUV411:    pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV411P; break;
        case DC1394_COLOR_CODING_YUV422:    pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV420; break;
        case DC1394_COLOR_CODING_YUV444:    pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV420; break;
        case DC1394_COLOR_CODING_RGB8:      pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB; break;
        case DC1394_COLOR_CODING_MONO16:    pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16; break;
        case DC1394_COLOR_CODING_RGB16:     pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_RGB16; break;
        case DC1394_COLOR_CODING_MONO16S:   pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_GRAY16; break;
        case DC1394_COLOR_CODING_RGB16S:    pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_RGB16; break;
        case DC1394_COLOR_CODING_RAW8:      pix = 0; break; // NEED TO QUERY BAYER MODE
        case DC1394_COLOR_CODING_RAW16:     pix = 0; break; // NEED TO QUERY BAYER MODE
        default: pix = -1; break;
    }

    if (pix == 0) {
        switch (filter) {
            case DC1394_COLOR_FILTER_RGGB:  pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB; break;
            case DC1394_COLOR_FILTER_GBRG:  pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG; break;
            case DC1394_COLOR_FILTER_GRBG:  pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG; break;
            case DC1394_COLOR_FILTER_BGGR:  pix = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR; break;
            default: pix = -1; break;
        }
    }

    return pix;
}

//----------------------------------------------------------------------------------
// Called when program shuts down 
//----------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\nmy_signal_handler()\n");
    if (state.done) {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    } else {
        state.done = 1;
    }
}

//----------------------------------------------------------------------
// Main Loop
//----------------------------------------------------------------------
int
main (int argc, char **argv)
{
    dc1394error_t status;

    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // install custom signal handler
    struct sigaction act = {
        .sa_sigaction = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    config_t config = {0};
    
    // Read in the command line options
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "PERLS dc1394 Camera Driver: Publishes LCM image_t messages.");
    getopt_add_description (gopt, "Note: Config file settings are overridden by specifying a camera id with '-i',");
    getopt_add_description (gopt, "for a config file example, see perls_dc1394_camera.h");
    getopt_add_bool   (gopt,    'D',    "daemon",            0,   	    "Run as system daemon");
    getopt_add_bool   (gopt,    'l',    "list",              0,   	    "List FireWire cams on all buses");
    getopt_add_string (gopt,    'q',    "query",             "ID",      "Query camera by id for supported modes");
    getopt_add_string (gopt,    'i',    "camid",             "ID",	    "Specifies single camera ID to be used");
    getopt_add_string (gopt,    'c',    "channel",           "IMAGE",   "Image LCM channel name");
    getopt_add_bool   (gopt,    0,      "clean",             0,         "Clean entire FW bus (fixes resource busy)");
    getopt_add_bool   (gopt,    'h',    "help",              0,   	    "Display this help");
    getopt_add_spacer (gopt, "Camera configuration settings:");
    getopt_add_string (gopt,    'M',    "videomode",         "",         "DC1394 video mode");
    getopt_add_bool   (gopt,    'B',    "1394b",             0,          "Enable 1394b data rates");
    getopt_add_double (gopt,    'F',    "framerate",         "0",          "Framerate in fps");
    getopt_add_double (gopt,    'b',    "brightness",        "0",          "Brightness in \%");
    getopt_add_double (gopt,    'e',    "exposure",          "0",          "Exposure in EV");
    getopt_add_double (gopt,    'g',    "gamma",             "0",          "Gamma");
    getopt_add_double (gopt,    's',    "shutter",           "0",          "Shutter in s");
    getopt_add_double (gopt,    'G',    "gain",              "0",          "Gain in dB");
    getopt_add_double (gopt,    'r',    "whitebalance_red",  "0",          "White balance red");
    getopt_add_double (gopt,    'w',    "whitebalance_blue", "0",          "White balance blue");
    getopt_add_spacer (gopt, "FORMAT7 video mode settings:");
    getopt_add_string (gopt,    'C',    "colorcode",         "",           "DC1394 color code");
    getopt_add_int    (gopt,    'W',    "width",             "",           "Image width (default MAX)");
    getopt_add_int    (gopt,    'H',    "height",            "",           "Image height (default MAX)");
    getopt_add_int    (gopt,    'X',    "startx",            "0",          "Starting X pos. of ROI");
    getopt_add_int    (gopt,    'Y',    "starty",            "0",          "Starting Y pos. of ROI");


    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");

        return EXIT_FAILURE;
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");

        return EXIT_SUCCESS;
    }
    //start as daemon if asked
    if (getopt_get_bool (gopt, "daemon")) {
        daemon_fork ();
        state.is_daemon = 1;
    }
    else
        state.is_daemon = 0;

    if (getopt_get_bool (gopt, "list")) {
        if (perls_dc1394_camera_list () == DC1394_SUCCESS);
            return EXIT_SUCCESS;
        return EXIT_FAILURE;
    }

    if (getopt_has_flag (gopt, "query")) {
        if (perls_dc1394_camera_query (getopt_get_string (gopt, "query")) != DC1394_SUCCESS)
            return EXIT_SUCCESS;
        return EXIT_FAILURE;
    }

    if (getopt_get_bool (gopt, "clean")) {
        if (perls_dc1394_camera_clean () != DC1394_SUCCESS)
            return EXIT_SUCCESS;
        return EXIT_FAILURE;
    }

    if (getopt_has_flag (gopt, "camid")) {
        printf ("Camera specified, ignoring config file settings...\n");

        config.num_cameras = 1;
        config.camera_configs = calloc (config.num_cameras,  sizeof (cam_config_t));
        strcpy (config.camera_configs[0].cam_id, getopt_get_string (gopt, "camid"));

        strcpy (config.camera_configs[0].image_channel, getopt_get_string (gopt, "channel"));

        if (getopt_has_flag (gopt, "videomode")) {
            int ret = dc1394_videomode_string_to_int (getopt_get_string (gopt, "videomode"));
            if (ret == -1) {
                ERROR ("Video mode [%s] not recognized", getopt_get_string (gopt, "videomode"));
                free (config.camera_configs);
                return EXIT_FAILURE;
            }

            config.camera_configs[0].dc1394_video_mode = ret;
        }

        if (getopt_has_flag (gopt, "colorcode")) {
            int ret = dc1394_colorcode_string_to_int (getopt_get_string (gopt, "colorcode"));
            if (ret == -1) {
                ERROR ("Color code [%s] not recognized", getopt_get_string (gopt, "colorcode"));
                free (config.camera_configs);
                return EXIT_FAILURE;
            }

            config.camera_configs[0].dc1394_color_coding = ret;
        }

        if (getopt_has_flag (gopt, "framerate")) {
            config.camera_configs[0].framerate.manual = 1;
            config.camera_configs[0].framerate.value = getopt_get_double (gopt, "framerate");
        }

        if (getopt_has_flag (gopt, "1394b")) {
            config.camera_configs[0].speed1394b.manual = 1;
            config.camera_configs[0].speed1394b.value = 1;
        }

        if (getopt_has_flag (gopt, "width")) {
            config.camera_configs[0].width.manual = 1;
            config.camera_configs[0].width.value = getopt_get_double (gopt, "width");
        }

        if (getopt_has_flag (gopt, "height")) {
            config.camera_configs[0].height.manual = 1;
            config.camera_configs[0].height.value = getopt_get_double (gopt, "height");
        }

        if (getopt_has_flag (gopt, "startx")) {
            config.camera_configs[0].startx.manual = 1;
            config.camera_configs[0].startx.value = getopt_get_double (gopt, "startx");
        }

        if (getopt_has_flag (gopt, "starty")) {
            config.camera_configs[0].starty.manual = 1;
            config.camera_configs[0].starty.value = getopt_get_double (gopt, "starty");
        }

        if (getopt_has_flag (gopt, "brightness")) {
            config.camera_configs[0].brightness.manual = 1;
            config.camera_configs[0].brightness.value = getopt_get_double (gopt, "brightness");
        }

        if (getopt_has_flag (gopt, "exposure")) {
            config.camera_configs[0].exposure.manual = 1;
            config.camera_configs[0].exposure.value = getopt_get_double (gopt, "exposure");
        }

        if (getopt_has_flag (gopt, "gamma")) {
            config.camera_configs[0].gamma.manual = 1;
            config.camera_configs[0].gamma.value = getopt_get_double (gopt, "gamma");
        }

        if (getopt_has_flag (gopt, "shutter")) {
            config.camera_configs[0].shutter.manual = 1;
            config.camera_configs[0].shutter.value = getopt_get_double (gopt, "shutter");
        }

        if (getopt_has_flag (gopt, "gain")) {
            config.camera_configs[0].gain.manual = 1;
            config.camera_configs[0].gain.value = getopt_get_double (gopt, "gain");
        }

        if (getopt_has_flag (gopt, "whitebalance_blue")) {
            config.camera_configs[0].whitebalance_blue.manual = 1;
            config.camera_configs[0].whitebalance_blue.value = getopt_get_double (gopt, "whitebalance_blue");
        }

        if (getopt_has_flag (gopt, "whitebalance_red")) {
            config.camera_configs[0].whitebalance_red.manual = 1;
            config.camera_configs[0].whitebalance_red.value = getopt_get_double (gopt, "whitebalance_red");
        }
    }
    
    
    // load in config file option
    if (config.num_cameras == 0) 
        perls_dc1394_camera_load_cfg (&config);

    // initialize lcm 
    state.lcm = lcm_create (NULL);
    if (!state.lcm) {
        printf ("ERROR: lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }    

    printf ("num cameras = %d\n", config.num_cameras);
    // allocate handles for each camera
    PERLS_dc1394_camera_t **cameras = calloc (config.num_cameras, sizeof(PERLS_dc1394_camera_t*));

    for (int i=0; i < config.num_cameras; ++i) {

        // initialize and clean camera
        cameras[i] = perls_dc1394_camera_init (config.camera_configs[i].cam_id);

        if (cameras[i] == NULL) {
            ERROR ("Problem initializing camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }

        
        printf ("%s %s - %lx\n", cameras[i]->cam->vendor,cameras[i]->cam->model, cameras[i]->cam->guid);

        // set isochronous speed depending on 1394b compatibiliity
        // NOTE: unfortunately, we have to manually control the input here because Pt Grey
        // USB cameras report as 'bmode_capable'.. This may be fixed in future firmware updates
        if (cameras[i]->cam->bmode_capable > 0 && config.camera_configs[i].speed1394b.manual > 0
                && config.camera_configs[i].speed1394b.value > 0) {
            // attempt to set operation mode to 1394b
            status = perls_dc1394_camera_set_operation_mode (cameras[i], DC1394_OPERATION_MODE_1394B);
            if (status != DC1394_SUCCESS) {
                ERROR ("Problem setting operation mode on camera [%s]", config.camera_configs[i].cam_id);
                free (config.camera_configs);
                for (int x=0; x<config.num_cameras; ++x)
                    if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
                exit (EXIT_FAILURE);
            }

            // set iso speed 
            status = perls_dc1394_camera_set_iso_speed (cameras[i], DC1394_ISO_SPEED_800);
            if (status != DC1394_SUCCESS) {
                ERROR ("Problem setting iso speed on camera [%s]", config.camera_configs[i].cam_id);
                free (config.camera_configs);
                for (int x=0; x<config.num_cameras; ++x)
                    if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
                exit (EXIT_FAILURE);
            }
        }
        else {
            status = perls_dc1394_camera_set_operation_mode (cameras[i], DC1394_OPERATION_MODE_LEGACY);
            if (status != DC1394_SUCCESS) {
                ERROR ("Problem setting operation mode on camera [%s]", config.camera_configs[i].cam_id);
                free (config.camera_configs);
                for (int x=0; x<config.num_cameras; ++x)
                    if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
                exit (EXIT_FAILURE);
            }
            status = perls_dc1394_camera_set_iso_speed (cameras[i], DC1394_ISO_SPEED_400);
            if (status != DC1394_SUCCESS) {
                ERROR ("Problem setting iso speed on camera [%s]", config.camera_configs[i].cam_id);
                free (config.camera_configs);
                for (int x=0; x<config.num_cameras; ++x)
                    if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
                exit (EXIT_FAILURE);
            }
        }

        // set video mode
        status = perls_dc1394_camera_set_video_mode (cameras[i], config.camera_configs[i].dc1394_video_mode);
        if (status != DC1394_SUCCESS) {
            ERROR ("Problem setting video mode on camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }

        // determine if video mode is FORMAT_7 or not
        if (dc1394_is_video_mode_scalable (config.camera_configs[i].dc1394_video_mode)) {
            double f;
            int l, t, w, h;
            f = config.camera_configs[i].framerate.manual ? config.camera_configs[i].framerate.value : -1;
            l = config.camera_configs[i].startx.manual ? config.camera_configs[i].startx.value : -1;
            t = config.camera_configs[i].starty.manual ? config.camera_configs[i].starty.value : -1;
            w = config.camera_configs[i].width.manual ? config.camera_configs[i].width.value : -1;
            h = config.camera_configs[i].height.manual ? config.camera_configs[i].height.value : -1;

            status = perls_dc1394_camera_set_roi (cameras[i], config.camera_configs[i].dc1394_color_coding, f, l, t, w, h);
        }
        else {
            // ALL OTHER FORMATS

            // set framerate
            if (config.camera_configs[i].framerate.manual)
                status = perls_dc1394_camera_set_frame_rate (cameras[i], config.camera_configs[i].framerate.value);
            else
                status = perls_dc1394_camera_set_frame_rate (cameras[i], -1);
        }

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem configuring camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }


        status = perls_dc1394_camera_get_color_coding (cameras[i], &config.camera_configs[i].dc1394_color_coding);
        status += perls_dc1394_camera_get_bayer_pattern (cameras[i], &config.camera_configs[i].dc1394_color_filter);
        config.camera_configs[i].bot_pixelformat = pixelformat_dc1394_to_bot (
                config.camera_configs[i].dc1394_color_coding, config.camera_configs[i].dc1394_color_filter);

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem obtaining color code/filter for camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }

        //setup embedded timestamp
        status = perls_dc1394_camera_embed_timestamp_into_frame (cameras[i]);
        if (status != DC1394_SUCCESS) {
            ERROR ("Problem embedding timestamp into camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }
        

        // BRIGHTNESS
        status = perls_dc1394_camera_set_brightness (cameras[i], 
                config.camera_configs[i].brightness.manual, 
                config.camera_configs[i].brightness.value);

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem setting brightness of camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }

        // EXPOSURE
        status = perls_dc1394_camera_set_exposure (cameras[i], 
                config.camera_configs[i].exposure.manual, 
                config.camera_configs[i].exposure.value);

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem setting exposure of camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }

        // GAMMA
        status = perls_dc1394_camera_set_gamma (cameras[i], 
                config.camera_configs[i].gamma.manual, 
                config.camera_configs[i].gamma.value);

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem setting gamma of camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }

        // SHUTTER
        status = perls_dc1394_camera_set_shutter (cameras[i], 
                config.camera_configs[i].shutter.manual, 
                config.camera_configs[i].shutter.value);

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem setting shutter of camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }

        // GAIN
        status = perls_dc1394_camera_set_gain (cameras[i], 
                config.camera_configs[i].gain.manual, 
                config.camera_configs[i].gain.value);

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem setting gain of camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }
        

        // WHITEBALANCE
        status = perls_dc1394_camera_set_whitebalance (cameras[i], 
                config.camera_configs[i].whitebalance_blue.manual && config.camera_configs[i].whitebalance_red.manual,
                config.camera_configs[i].whitebalance_blue.value, config.camera_configs[i].whitebalance_red.value);

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem setting whitebalance of camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }
    }

    for (int i=0; i < config.num_cameras; ++i) {
        // cameras finally set up, start video
        status = perls_dc1394_camera_start_video_transmission (cameras[i]);

        if (status != DC1394_SUCCESS) {
            ERROR ("Problem starting video transmission on camera [%s]", config.camera_configs[i].cam_id);
            free (config.camera_configs);
            for (int x=0; x<config.num_cameras; ++x)
                if (cameras[x]) perls_dc1394_camera_camera_free (cameras[x]);
            exit (EXIT_FAILURE);
        }
    }
    


    dc1394video_frame_t **frames = calloc (config.num_cameras, sizeof(dc1394video_frame_t*));
    bot_core_image_t *img = calloc (config.num_cameras, sizeof (bot_core_image_t));
    int count = 0;

    //unused meta data filled
    for (int i=0; i < config.num_cameras; ++i) {
        img[i].nmetadata = 0;
        img[i].metadata = NULL;
    }
    
    // setup timestamp sync to estimate system time of camera framegrab
    state.tss = calloc (config.num_cameras, sizeof (timestamp_sync_state_t*));
    for (int i=0; i < config.num_cameras; ++i)
        state.tss[i] = timestamp_sync_init (1000000, 128000000, 1);
    
    // main capture loop
    int64_t utime_prev = 0;
    printf ("starting capture loop...\n");
    while (!state.done) {         
        for (int i=0; i < config.num_cameras; ++i) {
            if (dc1394_capture_dequeue (cameras[i]->cam, DC1394_CAPTURE_POLICY_WAIT, &frames[i])!=DC1394_SUCCESS) {
                printf ("DC1394 dequeue failed\n");
                return FALSE;
            }

            while (frames[i]->frames_behind > 0) {
                dc1394_capture_enqueue (cameras[i]->cam, frames[i]);
                if (dc1394_capture_dequeue (cameras[i]->cam, DC1394_CAPTURE_POLICY_WAIT, &frames[i])!=DC1394_SUCCESS) {
                    printf ("DC1394 dequeue failed\n");
                    return FALSE;
                }
            }
        }
        
        // calculate timestamp
        uint64_t timestamp;
        int64_t utime;
        for (int i=0; i < config.num_cameras; ++i) {
            status = perls_dc1394_camera_parse_embedded_timestamp (cameras[i], frames[i]->image, &timestamp);
            if (status != DC1394_SUCCESS)
                utime = timestamp_sync (state.tss[i], timestamp, timestamp_now());
            else
                utime = timestamp;
            
            // Pack structure and publish bot_core_image_t struct
            img[i].utime = utime;
            img[i].width = frames[i]->size[0];
            img[i].height = frames[i]->size[1];
            img[i].row_stride = frames[i]->stride;
            img[i].size = frames[i]->image_bytes;
            img[i].data = frames[i]->image;
            img[i].pixelformat = config.camera_configs[i].bot_pixelformat;
        }
            
        // Publish over LCM
        for (int i=0; i < config.num_cameras; ++i)
            bot_core_image_t_publish (state.lcm, config.camera_configs[i].image_channel, &img[i]);      

        printf ("FRAME #%d: ts=%"PRId64" frame_rate=%0.2lf Hz width=%d, height=%d, size=%d \r",
                count, img[0].utime,
                (1000000.0/(double)(utime-utime_prev)),
                img[0].width,img[0].height,img[0].size);

        utime_prev = utime;
        
        for (int i=0; i<config.num_cameras; ++i)
            dc1394_capture_enqueue (cameras[i]->cam, frames[i]);
        count++;
    }

    for (int i=0; i < config.num_cameras; ++i) {
        if (state.tss[i])
            timestamp_sync_free (state.tss[i]);

        perls_dc1394_camera_stop_video_transmission (cameras[i]);
        perls_dc1394_camera_camera_free (cameras[i]);
    }

    free (config.camera_configs);
    getopt_destroy (gopt);

    printf ("\nDone.\n");
    
    return EXIT_SUCCESS;
}
