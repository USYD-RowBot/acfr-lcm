#ifndef __RTVAN_SHARED_MEMORY_H__
#define __RTVAN_SHARED_MEMORY_H__

#include <stdbool.h>

// external linking req'd
#include <glib.h>
#include <opencv/cv.h>
#include <bot_param/param_client.h>

#include "perls-common/cache.h"
#include "perls-common/getopt.h"

#include "perls-vision/calib.h"
#include "perls-vision/opencv_util.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_van_options_t.h"

#include "van_util.h"

// internally used van lcm messages
#define VAN_DVL_BATHY_CHANNEL              "VAN_DVL_BATHY"
#define VAN_CAMERA_POSE_UW_CHANNEL         "VAN_CAMERA_POSE_UW"
#define VAN_CAMERA_POSE_PERI_CHANNEL       "VAN_CAMERA_POSE_PERI"
#define VAN_FEATURE_COLLECTION_CHANNEL     "VAN_FEATURES"
#define VAN_WORDS_CHANNEL                  "VAN_WORDS"
#define VAN_PLINK_CHANNEL                  "VAN_PLINKS"
#define VAN_VLINK_CHANNEL                  "VAN_VLINKS"
#define VAN_OPTIONS_CHANNEL                "VAN_OPTIONS"
#define VAN_PLOT_DEBUG_CHANNEL             "VAN_PLOT_DEBUG"
#define VAN_VERIFY_CHANNEL                 "VAN_VERIFY_LINK"
#define VAN_V2C_POSE_UW_CHANNEL            "VAN_V2C_POSE_UW"
#define VAN_V2C_POSE_PERI_CHANNEL          "VAN_V2C_POSE_PERI"
#define VAN_SALIENCY_CHANNEL               "VAN_SALIENCY"

#define SE_PLINK_CHANNEL                   "SE_PROPOSE_LINK_SERVER"     // plink from server
#define SE_VLINK_CHANNEL                   "SE_PUBLISH_LINK_CLIENT"     // vlink from client
#define SE_RETURN_ST_CHANNEL               "SE_RETURN_STATE_SERVER"     // return from server
#define SE_REQUEST_ST_CHANNEL              "SE_REQUEST_STATE_CLIENT"    // request from client
#define SE_ADD_NODE_CHANNEL                "SE_ADD_NODE_CLIENT"         // add node from client
#define SE_ADD_MODE_ACK_CHANNEL            "SE_ADD_NODE_ACK"            // only server reports it
#define SE_OPTION_ACK_CHANNEL              "SE_OPTION_ACK"              // option ack from seserver
#define SE_OPTION_CMD_CHANNEL              "SE_OPTION_VIEWER"           // direct command from viewer
#define SE_GOTO_CHANNEL                    "SE_GOTO"                    // se goto waypoint message from seserver
#define SE_SAVE_ISAM_CHANNEL               "SE_SAVE_ISAM"               // save isam graph via lcm

#define VAN_PLOT_CAM_TARGET                "VAN_PLOT_CAM_TARGET"        // target image via bot_coregl

#define VAN_LINK_INFO_CHANNEL              "VAN_LINK_INFO"              // contain vlink info for post processing

#define TIMER_SURF_DT                       1

#ifdef __cplusplus
extern "C" {
#endif

typedef struct camera_params camera_params_t;
struct camera_params {
    perllcm_van_calib_t calib;
    vis_cvu_map_t *map;
    IplImage      *mask;
    IplImage      *mask_siftgpu;
};

typedef struct shm shm_t;
struct shm {
    GStaticMutex mutex;
    bool         done;
    BotParam    *param;
    getopt_t    *gopt;
    char        *logdir;

    // img_warp list
    cache_t     *imgcache;

    // lcm parent object
    lcm_t      *lcm;
    const char *cameraUw_rootkey, *cameraPeri_rootkey;
    const char *bot_core_image_t_channelUw, *bot_core_image_t_channelPeri;
    const char *bot_core_image_sync_t_channelUw, *bot_core_image_sync_t_channelPeri;

    // camera calibration
    camera_params_t waterUw, waterPeri;
    camera_params_t airUw, airPeri;

    // thread id's
    GThread *tid_feature_thread;
    GThread *tid_hauv_thread;
    GThread *tid_saliency_thread;
    GThread *tid_link_thread;
    GThread *tid_plot_thread;
    GThread *tid_isam_interf_thread;
    GThread *tid_twoview_thread;

    // plot / link options
    perllcm_van_options_t van_opts;
    bool active;
    int  imgstep;

    // utimelist for valid nodes
    GSList *utimelist;
};

extern shm_t *shm;

shm_t *
shared_memory_init (int argc, char *argv[]);

void
shared_memory_free (shm_t *shm);

#ifdef __cplusplus
}
#endif

#endif //__RTVAN_SHARED_MEMORY_H__
