#ifndef __PERLS_PVN_IMAGE_DATA_H__
#define __PERLS_PVN_IMAGE_DATA_H__

#include <iostream>
#include <vector>
#include <string>


#include <bot_param/param_client.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <lcm/lcm-cpp.hpp>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/lcm_util.h"
#include "perls-common/units.h"
#include "perls-vision/opencv_util.h"

#include "perls-lcmtypes++/bot_core/image_t.hpp"
#include "perls-lcmtypes++/perllcm/van_feature_collection_t.hpp"

#include "perls-lcmtypes/perllcm_vis_cvu_map_t.h"

/**
 * @defgroup PerlsPVN 
 * @brief C++ Utilities for Persistent Visual Navigation *
 * @{
 */


/**
 * @brief Load camera configure information from the config file
 * 
 */
class PVNCamConfig {
    
  public:
    std::string image_channel;
    std::string feature_channel;
    std::string cam_dir;
    double x_hs[6];         // head to camera (identity when not lb3)
    double x_vh[6];         // vehicle to head
    double x_vel_h[6];      // velodyne to sensor head
    Eigen::Matrix < double, 3, 3, Eigen::RowMajor > K;
    vis_cvu_map_t *udist_map;
    IplImage      *mask;
    int height;
    int width;
    
    
    PVNCamConfig ();
    
    /**
    * @brief Load camera configure information from the config file
    *
    * @param *param pointer to botparam config file
    * @param cam_num camera number [0 ... 5]
    * 
    */
    int
    LoadLB3Config (BotParam *param, int cam_num);
    
    /**
    * @brief Load camera configure information from the config file
    *
    * @param *param pointer to botparam config file
    * @param *cam_key string of camera key in config file, will be expanded
    * with .x_vs and .cam_calib
    * @param *feat_key string of feature key in config file,
    * will be expanded with .image_channel, .feature_channel, .logdir
    * 
    */
    int
    LoadMonoConfig (BotParam *param, std::string cam_key, std::string feature_key);
    
    ~PVNCamConfig ();
        
};

/**
 * @brief image and feature data, allows for multiple images, for stero and omni-cameras
 * 
 */
class PVNImageData {
  private:
    
    std::string file_extension;
    std::vector<PVNCamConfig> camconf;
    std::vector<bool> received;
    std::vector<std::string> image_dirs;
        
    bool
    data_ready (void);
    
    void
    feature_cb (const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                const perllcm::van_feature_collection_t *msg);
    
    int
    load_cached (lcm::LCM *lcm);
    
    int
    load_std_fmt (std::vector<bot_core::image_t> &images);
    
    int
    load_pgr (std::vector<bot_core::image_t> &images, bool color = false);
    
    vector<bot_core::image_t>
    undistort_images (vector<bot_core::image_t> images);
    
    vector<bot_core::image_t>
    resize_images (std::vector<bot_core::image_t> images, int resize[2]);
    
  public:
    
    int64_t utime;
    int num_images;
    int usec_offset;
    
    // raw features and depth map
    std::vector<perllcm::van_feature_collection_t> feature_collections;
    // images
    vector<bot_core::image_t> images;
    
    PVNImageData (std::vector<PVNCamConfig> _camconf, std::vector<std::string> _image_dirs,
               int64_t _utime, std::string _file_extension, int _usec_offset = 0);
    
    ~PVNImageData (void);
    
    /**
     * @brief calculate the features for this image
     */
    int    
    load_features (lcm::LCM *lcm, int resize[2] = NULL);
    
    /**
     * @brief load the original images
     */
    int    
    load_orig_img (lcm::LCM *lcm);
    
    /**
     * @brief copy feature data into a pvn_eview_map_exemplar
     */
    perllcm::pvn_eview_map_exemplar_t
    to_pvn_eview_map_exemplar_t (perllcm::pose3d_t x_n_e);
    
    /**
     * @brief copy feature data into a opencv matrix
     */
    cv::Mat
    to_opencv_mat (void);
    
    
    /**
     * @brief get the number of features
     */
    int
    get_num_feats (void);
    
};

/**
 * @brief helper function to load all configs for the lb3
 * 
 */
std::vector<PVNCamConfig>
pvn_cam_config_load_lb3 (BotParam *param);

/**
 * @}
 */

#endif // __PERLS_PVN_IMAGE_DATA_H__