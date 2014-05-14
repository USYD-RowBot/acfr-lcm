#ifndef __PERLS_PVN_UTIL_H__
#define __PERLS_PVN_UTIL_H__

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <Eigen/Core>

#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>

#include "perls-lcmtypes/perllcm_velodyne_laser_return_collection_t.h"
#include "perls-lcmtypes++/perllcm/van_scene_prior_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_vis_map_cluster_t.hpp"
#include "perls-lcmtypes++/perllcm/van_feature_collection_t.hpp"
#include "perls-lcmtypes++/perllcm/viewer_image_pccs_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_vis_map_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_eview_map_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_eview_map_neighborhood_t.hpp"


/**
 * @defgroup PerlsPVNUtil Persistent Visual Navigation Utility
 * @brief PVN helper library.
 * @ingroup PerlsPVN
 * @include: perls-pvn/pvn_util.h
 *
 * @{
 */


using namespace std;

/**
 * @brief transform a laser return collection into xyz point vector
 */
void
pvnu_lrc_to_xyz (std::vector < std::vector <float> > *xyz,
                          const perllcm_velodyne_laser_return_collection_t *lrc);

/**
 * @brief transform an exemplar into xyz vector
 */
void
pvnu_exemplar_to_xyz (std::vector < std::vector <float> > *xyz,
                 const perllcm::pvn_eview_map_exemplar_t *mne);

/**
 * @brief transform a pvn vis map cluster into the camera frame
 */
void
pvnu_vmc_to_xyz (std::vector < std::vector <float> > *xyz,
                          const perllcm::pvn_vis_map_cluster_t *vmc);

/**
 * @brief project 3D points in camera frame onto image plane. 
 */
void
pvnu_project_to_image (std::vector < std::vector <float> > *uv,
                       std::vector <int> *proj_inds,
                       std::vector < std::vector <float> > *xyz_cam_out,
                       const std::vector < std::vector <float> > xyz,
                       const double x_w_ci[6],
                       const Eigen::Matrix3d K,
                       int max_height,
                       int max_width,
                       double max_range,
                       bool lcmgl);

/**
 * @brief image to map matching with pccs  
 */
void
pvnu_image_map_matching (std::vector<int> *sel_map_out,
                        std::vector<int> *sel_img_out,
                        perllcm::van_feature_t *f_img,
                        perllcm::van_feature_t *f_map,
                        vector< vector<float> > xyz_map,
                        double x_w_ci[6],
                        double S_w_ci[6*6],
                        double K[3*3],
                        double sec_nn_thresh,
                        int use_pccs,
                        perllcm::viewer_image_pccs_t *vis_pccs_out);

/**
 * @brief pack a van scene prior 
 */
perllcm::van_scene_prior_t
pvnu_pack_scene_prior (std::vector < std::vector <float> > xyz_cam,
                       std::vector < std::vector <float> > uv,
                       std::vector <int> *proj_inds,
                       int64_t utime);


/**
 * @brief reject outliers based on reprojetion error
 */
std::vector<int>
pvnu_median_outlier_rejection (std::vector < float > u, std::vector < float > v,
                               std::vector < float > up, std::vector < float > vp,
                               float *reproj_error);


/**
 * @brief wait for lcm callbacks with timeout
 */
int
pvnu_lcm_handle_timeout (lcm::LCM *lcm, struct timeval *timeout);


/**
 * @brief read a binary lcm object written to file
 */
template <class type>
int32_t
pvnu_lcm_fread (const char* filename, type *lcm_out) {
    
        FILE *stream = fopen (filename, "r");
    if (!stream) {
        ERROR ("unable to read file %s!", filename);
        return -1;
    }

    size_t numel=0;
    int64_t magic = 0;
    numel += fread (&magic, sizeof magic, 1, stream);
    int64_t fingerprint = lcm_out->getHash();
    if (magic != fingerprint) {
        fclose (stream);
        ERROR ("invalid fingerprint for %s!", filename);
        return -3;
    }

    int32_t data_size=0;
    numel += fread (&data_size, sizeof data_size, 1, stream);
    uint8_t *buf = (uint8_t *)malloc (data_size);
    numel += fread (buf, data_size, 1, stream);
    fclose (stream);

    int ret = lcm_out->decode(buf, 0, data_size);
    if (numel != 3 || ret !=data_size) {
        free (buf);
        ERROR ("unable to decode %s!\n", filename);
        return -2;
    }

    free (buf);
    return data_size;
    
}

/**
 * @brief write a binary lcm object to file
 */
template <class type>
int32_t
pvnu_lcm_fwrite (const char* filename, type lcm_in) {
    
    FILE *stream = fopen (filename, "w");
    if (!stream) {
        ERROR ("unable to create file %s!", filename);
        return -1;
    }

    int32_t max_data_size = lcm_in.getEncodedSize();
    uint8_t *buf = (uint8_t *)malloc (max_data_size);
    if (!buf) return -1;
    int32_t data_size = lcm_in.encode(buf, 0, max_data_size);
    if (data_size < 0) {
        free (buf);
        ERROR ("unable to encode %s!", filename);
        return data_size;
    }

    size_t numel=0;
    int64_t fingerprint = lcm_in.getHash();
    numel += fwrite (&fingerprint, sizeof fingerprint, 1, stream);
    numel += fwrite (&data_size, sizeof data_size, 1, stream);
    numel += fwrite (buf, data_size, 1, stream);
    
    fclose (stream);
    if (numel != 3) {
        free (buf);
        ERROR ("unable to write data to %s!", filename);
        return -2;
    }

    free (buf);
    return (sizeof fingerprint + sizeof data_size + data_size);
    
    
}


/**
 * @}
 */
#endif // __PERLS_PVN_UTIL_H__
