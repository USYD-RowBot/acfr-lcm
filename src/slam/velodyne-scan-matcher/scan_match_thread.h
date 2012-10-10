#ifndef PERLS_VELODYNE_SCAN_MATCHER_SCAN_MATCH_THREAD_H
#define PERLS_VELODYNE_SCAN_MATCHER_SCAN_MATCH_THREAD_H

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <gicp.h> //for segals code
#include <glib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>
#include <lcm/lcm-cpp.hpp>

#include "perls-sensors/velodyne.h"

#include "perls-common/error.h"

#include "perls-math/gsl_util.h"
#include "perls-math/ssc.h"

#include "perls-lcmtypes++/perllcm/isam_plink_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_vlink_t.hpp"

#include "vis_thread.h"

typedef struct _icp_params_t icp_params_t;
struct _icp_params_t {
    uint8_t use_icp;
    uint8_t use_gicp;
    
    int max_iters;
    double d_max;
    int debug;
    double gicp_epsilon;
};

typedef struct _thread_pool_data_t thread_pool_data_t;
struct _thread_pool_data_t {
    GThreadPool *pool;     // thread pool
    GMutex *vis_mutex;
};

typedef struct pool_data pool_data_t;
struct pool_data {
    perllcm::isam_plink_t plink;
    perllcm_velodyne_laser_return_collection_t *lrc_i;
    perllcm_velodyne_laser_return_collection_t *lrc_j;
    int visualize;
    vis_thread_data_t *vis_t_data;
    icp_params_t icp_params;
    char *channel_vlink;
    double R[6*6];
    double mahal_dist_t;
    GMutex *gicp_mutex;
};

void
pooldata_free (pool_data_t *pdata);

void *
scan_match_thread_f (void *user_data);

#endif
