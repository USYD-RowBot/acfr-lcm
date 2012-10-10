#ifndef PERLS_VELODYNE_SCAN_MATCHER_H
#define PERLS_VELODYNE_SCAN_MATCHER_H

#include <deque>

#include <glib.h>

#include <lcm/lcm-cpp.hpp>

#include "perls-sensors/velodyne.h"
#include "perls-common/cache.h"
#include "perls-math/gsl_util.h"

#include "perls-lcmtypes++/perllcm/position_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_plink_collection_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_plink_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_vlink_t.hpp"
#include "perls-lcmtypes++/perllcm/pose3d_t.hpp"
#include "perls-lcmtypes++/senlcm/velodyne_t.hpp"

#include "interf_thread.h"
#include "vis_thread.h"
#include "scan_match_thread.h"

#define POOL_THREADS_MAX 2

using namespace std;

class VelodyneScanMatcher {
    private:
        lcm::LCM lcm;   
        BotParam *param;
        
        // buffer to catch the last couple of scans so that when we get a drop
        // node message we can go back in time and find the closest one
        BotPtrCircular   *velodyne_data_delay_circ;
        GMutex *circ_mutex;
        
        // cache holds full scans that correspond to nodes we might want to match against
        cache_t *velodyne_data_cache;
        
        // queue of drop node and plink commands
        GAsyncQueue *gq;
        
        char *channel_isam_vlink;
        
        int visualize;
        GThread *vis_thread;
        vis_thread_data_t vis_t_data;
        
        GThread *interf_thread;
        interf_thread_data_t interf_t_data;
        
        thread_pool_data_t t_pool_data;
        
        char *logdir;
        double mahal_dist_t;  // mahalanobis distance threshold
        double R[6*6];  // measurement uncertianty from config file.
        
        std::deque< int64_t > drop_scan_queue;
        
        icp_params_t icp_params;
        // hack for now, GICP breaks when certian functions are called at the same time
        // even when they are called on a local instance of the gicp class
        // not getting the most out of multi-threading when this is used
        // hopefully when gicp is in pcl we wont have to do this
        GMutex *gicp_mutex;
        
        void
        process_queues (void);
        
        void
        process_plink (perllcm::isam_plink_t *plink);
        
        void
        process_drop_node (int64_t utime);
    
    public:
        int is_done;      

        VelodyneScanMatcher ();
        ~VelodyneScanMatcher ();
        
        void
        run ();
        
};

#endif /* PERLS_VELODYNE_SCAN_MATCHER_H */
