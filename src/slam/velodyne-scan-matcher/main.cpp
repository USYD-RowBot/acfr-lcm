#include <iostream>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/error.h"
#include "perls-common/units.h"

#include "velodyne_scan_matcher.h"

#define VELODYNE_DATA_DELAY_CIRC_SIZE 10 // about a second worth of data at 10hz.
#define VELODYNE_DATA_CACHE_SIZE 10 // trade off between having to pull from disk and memory requirements

using namespace std;

void
circ_free_velodyne_data(void *user, void *p) {
    perllcm_velodyne_laser_return_collection_t *lrc = (perllcm_velodyne_laser_return_collection_t *)p;
    velodyne_free_laser_return_collection (lrc);
}

static void
velodyne_free_laser_return_collection_wrapper (void *value)
{
    perllcm_velodyne_laser_return_collection_t *lrc = (perllcm_velodyne_laser_return_collection_t *)value;
    velodyne_free_laser_return_collection (lrc);
}

static void
gq_free (gpointer element)
{
    queue_element_t *qe = (queue_element_t *) element;
    
    switch (qe->type) {
        case QE_DROP_SCAN:
            delete (perllcm::isam_vlink_t *) qe->msg;
        break;
        case QE_PLINK:
            delete (perllcm::isam_plink_t *) qe->msg;        
        break;
        default:
            ERROR ("Unknown queue element type !");
    }
    free (qe);
}

static gint
gq_sort (gconstpointer a, gconstpointer b, gpointer user_data)
{
    const queue_element_t *A = (queue_element_t *)a;
    const queue_element_t *B = (queue_element_t *)b;
    
    if (A->utime > B->utime){
        return 1;
    } else if (A->utime == B->utime){
                
        if (A->type != B->type) {
            
            if (A->type == QE_DROP_SCAN && B->type == QE_PLINK) { // drop scans before plinsk (very important)
                return -1;
            } else if (A->type == QE_PLINK && B->type == QE_DROP_SCAN) {
                return 1;
            } 
            
            return 0; // unspecified case
        } else {
            return 0;
        }
    } else {
        return -1;
    }
}

void
VelodyneScanMatcher::process_queues (void) {
    
    int64_t timeout = 1e5;
    GTimeVal end_time;
    g_get_current_time (&end_time);
    g_time_val_add (&end_time, timeout);
    g_async_queue_sort (gq, &gq_sort, NULL);
    queue_element_t *qe = (queue_element_t *)g_async_queue_timed_pop (gq, &end_time);
    
    if (qe != NULL) {
        
        switch (qe->type) {
            case QE_DROP_SCAN: {
                perllcm::heartbeat_t *hb = (perllcm::heartbeat_t *)qe->msg;
                process_drop_node (hb->utime);
                break;      
            }
            case QE_PLINK:{
                perllcm::isam_plink_t *plink = (perllcm::isam_plink_t *)qe->msg;
                process_plink (plink);
                break;
            }
        }
    }
    
}

void
VelodyneScanMatcher::process_drop_node (int64_t utime) {


    std::cout << "[main]\t\tProcessing drop scan requested at " << utime << std::endl;

    // find nearest scann in circular buffer, motion compensate it based
    // based on velocities and add it to the cache    
    int idx_closest = 0; 
    int64_t utime_dist_min = 1e5; // half of one tenth of a second
    bool any_negative = false;
    
    //because the timestamps of scans start at the first laser poit it is possible that
    //we get a drop node request before we have a scan close to it. therefore
    //if we notice that the requested utime is later (>) than all avalible scans we should wait 5/100 ~ 1/10 sec and try again
    g_mutex_lock (circ_mutex);
    bool retry = true;
    while (retry) {
    
        int len = bot_ptr_circular_size(velodyne_data_delay_circ);

        if (0 == len){
            g_mutex_unlock (circ_mutex);
            return;
        }

        any_negative = false;
        idx_closest = 0; 
        utime_dist_min = 1e5;
        for (int cidx = 0; cidx < len; cidx++) {
    
            perllcm_velodyne_laser_return_collection_t *lrc = (perllcm_velodyne_laser_return_collection_t *)
                                                                bot_ptr_circular_index(velodyne_data_delay_circ, cidx);
                                                    
            int64_t utime_dist = (utime - lrc->utime);
            if (abs (utime_dist) < utime_dist_min) {
                utime_dist_min = utime_dist;
                idx_closest = cidx;
            }
            if (utime_dist < 0) {
                any_negative = true;
            }
        }
        
        if (utime_dist_min == (int64_t) 1e5 && !any_negative) {
            g_mutex_unlock (circ_mutex);
            timeutil_usleep (2e5);
            g_mutex_lock (circ_mutex);
            retry = true;
        } else {
            retry = false;
        }
        
    }
    
    if (utime_dist_min == (int64_t) 1e5 && any_negative) {
        ERROR ("ERROR: No scans within one %lf for velodyne data!", (double)(1e5/1e6));
        g_mutex_unlock (circ_mutex);
        return;
    }
    
    perllcm_velodyne_laser_return_collection_t *lrc = (perllcm_velodyne_laser_return_collection_t *)
                                                        bot_ptr_circular_index (velodyne_data_delay_circ, idx_closest);
    
    // TODO motion compensate lrc to this timestamp
    // should keep local buffer of poses if you want
    lrc->utime = utime;
    
    // write to file
    char filename[PATH_MAX];
    snprintf (filename, sizeof filename, "%s/%ld.lrc", logdir, lrc->utime);
    int32_t ret = LCMU_FWRITE (filename, lrc, perllcm_velodyne_laser_return_collection_t);
    if (ret < 0)
        ERROR ("couldn't write %s to disk!", filename);
    
    // add lrc to cache
    cache_push (velodyne_data_cache, lrc->utime, perllcm_velodyne_laser_return_collection_t_copy (lrc));
    g_mutex_unlock (circ_mutex);

}

void
VelodyneScanMatcher::process_plink (perllcm::isam_plink_t *plink) {
    
    std::cout << "[main]\t\tProcessing plink" << std::endl;
    
    // scan i
    perllcm_velodyne_laser_return_collection_t *lrc_i = (perllcm_velodyne_laser_return_collection_t *)
                                                         cache_pop (velodyne_data_cache, plink->utime_i);
    if (!lrc_i) {
        char filename[PATH_MAX];
        snprintf (filename, sizeof filename, "%s/%ld.lrc", logdir, plink->utime_i);
        int32_t ret = LCMU_FREAD (filename, &lrc_i, perllcm_velodyne_laser_return_collection_t);
        if (ret < 0) {
            printf ("Couldn't read %s from disk! Skipping Link. ret=%d \n", filename, ret);
            return;
        } else 
            cache_push (velodyne_data_cache, plink->utime_i, lrc_i);
    }
    
    // scan j
    perllcm_velodyne_laser_return_collection_t *lrc_j = (perllcm_velodyne_laser_return_collection_t *)
                                                         cache_pop (velodyne_data_cache, plink->utime_j);
    if (!lrc_j) {
        char filename[PATH_MAX];
        snprintf (filename, sizeof filename, "%s/%ld.lrc", logdir, plink->utime_j);
        int32_t ret = LCMU_FREAD (filename, &lrc_j, perllcm_velodyne_laser_return_collection_t);
        if (ret < 0) {
            printf ("Couldn't read %s from disk! Skipping Link. ret=%d \n", filename, ret);
            return;
        } else 
            cache_push (velodyne_data_cache, plink->utime_j, lrc_j);
    }
    
    // warn if number of concurrent pool threads is unable to keep up
    gint ntasks = g_thread_pool_unprocessed (t_pool_data.pool);
    if (ntasks >= g_thread_pool_get_max_threads (t_pool_data.pool)) {
        printf ("Warning: number of unprocessed tasks (%u) exceeds queue (%u)\n",
                ntasks, g_thread_pool_get_max_threads (t_pool_data.pool));
    }
    
    // fire off pool thread
    if (lrc_i && lrc_j) {
        pool_data_t *pdata = (pool_data_t *)malloc (sizeof (*pdata));
        pdata->plink = *plink;
        pdata->lrc_i   = perllcm_velodyne_laser_return_collection_t_copy (lrc_i);
        pdata->lrc_j   = perllcm_velodyne_laser_return_collection_t_copy (lrc_j);
        pdata->visualize = visualize;
        pdata->vis_t_data = &vis_t_data;
        pdata->icp_params = icp_params;
        pdata->gicp_mutex = gicp_mutex;
        pdata->channel_vlink = strdup(channel_isam_vlink);
        pdata->mahal_dist_t = mahal_dist_t;
        //memcpy (pdata->x_vs, x_vs, 6*sizeof (double));
        memcpy (pdata->R, R, 6*6*sizeof (double));
        g_thread_pool_push (t_pool_data.pool, pdata, NULL);
    }
}


VelodyneScanMatcher::VelodyneScanMatcher () {

    // initalize
    is_done = 0;
    g_thread_init(NULL);
    
    gq = g_async_queue_new_full (&gq_free);

    // open param cfg file
    param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        std::cout << "could not find " << BOTU_PARAM_DEFAULT_CFG << std::endl;
    }
    std::cout << "Opened Param File [" << BOTU_PARAM_DEFAULT_CFG << "]" << std::endl;
    
    // read from param file
    channel_isam_vlink = bot_param_get_str_or_fail (param, "isamServer.lcm_channels.VLINK_CHANNEL");
    
    velodyne_data_delay_circ = bot_ptr_circular_new (VELODYNE_DATA_DELAY_CIRC_SIZE,
                                                     circ_free_velodyne_data, NULL);
    
    velodyne_data_cache = cache_new (VELODYNE_DATA_CACHE_SIZE, NULL, &velodyne_free_laser_return_collection_wrapper);
    
    logdir = bot_param_get_str_or_fail (param, "velodyne-scan-matcher.logdir");
    
    gsl_matrix_view R_v = gsl_matrix_view_array (R, 6, 6);
    botu_param_read_covariance (&R_v.matrix, param, (char *)"velodyne-scan-matcher.R");
    
    mahal_dist_t = 0.9;
    bot_param_get_double (param, "velodyne-scan-matcher.mahal_dist_t", &mahal_dist_t);

    // check lcm connection
    if(!lcm.good()) {
        is_done = 1;
        ERROR ("lcm_create () failed!");
    }
    
    // helper thread pool
    t_pool_data.pool = g_thread_pool_new ((GFunc)scan_match_thread_f, &t_pool_data, POOL_THREADS_MAX, 1, NULL);
    
    visualize = 0;
    bot_param_get_int (param, "velodyne-scan-matcher.visualize", &visualize);
    if (visualize) {
        // kick off visualizer thread
        vis_t_data.vis_mutex = g_mutex_new();
        vis_thread = g_thread_create((GThreadFunc)vis_thread_f, &vis_t_data, TRUE, NULL);
        
    }

    if (bot_param_has_key (param, "velodyne-scan-matcher.gicp")) {
        icp_params.use_gicp = 1;
        icp_params.use_icp = 0;
        icp_params.max_iters = 10;
        bot_param_get_int (param, "velodyne-scan-matcher.gicp.max_iters", &icp_params.max_iters);
        icp_params.d_max = 3;
        bot_param_get_double (param, "velodyne-scan-matcher.gicp.d_max", &icp_params.d_max);
        icp_params.gicp_epsilon = 0.001;
        bot_param_get_double (param, "velodyne-scan-matcher.gicp.epsilon", &icp_params.gicp_epsilon);
        icp_params.debug = 0;
        bot_param_get_int (param, "velodyne-scan-matcher.gicp.debug", &icp_params.debug);
    } else if (bot_param_has_key (param, "velodyne-scan-matcher.icp")) {
        icp_params.use_gicp = 0;
        icp_params.use_icp = 1;    
        icp_params.max_iters = 10;
        bot_param_get_int (param, "velodyne-scan-matcher.icp.max_iters", &icp_params.max_iters);
        icp_params.d_max = 3;
        bot_param_get_double (param, "velodyne-scan-matcher.icp.d_max", &icp_params.d_max);
        icp_params.debug = 0;
        bot_param_get_int (param, "velodyne-scan-matcher.icp.debug", &icp_params.debug);
    } else {
        // default usg gicp    
        icp_params.use_gicp = 1;
        icp_params.use_icp = 0;
        icp_params.max_iters = 10;
        icp_params.d_max = 3;
        icp_params.gicp_epsilon = 0.001;
        icp_params.debug = 0;
    }

    gicp_mutex = g_mutex_new ();
    circ_mutex = g_mutex_new ();
    
    // kick off interface thread
    interf_t_data.lcm = &lcm;
    interf_t_data.param = param;
    interf_t_data.gq = gq;
    interf_t_data.velodyne_data_delay_circ = velodyne_data_delay_circ;
    interf_t_data.circ_mutex = circ_mutex;
    interf_t_data.done = &is_done;
    interf_thread = g_thread_create((GThreadFunc)interf_thread_f, &interf_t_data, TRUE, NULL);
    
}


VelodyneScanMatcher::~VelodyneScanMatcher () {
    
    cache_destroy (velodyne_data_cache);
    bot_ptr_circular_destroy (velodyne_data_delay_circ);
    if (visualize) {
        g_mutex_free (vis_t_data.vis_mutex);
    }
    g_mutex_free (gicp_mutex);
    g_mutex_free (circ_mutex);
}


// ----------------------------------------------------------------------------
// Main Loop
// ----------------------------------------------------------------------------
void
VelodyneScanMatcher::run () {

    while (!is_done) {
        process_queues ();      
    }
    
}

VelodyneScanMatcher *g_vsm;
void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t) {
    std::cout << "Sigint caught. Quitting ..." << std::endl;
    if (g_vsm->is_done ) {
        std::cout << "Goodbye" << std::endl;
        exit (EXIT_FAILURE);
    } 
    else
        g_vsm->is_done = 1;
}

int
main (int argc, char *argv[])
{
    
    fasttrig_init ();

    VelodyneScanMatcher vsm = VelodyneScanMatcher::VelodyneScanMatcher ();
    g_vsm = &vsm;
    
    // install custom signal handler
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    // kick off the manager
    vsm.run ();
    
    return 0;
    
}
