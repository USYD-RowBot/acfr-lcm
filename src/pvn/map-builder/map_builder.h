#ifndef __PERLS_PVN_MAP_BUILDER_H__
#define __PERLS_PVN_MAP_BUILDER_H__

#include <iostream>
#include <fstream>
#include <algorithm>
#include <map>
#include <set>
#include <deque>


#include <bot_param/param_client.h>
//#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <lcm/lcm-cpp.hpp>
#include <opencv/cv.h>

#include "perls-lcmtypes++/bot_core/image_t.hpp"
#include "perls-lcmtypes++/perllcm/van_feature_collection_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_vis_map_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_vis_vocab_t.hpp"
#include "perls-lcmtypes++/perllcm/viewer_image_pccs_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_eview_map_t.hpp"
#include "perls-lcmtypes++/perllcm/pose3d_collection_t.hpp"
#include "perls-lcmtypes/perllcm_velodyne_laser_return_collection_t.h"
#include "perls-lcmtypes/perllcm_pose3d_collection_t.h"
#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_pvn_laser_map_t.h"

#include "perls-math/gsl_util_array.h"
#include "perls-math/gsl_util_vector.h"
#include "perls-math/gsl_util_matrix.h"
#include "perls-math/gsl_util_blas.h"
#include "perls-math/ssc.h"
#include "perls-math/gsl_util.h"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/magic.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/units.h"
#include "perls-vision/ladybug3_util.h"
#include "perls-vision/camera.h"
#include "perls-vision/opencv_util.h"
#include "perls-vision/botimage.h"
#include "perls-common/cache.h"

#include "perls-sensors/velodyne.h"
#include "perls-pvn/pvn_util.h"
#include "perls-pvn/pvn_image_data.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define LB3_NUM_CAMS 6
#define FEAT_2ND_NN_THRESH 0.8
#define USE_RANSAC 0
#define USE_PCCS 1
#define USE_RANSAC_EVIEW 1
#define USE_PCCS_EVIEW 1
#define RANSAC_MIN_PUTATIVE_CORRS 8
#define RANSAC_MIN_INLIER_CORRS 6
#define MEDOR_MIN_PUTATIVE_CORRS 8
#define MEDOR_MIN_INLIER_CORRS 6
#define MERG_MAX_DIST 0.4 // meters
#define MIN_FEATS_PER_CLUST 200
#define VOCAB_MIN_USEC_DIST 1e5 //1e6
#define VOCAB_MAX_FEATURES  5e6 //10e6 = 5GB worth the maximum number of features to collect before trying to cluster
#define VOCAB_MAX_SIZE 100e3

#define NEIGHBORHOOD_T_NEW_MIN 10 // in meters
#define NEIGHBORHOOD_R_NEW_MIN 360*DTOR // in radians
#define NEIGHBORHOOD_T_COMP_MIN 2
#define NEIGHBORHOOD_R_COMP_MIN NEIGHBORHOOD_R_NEW_MIN
#define MERGE_EXEMPLAR_MIN_INLIERS 75
//#define NEW_EXEMPLAR_MAX_INLIERS 25
#define MAX_EXEMPLAR_PER_NEIGHBORHOOD 4
#define CLT_MAP_PRIOR_ALPHA 2.0
#define EVIEW_BACKUP_EVERY 200



static inline bool
file_exists(const char *filename)
{
  ifstream ifile(filename);
  return ifile;
}

class ExemplarMatchData {
  private:
        
  public:    
    vector<int> f_inliers[LB3_NUM_CAMS];
    vector<int> map_inliers[LB3_NUM_CAMS];   
};

class PVNMapBuilder {
    private:
        lcm::LCM lcm;
        
        BotParam *param;
        getopt_t *gopt;
        
        // config
        char *map_name;             // name of the current map that we are working on
        char *scan_dir;             // path to directory where scans are stored
        char *graph_dir;            // directory where isam graph is saved (graph.isam and pc.dat in this folder)
        char *map_dir;              // path to directory where maps are stored
        char *las_dir;              // path to directory navteq .las files are stored
        double las_subsample_pct;   // percent to subsample navteq las files
        int use_camera[6];          // mask of which cameras to use
        int lm_force_lite;          // convert full laser returns to lite laser returns
        int lm_every_nth_return;    // include only ever nth return
        bool lm_color;              // use color data from images
        char *image_dir;            // directory with .pgr images
        int eview_backup;
        int64_t eview_end_at;
        
        //double x_vs_laser[6];
        
        BotGPSLinearize *llxy;  //struct containing gps origin used to convert between local xy
        double org_alt;
        
        // SEED MAP BUILD FUNCTIONS---------------------------------------------
        void
        run_build_seed_map ();
        
        void
        process_las (char *filename, std::ofstream *fgraph);
        
        // LAS MAP BUILD FUNCTIONS----------------------------------------------
        void
        run_build_laser_map ();
        
        
        // EVIEW MAP BUILD FUNCTIONS--------------------------------------------
        void
        run_build_eview_map ();
        
        perllcm::pvn_eview_map_t
        eview_map_new (perllcm::pose3d_collection_t pc, char *fmap);
        
        perllcm::pvn_eview_map_t
        eview_map_update (perllcm::pose3d_collection_t pc, char *fmap);
        
        //int
        //eview_map_matching (perllcm::pvn_eview_map_neighborhood_t *mn,
        //                    ImageData *id, CamConfig *camconf, perllcm::pose3d_t *p_v,
        //                    vector<int> &inlier_cnts);
        
        perllcm_velodyne_laser_return_collection_t *
        get_close_lrcs (perllcm::pose3d_collection_t *pc, int idx, cache_t *lrc_data_cache);
        
        //void
        //assoc_laser_features (perllcm_velodyne_laser_return_collection_t *lrc,
        //                      ImageData *id, CamConfig *camconf, perllcm::pose3d_t *p_v);
        

        // EVIEW PROCESS FUNCTIONS ---------------------------------------------
        void
        run_eview_process (char *fname);
        
        // VOCAB GENERATION FUNCTIONS ------------------------------------------
        void
        run_build_vocab ();
        
        // CHOW LIU TREE FUNCTIONS ---------------------------------------------
        void
        run_build_chow_liu_tree ();
        
        // SPARSIFICATION FUNCTIONS --------------------------------------------
        void
        run_sparsify_map ();

    public:
        
        bool is_done;
        
        PVNMapBuilder (int argc, char *argv[]);
        ~PVNMapBuilder ();
        
        void
        run ();        
};


#endif // __PERLS_PVN_MAP_BUILDER_H__
