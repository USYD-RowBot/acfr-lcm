#ifndef __PERLS_PVN_TMPCOR_H__
#define __PERLS_PVN_TMPCOR_H__


#include <iostream>
#include <fstream>
#include <algorithm>
#include <map>
#include <set>
#include <deque>


#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>

#include "perls-lcmtypes++/bot_core/image_t.hpp"
#include "perls-lcmtypes++/perllcm/van_feature_collection_t.hpp"

#include "perls-math/ssc.h"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/magic.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/units.h"
#include "perls-vision/camera.h"

#include "perls-sensors/velodyne.h"
#include "perls-pvn/pvn_util.h"
#include "perls-pvn/pvn_eview.h"
#include "perls-pvn/pvn_image_data.h"
#include "perls-pvn/pvn_chowliutree.h"

#define DO_PLOT 0
#define SIFT_2ND_NN_THRESH 0.8

// for GR dataset
#define EXEMPLAR_MATCH_REPROJ_ERROR_THRESH 5.0 //(5.0 better pr but very slightly worse ROC than 10.0)
#define EXEMPLAR_MATCH_INLIER_THRESH 20
#define EXEMPLAR_MAX_PER_NEIGHBORHOOD 9
#define CLT_PRIOR_STRENGTH 4.0
#define CLT_PRIOR_MODE_TARGET 0.20
#define RESIZE_IMAGE_SIZE {640, 480}

// for nyc dataset
//#define EXEMPLAR_MATCH_REPROJ_ERROR_THRESH 5.0 
//#define EXEMPLAR_MATCH_INLIER_THRESH 15
//#define EXEMPLAR_MAX_PER_NEIGHBORHOOD 9
//#define CLT_PRIOR_STRENGTH 4.0
//#define CLT_PRIOR_MODE_TARGET 0.10
//#define RESIZE_IMAGE_SIZE {320, 240}


#define CLT_USE_SOFT_EVIDENCE 0

class PVNTempCor {
    private:
        lcm::LCM lcm;
        
        BotParam *param;
        getopt_t *gopt;
        
        int every_nth;
        int random_step;
        
        void
        run_webcam_process (char *dir);
        
        void
        run_preprocess (char *dir);
        
        perllcm::pvn_conditions_t
        read_conditions (std::string file);
        
        vector<int64_t> 
        read_utimes (char *dir);
        
        int
        count_num_cams (char *dir);
        
        void
        update_predict_data_full (perllcm::pvn_eview_map_t *eview_map,
                                  int irun, int ii_cam,
                                  map<int64_t, float> P, int64_t obs_utime,
                                  perllcm::pvn_conditions_t obs_conds, char *dir);
        
        vector<int>
        feat_match_neighborhood (perllcm::pvn_eview_map_neighborhood_t *mn,
                                 PVNImageData *id,
                                 vector<int> *exemplar_num_feats,
                                 vector<float> *reproj_error);
        
        
        void
        update_last_match (perllcm::pvn_eview_map_neighborhood_t &mn, // the existing map neighborhood
                           perllcm::pvn_eview_map_t &eview_map,       // the map (has the match_data)
                           perllcm::pvn_eview_map_exemplar_t mne);
        
        // winston churchil and paul newman style update
        // add examples whenever current view does not match original
        void
        simple_wcpn_update (perllcm::pvn_eview_map_neighborhood_t &mn,
                            perllcm::pvn_eview_map_t &eview_map,
                            perllcm::pvn_eview_map_exemplar_t mne);
        
        // forget least reacently used exemplar
        void
        simple_forget_update (perllcm::pvn_eview_map_neighborhood_t &mn,
                              perllcm::pvn_eview_map_t &eview_map,
                              perllcm::pvn_eview_map_exemplar_t mne);
        
        //// kurt konolige and james bowman style updated
        //// based on visually connected clusters delete LRU exemplar in clusters first
        //// then delete LRU singelton clusters
        //void
        //kdjb_update (perllcm::pvn_eview_map_neighborhood_t &mn,
        //             perllcm::pvn_eview_map_t &eview_map,
        //             perllcm::pvn_eview_map_exemplar_t mne);
        
        
        

    public:
        
        bool is_done;
        
        PVNTempCor (int argc, char *argv[]);
        ~PVNTempCor ();
        
        void
        run ();        
};

#endif // __PERLS_PVN_TMPCOR_H__
