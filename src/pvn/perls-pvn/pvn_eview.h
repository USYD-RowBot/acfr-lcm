#ifndef __PERLS_PVN_EVIEW_H__
#define __PERLS_PVN_EVIEW_H__

#include <iostream>
#include <vector>
#include <set>
#include <map>

#include <bot_param/param_client.h>

#include "perls-lcmtypes++/perllcm/pvn_eview_map_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_eview_map_neighborhood_t.hpp"


/**
 * @defgroup PerlsPVNEView Persistent Visual Navigation Exemplar View Map Helper Functions
 * @brief PVN helper library.
 * @ingroup PerlsPVN
 * @include: perls-pvn/pvn_eview.h
 *
 * @{
 */

using namespace std;

/**
 *  @brief calculate beta prior coefs
 */
void
pvn_eview_beta_prior_coefs(double mode_target, double  strength, double *a, double *b);


/**
 *  @brief loads environmental conditions from config file
 */
perllcm::pvn_conditions_t
pvn_eview_load_conditions (BotParam *param, char *base_key);


/**
 * @brief Generate a list of observations based on the current run data
 *
 */
map<int64_t, bool>
pvn_eview_run_data_2_obs (perllcm::pvn_eview_map_match_data_t *rd, set<int64_t> *var_ids);

/**
 * @brief Generate a list of observations based on the current run data
 *
 */
map<int64_t, float>
pvn_eview_run_data_2_obs_soft (perllcm::pvn_eview_map_match_data_t *rd, set<int64_t> *var_ids);

/**
 *  @brief updates the run data for an eview map
 */
void
pvn_eview_update_match_data (perllcm::pvn_eview_map_match_data_t *md,
                             perllcm::pvn_eview_map_neighborhood_t *mn,
                             vector<int8_t> match_success, vector<int> inlier_cnts, 
                             int64_t obs_utime, int obs_num_feats,
                             vector<int> exemplar_num_feats,
                             vector<float> reproj_error,
                             vector<bool> mask = std::vector<bool>(0,0));

/**
 *  @brief updates the run data predict data for an eview map
 */
void
pvn_eview_update_predict_data (perllcm::pvn_eview_map_predict_data_collection_t *pdc,
                               perllcm::pvn_eview_map_neighborhood_t *mn,
                               vector<int8_t> match_success, vector<int> inlier_cnts, 
                               map<int64_t, float> P, int64_t obs_utime,
                               perllcm::pvn_conditions_t obs_conds, int obs_num_feats,
                               vector<int> exemplar_num_feats,
                               vector<float> reproj_error);

/**
 *  @brief determine if a match is a success
 */
vector<int8_t>
pvn_eview_calc_match_success_v (vector<int> inlier_cnts, vector<float> reproj_error,
                                int min_inliers, float max_reproj_error);

/**
 *  @brief determine if a match is a success
 */
int8_t
pvn_eview_calc_match_success(int inlier_cnt, float reproj_error,
                             int min_inliers, float max_reproj_error);


/**
 * @}
 */
#endif // __PERLS_PVN_UTIL_H__