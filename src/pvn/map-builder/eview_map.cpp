#include <pcl/point_types.h>
#include <bot_lcmgl_client/lcmgl.h>

#include "perls-lcmtypes/perllcm_van_feature_attr_siftgpu_t.h"

#include "perls-pvn/pvn_chowliutree.h"
#include "perls-pvn/combination.hpp"

#include "map_builder.h"


#define LRC_DATA_CACHE_SIZE 5

using namespace std;
using namespace Eigen;


//perllcm_velodyne_laser_return_collection_t *
//PVNMapBuilder::get_close_lrcs (perllcm::pose3d_collection_t *pc, int idx, cache_t *lrc_data_cache) {
//    
//    // current pose
//    perllcm::pose3d_t *po = &(pc->pose[idx]);
//    const double min_range = 1.0;
//    
//    // sequential indicies should be in temporal order
//    std::vector<int> inds_include; 
//    const int num_ba = 5; // number of scans before or after current to include
//    const double max_dist = 2; // maximum distance in meters for scans to consider
//    for (int i=0; i<pc->npose; i++) {
//        
//        if (i == idx)
//            continue;
//        
//        if (i < idx-num_ba || i > idx+num_ba) { //outside temporal window check distance
//            perllcm::pose3d_t *pi = &(pc->pose[i]);
//            double dist = sqrt (pow(po->mu[0] - pi->mu[0], 2) +
//                                pow(po->mu[1] - pi->mu[1], 2));
//            if (dist < max_dist)
//                inds_include.push_back(i);
//        } else {
//            inds_include.push_back(i);
//        }
//    }
//    
//    // use vectors to temporarily store xyz intensity
//    vector <double> x;
//    vector <double> y;
//    vector <double> z;
//    vector <uint8_t> intensity;
//    
//    // load scan associated with idx
//    int64_t utime_o = pc->pose[idx].utime;
//    perllcm_velodyne_laser_return_collection_t *lrc = (perllcm_velodyne_laser_return_collection_t *)
//                                                        cache_pop (lrc_data_cache, utime_o);
//    if (NULL == lrc) {                                                    
//        char filename[PATH_MAX];
//        snprintf (filename, sizeof filename, "%s/%ld.lrc", scan_dir, utime_o);
//        int32_t ret = LCMU_FREAD (filename, &lrc, perllcm_velodyne_laser_return_collection_t);
//        if (ret < 0) {
//            cout << "[laser map]\tCouldn't read " << filename << " from disk. ret=" << ret << endl;
//            return NULL;
//        } else
//            cache_push (lrc_data_cache, utime_o, lrc);
//    }
//
//    double x_w_so[6];
//    double x_vs[6];
//    memcpy(x_vs, lrc->x_vs, 6*sizeof (double));
//    ssc_head2tail (x_w_so, NULL, pc->pose[idx].mu, x_vs);
//    
//    // copy features over
//    for (int j=0; j<lrc->num_lr; j++) {
//        
//        if (check_min_range (lrc->laser_returns[j].xyz, min_range)) {
//            x.push_back (lrc->laser_returns[j].xyz[0]); 
//            y.push_back (lrc->laser_returns[j].xyz[1]);
//            z.push_back (lrc->laser_returns[j].xyz[2]);
//            intensity.push_back (lrc->laser_returns[j].intensity);
//        }
//    }
//    for (int j=0; j<lrc->num_lrl; j++) {
//        if (check_min_range (lrc->laser_returns_lite[j].xyz, min_range)) {
//            x.push_back (lrc->laser_returns_lite[j].xyz[0]); 
//            y.push_back (lrc->laser_returns_lite[j].xyz[1]);
//            z.push_back (lrc->laser_returns_lite[j].xyz[2]);
//            intensity.push_back (lrc->laser_returns_lite[j].intensity);
//        }
//    }
//
//    // load the laser scans
//    for (size_t i=0; i<inds_include.size(); i++) {
//        
//        int64_t utime = pc->pose[inds_include[i]].utime;
//        perllcm_velodyne_laser_return_collection_t *lrc = (perllcm_velodyne_laser_return_collection_t *)
//                                                           cache_pop (lrc_data_cache, utime);
//        if (NULL == lrc) {
//            char filename[PATH_MAX];
//            snprintf (filename, sizeof filename, "%s/%ld.lrc", scan_dir, utime);
//            int32_t ret = LCMU_FREAD (filename, &lrc, perllcm_velodyne_laser_return_collection_t);
//            if (ret < 0) {
//                cout << "[laser map]\tCouldn't read " << filename << " from disk. ret=" << ret << endl;
//                continue;
//            }
//            else
//                cache_push (lrc_data_cache, utime, lrc);
//        }
//
//        double x_w_si[6];
//        ssc_head2tail (x_w_si, NULL, pc->pose[inds_include[i]].mu, lrc->x_vs);
//        double x_so_si[6];
//        ssc_tail2tail (x_so_si, NULL, x_w_so, x_w_si);
//        
//        for (int j=0; j<lrc->num_lr; j++) {
//            
//            if (check_min_range (lrc->laser_returns[j].xyz, min_range)) {
//                double x_so_lr[6] = {0};
//                double x_si_lr[6] = {lrc->laser_returns[j].xyz[0], 
//                                     lrc->laser_returns[j].xyz[1],
//                                     lrc->laser_returns[j].xyz[2], 0,0,0};
//                ssc_head2tail (x_so_lr, NULL, x_so_si, x_si_lr);
//                
//                x.push_back (x_so_lr[0]); 
//                y.push_back (x_so_lr[1]);
//                z.push_back (x_so_lr[2]);
//                intensity.push_back (lrc->laser_returns[j].intensity);
//            }
//        }
//        for (int j=0; j<lrc->num_lrl; j++) {
//            
//            if (check_min_range (lrc->laser_returns_lite[j].xyz, min_range)) {
//                double x_so_lr[6] = {0};
//                double x_si_lr[6] = {lrc->laser_returns_lite[j].xyz[0], 
//                                     lrc->laser_returns_lite[j].xyz[1],
//                                     lrc->laser_returns_lite[j].xyz[2], 0,0,0};
//                ssc_head2tail (x_so_lr, NULL, x_so_si, x_si_lr);
//                
//                x.push_back (x_so_lr[0]); 
//                y.push_back (x_so_lr[1]);
//                z.push_back (x_so_lr[2]);
//                intensity.push_back (lrc->laser_returns_lite[j].intensity);
//            }
//        }
//    }
//    
//    // pack lrc to return
//    perllcm_velodyne_laser_return_collection_t *lrc_out =
//                                (perllcm_velodyne_laser_return_collection_t *)
//                                 calloc (1, sizeof (*lrc_out));
//
//    lrc_out->num_lr = 0;
//    lrc_out->laser_returns = NULL;
//    lrc_out->num_lrl = x.size();
//    lrc_out->laser_returns_lite = (perllcm_velodyne_laser_return_lite_t *)
//                                   calloc (lrc_out->num_lrl, sizeof (*(lrc_out->laser_returns_lite)));
//    
//    lrc_out->utime = po->utime;
//    memcpy (lrc_out->x_vs, x_vs, 6*sizeof(double));
//    memcpy (lrc_out->pose, po->mu, 6*sizeof(double));
//    lrc_out->has_pose = 1;
//    for (int i=0; i<lrc_out->num_lrl; i++) {
//        lrc_out->laser_returns_lite[i].xyz[0] = x[i];
//        lrc_out->laser_returns_lite[i].xyz[1] = y[i];
//        lrc_out->laser_returns_lite[i].xyz[2] = z[i];
//        lrc_out->laser_returns_lite[i].intensity = intensity[i];
//    }
//    
//    ////// lcmgl debug code
//    ////bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm.getUnderlyingLCM(), "DEBUG");
//    ////
//    ////lcmglPointSize(10);
//    ////lcmglBegin(LCMGL_POINTS);
//    ////lcmglColor3f(0.0, 0.0, 1.0);
//    ////for (size_t i=0; i<inds_include.size(); i++) {
//    ////    lcmglVertex3d(pc->pose[inds_include[i]].mu[0],
//    ////                  pc->pose[inds_include[i]].mu[1],
//    ////                  pc->pose[inds_include[i]].mu[2]);
//    ////}
//    ////lcmglColor3f(1.0, 0.0, 0.0);
//    ////lcmglVertex3d(po->mu[0], po->mu[1], po->mu[2]);
//    ////lcmglEnd();
//    ////
//    ////lcmglPointSize(2);
//    ////lcmglBegin(LCMGL_POINTS);
//    ////lcmglColor3f(0.0, 1.0, 1.0);
//    ////for (size_t i=0; i<x.size(); i++) {
//    ////    lcmglVertex3d(x[i], y[i], z[i]);
//    ////}
//    ////lcmglEnd();
//    ////
//    ////bot_lcmgl_switch_buffer(lcmgl);
//    ////bot_lcmgl_destroy(lcmgl);
//
//    return lrc_out;
//}
        
//void
//PVNMapBuilder::assoc_laser_features (perllcm_velodyne_laser_return_collection_t *lrc,
//                                     ImageData *id, CamConfig *camconf, perllcm::pose3d_t *p_v) {
//    
//    if (lrc->num_lr != 0)
//        ERROR ("Expecting only lite returns at this point!");
//
//    GThreadPool *pool = g_thread_pool_new ((GFunc)assoc_laser_features_thread, NULL, LB3_NUM_CAMS, 1, NULL);
//        
//    // for each camera
//    for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
//        
//        // kick off a thread for this camera
//        f2lrc_pool_data_t *pdata = (f2lrc_pool_data_t *)malloc (sizeof (*pdata));
//        
//        // find this x_velodyne_camera for this camera
//        ssc_head2tail (pdata->x_vel_ci, NULL, camconf->x_vel_h, camconf->x_hs[icam]);
//        
//        // find x_world_camera_i
//        double x_w_h[6]; // world to lb3 head
//        ssc_head2tail (x_w_h, NULL, p_v->mu, camconf->x_vh);
//        ssc_head2tail (pdata->x_w_ci, NULL, x_w_h, camconf->x_hs[icam]);
//                
//        pdata->lrc = perllcm_velodyne_laser_return_collection_t_copy (lrc);
//        pdata->camconf = camconf;
//        pdata->icam = icam;
//        pdata->id = id;
//        pdata->lcm = &lcm;
//        
//        g_thread_pool_push (pool, pdata, NULL);
//        
//    }
//    
//    // blocks untill all tasks are done
//    g_thread_pool_free(pool, false, true);
//    
//    // lcmgl view
//    //bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm.getUnderlyingLCM(), "DEBUG");
//    //lcmglPushMatrix();
//    //lcmglRotated(180, 1, 0, 0);
//    //lcmglPointSize(1);
//    //lcmglBegin(LCMGL_POINTS);
//    //lcmglColor3f(0.0, 1.0, 1.0);
//    //for (int i=0; i<lrc->num_lrl; i++) {
//    //    lcmglVertex3d(lrc->laser_returns_lite[i].xyz[0],
//    //                  lrc->laser_returns_lite[i].xyz[1],
//    //                  lrc->laser_returns_lite[i].xyz[2]);
//    //}
//    //lcmglEnd();
//    //lcmglPopMatrix();
//    //for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
//    //    double x_vel_ci[6];
//    //    double x_ci_vel[6];
//    //    ssc_head2tail (x_vel_ci, NULL, camconf->x_vel_h, camconf->x_hs[icam]);
//    //    ssc_inverse (x_ci_vel, NULL, x_vel_ci);
//    //    
//    //    lcmglPushMatrix();
//    //    lcmglRotated(180, 1, 0, 0);
//    //    lcmglRotated(x_vel_ci[5]*RTOD, 0, 0, 1);
//    //    lcmglRotated(x_vel_ci[4]*RTOD, 0, 1, 0);
//    //    lcmglRotated(x_vel_ci[3]*RTOD, 1, 0, 0);
//    //    lcmglPointSize(5);
//    //    lcmglBegin(LCMGL_POINTS);
//    //    float rcolor[4] = {0};
//    //    bot_color_util_rand_color (rcolor, 1.0, 0.1);
//    //    lcmglColor3fv(rcolor);
//    //    for (int i=0; i<id->feats_3d[icam].npts; i++) {
//    //        lcmglVertex3d(id->feats_3d[icam].x_c[i], id->feats_3d[icam].y_c[i], id->feats_3d[icam].z_c[i]);
//    //    }
//    //    lcmglEnd();
//    //    lcmglPopMatrix();
//    //}
//    //bot_lcmgl_switch_buffer(lcmgl);
//    //bot_lcmgl_destroy(lcmgl);
//    
//}

//typedef struct evmm_pool_data evmm_pool_data_t;
//struct evmm_pool_data {
//    perllcm::pvn_eview_map_exemplar_t *mne;
//    CamConfig *camconf;
//    int icam;
//    int iexemplar;
//    ImageData *id;
//    ExemplarMatchData *emd;
//    lcm::LCM *lcm;
//    double x_e_ci[6];
//    double S_e_ci[6*6];
//};
//
//void
//evmm_pooldata_free (evmm_pool_data_t *pdata)
//{
//    free (pdata);
//}
//
//static void
//velodyne_free_laser_return_collection_wrapper (void *value)
//{
//    perllcm_velodyne_laser_return_collection_t *lrc = (perllcm_velodyne_laser_return_collection_t *)value;
//    velodyne_free_laser_return_collection (lrc);
//}
//
//
//bool
//in_existing_neighborhood (double test_pose[6], double neighborhood_pose[6], double t_min, double r_min) {
//
//    double t = sqrt(pow (test_pose[0] - neighborhood_pose[0], 2) +
//                    pow (test_pose[1] - neighborhood_pose[1], 2) +
//                    pow (test_pose[2] - neighborhood_pose[2], 2));
//    double r = abs(bot_mod2pi(pow (test_pose[5] - neighborhood_pose[5], 2)));
//    
//    if (t < t_min && r < r_min)
//        return true;
//    else
//        return false;
//    
//}
//
//
//vector<int>
//find_neighborhoods (perllcm::pose3d_collection_t *pc, char *graph_isam, 
//                           double t_min, double r_min) {
//    
//    
//    vector<int> inds_out;
//    
//    vector< pair<int, int> > nf_pairs;
//    for (int i=0; i<pc->npose; i++) {
//        nf_pairs.push_back( pair<int, int> (0,i) );
//    }
//
//    ifstream fisam;
//    fisam.open (graph_isam);
//    string line;
//    if (fisam.is_open()) {
//        
//        while ( fisam.good() ) {
//            getline (fisam, line);
//            if (!line.compare(0, 20, "Pose3d_Pose3d_Factor")) {
//                int ia, ib;
//                sscanf (line.c_str(), "Pose3d_Pose3d_Factor %d %d", &ia, &ib);
//                nf_pairs[ia].first++;
//                nf_pairs[ib].first++;
//            }
//        }
//        fisam.close();
//        
//    } else {
//        ERROR ("Failed to open %s!", graph_isam);
//        
//    }
//    
//    // sort in order of decending number of links
//    sort (nf_pairs.begin(), nf_pairs.end());
//    reverse (nf_pairs.begin(), nf_pairs.end());
//    //for (size_t j=0; j<nf_pairs.size(); j++){
//    //    cout << nf_pairs[j].first << " -- "  << nf_pairs[j].second << endl;
//    //}
//    
//    // start with most connected node as neighborhood and add new nodes 
//    vector< vector<double> > centers;
//    for (int i=0; i<pc->npose; i++) {
//        int ind = nf_pairs[i].second;
//        
//        bool in_existing = false;
//        for (size_t j=0; j<centers.size(); j++) {
//            in_existing |= in_existing_neighborhood (pc->pose[ind].mu, &(centers[j][0]), t_min, r_min);
//        }
//        
//        if (!in_existing) {
//            vector<double> tmp (6, 0);
//            copy(pc->pose[ind].mu, pc->pose[ind].mu+6, tmp.begin());    
//            centers.push_back (tmp);
//            
//            inds_out.push_back (ind);
//        }
//        
//    }
//    
//    ////// debug output
//    ////lcm::LCM lcm;
//    ////bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm.getUnderlyingLCM(), "DEBUG");
//    ////lcmglPushMatrix();
//    ////lcmglRotated(180, 1, 0, 0);
//    ////
//    ////for (int i=0; i<pc->npose; i++) {
//    ////    lcmglPointSize(2*nf_pairs[i].first);
//    ////    lcmglBegin(LCMGL_POINTS);
//    ////    lcmglColor3f(0.0, 0.0, 1.0);
//    ////    int j = nf_pairs[i].second;
//    ////    lcmglVertex3d(pc->pose[j].mu[0], pc->pose[j].mu[1], pc->pose[j].mu[2]+0.5);
//    ////    lcmglEnd();
//    ////}
//    ////
//    ////lcmglPointSize(5);
//    ////lcmglBegin(LCMGL_POINTS);
//    ////lcmglColor3f(1.0, 0.0, 0.0);
//    ////for (size_t i=0; i<inds_out.size(); i++) {
//    ////    int j = inds_out[i];
//    ////    lcmglVertex3d(pc.pose[j].mu[0], pc.pose[j].mu[1], pc.pose[j].mu[2]-0.5);  
//    ////}
//    ////lcmglEnd();
//    ////
//    ////lcmglPopMatrix();
//    ////bot_lcmgl_switch_buffer(lcmgl);
//    ////bot_lcmgl_destroy(lcmgl);    
//    ////
//    return inds_out;
//    
//}
//
//
//perllcm::pvn_eview_map_exemplar_t
//new_exemplar (int64_t utime, perllcm::pose3d_t x_n_e, CamConfig *camconf, ImageData *id, BotParam *param) {
//    
//    perllcm::pvn_eview_map_exemplar_t mne;
//    
//    mne.utime = utime;
//    mne.utime_last_merge = utime;
//    char base_key[1024] = {"pvn-map-builder.conditions"};
//    mne.conditions = pvnu_load_conditions (param, base_key);
//    
//    mne.x_n_e = x_n_e;
//    mne.npts = 0;
//    mne.keylen = id->feats_3d[0].keylen;
//    mne.max_xyz[0] = -1e6; mne.max_xyz[1] = -1e6; mne.max_xyz[2] = -1e6;
//    mne.min_xyz[0] = 1e6; mne.min_xyz[1] = 1e6; mne.min_xyz[2] = 1e6;
//    
//    for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
//        
//        double x_v_ci[6] = {0}; // vehicle to to lb3 head
//        ssc_head2tail (x_v_ci, NULL, camconf->x_vh, camconf->x_hs[icam]);
//        
//        Feats3D *f3d = &(id->feats_3d[icam]);
//        
//        mne.npts += f3d->npts;
//        for (int j=0; j<f3d->npts; j++) {
//            double x_ci_f[6] = {f3d->x_c[j], f3d->y_c[j], f3d->z_c[j], 0,0,0};
//            double x_v_f[6] = {0};
//            ssc_head2tail (x_v_f, NULL, x_v_ci, x_ci_f);
//            mne.x.push_back(x_v_f[0]);
//            mne.y.push_back(x_v_f[1]);
//            mne.z.push_back(x_v_f[2]);
//            
//            if (x_v_f[0] > mne.max_xyz[0]) mne.max_xyz[0] = x_v_f[0];
//            if (x_v_f[1] > mne.max_xyz[1]) mne.max_xyz[1] = x_v_f[1];
//            if (x_v_f[2] > mne.max_xyz[2]) mne.max_xyz[2] = x_v_f[2];
//            if (x_v_f[0] < mne.min_xyz[0]) mne.min_xyz[0] = x_v_f[0];
//            if (x_v_f[1] < mne.min_xyz[1]) mne.min_xyz[1] = x_v_f[1];
//            if (x_v_f[2] < mne.min_xyz[2]) mne.min_xyz[2] = x_v_f[2];
//            
//        }
//
//        mne.keys.insert (mne.keys.end(), f3d->keys.begin(), f3d->keys.end());
//        mne.key_scale.insert (mne.key_scale.end(), f3d->key_scale.begin(), f3d->key_scale.end());
//        mne.key_orientation.insert (mne.key_orientation.end(), f3d->key_orientation.begin(), f3d->key_orientation.end());
//        vector <int> tmp (f3d->npts, 1);
//        mne.vocab_id.insert(mne.vocab_id.end(), tmp.begin(), tmp.end());
//        mne.build_key_update_cnt.insert(mne.build_key_update_cnt.end(), tmp.begin(), tmp.end());
//        mne.build_xyz_update_cnt.insert(mne.build_xyz_update_cnt.end(), tmp.begin(), tmp.end());
//        
//    }
//    
//    return mne;
//    
//}
//
//void
//new_neighborhood (perllcm::pvn_eview_map_t *eview_map, perllcm::pose3d_t *p,
//                  CamConfig *camconf, ImageData *id, BotParam *param,
//                  int irun) {
//    
//    // first exemplar in new neighborhood has same pose as neighborhood
//    perllcm::pose3d_t x_n_e;
//    memset (x_n_e.mu, 0, 6*sizeof (double));
//    memset (x_n_e.Sigma, 0, 6*6*sizeof (double));
//    
//    perllcm::pvn_eview_map_exemplar_t mne;
//    mne = new_exemplar (p->utime, x_n_e, camconf, id, param);
//    
//    perllcm::pvn_eview_map_neighborhood_t mn;
//    mn.pose = *p;
//    mn.ne = 1;
//    mn.exemplars.push_back (mne);
//    
//    eview_map->neighborhoods.push_back(mn);
//    eview_map->nn++;
//    eview_map->match_data[irun].exemplar_utimes.push_back(p->utime);
//    eview_map->match_data[irun].observations.push_back(1);
//    eview_map->match_data[irun].n++;
//}
//
//perllcm::pvn_eview_map_t
//PVNMapBuilder::eview_map_new (perllcm::pose3d_collection_t pc, char *fmap) {
//    
//    
//    int64_t tic;
//    
//    perllcm::pvn_eview_map_t eview_map;
//    
//    cache_t *lrc_data_cache;
//    lrc_data_cache = cache_new (LRC_DATA_CACHE_SIZE, NULL, &velodyne_free_laser_return_collection_wrapper);
//    
//    // load the camera channel names
//    CamConfig camconf = CamConfig (param, use_camera);
//    
//    cout << "[eview map]\tInit new map: " << fmap << endl;
//    
//    // initialize map
//    eview_map.orglat = llxy->lat0_deg;
//    eview_map.orglon = llxy->lon0_deg;
//    eview_map.orgalt = org_alt;
//    eview_map.nn = 0;
//    eview_map.neighborhoods.resize(eview_map.nn);
//    eview_map.nr = 1;
//    eview_map.match_data.resize(eview_map.nr);
//    int irun = 0;
//    eview_map.match_data[irun].n = 0;
//    eview_map.match_data[irun].exemplar_utimes.resize(0);
//    eview_map.match_data[irun].observations.resize(0);
//    eview_map.predict_data_colls.resize(eview_map.nr);
//    eview_map.predict_data_colls[irun].n = 0;    
//    eview_map.predict_data_colls[irun].predict_data.resize(0);      
//    
//    // find the neighboorhoods
//    tic = timestamp_now();
//    char graph_isam[PATH_MAX];
//    sprintf (graph_isam, "%s/graph.isam", graph_dir);
//    vector<int> neighborhood_inds;
//    neighborhood_inds = find_neighborhoods (&pc, graph_isam, NEIGHBORHOOD_T_NEW_MIN, NEIGHBORHOOD_R_NEW_MIN);
//    cout << "[eview map]\t" << neighborhood_inds.size();
//    cout << " metric neighborhoods found (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//    
//    for (size_t i=0; i<neighborhood_inds.size(); i++) {
//
//        perllcm::pose3d_t *p = &(pc.pose[neighborhood_inds[i]]);
//    
//        cout << "[eview map]\tProcessing node " << p->utime << " " << i+1 << "/" << neighborhood_inds.size();
//        cout <<  " =======================================" << endl;
//        
//        // load images and features
//        tic = timestamp_now();
//        ImageData id = ImageData::ImageData (&camconf, image_dir, use_camera, p->utime);
//        if (id.load (&lcm)) {
//            cout << "[eview map]\tNo image data found, skipping." << endl;
//            continue;
//        }
//        cout << "[eview map]\tLoaded image data (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//        
//        // load nearby laser scans
//        tic = timestamp_now();
//        perllcm_velodyne_laser_return_collection_t *lrc = get_close_lrcs (&pc, neighborhood_inds[i], lrc_data_cache);
//        if (NULL == lrc) {
//            cout << "[vis map]\tNo laser return data found, skipping." << endl;
//            continue;
//        }
//        cout << "[eview map]\tLoaded laser data (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//        
//        // associate features with 3D pcoints
//        tic = timestamp_now();
//        assoc_laser_features (lrc, &id, &camconf, p);
//        cout << "[eview map]\t3D points found for features (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//        perllcm_velodyne_laser_return_collection_t_destroy (lrc);
//            
//        // add new metric neighborhood
//        new_neighborhood (&eview_map, p, &camconf, &id, param, irun);
//        cout << "[eview map]\tAdded new neighborhood to map" << endl;
//    }
//    
//    cache_destroy (lrc_data_cache);
//    
//    return eview_map;       
//}
//
//
//void *
//eview_map_matching_thread (void *user_data) {
//    
//    evmm_pool_data_t *pdata = (evmm_pool_data_t *) user_data;
//    
//    perllcm::pvn_eview_map_exemplar_t *mne = pdata->mne;
//
//    int icam = pdata->icam;
//    int iexemplar = pdata->iexemplar;
//    
//    perllcm::van_feature_collection_t *fc = &(pdata->id->feature_collections[icam]);
//    perllcm::van_feature_t *f_img = &(fc->f[0]);
//    
//    int64_t total_sum=0, project_sum=0, match_sum=0, outlier_reject_sum=0;    
//
//    int64_t tic_total, tic_project, tic_match, tic_outlier_reject;
//    int64_t toc_total, toc_project, toc_match, toc_outlier_reject;
//    
//    tic_total = timestamp_now();
//    // project the exemplar the current frame
//    tic_project = timestamp_now();
//    std::vector < std::vector <float> > xyz;
//    pvnu_exemplar_to_xyz (&xyz, mne);
//    
//    std::vector <int> proj_inds;
//    std::vector < std::vector <float> > uv_map;
//    pvnu_project_to_image (&uv_map, &proj_inds, NULL,
//                           xyz,
//                           pdata->x_e_ci,
//                           pdata->camconf->K[icam],
//                           pdata->camconf->height[icam],
//                           pdata->camconf->width[icam],
//                           100, false);
//    toc_project = timestamp_now();
//    
//    // match features (map to image pccs) -------------------------------------
//    perllcm::van_feature_t f_tmp; perllcm::van_feature_t *f_map = &f_tmp; // so that we have a pointer matching f_img
//    f_map->keylen = f_img->keylen;
//    f_map->npts = (int) proj_inds.size();
//    for (int i=0; i<f_map->npts; i++) {
//        f_map->u.push_back (uv_map[i][0]); 
//        f_map->v.push_back (uv_map[i][1]);
//        f_map->keys.push_back (mne->keys[proj_inds[i]]);
//    }
//    
//    vector< vector<float> > xyz_map_proj;
//    xyz_map_proj.resize (proj_inds.size());
//    for (size_t i=0; i<proj_inds.size(); i++) {
//        xyz_map_proj[i].resize(3);
//        xyz_map_proj[i][0] = mne->x[proj_inds[i]];
//        xyz_map_proj[i][1] = mne->y[proj_inds[i]];
//        xyz_map_proj[i][2] = mne->z[proj_inds[i]];    
//    }
//       
//    perllcm::viewer_image_pccs_t vis_pccs;
//    
//    tic_match = timestamp_now();
//    std::vector<int> sel_map;
//    std::vector<int> sel_img;
//    pvnu_image_map_matching (&sel_map, &sel_img, f_img, f_map, xyz_map_proj,
//                            pdata->x_e_ci, pdata->S_e_ci, pdata->camconf->K[icam].data(),
//                            FEAT_2ND_NN_THRESH, USE_PCCS_EVIEW, &vis_pccs);
//    toc_match = timestamp_now();
//    
//    // plot putative correspondences
//    char pccs_chan[64]= {0};
//    sprintf (pccs_chan, "PCCS_CAM%d", icam);
//    vis_pccs.utime = pdata->id->utime;
//    pdata->lcm->publish (pccs_chan, &vis_pccs);
//    
//    // outlier rejection 
//    tic_outlier_reject = timestamp_now();
//    vector<int> inliers;
//    if (USE_RANSAC_EVIEW && sel_map.size() >= RANSAC_MIN_PUTATIVE_CORRS) {
//        
//        // perform ransac
//        vector<cv::Point3f> XYZp;
//        vector<cv::Point2f> uvp;
//        for (size_t i=0; i<sel_map.size (); i++) {
//        
//            cv::Point3f tmp3 (xyz_map_proj[sel_map[i]][0],
//                              xyz_map_proj[sel_map[i]][1],
//                              xyz_map_proj[sel_map[i]][2]);
//            XYZp.push_back (tmp3);
//            cv::Point2f tmp2 (f_img->u[sel_img[i]], f_img->v[sel_img[i]]);
//            uvp.push_back (tmp2);
//            
//        }
//        cv::Mat cvK (3, 3, CV_64F, pdata->camconf->K[icam].data());
//        vector<double> rvec_out (3, 0), tvec_out (3, 0);
//        vector<double> distcoefs (4, 0);
//        
//        cv::solvePnPRansac(XYZp, uvp, cvK, distcoefs, rvec_out, tvec_out,
//                           false, // use guess
//                           2000, // number of itterations
//                           20.0, // inlier if less that this reprojection error 8.0 default
//                           100, // min inliers count (exit if you get this many inliers)
//                           inliers,
//                           CV_ITERATIVE);
//        // doesn't set these as output
//        // cout << rvec_out[0] << " " << rvec_out[1] << " " << rvec_out[2] << endl;
//        // cout << tvec_out[0] << " " << tvec_out[1] << " " << tvec_out[2] << endl;
//        
//        if (inliers.size() < RANSAC_MIN_INLIER_CORRS)
//            inliers.resize(0);
//    
//    } else if (sel_map.size() >= MEDOR_MIN_PUTATIVE_CORRS) {
//                
//        std::vector < float > u, v, up, vp;
//        for (size_t i=0; i<sel_map.size(); i++) {
//            u.push_back (f_img->u[sel_img[i]]);
//            v.push_back (f_img->v[sel_img[i]]);
//            up.push_back (f_map->u[sel_map[i]]);
//            vp.push_back (f_map->v[sel_map[i]]);
//        }
//        
//        inliers = pvnu_median_outlier_rejection (u, v, up, vp);
//    }
//    toc_outlier_reject = timestamp_now();
//    
//    if (1) { // for visualization
//        perllcm::viewer_image_pccs_t vis_pccs;
//        
//        vis_pccs.utime = pdata->id->utime;
//        
//        vis_pccs.num_corrs = inliers.size();
//        vis_pccs.num_covp = 0;
//        
//        for (int i=0; i<vis_pccs.num_corrs; i++) {
//            vis_pccs.u.push_back (f_img->u[sel_img[inliers[i]]]);
//            vis_pccs.v.push_back (f_img->v[sel_img[inliers[i]]]);
//            vis_pccs.up.push_back (f_map->u[sel_map[inliers[i]]]);
//            vis_pccs.vp.push_back (f_map->v[sel_map[inliers[i]]]);
//        }
//    
//        vis_pccs.type = perllcm::viewer_image_pccs_t::TYPE_INLIER;
//        
//        char pccs_chan[64]= {0};
//        sprintf (pccs_chan, "PCCS_CAM%d", icam);
//        pdata->lcm->publish (pccs_chan, &vis_pccs);
//    }
//    toc_total = timestamp_now();
//    
//    total_sum += (toc_total - tic_total);
//    project_sum += (toc_project - tic_project);
//    match_sum += (toc_match - tic_match);
//    outlier_reject_sum += (toc_outlier_reject - tic_outlier_reject);
// 
//    // set output
//    for (size_t i=0; i<inliers.size(); i++) {
//        pdata->emd[iexemplar].f_inliers[icam].push_back (sel_img[inliers[i]]);
//        pdata->emd[iexemplar].map_inliers[icam].push_back (proj_inds[sel_map[inliers[i]]]);
//    }     
//    
//    cout << "[eview map]\tImage to exemplar feature matching for cam " << icam << ", exemplar " << iexemplar;
//    cout << ". (" << total_sum/1e6 << " sec)" << endl;
//    cout << "\t\t\t" << f_map->npts << ":" << f_img->npts << " (fmap:fimage), ";
//    cout << inliers.size() << "/" << sel_map.size() << " inliers." << endl;
//    
//    evmm_pooldata_free (pdata);
//    return NULL;
//}
//
//int
//PVNMapBuilder::eview_map_matching (perllcm::pvn_eview_map_neighborhood_t *mn,
//                                   ImageData *id, CamConfig *camconf, perllcm::pose3d_t *p_v,
//                                   vector<int> &inlier_cnts) {
//    
//    GThreadPool *pool = g_thread_pool_new ((GFunc)eview_map_matching_thread, NULL, LB3_NUM_CAMS, 1, NULL);
//    
//    ExemplarMatchData emd[mn->ne];
//        
//    // for each camera
//    for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
//    
//        for (int iexemplar=0; iexemplar<mn->ne; iexemplar++) {
//            
//            perllcm::pvn_eview_map_exemplar_t *mne = &(mn->exemplars[iexemplar]);        
//            
//            // kick off a thread for this camera
//            evmm_pool_data_t *pdata = (evmm_pool_data_t *)malloc (sizeof (*pdata));
//            
//            double x_w_e[6];
//            Matrix <double, 6, 12, RowMajor > J0;
//            ssc_head2tail (x_w_e, J0.data(), mn->pose.mu, mne->x_n_e.mu);
//            Matrix <double, 6, 6, RowMajor > S_w_n (mn->pose.Sigma);
//            Matrix <double, 6, 6, RowMajor > S_n_e (mne->x_n_e.Sigma);
//            Matrix <double, 6, 6, RowMajor > S_w_e;
//            S_w_e = J0.topLeftCorner(6,6)*S_w_n*J0.topLeftCorner(6,6).transpose();
//            S_w_e += J0.bottomRightCorner(6,6)*S_n_e*J0.bottomRightCorner(6,6).transpose();
//            
//            double x_e_v[6];
//            Matrix <double, 6, 12, RowMajor > J1;
//            ssc_tail2tail (x_e_v, J1.data(), x_w_e, p_v->mu);
//            Matrix <double, 6, 6, RowMajor > S_w_v (p_v->Sigma);
//            Matrix <double, 6, 6, RowMajor > S_e_v;
//            S_e_v = J0.topLeftCorner(6,6)*S_w_e*J1.topLeftCorner(6,6).transpose();
//            S_e_v += J0.bottomRightCorner(6,6)*S_w_v*J1.bottomRightCorner(6,6).transpose();
//            
//            double x_e_h[6]; // exemplar to lb3 head
//            Matrix <double, 6, 12, RowMajor > J2;
//            ssc_head2tail (x_e_h, J2.data(), x_e_v, camconf->x_vh);
//            Matrix <double, 6, 6, RowMajor > S_e_h;
//            S_e_h = J1.topLeftCorner(6,6)*S_e_v*J2.topLeftCorner(6,6).transpose();
//            
//            Matrix <double, 6, 6, RowMajor > S_e_ci;
//            if (1) { // TMP HACK
//                ssc_head2tail (pdata->x_e_ci, NULL, x_e_h, camconf->x_hs[icam]);
//                // find covariance HACK Because Head to sensor transforms are all near euler singularity
//                Matrix <double, 3, 3, RowMajor > R21;
//                so3_rotxyz (R21.data(), &(camconf->x_hs[icam][3]));
//                Matrix <double, 3, 3, RowMajor > R12  = R21.transpose();
//                S_e_ci.setZero();
//                S_e_ci.topLeftCorner(3,3) = R12 * S_e_h.topLeftCorner(3,3) * R12.transpose();
//                S_e_ci.bottomRightCorner(3,3) = R12 * S_e_h.bottomRightCorner(3,3) * R12.transpose();
//            }
//            else {
//                Matrix <double, 6, 12, RowMajor > J3;
//                ssc_head2tail (pdata->x_e_ci, J3.data(), x_e_h, camconf->x_hs[icam]);
//                S_e_ci = J2.topLeftCorner(6,6)*S_e_h*J3.topLeftCorner(6,6).transpose();
//            }
//        
//            memcpy (pdata->S_e_ci, S_e_ci.data(), 6*6*sizeof (double));
//            pdata->mne = mne; // only read in thread
//            pdata->camconf = camconf;
//            pdata->icam = icam;
//            pdata->iexemplar = iexemplar;
//            pdata->id = id;
//            pdata->emd = emd;
//            pdata->lcm = &lcm;
//            
//            g_thread_pool_push (pool, pdata, NULL);
//        }
//    }
//    
//    // blocks untill all tasks are done
//    g_thread_pool_free(pool, false, true);
//    
//    // find the best exemplar
//    
//    int max_inliers = 0;
//    int i_emi = -1;
//    inlier_cnts.resize(mn->ne);
//    for (int iexemplar=0; iexemplar<mn->ne; iexemplar++) {
//        
//        int total_inliers = 0; 
//        for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
//            total_inliers += emd[iexemplar].f_inliers[icam].size();
//        }
//        
//        inlier_cnts[iexemplar] = total_inliers;
//        if (total_inliers > max_inliers) {
//            max_inliers = total_inliers;
//            i_emi = iexemplar;
//        }
//    }
//    
//    if (max_inliers > MERGE_EXEMPLAR_MIN_INLIERS) {
//        cout << "[eview map]\tCurrent view matches exemplar " << i_emi;
//        cout << ", with " << max_inliers << " inliers."<< endl;
//        
//        // set output
//        for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
//            id->f_inliers[icam] = emd[i_emi].f_inliers[icam];
//            id->map_inliers[icam] = emd[i_emi].map_inliers[icam];
//        }
//        
//        return i_emi;
//    } else {
//        cout << "[eview map]\tNo match found adding new exemplar: max inliers = " << max_inliers << endl;
//        return -1;
//    }
//    
//    //// draw map for debugging
//    //bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm.getUnderlyingLCM(), "DEBUG");
//    //lcmglPushMatrix();
//    //lcmglRotated(180, 1, 0, 0);
//    //lcmglPointSize(10);
//    //lcmglColor3f(1.0, 0.0, 0.0);
//    //lcmglBegin(LCMGL_POINTS);
//    //    lcmglVertex3d(p_v->mu[0],
//    //                  p_v->mu[1],
//    //                  p_v->mu[2]);
//    //lcmglEnd();
//    //lcmglPopMatrix();
//    //bot_lcmgl_switch_buffer(lcmgl);
//    //bot_lcmgl_destroy(lcmgl);
//}
//
///*
//void
//merge_into_exemplar (perllcm::pvn_eview_map_exemplar_t *mne, ImageData *id,
//                     CamConfig *camconf, perllcm::pose3d_t *p_v, double x_w_n[6]) {
//            
//    int dist_pass_cnt = 0;
//    int dist_fail_cnt = 0;
//    int key_merge_cnt = 0;
//    int new_cnt = 0;
//    
//    mne->utime_last_merge = p_v->utime;
//    
//    // merge features
//    for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
//        
//        Feats3D *feats_3d = &(id->feats_3d[icam]);
//        vector<int> *f_2_feat3d_inds = &(id->f_2_feat3d_inds[icam]);
//        vector<int> *f_inliers = &(id->f_inliers[icam]);
//        vector<int> *map_inliers = &(id->map_inliers[icam]);
//        perllcm::van_feature_collection_t *fc = &(id->feature_collections[icam]);
//        perllcm::van_feature_t *f = &(fc->f[0]);        
//        
//        for (size_t i=0; i<f_inliers->size(); i++) {
//            
//            int f_ind = (*f_inliers)[i];
//            int f3d_ind = (*f_2_feat3d_inds)[f_ind];
//            int mne_ind = (*map_inliers)[i];
//            
//            if (f3d_ind != -1) { // we have a 3D point for this feature
//                
//                // given points in world frame we want to compare them in the exemplar frame
//                double x_w_f[6] = {feats_3d->x_w[f3d_ind], feats_3d->y_w[f3d_ind], feats_3d->z_w[f3d_ind], 0,0,0};
//                double x_w_e[6] = {0};
//                ssc_head2tail(x_w_e, NULL, x_w_n, mne->x_n_e.mu);
//                double x_e_f[6] = {0};
//                ssc_tail2tail(x_e_f, NULL, x_w_e, x_w_f);
//                
//                double dist = sqrt (pow (x_e_f[0] - mne->x[mne_ind], 2) +
//                                    pow (x_e_f[1] - mne->y[mne_ind], 2) +
//                                    pow (x_e_f[2] - mne->z[mne_ind], 2));
//                
//                if (dist >= MERG_MAX_DIST) {
//                    dist_fail_cnt++;
//                    continue;
//                }
//                dist_pass_cnt++;
//                
//                // merg 3D location (updating average)
//                double n = (double) mne->build_xyz_update_cnt[mne_ind];
//                mne->x[mne_ind] = mne->x[mne_ind]*(n/(n+1.0)) + x_e_f[0]*(1/(n+1.0));
//                mne->y[mne_ind] = mne->y[mne_ind]*(n/(n+1.0)) + x_e_f[1]*(1/(n+1.0));
//                mne->z[mne_ind] = mne->z[mne_ind]*(n/(n+1.0)) + x_e_f[2]*(1/(n+1.0));                       
//                mne->build_xyz_update_cnt[mne_ind]++;
//                
//                // flag (remove) this feat3d so that we dont add it again later
//                (*f_2_feat3d_inds)[f_ind] = -1;
//            } 
//            
//            // merge sift keys (updating average)
//            double n = (double) mne->build_key_update_cnt[mne_ind];
//            for (int j=0; j<mne->keylen; j++) {
//                mne->keys[mne_ind][j] = mne->keys[mne_ind][j]*(n/(n+1.0)) + f->keys[f_ind][j]*(1/(n+1.0));
//            }
//            
//            perllcm_van_feature_attr_siftgpu_t *attr = (perllcm_van_feature_attr_siftgpu_t *)malloc (sizeof (*attr));
//            perllcm_van_feature_attr_siftgpu_t_decode (&f->attr[0], 0, f->attrsize, attr);
//            mne->key_scale[mne_ind] = mne->key_scale[mne_ind]*(n/(n+1.0)) + attr->s[f_ind]*(1/(n+1.0));
//            mne->key_orientation[mne_ind] = mne->key_orientation[mne_ind]*(n/(n+1.0)) + attr->o[f_ind]*(1/(n+1.0));
//            perllcm_van_feature_attr_siftgpu_t_destroy (attr);
//            
//            mne->build_key_update_cnt[mne_ind]++;
//            key_merge_cnt++;
//        }
//       
//    }
//    
//    // remove any features that haven't been matched at least once
//    // could spatialy delay but need to do something because map size becomes too large
//    int original_cnt = mne->npts;
//    int kept_cnt = pvnu_remove_unmatched_features (mne);
//    
//    //// add new features from thoes that have an associated 3D poisiton ---------
//    for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
//      
//        Feats3D *feats_3d = &(id->feats_3d[icam]);
//        vector<int> *f_2_feat3d_inds = &(id->f_2_feat3d_inds[icam]);
//        perllcm::van_feature_collection_t *fc = &(id->feature_collections[icam]);
//        perllcm::van_feature_t *f = &(fc->f[0]);
//        
//        Feats3D f3d_add;
//        f3d_add.keylen = f->keylen;
//   
//        for (size_t i=0; i<f_2_feat3d_inds->size(); i++) {
//    
//            int f3d_ind = (*f_2_feat3d_inds)[i];
//            if (f3d_ind == -1)
//                continue;
//        
//            // given points in world frame we want to compare them in the exemplar frame
//            double x_w_f[6] = {feats_3d->x_w[f3d_ind], feats_3d->y_w[f3d_ind], feats_3d->z_w[f3d_ind], 0,0,0};
//            double x_w_e[6] = {0};
//            ssc_head2tail(x_w_e, NULL, x_w_n, mne->x_n_e.mu);
//            double x_e_f[6] = {0};
//            ssc_tail2tail(x_e_f, NULL, x_w_e, x_w_f);
//            
//            mne->x.push_back(x_e_f[0]);
//            mne->y.push_back(x_e_f[1]);
//            mne->z.push_back(x_e_f[2]);
//            
//            if (x_e_f[0] > mne->max_xyz[0]) mne->max_xyz[0] = x_e_f[0];
//            if (x_e_f[1] > mne->max_xyz[1]) mne->max_xyz[1] = x_e_f[1];
//            if (x_e_f[2] > mne->max_xyz[2]) mne->max_xyz[2] = x_e_f[2];
//            if (x_e_f[0] < mne->min_xyz[0]) mne->min_xyz[0] = x_e_f[0];
//            if (x_e_f[1] < mne->min_xyz[1]) mne->min_xyz[1] = x_e_f[1];
//            if (x_e_f[2] < mne->min_xyz[2]) mne->min_xyz[2] = x_e_f[2];
//                
//            mne->vocab_id.push_back(1);
//            mne->build_key_update_cnt.push_back(1);
//            mne->build_xyz_update_cnt.push_back(1);
//            
//            mne->keys.push_back (feats_3d->keys[f3d_ind]);
//            mne->key_scale.push_back (feats_3d->key_scale[f3d_ind]);
//            mne->key_orientation.push_back (feats_3d->key_orientation[f3d_ind]);
//            mne->npts++;
//            new_cnt++;
//        }
//        
//    }
//    
//    cout << "[eview map]\tExemplar updated" << endl;
//    cout << "\t\t\t" << dist_pass_cnt << "/" << dist_pass_cnt+dist_fail_cnt << " passed dist check for merge." << endl;
//    cout << "\t\t\t" << key_merge_cnt << " sift keys merged." << endl;
//    cout << "\t\t\t" << original_cnt-kept_cnt << " unmatched sift keys removed." << endl;
//    cout << "\t\t\t" << kept_cnt << " matched sift keys kept." << endl;
//    cout << "\t\t\t" << new_cnt << " new unmatched features added." << endl;
//
//}
//*/
//
//
//
//int
//forget_old_exemplars (perllcm::pvn_eview_map_t *eview_map, int max_epn) {
//    
//    int forgot_cnt = 0;
//    
//    for (int i=0; i<eview_map->nn; i++) {
//        
//        perllcm::pvn_eview_map_neighborhood_t *mn = &(eview_map->neighborhoods[i]);
//        
//        int check_clique_size = mn->ne;
//        while (mn->ne > max_epn) {
//            
//            // build adjacency matrix
//            MatrixXd adj(mn->ne,mn->ne);
//            adj.setZero();
//            for (int row=0; row<mn->ne; row++) {
//                for (int col=row; col<mn->ne; col++) {
//                
//                    int64_t utime_row = mn->exemplars[row].utime;
//                    int64_t utime_col = mn->exemplars[col].utime;
//                    
//                    bool co_observed = false;
//                    for (int j=0; j<eview_map->nr && !co_observed; j++) { // loop over runs
//
//                        perllcm::pvn_eview_map_match_data_t *rd = &(eview_map->match_data[j]);
//
//                        bool row_observed = false;
//                        bool col_observed = false;
//
//                        for (int k=0; k<rd->n && !co_observed; k++) { // loop over data in run
//
//                            if ((rd->exemplar_utimes[k] == utime_row) && ((int8_t)rd->observations[k] == 1)) 
//                                row_observed |= true;
//
//                            if ((rd->exemplar_utimes[k] == utime_col) && ((int8_t)rd->observations[k] == 1)) 
//                                col_observed |= true;
//
//                            if (row_observed && col_observed) 
//                                co_observed = true;
//                        }    
//                    }
//                    
//                    if (co_observed) {
//                       adj(row, col) = 1;
//                       adj(col, row) = 1;
//                    }
//
//                }
//            }
//        
//            // start at full set and work backwards
//            int n = mn->ne;
//            int r = check_clique_size;
//            std::vector<int> v_int(n);
//            for (int j = 0; j < n; ++j)
//                v_int[j] = j; 
//           
//            bool is_clique;
//            vector <int> clique;
//            do {
//                is_clique = true;
//                clique.clear ();
//            
//                
//                // loop over nodes in this set
//                for (int j = 0; j < r && is_clique; ++j) {
//                    
//                    // make sure connect to everyting currently in clique
//                    for (size_t k=0; k<clique.size() && is_clique; k++) {
//                        if (adj(v_int[j], clique[k]) == 0)
//                            is_clique = false;
//                    }
//                    clique.push_back (v_int[j]);
//                }
//              
//            } while (!is_clique && boost::next_combination(v_int.begin(), v_int.begin() + r, v_int.end()));
//            
//            
//            if (is_clique) {
//                           
//                // find lease recently used in clique
//                int i_lru = clique[0];
//                int64_t utime_lru = mn->exemplars[clique[0]].utime_last_merge;
//                int64_t utime_add_lru = mn->exemplars[clique[0]].utime;
//                for (size_t j=1; j<clique.size(); j++) {
//                    if (mn->exemplars[clique[j]].utime_last_merge < utime_lru) {
//                        i_lru = clique[j];
//                        utime_lru = mn->exemplars[clique[j]].utime_last_merge;
//                        utime_add_lru  = mn->exemplars[clique[j]].utime;
//                    } else if (mn->exemplars[clique[j]].utime_last_merge == utime_lru && // use time the same
//                               mn->exemplars[clique[j]].utime > utime_add_lru) { // but insert time newer
//                        i_lru = clique[j];
//                        utime_lru = mn->exemplars[clique[j]].utime_last_merge;
//                        utime_add_lru  = mn->exemplars[clique[j]].utime;
//                    }
//                }
//                
//                cout << "Adjacency Matrix" << endl << adj << endl;
//                cout << "Removing " << i_lru << " from clique: ";
//                for(size_t j=0; j<clique.size(); j++)
//                    cout << clique[j] << " ";
//                cout << endl;
//                
//                // remove data from match_data for this exemplar
//                int64_t utime_remove = mn->exemplars[i_lru].utime;
//                for (int j=0; j<eview_map->nr; j++) { // loop over runs
//                    perllcm::pvn_eview_map_match_data_t *rd = &(eview_map->match_data[j]);
//                    
//                    
//                    int32_t n_new = 0;
//                    vector<int64_t> exemplar_utimes_new;
//                    vector<int8_t> observations_new;
//                    for (int k=0; k<rd->n; k++) { // loop over data in run
//                        if (rd->exemplar_utimes[k] != utime_remove) {
//                            n_new++;
//                            exemplar_utimes_new.push_back (rd->exemplar_utimes[k]);
//                            observations_new.push_back (rd->observations[k]);
//                        }
//                    }
//                    rd->n = n_new;
//                    rd->exemplar_utimes = exemplar_utimes_new;
//                    rd->observations = observations_new;
//                    
//                }
//                
//                mn->exemplars.erase (mn->exemplars.begin() + i_lru);
//                mn->ne--;
//            
//                forgot_cnt++;
//                
//            }
//            check_clique_size--;
//
//        }
//    }
//    
//    return forgot_cnt;
//}
//
//
//perllcm::pvn_eview_map_t
//PVNMapBuilder::eview_map_update (perllcm::pose3d_collection_t pc, char *fmap) {
//
//    perllcm::pvn_eview_map_t eview_map;
//    
//    int64_t tic;
//
//    int32_t ret = pvnu_lcm_fread <perllcm::pvn_eview_map_t> (fmap, &eview_map);
//    if (ret < 0) {
//        cout << "[eview map]\tCouldn't read " << fmap << " from disk. ret=" << ret << endl;
//        exit (-1);
//    }
//    cout << "[eview map]\tLoaded existing map: " << fmap << endl;
//    
//    // starting from a backup
//    int start_i = 0;
//    int irun = 0;
//    PVNCLTree clt;
//    if (eview_backup > 0) {
//        
//        start_i = eview_backup;
//        
//        // build the chow-liu tree from previous run data, dont include this current run
//        tic = timestamp_now();
//        vector<perllcm::pvn_eview_map_match_data_t> match_data;
//        for (int j=0; j<(eview_map.nr-1) ; j++)
//            match_data.push_back (eview_map.match_data[j]);
//        
//        clt.make (match_data, 0, CLT_MAP_PRIOR_ALPHA);
//        cout << "[cl tree]\tBuilt Chow Liu Tree: ( " << (timestamp_now() - tic)/1e6 << " sec.)" << endl;    
//        
//        irun = eview_map.nr-1;        
//        cout << "[eview map]\tContinuing run number: " << eview_map.nr << endl;
//    
//    } else { // starting a new run
//        
//        start_i = 0;
//        
//        // build the chow-liu tree from previous run data
//        tic = timestamp_now();
//        clt.make (eview_map.match_data, 0, CLT_MAP_PRIOR_ALPHA);
//        cout << "[cl tree]\tBuilt Chow Liu Tree: ( " << (timestamp_now() - tic)/1e6 << " sec.)" << endl;    
//        
//        // this is a new run
//        irun = eview_map.nr;
//        eview_map.nr++;
//        eview_map.match_data.resize(eview_map.nr);
//        eview_map.match_data[irun].n = 0;
//        eview_map.match_data[irun].exemplar_utimes.resize(0);
//        eview_map.match_data[irun].observations.resize(0);
//        eview_map.predict_data_colls.resize(eview_map.nr);
//        eview_map.predict_data_colls[irun].n = 0;    
//        eview_map.predict_data_colls[irun].predict_data.resize(0);      
//        cout << "[eview map]\tStarting run number: " << eview_map.nr << endl;
//    }
//   
//    // load the camera channel names
//    CamConfig camconf = CamConfig (param, use_camera); 
//    
//    cache_t *lrc_data_cache;
//    lrc_data_cache = cache_new (LRC_DATA_CACHE_SIZE, NULL, &velodyne_free_laser_return_collection_wrapper);
//    
//    int last_backup = 0;
//    for (int i=start_i; i<pc.npose; i++) {
//
//        perllcm::pose3d_t *p = &(pc.pose[i]);
//        
//        if (eview_end_at > 0 && p->utime > eview_end_at) {
//            cout << "[eview map]\tDone Processing " << p->utime << " > " << eview_end_at << endl;
//            break;
//        }
//    
//        cout << "[eview map]\tProcessing node " << p->utime << " " << i+1 << "/" << pc.npose;
//        cout <<  " =======================================" << endl;
//        cout << "[eview map]\tLast backup i = " << last_backup << endl;
//        
//	// tasks
//        bool in_existing = false; // add as a new neighborhood
//        int compare_with = -1; // compare with current neighborhood
//        for (int j=0; j<eview_map.nn; j++) {
//            in_existing |= in_existing_neighborhood (p->mu, eview_map.neighborhoods[j].pose.mu, 
//                                                     NEIGHBORHOOD_T_NEW_MIN, NEIGHBORHOOD_R_NEW_MIN);
//            
//            if (in_existing_neighborhood (p->mu, eview_map.neighborhoods[j].pose.mu,
//                                          NEIGHBORHOOD_T_COMP_MIN, NEIGHBORHOOD_R_COMP_MIN)) {
//                compare_with = j;
//            }
//        }
//
//        // if we dont want to compare or add as a new neighborhood skip.
//        if (in_existing && compare_with == -1) {
//            cout << "[eview map]\tSkipping " << endl;
//            continue;
//        }
//        
//        // load images and features
//        tic = timestamp_now();
//        ImageData id = ImageData::ImageData (&camconf, image_dir, use_camera, p->utime);
//        if (id.load (&lcm)) {
//            cout << "[eview map]\tNo image data found, skipping." << endl;
//            continue;
//        }
//        cout << "[eview map]\tLoaded image data (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//        
//        // load nearby laser scans
//        tic = timestamp_now();
//        perllcm_velodyne_laser_return_collection_t *lrc = get_close_lrcs (&pc, i, lrc_data_cache);
//        if (NULL == lrc) {
//            cout << "[vis map]\tNo laser return data found, skipping." << endl;
//            continue;
//        }
//        cout << "[eview map]\tLoaded laser data (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//        
//        // associate features with 3D pcoints
//        tic = timestamp_now();
//        assoc_laser_features (lrc, &id, &camconf, p);
//        cout << "[eview map]\t3D points found for features (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//        perllcm_velodyne_laser_return_collection_t_destroy (lrc);
//        
//        if(!in_existing) {
//            
//            // add new metric neighborhood
//            new_neighborhood (&eview_map, p, &camconf, &id, param, irun);
//            
//            cout << "[eview map]\tAdded new neighborhood to map" << endl;
//            
//        } else if (compare_with != -1) {
//            
//            perllcm::pvn_eview_map_neighborhood_t *mn = &(eview_map.neighborhoods[compare_with]);
//            
//            // predict probability of matching exemplars in this neighboorhood
//            tic = timestamp_now();
//            map<int64_t, bool> obs = pvnu_make_run_data_make_obs (&(eview_map.match_data[irun]), NULL);
//            map<int64_t, float> P = clt.sumProduct (obs);
//            cout << "[eview map]\tPredict visual matching for exemplars (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//
//            // visual matching
//            tic = timestamp_now();
//            vector<int> inlier_cnts;
//            eview_map_matching (mn, &id, &camconf, p, inlier_cnts);
//            cout << "[eview map]\tVisual matching between features and neighborhood (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//            
//            // update last used time
//            for (int j=0; j<mn->ne; j++) {
//                if (inlier_cnts[j] > MERGE_EXEMPLAR_MIN_INLIERS)
//                    mn->exemplars[j].utime_last_merge = p->utime;
//            }
//            
//            // update the observation data
//            pvnu_update_run_data (&(eview_map.match_data[irun]), mn, inlier_cnts, MERGE_EXEMPLAR_MIN_INLIERS);
//            
//            // add an entry in the predict data
//            perllcm::pvn_eview_map_predict_data_t pd;
//            pd.utime = p->utime;
//            pd.ne = mn->ne;
//            for (int j=0; j<pd.ne; j++) {
//                
//                int64_t utime_e = mn->exemplars[j].utime;
//                pd.exemplar_utimes.push_back(utime_e);
//                
//                if (P.find (utime_e)  != P.end ())
//                    pd.p_observations.push_back(P[utime_e]);
//                else
//                    pd.p_observations.push_back(-1.0);
//                
//                pd.inlier_cnts.push_back(inlier_cnts[j]);
//                if (inlier_cnts[j] > MERGE_EXEMPLAR_MIN_INLIERS) 
//                    pd.observations.push_back(1);
//                else
//                    pd.observations.push_back(0);
//            }
//            perllcm::pvn_eview_map_predict_data_run_t *pdr = &(eview_map.predict_data_colls[irun]);
//            pdr->n++;
//            pdr->predict_data.push_back (pd);
//            
//            
//            // add this exemplar to neighborhood
//            // find pose in neighboorhood frame
//            perllcm::pose3d_t x_n_e;
//            Matrix <double, 6, 12, RowMajor > J;
//            ssc_tail2tail (x_n_e.mu, J.data(), mn->pose.mu, p->mu);
//            Matrix <double, 6, 6, RowMajor > S_w_n (mn->pose.Sigma);
//            Matrix <double, 6, 6, RowMajor > S_w_e (p->Sigma);
//            Matrix <double, 6, 6, RowMajor > S_n_e;
//            S_n_e = J.topLeftCorner(6,6)*S_w_n*J.topLeftCorner(6,6).transpose();
//            S_n_e += J.bottomRightCorner(6,6)*S_w_e*J.bottomRightCorner(6,6).transpose();
//            memcpy (x_n_e.Sigma, S_n_e.data(), 6*6*sizeof (double));
//            
//            perllcm::pvn_eview_map_exemplar_t mne;
//            mne = new_exemplar (p->utime, x_n_e, &camconf, &id, param);
//        
//            mn->exemplars.push_back(mne);    
//            eview_map.neighborhoods[compare_with].ne++;
//            
//            // mark this exemplar as being observed
//            eview_map.match_data[irun].exemplar_utimes.push_back(p->utime);
//            eview_map.match_data[irun].observations.push_back(1);
//            eview_map.match_data[irun].n++; 
//        
//            
//        } else {
//            ERROR ("SHOULDN'T EVER GET HERE!");
//            
//        }
//        
//        // associate features with 3D pcoints
//        tic = timestamp_now();
//        int forgot_cnt = forget_old_exemplars (&eview_map, MAX_EXEMPLAR_PER_NEIGHBORHOOD);
//        cout << "[eview map]\tForgot " << forgot_cnt << " exemplars (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
//        
//        if (i > 0 && 0 == (i % EVIEW_BACKUP_EVERY)) {
//            last_backup = i;
//            char filename[PATH_MAX];
//            snprintf (filename, sizeof filename, "%s/%s-backup.evmap", map_dir, map_name);
//            ret = pvnu_lcm_fwrite <perllcm::pvn_eview_map_t> (filename, eview_map);
//            if (ret < 0)
//                ERROR ("couldn't write %s to disk!", filename);
//            cout << "[eview map]\tWrote backup eview map to disk: " << filename << endl;           
//        }
//        
//        
//    }
//    
//    cache_destroy (lrc_data_cache);
//    
//    
//    return eview_map;
//
//}
//


void
PVNMapBuilder::run_build_eview_map ()
{
    //int64_t tic_start = timestamp_now ();
    //
    //// parse in graph
    //char graph_pc[PATH_MAX];
    //sprintf (graph_pc, "%s/pc.dat", graph_dir);
    //cout << "[eview map]\tLoading graph: " << graph_pc << endl;
    //perllcm::pose3d_collection_t pc;
    //int32_t ret = pvnu_lcm_fread <perllcm::pose3d_collection_t> (graph_pc, &pc);
    //if (ret < 0) {
    //    cout << "[eview map]\tCouldn't read " << graph_pc << " from disk. ret=" << ret << endl;
    //    return;
    //}
    //cout << "[eview map]\tFound " << pc.npose << " graph nodes." << endl;
    //
    //
    //// decide if we are updating or initalizing
    //perllcm::pvn_eview_map_t eview_map;
    //char fmap[PATH_MAX];
    //sprintf (fmap, "%s/%s.evmap", map_dir, map_name);
    //cout << "[eview map]\tTrying to load existing map: " << fmap << endl;
    //if (file_exists (fmap) || eview_backup > 0) {
    //    
    //    if (eview_backup > 0)
    //        sprintf (fmap, "%s/%s-backup.evmap", map_dir, map_name);
    //        
    //    eview_map = eview_map_update (pc, fmap);
    //    
    //} else {
    //    eview_map = eview_map_new (pc, fmap);
    //}  
    //
    ////write map
    //char filename[PATH_MAX];
    //snprintf (filename, sizeof filename, "%s/%s-updated.evmap", map_dir, map_name);
    //ret = pvnu_lcm_fwrite <perllcm::pvn_eview_map_t> (filename, eview_map);
    //if (ret < 0)
    //    ERROR ("couldn't write %s to disk!", filename);
    //cout << "[eview map]\tWrote eview map to disk: " << filename << endl;
    //cout << "[eview map]\tDone (" << (timestamp_now()-tic_start)/1e6 << " secs)." << endl;
    //
}
