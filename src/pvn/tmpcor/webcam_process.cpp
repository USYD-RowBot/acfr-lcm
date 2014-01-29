#include <opencv/cv.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "tmpcor.h"

void
setup_backup (char *dir, int irun) {
    char backup_dir[PATH_MAX];
    sprintf(backup_dir, "%s/eview_map/run_%d", dir, irun);
    boost::filesystem::remove_all(backup_dir);
    boost::filesystem::create_directory(backup_dir);
}

void
backup_map (char *dir, int irun, int ncams) {
    char backup_dir[PATH_MAX];
    sprintf(backup_dir, "%s/eview_map/run_%d", dir, irun);
    // copy map 
    char map_src[PATH_MAX], map_dest[PATH_MAX];
    sprintf(map_src, "%s/eview_map/map.evmap", dir);
    sprintf(map_dest, "%s/map.evmap", backup_dir);
    boost::filesystem::path map_src_path(map_src); boost::filesystem::path map_dest_path(map_dest);
    boost::filesystem::copy_file(map_src_path, map_dest_path, boost::filesystem::copy_option::overwrite_if_exists);
    // copy neighborhoods
    for (int ii_cam=0; ii_cam<ncams; ii_cam++) {
        char mn_src[PATH_MAX], mn_dest[PATH_MAX];
        sprintf(mn_src, "%s/eview_map/%d.mn", dir, ii_cam);
        sprintf(mn_dest, "%s/%d.mn", backup_dir, ii_cam);
        boost::filesystem::path mn_src_path(mn_src); boost::filesystem::path mn_dest_path(mn_dest);
        boost::filesystem::copy_file(mn_src_path, mn_dest_path, boost::filesystem::copy_option::overwrite_if_exists);
    }
}

void
PVNTempCor::update_predict_data_full (perllcm::pvn_eview_map_t *eview_map,
                                      int irun, int ii_cam,
                                      map<int64_t, float> P, int64_t obs_utime,
                                      perllcm::pvn_conditions_t obs_conds, char *dir) {
    
    perllcm::pvn_eview_map_predict_data_collection_t pdcf;
    pdcf.n = 0;    
    
                    
    for (int i=0; i<eview_map->nn; i++) {
        
        // load the neighborhood
        char mn_file[PATH_MAX];
        sprintf(mn_file, "%s/eview_map/%d.mn", dir, i);
        perllcm::pvn_eview_map_neighborhood_t mn;
        pvnu_lcm_fread <perllcm::pvn_eview_map_neighborhood_t> (mn_file, &mn);
        
        // not set yet TODO
        std::vector<int> inlier_cnts (mn.ne, 0);
        std::vector<float> reproj_error (mn.ne, 0);
        std::vector<int8_t> match_success (mn.ne, 0);
        
        pvn_eview_update_predict_data (&pdcf, &mn, match_success, inlier_cnts,
                                       P, obs_utime, obs_conds, 0,
                                       inlier_cnts, reproj_error);
    }
    
    // write pdcf to file
    char backup_dir[PATH_MAX];
    sprintf(backup_dir, "%s/eview_map/run_%d", dir, irun);
    char filename[PATH_MAX];
    sprintf(filename, "%s/%d.pdcf", backup_dir, ii_cam);
    int ret = pvnu_lcm_fwrite <perllcm::pvn_eview_map_predict_data_collection_t> (filename, pdcf);
    if (ret < 0)
        ERROR ("couldn't write %s to disk!", filename);
    
                    
}

vector<int>
PVNTempCor::feat_match_neighborhood (perllcm::pvn_eview_map_neighborhood_t *mn,
                                     PVNImageData *id,
                                     vector<int> *exemplar_num_feats,
                                     vector<float> *reproj_error) {

    int64_t tic = timestamp_now ();
    
    // current image descriptors
    // dont see any real speed improvment here using FLANN 
    cv::BruteForceMatcher< cv::L2<float> > matcher;
    //cv::FlannBasedMatcher matcher;
    vector<cv::Mat> train_desc_vec;
    train_desc_vec.push_back (id->to_opencv_mat());
    matcher.add(train_desc_vec);
    
    // compare the current image with all previous images
    vector<int> putative_cnt (mn->ne, 0);
    vector<int> inlier_cnt (mn->ne, 0);
    if (NULL != reproj_error) {
        reproj_error->resize(mn->ne);
    }
    if (NULL != exemplar_num_feats) {
        exemplar_num_feats->resize(mn->ne);
    }
    for (int i=0; i<mn->ne; i++) {
        
        perllcm::pvn_eview_map_exemplar_t *mne = &(mn->exemplars[i]);
        
        if (NULL != exemplar_num_feats) {
            (*exemplar_num_feats)[i] = (mne->npts);
        }
    
        // neighborhood exemplar descriptors
        cv::Mat query_desc (mne->npts, mne->keylen, CV_32F);
        for (int j=0; j<mne->npts; j++) {
            for (int k=0; k<mne->keylen; k++) {
                query_desc.at<float> (j, k) = mne->keys[j][k];
            }
        }    
        // matching descriptors
        vector< vector<cv::DMatch> > knn_matches;
        matcher.knnMatch(query_desc, knn_matches, 2);
        vector<cv::DMatch> matches;
        for (size_t j=0; j<knn_matches.size(); j++) {
            if (knn_matches[j].size() != 2)
                continue;
            
            double d_1st = knn_matches[j][0].distance;
            double d_2nd = knn_matches[j][1].distance;
            
            if (d_1st/d_2nd < pow(SIFT_2ND_NN_THRESH, 2)) {
                matches.push_back (knn_matches[j][0]);
            }            
        }
        putative_cnt[i] = matches.size();
        if (putative_cnt[i] < 10) {
            continue;
        }
        
        // publish putative for debugging
        if (DO_PLOT) {
            perllcm::viewer_image_pccs_t vis_pccs;
            vis_pccs.utime = id->utime + id->usec_offset;
            vis_pccs.num_corrs = matches.size();
            vis_pccs.num_covp = 0;
            for (int j=0; j<vis_pccs.num_corrs; j++) {
                vis_pccs.u.push_back  (id->feature_collections[0].f[0].u[matches[j].trainIdx]);
                vis_pccs.v.push_back  (id->feature_collections[0].f[0].v[matches[j].trainIdx]);
                vis_pccs.up.push_back (mne->u[matches[j].queryIdx]);
                vis_pccs.vp.push_back (mne->v[matches[j].queryIdx]);
            }
            vis_pccs.type = perllcm::viewer_image_pccs_t::TYPE_PUTATIVE;
            char pccs_chan[64]= {0};
            sprintf (pccs_chan, "PCCS_WEBCAM_IMG");
            lcm.publish (pccs_chan, &vis_pccs);
            //timeutil_usleep(1e6);
        }
        
        size_t nmatches = matches.size();
        std::vector<float> u(nmatches, 0.0), v(nmatches, 0.0), up(nmatches, 0.0), vp(nmatches, 0.0);
        for (size_t j=0; j<matches.size(); j++) {
            u[j] = id->feature_collections[0].f[0].u[matches[j].trainIdx];
            v[j] = id->feature_collections[0].f[0].v[matches[j].trainIdx];
            up[j] = mne->u[matches[j].queryIdx];
            vp[j] = mne->v[matches[j].queryIdx];
        }
        
        float reproj_error_tmp = -1.0;
        std::vector<int> inliers = pvnu_median_outlier_rejection (u, v, up, vp, &reproj_error_tmp);
        inlier_cnt[i] = inliers.size();
        
        if (NULL != reproj_error) {
            (*reproj_error)[i] = reproj_error_tmp;
        }
        
        // publish inliers for debugging
        if (DO_PLOT) {
            perllcm::viewer_image_pccs_t vis_pccs;
            vis_pccs.utime = id->utime + id->usec_offset;
            vis_pccs.num_corrs = inliers.size();
            vis_pccs.num_covp = 0;
            for (int j=0; j<vis_pccs.num_corrs; j++) {
                vis_pccs.u.push_back  (u[inliers[j]]);
                vis_pccs.v.push_back  (v[inliers[j]]);
                vis_pccs.up.push_back (up[inliers[j]]);
                vis_pccs.vp.push_back (vp[inliers[j]]);
            }
            vis_pccs.type = perllcm::viewer_image_pccs_t::TYPE_INLIER;
            char pccs_chan[64]= {0};
            sprintf (pccs_chan, "PCCS_WEBCAM_IMG");
            lcm.publish (pccs_chan, &vis_pccs);
            //timeutil_usleep(1e6);
        }
        
    }
    
    if (DO_PLOT) {
        cout << "[webcam]\tMatching (" << (timestamp_now () - tic)/1e6 << " sec): " << endl;
        for (int i=0; i<mn->ne; i++) {
            cout << "\tExemplar " << i << ": Putative " << putative_cnt[i] << ", Inliers " << inlier_cnt[i] << endl;
        }
    }
    
    return inlier_cnt;
    
}

void
PVNTempCor::simple_wcpn_update (perllcm::pvn_eview_map_neighborhood_t &mn, // the existing map neighborhood
                                perllcm::pvn_eview_map_t &eview_map,       // the map (has the match_data)
                                perllcm::pvn_eview_map_exemplar_t mne) {   // the new view
    
    perllcm::pvn_eview_map_match_data_t *md = &(eview_map.match_data[eview_map.nr-1]);
    
    bool is_match = false;
    for (int i=0; i<md->n; i++) {

        if (mne.utime != md->obs_utimes[i]) // only look at match data associated with this views observation
            continue;
        
        is_match |= pvn_eview_calc_match_success(md->num_inliers[i],
                                                 md->reproj_error[i],
                                                 EXEMPLAR_MATCH_INLIER_THRESH,
                                                 EXEMPLAR_MATCH_REPROJ_ERROR_THRESH);
    }
    
    // no good match add a new exemplar
    if (!is_match) {
        
        // update the map neighborhood
        mn.ne++;
        mn.exemplars.resize(mn.ne);
        mn.exemplars[mn.ne-1] = mne;
        
        // update the match_data w/ first observation of new exemplar
        md->exemplar_utimes.push_back (mne.utime);
        md->num_inliers.push_back (mne.npts);
        md->obs_utimes.push_back (mne.utime);
        md->obs_num_feats.push_back (mne.npts);
        md->exemplar_num_feats.push_back (mne.npts);
        md->reproj_error.push_back (0.0);
        md->match_success.push_back (1);
        md->n++;
        
        cout << "[webcam]\tNew exemplar added (" << mn.ne << ")." << endl;
    } else {
        cout << "[webcam]\tNew exemplar NOT added (" << mn.ne << "). " << endl;
    }
    
    // no forgetting
}


void
PVNTempCor::simple_forget_update (perllcm::pvn_eview_map_neighborhood_t &mn, // the existing map neighborhood
                                  perllcm::pvn_eview_map_t &eview_map,       // the map (has the match_data)
                                  perllcm::pvn_eview_map_exemplar_t mne) {   // the new view
    
    perllcm::pvn_eview_map_match_data_t *md = &(eview_map.match_data[eview_map.nr-1]);
    
    bool is_match = false;
    for (int i=0; i<md->n; i++) {

        if (mne.utime != md->obs_utimes[i]) // only look at match data associated with this views observation
            continue;
        
        is_match |= pvn_eview_calc_match_success(md->num_inliers[i],
                                                 md->reproj_error[i],
                                                 EXEMPLAR_MATCH_INLIER_THRESH,
                                                 EXEMPLAR_MATCH_REPROJ_ERROR_THRESH);
    }
    
    // no good match add a new exemplar
    if (!is_match) {

        // want to add a new exemplar
        // might need to forget least recently used to make room
        if (mn.ne > EXEMPLAR_MAX_PER_NEIGHBORHOOD-1) {
            // find least recently used
            int64_t utime_lru_last_match = mne.utime; // most recent possible
            int64_t utime_lru = 0;
            int ind_lru = -1;
        
            for (int i=0; i<mn.ne; i++) {
                if (mn.exemplars[i].utime_last_match < utime_lru_last_match) {
                    utime_lru_last_match = mn.exemplars[i].utime_last_match;
                    utime_lru = mn.exemplars[i].utime;
                    ind_lru = i;
                }            
            }
            if (ind_lru == -1)
                ERROR ("COULD NOT FIND LRU: SHOULD NOT HAPPEN");
                
            cout << "[webcam]\tForgeting Exemplar " << ind_lru << ".";
            if (utime_lru_last_match != 0)
                cout << " Last used " << (mne.utime-utime_lru_last_match)/(1e6*60*60*24) << " days ago." << endl;
            else
                cout << " Never used." << endl;
                
            //remove match data and predict data associated with the exemplar
            for (int i=0; i<eview_map.nr; i++) { // loop over runs
                perllcm::pvn_eview_map_match_data_t *rd = &(eview_map.match_data[i]);
                
                // match data
                // need to forget because used to build chow-liu tree, don't want to include forgotten variables
                int32_t n_new = 0;
                vector<int64_t> exemplar_utimes_new;
                vector<int64_t> obs_utimes_new;
                vector<int32_t> num_inliers_new;
                vector<int8_t> match_success_new;
                
                for (int k=0; k<rd->n; k++) { // loop over data in run
                    if (rd->exemplar_utimes[k] != utime_lru) {
                        n_new++;
                        exemplar_utimes_new.push_back (rd->exemplar_utimes[k]);
                        obs_utimes_new.push_back (rd->obs_utimes[k]);
                        num_inliers_new.push_back (rd->num_inliers[k]);
                        match_success_new.push_back (rd->match_success[k]);
                    }
                }
                //cout << "Removed " << rd->n-n_new << " match data entries."<< endl;
                rd->n = n_new;
                rd->exemplar_utimes = exemplar_utimes_new;
                rd->obs_utimes = obs_utimes_new;
                rd->num_inliers = num_inliers_new;
                rd->match_success = match_success_new;
                
            }
            
            mn.exemplars.erase (mn.exemplars.begin() + ind_lru);
            mn.ne--;
        
            
        }
        
        // update the map neighborhood
        mn.ne++;
        mn.exemplars.resize(mn.ne);
        mn.exemplars[mn.ne-1] = mne;
        
        // update the match_data w/ first observation of new exemplar
        md->exemplar_utimes.push_back (mne.utime);
        md->num_inliers.push_back (mne.npts);
        md->obs_utimes.push_back (mne.utime);
        md->obs_num_feats.push_back (mne.npts);
        md->exemplar_num_feats.push_back (mne.npts);
        md->reproj_error.push_back (0.0);
        md->match_success.push_back (1);

        md->n++;
        
        cout << "[webcam]\tNew exemplar added (" << mn.ne << ")." << endl;
    } else {
        cout << "[webcam]\tNew exemplar NOT added (" << mn.ne << ")." << endl;
    }    
    
}

set < pair<int64_t, int64_t> >
compute_mi_edge_mask (char *dir, vector<int64_t> neighborhood_ids) {

    set < pair<int64_t, int64_t> > mask;
    
    for (size_t n=0; n<neighborhood_ids.size(); n++) {
    
        // load each neighborhood
        char mn_file[PATH_MAX];
        sprintf(mn_file, "%s/eview_map/%ld.mn", dir, neighborhood_ids[n]);
        perllcm::pvn_eview_map_neighborhood_t mn;
        pvnu_lcm_fread <perllcm::pvn_eview_map_neighborhood_t> (mn_file, &mn);
        
        // loop over each pair of exemplars
        for (int i=0; i<mn.ne; i++) {
            for (int j=i+1; j<mn.ne; j++) {
                pair <int64_t, int64_t> tmp (mn.exemplars[i].utime, mn.exemplars[j].utime);
                mask.insert (tmp);                        
            }
        }
    }    
    return mask;
}



void
PVNTempCor::run_webcam_process (char *dir) {
    
    srand ( time(NULL) );
    
    // parse in the meta data
    std::string dir_str (dir);
    int ncams = count_num_cams (dir);
    vector<int64_t> utimes = read_utimes (dir);
    cout << "[webcam]\tRead Metadata in: " << dir << endl;
    cout << "[webcam]\tNum Cams = " << ncams << ", Num Samples = " << utimes.size() << endl;
    
    // create a empty map
    perllcm::pvn_eview_map_t eview_map = {0};
    eview_map.orglat = 0.0; eview_map.orglon = 0.0; eview_map.orgalt = 0.0;
    eview_map.nn = 0;
    eview_map.neighborhood_ids[eview_map.nn];
    eview_map.neighborhood_poses[eview_map.nn];
    eview_map.nr = 0;
    eview_map.match_data.resize(eview_map.nr);
    // create an empty predict data
    std::vector <perllcm::pvn_eview_map_predict_data_collection_t> predict_data_colls;
    
    
    int irun = -1;
    // clear out map folder if existing
    char map_dir[PATH_MAX];
    sprintf(map_dir, "%s/eview_map", dir);
    boost::filesystem::remove_all(map_dir);
    boost::filesystem::create_directory(map_dir);

    // load camera config
    PVNCamConfig camconf;
    if (camconf.LoadMonoConfig (param, "webcam", "webcam-feature")) {
        ERROR ("Failed to load camera config!");
    }
    std::vector<PVNCamConfig> camconfs;
    camconfs.push_back(camconf);
    
    int step = 1;    
    // loop over each utime
    for (size_t ii_utime=0; ii_utime<utimes.size(); ii_utime += step) {
        
        if (random_step)
            step = ( rand() % random_step + 1 );
        else
            step =  every_nth;

        cout << "[webcam]\tStepping " << step << " hours." << endl;
        
        int64_t utime = utimes[ii_utime];
        
        // SETUP ---------------------------------------------------------------
        // read conditions for this utime
        std::string conds_file = dir_str + "/conds/" + boost::lexical_cast<std::string>(utime) + ".cond";
        perllcm::pvn_conditions_t conds = read_conditions (conds_file);
        
        // compute chow-liu tree
        PVNCLTree clt;
        if (eview_map.match_data.size() > 0) {    
            // build the chow-liu tree from previous run data, dont include this current run
            set < pair<int64_t, int64_t> > mask = compute_mi_edge_mask (dir, eview_map.neighborhood_ids);
            int64_t tic = timestamp_now();
            double a, b;
            pvn_eview_beta_prior_coefs(CLT_PRIOR_MODE_TARGET, CLT_PRIOR_STRENGTH, &a, &b);
            clt.make (eview_map.match_data, 0, a, b, mask);
            cout << "[cl tree]\tBuilt Chow Liu Tree: ( " << (timestamp_now() - tic)/1e6 << " sec.)" << endl;
        }
        
        // this is a new run
        irun++;
        eview_map.nr++;
        perllcm::pvn_eview_map_match_data_t md_tmp = {0};
        eview_map.match_data.resize(eview_map.nr);
        eview_map.match_data[irun] = md_tmp;
        perllcm::pvn_eview_map_predict_data_collection_t pdc_tmp = {0};
        eview_map.predict_data_colls.resize(eview_map.nr);
        eview_map.predict_data_colls[irun] = pdc_tmp;
        perllcm::pvn_eview_map_match_data_t *md = &(eview_map.match_data[irun]);
        perllcm::pvn_eview_map_predict_data_collection_t *pdc = &(eview_map.predict_data_colls[irun]);        
        
        cout << "[webcam]\tStarting run number: " << eview_map.nr << endl;
    
        setup_backup (dir, irun);
        
        // backup the clt
        char clt_file[PATH_MAX];    
        sprintf(clt_file, "%s/eview_map/run_%d/clt.txt", dir, irun);
        clt.print_tree_to_file (clt_file);
        sprintf(clt_file, "%s/eview_map/clt.txt", dir);
        clt.print_tree_to_file (clt_file);
    
        // LOCALIZE ------------------------------------------------------------
        
        // randomize trajectory
        vector<int> rand_inds;
        for (int i=0; i<ncams; i++) rand_inds.push_back(i); 
        random_shuffle (rand_inds.begin(), rand_inds.end() );       
        
        // loop over each cam (locations)
        int64_t loc_tic = timestamp_now();
        for (int ii=0; ii<ncams; ii++) {
            int ii_cam = rand_inds[ii];
        
            // load image features
            std::vector<std::string> image_dirs;
            image_dirs.push_back (dir_str + "/" + boost::lexical_cast<std::string>(ii_cam));
            PVNImageData id(camconfs, image_dirs, utime, "jpg", ii_cam);
            int resize[2] = RESIZE_IMAGE_SIZE;
            if (id.load_features (&lcm, resize))
                continue;
            
            // try to load neighborhood if not make a new one
            char mn_file[PATH_MAX];
            sprintf(mn_file, "%s/eview_map/%d.mn", dir, ii_cam);
            perllcm::pvn_eview_map_neighborhood_t mn;
            int32_t ret = pvnu_lcm_fread <perllcm::pvn_eview_map_neighborhood_t> (mn_file, &mn);
            if (ret < 0) {
                cout << "[webcam]\tCreating new neighborhood: " << ii_cam << endl;
                
                perllcm::pose3d_t p = {0};
                mn.id = ii_cam;
                mn.pose = p;
                mn.ne = 1;
                mn.exemplars.resize(mn.ne);
                perllcm::pose3d_t x_n_e = {0};
                mn.exemplars[0] = id.to_pvn_eview_map_exemplar_t (x_n_e);
                mn.exemplars[0].conditions = conds;
                
                // update match data with a perfect match
                vector<int> inlier_cnts (1,  mn.exemplars[0].npts);
                vector<float> reproj_error (1, 0.0);
                std::vector<int8_t> match_success (1, 1.0);
                pvn_eview_update_match_data (md, &mn, match_success, inlier_cnts,
                                             id.utime + id.usec_offset,
                                             mn.exemplars[0].npts, inlier_cnts, reproj_error);
                
                // save neighborhood
                ret = pvnu_lcm_fwrite <perllcm::pvn_eview_map_neighborhood_t> (mn_file, mn);
                if (ret < 0)
                    ERROR ("couldn't write %s to disk!", mn_file);
                    
                // add neighborhood to map
                eview_map.nn++;
                eview_map.neighborhood_ids.push_back (ii_cam);
                perllcm::pose3d_t x_w_n = {0};
                eview_map.neighborhood_poses.push_back (x_w_n);
                
            } else {
                
                
                // predict probability of matching exemplars in this neighboorhood
                //int64_t tic = timestamp_now();             
                
                map<int64_t, float> P;
                if (CLT_USE_SOFT_EVIDENCE){
                    map<int64_t, float> obs = pvn_eview_run_data_2_obs_soft (md, NULL);
                    P = clt.sumProduct (obs);
                }else{
                    map<int64_t, bool> obs = pvn_eview_run_data_2_obs (md, NULL);
                    P = clt.sumProduct (obs);
                }
                //cout << "[webcam]\tPredict visual matching for exemplars (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
                
                // Perform matching
                vector<int> exemplar_num_feats (0, 0);
                vector<float> reproj_error (0, 0);
                vector<int> inlier_cnts = feat_match_neighborhood (&mn, &id, &exemplar_num_feats, &reproj_error);
                
                vector<int8_t> match_success = pvn_eview_calc_match_success_v (inlier_cnts, reproj_error,
                                                                               EXEMPLAR_MATCH_INLIER_THRESH,
                                                                               EXEMPLAR_MATCH_REPROJ_ERROR_THRESH);
// HACK in to test partial observation
//vector<bool> match_data_mask (0,0);
//if (mn.ne <= 4) {
//    vector<bool> tmp (mn.ne, true);
//    match_data_mask = tmp;
//} else {
//    
//    // find two best observations
//    float max1 = -1.0, max2 = -1.0;
//    int max1_ind = -1, max2_ind = -1;
//    
//    for (int j=0; j<mn.ne; j++) {
//        int64_t utime_e = mn.exemplars[j].utime;
//        if (P.find (utime_e)  != P.end ()) {
//            if (P[utime_e] > max1){
//                max1 = P[utime_e];
//                max1_ind = j;
//            } else if (P[utime_e] > max2){
//                max2 = P[utime_e];
//                max2_ind = j;
//            }
//        } else {
//            ERROR ("MISSING POBS!");
//        }
//    }
//    
//    vector<bool> tmp (mn.ne, false);
//    match_data_mask = tmp;
//    vector<int> remaining_inds (0,0);
//    for (int j=0; j<mn.ne; j++) {
//        if (j == max1_ind || j == max2_ind)
//            match_data_mask[j] = true;
//        else
//            remaining_inds.push_back(j);
//    }
//    
//    random_shuffle (remaining_inds.begin(), remaining_inds.end());
//    match_data_mask[remaining_inds[0]] = true;
//    match_data_mask[remaining_inds[1]] = true;
//    
//}
//cout << "=============================================================" << endl;
//for (int j=0; j<mn.ne; j++) {
//    cout << match_data_mask[j] << " ";
//}
//cout << endl <<"===========================================================" << endl;
                
                
                // update match data
                pvn_eview_update_match_data (md, &mn, match_success, inlier_cnts,
                                             id.utime + id.usec_offset,
                                             id.get_num_feats(), exemplar_num_feats,
                                             reproj_error); //, match_data_mask);
                
                // update predict data
                pvn_eview_update_predict_data (pdc, &mn, match_success, inlier_cnts,
                                               P, id.utime + id.usec_offset,
                                               conds, id.get_num_feats(), exemplar_num_feats,
                                               reproj_error);
                
                // update full predict data
                // probably wont use in the actual implementation as it takes to long
                // but use for tempcor
                update_predict_data_full (&eview_map, irun, ii_cam, P,
                                          id.utime + id.usec_offset, conds, dir);
                
                // save exemplar
                perllcm::pose3d_t x_n_e = {0};
                perllcm::pvn_eview_map_exemplar_t mne = id.to_pvn_eview_map_exemplar_t (x_n_e);
                mne.conditions = conds;
                char mne_file[PATH_MAX];
                sprintf(mne_file, "%s/eview_map/%d.mne", dir, ii_cam);
                pvnu_lcm_fwrite <perllcm::pvn_eview_map_exemplar_t> (mne_file, mne);
                
            }
            
            // save eview map
            char evm_file[PATH_MAX];
            sprintf(evm_file, "%s/eview_map/map.evmap", dir);
            pvnu_lcm_fwrite <perllcm::pvn_eview_map_t> (evm_file, eview_map);
            
            // save map neighborhood (needed because utimes_last_matched get updated)
            pvnu_lcm_fwrite <perllcm::pvn_eview_map_neighborhood_t> (mn_file, mn);
            
            
        }
        cout << "[webcam]\tMatching (" << (timestamp_now () - loc_tic)/1e6 << " sec): " << endl;
        
        // back up map folder after this run
        backup_map (dir, irun, ncams);
        
        if (irun <= 0)
            continue;
        
        // POST PROCESS --------------------------------------------------------
        // loop over each cam (locations)
        int64_t pp_tic = timestamp_now ();
        for (int ii_cam=0; ii_cam<ncams; ii_cam++) {
        
            // try to load neighborhood and new exemplar
            char mn_file[PATH_MAX];
            sprintf(mn_file, "%s/eview_map/%d.mn", dir, ii_cam);
            perllcm::pvn_eview_map_neighborhood_t mn;
            int32_t ret = pvnu_lcm_fread <perllcm::pvn_eview_map_neighborhood_t> (mn_file, &mn);
            if (ret < 0) {
                ERROR ("Failed to load neighborhood: %s", mn_file);
                continue;
            }
                
            char mne_file[PATH_MAX];
            sprintf(mne_file, "%s/eview_map/%d.mne", dir, ii_cam);
            perllcm::pvn_eview_map_exemplar_t mne;
            ret = pvnu_lcm_fread <perllcm::pvn_eview_map_exemplar_t> (mne_file, &mne);
            if (ret < 0) {
                ERROR ("Failed to load neighborhood exemplar: %s", mne_file);
                continue;
            }
            
            // update map ------------------------------------------------------
            
            // simple update with no forgetting, add when can't explain with current map exemplars
            // simple_wcpn_update (mn, eview_map, mne);
            
            // simple update with forgetting
            simple_forget_update (mn, eview_map, mne);
            
            // -----------------------------------------------------------------
            
            // save updated neighborhood
            ret = pvnu_lcm_fwrite <perllcm::pvn_eview_map_neighborhood_t> (mn_file, mn);
            if (ret < 0)
                ERROR ("couldn't write %s to disk!", mn_file);
                
            // save updated map
            char evm_file[PATH_MAX];
            sprintf(evm_file, "%s/eview_map/map.evmap", dir);
            pvnu_lcm_fwrite <perllcm::pvn_eview_map_t> (evm_file, eview_map);
            
            // delete mne file
            boost::filesystem::remove(mne_file);
            
        }
        cout << "[webcam]\tPost Processing (" << (timestamp_now () - pp_tic)/1e6 << " sec): " << endl;

        
        
    }

}
