#include "map_builder.h"

#include "perls-pvn/pvn_chowliutree.h"

void
PVNMapBuilder::run_eview_process (char *fname) {
//
//    perllcm::pvn_eview_map_t eview_map;
//    int32_t ret = pvnu_lcm_fread <perllcm::pvn_eview_map_t> (fname, &eview_map);
//    if (ret < 0) {
//        cout << "[eview map]\tCouldn't read " << fname << " from disk. ret=" << ret << endl;
//        exit (-1);
//    }
//    cout << "[eview map]\tLoaded existing map: " << fname << endl;
//    
//    char base_name[PATH_MAX] = {0};
//    size_t i=0;
//    while (i<strlen(fname) && fname[i] != '.') {
//        base_name[i] = fname[i];
//        i++;
//    }
//    base_name[i] = '\0';    
//    cout << "[eview map]\tSplitting file using basename: " << base_name << endl;
//  
//    //calculate the chow liu tree at the start of this run
//    PVNCLTree clt;
//    vector<perllcm::pvn_eview_map_match_data_t> run_data_prev;
//    for (int j=0; j<(eview_map.nr-1) ; j++)
//        run_data_prev.push_back (eview_map.match_data[j]);
//    clt.make (run_data_prev, 0, CLT_MAP_PRIOR_ALPHA);
//    cout << "[cl tree]\tBuilt Chow Liu Tree" << endl;
//
//    char hist_all_fname[PATH_MAX] = {0};
//    sprintf (hist_all_fname, "%s-hist-all.txt", base_name);
//    ofstream fhist_all;
//    fhist_all.open(hist_all_fname);
//    char hist_unseen_fname[PATH_MAX] = {0};
//    sprintf (hist_unseen_fname, "%s-hist-unseen.txt", base_name);
//    ofstream fhist_unseen;
//    fhist_unseen.open(hist_unseen_fname);
//
//
//    // run through this runs data and save histograms of maximum lilihood map
//    int irun = eview_map.nr-1;
//    perllcm::pvn_eview_map_match_data_t match_data;
//    // copy over part of the current run int run data
//    int first_n = round(eview_map.match_data[irun].n*0.1);
//    for (int n=0; n<first_n; n++) {
//        match_data.exemplar_utimes.push_back(eview_map.match_data[irun].exemplar_utimes[n]);
//        match_data.observations.push_back(eview_map.match_data[irun].observations[n]);
//    }
//    match_data.n = first_n;
//   
//    map<int64_t, bool> obs = pvnu_make_run_data_make_obs (&match_data, NULL);
//    map<int64_t, float> P = clt.sumProduct (obs);
//
//
//    int all_sky_sunny = 0, all_sky_cloudy = 0, all_sky_overcast = 0;
//    int all_tod_morning = 0, all_tod_midday = 0, all_tod_afternoon = 0;
//    int all_snow_yes = 0, all_snow_no = 0;
//    int all_foliage_yes = 0, all_foliage_no = 0;
//    //loop over each neighborhood and find the most likley exemplar and add its conditions to histogram
//    for (int n=0; n<eview_map.nn; n++) {
//        perllcm::pvn_eview_map_neighborhood_t *mn = &(eview_map.neighborhoods[n]);
//        
//        // find the max
//        double max_p = 0.0;
//        int max_ind = -1;
//        for (int j=0; j < mn->ne; j++) {
//            perllcm::pvn_eview_map_exemplar_t *ne = &(mn->exemplars[j]);
//            if (P[ne->utime] > max_p) {
//                max_p = P[ne->utime];
//                max_ind = j;
//            } 
//        }
//
//        //update counts
//        perllcm::pvn_conditions_t *ec = &(mn->exemplars[max_ind].conditions);
//        if (ec->sky == perllcm::pvn_conditions_t::SKY_SUNNY) all_sky_sunny++;
//        if (ec->sky == perllcm::pvn_conditions_t::SKY_CLOUDY) all_sky_cloudy++;
//        if (ec->sky == perllcm::pvn_conditions_t::SKY_OVERCAST) all_sky_overcast++;
//        if (ec->tod == perllcm::pvn_conditions_t::TOD_MORNING) all_tod_morning++;
//        if (ec->tod == perllcm::pvn_conditions_t::TOD_MIDDAY) all_tod_midday++;
//        if (ec->tod == perllcm::pvn_conditions_t::TOD_AFTERNOON) all_tod_afternoon++;
//        if (ec->snow == 0) all_snow_no++;
//        if (ec->snow == 1) all_snow_yes++;
//        if (ec->foliage == 0) all_foliage_no++;
//        if (ec->foliage == 1) all_foliage_yes++;
//    }
//
//    // write to file
//    fhist_all << all_sky_sunny << ", " << all_sky_sunny << ", " << all_sky_sunny << ", ";
////    fhist_all << endl
//
//    fhist_all.close();
//    fhist_unseen.close();
//
//
//return;
//
//
//    // look at mi between views in the center of diag
//    // first find all neighborhoods in this region
//    double x_box[2] = {-153.8212, -121.7468}; // center of diag
//    double y_box[2] = {-516.1172, -487.7724}; // center of diag
//    //double x_box[2] = {-76.9919, -65.0572 }; // in front of cse
//    //double y_box[2] = {-552.6671, -528.0518 }; // in front of cse
//    //double x_box[2] = {-311.7698, -288.5181}; // in front of dude
//    //double y_box[2] = {-554.5890, -532.8374}; // in front of dude
//    vector<int> niis;
//    for (int n=0; n<eview_map.nn; n++) {
//        if (eview_map.neighborhoods[n].pose.mu[0] > x_box[0] &&
//            eview_map.neighborhoods[n].pose.mu[0] < x_box[1] &&
//            eview_map.neighborhoods[n].pose.mu[1] > y_box[0] &&
//            eview_map.neighborhoods[n].pose.mu[1] < y_box[1]) {
//
//            niis.push_back(n);
//        }
//    }
//    set<int64_t> exemplars;
//    for (size_t n=0; n<niis.size(); n++) {
//        perllcm::pvn_eview_map_neighborhood_t *mn = &(eview_map.neighborhoods[niis[n]]);
//
//        for (int j=0; j < mn->ne; j++) {
//            exemplars.insert(mn->exemplars[j].utime);
//        }
//    }
//    cout << "[eview map]\tMax mi in a given region" << endl;
//    clt.find_max_mi (exemplars);
//
//    // look at each pair in tree and find those that are close spatially
//    cout << "[eview map]\tCL Tree branches in same region" << endl;
//    map<int64_t, clNode>::iterator itt;
//    for (itt = clt.tree.begin(); itt !=clt.tree.end(); ++itt) {
//
//        if (itt->second.is_root())
//            continue;
//
//        int64_t id1 = itt->second.id;
//        int64_t id2 = itt->second.pid;
//        double xy1[2] = {0};
//        double xy2[2] = {0};
//
//        // loop over exemplars, are these to exemplars spatialy close
//        for (int n=0; n<eview_map.nn; n++) {
//            perllcm::pvn_eview_map_neighborhood_t *mn = &(eview_map.neighborhoods[n]);
//            for (int j=0; j < mn->ne; j++) {
//
//                if(id1 == mn->exemplars[j].utime) {
//                    xy1[0] = mn->pose.mu[0];
//                    xy1[1] = mn->pose.mu[1];
//                }
//                if(id2 == mn->exemplars[j].utime) {
//                    xy2[0] = mn->pose.mu[0];
//                    xy2[1] = mn->pose.mu[1];
//                }
//            
//            }
//        }
//
//        double dist = sqrt(pow(xy1[0]-xy2[0],2) + pow(xy1[1]-xy2[1],2));
//        if (dist < 30) {
//            cout << id1 << " <--> " << id2 << " " << itt->second.p_mi  << endl;
//        }
//    }
//
//
//    /*    
//    char neighborhood_fname[PATH_MAX] = {0};
//    for (int n=0; n<eview_map.nn; n++) {
//        sprintf (neighborhood_fname, "%s-%d.neighborhood", base_name, n);   
//        pvnu_lcm_fwrite <perllcm::pvn_eview_map_neighborhood_t> (neighborhood_fname, eview_map.neighborhoods[n]);
//    }
//    cout << "[eview map]\tNeighborhoods written." << endl;
//    */
//    
//    char run_data_fname[PATH_MAX] = {0};
//    for (int n=0; n<eview_map.nr; n++) {
//        sprintf (run_data_fname, "%s-%d.rundata", base_name, n);   
//        pvnu_lcm_fwrite <perllcm::pvn_eview_map_match_data_t> (run_data_fname, eview_map.match_data[n]);
//    }
//    cout << "[eview map]\tRun data written." << endl;
//    
//    char pred_data_fname[PATH_MAX] = {0};
//    for (int n=0; n<eview_map.nr; n++) {
//        sprintf (pred_data_fname, "%s-%d.preddata", base_name, n);   
//        pvnu_lcm_fwrite <perllcm::pvn_eview_map_predict_data_run_t> (pred_data_fname, eview_map.predict_data_colls[n]);
//    }
//    cout << "[eview map]\tPredict data written." << endl;
    
}
