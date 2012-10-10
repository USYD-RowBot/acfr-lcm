#include <bot_lcmgl_client/lcmgl.h>

#include "map_builder.h"
#include "perls-pvn/pvn_chowliutree.h"

using namespace std;
using namespace Eigen;

void
test_sum_product () {
    
    srand ( time(NULL) ); 
    
    vector <perllcm::pvn_eview_map_match_data_t> match_data;
    perllcm::pvn_eview_map_match_data_t rd;
    
    double ru;
    
    int num_samples = 10000;
    match_data.resize(num_samples);
    rd.n = 5; 
    for (int i=0; i<num_samples; i++) {
        rd.exemplar_utimes.clear(); rd.match_success.clear();
        rd.exemplar_utimes.resize(rd.n); rd.match_success.resize(rd.n);
        
        // calculate x1
        ru = (double)rand()/(double)RAND_MAX;
        if (ru > 0.75) {
            rd.exemplar_utimes[0] = 1;
            rd.match_success[0] = 1; 
        } else {
            rd.exemplar_utimes[0] = 1;
            rd.match_success[0] = 0;
        }
        
        // calculate x2|x1
        ru = (double)rand()/(double)RAND_MAX;
        rd.exemplar_utimes[1] = 2;
        if (rd.match_success[0] == 0) {
            if (ru > 0.1) {
                rd.match_success[1] = 1; 
            } else {
                rd.match_success[1] = 0;
            }
        } else {
            if (ru > 0.9) {
                rd.match_success[1] = 1; 
            } else {
                rd.match_success[1] = 0;
            }
        }
        
        // calculate x3|x2
        ru = (double)rand()/(double)RAND_MAX;
        rd.exemplar_utimes[2] = 3;
        if (rd.match_success[1] == 0) {
            if (ru > 0.4) {
                rd.match_success[2] = 1; 
            } else {
                rd.match_success[2] = 0;
            }
        } else {
            if (ru > 0.6) {
                rd.match_success[2] = 1; 
            } else {
                rd.match_success[2] = 0;
            }
        }
        
        // calculate x4|x2
        ru = (double)rand()/(double)RAND_MAX;
        rd.exemplar_utimes[3] = 4;
        if (rd.match_success[1] == 0) {
            if (ru > 0.2) {
                rd.match_success[3] = 1; 
            } else {
                rd.match_success[3] = 0;
            }
        } else {
            if (ru > 0.3) {
                rd.match_success[3] = 1; 
            } else {
                
                rd.match_success[3] = 0;
            }
        }
        
        // calculate x5|x4
        ru = (double)rand()/(double)RAND_MAX;
        rd.exemplar_utimes[4] = 5;
        if (rd.match_success[3] == 0) {
            if (ru > 0.6) {
                rd.match_success[4] = 1; 
            } else {
                rd.match_success[4] = 0;
            }
        } else {
            if (ru > 0.4) {
                rd.match_success[4] = 1; 
            } else {
                
                rd.match_success[4] = 0;
            }
        }
        
        match_data[i] = rd;
    }

    PVNCLTree clt, cltp;
    clt.make (match_data);
    clt.print_tree ();
    cltp.make (match_data, 0, 2.0);
    map<int64_t, bool> obs;
    
    cout << "NO OBS ======================================================" << endl;
    obs.clear();
    map<int64_t, float> P = clt.sumProduct (obs);
    map<int64_t, float>::iterator it;
    for (it=P.begin(); it!=P.end(); it++) {
        cout << "[cl tree]\t\tP(X" << it->first << "=1) = " << it->second << endl;
    }
    cout << "looking for true marginals to match" << endl;
    
    cout << "NO OBS w/ PRIOR =============================================" << endl;
    obs.clear();
    P = cltp.sumProduct (obs);
    for (it=P.begin(); it!=P.end(); it++) {
        cout << "[cl tree]\t\tP(X" << it->first << "=1) = " << it->second << endl;
    }
    cout << "looking for change towards prior" << endl;
    
    cout << "X1 = 1 ======================================================" << endl;
    obs.clear();
    obs[1] = 1;
    P = clt.sumProduct (obs);
    for (it=P.begin(); it!=P.end(); it++) {
        cout << "[cl tree]\t\tP(X" << it->first << "=1) = " << it->second << endl;
    }
    cout << "looking for p(x1=1) = 0.99" << endl;
    cout << "looking for p(x2=1) = 0.1" << endl;
    cout << "looking for p(x4=1) to increase slightly from marginal" << endl;
    
    cout << "X2 = 0 ======================================================" << endl;
    obs.clear();
    obs[2] = 0;
    P = clt.sumProduct (obs);
    for (it=P.begin(); it!=P.end(); it++) {
        cout << "[cl tree]\t\tP(X" << it->first << "=1) = " << it->second << endl;
    }
    cout << "looking for p(x2 = 1) = 0.01" << endl;
    cout << "looking for p(x3 = 1) = 0.6" << endl;
    cout << "looking for p(x4 = 1) = 0.8" << endl;
    cout << "looking for p(x1 = 1) to increase ~ 0.75" << endl;
    
    
    cout << "X1 = 1, X3 = 1, X4 = 1 ======================================" << endl;
    obs.clear();
    obs[1] = 1;
    obs[3] = 1;
    obs[4] = 1;
    P = clt.sumProduct (obs);
    for (it=P.begin(); it!=P.end(); it++) {
        cout << "[cl tree]\t\tP(X" << it->first << "=1) = " << it->second << endl;
    }
    cout << "looking for p(x1 = 1) = 0.99" << endl;
    cout << "looking for p(x2 = 1) to decrease significantly from marginal" << endl;
    cout << "looking for p(x3 = 1) = 0.99" << endl;
    cout << "looking for p(x4 = 1) = 0.99" << endl;
    
    cout << "X4 = 0 ======================================================" << endl;
    obs.clear();
    obs[4] = 0;
    P = clt.sumProduct (obs);
    for (it=P.begin(); it!=P.end(); it++) {
        cout << "[cl tree]\t\tP(X" << it->first << "=1) = " << it->second << endl;
    }
    cout << "looking for p(x4 = 1) = 0.01" << endl;
    cout << "looking for p(x3 = 1) to decrese slightly (sibling messages durring root to leaf)" << endl;
        
}

void
PVNMapBuilder::run_build_chow_liu_tree () {
    
    
    //perllcm::pvn_eview_map_t eview_map;
    
    //char fmap[PATH_MAX];
    //sprintf (fmap, "%s/%s.evmap", map_dir, map_name);
    //cout << "[eview map]\tTrying to load existing map: " << fmap << endl;
    //int32_t ret = pvnu_lcm_fread <perllcm::pvn_eview_map_t> (fmap, &eview_map);
    //if (ret < 0) {
    //    cout << "[cl tree]\tCouldn't read " << fmap << " from disk. ret=" << ret << endl;
    //    exit (-1);
    //}
    //cout << "[cl tree]\tLoaded existing map: " << fmap << endl;
    
    // test
    test_sum_product ();

    //int64_t tic;
    //tic = timestamp_now();
    //PVNCLTree clt;
    //clt.make (eview_map.match_data);
    //clt.print_tree ();
    //cout << "[cl tree]\tBuilt Chow Liu Tree: ( " << (timestamp_now() - tic)/1e6 << " sec.)" << endl;
    
    
}
