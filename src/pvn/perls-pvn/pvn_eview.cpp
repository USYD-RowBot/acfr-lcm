#include <stdio.h>
#include <stdlib.h>

#include <gsl/gsl_cdf.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-math/ssc.h"
#include "perls-math/gsl_util.h"


#include "pvn_eview.h"


void
pvn_eview_beta_prior_coefs(double mode_target, double strength, double *a, double *b) {
    
    *a = ((strength-2)*mode_target+1);
    *b = ((strength-1)-(strength-2)*mode_target);
    
    if (*a < 1.0 || *a > (strength-1.0))
        ERROR ("Invalid alpha = %lf, target = %lf, strength = %lf", *a, mode_target, strength);
    if (*b < 1.0 || *b > (strength-1.0))
        ERROR ("Invalid beta = %lf target = %lf, strength = %lf", *b, mode_target, strength);
    
}

double
pvn_eview_calc_norm_inlier_score (int obs_nf, int exemplar_nf, int num_inliers) {
    
    int min_in = min (obs_nf, exemplar_nf);
    double nis = (double)num_inliers/(double)min_in;
    
    if (nis < 0.0 || nis > 1.0)
        ERROR ("Bad normalized inlier score: %lf", nis);
    
    return nis;
}

double
soft_evidence_likelihood (double q, vector<double> z, double a, double b,
                          double a0, double b0, double a1, double b1) {
    double l;
    double pZgQ = 0.0;
    for (size_t i=0; i<z.size(); i++)
        pZgQ = pZgQ + log(gsl_ran_beta_pdf(z[i], a0, b0)*(1-q) + gsl_ran_beta_pdf(z[i], a1, b1)*(q));
    
    double pQ = gsl_ran_beta_pdf(q, a, b);    

    l = pZgQ + log(pQ);

    return l;
}

double
nd_soft_evidence_likelihood (double q, vector<double> z, double a, double b,
                             double a0, double b0, double a1, double b1) {
    
    double dq = fmax(abs(1e-4*q), 1e-6);
    
    double y = soft_evidence_likelihood (q, z, a, b, a0, b0, a1, b1);
    double y_prime = soft_evidence_likelihood (q+dq, z, a, b, a0, b0, a1, b1);
    
    return (y_prime-y)/dq;
}

double
soft_evidence_map_gradient_descent (vector<double> z, double a, double b,
                                    double a0, double b0, double a1, double b1) {

    int i = 0; 
    double q_gd_max = 0.5;
    double q_gd_max_prev = q_gd_max;
    double gamma = 10; 
    double step_size = 1e6; 
    while (i < 100 && step_size > 1e-3) {
        q_gd_max_prev = q_gd_max;

        //get gradient
        double g = nd_soft_evidence_likelihood(q_gd_max, z, a, b, a0, b0, a1, b1);
        
        while (fabs(q_gd_max - (q_gd_max + gamma*g)) > fmax (q_gd_max, 1.0-q_gd_max)/2.0)
            gamma = gamma/1.1;
        
        q_gd_max = q_gd_max + gamma*g;
        step_size = fabs(q_gd_max - q_gd_max_prev);
        
        if (q_gd_max >= 1.0 || q_gd_max <= 0.0) {
            gamma = gamma/1.1;
            // remove effects of this trial
            q_gd_max = q_gd_max_prev;
        }
        
        //printf("ittr %d, step norm = %0.7lf, q = %0.2lf \n", i, step_size, q_gd_max);
        i = i+1;
    }


    //printf("Estimate %0.3lf, Finished in %d ittr, step norm = %0.7lf \n", q_gd_max, i, step_size);
    //cout << "Z == ";
    //for (size_t i=0; i<z.size(); i++) 
    //    cout << z[i] << " ";
    //cout << endl;

    if (isnan (q_gd_max))
        ERROR ("q_map is nan!");
    if (q_gd_max > 1.0 || q_gd_max < 0.0)
        ERROR ("Bad q_map = %lf", q_gd_max);    
        

    return q_gd_max;
    
}


map<int64_t, float>
pvn_eview_run_data_2_obs_soft (perllcm::pvn_eview_map_match_data_t *md, set<int64_t> *var_ids) {
    
    
    double a, b, a0, b0, a1, b1;
    // for match_min_inliers = 20
    //pvn_eview_beta_prior_coefs(0.20, 2.2, &a, &b); // beta coefs for p(q)
    pvn_eview_beta_prior_coefs(0.20, 2.2, &a, &b); // beta coefs for p(q)
    pvn_eview_beta_prior_coefs(0.0, 90, &a0, &b0); //beta coefs for p(z) given e=0
    pvn_eview_beta_prior_coefs(0.05, 30, &a1, &b1); //beta coefs for p(z) given e=1

    // collect all measurements
    map<int64_t, vector<double> > z;
    for (int j=0; j<md->n; j++) {
        
        int64_t id = md->exemplar_utimes[j];

        // add this utime to the list of variables (a set so wont add if duplicate)
        if (NULL != var_ids)
            var_ids->insert (id);
        
        double nis = pvn_eview_calc_norm_inlier_score (md->obs_num_feats[j],
                                                       md->exemplar_num_feats[j],
                                                       md->num_inliers[j]);
        z[id].push_back (nis);
    }
    
    // loop over each set of measurements and find the MAP estimate of P
    map<int64_t, float> P;
        
    map<int64_t, vector<double> >::iterator itz;
    for (itz=z.begin(); itz!=z.end(); itz++) {
        P[itz->first] = soft_evidence_map_gradient_descent (itz->second, a, b,
                                                            a0, b0, a1, b1);
    }
    
    return P;
        
}


map<int64_t, bool>
pvn_eview_run_data_2_obs (perllcm::pvn_eview_map_match_data_t *md, set<int64_t> *var_ids) {
    
    map<int64_t, bool> m;

    for (int j=0; j<md->n; j++) {
        
        int64_t id = md->exemplar_utimes[j];
        
        // add this utime to the list of variables (a set so wont add if duplicate)
        if (NULL != var_ids)
            var_ids->insert (id);
        
        // check if this key already exists
        if ( m.find(id) != m.end() ) {
            m[id] |= (bool)md->match_success[j];
        } else {
            m[id] = (bool)md->match_success[j];
        }
        
    }
    
    return m;
        
}

perllcm::pvn_conditions_t
pvn_eview_load_conditions (BotParam *param, char *base_key) {
    

    perllcm::pvn_conditions_t pconds;
    char key[1024];
    int tmp;
    sprintf (key, "%s.sky", base_key);
    tmp = perllcm::pvn_conditions_t::SKY_UNKNOWN;
    bot_param_get_int (param, key, &tmp);
    pconds.sky = (int8_t)tmp;

    sprintf (key, "%s.tod", base_key);
    tmp = perllcm::pvn_conditions_t::TOD_UNKNOWN;
    bot_param_get_int (param, key, &tmp);
    pconds.tod = (int8_t)tmp;

    sprintf (key, "%s.snow", base_key);
    tmp = -1;
    bot_param_get_int (param, key, &tmp);
    pconds.snow = (int8_t)tmp;

    sprintf (key, "%s.foliage", base_key);
    tmp = -1;
    bot_param_get_int (param, key, &tmp);
    pconds.foliage = (int8_t)tmp;
    
    return pconds;
    
}


void
pvn_eview_update_match_data (perllcm::pvn_eview_map_match_data_t *md,
                             perllcm::pvn_eview_map_neighborhood_t *mn,
                             vector<int8_t> match_success, vector<int> inlier_cnts, 
                             int64_t obs_utime, int obs_num_feats,
                             vector<int> exemplar_num_feats,
                             vector<float> reproj_error,
                             vector<bool> mask) {
     
    for (int i=0; i<mn->ne; i++) {
        
        // dont add matches that are masked out
        if (mask.size() == (size_t)mn->ne && !mask[i]){
cout << "============= SKIPPING ===================================" << endl;
            continue;
        }
        
        int64_t utime_exemplar = mn->exemplars[i].utime;
        
        md->exemplar_utimes.push_back (utime_exemplar);
        md->num_inliers.push_back (inlier_cnts[i]);
        md->obs_utimes.push_back (obs_utime);
        md->obs_num_feats.push_back (obs_num_feats);
        md->exemplar_num_feats.push_back (exemplar_num_feats[i]);
        md->reproj_error.push_back (reproj_error[i]);
        md->match_success.push_back (match_success[i]);
        
        if (match_success[i]){
            mn->exemplars[i].utime_last_match = obs_utime;
        }
            
        md->n++;
    }
}

void
pvn_eview_update_predict_data (perllcm::pvn_eview_map_predict_data_collection_t *pdc,
                               perllcm::pvn_eview_map_neighborhood_t *mn,
                               vector<int8_t> match_success, vector<int> inlier_cnts, 
                               map<int64_t, float> P, int64_t obs_utime,
                               perllcm::pvn_conditions_t obs_conds, int obs_num_feats,
                               vector<int> exemplar_num_feats,
                               vector<float> reproj_error) {

    // add an entry in the predict data
    perllcm::pvn_eview_map_predict_data_t pd;
    pd.obs_utime = obs_utime;
    pd.obs_conds = obs_conds;
    pd.obs_num_feats = obs_num_feats;
    pd.neighborhood_id = mn->id;
    pd.ne = mn->ne;
    for (int j=0; j<pd.ne; j++) {
        
        int64_t utime_e = mn->exemplars[j].utime;
        pd.exemplar_utimes.push_back (utime_e);
        pd.exemplar_num_feats.push_back (exemplar_num_feats[j]);
        pd.reproj_error.push_back (reproj_error[j]);
        pd.match_success.push_back(match_success[j]);
        pd.inlier_cnts.push_back(inlier_cnts[j]);
        
        if (P.find (utime_e)  != P.end ())
            pd.p_observations.push_back(P[utime_e]);
        else
            pd.p_observations.push_back(-1.0);
        
    }
    pdc->n++;
    pdc->predict_data.push_back (pd);
    
}

vector<int8_t>
pvn_eview_calc_match_success_v (vector<int> inlier_cnts, vector<float> reproj_error,
                              int min_inliers, float max_reproj_error) {
    
    vector<int8_t> match_success;
    
    
    for (size_t i=0; i<inlier_cnts.size(); i++) {
    
        match_success.push_back (pvn_eview_calc_match_success(inlier_cnts[i],
                                                              reproj_error[i],
                                                              min_inliers,
                                                              max_reproj_error));
            
    }
    
    return match_success;
}

int8_t
pvn_eview_calc_match_success(int inlier_cnt, float reproj_error,
                             int min_inliers, float max_reproj_error) {
    
    int8_t match_success;
        
    if (inlier_cnt >= min_inliers && reproj_error <= max_reproj_error)
        match_success = 1;
    else
        match_success = 0;
        
    return match_success;
    
}


