#ifndef __PERLS_PVN_SPARSE_H__
#define __PERLS_PVN_SPARSE_H__

#include <iostream>
#include <fstream>
#include <algorithm>
#include <map>
#include <set>
#include <deque>

#include <boost/filesystem.hpp>
#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>
#include "eigen_utils.h"
#include <isam/isam.h>

#include "perls-lcmtypes++/perllcm/pose3d_collection_t.hpp"
#include "perls-lcmtypes++/perllcm/matrix_d_t.hpp"

//#include "perls-lcmtypes++/perllcm/isam_request_state_t.hpp"
//#include "perls-lcmtypes++/perllcm/isam_return_state_t.hpp"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/magic.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/units.h"
#include "perls-math/ssc.h"

#include "perls-pvn/pvn_util.h"
#include "perls-isam/gparser.h"
#include "perls-isam/isam_util.h"
#include "perls-isam/user_factors.h"
#include "perls-isam/glc_factors.h"

#include "chowliutree.h"

#define DEBUG 0

#define EPS 1e-8

#define MARG_EVERY_NTH "3"

#define BATCH_EVERY 200
#define SLAM_BATCH_MAX_ITTR 200
#define SLAM_MOD_UPDATE 1                   // default 1
#define SLAM_MOD_BATCH 100                  // default 100
#define SLAM_MOD_SOLVE 1                    // default 1
#define SLAM_METHOD GAUSS_NEWTON            // default
//#define SLAM_METHOD LEVENBERG_MARQUARDT
//#define SLAM_METHOD DOG_LEG

class PVNSparse {
    private:
        lcm::LCM lcm;
        
        BotParam *param;
        getopt_t *gopt;
        
        char *channel_isam_request_state;
        char *channel_isam_return_state;
        
        char *dir;  // the working directory
        char *name;  // the name of the experiment
        
        bool clt_sparse;
        bool world_lin;
        bool rs_inc_x_o_w;
        bool add_meas_after;
        
        int marg_every;
        int keep_every;
        
        void
        run_baseline_marginalization (void);
        
        void
        run_compound_marginalization (void);
        
        void
        run_lifting_marginalization (void);
        
        void
        run_lifting_marginalization_glc (void);
        
        
        void
        run_gen_simple_graph (void);
                
        void
        load_isam_graph (isam::Slam *slam, map<int64_t,
                         isam::Pose3d_Node*> *nodes);
        

    public:
        
        bool is_done;
        
        PVNSparse (int argc, char *argv[]);
        ~PVNSparse ();
        
        void
        run ();        
};

void
write_matrix (char* file, Eigen::MatrixXd mat);

void
write_inds (char* file, vector<int> inds);

Eigen::MatrixXd
load_matrix (char* file);

vector<int>
load_inds (char* file);

void
write_meas_to_add (char * file, vector<Factor*> factors);

vector<Factor*>
load_meas_to_add (char* file, map<int64_t, Pose3d_Node*> &pose3d_nodes);

void
write_pc (char *file, Slam *slam, vector<int> *inds_to_keep);

void
write_markov_pairs (char *file, Slam *slam);

void
setup_isam (Slam *slam);

void
caculate_and_write_results (Slam *slam, char *dir, char *name_prefix, char *exp_name, double sec);

// shared between lifting code
vector< isam::Node* >
get_marginalization_clique_nodes (Slam *slam, Pose3d_Node *marg_nod);

MatrixXd
get_marginalization_clique_information (Slam *slam, Pose3d_Node *marg_node,
                                        vector<Node*>& clique_nodes,
                                        vector<Factor*> *ic_factors);


vector<Factor*>
get_intra_clique_factors (vector<Node*> clique_nodes, Node *m_node);


void
check_valid_noise (MatrixXd A); 

#endif // __PERLS_PVN_SPARSE_H__
