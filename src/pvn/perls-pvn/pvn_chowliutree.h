#pragma once

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <string>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <valarray>
#include <numeric>
#include <algorithm>
#include <iterator>

using namespace std;

//utilities
#include <ctime>
#include <cmath>
using std::min; using std::max;

#include "perls-lcmtypes++/perllcm/pvn_eview_map_t.hpp"


//-------------##TRAINING DATA##---------------
// training data used to create the chow-liu tree.
// after the tree is made all the important data is then stored in the tree
// itself and the training data is no longer needed therefore creating the
// chowliu tree is the only place is is used
class PVNCLTrainData {

private:

    set <int64_t> var_ids; //variable ids
    map <int64_t, float> marginals;
    int num_samples;
    
    bool use_prior;
    double prior_alpha;
    double prior_beta;
    
    // each row represents a sample each sample is a map where an id maps to
    // a boollean if it has been observered or not data which we have not
    // attempted to observe will not be in the map
    vector< map<int64_t, bool> > data;
    
    void
    make_marginals();

public:

    PVNCLTrainData(); ~PVNCLTrainData();

    int makeTrainingData(const vector<perllcm::pvn_eview_map_match_data_t> &match_data,
                         double prior_alpha, double prior_beta);

    set <int64_t>
    get_var_ids ();
    
    int
    get_num_ids ();

    // these are the interface functions that the training data needs to
    // provide
    double
    P(int64_t id, bool val);
    
    double
    JP(int64_t ida, bool aval, int64_t idb, bool bval); //a & b
    
    double
    CP(int64_t ida, bool aval, int64_t idb, bool bval); // a | b
};

//-------------##CHOW-LIU TREE##---------------//

// tree is a vector of these nodes
class clNode {
 
  public:
    int64_t id;
    int64_t pid;
    vector<int64_t> cids;
    float Pq;       // probability node q = true
    float Pq_p;     // probability node q = true | parrent = true
    float Pq_np;    // probability node q = true | parrent = false
    int depth;      // depth in tree
    float p_mi;     // mutual information with parent
    float p_rho;    // correlation coef with parent
    
    // message storage for sum-product algorithm
    vector< vector<float> > u_xc_self;
    vector<float> u_xp_self;
    float observation; // lessthan 0 for unobserved, otherwise p(q) = 1
    
    bool
    is_root () {
        if (pid == -1)
            return true;
        else
            return false;
    };
    
    bool
    is_leaf () {
        if(cids.size() == 0)
            return true;
        else    
            return false;
    };
    
    void
    clear_msgs () {
        u_xc_self.resize(0);
        u_xp_self.resize(2);
        u_xp_self[0] = 0;
        u_xp_self[1] = 0;
        observation = -1.0;
    }
    
    bool
    all_child_msgs() {
        return (u_xc_self.size() == cids.size());
    }
    
};

typedef struct _mutual_information {

    float mi;
    float rho;
    int64_t id1;
    int64_t id2;

} mutual_information;

class PVNCLTree {

private:

    //has to be built recursively due to underlying tree structure
    void
    recAddToTree(int64_t, int64_t, int, double, double, PVNCLTrainData & train_data, list<mutual_information> &);
    
    static bool
    clNodeCompare(const clNode &first, const clNode &second) ;
    
    static bool
    sortInfoScores(mutual_information &first, mutual_information &second);
    
    double
    calcMutInfo(PVNCLTrainData &train_data, int64_t id1, int64_t id2);
    
    double
    calcRho(PVNCLTrainData &train_data, int64_t id1, int64_t id2);
    
    void
    createBaseEdges(list<mutual_information> &edges, PVNCLTrainData &train_data,
                    double info_threshold, set < pair<int64_t, int64_t> > mask);
    
    int
    reduceEdgesToMinSpan(list<mutual_information> &edges, PVNCLTrainData &train_data);

    void
    recPassMsgToRoot(vector <int64_t> proc_ids);
    
    void
    recPassMsgToLeaves(int64_t id);

    // save the edges if we want to comeback and compare specific nodes later
    list<mutual_information> edges_save;

    int64_t root_id;
    vector <int64_t> leaf_ids;
    
    void
    clear_msgs () {
        map<int64_t, clNode>::iterator it;
        for (it=tree.begin(); it!=tree.end(); it++) {
            it->second.clear_msgs();
        }
    }

public:

    //tree
    map<int64_t, clNode> tree;

    //constructors
    PVNCLTree(); ~PVNCLTree();

    //make
    int
    make(const vector <perllcm::pvn_eview_map_match_data_t> &match_data,
         double info_threshold = 0, double prior_alpha = -1, double prior_beta = -1,
         set < pair<int64_t, int64_t> > mask = set < pair<int64_t, int64_t> >());
    
    void
    print_tree (void);
    
    void
    print_tree_to_file (char *file);

    //test function delete later
    void
    find_max_mi (set<int64_t> exemplars);
    
    // given a list of observations find probabilities of all unobserved variables
    map<int64_t, float>
    sumProduct (map<int64_t, bool> obs);
    
    map<int64_t, float>
    sumProduct (map<int64_t, float> obs);

};
