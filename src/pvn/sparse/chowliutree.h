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

#include "eigen_utils.h"
#include <Eigen/LU>

using namespace std;
using namespace Eigen;

//utilities
#include <ctime>
#include <cmath>

//-------------##TRAINING DATA##---------------
// training data used to create the chow-liu tree.
// after the tree is made all the important data is then stored in the tree
// itself and the training data is no longer needed therefore creating the
// chowliu tree is the only place is is used
class CLTInformation {

private:

    MatrixXd L;
    int state_len;
    set <int64_t> var_ids; //variable ids
    map <int64_t, vector<int> > id2inds;
    
    

public:
    

    CLTInformation (const MatrixXd &_L, int _state_len)
        : L(_L), state_len(_state_len)
    {
        int n_states = L.rows()/state_len;
        for (int i=0; i<n_states; i++) {
            var_ids.insert(i);
            
            vector<int> tmp;
            for (int j=0; j<state_len; j++)
                tmp.push_back(i*6 + j);
                
            id2inds[i] = tmp;
        }
            
    }
    
    ~CLTInformation() {
        
    }
    
    int
    get_num_ids ();
    
    set <int64_t>
    get_var_ids ();

    MatrixXd
    marginal(int64_t id);
    
    MatrixXd
    joint(int64_t ida, int64_t idb); // a, b
    
    MatrixXd
    conditional(int64_t ida, int64_t idb); // a | b

};

//-------------##CHOW-LIU TREE##---------------//

// tree is a vector of these nodes
class CLTNode {
 
  public:
    int64_t id;
    int64_t pid;
    vector<int64_t> cids;
    MatrixXd marginal;       // marginal information
    MatrixXd conditional;    // conditional information with parrent
    MatrixXd joint;          // joint information with parrent
    int depth;      // depth in tree
    float p_mi;     // mutual information with parent
  
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
    
};

typedef struct _mutual_information {

    float mi;
    int64_t id1;
    int64_t id2;

} mutual_information;

class CLTree {

private:

    //has to be built recursively due to underlying tree structure
    void
    recAddToTree(int64_t, int64_t, int, double, CLTInformation &, list<mutual_information> &);
    
    static bool
    clNodeCompare(const CLTNode &first, const CLTNode &second) ;
    
    static bool
    sortInfoScores(mutual_information &first, mutual_information &second);
    
    double
    calcMutInfo(CLTInformation &clt_info, int64_t id1, int64_t id2);
    
    void
    createBaseEdges(list<mutual_information> &edges, CLTInformation &clt_info);
    
    int
    reduceEdgesToMinSpan(list<mutual_information> &edges, CLTInformation &clt_info);

    // save the edges if we want to comeback and compare specific nodes later
    list<mutual_information> edges_save;

    int64_t root_id;
    vector <int64_t> leaf_ids;

public:

    //tree
    map<int64_t, CLTNode> tree;

    //constructors
    CLTree(); ~CLTree();

    //make
    int
    make(const MatrixXd &L, int state_len);
    
    void
    print_tree (void);
    
    void
    print_tree_to_file (char *file);

    //test function delete later
    void
    find_max_mi (set<int64_t> exemplars);

};
