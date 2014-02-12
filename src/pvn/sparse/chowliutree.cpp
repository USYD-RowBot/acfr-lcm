#include "perls-common/error.h"
#include "perls-math/gsl_util_rand.h"

#include "chowliutree.h"



//-------------##CHOW-LIU TREE##---------------//

CLTree::CLTree() {};

CLTree::~CLTree() {};


void
CLTree::find_max_mi (set<int64_t> ids) {

    list<mutual_information> edges_tmp;
    edges_tmp.clear();
    // look through edges_save and move them to edges if they connect two nodes in ids 
    list<mutual_information>::iterator it;
    for (it = edges_save.begin(); it != edges_save.end(); ++it) {
        if (ids.count(it->id1) && ids.count(it->id2)) {
            edges_tmp.push_back(*it);
        }
    }
    // sort the edges
    edges_tmp.sort(sortInfoScores);
    // print out top ten
    int count = 0;
    for (it = edges_tmp.begin(); it != edges_tmp.end() && count < 10; ++it, ++count) {
        cout << it->id1 << " <--> " <<  it->id2 << ", MI =  " << it->mi << endl;
    }
}

int
CLTree::make(const MatrixXd &L, int state_len) {
        
    //make the training data
    CLTInformation clt_info(L, state_len);
    
    // make sure we have at least two nodes otherwise return a trival tree
    if (clt_info.get_num_ids() == 1) {

        set <int64_t> ids = clt_info.get_var_ids ();

        int64_t id =  *ids.begin();
        CLTNode new_node;
        new_node.id = id;
        new_node.pid = -1;
        new_node.marginal = clt_info.marginal(id);
        new_node.conditional = new_node.marginal;
        new_node.joint = new_node.marginal;
        new_node.depth = 0;
        new_node.p_mi = 0.0;
        root_id = new_node.id;

        tree[new_node.id] = new_node;
        
        return 0;
    }

//clt_info.pin_L(); // hack to insure not singular

    //calculate the parent nodes based on maximising mutual information
    list<mutual_information> edges;
    createBaseEdges(edges, clt_info);
    edges_save = edges;
    if(reduceEdgesToMinSpan(edges, clt_info)) return -1;
    
//clt_info.unpin_L(); // hack to insure not singular
    
    //recursively build the tree into correct data structure
    tree.clear();
    recAddToTree (edges.front().id1, edges.front().id2, 0, edges.front().mi, clt_info, edges);
    
    return 0;
}

void
CLTree::print_tree (void) {
    
    cout << "[cl tree]\tFinal Tree:" << endl;
    map<int64_t, CLTNode>::iterator it;
    for (it=tree.begin(); it!=tree.end(); it++) {
        if (it->second.is_root())
            cout << "[cl tree]\t\tP(" << it->second.id << ")" << endl;
        else
            cout << "[cl tree]\t\tP(" << it->second.id << "|" << it->second.pid << ")" << endl;
    }
    cout << "[cl tree]\t\tRoot = " << root_id << endl;
    cout << "[cl tree]\t\tLeaves = ";
    for (size_t i=0; i<leaf_ids.size(); i++) cout << leaf_ids[i] << " ";
    cout << endl;
}

void
CLTree::print_tree_to_file (char *file) {
    
    ofstream out;
    out.open(file);
    
    if (!out.is_open()) {
        ERROR ("Failed to open %s", file);
        return;
    }
    
    map<int64_t, CLTNode>::iterator it;
    for (it=tree.begin(); it!=tree.end(); it++) {
        out << it->second.id << ", " << it->second.pid << ", " << it->second.p_mi << endl;
    }
    
    out.close();
}



void
CLTree::recAddToTree(int64_t id, int64_t pid, int depth, double mi,
                        CLTInformation &clt_info, list<mutual_information> &edges) {
        
    CLTNode new_node;
//cout << "Adding REC =============== "  << endl;
    // arbitrarily choose second id as parrent
    if (depth == 0) { // add root node
        new_node.id = id;
        new_node.pid = -1;
        new_node.marginal = clt_info.marginal(id);
        new_node.conditional = new_node.marginal;
        new_node.joint = new_node.marginal;
        new_node.depth = depth;
        new_node.p_mi = 0.0;
        root_id = new_node.id;
    } else {
        new_node.id = id;
        new_node.pid = pid;
        new_node.marginal = clt_info.marginal(id);;
        new_node.conditional = clt_info.conditional(id, pid);
        new_node.joint = clt_info.joint(id, pid);
        new_node.depth = depth;
        new_node.p_mi = mi;
    }
    
    //find all children and do the same
    vector<int64_t> cids;
    vector<double> mis;
    list<mutual_information>::iterator edge = edges.begin();
    while(edge != edges.end()) {
        if(edge->id1 == new_node.id) {
            cids.push_back(edge->id2);
            mis.push_back(edge->mi);
            edge = edges.erase(edge);
            continue;
        }
        if(edge->id2 == new_node.id) {
            cids.push_back(edge->id1);
            mis.push_back(edge->mi);
            edge = edges.erase(edge);
            continue;
        }
        edge++;
    }
    for(size_t i=0; i < cids.size(); i++) {
        new_node.cids.push_back(cids[i]);
        recAddToTree(cids[i], new_node.id, depth+1, mis[i], clt_info, edges);
    }
    if (cids.size() == 0)
        leaf_ids.push_back(new_node.id);
    
    tree[new_node.id] = new_node;
}

bool
CLTree::sortInfoScores(mutual_information &first, mutual_information &second)  {
        return first.mi > second.mi;
}

double
CLTree::calcMutInfo(CLTInformation &clt_info, int64_t id1, int64_t id2) {

    MatrixXd L_1g2 = clt_info.conditional (id1, id2);
    MatrixXd L_1 = clt_info.marginal (id1);
    
    // use pdet 
    //double ldL_1g2 = plogdet(L_1g2);
    //double ldL_1 = plogdet(L_1);
    //double mi = 0.5*(ldL_1g2 - ldL_1);
    
    // use normal det, must be pinned
    double ldL_1g2 = log ((L_1g2 +  MatrixXd::Identity(L_1g2.rows(), L_1g2.cols())).determinant());
    double ldL_1 = log ((L_1 +  MatrixXd::Identity(L_1.rows(), L_1.cols())).determinant());
    double mi = 0.5*(ldL_1g2 - ldL_1);

    //cout << "DET TEST ===================================================================== " << endl;
    //cout << test_mi << " - " << mi << endl;
    //cout << id1 << " " << id2 << ", mi diff = " << fabs(test_mi - mi) << endl;
    
    return mi;
    
    //gsl_rng *rng = gslu_rand_rng_alloc ();
    //double n = gslu_rand_uniform (rng);
    //gsl_rng_free (rng);
    //return n;
} 


void
CLTree::createBaseEdges(list<mutual_information> &edges, CLTInformation &clt_info)  {
        
    set <int64_t> var_ids = clt_info.get_var_ids();
    int nvars = var_ids.size();
    double average = 0;
    double npairs = floor(pow((double)nvars, 2.0) / 2) -  floor((double)nvars/2);
    mutual_information mi_tmp;

    cout << "[cl tree]\tCalculating the Mutual Information." << endl;
    
    if (nvars < 2)
        return;
    
    set<int64_t>::iterator it1;
    set<int64_t>::iterator it2;
    for (it1 = var_ids.begin(); it1 != var_ids.end(); ++it1) {
        for (it2 = it1; ++it2 != var_ids.end();) {
    
            mi_tmp.id1 = (*it1);
            mi_tmp.id2 = (*it2);
                    
            mi_tmp.mi = (float)calcMutInfo(clt_info, mi_tmp.id1, mi_tmp.id2);
            
            edges.push_back(mi_tmp);
            average += mi_tmp.mi;
        }
    }

    edges.sort(sortInfoScores);
    cout << "[cl tree]\t\tMinimum Information: " << edges.back().mi;
    cout << " (" << edges.back().id1 << "," << edges.back().id2 << ")" << endl;
    cout << "[cl tree]\t\tMaximum Information: " << edges.front().mi;
    cout << " (" << edges.front().id1 << "," << edges.front().id2 << ")" << endl;
    cout << "[cl tree]\t\tAverage Information: " << average / npairs << endl;
    cout << "[cl tree]\t\tTop 10 MI pairs:" << endl;
    list<mutual_information>::iterator it;
    int count = 0;
    for (it = edges.begin(); it != edges.end() && count < 10; ++it, ++count) {
        cout << "[cl tree]\t\t"<< it->id1 << " <--> " <<  it->id2 << ", MI =  " << it->mi << endl;
    }



}

int
CLTree::reduceEdgesToMinSpan(list<mutual_information> &edges, CLTInformation &clt_info) {

    //initialise groups
    set <int64_t> var_ids = clt_info.get_var_ids();
    
    map<int64_t, int> groups;
    map<int64_t, int>::iterator groupIt;
    
    set <int64_t>::iterator  idIt;
    int i=0;
    for (idIt = var_ids.begin(); idIt != var_ids.end(); ++idIt) {
        groups[(*idIt)] = i; // assign each id to a different group initially
        ++i;
    }
    
    int group1, group2;
    double average = 0;

    cout << "[cl tree]\tReducing List to Minimum Spanning Tree" << endl;
    
    set<int64_t>::iterator it1;
    list<mutual_information>::iterator edge = edges.begin();
    
    while(edge != edges.end()) {
        
        if(groups[edge->id1] != groups[edge->id2]) {
            group1 = groups[edge->id1];
            group2 = groups[edge->id2];
            
            // merge group2 into group 1
            for(groupIt = groups.begin(); groupIt != groups.end(); groupIt++)   
                    if(groupIt->second == group2) groupIt->second = group1;
            
            average += edge->mi;
            edge++;
        } else {
            edge = edges.erase(edge);
        }
    }

    if(edges.size() != var_ids.size() - 1) {
        cout << "Not enough edges to complete the spanning tree. Decrease "
                "the information threshold, or remove edge mask to increase edges. " 
                "Tree not built." << endl;
        return -1;
    }

    cout << "[cl tree]\t\tMinimum Information: " << edges.back().mi;
    cout << " (" << edges.back().id1 << "," << edges.back().id2 << ")" << endl;
    cout << "[cl tree]\t\tMaximum Information: " << edges.front().mi;
    cout << " (" << edges.front().id1 << "," << edges.front().id2 << ")" << endl;
    cout << "[cl tree]\t\tAverage Information: " << average / edges.size() << endl;

    return 0;

}

//-------------##TRAINING DATA##---------------//

set <int64_t>
CLTInformation::get_var_ids () {
    return var_ids;
}

int
CLTInformation::get_num_ids () {
    return var_ids.size();
}

MatrixXd
CLTInformation::marginal(int64_t id) {

    vector<int> iia(0);
    vector<int> iib(0);

//cout << "L: " << endl << L << endl;
    
    iia.insert (iia.end(), id2inds[id].begin(), id2inds[id].end() );
    
    set <int64_t>::iterator  idIt;
    for (idIt = var_ids.begin(); idIt != var_ids.end(); ++idIt) {
        
        if (id != *idIt)
            iib.insert (iib.end(), id2inds[*idIt].begin(), id2inds[*idIt].end() );
    }
    
    if (iib.size() > 0) {
  
        MatrixXd Lbb = L(iib, iib);    
        MatrixXd Lbbinv = posdef_pinv(Lbb); 
        return L(iia, iia) - L(iia, iib) * Lbbinv * L(iib, iia);
        //return L(iia, iia) - L(iia, iib) * L(iib, iib).inverse() * L(iib, iia);
        
    } else
        return L(iia, iia);
}

MatrixXd
CLTInformation::joint(int64_t ida, int64_t idb) {
    
    vector<int> iia(0);
    vector<int> iib(0);
   
    iia.insert (iia.end(), id2inds[ida].begin(), id2inds[ida].end() );
    iia.insert (iia.end(), id2inds[idb].begin(), id2inds[idb].end() );

    set <int64_t>::iterator  idIt;
    for (idIt = var_ids.begin(); idIt != var_ids.end(); ++idIt) {
        if (ida != *idIt && idb != *idIt)
            iib.insert (iib.end(), id2inds[*idIt].begin(), id2inds[*idIt].end() );
    }

    if (iib.size() > 0) {
  
        MatrixXd Lbb = L(iib, iib);    
        MatrixXd Lbbinv = posdef_pinv(Lbb);
        return L(iia, iia) - L(iia, iib) * Lbbinv * L(iib, iia);
        //return L(iia, iia) - L(iia, iib) * L(iib, iib).inverse() * L(iib, iia);
        
    } else
        return L(iia, iia);
}

MatrixXd
CLTInformation::conditional(int64_t ida, int64_t idb) {

    MatrixXd Lj = joint(ida, idb);

    return Lj.block(0, 0, state_len, state_len);
}
