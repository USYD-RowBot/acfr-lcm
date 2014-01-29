#include "perls-common/error.h"

#include "pvn_chowliutree.h"
#include "pvn_eview.h"
#include "pvn_util.h"

#define PVN_CLT_POS_EVIDENCE 0.99
#define PVN_CLT_NEG_EVIDENCE 0.01
#define PVN_CLT_CP_JP_PRIOR_STRENGHT 4

vector <float> 
safe_normalize (vector <float> p) {
    double sum = p[0] + p[1];
    vector <float> p_out (2,0);
    p_out[0] = p[0]/sum;
    p_out[1] = p[1]/sum;
    
    // bad things happen when we start assiging 0 probabilities to events
    if (p_out[0] > 0.9999)
        p_out[0] = 0.9999;
    if (p_out[1] > 0.9999)
        p_out[1] = 0.9999;
    if (p_out[0] < 0.0001)
        p_out[0] = 0.0001;
    if (p_out[1] < 0.0001)
        p_out[1] = 0.0001;
        
    return p_out;
}


//-------------##CHOW-LIU TREE##---------------//

PVNCLTree::PVNCLTree() {};

PVNCLTree::~PVNCLTree() {};

void
PVNCLTree::recPassMsgToRoot(vector<int64_t> proc_ids) {
    
    // since we have no factor with more than pairwise connections we
    // simplify and think about messages between nodes
    
    set<int64_t> proc_ids_next;
    proc_ids_next.clear();
    
    for (size_t i=0; i<proc_ids.size(); i++) {
        
        int64_t id = proc_ids[i];
        int64_t pid = tree[id].pid;
        
        float P_x0_xp0 = 1.0 - tree[id].Pq_np;
        float P_x0_xp1 = 1.0 - tree[id].Pq_p;
        float P_x1_xp0 = tree[id].Pq_np;
        float P_x1_xp1 = tree[id].Pq_p;

        // message child node to parent node
        //  = product of factor and messages from all children, summed over current node from this factor    
        vector<float> u_x_xp (2,0);  // message [u_x_xp(xp=0), u_x_xp(xp=1)]
        if (tree[id].is_leaf()) { // no previous messages

            //if (tree[id].observation == 1) {
            //    // u_x_xp = factor child state = true
            //    u_x_xp[0] = P_x1_xp0;
            //    u_x_xp[1] = P_x1_xp1;  
            //} else if (tree[id].observation == 0) {
            //    // u_x_xp = factor child state = false
            //    u_x_xp[0] = P_x0_xp0;
            //    u_x_xp[1] = P_x0_xp1;
            if (tree[id].observation >= 0.0) {
                // u_x_xp = factor summed over all child states
                u_x_xp[0] = P_x0_xp0*(1.0 - tree[id].observation) + P_x1_xp0*tree[id].observation; 
                u_x_xp[1] = P_x0_xp1*(1.0 - tree[id].observation) + P_x1_xp1*tree[id].observation; 
            } else {
                // u_x_xp = factor summed over all child states
                u_x_xp[0] = P_x0_xp0 + P_x1_xp0; // should be 1 when child isnt observed, this just shows why
                u_x_xp[1] = P_x0_xp1 + P_x1_xp1; // should be 1 when child isnt observed, this just shows why
            }
            tree[pid].u_xc_self.push_back (u_x_xp); // save message in parents queue
            if (!tree[pid].is_root()) {
                proc_ids_next.insert (pid);  // add parent to the to_process list
            }
            
        } else if ( tree[id].all_child_msgs() ) {

            // take product over all child messages
            u_x_xp[0] = 1; u_x_xp[1] = 1;
            for (size_t j=0; j<tree[id].u_xc_self.size(); j++) {
                u_x_xp[0] *= tree[id].u_xc_self[j][0];
                u_x_xp[1] *= tree[id].u_xc_self[j][1];
                u_x_xp = safe_normalize (u_x_xp);
            }
            
            // now multiply by factor and sum over current node
            //if (tree[id].observation == 1) {
            //    // u_x_xp = factor child states = true
            //    u_x_xp[0] = u_x_xp[0]*P_x1_xp0;
            //    u_x_xp[1] = u_x_xp[1]*P_x1_xp1;
            //} else if (tree[id].observation == 0) {
            //    // u_x_xp = factor child state = false
            //    u_x_xp[0] = u_x_xp[0]*P_x0_xp0;
            //    u_x_xp[1] = u_x_xp[1]*P_x0_xp1;
            if (tree[id].observation >= 0.0)  {    
                u_x_xp[0] = u_x_xp[0]*P_x0_xp0*(1.0 - tree[id].observation) + u_x_xp[0]*P_x1_xp0*tree[id].observation;
                u_x_xp[1] = u_x_xp[1]*P_x0_xp1*(1.0 - tree[id].observation) + u_x_xp[1]*P_x1_xp1*tree[id].observation;
            } else {
                // u_x_xp = factor summed over all child states
                u_x_xp[0] = u_x_xp[0]*P_x0_xp0 + u_x_xp[0]*P_x1_xp0;
                u_x_xp[1] = u_x_xp[1]*P_x0_xp1 + u_x_xp[1]*P_x1_xp1;
            }
            u_x_xp = safe_normalize (u_x_xp);
             
            tree[pid].u_xc_self.push_back (u_x_xp); // save message in parents queue           
            if (!tree[pid].is_root()) {
                proc_ids_next.insert (pid);  // add parent to the to_process list
            }
            
            // it is possible that a parent will receive its last message from
            // its children right before this. that last child will have added the parrent 
            // to the run next list, but now we are done with it. so make sure to remove it
            proc_ids_next.erase (id);
            
        } else { // not ready yet re-add to queue

            proc_ids_next.insert (id);
        } 
    }
    
    if (proc_ids_next.size() > 0) {
        // put int vector
        vector<int64_t> tmp;
        set<int64_t>::iterator it;
        for (it = proc_ids_next.begin(); it != proc_ids_next.end(); it++) {
            tmp.push_back (*it);   
        }
        recPassMsgToRoot(tmp);
    }
    
}

void
PVNCLTree::recPassMsgToLeaves(int64_t id) {

    for (size_t i=0; i<tree[id].cids.size(); i++) {
        
        int64_t cid = tree[id].cids[i];

        vector<float> u_xp_x (2,0);  // message [u_xp_x(x=0), u_xp_x(x=1)]
        if (tree[id].is_root()) {
            
            // for root message
            //if (tree[id].observation == 1) {
            //    u_xp_x[0] = PVN_CLT_NEG_EVIDENCE;
            //    u_xp_x[1] = PVN_CLT_POS_EVIDENCE; 
            //} else if (tree[id].observation == 0) {
            //    u_xp_x[0] = PVN_CLT_POS_EVIDENCE;
            //    u_xp_x[1] = PVN_CLT_NEG_EVIDENCE;
            if (tree[id].observation >= 0.0) {
                u_xp_x[0] = 1.0 - tree[id].observation;
                u_xp_x[1] = tree[id].observation;
            } else {
                u_xp_x[0] = 1.0 - tree[id].Pq;
                u_xp_x[1] = tree[id].Pq;
            }
            
            tree[id].u_xp_self = u_xp_x;
            
        } else {
            u_xp_x = tree[id].u_xp_self;
        }
        
        // also product of all siblings
        for (size_t j=0; j<tree[id].cids.size(); j++) {
            int64_t sid = tree[id].cids[j]; // sibling id
            if (sid != cid) {
               u_xp_x[0] *= tree[id].u_xc_self[j][0];
               u_xp_x[1] *= tree[id].u_xc_self[j][1];
               u_xp_x = safe_normalize (u_xp_x);
            }
        } 
        
        vector<float> u_x_xc (2,0);  // message [u_x_xc(xc=0), u_x_xc(xc=1)]
        // u_xp_x * p(x_c | x) summed over x
        
        float P_c0_x0 = 1.0 - tree[cid].Pq_np;
        float P_c1_x0 = tree[cid].Pq_np;
        float P_c0_x1 = 1.0 - tree[cid].Pq_p;
        float P_c1_x1 = tree[cid].Pq_p;
        
        //if (tree[id].observation == 1) { //x=1
        //    u_x_xc[0] = u_xp_x[1]*P_c0_x1;
        //    u_x_xc[1] = u_xp_x[1]*P_c1_x1;
        //} else if (tree[id].observation == 0) { //x=0
        //    u_x_xc[0] = u_xp_x[0]*P_c0_x0;
        //    u_x_xc[1] = u_xp_x[0]*P_c1_x0;
        if (tree[id].observation >= 0) {
            u_x_xc[0] = u_xp_x[0]*P_c0_x0*(1.0 - tree[id].observation) + u_xp_x[1]*P_c0_x1*tree[id].observation;
            u_x_xc[1] = u_xp_x[0]*P_c1_x0*(1.0 - tree[id].observation) + u_xp_x[1]*P_c1_x1*tree[id].observation;
        } else {
            u_x_xc[0] = u_xp_x[0]*P_c0_x0 + u_xp_x[1]*P_c0_x1;
            u_x_xc[1] = u_xp_x[0]*P_c1_x0 + u_xp_x[1]*P_c1_x1;
        }
        
        u_x_xc = safe_normalize (u_x_xc);
        
        tree[cid].u_xp_self = u_x_xc;

        if (!tree[cid].is_leaf())
            recPassMsgToLeaves(cid);   
    }
    
}

map<int64_t, float>
PVNCLTree::sumProduct (map<int64_t, bool> obs) {
    
    map<int64_t, float> obs_float;
    
    map<int64_t, bool>::iterator itObs;
    for (itObs=obs.begin(); itObs!=obs.end(); itObs++) {
        
        if (itObs->second)
            obs_float[itObs->first] = PVN_CLT_POS_EVIDENCE;
        else
            obs_float[itObs->first] = PVN_CLT_NEG_EVIDENCE;
    }
    
    return sumProduct (obs_float);
}


map<int64_t, float>
PVNCLTree::sumProduct (map<int64_t, float> obs) {
    
    map<int64_t, float> P;
    
    // clear all messages in tree
    clear_msgs ();
    
    // copy observations into tree
    map<int64_t, float>::iterator itObs;
    for (itObs=obs.begin(); itObs!=obs.end(); itObs++) {
        
        if (tree.count(itObs->first) == 0) {
            ERROR ("Observation of element not in tree");
            continue;
        }
        
        tree[itObs->first].observation = itObs->second;
    }
    
    // start at each leaf node and pass message to root
    recPassMsgToRoot (leaf_ids);

    // start at the root and pass message to all leaves
    recPassMsgToLeaves (root_id);
    
    //calculate marginals for every node in the tree
    map<int64_t, clNode>::iterator it;
    for (it=tree.begin(); it!=tree.end(); it++) {

        if (it->second.observation >= 0) {
            P[it->first] = it->second.observation; 
        } else {
           
            vector <float> P_x (2,0);
            //message from parent
            P_x[0] = it->second.u_xp_self[0];
            P_x[1] = it->second.u_xp_self[1];
    
            if (!it->second.is_leaf()) {
                // messages from each child
                for (size_t i=0; i<it->second.u_xc_self.size() ; i++) {
                    P_x[0] *= it->second.u_xc_self[i][0];
                    P_x[1] *= it->second.u_xc_self[i][1];
                    P_x = safe_normalize (P_x);
                }
            }
            // normalize and put into output
            P[it->first] = P_x[1]/(P_x[0] + P_x[1]);
            
        }
    }
    
    return P;
    
}


void
PVNCLTree::find_max_mi (set<int64_t> ids) {

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
PVNCLTree::make(const vector <perllcm::pvn_eview_map_match_data_t> &match_data,
                double info_threshold, double prior_alpha, double prior_beta,
                set < pair<int64_t, int64_t> > mask) {
        
    //make the training data
    PVNCLTrainData train_data;
    if(train_data.makeTrainingData(match_data, prior_alpha, prior_beta))
        return -1;

    //calculate the parent nodes based on maximising mutual information
    list<mutual_information> edges;
    createBaseEdges(edges, train_data, info_threshold, mask);
    edges_save = edges;
    if(reduceEdgesToMinSpan(edges, train_data)) return -1;
    
    //recursively build the tree into correct data structure
    tree.clear();
    recAddToTree (edges.front().id1, edges.front().id2, 0,
                  edges.front().mi, edges.front().rho,
                  train_data, edges);
    
    return 0;
}

void
PVNCLTree::print_tree (void) {
    
    cout << "[cl tree]\tFinal Tree:" << endl;
    map<int64_t, clNode>::iterator it;
    for (it=tree.begin(); it!=tree.end(); it++) {
        cout << "[cl tree]\t\tP(X" << it->second.id << ") = " << it->second.Pq << endl;
        cout << "[cl tree]\t\tP(X" << it->second.id << "=1|X" << it->second.pid << "=1) = " << it->second.Pq_p << endl;
        cout << "[cl tree]\t\tP(X" << it->second.id << "=1|X" << it->second.pid << "=0) = " << it->second.Pq_np << endl;
    }
    cout << "[cl tree]\t\tRoot = " << root_id << endl;
    cout << "[cl tree]\t\tLeaves = ";
    for (size_t i=0; i<leaf_ids.size(); i++) cout << leaf_ids[i] << " ";
    cout << endl;
}

void
PVNCLTree::print_tree_to_file (char *file) {
    
    ofstream out;
    out.open(file);
    
    if (!out.is_open()) {
        ERROR ("Failed to open %s", file);
        return;
    }
    
    map<int64_t, clNode>::iterator it;
    for (it=tree.begin(); it!=tree.end(); it++) {
        out << it->second.id << ", " << it->second.pid << ", " << it->second.p_mi << ", " << it->second.p_rho << endl;
    }
    
    out.close();
}



void
PVNCLTree::recAddToTree(int64_t id, int64_t pid, int depth, double mi, double rho,
                        PVNCLTrainData &train_data, list<mutual_information> &edges) {
        
    clNode new_node;
    
    // arbitrarily choose second id as parrent
    if (depth == 0) { // add root node
        new_node.id = id;
        new_node.pid = -1;
        new_node.Pq = (float)train_data.P(id, true);
        new_node.Pq_p = (float)train_data.P(id, true);
        new_node.Pq_np = (float)train_data.P(id, true);
        new_node.depth = depth;
        new_node.p_mi = 0.0;
        new_node.p_rho = 0.0;
        root_id = new_node.id;
    } else {
        new_node.id = id;
        new_node.pid = pid;
        new_node.Pq = (float)train_data.P(id, true);
        new_node.Pq_p = (float)train_data.CP(id, true, pid, true);
        new_node.Pq_np = (float)train_data.CP(id, true, pid, false);
        new_node.depth = depth;
        new_node.p_mi = mi;
        new_node.p_rho = rho;
    }
    
    //find all children and do the same
    vector<int64_t> cids;
    vector<double> mis;
    vector<double> rhos;
    list<mutual_information>::iterator edge = edges.begin();
    while(edge != edges.end()) {
        if(edge->id1 == new_node.id) {
            cids.push_back(edge->id2);
            mis.push_back(edge->mi);
            rhos.push_back(edge->rho);
            edge = edges.erase(edge);
            continue;
        }
        if(edge->id2 == new_node.id) {
            cids.push_back(edge->id1);
            mis.push_back(edge->mi);
            rhos.push_back(edge->rho);
            edge = edges.erase(edge);
            continue;
        }
        edge++;
    }
    for(size_t i=0; i < cids.size(); i++) {
        new_node.cids.push_back(cids[i]);
        recAddToTree(cids[i], new_node.id, depth+1, mis[i], rhos[i], train_data, edges);
    }
    if (cids.size() == 0)
        leaf_ids.push_back(new_node.id);
    
    tree[new_node.id] = new_node;
}

bool
PVNCLTree::sortInfoScores(mutual_information &first, mutual_information &second)  {
        return first.mi > second.mi;
}

double
PVNCLTree::calcMutInfo(PVNCLTrainData &train_data, int64_t id1, int64_t id2) {
        
    double accumulation = 0;
    double P00 = train_data.JP(id1, false, id2, false);
    if(P00) accumulation += P00 * log(P00 / 
            (train_data.P(id1, false)*train_data.P(id2, false)));

    double P01 = train_data.JP(id1, false, id2, true);
    if(P01) accumulation += P01 * log(P01 / 
            (train_data.P(id1, false)*train_data.P(id2, true)));

    double P10 = train_data.JP(id1, true, id2, false);
    if(P10) accumulation += P10 * log(P10 / 
            (train_data.P(id1, true)*train_data.P(id2, false)));

    double P11 = train_data.JP(id1, true, id2, true);
    if(P11) accumulation += P11 * log(P11 / 
            (train_data.P(id1, true)*train_data.P(id2, true)));

    return accumulation;
}

double
PVNCLTree::calcRho(PVNCLTrainData &train_data, int64_t id1, int64_t id2) {

    double mu1 = train_data.P(id1, true);
    double mu2 = train_data.P(id2, true);

    double std1 = sqrt(mu1 * (1.0 - mu1));
    double std2 = sqrt(mu2 * (1.0 - mu2));
    
    double P00 = train_data.JP(id1, false, id2, false);
    double P01 = train_data.JP(id1, false, id2, true);
    double P10 = train_data.JP(id1, true, id2, false);
    double P11 = train_data.JP(id1, true, id2, true);    
    
    double rho = (0.0 - mu1) * (0.0 - mu2) * P00 +
                 (0.0 - mu1) * (1.0 - mu2) * P01 +
                 (1.0 - mu1) * (0.0 - mu2) * P10 +
                 (1.0 - mu1) * (1.0 - mu2) * P11;
    return rho / (std1*std2);
}



void
PVNCLTree::createBaseEdges(list<mutual_information> &edges, PVNCLTrainData &train_data,
                           double info_threshold, set < pair<int64_t, int64_t> > mask)  {
        
    set <int64_t> var_ids = train_data.get_var_ids();
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
            
            // don't add the potential edge if it is in the mask
            pair <int64_t, int64_t> edge_pair_f (mi_tmp.id1, mi_tmp.id2);
            pair <int64_t, int64_t> edge_pair_b (mi_tmp.id2, mi_tmp.id1);
            if (mask.count (edge_pair_f) != 0 || mask.count (edge_pair_b) != 0)
                continue;
            
            mi_tmp.mi = (float)calcMutInfo(train_data, mi_tmp.id1, mi_tmp.id2);
            mi_tmp.rho = (float)calcRho(train_data, mi_tmp.id1, mi_tmp.id2);
            
            if(mi_tmp.mi >= info_threshold) {
                edges.push_back(mi_tmp);
                average += mi_tmp.mi;
            }
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
        cout << it->id1 << " <--> " <<  it->id2 << ", MI =  " << it->mi << endl;
    }



}

int
PVNCLTree::reduceEdgesToMinSpan(list<mutual_information> &edges, PVNCLTrainData &train_data) {

    //initialise groups
    set <int64_t> var_ids = train_data.get_var_ids();
    
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
PVNCLTrainData::PVNCLTrainData() {
        num_samples = 0;
};

PVNCLTrainData::~PVNCLTrainData() {

};

int
PVNCLTrainData::makeTrainingData(const vector <perllcm::pvn_eview_map_match_data_t> &match_data, double _prior_alpha, double _prior_beta) {

    if (_prior_alpha > 1) {
        use_prior = true;
    } else {
        use_prior = false;
    }
    prior_alpha = _prior_alpha;
    prior_beta = _prior_beta;

    marginals.clear();
    var_ids.clear();
    num_samples = match_data.size();
    data.resize(num_samples);
    
    for (int i=0; i<num_samples; i++) {

        perllcm::pvn_eview_map_match_data_t rd = match_data[i];
        
        map<int64_t, bool> m = pvn_eview_run_data_2_obs (&rd, &var_ids);
        
        data[i] = m;      
    }
    
    make_marginals();
    
    return 0;
}

set <int64_t>
PVNCLTrainData::get_var_ids () {
    return var_ids;
}

int
PVNCLTrainData::get_num_ids () {
    return var_ids.size();
}


double
map_estimate (int cnt, int total, bool use_prior, double prior_alpha, double prior_beta) {
    if (use_prior)
        //return ((double)cnt + (prior_alpha - 1))/((double)total + 2.0*(prior_alpha - 1));
        return ((double)cnt + (prior_alpha - 1))/((double)total + (prior_alpha + prior_beta - 2));
    else {
        if (total > 0) {
            return 0.01 + (0.98 * (double)cnt/(double)total);
        } else {
            return 0.5; // no knowladge prior    
        }
    }
}



void
PVNCLTrainData::make_marginals() {
    
    //the probability of the var =  #of samples with var = 1 / total samples where var was measured

    // itterate over all varialbes
    set<int64_t>::iterator iter;
    for (iter = var_ids.begin(); iter != var_ids.end(); ++iter) {
        int64_t id = (*iter);

        int true_cnt = 0, total_cnt = 0;   
        for (int i=0; i<num_samples; i++)  {
            
            // check if this key exists in this sample
            if ( data[i].find(id) != data[i].end() ) {
                
                //cout << data[i][id] << endl;
                
                if (data[i][id] == true) {
                    true_cnt++;
                }
                total_cnt++;
            }
            
        }
        //cout << true_cnt << "/" << total_cnt << endl;
        marginals[id] = map_estimate(true_cnt, total_cnt, use_prior, prior_alpha, prior_beta);

        //cout << id << ": " << marginals[id] << " = " << true_cnt << "/" << total_cnt << endl;
    }
    
}

double
PVNCLTrainData::P(int64_t id, bool val) {
    
    if (val)
        return marginals[id];
    else
        return 1.0 - marginals[id];
    
}

double
PVNCLTrainData::JP(int64_t ida, bool aval, int64_t idb, bool bval) {
    
    int cnt = 0, total_cnt = 0;   
    for (int i=0; i<num_samples; i++)  {
        
        // check if both ids exists in this sample
        if (data[i].find(ida) != data[i].end()  &&
            data[i].find(idb) != data[i].end()) {
            
            if (data[i][ida] == aval && data[i][idb] == bval) {
                cnt++;
            }
            total_cnt++;
        }
        
    }
    double jp;
    if (total_cnt > 0) {
        double a, b;
        pvn_eview_beta_prior_coefs (P(ida, aval)*P(idb, bval), PVN_CLT_CP_JP_PRIOR_STRENGHT, &a, &b);
        jp = map_estimate(cnt, total_cnt, use_prior, a, b);
    } else  { // if not co-observed assume independent
        jp = P(ida, aval) * P(idb, bval);
    }
    //cout << jp << " = " << cnt << "/" << total_cnt << endl;
    
    return jp;
}

double
PVNCLTrainData::CP(int64_t ida, bool aval, int64_t idb, bool bval) {

    int cnt = 0, total_cnt = 0;   
    for (int i=0; i<num_samples; i++)  {
        
        // check if both ids exists in this sample
        if (data[i].find(ida) != data[i].end()  &&
            data[i].find(idb) != data[i].end()) {
            
            if (data[i][idb] == bval) {
                if (data[i][ida] == aval) {
                    cnt++;
                }
                total_cnt++;
            }
        }
        
    }
    double cp = 0;
    if (total_cnt > 0) {
        double a, b;
        pvn_eview_beta_prior_coefs (P(ida, aval), PVN_CLT_CP_JP_PRIOR_STRENGHT, &a, &b);
        cp = map_estimate(cnt, total_cnt, use_prior, a, b);
    }
    else
        cp = P(ida, aval); // with no evidence assume independence
    
    return cp;

}
