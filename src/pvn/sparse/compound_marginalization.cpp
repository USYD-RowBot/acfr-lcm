#include <boost/utility.hpp>

#include "sparse.h"

using namespace std;
using namespace Eigen;
using namespace isam;

Pose3d_Factor*
compound_p3d_p3dp3d (int id_m, Pose3d_Factor *f_a, Pose3d_Pose3d_Factor *f_b) {
    
    // get variables for f_a
    const Pose3d x_l_i  = f_a->measurement();
    std::vector<Node*>& f_a_nodes = f_a->nodes();
    const Eigen::MatrixXd sqinf_a  = f_a->sqrtinf(); 
    Pose3d_Node *node_a = dynamic_cast<Pose3d_Node*>(f_a_nodes[0]); 
    int id_a = node_a->unique_id(); 
    if (id_a != id_m) ERROR ("Wrong factor id %d != %d!", id_a, id_m);
    
    // get variables for f_b
    std::vector<Node*>& f_b_nodes = f_b->nodes();
    const Eigen::MatrixXd sqinf_b  = f_b->sqrtinf();
    Pose3d_Node *node_b_0 = dynamic_cast<Pose3d_Node*>(f_b_nodes[0]);
    Pose3d_Node *node_b_1 = dynamic_cast<Pose3d_Node*>(f_b_nodes[1]); 
    int id_b_0 = node_b_0->unique_id();
    int id_b_1 = node_b_1->unique_id();
    
    // compound
    double X_l_i[6]; 
    isamu_isam2van_pose3d (x_l_i, X_l_i);
    Matrix<double, 6, 6, RowMajor> R_l_i;
    isamu_isam2van_sqrtinf3d  (sqinf_a, R_l_i.data());
    
    double X_i_j[6];
    Matrix<double, 6, 6, RowMajor> R_i_j;
    Pose3d_Node *node_j;
    if (id_b_0 == id_m) { //direct compound
        
        const Pose3d x_i_j  = f_b->measurement();
        isamu_isam2van_pose3d (x_i_j, X_i_j);
        isamu_isam2van_sqrtinf3d  (sqinf_b, R_i_j.data());
        
        node_j = node_b_1;
        
    } else if (id_b_1 == id_m) { // compound with inverse
        
        const Pose3d x_j_i  = f_b->measurement();
        double X_j_i[6];
        isamu_isam2van_pose3d (x_j_i, X_j_i);
        Matrix<double, 6, 6, RowMajor> R_j_i;
        isamu_isam2van_sqrtinf3d  (sqinf_b, R_j_i.data());
        
        Matrix<double, 6, 6, RowMajor> J;
        ssc_inverse(X_i_j, J.data(), X_j_i);
        R_i_j = J * R_j_i * J.transpose();
        
        node_j = node_b_0;
        
    } else {
        ERROR ("ID mismatch in factors!");
        exit(EXIT_FAILURE);
    }
    
    double X_l_j[6];
    Matrix<double, 6, 12, RowMajor> J;
    ssc_head2tail(X_l_j, J.data(), X_l_i, X_i_j);
    Matrix<double, 6, 6, RowMajor> R_l_j;
    R_l_j = J.topLeftCorner(6,6) * R_l_i * J.topLeftCorner(6,6).transpose();
    R_l_j += J.bottomRightCorner(6,6) * R_i_j * J.bottomRightCorner(6,6).transpose();
    
    if (DEBUG) {
        check_valid_noise (R_l_j.inverse());
    }
    
    Pose3d_Factor *f_add = new Pose3d_Factor(node_j, isamu_van2isam_pose3d (X_l_j),
                                             SqrtInformation (isamu_van2isam_sqrtinf3d  (R_l_j.data())));
    
    if (DEBUG) {
        cout << "[compound]\t\tAdding Pose3d_Factor: local frame to " << node_j->unique_id() << endl;
    }
    
    return f_add;
}

vector<Pose3d_Factor*>
compound_pose3d_factors (Pose3d_Node *marg_node) {

    vector<Pose3d_Factor*> new_pose3d_factors;

    // get the list of factors from the node to be removed
    const list<Factor*>& factors = marg_node->factors();
    int id_m = marg_node->unique_id();
    
    if (DEBUG) {
        cout << "[compound]\tCompounding Pose3d factors: " << factors.size() << " factors in markov blanket." << endl;
        for (list<Factor*>::const_iterator it = factors.begin(); it!=factors.end(); it++){
            std::vector<Node*>& nodes = (*it)->nodes();
            if(0 == strcmp ((*it)->name(), "Pose3d_Pose3d_Factor") ) {            
                cout << "[compound]\t\t" << nodes[0]->unique_id() << " to " << nodes[1]->unique_id() << endl;
            } else {
                cout << "[compound]\t\tLocal to " << nodes[0]->unique_id() << endl;
            }
        }
    }

    for (list<Factor*>::const_iterator it_a = factors.begin(); it_a!=factors.end(); it_a++) {
          
        if(0 != strcmp ((*it_a)->name(), "Pose3d_Factor") )
            continue;
        
        Pose3d_Factor *f_a = dynamic_cast<Pose3d_Factor*>(*it_a);
        
        // loop over all Pose3d_Pose3d_Factors
        for (list<Factor*>::const_iterator it_b = factors.begin(); it_b!=factors.end(); it_b++) {
        
            if(0 != strcmp ((*it_b)->name(), "Pose3d_Pose3d_Factor"))
                continue;
            
            Pose3d_Pose3d_Factor *f_b = dynamic_cast<Pose3d_Pose3d_Factor*>(*it_b);
            
            Pose3d_Factor* f_add = compound_p3d_p3dp3d (id_m, f_a, f_b);
            
            // add the new factor
            new_pose3d_factors.push_back(f_add);
        }
    }
        
    return new_pose3d_factors;
}


Pose3d_Pose3d_Factor*
compound_p3dp3d_p3dp3d (int id_m, Pose3d_Pose3d_Factor *f_a, Pose3d_Pose3d_Factor *f_b) {
    
    // get variables for f_a
    std::vector<Node*>& f_a_nodes = f_a->nodes();
    const Eigen::MatrixXd sqinf_a  = f_a->sqrtinf(); 
    Pose3d_Node *node_a_0 = dynamic_cast<Pose3d_Node*>(f_a_nodes[0]);
    Pose3d_Node *node_a_1 = dynamic_cast<Pose3d_Node*>(f_a_nodes[1]); 
    int id_a_0 = node_a_0->unique_id();
    int id_a_1 = node_a_1->unique_id(); 
    
    double X_i_m[6]; // transfrom from non-marg node in factor a to marg node
    Matrix<double, 6, 6, RowMajor> R_i_m;
    Pose3d_Node *node_i;
    if (id_a_0 == id_m) { // compound with inverse
        
        const Pose3d x_m_i  = f_a->measurement();
        double X_m_i[6];
        isamu_isam2van_pose3d (x_m_i, X_m_i);
        Matrix<double, 6, 6, RowMajor> R_m_i;
        isamu_isam2van_sqrtinf3d  (sqinf_a, R_m_i.data());
        
        Matrix<double, 6, 6, RowMajor> J;
        ssc_inverse(X_i_m, J.data(), X_m_i);
        R_i_m = J * R_m_i * J.transpose();
        
        node_i = node_a_1;
        
    } else if (id_a_1 == id_m) { // direct compound
        
        const Pose3d x_i_m  = f_a->measurement();
        isamu_isam2van_pose3d (x_i_m, X_i_m);
        isamu_isam2van_sqrtinf3d  (sqinf_a, R_i_m.data());            
        
        node_i = node_a_0;
        
    } else {
        ERROR ("ID mismatch in factors!");
        exit(EXIT_FAILURE);
    }
    
    // get variables for f_b
    std::vector<Node*>& f_b_nodes = f_b->nodes();
    const Eigen::MatrixXd sqinf_b  = f_b->sqrtinf();
    Pose3d_Node *node_b_0 = dynamic_cast<Pose3d_Node*>(f_b_nodes[0]);
    Pose3d_Node *node_b_1 = dynamic_cast<Pose3d_Node*>(f_b_nodes[1]); 
    int id_b_0 = node_b_0->unique_id();
    int id_b_1 = node_b_1->unique_id();
        
    double X_m_j[6];
    Matrix<double, 6, 6, RowMajor> R_m_j;
    Pose3d_Node *node_j;
    if (id_b_0 == id_m) { //direct compound
        
        const Pose3d x_m_j  = f_b->measurement();
        isamu_isam2van_pose3d (x_m_j, X_m_j);
        isamu_isam2van_sqrtinf3d  (sqinf_b, R_m_j.data());
        
        node_j = node_b_1;
        
    } else if (id_b_1 == id_m) { // compound with inverse
        
        const Pose3d x_j_m  = f_b->measurement();
        double X_j_m[6];
        isamu_isam2van_pose3d (x_j_m, X_j_m);
        Matrix<double, 6, 6, RowMajor> R_j_m;
        isamu_isam2van_sqrtinf3d  (sqinf_b, R_j_m.data());
        
        Matrix<double, 6, 6, RowMajor> J;
        ssc_inverse(X_m_j, J.data(), X_j_m);
        R_m_j = J * R_j_m * J.transpose();
        
        node_j = node_b_0;
        
    } else {
        ERROR ("ID mismatch in factors!");
        exit(EXIT_FAILURE);
    }
    
    double X_i_j[6];
    Matrix<double, 6, 12, RowMajor> J;
    ssc_head2tail(X_i_j, J.data(), X_i_m, X_m_j);
    Matrix<double, 6, 6, RowMajor> R_i_j;
    R_i_j = J.topLeftCorner(6,6) * R_i_m * J.topLeftCorner(6,6).transpose();
    R_i_j += J.bottomRightCorner(6,6) * R_m_j * J.bottomRightCorner(6,6).transpose();
    
    Pose3d_Pose3d_Factor *f_add = NULL;
    if (node_i->unique_id() < node_j->unique_id()) {

        if (DEBUG) {
            check_valid_noise (R_i_j.inverse());
        }
    
        f_add = new Pose3d_Pose3d_Factor(node_i, node_j,
                                         isamu_van2isam_pose3d (X_i_j),
                                         SqrtInformation (isamu_van2isam_sqrtinf3d  (R_i_j.data())));
        
    } else if (node_i->unique_id() > node_j->unique_id()) {
        
        // always default to odometry-like constraints from older nodes to newer nodes
        // kind of a hack but allows it to work with our graph parsing code
        double X_j_i[6];
        Matrix<double, 6, 6, RowMajor> J;
        ssc_inverse(X_j_i, J.data(), X_i_j);
        Matrix<double, 6, 6, RowMajor> R_j_i;
        R_j_i = J * R_i_j * J.transpose();
        
        if (DEBUG) {
            check_valid_noise (R_j_i.inverse());
        }    
        
        f_add = new Pose3d_Pose3d_Factor(node_j, node_i,
                                         isamu_van2isam_pose3d (X_j_i),
                                         SqrtInformation (isamu_van2isam_sqrtinf3d  (R_j_i.data())));
        
    }
    
    
    if (DEBUG && f_add != NULL) {
        cout << "[compound]\t\tAdding Pose3d_Pose3d_Factor: " << node_i->unique_id() << " to " << node_j->unique_id() << endl;
    }
    
    return f_add;
}

vector<Pose3d_Pose3d_Factor*>
compound_pose3d_pose3d_factors (Pose3d_Node *marg_node) {

    vector<Pose3d_Pose3d_Factor*> new_pose3d_pose3d_factors;

    // get the list of factors from the node to be removed
    const list<Factor*>& factors = marg_node->factors();
    int id_m = marg_node->unique_id();
    
    if (DEBUG) {
        cout << "[compound]\tCompounding Pose3d Pose3d factors: " << factors.size() << " factors in markov blanket." << endl;
        for (list<Factor*>::const_iterator it = factors.begin(); it!=factors.end(); it++){
            std::vector<Node*>& nodes = (*it)->nodes();
            if(0 == strcmp ((*it)->name(), "Pose3d_Pose3d_Factor") ) {            
                cout << "[compound]\t\t" << nodes[0]->unique_id() << " to " << nodes[1]->unique_id() << endl;
            } else {
                cout << "[compound]\t\tLocal to " << nodes[0]->unique_id() << endl;
            }
        }
    }

    for (list<Factor*>::const_iterator it_a = factors.begin(); it_a!=factors.end(); it_a++) {
          
        if(0 != strcmp ((*it_a)->name(), "Pose3d_Pose3d_Factor") )
            continue;
        
        Pose3d_Pose3d_Factor *f_a = dynamic_cast<Pose3d_Pose3d_Factor*>(*it_a);
        
        // loop over all Pose3d_Pose3d_Factors
        for (list<Factor*>::const_iterator it_b = boost::next(it_a); it_b!=factors.end(); it_b++) {
        
            if(0 != strcmp ((*it_b)->name(), "Pose3d_Pose3d_Factor"))
                continue;
            
            Pose3d_Pose3d_Factor *f_b = dynamic_cast<Pose3d_Pose3d_Factor*>(*it_b);
            
            Pose3d_Pose3d_Factor* f_add = compound_p3dp3d_p3dp3d (id_m, f_a, f_b);
            
            // add the new factor
            if (NULL != f_add)
                new_pose3d_pose3d_factors.push_back(f_add);
        }
    }
        
    return new_pose3d_pose3d_factors;
}

vector<Pose3d_Pose3d_Factor*> 
merge_p3d_p3d_factors(vector<Pose3d_Pose3d_Factor*> factors) {
    
    vector<Pose3d_Pose3d_Factor*> merged_factors;
    
    while (factors.size()) {
        
        vector<Pose3d_Pose3d_Factor*> tbp_factors;  //to be processed factors
        vector<Node *> nodes0 = factors[0]->nodes();
        
        MatrixXd L_merg = factors[0]->sqrtinf().transpose() * factors[0]->sqrtinf();
        VectorXd mu_merg = factors[0]->measurement().vector();
        
        for (size_t i=1; i<factors.size(); i++) {
       
            vector<Node *> nodesi = factors[i]->nodes();    
        
            if (nodesi[0] == nodes0[0] && nodesi[1] == nodes0[1]) {
                
                MatrixXd L_i = factors[i]->sqrtinf().transpose() * factors[i]->sqrtinf();
                VectorXd mu_i = factors[i]->measurement().vector();
                VectorXd tmp = L_merg*mu_merg + L_i*mu_i;                             
                //mu_merg = (L_merg + L_i).ldlt().solve(tmp);
                mu_merg = (L_merg + L_i).inverse() * tmp;
                L_merg += L_i;
             
            }else {
                tbp_factors.push_back(factors[i]);
            }
        }
       
        Pose3d_Pose3d_Factor *f_merg;
        Pose3d_Node *node_i = dynamic_cast<Pose3d_Node *>(nodes0[0]);
        Pose3d_Node *node_j = dynamic_cast<Pose3d_Node *>(nodes0[1]);
        f_merg = new Pose3d_Pose3d_Factor(node_i, node_j,
                                          Pose3d (mu_merg(0), mu_merg(1), mu_merg(2),
                                                  mu_merg(3), mu_merg(4), mu_merg(5)),
                                          Information (L_merg));
        merged_factors.push_back (f_merg);
        
        factors = tbp_factors;

    }
    
    return merged_factors;
}


MatrixXd
get_markov_blanket_conditional_information (Slam *slam, Pose3d_Node *marg_node, vector<Node*> clique_nodes) {
    
    list<Node*> all_nodes = slam->get_nodes();
    int64_t tic;
    
    vector<int> inds;
    // loop over nodes in markov blanket to find their inds in full matrix
    for (size_t n=0; n<clique_nodes.size(); n++){
        int i = 0;
        for (list<Node*>::const_iterator it_b = all_nodes.begin(); it_b!=all_nodes.end(); it_b++){
            if (clique_nodes[n]->unique_id() == (*it_b)->unique_id()) {
                for (int j=0; j<6; j++) {
                    inds.push_back(6*i + j);
                }
            }
            i++;
        }
    }    
    
    int n = inds.size();
    
    // add in node to be marganalized
    int i = 0;
    for (list<Node*>::const_iterator it = all_nodes.begin(); it!=all_nodes.end(); it++){
        if (marg_node->unique_id() == (*it)->unique_id()) {
            for (int j=0; j<6; j++) {
                inds.push_back(6*i + j);
            }
        }
        i++;
    }
    
    // conditional Information Recovery
    tic = timestamp_now();
    MatrixXd L = isamu_conditional_information_recovery (slam, inds);
    std::cout << "[lifting]\tConditional Information Recovery: " << (timestamp_now() - tic)/1e6 << " secs." << std::endl;
        
    // marganalize out node from conditional
    MatrixXd L_marg = L.topLeftCorner(n,n) - (L.topRightCorner(n,6) * L.bottomRightCorner(6,6).inverse() * L.bottomLeftCorner(6,n));


    return L_marg;
}

vector<Pose3d_Pose3d_Factor*> 
sparsify_p3d_p3d_factors(Slam *slam, vector<Pose3d_Pose3d_Factor*> factors, Pose3d_Node *marg_node) {
    
    vector<Pose3d_Pose3d_Factor*> s_factors;
    
    vector<Node*> clique_nodes = get_marginalization_clique_nodes (slam, marg_node);
    MatrixXd L = get_markov_blanket_conditional_information (slam, marg_node, clique_nodes);
    
    CLTree clt;
    clt.make(L, 6);
    
    // loop over tree and include factors that meet are in tree
    map<int64_t, CLTNode>::iterator it;
    for (it=clt.tree.begin(); it!=clt.tree.end(); it++) {
        
        if (it->second.is_root())
            continue;
        
        Node* node1 = clique_nodes[it->second.id];
        Node* node2 = clique_nodes[it->second.pid];
        
        bool found = false;
        
        for (size_t i=0; i<factors.size(); i++) {

            vector<Node *> nodesi = factors[i]->nodes();    

            if ((nodesi[0] == node1 && nodesi[1] == node2) ||
                (nodesi[0] == node2 && nodesi[1] == node1)) {
                s_factors.push_back(factors[i]);
                found = true;
            }
            
        }
        
        if (!found) {
            cout << "FAILED TO FIND LINK IN TREE FOR " << node1->unique_id() << " to " << node2->unique_id() << endl;
            exit(0);
        }
    }
    
    return s_factors;
}

void
compound_marg_node (Slam *slam, Pose3d_Node *marg_node, bool clt_sparse) {

    // first take care of Pose3d_Factors
    vector<Pose3d_Factor*> new_pose3d_factors;
    new_pose3d_factors = compound_pose3d_factors (marg_node);   

    // then take care of Pose3d_Pose3d_Factors
    vector<Pose3d_Pose3d_Factor*> new_pose3d_pose3d_factors;
    new_pose3d_pose3d_factors = compound_pose3d_pose3d_factors (marg_node);
    
    if (clt_sparse){
        new_pose3d_pose3d_factors = sparsify_p3d_p3d_factors (slam, new_pose3d_pose3d_factors, marg_node);    
    }

    // remove node and delete all adjacent factors
    slam->remove_node(marg_node);  // Note that the node itself is not deallocated.
    
    // merge any factors between the same nodes
    // on large graphs this blows up for some reason, but otherwise even moderatly sized graphs are to large
    new_pose3d_pose3d_factors = merge_p3d_p3d_factors (new_pose3d_pose3d_factors);

    for(size_t i=0; i<new_pose3d_factors.size(); i++)
        slam->add_factor(new_pose3d_factors[i]);
    
    for(size_t i=0; i<new_pose3d_pose3d_factors.size(); i++)
        slam->add_factor(new_pose3d_pose3d_factors[i]);    

}


void
PVNSparse::run_compound_marginalization (void) {
    
    Slam slam;
    setup_isam (&slam);
    
    std::map<int64_t, Pose3d_Node*> pose3d_nodes;
    load_isam_graph (&slam, &pose3d_nodes);
    
    // load the nodes to marganalize list
    char inds_to_marg_file[PATH_MAX];
    sprintf (inds_to_marg_file, "%s/processed/inds_to_marg.txt", dir);
    vector<int> inds_to_marg = load_inds (inds_to_marg_file);
    
    
    int64_t total_tic = timestamp_now();

    for (int i=(int)inds_to_marg.size()-1; i>=0;  i--) {
        //int ii_m = pose3d_nodes[inds_to_marg[i]]->unique_id();
        int ii_m = inds_to_marg[i];
        int64_t tic = timestamp_now();
        compound_marg_node (&slam, pose3d_nodes[ii_m], clt_sparse);
        cout << "[compound]\tRemoved node " << pose3d_nodes[ii_m]->unique_id();
        cout << " by compounding. (" << i << "/" << inds_to_marg.size()-1 << ", ";
        cout << (timestamp_now() - tic)/1e6 << " secs)" << endl;
        
        // sometimes isam seems to get into a bad state where the optimization diverges
        // if you modify the graph too much with out optimizing
        //if (!(i%BATCH_EVERY)) {
        //    slam.batch_optimization();
        //    slam.print_stats();
        //}
    }
    
    int64_t total_toc = timestamp_now();
    
    if (add_meas_after) {
        cout << "[baseline]\tAdding additional measurements" << std::endl;
        char meas_to_add_file[PATH_MAX];
        sprintf (meas_to_add_file, "%s/meas_to_add.txt", dir);
        vector<Factor*> f2add = load_meas_to_add (meas_to_add_file, pose3d_nodes);
        for (size_t i=0; i<f2add.size(); i++) {
            slam.add_factor(f2add[i]);
        }
    }
    
    // run slam
    cout << "[compound]\tRunning batch optimization." << endl;
    slam.batch_optimization();
    
    caculate_and_write_results (&slam, dir, (char *)"comp", name,
                                (total_toc - total_tic)/1e6);
       
}       
