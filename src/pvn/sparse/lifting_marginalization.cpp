#include "sparse.h"

using namespace std;
using namespace Eigen;
using namespace isam;

vector< isam::Node* >
get_marginalization_clique_nodes (Slam *slam, Pose3d_Node *marg_node) {
    
    vector< isam::Node* > node_vector;
    
    int id_m = marg_node->unique_id();
    
    std::list< isam::Node* > node_list;
    const list<Factor*>& factors = marg_node->factors();
    for (list<Factor*>::const_iterator it = factors.begin(); it!=factors.end(); it++) {
        
        std::vector<Node*>& f_nodes = (*it)->nodes();
        
        for (size_t i=0; i<f_nodes.size(); i++) {
        
            int id_i = f_nodes[i]->unique_id();
            
            if (id_m != id_i) {
                
                // try to add second node in factor
                if (node_list.end() == find (node_list.begin(), node_list.end(), f_nodes[i])) { // not already added
                    node_list.push_back(f_nodes[i]);
                    node_vector.push_back (f_nodes[i]);
                }       
            }   
        }        
    }
    
    return node_vector;
}


vector<Factor*>
get_intra_clique_factors (vector<Node*> clique_nodes, Node *m_node) { 
    
    vector<Factor*> ic_factors;
    
    // loop over each node
    for (size_t i=0; i<clique_nodes.size(); i++) {
        
        // get this nodes factors
        const std::list<Factor*> factors = clique_nodes[i]->factors();
        
        for (list<Factor*>::const_iterator it = factors.begin(); it!=factors.end(); it++) {
        
            // make sure factor hasnt already been added to the list
            if (ic_factors.end() != find (ic_factors.begin(), ic_factors.end(), (*it)))
                continue;
            
            std::vector<Node*>& f_nodes = (*it)->nodes();
            
            // don't include prior factors, they don't effect sparsity and pure root shift can't represent
            // world frame pose priors.
            // another option would be to include the inverse of the root noode in the root shift
            if (0 == strcmp((*it)->name(), "Pose3d_Factor") ||
                0 == strcmp((*it)->name(), "Pose3d_xyz_Factor") ||
                0 == strcmp((*it)->name(), "Pose3d_h_Factor") ||
                0 == strcmp((*it)->name(), "Pose3d_z_Factor") ||
                0 == strcmp((*it)->name(), "Pose3d_rp_Factor") ||
                0 == strcmp((*it)->name(), "Pose3d_xy_Factor"))
                continue;
            
            // nodes in these factors can be: the marg node, nodes in the clique, or nodes ouside the clique
            // we wish to return factors strictly between nodes in the clique, not outside nor margnode
            
            // is the marg node in this factor
            if (f_nodes.end() != find (f_nodes.begin(), f_nodes.end(), m_node))
                continue; // m_node found
            
            // strctly include in clique
            bool ic = true;
            for (size_t j=0; j<f_nodes.size() && ic; j++) {
            
                // if we find a factor node that is not in the clique
                if (clique_nodes.end() == find (clique_nodes.begin(), clique_nodes.end(), f_nodes[j])) 
                    ic = false;
            }
            
            if (ic)
                ic_factors.push_back(*it);
            
        }
    }
    
    return ic_factors;

}

MatrixXd
get_marginalization_clique_information (Slam *slam, Pose3d_Node *marg_node, vector<Node*>& clique_nodes, vector<Factor*> *ic_factors){
    
    vector<Node*> all_nodes = clique_nodes;
    all_nodes.push_back(marg_node);
    
    MatrixXd L (all_nodes.size()*6,all_nodes.size()*6); // clique nodes first then marg node at end
    L.setZero();
    
    const list<Factor*>& marg_node_factors = marg_node->factors();
    list<Factor*> factors (marg_node_factors);
    // if supplied ic factors add them to list
    if (ic_factors != NULL) {
        for (size_t i=0; i<ic_factors->size(); i++)
            factors.push_back((*ic_factors)[i]); 
    }
    
    for (list<Factor*>::iterator it = factors.begin(); it!=factors.end(); it++) {
        Factor *f = *it;
//cout << "FACTOR NAME " << f->name() << endl;
        // information added by this factor
        MatrixXd H = isamu_get_weighted_jacobian (f);
        MatrixXd dL = H.transpose()*H;


        // find overlap with L and add it
        vector<Node*>& f_nodes = f->nodes();
        vector<int> L_inds;
        vector<int> dL_inds;
        for (size_t j=0; j<f_nodes.size(); j++) {

            // loop over b_nodes to see which inds this node overlaps with
            for (size_t k=0; k<all_nodes.size(); k++) {
                if (f_nodes[j] == all_nodes[k]) {
                    for (int h=0; h<6; h++)
                        L_inds.push_back(6*k + h);
                    for (int h=0; h<6; h++)
                        dL_inds.push_back(6*j + h);    
                }
            }
        }

        if (L_inds.size() > 0 && dL_inds.size() > 0) {
            
            L(L_inds, L_inds) +=  dL(dL_inds, dL_inds);
//cout << "dL" << endl << dL << endl;
//cout << "L" << endl << L << endl;

            
            if (DEBUG) {        
                cout << "[lifting]\tAdding info from " << f->name() << " factor between nodes : ";
                for (size_t n=0; n<f_nodes.size(); n++)
                    cout << f_nodes[n]->unique_id() << " ";
                cout << endl;
            }
            
        } else {
            ERROR ("No overlap between factor and clique!");
        }
        
    }
    
    // perform marginalization
    int n = L.rows()-6;
    MatrixXd Lbb = L.bottomRightCorner(6,6);
    MatrixXd L_marg = L.topLeftCorner(n,n) - (L.topRightCorner(n,6) * posdef_pinv(Lbb, EPS) * L.bottomLeftCorner(6,n));
    //MatrixXd L_marg = L.topLeftCorner(n,n) - (L.topRightCorner(n,6) * L.bottomRightCorner(6,6).inverse() * L.bottomLeftCorner(6,n));
    return L_marg;
}


Factor*
lift_rs_factor (MatrixXd& L, vector<Node*> nodes, bool rs_inc_x_o_w) {
    
    int np = nodes.size();
    
    MatrixXd F;
    VectorXd z;    
    isamu_root_shift (z, &F, nodes, LINPOINT, rs_inc_x_o_w);
    
    MatrixXd Fpinv = pinv(F, EPS);
    MatrixXd L_noise = Fpinv.transpose() * L * Fpinv;
    
    if (DEBUG) {
        FullPivLU<MatrixXd> lu(F);    
        cout << "[lifting]\tTarget Jacobian: (" << F.rows() << " x " << F.cols() << ") rank = " <<  lu.rank() <<  endl;
        FullPivLU<MatrixXd> lun(L_noise);
        cout << "[lifting]\tLifted Meas. Noise: (" << L_noise.rows() << " x " << L_noise.cols() << ") rank = " <<  lun.rank() <<  endl;
        double recreate_test = (F.transpose()*L_noise*F - L).array().abs().matrix().lpNorm<Infinity>();
        cout << "[lifting]\tTarget Recreate Test: " << recreate_test << endl;
        if (recreate_test > 10) {
            cout << "[ERROR]\tRecreate Test Failed!" << endl;
            exit(0);
        }
    }

    vector<Pose3d_Node*> p3d_nodes;
    for (int i=0; i<np; i++) 
        p3d_nodes.push_back (dynamic_cast<Pose3d_Node*>(nodes[i]));
        
    if (DEBUG) {
        check_valid_noise (L_noise);
    }    
    
    Factor *f_add = new RS_Factor(p3d_nodes, z, Information(L_noise), rs_inc_x_o_w); 
    
    if (DEBUG && f_add != NULL) {
        cout << "[lifting]\t\tComputed RS_Factor " << f_add->unique_id() << " between nodes ";
        for (int i=0; i<np; i++)
            cout << nodes[i]->unique_id() << " ";
        cout << endl;
    }
    
    return f_add;
    
}

vector<Factor*>
lift_rs_factors (MatrixXd& L, vector<Node*> clique_nodes, bool clt_sparse, bool rs_inc_x_o_w) {
    
    vector<Factor*> new_rs_factors;
    
    if (clt_sparse) {
        
        CLTree clt;
        clt.make(L, 6);
        clt.print_tree();
        
        map<int64_t, CLTNode>::iterator it;
        for (it=clt.tree.begin(); it!=clt.tree.end(); it++) {
            
            if (it->second.is_root()) {
                
                MatrixXd La = it->second.marginal;
                double mag_test = La.array().abs().matrix().lpNorm<Infinity>();
                MatrixXd U = La.llt().matrixU();
                double chol_test = (La -  U.transpose()*U).array().abs().matrix().lpNorm<Infinity>();
                // have to ignore factors we cant represent
                if (mag_test > 1e-6 && chol_test < 1e-4) {
                    Pose3d_Node *node = dynamic_cast<Pose3d_Node*>(clique_nodes[it->second.id]);
                    
                    if (DEBUG) {
                        check_valid_noise (La);
                    }
                    
                    Factor *f_add = new Pose3d_Factor(node, node->value0(),Information(La));
                    new_rs_factors.push_back (f_add);
                }
                
            } else {
                vector<Node*> f_nodes;
                f_nodes.push_back(clique_nodes[it->second.id]);
                f_nodes.push_back(clique_nodes[it->second.pid]);
                MatrixXd Lagb = it->second.conditional;
                MatrixXd Lab = it->second.joint;
                MatrixXd Hagb(6,12);
                Hagb.block<6,6>(0,0) = MatrixXd::Identity (6,6);
                //Hagb.block<6,6>(0,6) = Lab.block<6,6>(0,0).inverse() * Lab.block<6,6>(0,6);
                MatrixXd Laa = Lab.block<6,6>(0,0);
                Hagb.block<6,6>(0,6) = posdef_pinv(Laa, EPS) * Lab.block<6,6>(0,6);
                MatrixXd Lt = Hagb.transpose() * Lagb * Hagb;
                new_rs_factors.push_back (lift_rs_factor (Lt, f_nodes, rs_inc_x_o_w));
            }
        }
        
    } else {
        new_rs_factors.push_back (lift_rs_factor (L, clique_nodes, rs_inc_x_o_w));    
    }
        
    return new_rs_factors;
}

void
lifting_marg_node (Slam *slam, Pose3d_Node *marg_node, bool clt_sparse, bool rs_inc_x_o_w) {

    vector< isam::Node* > clique_nodes = get_marginalization_clique_nodes (slam, marg_node);
    
    if (DEBUG) {        
        cout << "[lifting]\tMarginalization Clique Nodes: ";
        for (size_t i=0; i<clique_nodes.size(); i++)
            cout << clique_nodes[i]->unique_id() << " ";
        cout << endl;
    }
    
    // get intra-clique factors, those in the clique that are not directly connected to the marg node
    vector<Factor*> ic_factors = get_intra_clique_factors (clique_nodes, marg_node);
    
    MatrixXd L = get_marginalization_clique_information (slam, marg_node, clique_nodes, &ic_factors);
    
    // lift factors
    vector<Factor*> new_rs_factors;
    new_rs_factors = lift_rs_factors (L, clique_nodes, clt_sparse, rs_inc_x_o_w);
    
    // remove node and delete all adjacent factors
    slam->remove_node(marg_node);  // Note that the node itself is not deallocated.
    
    // remove all ic factors
    for(size_t i=0; i<ic_factors.size(); i++) {
        slam->remove_factor(ic_factors[i]);
        cout << "[lifting]\t\tRemoving intra-clique factor " << ic_factors[i]->unique_id() << " between nodes ";
        vector<Node*> tmp = ic_factors[i]->nodes();
        for (size_t i=0; i<tmp.size(); i++) 
            cout << tmp[i]->unique_id() << " ";
        cout << endl;
    }
    
    // add rs factors
    for(size_t i=0; i<new_rs_factors.size(); i++) {
        slam->add_factor(new_rs_factors[i]);
        if (DEBUG)
            cout << "[lifting]\tAdded RS Factor: " << new_rs_factors[i]->unique_id() << endl;
    }
    
}

void
PVNSparse::run_lifting_marginalization (void) {
    
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
        lifting_marg_node (&slam, pose3d_nodes[ii_m], clt_sparse, rs_inc_x_o_w);
        cout << "[lifting]\tRemoved node " << pose3d_nodes[ii_m]->unique_id();
        cout << " by lifting. (" << i << "/" << inds_to_marg.size()-1 << ", ";
        cout << (timestamp_now() - tic)/1e6 << " secs)" << endl;
        
        // sometimes isam seems to get into a bad state where the optimization diverges
        // if you modify the graph too much with out optimizing occasionally in between
        //if (!(i%BATCH_EVERY))
        //    slam.batch_optimization();
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
    cout << "[lifting]\tRunning batch optimization." << endl;
    slam.batch_optimization();
    
    caculate_and_write_results (&slam, dir, (char *)"lift", name,
                                (total_toc - total_tic)/1e6);
}
