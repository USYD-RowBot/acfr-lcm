#include <gsl/gsl_sf_gamma.h>

#include "sparse.h"

using namespace std;
using namespace Eigen;
using namespace isam;

// see matlabs cholcov
MatrixXd
cholcov (MatrixXd A, double eps = numeric_limits<float>::epsilon()) {
    
    A = (A.transpose() + A) / 2.0;
    
    SelfAdjointEigenSolver<MatrixXd> eigensolver(A);
    if (eigensolver.info() != Success) {
        ERROR ("Failed to compute eigenvalue decomposition");
        cout << "A:" << endl << A << endl;
    }
         
    VectorXd D = eigensolver.eigenvalues();
    MatrixXd U = eigensolver.eigenvectors();
    
    double tol = eps * D.size();
    
    vector<int> inds_pos;
    vector<int> inds_all;
    for (int i=0; i<D.size(); i++) {
        inds_all.push_back(i);
        if (D(i) > tol)
            inds_pos.push_back(i);
    }
    
    if (inds_pos.size() == 0) {
        return MatrixXd();
    }
    
    MatrixXd B;
    if (inds_pos.size() == (unsigned int)D.size()) {
    
        // full rank take a cholesky decomp
        B = A.llt().matrixL().transpose();
        
    } else {
        
        // not full rank
        
        // what???    
        //    % Pick eigenvector direction so max abs coordinate is positive
        //    [ignore,maxind] = max(abs(U),[],1);
        //    negloc = (U(maxind + (0:n:(m-1)*n)) < 0);
        //    U(:,negloc) = -U(:,negloc);
     
        VectorXd Dsub_sqrt(inds_pos.size());
        for (int i=0; i<Dsub_sqrt.size(); i++)
            Dsub_sqrt(i) = sqrt(D(inds_pos[i]));
        
        B = Dsub_sqrt.asDiagonal() * U(inds_all,inds_pos).transpose();

    }

    if (DEBUG) {
        double recreate_test = (B.transpose()*B - A).array().abs().matrix().lpNorm<Infinity>();
        cout << "[lifting]\tCholcov Recreate Test: " << recreate_test << endl;
        if (recreate_test > 10) {
            cout << "[ERROR]\tCholcov Recreate Test Failed!" << endl;
            cout << "A" << endl << A << endl;
//exit(0);
        }
    }

    return B;   
}

void
glc_root_shift (MatrixXd& L, vector<Node*> clique_nodes, MatrixXd &U, VectorXd &x_rs, bool rs_inc_x_o_w) {
    
        
    MatrixXd F;
    isamu_root_shift (x_rs, &F, clique_nodes, LINPOINT, rs_inc_x_o_w);
    MatrixXd Fpinv = pinv(F, EPS);
    MatrixXd UTU = Fpinv.transpose() * L * Fpinv;

    U = cholcov(UTU, EPS);
        
    if (DEBUG) {
        FullPivLU<MatrixXd> lu(F);    
        cout << "[lifting]\tTarget Jacobian: (" << F.rows() << " x " << F.cols() << ") rank = " <<  lu.rank() <<  endl;
        MatrixXd H = U*F;
        FullPivLU<MatrixXd> luh(H);
        cout << "[lifting]\tLifted GLC+Jacobian: (" << H.rows() << " x " << H.cols() << ") rank = " <<  luh.rank() <<  endl;
        double recreate_test = (H.transpose()*H - L).array().abs().matrix().lpNorm<Infinity>();
        cout << "[lifting]\tTarget Recreate Test: " << recreate_test << endl;
        if (recreate_test > 10) {
            cout << "[ERROR]\tRecreate Test Failed!" << endl;
            cout << "L" << endl << L << endl;
            cout << "F" << endl << F << endl;
            cout << "U" << endl << U << endl;
            cout << "UTU" << endl << UTU << endl;
//exit(0);
        }
    }
    
}

Factor*
lift_glc_factor (MatrixXd& L, vector<Node*> clique_nodes, bool world_lin, bool rs_inc_x_o_w) {
    
    int np = clique_nodes.size();
    int n = np * 6;
        
    Factor *f_add = NULL;
    if (world_lin) { // H with linearization on world frame xfms
        
        MatrixXd U = cholcov(L, EPS);
        
        VectorXd x(n);
        // fill x with the poses
        for (int i=0; i<np; i++) {
            x.segment<6>(i*6) = clique_nodes[i]->vector(LINPOINT);
        }
        
        vector<Pose3d_Node*> p3d_nodes;
        for (int i=0; i<np; i++) 
            p3d_nodes.push_back (dynamic_cast<Pose3d_Node*>(clique_nodes[i]));
        
        f_add = new GLC_Factor(p3d_nodes, x, U);        
    
    } else { // H with linearization on root shifted xfms
        
        MatrixXd U;
        VectorXd x_rs;
        glc_root_shift (L, clique_nodes, U, x_rs, rs_inc_x_o_w);
        if (DEBUG && U.rows() == 0) {
            cout << "[ERROR]\t\tComputed No Rank Factor!" << endl;
            if (L.rows() <= 6) {
                return NULL;
            } else {
                cout << "L" << endl << L << endl;
                exit(0);
            }
        }
        
        vector<Pose3d_Node*> p3d_nodes;
        for (int i=0; i<np; i++) 
            p3d_nodes.push_back (dynamic_cast<Pose3d_Node*>(clique_nodes[i]));

        f_add = new GLC_RS_Factor(p3d_nodes, x_rs, U, rs_inc_x_o_w); 

    }

    if (DEBUG && f_add != NULL) {
        cout << "[lifting]\t\tComputed GLC_Factor " << f_add->unique_id() << " between nodes ";
        for (int i=0; i<np; i++)
            cout << clique_nodes[i]->unique_id() << " ";
        cout << endl;
    }
    
    return f_add;
    
}


vector<Factor*>
lift_glc_factors (MatrixXd& L, vector<Node*> clique_nodes, bool clt_sparse, bool world_lin, bool rs_inc_x_o_w) {
    
    vector<Factor*> new_glc_factors;    
    
    if (clt_sparse) {

        CLTree clt;
        clt.make(L, 6);
        clt.print_tree();
        
        map<int64_t, CLTNode>::iterator it;
        for (it=clt.tree.begin(); it!=clt.tree.end(); it++) {
            
            if (it->second.is_root()) {
                MatrixXd La = it->second.marginal;
                double mag_test = La.array().abs().matrix().lpNorm<Infinity>();
                if (mag_test > 1e-6) {
                    vector<Node*> f_nodes;
                    f_nodes.push_back(clique_nodes[it->second.id]);
                    Factor * f_add = lift_glc_factor (La, f_nodes, world_lin, rs_inc_x_o_w);
                    if (f_add != NULL)
                        new_glc_factors.push_back (f_add);
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
                Hagb.block<6,6>(0,6) = posdef_pinv(Laa) * Lab.block<6,6>(0,6);
                MatrixXd Lt = Hagb.transpose() * Lagb * Hagb;
                Factor * f_add = lift_glc_factor (Lt, f_nodes, world_lin, rs_inc_x_o_w);
                if (f_add != NULL)
                    new_glc_factors.push_back (f_add);
            }
        }
        
    } else {
//cout << "LIFTING GLC FACTOR " << endl;
        Factor * f_add = lift_glc_factor (L, clique_nodes, world_lin, rs_inc_x_o_w);
        if (f_add != NULL)
            new_glc_factors.push_back (f_add);
    }
        
    return new_glc_factors;
}

void
lifting_marg_node_glc (Slam *slam, Pose3d_Node *marg_node, bool clt_sparse, bool world_lin, bool rs_inc_x_o_w) {

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
    
    // lift glc factors
    vector<Factor*> new_glc_factors;
    new_glc_factors = lift_glc_factors (L, clique_nodes, clt_sparse, world_lin, rs_inc_x_o_w);
    
    // remove node and delete all adjacent factors
    slam->remove_node(marg_node);  // Note that the node itself is not deallocated.
    
    // add glc factors
    for(size_t i=0; i<new_glc_factors.size(); i++) {
        slam->add_factor(new_glc_factors[i]);
        if (DEBUG)
            cout << "[lifting]\tAdded GLC Factor: " << new_glc_factors[i]->unique_id() << endl;
    }
    
    // remove all ic factors
    for(size_t i=0; i<ic_factors.size(); i++) {
        slam->remove_factor(ic_factors[i]);
        cout << "[lifting]\t\tRemoving intra-clique factor " << ic_factors[i]->unique_id() << " between nodes ";
        vector<Node*> tmp = ic_factors[i]->nodes();
        for (size_t i=0; i<tmp.size(); i++) 
            cout << tmp[i]->unique_id() << " ";
        cout << endl;
    }
    
    
}

void
PVNSparse::run_lifting_marginalization_glc (void) {
    
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
        lifting_marg_node_glc (&slam, pose3d_nodes[ii_m], clt_sparse, world_lin, rs_inc_x_o_w);
        cout << "[lifting]\tRemoved node " << pose3d_nodes[ii_m]->unique_id();
        cout << " by lifting. (" << i << "/" << inds_to_marg.size()-1 << ", ";
        cout << (timestamp_now() - tic)/1e6 << " secs)" << endl;
        
        // sometimes isam seems to get into a bad state where the optimization diverges
        // if you modify the graph too much with out optimizing
        //if (!(i%BATCH_EVERY))
        //    slam.batch_optimization();
        //slam.update();
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
    
    caculate_and_write_results (&slam, dir, (char *)"lift_glc", name,
                                (total_toc - total_tic)/1e6);
       
}


//MatrixXd
//get_markov_blanket_marginal_information_glc (Slam *slam, Pose3d_Node *marg_node, vector<Node*> *nodes) {
//    
//    const Covariances& covariances = slam->covariances();
//    list< isam::Node* > node_list = get_markov_blanket_nodes_glc (slam, marg_node, nodes);
//    MatrixXd P = covariances.marginal(node_list);
//    MatrixXd L = P.inverse();
//    
//    return L;
//}





//void 
//get_blanket_factors (vector<Node*> b_nodes,                  // nodes in blanket (does not include node to be marganalized)
//                     Node * m_node,                          // node to be marganalized
//                     vector<Factor*>& in_blanket_factors,    // factors totaly within the blanket
//                     vector<Factor*>& out_blanket_factors) { // factors that connect to the blanket
//    
//    // loop over each node
//    for (size_t i=0; i<b_nodes.size(); i++) {
//        
//        // get this nodes factors
//        const std::list<Factor*> factors = b_nodes[i]->factors();
//        
//        for (list<Factor*>::const_iterator it = factors.begin(); it!=factors.end(); it++) {
//        
//            // make sure factor hasnt already been added to one of the lists
//            if (in_blanket_factors.end() != find (in_blanket_factors.begin(), in_blanket_factors.end(), (*it)) ||
//                out_blanket_factors.end() != find (out_blanket_factors.begin(), out_blanket_factors.end(), (*it)))
//                continue;
//            
//        
//            if(0 == strcmp ((*it)->name(), "Pose3d_Factor") ||
//               0 == strcmp ((*it)->name(), "Pose3d_Pose3d_Factor") ||
//               0 == strcmp ((*it)->name(), "GLC_Factor") ) {
//                
//                // if a glc factor is contained entirely in the blanket remove it
//                std::vector<Node*>& f_nodes = (*it)->nodes();
//                
//                // nodes in these factors can be: the marg node, nodes in the blanket, or nodes ouside the blanket
//                // in_blanket: factors strictly between nodes in blanket, not outside or margnode
//                // out_blanket: factors that include both nodes in blanket and nodes outside, but not the marg node
//                bool in_blanket = true;
//                bool out_blanket = true;
//                // is the marg node in this factor
//                if (f_nodes.end() != find (f_nodes.begin(), f_nodes.end(), m_node)) {
//                    // m_node found, cant be outside, otherwise entire factor would be in blanket
//                    out_blanket = false;
//                    in_blanket = false;
//                }
//                
//                for (size_t j=0; j<f_nodes.size() && in_blanket; j++) {
//                
//                    // if we find a factor node that is not in the blanket then the whole factor cannot be containted in blanket
//                    if (b_nodes.end() == find (b_nodes.begin(), b_nodes.end(), f_nodes[j]))
//                        in_blanket = false;
//                }
//                
//                if (in_blanket)
//                    in_blanket_factors.push_back(*it);
//                else if (out_blanket) // !strictly in blanket, and doesn't include m_node, must include outside
//                    out_blanket_factors.push_back(*it);
//                        
//            } else {
//                ERROR ("Unexpected factor type: %s", (*it)->name());
//            }
//            
//        }
//    }
//
//}
//
//
//MatrixXd
//remove_out_blanket_info (MatrixXd L, vector<Node*> b_nodes, vector<Factor*> ob_factors) {
//    
//    for (size_t i=0; i<ob_factors.size(); i++) {
//
//        // information added by this factor
//        MatrixXd Omega = ob_factors[i]->sqrtinf().transpose() * ob_factors[i]->sqrtinf();
//        MatrixXd H = isamu_get_jacobian (ob_factors[i]);
//        MatrixXd dL = H.transpose()*Omega*H;
//
//        // find overlap with L and subtract it
//        vector<Node*>& f_nodes = ob_factors[i]->nodes();
//        vector<int> L_inds;
//        vector<int> dL_inds;
//        for (size_t j=0; j<f_nodes.size(); j++) {
//
//            // loop over b_nodes to see which inds this node overlaps with
//            for (size_t k=0; k<b_nodes.size(); k++) {
//                if (f_nodes[j] == b_nodes[k]) {
//                    for (int h=0; h<6; h++)
//                        L_inds.push_back(6*k + h);
//                    for (int h=0; h<6; h++)
//                        dL_inds.push_back(6*j + h);    
//                }
//            }
//        }
//
//        //MatrixXd tmp = L(L_inds, L_inds);
//        if (L_inds.size() > 0 && dL_inds.size() > 0) {
//            
//            L(L_inds, L_inds) -=  dL(dL_inds, dL_inds);
//            
//            if (DEBUG) {        
//                cout << "[lifting]\tRemoving info from out-of-blanket factor between nodes : ";
//                for (size_t n=0; n<f_nodes.size(); n++)
//                    cout << f_nodes[n]->unique_id() << " ";
//                cout << endl;
//            }
//            
//        }
//    }    
//    
//    return L;
//    
//}