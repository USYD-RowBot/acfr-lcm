#include "map_builder.h"

#include <isam/isam.h>

#include "perls-math/dm.h"

using namespace std;
using namespace Eigen;
using namespace isam;


Matrix<double, 6, 6, RowMajor>
order_isam2van(Matrix<double, 6, 6, RowMajor> in) {
    
    Matrix<double, 6, 6, RowMajor> out;
    
    int row, col;
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            row = i, col = j;
            if (i == 3) row = 5;
            if (i == 5) row = 3;
            if (j == 3) col = 5;
            if (j == 5) col = 3;
            out (i,j) = in (row,col);
        }
    }
    
    return out;
    
}

void
add_noise(double *mu, MatrixXd Q, int size) {
    gsl_rng *rng = gslu_rand_rng_alloc ();
    
    double out[size];
    gsl_vector_view out_v  = gsl_vector_view_array (out, size);
    gsl_vector_view mu_v  = gsl_vector_view_array (mu, size);
    gsl_matrix_view Q_v = gsl_matrix_view_array (Q.data(), size, size); 
    gslu_rand_gaussian_vector (rng, &out_v.vector, &mu_v.vector, &Q_v.matrix, NULL);
    gsl_rng_free (rng);
    
    memcpy(mu, out, size*sizeof (double));
}

void
test_lift_constraints_loop () {
    
    // known poses
    double x1[6] = {0,  0,  0,  0,          M_PI/12.0, -M_PI/6.0};
    double x2[6] = {10, 10, 0,  0,          0,          M_PI/4.0};
    double x3[6] = {20, 20, 0, -M_PI/10.0,  M_PI/10.0,  M_PI/8.0};
    double x4[6] = {30, 30, 0,  M_PI/10.0, -M_PI/8.0,   M_PI/4.0};
        
    // build measurements
    double z1[6], z2[6], z3[6], z4[6], z5[6];
    ssc_tail2tail(z1, NULL, x1, x2);
    ssc_tail2tail(z2, NULL, x2, x3);
    ssc_tail2tail(z3, NULL, x1, x3);
    ssc_tail2tail(z4, NULL, x3, x4);
    ssc_tail2tail(z5, NULL, x1, x4);
    
    // add a bearing only measurement
    double z3i[6];
    ssc_inverse(z3i, NULL, z3);
    gsl_vector_view z3i_v = gsl_vector_view_array (z3i, 6);
    GSLU_MATRIX_VIEW (Sigma3, 6, 6, {0});
    double z3b[5];
    gsl_vector_view z3b_v = gsl_vector_view_array (z3b, 5);
    GSLU_MATRIX_VIEW (Sigma3b, 5, 5);
    dm_trans2dm_pose_cov (&z3i_v.vector, &Sigma3.matrix, &z3b_v.vector, &Sigma3b.matrix, NULL);
    
    // add noise to measurements
    Matrix<double, 6, 6, RowMajor> Q = 1e-2 * Matrix<double, 6, 6>::Identity();
    Noise Qsqinf = Information(Q.inverse());
    Matrix<double, 5, 5, RowMajor> Qb= 1e-2 * Matrix<double, 5, 5>::Identity();
    Noise Qbsqinf = Information(Qb.inverse());
    
    add_noise (z1, Q, 6);
    add_noise (z2, Q, 6);
    add_noise (z3, Q, 6);
    add_noise (z3b, Qb, 5);
    add_noise (z4, Q, 6);
    add_noise (z5, Q, 6);

    
    // -------------------------------------------------------------------------
    // build the original graph
    Slam slam;
    
    // nodes
    Pose3d_Node* n1 = new Pose3d_Node();
    Pose3d_Node* n2 = new Pose3d_Node();
    Pose3d_Node* n3 = new Pose3d_Node();
    // add nodes to graph
    slam.add_node(n1); slam.add_node(n2); slam.add_node(n3);
      
    // create factors and add them to the graph
    Pose3d z0p(x1[0], x1[1], x1[2], x1[5], x1[4], x1[3]);
    Pose3d_Factor* z0f = new Pose3d_Factor(n1, z0p, Qsqinf);
    slam.add_factor(z0f);
    
    Pose3d z1p(z1[0], z1[1], z1[2], z1[5], z1[4], z1[3]); 
    Pose3d_Pose3d_Factor* z1f = new Pose3d_Pose3d_Factor(n1, n2, z1p, Qsqinf);
    slam.add_factor(z1f);
    
    Pose3d z2p(z2[0], z2[1], z2[2], z2[5], z2[4], z2[3]); 
    Pose3d_Pose3d_Factor* z2f = new Pose3d_Pose3d_Factor(n2, n3, z2p, Qsqinf);
    slam.add_factor(z2f);
    
    //Pose3db z3bp(z3b[0], z3b[1], z3b[4], z3b[3], z3b[2]);
    //Pose3db_Pose3db_Factor* z3bf = new Pose3db_Pose3db_Factor(n1, n3, z3bp, Qbsqinf);
    //slam.add_factor(z3bf);
    
    Pose3d z3p(z3[0], z3[1], z3[2], z3[5], z3[4], z3[3]); 
    Pose3d_Pose3d_Factor* z3f = new Pose3d_Pose3d_Factor(n1, n3, z3p, Qsqinf);
    slam.add_factor(z3f);

    // optimize the graph
    slam.batch_optimization();
    
    // get covariance
    const Covariances& covariances = slam.covariances();
    std::list< isam::Node* > node_list;
    node_list.push_back(n1); node_list.push_back(n2); node_list.push_back(n3);
    MatrixXd cov_full = covariances.marginal(node_list);
    MatrixXd A = matrix_of_sparseMatrix(slam.jacobian());
    MatrixXd L = A.transpose()*A; 
    //MatrixXd L = cov_full.inverse();
    
    // print the output
    cout << "mu_x1: " << n1->value() << endl;
    cout << "mu_x2: " << n2->value() << endl;
    cout << "mu_x3: " << n3->value() << endl;
    //cout << "Sigma:" << endl;
    //cout << cov_full << endl << endl;
    
    // -------------------------------------------------------------------------
    // generate a new set of measurements
    // choose x1 as base
    double z1_hat[6] = {0};
    double z2_hat[6] = {0};
    double z3_hat[6] = {0}; //x1 to x3
    Matrix<double, 6, 12, RowMajor> J;
    double mu_x1[6] = {n1->value().x(), n1->value().y(), n1->value().z(),
                       n1->value().roll(), n1->value().pitch(), n1->value().yaw()};
    double mu_x2[6] = {n2->value().x(), n2->value().y(), n2->value().z(),
                       n2->value().roll(), n2->value().pitch(), n2->value().yaw()};
    double mu_x3[6] = {n3->value().x(), n3->value().y(), n3->value().z(),
                       n3->value().roll(), n3->value().pitch(), n3->value().yaw()};

    ssc_tail2tail(z1_hat, J.data(), mu_x1, mu_x2);
    Matrix<double, 6, 6, RowMajor> H1_x1 = J.topLeftCorner(6,6);
    Matrix<double, 6, 6, RowMajor> H1_x2 = J.topRightCorner(6,6);
    ssc_tail2tail(z2_hat, J.data(), mu_x2, mu_x3);
    Matrix<double, 6, 6, RowMajor> H2_x2 = J.topLeftCorner(6,6);
    Matrix<double, 6, 6, RowMajor> H2_x3 = J.topRightCorner(6,6);
    ssc_tail2tail(z3_hat, J.data(), mu_x1, mu_x3);
    Matrix<double, 6, 6, RowMajor> H3_x1 = J.topLeftCorner(6,6);
    Matrix<double, 6, 6, RowMajor> H3_x3 = J.topRightCorner(6,6);
    
    //solve for the measurement uncertianties
    Matrix<double, 6, 6, RowMajor> L_x1x2 = order_isam2van(L.block(0,6,6,6));
    Matrix<double, 6, 6, RowMajor> L_x2x1 = order_isam2van(L.block(6,0,6,6));
    Matrix<double, 6, 6, RowMajor> Omega_12_a = H1_x1.transpose().inverse() * L_x1x2 * H1_x2.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_12_b = H1_x2.transpose().inverse() *L_x2x1 * H1_x1.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_12_hat = order_isam2van((Omega_12_a +
                                                                  Omega_12_a.transpose() +
                                                                  Omega_12_b +
                                                                  Omega_12_b.transpose())/4.0);
    
    Matrix<double, 6, 6, RowMajor> L_x2x3 = order_isam2van(L.block(6,12,6,6));
    Matrix<double, 6, 6, RowMajor> L_x3x2 = order_isam2van(L.block(12,6,6,6));
    Matrix<double, 6, 6, RowMajor> Omega_23_a = H2_x2.transpose().inverse() * L_x2x3 * H2_x3.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_23_b = H2_x3.transpose().inverse() *L_x3x2 * H2_x2.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_23_hat  = order_isam2van((Omega_23_a +
                                                                   Omega_23_a.transpose() +
                                                                   Omega_23_b +
                                                                   Omega_23_b.transpose())/4.0);
    
    Matrix<double, 6, 6, RowMajor> L_x1x3 = order_isam2van(L.block(0,12,6,6));
    Matrix<double, 6, 6, RowMajor> L_x3x1 = order_isam2van(L.block(12,0,6,6));
    Matrix<double, 6, 6, RowMajor> Omega_13_a = H3_x1.transpose().inverse() * L_x1x3 * H3_x3.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_13_b = H3_x3.transpose().inverse() *L_x3x1 * H3_x1.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_13_hat  = order_isam2van((Omega_13_a +
                                                                   Omega_13_a.transpose() +
                                                                   Omega_13_b +
                                                                   Omega_13_b.transpose())/4.0);

//cout << "L_x1x3 = [" << endl << L_x1x3 << "];" << endl;
//cout << "H3_x1 = [" << endl << H3_x1 << "];" << endl;
//cout << "H3_x3 = [" << endl << H3_x3 << "];" << endl;
//cout << "Omega_13_a = [" << endl << Omega_13_a << "];" << endl;
//cout << "Omega_13_b = [" << endl << Omega_13_b << "];" << endl;
//cout << "Omega_13_hat = [" << endl << Omega_13_hat << "];" << endl;   
  
    // -------------------------------------------------------------------------
    // build a new graph with the relative links
    Slam slam_hat;
    
    // nodes
    Pose3d_Node* n1_hat = new Pose3d_Node();
    Pose3d_Node* n2_hat = new Pose3d_Node();
    Pose3d_Node* n3_hat = new Pose3d_Node();
    // add nodes to graph
    slam_hat.add_node(n1_hat); slam_hat.add_node(n2_hat); slam_hat.add_node(n3_hat);
      
    // create factors
    // all factors not directly between x1, x2, and or x3
    // this includes partial factors
    Pose3d_Factor* z0f_hat = new Pose3d_Factor(n1_hat, z0p, Qsqinf);
    // all synthetic measurements   
    Pose3d z1p_hat(z1_hat[0], z1_hat[1], z1_hat[2], z1_hat[5], z1_hat[4], z1_hat[3]); 
    Pose3d_Pose3d_Factor* z1f_hat = new Pose3d_Pose3d_Factor(n1_hat, n2_hat, z1p_hat, Information(Omega_12_hat));
    Pose3d z2p_hat(z2_hat[0], z2_hat[1], z2_hat[2], z2_hat[5], z2_hat[4], z2_hat[3]);
    Pose3d_Pose3d_Factor* z2f_hat = new Pose3d_Pose3d_Factor(n2_hat, n3_hat, z2p_hat, Information(Omega_23_hat));
    Pose3d z3p_hat(z3_hat[0], z3_hat[1], z3_hat[2], z3_hat[5], z3_hat[4], z3_hat[3]);
    Pose3d_Pose3d_Factor* z3f_hat = new Pose3d_Pose3d_Factor(n1_hat, n3_hat, z3p_hat, Information(Omega_13_hat));
    
    
    // add factors to the graph
    slam_hat.add_factor(z0f_hat);
    slam_hat.add_factor(z1f_hat);
    slam_hat.add_factor(z2f_hat);
    slam_hat.add_factor(z3f_hat);

    // optimize the graph
    slam_hat.batch_optimization();
    
    // get covariance
    const Covariances& covariances_hat = slam_hat.covariances();
    std::list< isam::Node* > node_list_hat;
    node_list_hat.push_back(n1_hat); node_list_hat.push_back(n2_hat); node_list_hat.push_back(n3_hat);
    MatrixXd cov_full_hat = covariances_hat.marginal(node_list_hat);
    MatrixXd A_hat = matrix_of_sparseMatrix(slam_hat.jacobian());
    MatrixXd L_hat = A_hat.transpose()*A_hat; 
    
    // print the output
    cout << "mu_x1: " << n1->value() << endl;
    cout << "mu_x2: " << n2->value() << endl;
    cout << "mu_x3: " << n3->value() << endl;
    //cout << "Sigma:" << endl;
    //cout << cov_full_hat << endl << endl;
    
    cout << "Sigma difference:" << endl;
    MatrixXd diff = (cov_full_hat-cov_full).array().abs();
    MatrixXd diff_out(18,18);
    for (int i=0; i<18; i++) {
        for (int j=0; j<18; j++) {
            
            if (diff(i,j) > 1e-6)
                diff_out(i,j) = diff(i,j);
            else
                diff_out(i,j) = 0;
        }
    }
    //cout << (diff_out) << endl << endl;
    cout << "difference, mean = " << diff.mean();
    cout << " max = " << diff.lpNorm<Infinity>() << endl;
    
    cout << "Information difference:" << endl;
    diff = (L_hat-L).array().abs();
    for (int i=0; i<18; i++) {
        for (int j=0; j<18; j++) {
            
            if (diff(i,j) > 1e-6)
                diff_out(i,j) = diff(i,j);
            else
                diff_out(i,j) = 0;
        }
    }
    //cout << (diff_out) << endl << endl;
    cout << "difference, mean = " << diff.mean();
    cout << " max = " << diff.lpNorm<Infinity>() << endl;
    
    
    //--------------------------------------------------------------------------
    // add new poses with loop closure and makesure they are the same
    Pose3d_Node* n4 = new Pose3d_Node();
    slam.add_node(n4);   
    Pose3d_Node* n4_hat = new Pose3d_Node();
    slam_hat.add_node(n4_hat); 
    
    Pose3d z4p(z4[0], z4[1], z4[2], z4[5], z4[4], z4[3]); 
    Pose3d_Pose3d_Factor* z4f = new Pose3d_Pose3d_Factor(n3, n4, z4p, Qsqinf);
    Pose3d_Pose3d_Factor* z4f_hat = new Pose3d_Pose3d_Factor(n3_hat, n4_hat, z4p, Qsqinf);
    slam.add_factor(z4f);
    slam_hat.add_factor(z4f_hat);
    
    Pose3d z5p(z5[0], z5[1], z5[2], z5[5], z5[4], z5[3]);
    Pose3d_Pose3d_Factor* z5f = new Pose3d_Pose3d_Factor(n1, n4, z5p, Qsqinf);
    Pose3d_Pose3d_Factor* z5f_hat = new Pose3d_Pose3d_Factor(n1_hat, n4_hat, z5p, Qsqinf);
    slam.add_factor(z5f);
    slam_hat.add_factor(z5f_hat);
    
    
    // optimize the graphs
    slam.batch_optimization();
    slam_hat.batch_optimization();
    
    // get covariances
    const Covariances& covariances1 = slam.covariances();
    node_list.clear();
    node_list.push_back(n1); node_list.push_back(n2);
    node_list.push_back(n3); node_list.push_back(n4);
    cov_full = covariances1.marginal(node_list);
    A = matrix_of_sparseMatrix(slam.jacobian());
    L = A.transpose()*A;
    
    const Covariances& covariances1_hat = slam_hat.covariances();
    node_list_hat.clear();
    node_list_hat.push_back(n1_hat); node_list_hat.push_back(n2_hat);
    node_list_hat.push_back(n3_hat); node_list_hat.push_back(n4_hat);
    cov_full_hat = covariances1_hat.marginal(node_list_hat);
    A_hat = matrix_of_sparseMatrix(slam_hat.jacobian());
    L_hat = A_hat.transpose()*A_hat; 
    
    // print the output
    cout << "mu_x1: " << n1->value() << endl;
    cout << "mu_x2: " << n2->value() << endl;
    cout << "mu_x3: " << n3->value() << endl;
    cout << "mu_x4: " << n4->value() << endl;
    cout << "mu_x1: " << n1_hat->value() << endl;
    cout << "mu_x2: " << n2_hat->value() << endl;
    cout << "mu_x3: " << n3_hat->value() << endl;
    cout << "mu_x4: " << n4_hat->value() << endl;
    
    cout << "Sigma difference:" << endl;
    diff = (cov_full_hat-cov_full).array().abs();
    diff_out = diff;
    for (int i=0; i<24; i++) {
        for (int j=0; j<24; j++) {
            
            if (diff(i,j) > 1e-6)
                diff_out(i,j) = diff(i,j);
            else
                diff_out(i,j) = 0;
        }
    }
    //cout << (diff_out) << endl << endl;
    cout << "difference, mean = " << diff.mean();
    cout << " max = " << diff.lpNorm<Infinity>() << endl;
    
    cout << "Information difference:" << endl;
    diff = (L_hat-L).array().abs();
    for (int i=0; i<18; i++) {
        for (int j=0; j<18; j++) {
            
            if (diff(i,j) > 1e-6)
                diff_out(i,j) = diff(i,j);
            else
                diff_out(i,j) = 0;
        }
    }
    //cout << (diff_out) << endl << endl;
    cout << "difference, mean = " << diff.mean();
    cout << " max = " << diff.lpNorm<Infinity>() << endl;
}
































































void
test_lift_constraints () {
    
    // known poses
    double x1[6] = {0,  0,  0,  0,          M_PI/12.0, -M_PI/6.0};
    double x2[6] = {10, 10, 0,  0,          0,          M_PI/4.0};
    double x3[6] = {20, 20, 0, -M_PI/10.0,  M_PI/10.0,  M_PI/8.0};
    
    // build measurements
    double z1[6], z2[6], z3[6];
    ssc_tail2tail(z1, NULL, x1, x2);
    ssc_tail2tail(z2, NULL, x2, x3);
    ssc_tail2tail(z3, NULL, x1, x3);
    
    // add a bearing only measurement
    double z3i[6];
    ssc_inverse(z3i, NULL, z3);
    gsl_vector_view z3i_v = gsl_vector_view_array (z3i, 6);
    GSLU_MATRIX_VIEW (Sigma3, 6, 6, {0});
    double z3b[5];
    gsl_vector_view z3b_v = gsl_vector_view_array (z3b, 5);
    GSLU_MATRIX_VIEW (Sigma3b, 5, 5);
    dm_trans2dm_pose_cov (&z3i_v.vector, &Sigma3.matrix, &z3b_v.vector, &Sigma3b.matrix, NULL);
    
    // add noise to measurements
    
    
    // -------------------------------------------------------------------------
    // build the original graph
    Slam slam;
    
    Matrix<double, 6, 6> Q = 1e-1 * Matrix<double, 6, 6>::Identity();
    Noise Qsqinf = Information(Q.inverse());
    Matrix<double, 5, 5> Qb= 1e-1 * Matrix<double, 5, 5>::Identity();
    Noise Qbsqinf = Information(Qb.inverse());
  
    // nodes
    Pose3d_Node* n1 = new Pose3d_Node();
    Pose3d_Node* n2 = new Pose3d_Node();
    Pose3d_Node* n3 = new Pose3d_Node();
    // add nodes to graph
    slam.add_node(n1); slam.add_node(n2); slam.add_node(n3);
      
    // create factors and add them to the graph
    Pose3d z0p(x1[0], x1[1], x1[2], x1[5], x1[4], x1[3]);
    Pose3d_Factor* z0f = new Pose3d_Factor(n1, z0p, Qsqinf);
    slam.add_factor(z0f);
    
    Pose3d z1p(z1[0], z1[1], z1[2], z1[5], z1[4], z1[3]); 
    Pose3d_Pose3d_Factor* z1f = new Pose3d_Pose3d_Factor(n1, n2, z1p, Qsqinf);
    slam.add_factor(z1f);
    
    Pose3d z2p(z2[0], z2[1], z2[2], z2[5], z2[4], z2[3]); 
    Pose3d_Pose3d_Factor* z2f = new Pose3d_Pose3d_Factor(n2, n3, z2p, Qsqinf);
    slam.add_factor(z2f);
    
    Pose3db z3bp(z3b[0], z3b[1], z3b[4], z3b[3], z3b[2]);
    Pose3db_Pose3db_Factor* z3bf = new Pose3db_Pose3db_Factor(n1, n3, z3bp, Qbsqinf);
    slam.add_factor(z3bf);
    
    Pose3d z3p(z3[0], z3[1], z3[2], z3[5], z3[4], z3[3]); 
    Pose3d_Pose3d_Factor* z3f = new Pose3d_Pose3d_Factor(n1, n3, z3p, Qsqinf);
    slam.add_factor(z3f);

    // optimize the graph
    slam.batch_optimization();
    
    // get covariance
    const Covariances& covariances = slam.covariances();
    std::list< isam::Node* > node_list;
    node_list.push_back(n1); node_list.push_back(n2); node_list.push_back(n3);
    MatrixXd cov_full = covariances.marginal(node_list);
    MatrixXd A = matrix_of_sparseMatrix(slam.jacobian());
    MatrixXd L = A.transpose()*A; 
    //MatrixXd L = cov_full.inverse();
    
    // print the output
    cout << "mu_x1: " << n1->value() << endl;
    cout << "mu_x2: " << n2->value() << endl;
    cout << "mu_x3: " << n3->value() << endl;
    //cout << "Sigma:" << endl;
    //cout << cov_full << endl << endl;
    
    // -------------------------------------------------------------------------
    // generate a new set of measurements
    // choose x1 as base
    double z1_hat[6] = {0};
    double z2_hat[6] = {0};
    double z3_hat[6] = {0}; //x1 to x3
    Matrix<double, 6, 12, RowMajor> J;
    double mu_x1[6] = {n1->value().x(), n1->value().y(), n1->value().z(),
                       n1->value().roll(), n1->value().pitch(), n1->value().yaw()};
    double mu_x2[6] = {n2->value().x(), n2->value().y(), n2->value().z(),
                       n2->value().roll(), n2->value().pitch(), n2->value().yaw()};
    double mu_x3[6] = {n3->value().x(), n3->value().y(), n3->value().z(),
                       n3->value().roll(), n3->value().pitch(), n3->value().yaw()};
#if 1 // non-linear constraints
    ssc_tail2tail(z1_hat, J.data(), mu_x1, mu_x2);
    Matrix<double, 6, 6, RowMajor> H1_x1 = J.topLeftCorner(6,6);
    Matrix<double, 6, 6, RowMajor> H1_x2 = J.topRightCorner(6,6);
    ssc_tail2tail(z2_hat, J.data(), mu_x2, mu_x3);
    Matrix<double, 6, 6, RowMajor> H2_x2 = J.topLeftCorner(6,6);
    Matrix<double, 6, 6, RowMajor> H2_x3 = J.topRightCorner(6,6);
    ssc_tail2tail(z3_hat, J.data(), mu_x1, mu_x3);
    Matrix<double, 6, 6, RowMajor> H3_x1 = J.topLeftCorner(6,6);
    Matrix<double, 6, 6, RowMajor> H3_x3 = J.topRightCorner(6,6);
#else // linear difference constraints NEED TO IMPLEMENT FACTOR FOR THIS TO WORK
    z1_hat[0] = mu_x2[0] - mu_x1[0]; z1_hat[1] = mu_x2[1] - mu_x1[1]; z1_hat[2] = mu_x2[2] - mu_x1[2];
    z1_hat[3] = bot_mod2pi(mu_x2[3] - mu_x1[3]);
    z1_hat[4] = bot_mod2pi(mu_x2[4] - mu_x1[4]);
    z1_hat[5] = bot_mod2pi(mu_x2[5] - mu_x1[5]);
    Matrix<double, 6, 6, RowMajor> H1_x1 = -1.0 * Matrix<double, 6, 6>::Identity();
    Matrix<double, 6, 6, RowMajor> H1_x2 = Matrix<double, 6, 6>::Identity();
    z2_hat[0] = mu_x3[0] - mu_x2[0]; z2_hat[1] = mu_x3[1] - mu_x2[1]; z2_hat[2] = mu_x3[2] - mu_x2[2];
    z2_hat[3] = bot_mod2pi(mu_x3[3] - mu_x2[3]);
    z2_hat[4] = bot_mod2pi(mu_x3[4] - mu_x2[4]);
    z2_hat[5] = bot_mod2pi(mu_x3[5] - mu_x2[5]);
    Matrix<double, 6, 6, RowMajor> H2_x2 = -1.0 * Matrix<double, 6, 6>::Identity();
    Matrix<double, 6, 6, RowMajor> H2_x3 = Matrix<double, 6, 6>::Identity();
    z3_hat[0] = mu_x3[0] - mu_x1[0]; z3_hat[1] = mu_x3[1] - mu_x1[1]; z3_hat[2] = mu_x3[2] - mu_x1[2];
    z3_hat[3] = bot_mod2pi(mu_x3[3] - mu_x1[3]);
    z3_hat[4] = bot_mod2pi(mu_x3[4] - mu_x1[4]);
    z3_hat[5] = bot_mod2pi(mu_x3[5] - mu_x1[5]);
    Matrix<double, 6, 6, RowMajor> H3_x1 = -1.0 * Matrix<double, 6, 6>::Identity();
    Matrix<double, 6, 6, RowMajor> H3_x3 = Matrix<double, 6, 6>::Identity();
#endif
    
    //solve for the measurement uncertianties
    Matrix<double, 6, 6, RowMajor> L_x1x2 = order_isam2van(L.block(0,6,6,6));
    Matrix<double, 6, 6, RowMajor> L_x2x1 = order_isam2van(L.block(6,0,6,6));
    Matrix<double, 6, 6, RowMajor> Omega_12_a = H1_x1.transpose().inverse() * L_x1x2 * H1_x2.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_12_b = H1_x2.transpose().inverse() *L_x2x1 * H1_x1.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_12_hat = order_isam2van((Omega_12_a +
                                                                  Omega_12_a.transpose() +
                                                                  Omega_12_b +
                                                                  Omega_12_b.transpose())/4.0);
    
    Matrix<double, 6, 6, RowMajor> L_x2x3 = order_isam2van(L.block(6,12,6,6));
    Matrix<double, 6, 6, RowMajor> L_x3x2 = order_isam2van(L.block(12,6,6,6));
    Matrix<double, 6, 6, RowMajor> Omega_23_a = H2_x2.transpose().inverse() * L_x2x3 * H2_x3.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_23_b = H2_x3.transpose().inverse() *L_x3x2 * H2_x2.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_23_hat  = order_isam2van((Omega_23_a +
                                                                   Omega_23_a.transpose() +
                                                                   Omega_23_b +
                                                                   Omega_23_b.transpose())/4.0);
    
    Matrix<double, 6, 6, RowMajor> L_x1x3 = order_isam2van(L.block(0,12,6,6));
    Matrix<double, 6, 6, RowMajor> L_x3x1 = order_isam2van(L.block(12,0,6,6));
    Matrix<double, 6, 6, RowMajor> Omega_13_a = H3_x1.transpose().inverse() * L_x1x3 * H3_x3.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_13_b = H3_x3.transpose().inverse() *L_x3x1 * H3_x1.inverse();
    Matrix<double, 6, 6, RowMajor> Omega_13_hat  = order_isam2van((Omega_13_a +
                                                                   Omega_13_a.transpose() +
                                                                   Omega_13_b +
                                                                   Omega_13_b.transpose())/4.0);

cout << "L_x1x3 = [" << endl << L_x1x3 << "];" << endl;
cout << "H3_x1 = [" << endl << H3_x1 << "];" << endl;
cout << "H3_x3 = [" << endl << H3_x3 << "];" << endl;
cout << "Omega_13_a = [" << endl << Omega_13_a << "];" << endl;
cout << "Omega_13_b = [" << endl << Omega_13_b << "];" << endl;
cout << "Omega_13_hat = [" << endl << Omega_13_hat << "];" << endl;   
    
    // -------------------------------------------------------------------------
    // build a new graph with the relative links
    Slam slam_hat;
    
    // nodes
    Pose3d_Node* n1_hat = new Pose3d_Node();
    Pose3d_Node* n2_hat = new Pose3d_Node();
    Pose3d_Node* n3_hat = new Pose3d_Node();
    // add nodes to graph
    slam_hat.add_node(n1_hat); slam_hat.add_node(n2_hat); slam_hat.add_node(n3_hat);
      
    // create factors
    // all factors not directly between x1, x2, and or x3
    // this includes partial factors
    Pose3d_Factor* z0f_hat = new Pose3d_Factor(n1_hat, z0p, Qsqinf);
    // all synthetic measurements   
    Pose3d z1p_hat(z1_hat[0], z1_hat[1], z1_hat[2], z1_hat[5], z1_hat[4], z1_hat[3]); 
    Pose3d_Pose3d_Factor* z1f_hat = new Pose3d_Pose3d_Factor(n1_hat, n2_hat, z1p_hat, Information(Omega_12_hat));
    Pose3d z2p_hat(z2_hat[0], z2_hat[1], z2_hat[2], z2_hat[5], z2_hat[4], z2_hat[3]);
    Pose3d_Pose3d_Factor* z2f_hat = new Pose3d_Pose3d_Factor(n2_hat, n3_hat, z2p_hat, Information(Omega_23_hat));
    Pose3d z3p_hat(z3_hat[0], z3_hat[1], z3_hat[2], z3_hat[5], z3_hat[4], z3_hat[3]);
    Pose3d_Pose3d_Factor* z3f_hat = new Pose3d_Pose3d_Factor(n1_hat, n3_hat, z3p_hat, Information(Omega_13_hat));
    
    
    // add factors to the graph
    slam_hat.add_factor(z0f_hat);
    slam_hat.add_factor(z1f_hat);
    slam_hat.add_factor(z2f_hat);
    slam_hat.add_factor(z3f_hat);

    // optimize the graph
    slam_hat.batch_optimization();
    
    // get covariance
    const Covariances& covariances_hat = slam_hat.covariances();
    std::list< isam::Node* > node_list_hat;
    node_list_hat.push_back(n1_hat); node_list_hat.push_back(n2_hat); node_list_hat.push_back(n3_hat);
    MatrixXd cov_full_hat = covariances_hat.marginal(node_list_hat);
    MatrixXd A_hat = matrix_of_sparseMatrix(slam_hat.jacobian());
    MatrixXd L_hat = A_hat.transpose()*A_hat; 
    
    // print the output
    cout << "mu_x1: " << n1->value() << endl;
    cout << "mu_x2: " << n2->value() << endl;
    cout << "mu_x3: " << n3->value() << endl;
    //cout << "Sigma:" << endl;
    //cout << cov_full_hat << endl << endl;
    
    cout << "Sigma difference:" << endl;
    MatrixXd diff = (cov_full_hat-cov_full).array().abs();
    MatrixXd diff_out(18,18);
    for (int i=0; i<18; i++) {
        for (int j=0; j<18; j++) {
            
            if (diff(i,j) > 1e-6)
                diff_out(i,j) = diff(i,j);
            else
                diff_out(i,j) = 0;
        }
    }
    cout << (diff_out) << endl << endl;
    cout << "difference, mean = " << diff.mean();
    cout << " max = " << diff.lpNorm<Infinity>() << endl;
    
    cout << "Information difference:" << endl;
    diff = (L_hat-L).array().abs();
    for (int i=0; i<18; i++) {
        for (int j=0; j<18; j++) {
            
            if (diff(i,j) > 1e-6)
                diff_out(i,j) = diff(i,j);
            else
                diff_out(i,j) = 0;
        }
    }
    cout << (diff_out) << endl << endl;
    cout << "difference, mean = " << diff.mean();
    cout << " max = " << diff.lpNorm<Infinity>() << endl; 
    
}

void
test_sparsify () {
    
    // known poses
    double x1[6] = {0,0,0,0,0,0};
    //double x2[6] = {1,1,0,0,0,M_PI/4.0};
    //double x2[6] = {2,2,0,0,0,0.0};
    double x2[6] = {10,0,0,0,0,0};
    double x3[6] = {20,0,0,0,0,0};
    
    // build measurements
    double z1[6], z2[6], z3[6];
    ssc_tail2tail(z1, NULL, x1, x2);
    ssc_tail2tail(z2, NULL, x2, x3);
    ssc_tail2tail(z3, NULL, x1, x3);
    
    // add noise to measurements
    z3[0] = z3[0] + 0.1;
        
    // -------------------------------------------------------------------------
    // build the original graph
    Slam slam;
    
    Matrix<double, 6, 6> Q = 1.0/1000.0 * Matrix<double, 6, 6>::Identity();
    Noise Qsqinf = Information(Q.inverse());
  
    // nodes
    Pose3d_Node* n1 = new Pose3d_Node();
    Pose3d_Node* n2 = new Pose3d_Node();
    Pose3d_Node* n3 = new Pose3d_Node();
    // add nodes to graph
    slam.add_node(n1); slam.add_node(n2); slam.add_node(n3);
      
    // create factors
    Pose3d z0p(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    Pose3d_Factor* z0f = new Pose3d_Factor(n1, z0p, Qsqinf);
    Pose3d z1p(z1[0], z1[1], z1[2], z1[5], z1[4], z1[3]); 
    Pose3d_Pose3d_Factor* z1f = new Pose3d_Pose3d_Factor(n1, n2, z1p, Qsqinf);
    Pose3d z2p(z2[0], z2[1], z2[2], z2[5], z2[4], z2[3]); 
    Pose3d_Pose3d_Factor* z2f = new Pose3d_Pose3d_Factor(n2, n3, z2p, Qsqinf);
    Pose3d z3p(z3[0], z3[1], z3[2], z3[5], z3[4], z3[3]); 
    Pose3d_Pose3d_Factor* z3f = new Pose3d_Pose3d_Factor(n1, n3, z3p, Qsqinf);
    
    // add factors to the graph
    slam.add_factor(z0f); slam.add_factor(z1f); slam.add_factor(z2f); slam.add_factor(z3f);

    // optimize the graph
    slam.batch_optimization();
    
    // get covariance
    const Covariances& covariances = slam.covariances();
    std::list< isam::Node* > node_list;
    node_list.push_back(n1); node_list.push_back(n2); node_list.push_back(n3);
    MatrixXd cov_full = covariances.marginal(node_list);
    
    // print the output
    cout << "mu_x1: " << n1->value() << endl;
    cout << "mu_x2: " << n2->value() << endl;
    cout << "mu_x3: " << n3->value() << endl;
    //cout << "Sigma:" << endl;
    //cout << cov_full << endl << endl;
    
    // -------------------------------------------------------------------------
    // marganalize out x2 (just strike rows in covariance form)
    MatrixXd S_x1x3(12,12);
    S_x1x3 << cov_full.topLeftCorner(6,6),    cov_full.topRightCorner(6,6),
              cov_full.bottomLeftCorner(6,6), cov_full.bottomRightCorner(6,6);
    //cout << "Sigma_x1x3:" << endl;
    //cout << S_x1x3 << endl << endl;
    
    
    // -------------------------------------------------------------------------
    // build the marganilized graph directly
    Slam slam_p;
    
    // nodes
    Pose3d_Node* n1_p = new Pose3d_Node();
    Pose3d_Node* n3_p = new Pose3d_Node();
    // add nodes to graph
    slam_p.add_node(n1_p); slam_p.add_node(n3_p);
      
    // create compound factors
#if 0
    double z12[6];
    Matrix<double, 6, 12, RowMajor> J;
    ssc_head2tail(z12, J.data(), z1, z2);
    
    Matrix<double, 6, 6, RowMajor> Q12;
    Q12 = J.topLeftCorner(6,6)*Q*J.topLeftCorner(6,6).transpose() +
          J.topRightCorner(6,6)*Q*J.topRightCorner(6,6).transpose();

    // rearange output
    Matrix<double, 6, 6, RowMajor> Q12r;
    int row, col;
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            row = i, col = j;
            if (i == 3) row = 5;
            if (i == 5) row = 3;
            if (j == 3) col = 5;
            if (j == 5) col = 3;
            //Q12r(i,j) = Q12(row,col);
            Q12r(row,col) = Q12(i,j);
        }
    }
    Noise Q12_sqinf = Information(Q12r.inverse());
    
    Pose3d_Factor* z0fp = new Pose3d_Factor(n1_p, z0p, Qsqinf);
    Pose3d z12p(z12[0], z12[1], z12[2], z12[5], z12[4], z12[3]); 
    Pose3d_Pose3d_Factor* z12f = new Pose3d_Pose3d_Factor(n1_p, n3_p, z12p, Q12_sqinf);
    Pose3d_Pose3d_Factor* z3fp = new Pose3d_Pose3d_Factor(n1_p, n3_p, z3p, Qsqinf);
    
    // add factors to the graph
    slam_p.add_factor(z0fp);     // same as original
    slam_p.add_factor(z12f); 
    slam_p.add_factor(z3fp);     // same as original
#else
    double z12[6];
    Matrix<double, 6, 12, RowMajor> J;
    ssc_head2tail(z12, J.data(), z1, z2);
    
    Matrix<double, 6, 6, RowMajor> Q12;
    Q12 = J.topLeftCorner(6,6)*Q*J.topLeftCorner(6,6).transpose() +
          J.topRightCorner(6,6)*Q*J.topRightCorner(6,6).transpose();

    // combination of uncertianties
    Matrix<double, 6, 6, RowMajor> Qc = (Q12.inverse() + Q.inverse()).inverse();
    
    // find new mean
    Matrix<double, 6, 1> z12e(z12);
    Matrix<double, 6, 1> z3e(z3);
    Matrix<double, 6, 1> zc;
    zc = Qc*Q12.inverse()*z12e + Qc*Q.inverse()*z3e;
    
    // rearange output
    Matrix<double, 6, 6, RowMajor> Qcr;
    int row, col;
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            row = i, col = j;
            if (i == 3) row = 5;
            if (i == 5) row = 3;
            if (j == 3) col = 5;
            if (j == 5) col = 3;
            //Q12r(i,j) = Q12(row,col);
            Qcr(row,col) = Qc(i,j);
        }
    }
    Noise Qc_sqinf = Information(Qcr.inverse());
    
    Pose3d_Factor* z0fp = new Pose3d_Factor(n1_p, z0p, Qsqinf);
    Pose3d zc_p(zc[0], zc[1], zc[2], zc[5], zc[4], zc[3]); 
    Pose3d_Pose3d_Factor* zcf = new Pose3d_Pose3d_Factor(n1_p, n3_p, zc_p, Qc_sqinf);
    
    // add factors to the graph
    slam_p.add_factor(z0fp);     // same as original
    slam_p.add_factor(zcf); 

#endif

    // optimize the graph
    slam_p.batch_optimization();
    
    // get covariance
    const Covariances& covariances_p = slam_p.covariances();
    std::list< isam::Node* > node_list_p;
    node_list_p.push_back(n1_p); node_list_p.push_back(n3_p);
    MatrixXd S_x1x3_p = covariances_p.marginal(node_list_p);
    
    // print the output
    cout << "mu_x1: " << n1_p->value() << endl;
    cout << "mu_x3: " << n3_p->value() << endl;
    
    cout << "S_x1x3:" << endl;
    cout << S_x1x3 << endl << endl;
    
    cout << "S_x1x3_p:" << endl;
    cout << S_x1x3_p << endl << endl;
    
    cout << "Sigma diff:" << endl;
    cout << S_x1x3_p - S_x1x3 << endl << endl;
        
    cout << "Sigma diff: avg = " << (S_x1x3_p - S_x1x3).mean();
    cout << ", max = " << (S_x1x3_p - S_x1x3).maxCoeff() << endl;
}




void
PVNMapBuilder::run_sparsify_map () {
        
    // test
    // test_sparsify ();
    // test_lift_constraints ();
    test_lift_constraints_loop ();
}