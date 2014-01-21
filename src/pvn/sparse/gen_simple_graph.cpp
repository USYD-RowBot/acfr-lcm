#include <boost/filesystem.hpp>

#include "perls-math/ssc.h"
#include "perls-math/gsl_util.h"

#include "sparse.h"

using namespace std;
using namespace Eigen;
using namespace isam;

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
add_pose3d_pose3d_factor (Slam *slam, Pose3d_Node *ni, Pose3d_Node *nj, double z[6], Noise sqinf) {
    
    Pose3d zp(z[0], z[1], z[2], z[5], z[4], z[3]); 
    Pose3d_Pose3d_Factor* zf = new Pose3d_Pose3d_Factor(ni, nj, zp, sqinf);
    slam->add_factor(zf);
    
}

void
gen_y_yp (char *dir) {

    
    // known poses
    double x0[6] = {0,  0,  0,  0,          M_PI/12.0, -M_PI/6.0};
    double x1[6] = {10, 10, 0,  0,          0,          M_PI/4.0};
    double x2[6] = {20, 20, 0, -M_PI/10.0,  M_PI/10.0,  M_PI/8.0};
    double x3[6] = {30, 30, 0,  M_PI/10.0, -M_PI/8.0,   M_PI/4.0};
    double x4[6] = {40, 40, 0,  -M_PI/10.0, -M_PI/8.0,  -M_PI/4.0};
    
    // sequential mesurements
    double z01[6], z12[6], z23[6], z34[6];
    ssc_tail2tail(z01, NULL, x0, x1);
    ssc_tail2tail(z12, NULL, x1, x2);
    ssc_tail2tail(z23, NULL, x2, x3);
    ssc_tail2tail(z34, NULL, x3, x4);
    
    // non-sequential mesurements
    double z13[6];
    ssc_tail2tail(z13, NULL, x1, x3);
    
    // add noise to measurements
    Matrix<double, 6, 6, RowMajor> Q = 1e-1 * Matrix<double, 6, 6>::Identity();
    Noise Qsqinf = Information(Q.inverse());
    add_noise (z01, Q, 6);
    add_noise (z12, Q, 6);
    add_noise (z23, Q, 6);
    add_noise (z34, Q, 6);
    add_noise (z13, Q, 6);
    
    // nodes
    Pose3d_Node* n0 = new Pose3d_Node();
    Pose3d_Node* n1 = new Pose3d_Node();
    Pose3d_Node* n2 = new Pose3d_Node();
    Pose3d_Node* n3 = new Pose3d_Node();
    Pose3d_Node* n4 = new Pose3d_Node();
    
    // add nodes to graph
    Slam sl;    
    sl.add_node(n0); sl.add_node(n1); sl.add_node(n2), sl.add_node(n3);
      
    // create factors and add them to the graph
    Pose3d z0p(x0[0], x0[1], x0[2], x0[5], x0[4], x0[3]);
    Pose3d_Factor* z0f = new Pose3d_Factor(n0, z0p, Qsqinf);
    sl.add_factor(z0f);
    
    // add sequential factors    
    add_pose3d_pose3d_factor (&sl, n0, n1, z01, Qsqinf);
    add_pose3d_pose3d_factor (&sl, n1, n2, z12, Qsqinf);
    add_pose3d_pose3d_factor (&sl, n2, n3, z23, Qsqinf);

    // optimize the graph
    sl.batch_optimization();
    
    
    // write the chain
    char graph_dir[PATH_MAX];
    sprintf(graph_dir, "%s/chain", dir);
    boost::filesystem::remove_all(graph_dir);
    boost::filesystem::create_directory(graph_dir);
    char graph_file[PATH_MAX];
    sprintf (graph_file, "%s/graph.isam", graph_dir);
    sl.save(graph_file);
    char inds_to_marg_file[PATH_MAX];
    sprintf (inds_to_marg_file, "%s/inds_to_marg.txt", graph_dir);
    vector<int> inds_to_marg; inds_to_marg.push_back(1);
    write_inds (inds_to_marg_file, inds_to_marg);
    char pc_file[PATH_MAX];
    sprintf (pc_file, "%s/pc.dat", graph_dir);
    write_pc (pc_file, &sl, NULL);
    char processed_dir[PATH_MAX];
    sprintf(processed_dir, "%s/processed", graph_dir);
    boost::filesystem::create_directory(processed_dir);
    
    //--------------------------------------------------------------------------
    // add non sequential factors
    add_pose3d_pose3d_factor (&sl, n1, n3, z13, Qsqinf);
    
    // optimize the graph
    sl.batch_optimization();
    
    // write the y shaped graph
    sprintf(graph_dir, "%s/y", dir);
    boost::filesystem::remove_all(graph_dir);
    boost::filesystem::create_directory(graph_dir);
    sprintf (graph_file, "%s/graph.isam", graph_dir);
    sl.save(graph_file);
    sprintf (inds_to_marg_file, "%s/inds_to_marg.txt", graph_dir);
    write_inds (inds_to_marg_file, inds_to_marg);
    sprintf (pc_file, "%s/pc.dat", graph_dir);
    write_pc (pc_file, &sl, NULL);
    sprintf(processed_dir, "%s/processed", graph_dir);
    boost::filesystem::create_directory(processed_dir);
    
    //--------------------------------------------------------------------------
    // add non sequential factors
    sl.add_node(n4);
    add_pose3d_pose3d_factor (&sl, n3, n4, z34, Qsqinf);
    
    // optimize the graph
    sl.batch_optimization();
    
    // write the y shaped graph
    sprintf(graph_dir, "%s/yp", dir);
    boost::filesystem::remove_all(graph_dir);
    boost::filesystem::create_directory(graph_dir);
    sprintf (graph_file, "%s/graph.isam", graph_dir);
    sl.save(graph_file);
    inds_to_marg.push_back(3);
    sprintf (inds_to_marg_file, "%s/inds_to_marg.txt", graph_dir);
    write_inds (inds_to_marg_file, inds_to_marg);
    sprintf (pc_file, "%s/pc.dat", graph_dir);
    write_pc (pc_file, &sl, NULL);
    sprintf(processed_dir, "%s/processed", graph_dir);
    boost::filesystem::create_directory(processed_dir);
    
}

void
gen_dumbbell (char *dir) {
    
    // known poses
    double x0[6] = {5,  5,  0, 0, 0, -M_PI/6.0};
    double x1[6] = {0,  0,  0, 0, 0,  M_PI/4.0};
    double x2[6] = {0,  10, 0, 0, 0,  M_PI/8.0};
    double x3[6] = {10, 10, 0, 0, 0,  M_PI/4.0};
    double x4[6] = {10, 0,  0, 0, 0, -M_PI/4.0};
    double x5[6] = {50, 50, 0, 0, 0, -M_PI/6.0};
    double x6[6] = {60, 50, 0, 0, 0,  M_PI/4.0};
    double x7[6] = {60, 60, 0, 0, 0,  M_PI/8.0};
    double x8[6] = {50, 60, 0, 0, 0,  M_PI/4.0};
    double x9[6] = {55, 55, 0, 0, 0, -M_PI/4.0};
    
    // measurement noise
    Matrix<double, 6, 6, RowMajor> Qs; Qs.setZero();
    Qs.topLeftCorner<3,3>() = 1e-1 * Matrix<double, 3, 3>::Identity();
    Qs.bottomRightCorner<3,3>() = 1e-3 * Matrix<double, 3, 3>::Identity();
    Noise Qs_sqinf = Information(isamu_van2isam_eigen (Qs.inverse()));
    Matrix<double, 6, 6, RowMajor> Ql; Ql.setZero();
    Ql.topLeftCorner<3,3>() = 2000 * Matrix<double, 3, 3>::Identity();
    Ql(2,2) = 10;
    Ql.bottomRightCorner<3,3>() = 1e-3 * Matrix<double, 3, 3>::Identity();
    Ql(5,5) = 50;
    Noise Ql_sqinf = Information(isamu_van2isam_eigen (Ql.inverse()));

cout << "Qs" << endl << Qs << endl;
cout << "Ql" << endl << Ql << endl;

    // Sequential measurements
    double z01[6], z12[6], z23[6], z34[6], z45[6], z56[6], z67[6], z78[6], z89[6];
    ssc_tail2tail(z01, NULL, x0, x1);   add_noise (z01, Qs, 6);
    ssc_tail2tail(z12, NULL, x1, x2);   add_noise (z12, Qs, 6);
    ssc_tail2tail(z23, NULL, x2, x3);   add_noise (z23, Qs, 6);
    ssc_tail2tail(z34, NULL, x3, x4);   add_noise (z34, Qs, 6);
    ssc_tail2tail(z45, NULL, x4, x5);   add_noise (z45, Ql, 6); //weak
    ssc_tail2tail(z56, NULL, x5, x6);   add_noise (z56, Qs, 6); 
    ssc_tail2tail(z67, NULL, x6, x7);   add_noise (z67, Qs, 6);
    ssc_tail2tail(z78, NULL, x7, x8);   add_noise (z78, Qs, 6);
    ssc_tail2tail(z89, NULL, x8, x9);   add_noise (z89, Qs, 6);
    
    // inter-cluster mesurements  (first cluster)   
    double z02[6], z03[6], z04[6], z14[6];
    ssc_tail2tail(z02, NULL, x0, x2);   add_noise (z02, Qs, 6);
    ssc_tail2tail(z03, NULL, x0, x3);   add_noise (z03, Qs, 6);
    ssc_tail2tail(z04, NULL, x0, x4);   add_noise (z04, Qs, 6);
    ssc_tail2tail(z14, NULL, x1, x4);   add_noise (z14, Qs, 6);
    
    // inter-cluster mesurements  (first cluster)   
    double z59[6], z69[6], z79[6], z58[6];
    ssc_tail2tail(z59, NULL, x5, x9);   add_noise (z59, Qs, 6);
    ssc_tail2tail(z69, NULL, x6, x9);   add_noise (z69, Qs, 6);
    ssc_tail2tail(z79, NULL, x7, x9);   add_noise (z79, Qs, 6);
    ssc_tail2tail(z58, NULL, x5, x8);   add_noise (z58, Qs, 6);
    
    // nodes
    Pose3d_Node* n0 = new Pose3d_Node();
    Pose3d_Node* n1 = new Pose3d_Node();
    Pose3d_Node* n2 = new Pose3d_Node();
    Pose3d_Node* n3 = new Pose3d_Node();
    Pose3d_Node* n4 = new Pose3d_Node();
    Pose3d_Node* n5 = new Pose3d_Node();
    Pose3d_Node* n6 = new Pose3d_Node();
    Pose3d_Node* n7 = new Pose3d_Node();
    Pose3d_Node* n8 = new Pose3d_Node();
    Pose3d_Node* n9 = new Pose3d_Node();
    
    // add nodes to graph
    Slam sl;    
    sl.add_node(n0); sl.add_node(n1); sl.add_node(n2); sl.add_node(n3); sl.add_node(n4);
    sl.add_node(n5); sl.add_node(n6); sl.add_node(n7); sl.add_node(n8); sl.add_node(n9);

    // add prior factor
    Pose3d z0p(x0[0], x0[1], x0[2], x0[5], x0[4], x0[3]);
    Pose3d_Factor* z0f = new Pose3d_Factor(n0, z0p, Information(Matrix<double,6,6>::Identity()*1e-8));
    sl.add_factor(z0f);
    
    Pose3d z1p(x1[0], x1[1], x1[2], x1[5], x1[4], x1[3]);
    Pose3d_Factor* z1f = new Pose3d_Factor(n1, z1p, Qs_sqinf);
    sl.add_factor(z1f);

    // add factors for sequential measurements
    add_pose3d_pose3d_factor (&sl, n0, n1, z01, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n1, n2, z12, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n2, n3, z23, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n3, n4, z34, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n4, n5, z45, Ql_sqinf); // one weak link between sets
    add_pose3d_pose3d_factor (&sl, n5, n6, z56, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n6, n7, z67, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n7, n8, z78, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n8, n9, z89, Qs_sqinf);
    // add factors for non-sequential measurements
    add_pose3d_pose3d_factor (&sl, n0, n2, z02, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n0, n3, z03, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n0, n4, z04, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n1, n4, z14, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n5, n9, z59, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n6, n9, z69, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n7, n9, z79, Qs_sqinf);
    add_pose3d_pose3d_factor (&sl, n5, n8, z58, Qs_sqinf);

    // optimize the graph
    sl.batch_optimization();
    sl.print_stats();
    
    // write the data
    char graph_dir[PATH_MAX];
    sprintf(graph_dir, "%s/dumbbell", dir);
    boost::filesystem::remove_all(graph_dir);
    boost::filesystem::create_directory(graph_dir);
    char graph_file[PATH_MAX];
    sprintf (graph_file, "%s/graph.isam", graph_dir);
    sl.save(graph_file);
    char inds_to_marg_file[PATH_MAX];
    sprintf (inds_to_marg_file, "%s/inds_to_marg.txt", graph_dir);
    vector<int> inds_to_marg; inds_to_marg.push_back(0); inds_to_marg.push_back(9);
    write_inds (inds_to_marg_file, inds_to_marg);
    char pc_file[PATH_MAX];
    sprintf (pc_file, "%s/pc.dat", graph_dir);
    write_pc (pc_file, &sl, NULL);
    char processed_dir[PATH_MAX];
    sprintf(processed_dir, "%s/processed", graph_dir);
    boost::filesystem::create_directory(processed_dir);
    
    // measurements to add file
    double z45_add[6];
    ssc_tail2tail(z45_add, NULL, x4, x5); add_noise (z45_add, Qs, 6); //strong, add later
    Pose3d zp(z45_add[0], z45_add[1], z45_add[2], z45_add[5], z45_add[4], z45_add[3]); 
    Pose3d_Pose3d_Factor* zf = new Pose3d_Pose3d_Factor(n4, n5, zp, Qs_sqinf);
    char meas_to_add_file[PATH_MAX];
    sprintf (meas_to_add_file, "%s/meas_to_add.txt", graph_dir);
    vector<Factor*> factors;
    factors.push_back(zf);
    write_meas_to_add (meas_to_add_file, factors);
    
}

void
PVNSparse::run_gen_simple_graph (void) {

    //gen_y_yp (dir);
    
    gen_dumbbell (dir);
    
}