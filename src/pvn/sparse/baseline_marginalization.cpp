#include "sparse.h"

using namespace std;
using namespace Eigen;
using namespace isam;

void
PVNSparse::run_baseline_marginalization (void) {
    
    // load data
    Slam slam;
    setup_isam (&slam);
    
    map<int64_t, Pose3d_Node*> pose3d_nodes;
    load_isam_graph (&slam, &pose3d_nodes);
        
    // note now pc.pose[i].mu == nodes[i]->value()
    // should be pc.pose[i].mu == pose3d_nodes[pc.pose[i].utime]->value()
    // cout << pc.pose[1].mu[0] << " " << pc.pose[1].mu[2] << endl;
    // cout << nodes[1]->value().x() << " " << nodes[1]->value().z() << endl;
    
    vector<int> inds_to_keep;
    vector<int> inds_to_keep_full;
    
    // try to load inds
    char inds_to_marg_file_in[PATH_MAX];
    sprintf (inds_to_marg_file_in, "%s/inds_to_marg.txt", dir);
    vector<int> inds_to_marg = load_inds (inds_to_marg_file_in);
    
    if (inds_to_marg.size()) {
        
        for (size_t i=0; i<pose3d_nodes.size(); i++) {
            if (inds_to_marg.end() == find (inds_to_marg.begin(), inds_to_marg.end(), i)) {
                inds_to_keep.push_back(i);
                for (int j=0; j<6; j++) {
                    inds_to_keep_full.push_back(6*i + j);
                }
            }
        }
        cout << "[baseline]\tMarginalizing inds in file" << std::endl;
        
    } else {

        for (size_t i=0; i<pose3d_nodes.size(); i++) {            
            if (((marg_every && !(i % marg_every)) || (keep_every && (i % keep_every))) && i != 0) {
                inds_to_marg.push_back(i);
            } else {
                inds_to_keep.push_back(i);
                for (int j=0; j<6; j++) {
                    inds_to_keep_full.push_back(6*i + j);
                }
            }
        }
        
    }
    
    if (add_meas_after) {
        cout << "[baseline]\tAdding additional measurements" << std::endl;
        char meas_to_add_file[PATH_MAX];
        sprintf (meas_to_add_file, "%s/meas_to_add.txt", dir);
        vector<Factor*> f2add = load_meas_to_add (meas_to_add_file, pose3d_nodes);
        for (size_t i=0; i<f2add.size(); i++) {
            slam.add_factor(f2add[i]);
        }
        slam.batch_optimization();
    }
    
    slam.print_stats();
    //slam.save("test.isam");
    
    //int64_t tic = timestamp_now();
    //const Covariances& covariances = slam.covariances();
    //MatrixXd cov_full = covariances.marginal(slam.get_nodes());
    //cov_full = isamu_van2isam_eigen (cov_full);
    //std::cout << "[baseline]\tCovariance Recovery: " << (timestamp_now() - tic)/1e6 << " secs." << std::endl;
    
    //tic = timestamp_now();
    //MatrixXd cov_marg = cov_full (inds_to_keep_full, inds_to_keep_full);
    //std::cout << "[baseline]\tMarginal Covariance Found: " << (timestamp_now() - tic)/1e6 << " secs." << std::endl;
    
    int64_t tic = timestamp_now();
    const Covariances& covariances = slam.covariances();
    list<Node*> nodes_subset;
    int i=0;
    for (map<int64_t, Pose3d_Node*>::iterator it = pose3d_nodes.begin(); it!=pose3d_nodes.end(); it++, i++) {
        if (inds_to_keep.end() != find(inds_to_keep.begin(), inds_to_keep.end(), i))
            nodes_subset.push_back(it->second);
    }
    MatrixXd cov_marg = covariances.marginal(nodes_subset);
    cov_marg = isamu_van2isam_eigen (cov_marg);
    cout << "[baseline]\tCovariance Recovery: " << (timestamp_now() - tic)/1e6 << " secs." << std::endl;
    
    // write output to file
    tic = timestamp_now();
    
    char processed_dir[PATH_MAX];
    sprintf(processed_dir, "%s/processed", dir);
    boost::filesystem::create_directory(processed_dir);
    
    char inds_to_marg_file[PATH_MAX];
    sprintf (inds_to_marg_file, "%s/processed/inds_to_marg.txt", dir);
    write_inds (inds_to_marg_file, inds_to_marg);
    
    char inds_to_keep_file[PATH_MAX];
    sprintf (inds_to_keep_file, "%s/processed/inds_to_keep.txt", dir);
    write_inds (inds_to_keep_file, inds_to_keep);
    
    // char full_file[PATH_MAX];
    // sprintf (full_file, "%s/processed/cov_full.mat", dir);
    // write_matrix (full_file, cov_full);
    
    char marg_file[PATH_MAX];
    sprintf (marg_file, "%s/processed/cov_marg.mat", dir);
    write_matrix (marg_file, cov_marg);
    
    char stats_file[PATH_MAX];
    sprintf (stats_file, "%s/processed/stats.txt", dir);
    ofstream f;
    f.open (stats_file);
    slam.print_stats(f);
    f.close();
    
    char mp_file[PATH_MAX];
    sprintf (mp_file, "%s/processed/markov_pairs.txt", dir);
    write_markov_pairs (mp_file, &slam);
    
    char pc_file[PATH_MAX];
    sprintf (pc_file, "%s/processed/poses.pc", dir);
    write_pc (pc_file, &slam, &inds_to_keep);    
    
    std::cout << "[baseline]\tFiles Written: " << (timestamp_now() - tic)/1e6 << " secs." << std::endl;
    
}