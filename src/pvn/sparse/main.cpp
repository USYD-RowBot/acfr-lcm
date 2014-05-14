#include "sparse.h"

using namespace std;


Eigen::MatrixXd
load_matrix (char* file) {
    
    perllcm::matrix_d_t in;
    pvnu_lcm_fread<perllcm::matrix_d_t> (file, &in);

    Eigen::MatrixXd mat(in.nrows, in.ncols);
    memcpy(mat.data(), &(in.data[0]), in.n*sizeof(double));
    
    return mat;
}

void
write_matrix (char* file, Eigen::MatrixXd mat) {
    
    perllcm::matrix_d_t out;
    out.ncols = mat.cols();
    out.nrows = mat.rows();
    out.n = out.ncols * out.nrows;
    out.data = vector<double>::vector (mat.data(), mat.data()+out.n);

    pvnu_lcm_fwrite<perllcm::matrix_d_t> (file, out);
}

vector<int>
load_inds (char* file) {
    
    string line;
    vector<int> inds(0,0);
    
    ifstream f;
    f.open (file);
    
    if (!f.is_open())
        return inds;
    
    getline (f, line);
    while (f.good()) {
      inds.push_back(atoi (line.c_str()));
      getline (f, line);
    }
    f.close();
    
    return inds;
}

void
write_inds (char* file, vector<int> inds) {
    ofstream f;
    f.open (file);
    for (size_t i=0; i<inds.size(); i++) {
        f << inds[i] << endl;
    }
    f.close();
}

vector<Factor*>
load_meas_to_add (char* file, map<int64_t, Pose3d_Node*> &pose3d_nodes) {
    
    string line;
    vector<Factor*> factors;
    
    ifstream f;
    f.open (file);
    
    if (!f.is_open())
        return factors;
    
    getline (f, line);
    while (f.good()) {
        Pose3d z;
        MatrixXd sqrtinf(6,6);
        sqrtinf.setZero();
        int64_t idx_i = 0, idx_j = 0;

        char factor_line[1024];
        strcpy (factor_line, line.c_str());

        char *pch = strtok (factor_line," ,;(){}");

        // input
        size_t dof = 6; size_t dim_inf = 21;
        double mu[6];   double inf[21];

        size_t token_counter = 0;
        int idx_mu = 0, idx_inf = 0;

        while (pch) {
            if (token_counter == 0) 
                std::string factor_name = pch;
    
            if (token_counter == 1)
                idx_i = atol (pch);
    
            if (token_counter == 2)
                idx_j = atol (pch);
    
            if (3 <= token_counter && token_counter < 3+dof) {
                mu[idx_mu] = atof (pch);
                idx_mu ++;
            }
    
            if (3+dof <= token_counter && token_counter < 3+dof+dim_inf) {
                inf[idx_inf] = atof (pch);
                idx_inf ++;
            }
    
            pch = strtok (NULL," ,;(){}");
            token_counter++;
        }

        // set z
        z.set (mu[0],mu[1],mu[2],mu[3],mu[4],mu[5]);
    
        // set sqrt information matrix
        sqrtinf <<
        inf[0], inf[1],  inf[2],  inf[3],  inf[4],  inf[5],
        0.0,    inf[6],  inf[7],  inf[8],  inf[9],  inf[10],
        0.0,    0.0,     inf[11], inf[12], inf[13], inf[14],
        0.0,    0.0,     0.0,     inf[15], inf[16], inf[17],
        0.0,    0.0,     0.0,     0.0,     inf[18], inf[19],
        0.0,    0.0,     0.0,     0.0,     0.0,     inf[20];

        Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(pose3d_nodes[idx_i],
                                                           pose3d_nodes[idx_j],
                                                           z, SqrtInformation(sqrtinf));
        
        factors.push_back(factor);
          
        getline (f, line);
    }
    f.close();
    
    return factors;
}

void
write_meas_to_add (char * file, vector<Factor*> factors) {
    ofstream f;
    f.open(file);
    for (size_t i=0; i<factors.size(); i++) {
        f << *factors[i] << endl;
    }
    f.close();    
}    

void
write_pc (char *file, Slam *slam, vector<int> *inds_to_keep) {
    
    const std::list<Node*> nodes = slam->get_nodes();
    
    perllcm::pose3d_collection_t pc;
    pc.utime = 0;
    
    int i = 0;
    for (list<Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++, i++) {
            
        if (inds_to_keep != NULL &&
            inds_to_keep->end() == find(inds_to_keep->begin(), inds_to_keep->end(), i)) {
            continue;
        }
            
        Pose3d_Node* node = dynamic_cast<Pose3d_Node*>(*it);
        
        const Covariances& covariances = slam->covariances();
        std::list<Node*> tmp_list; tmp_list.push_back(node);
        Eigen::Matrix<double, 6,6, Eigen::RowMajor> P = covariances.marginal(tmp_list);
        P = isamu_van2isam_eigen (P);
    
        perllcm::pose3d_t p;
        p.utime = node->unique_id();
        p.mu[0] = node->value().x();
        p.mu[1] = node->value().y();
        p.mu[2] = node->value().z();
        p.mu[3] = node->value().roll();
        p.mu[4] = node->value().pitch();
        p.mu[5] = node->value().yaw();
        memcpy (p.Sigma, P.data(), 6*6*sizeof(double));
        
        pc.pose.push_back(p);
    }
    
    pc.npose = pc.pose.size();
    pvnu_lcm_fwrite<perllcm::pose3d_collection_t> (file, pc);
    
}

void
setup_isam (Slam *slam) {
    
    // set slam properties
    Properties prop;
    prop.max_iterations = SLAM_BATCH_MAX_ITTR;
    prop.mod_update = SLAM_MOD_UPDATE;
    prop.mod_batch = SLAM_MOD_BATCH;
    prop.mod_solve = SLAM_MOD_SOLVE;
    prop.method = SLAM_METHOD;
    slam->set_properties (prop);    
}

VectorXd
load_mu_from_pc (char *file) {
    
    VectorXd mu;
    
    perllcm::pose3d_collection_t pc;    
    int32_t ret = pvnu_lcm_fread <perllcm::pose3d_collection_t> (file, &pc);
    if (ret < 0) {
        cout << "[loading]\tCouldn't read " << file << " from disk. ret=" << ret << endl;
        return mu;
    }
    
    mu = VectorXd (pc.npose*6);
    for (int i=0; i<pc.npose; i++) {
        mu (i*6 + 0) = pc.pose[i].mu[0];
        mu (i*6 + 1) = pc.pose[i].mu[1];
        mu (i*6 + 2) = pc.pose[i].mu[2];
        mu (i*6 + 3) = pc.pose[i].mu[3];
        mu (i*6 + 4) = pc.pose[i].mu[4];
        mu (i*6 + 5) = pc.pose[i].mu[5];
    }
    
    return mu;
}

void
PVNSparse::load_isam_graph (Slam *slam, map<int64_t, Pose3d_Node*> *nodes) {
    
    // load the isam graph
    char graph_isam[PATH_MAX];
    sprintf (graph_isam, "%s/graph.isam", dir);
    cout << "[loading]\tLoading ISAM graph: " << graph_isam << endl;
    gParser *gp = new gParser (lcm.getUnderlyingLCM(), graph_isam);
    if (!gp)
        cout << "[loading]\tError: init_gp()" << std::endl;
    gp->set_slam (slam, nodes);
    gp->fread_graph ();        
    std::cout << "[loading]\tDone parsing: " << nodes->size() << " nodes." << std::endl;
    slam->batch_optimization();
}

double
logdet_eig (MatrixXd A) {
    
    SelfAdjointEigenSolver<MatrixXd> eigensolver(A);
    VectorXd D = eigensolver.eigenvalues();    
    
    double eps = numeric_limits<double>::epsilon();
    eps *= max(A.cols(), A.rows()) * D.array().abs().maxCoeff();
cout << "eps " << eps << endl;     
    double tmp = 0;
    for (int i=0; i<D.size(); i++) {
        if (D(i) > eps)
            tmp += log(D(i));
    }  
    
    return tmp;
}

double
logdet_chol (MatrixXd A) {
    
    MatrixXd L = A.llt().matrixL();
    
    double eps = numeric_limits<double>::epsilon();
    eps *= max(A.cols(), A.rows());

    double tmp = 0;
    for (int i=0; i<A.rows(); i++) {
            tmp += log (L(i,i));
    }
    
    return 2.0 * tmp;
}

double
kld_gaussian (VectorXd &u0, MatrixXd &S0, VectorXd &u1, MatrixXd &S1) {

    int n = u1.size();
    
    MatrixXd S1inv = S1.inverse();
    //MatrixXd S1inv = ldlt_inverse (S1);
    double A = (S1inv * S0).trace();
    VectorXd du = u1 - u0;
    // deal with difference in angles
    for(int i=0; i<du.size(); i++) {
        if(i % 6 == 3 || i % 6 == 4 || i % 6 == 5)
            du(i) = standardRad(du(i));
    }   
    double B = du.transpose() * S1inv * du;
    double Ca = logdet_chol(S0);
    double Cb = logdet_chol(S1);
//    double Ca_eig = logdet_eig(S0);
//    double Cb_eig = logdet_eig(S1);
    double C = Ca - Cb; 

cout << "A: " << A << endl;
cout << "B: " << B << endl;
cout << "Ca: " << Ca << endl;
cout << "Cb: " << Cb << endl;
//cout << "Ca_eig: " << Ca_eig << endl;
//cout << "Cb_eig: " << Cb_eig << endl;
cout << "C: " << C << endl;
cout << "n: " << n << endl;

    double kld = 0.5*(A + B - C - n);
    
    return kld;
}

void
write_markov_pairs (char *file, Slam *slam) {
    
    ofstream f;
    f.open (file);
    
    const list<Factor*>& factors = slam->get_factors();
    const list<Node*>& nodes = slam->get_nodes();
    for (list<Factor*>::const_iterator it = factors.begin(); it != factors.end(); it++) {
        
        const vector<Node*>& f_nodes = (*it)->nodes();
        for (size_t i=0; i<f_nodes.size(); i++) {
            for (size_t j=i+1; j<f_nodes.size(); j++) {
                // map nodes to indicies
                
                int ind_i = distance (nodes.begin(),
                                      find(nodes.begin(), nodes.end(), f_nodes[i]));
                int ind_j = distance (nodes.begin(),
                                      find(nodes.begin(), nodes.end(), f_nodes[j]));
                
                f << ind_i << " " << ind_j << endl;  
            }
        }
    }
    
    f.close();    
}


void
caculate_and_write_results (Slam *slam, char *dir, char *name_prefix, char *exp_name, double sec) {
    
    char name[PATH_MAX];
    if (strcmp (exp_name, (char *)"") == 0)
        sprintf (name, "%s", name_prefix);
    else
        sprintf (name, "%s_%s", name_prefix, exp_name);
        
    char save_dir[PATH_MAX];
    sprintf(save_dir, "%s/processed/%s", dir, name);
    boost::filesystem::create_directory(save_dir);
    
    // recover covariance
    int64_t tic = timestamp_now();
    const Covariances& covariances = slam->covariances();
    MatrixXd cov = covariances.marginal(slam->get_nodes());
    cov = isamu_van2isam_eigen (cov);
    std::cout << "[output]\tCovariance Recovery: " << (timestamp_now() - tic)/1e6 << " secs." << std::endl;
    
    tic = timestamp_now();
    
    char true_cov_file[PATH_MAX];
    sprintf (true_cov_file, "%s/processed/cov_marg.mat", dir);
    MatrixXd true_cov = load_matrix (true_cov_file);
    
    char true_mu_file[PATH_MAX];
    sprintf (true_mu_file, "%s/processed/poses.pc", dir);
    VectorXd true_mu = load_mu_from_pc (true_mu_file);
    
    const std::list<Node*> nodes = slam->get_nodes();
    VectorXd mu(nodes.size()*6); int i=0;
    for (list<Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++, i++) {
        Pose3d_Node* node = dynamic_cast<Pose3d_Node*>(*it);
        mu(i*6 + 0) = node->value().x();
        mu(i*6 + 1) = node->value().y();
        mu(i*6 + 2) = node->value().z();
        mu(i*6 + 3) = node->value().roll();
        mu(i*6 + 4) = node->value().pitch();
        mu(i*6 + 5) = node->value().yaw();
    }
    double kld = kld_gaussian (true_mu, true_cov, mu, cov);
    std::cout << "[output]\tKLD =  " << kld << ", calculated " << (timestamp_now() - tic)/1e6 << " secs." << std::endl;
    
    char stats_file[PATH_MAX];
    sprintf (stats_file, "%s/processed/%s/stats.txt", dir, name);
    ofstream f;
    f.open(stats_file);
    slam->print_stats(f);
    f << "KLD = " << kld << endl;
    f << "time (seconds) = " << sec << endl;
    f.close();
    
    // save results
    // write output to file
    tic = timestamp_now();

    char mat_file[PATH_MAX];
    sprintf (mat_file, "%s/processed/%s/cov.mat", dir, name);
    write_matrix (mat_file, cov);
    
    char graph_file[PATH_MAX];
    sprintf (graph_file, "%s/processed/%s/graph.isam", dir, name);
    slam->save (graph_file);
    
    char pc_file[PATH_MAX];
    sprintf (pc_file, "%s/processed/%s/poses.pc", dir, name);
    write_pc (pc_file, slam, NULL);
    
    char mp_file[PATH_MAX];
    sprintf (mp_file, "%s/processed/%s/markov_pairs.txt", dir, name);
    write_markov_pairs (mp_file, slam);

    std::cout << "[output]\tFiles Written: " << (timestamp_now() - tic)/1e6 << " secs." << std::endl;
}

void
check_valid_noise (MatrixXd A) {
    
    
    MatrixXd U = A.llt().matrixU();
    double chol_test = (A -  U.transpose()*U).array().abs().matrix().lpNorm<Infinity>();
    if (chol_test > 1e-4) {
        cout << "[ERROR]\tNoise Matrix not Pos. Def. chol_test = " << chol_test << endl;
        cout << A << endl;
        exit(0);
    }
    FullPivLU<MatrixXd> lu(A);    
    if (lu.rank() != A.rows()) {
        cout << "[ERROR]\tNoise Matrix not full rank. rank = " << lu.rank() << " < " << A.rows() << endl;
        cout << A << endl;
        exit(0);
    }
    
}


PVNSparse::PVNSparse (int argc, char *argv[])
{
    // open param cfg file
    param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        std::cout << "could not find " << BOTU_PARAM_DEFAULT_CFG << std::endl;
    }
    std::cout << "Opened Param File [" << BOTU_PARAM_DEFAULT_CFG << "]" << std::endl;
    
    channel_isam_request_state = bot_param_get_str_or_fail (param, "isamServer.lcm_channels.REQUEST_ST_CHANNEL");
    channel_isam_return_state = bot_param_get_str_or_fail (param, "isamServer.lcm_channels.RETURN_ST_CHANNEL");
    
    // read command line args
    gopt = getopt_create ();
    
    getopt_add_description (gopt, "Map Sparsification Development Program");
    getopt_add_string (gopt,  'd',  "dir",              ".",            "Path to data to process");
    getopt_add_string (gopt,  'n',  "name",             "",             "Name of experiment");
    getopt_add_spacer (gopt, "------------------------------------------------");
    getopt_add_bool   (gopt,  'm',  "baseline_marg",    0,              "Preform baseline marginalization");
    getopt_add_bool   (gopt,  'c',  "compound_marg",    0,              "Preform measurement compounding marginalization");
    getopt_add_bool   (gopt,  'l',  "lifting_marg",     0,              "Preform lifting marginalization");
    getopt_add_bool   (gopt,  'x',  "lifting_marg_glc", 0,              "Preform Generic Linear Constraints lifting marginalization");
    getopt_add_spacer (gopt, "------------------------------------------------");
    getopt_add_bool   (gopt,  's',  "clt_sparse",       0,              "Sparsify using Chow-Liu Tree");
    getopt_add_bool   (gopt,  'w',  "world_lin",        0,              "Linearize in world frame");
    getopt_add_bool   (gopt,  'g',  "gen_graph",        0,              "Generate a simple graph");
    getopt_add_bool   (gopt,  'r',  "rs_inc_x_o_w",     1,              "Include x_origin_world in root shift");
    getopt_add_bool   (gopt,  'a',  "add_meas_after",   0,              "Add additional measurements after marginalization");
    getopt_add_int    (gopt,  'e',  "marg_every",       MARG_EVERY_NTH, "Marganalize every nth node (0 don't use)");
    getopt_add_int    (gopt,  'k',  "keep_every",       "0",            "Keep every nth node (0 don't use, priority over marg_every)");
    getopt_add_spacer (gopt, "------------------------------------------------");
    getopt_add_bool   (gopt,  'h',  "help",    	        0,                 "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);
    }
    
    clt_sparse = getopt_get_bool (gopt, "clt_sparse");
    world_lin = getopt_get_bool (gopt, "world_lin");
    rs_inc_x_o_w = getopt_get_bool (gopt, "rs_inc_x_o_w");
    add_meas_after = getopt_get_bool (gopt, "add_meas_after");
    
    marg_every = getopt_get_int (gopt, "marg_every");
    keep_every = getopt_get_int (gopt, "keep_every");
    if (keep_every)
        marg_every = 0;
    
    dir = g_strdup (getopt_get_string (gopt, "dir"));
    name = g_strdup (getopt_get_string (gopt, "name"));
    
    
    // check lcm connection
    if(!lcm.good()) {
        is_done = 1;
        ERROR ("lcm_create () failed!");
    }
    
}

PVNSparse::~PVNSparse ()
{
    bot_param_destroy (param);
}

// ----------------------------------------------------------------------------
// Main Loop
// ----------------------------------------------------------------------------
void
PVNSparse::run ()
{
    
    if (!getopt_get_bool (gopt, "gen_graph") &&
        !getopt_get_bool (gopt, "baseline_marg") &&
        !getopt_get_bool (gopt, "compound_marg") &&
        !getopt_get_bool (gopt, "lifting_marg") &&
        !getopt_get_bool (gopt, "lifting_marg_glc")) {
        cout << "No job selected ... exiting." << endl;
        exit (EXIT_SUCCESS);
    }
    
    if (getopt_get_bool (gopt, "gen_graph")) {    
        run_gen_simple_graph ();
        return;
    }
    
    if (getopt_get_bool (gopt, "baseline_marg")) {    
        run_baseline_marginalization ();
        return;
    }
        
    if (getopt_get_bool (gopt, "compound_marg")) {    
        run_compound_marginalization ();
        return;
    }
    
    if (getopt_get_bool (gopt, "lifting_marg")) {
        run_lifting_marginalization ();
        return;
    }
    
    if (getopt_get_bool (gopt, "lifting_marg_glc")) {
        run_lifting_marginalization_glc ();
        return;
    }
    
}
    

// ----------------------------------------------------------------------------
// Ctrl+C catching
// ----------------------------------------------------------------------------
PVNSparse *g_sparse;

void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    std::cout << "Sigint caught. Quitting ..." << std::endl;
    if (g_sparse->is_done ) {
        std::cout << "Goodbye" << std::endl;
        exit (EXIT_FAILURE);
    } 
    else
        g_sparse->is_done = 1;
}


// ----------------------------------------------------------------------------
// Main 
// ----------------------------------------------------------------------------
int
main (int argc, char *argv[])
{    
    
    fasttrig_init ();
    
    PVNSparse sparse (argc, argv);
    g_sparse = &sparse;
    
    // install custom signal handler
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    // kick off the manager
    sparse.run ();
    
    return 0;     
}
