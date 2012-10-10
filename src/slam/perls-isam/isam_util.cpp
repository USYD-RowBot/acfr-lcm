#include <vector>

#include <Eigen/SVD>

#include "isam_util.h"
#include "user_factors.h"
#include "user_nodes.h"
#include "glc_factors.h"

#include "perls-lcmtypes/perllcm_pose3d_t.h"

#include "perls-math/ssc.h"
#include "perls-common/error.h"
#include "perls-common/magic.h"

using namespace std;
using namespace isam;
using namespace Eigen;

// conversion between rph to hpr
Pose3d
isamu_van2isam_pose3d (const double z[6])
{
    return Pose3d (z[0], z[1], z[2], z[5], z[4], z[3]);      
}

Pose3db
isamu_van2isam_pose3db (const double z[5])
{
    return Pose3db (z[0], z[1], z[4], z[3], z[2]);      
}


MatrixXd
isamu_van2isam_eigen (MatrixXd in) {

    Eigen::MatrixXd out(in.rows(), in.cols());

    int row, col;
    for (int i=0; i<in.rows(); i++) {
        for (int j=0; j<in.cols(); j++) {
            row = i, col = j;
            if (i % 6 == 3) row = i+2;
            if (i % 6 == 5) row = i-2;
            if (j % 6 == 3) col = j+2;
            if (j % 6 == 5) col = j-2;
            out(i,j) = in(row,col);
        }
    }
    
    return out;
}

MatrixXd
isamu_isam2van_eigen (MatrixXd in) {
    
    return isamu_van2isam_eigen (in);
    
}

MatrixXd
isamu_van2isam_sqrtinf3d (const double R[36])
{
    MatrixXd cov(6,6);
    int row, col;
    for (int i=0;i<6; i++) {
        for (int j=0;j<6; j++) {
            row = i; col = j;
            if (i==3) row = 5;
            if (i==5) row = 3;
            if (j==3) col = 5;
            if (j==5) col = 3;
            cov(i,j) = R[row*6+col];
        }
    }
    MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();

    return sqrtinf;
}

MatrixXd
isamu_van2isam_sqrtinf3db (const double R[25])
{
    MatrixXd cov(5,5);
    int row, col;
    for (int i=0;i<5; i++) {
        for (int j=0;j<5; j++) {
            row = i; col = j;
            if (i==2) row = 4;
            if (i==4) row = 2;
            if (j==2) col = 4;
            if (j==4) col = 2;
            cov(i,j) = R[row*5+col];
        }
    }
    MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();

    return sqrtinf;
}

MatrixXd
isamu_van2isam_plane3d_meas_sqrtinf (const double R[Plane3dMeasurement::dim*Plane3dMeasurement::dim])
{
    printf ("\n");
    MatrixXd cov(4, 4);
    for (int row=0; row<Plane3dMeasurement::dim; row++)
        for (int col=0; col<Plane3dMeasurement::dim; col++)
            cov (row, col) = R[row*Plane3dMeasurement::dim+col];
    MatrixXd sqrtinf = cov.inverse ().llt ().matrixL ().transpose ();
    return sqrtinf;
}

Plane3d
isamu_van2isam_plane3d (const double z[Plane3d::dim])
{
    return Plane3d (z[0], z[1], z[2], z[3], z[4], z[5]);
}

Plane3dMeasurement
isamu_van2isam_plane3d_meas (const double z[Plane3dMeasurement::dim])
{
    return Plane3dMeasurement (z[0], z[1], z[2], z[3]);
}

void
isamu_isam2van_pose3d (isam::Pose3d p, double z[6])
{
    z[0] = p.x();
    z[1] = p.y();
    z[2] = p.z();
    z[3] = p.roll();
    z[4] = p.pitch();
    z[5] = p.yaw();
}

void
isamu_isam2van_pose3db (isam::Pose3db p, double z[5])
{
    z[0] = p.a();
    z[1] = p.e();
    z[2] = p.roll();
    z[3] = p.pitch();
    z[4] = p.yaw();
}

void
isamu_isam2van_plane3d (isam::Plane3d p, double z[6])
{
    z[0] = p.nx();
    z[1] = p.ny();
    z[2] = p.nz();
    z[3] = p.px();
    z[4] = p.py();
    z[5] = p.pz();
}

void
isamu_isam2van_plane3d_meas (isam::Plane3dMeasurement p, double z[4])
{
    z[0] = p.nx();
    z[1] = p.ny();
    z[2] = p.nz();
    z[3] = p.d();
}

void
isamu_isam2van_sqrtinf3d  (Eigen::MatrixXd sqrtinf, double R[36])
{
    MatrixXd L = sqrtinf.transpose()*sqrtinf;
    MatrixXd cov = L.inverse();
    
    int row, col;
    for (int i=0;i<6; i++) {
        for (int j=0;j<6; j++) {
            row = i; col = j;
            if (i==3) row = 5;
            if (i==5) row = 3;
            if (j==3) col = 5;
            if (j==5) col = 3;
            R[row*6+col] = cov(i,j);
        }
    }
}

void
isamu_isam2van_sqrtinf3db (Eigen::MatrixXd sqrtinf, double R[25])
{
    MatrixXd L = sqrtinf.transpose()*sqrtinf;
    MatrixXd cov = L.inverse();
    
    int row, col;
    for (int i=0;i<5; i++) {
        for (int j=0;j<5; j++) {
            row = i; col = j;
            if (i==2) row = 4;
            if (i==4) row = 2;
            if (j==2) col = 4;
            if (j==4) col = 2;
            R[row*5+col] = cov(i,j);
        }
    }
}

void
isamu_isam2van_plane3d_meas_sqrtinf (Eigen::MatrixXd sqrtinf, double R[16])
{
    MatrixXd L = sqrtinf.transpose()*sqrtinf;
    MatrixXd cov = L.inverse();
    
    for (int row=0;row<Plane3dMeasurement::dim; row++) {
        for (int col=0;col<Plane3dMeasurement::dim; col++) {
            R[row*Plane3dMeasurement::dim+col] = cov(row,col);
        }
    }
}

isam::Properties
isamu_init_slam (const perllcm_isam_init_t *init_t)
{
    isam::Properties prop;

    if (init_t) {
        prop.verbose                    = init_t->verbose;
        prop.quiet                      = init_t->quiet;
        prop.force_numerical_jacobian   = init_t->force_numerical_jacobian;
        //prop.method = init_t->method;
        prop.verbose                    = init_t->verbose;
        prop.epsilon1                   = init_t->epsilon1;
        prop.epsilon2                   = init_t->epsilon2;
        prop.epsilon3                   = init_t->epsilon3;
        prop.epsilon_abs                = init_t->epsilon_abs;
        prop.epsilon_rel                = init_t->epsilon_rel;
        prop.max_iterations             = init_t->max_iterations;
        prop.lm_lambda0                 = init_t->lm_lambda0;
        prop.lm_lambda_factor           = init_t->lm_lambda_factor;
        prop.continuable                = init_t->continuable;
        prop.last_lambda                = init_t->last_lambda;
        prop.mod_update                 = init_t->mod_update;
        prop.mod_batch                  = init_t->mod_batch;
        prop.mod_solve                  = init_t->mod_solve;
    }

    return prop;
}

isam::Properties
isamu_init_slam (BotParam *param)
{
    isam::Properties prop;

    // tmp
    double tmp_double;
    int    tmp_int;

    if (param) {
        if (0==bot_param_get_boolean (param, "isamServer.isam_init.force_numerical_jacobian", &tmp_int))
            prop.force_numerical_jacobian = tmp_int;
        if (0==bot_param_get_double (param, "isamServer.isam_init.epsilon1", &tmp_double))
            prop.epsilon1 = tmp_double;
        if (0==bot_param_get_double (param, "isamServer.isam_init.epsilon2", &tmp_double))
            prop.epsilon2 = tmp_double;
        if (0==bot_param_get_double (param, "isamServer.isam_init.epsilon3", &tmp_double))
            prop.epsilon3 = tmp_double;
        if (0==bot_param_get_double (param, "isamServer.isam_init.epsilon_abs", &tmp_double))
            prop.epsilon_abs = tmp_double;
        if (0==bot_param_get_double (param, "isamServer.isam_init.epsilon_rel", &tmp_double))
            prop.epsilon_rel = tmp_double;
        if (0==bot_param_get_double (param, "isamServer.isam_init.lm_lambda0", &tmp_double))
            prop.lm_lambda0 = tmp_double;
        if (0==bot_param_get_double (param, "isamServer.isam_init.lm_lambda_factor", &tmp_double))
            prop.lm_lambda_factor = tmp_double;
        if (0==bot_param_get_boolean (param, "isamServer.isam_init.mod_update", &tmp_int))
            prop.mod_update = tmp_int;
        if (0==bot_param_get_boolean (param, "isamServer.isam_init.mod_solve", &tmp_int))
            prop.mod_solve = tmp_int;
        if (0==bot_param_get_int (param, "isamServer.isam_init.max_iterations", &tmp_int))
            prop.max_iterations = tmp_int;
        if (0==bot_param_get_int (param, "isamServer.isam_init.mod_batch", &tmp_int))
            prop.mod_batch = tmp_int;
        if (0==bot_param_get_boolean (param, "isamServer.isam_init.verbose", &tmp_int))
            prop.verbose = tmp_int;
        if (0==bot_param_get_boolean (param, "isamServer.isam_init.quiet", &tmp_int))
            prop.quiet = tmp_int;
    }

    return prop;
}

Pose3d_Node*
isamu_add_node_pose3d (const perllcm_isam_add_node_t *msg, isam::Slam *m_slam)
{
    Pose3d_Node* new_node = new Pose3d_Node();    

    // initialize if possible
    if (msg->has_mu_o) {
        Pose3d pose0 (msg->mu_o[0], msg->mu_o[1], msg->mu_o[2], msg->mu_o[5], msg->mu_o[4], msg->mu_o[3]);
        new_node->init (pose0);
    }

    m_slam->add_node(new_node);

    return new_node;
}

Plane3d_Node*
isamu_add_node_plane3d (const perllcm_isam_add_node_t *msg, isam::Slam *m_slam)
{
    Plane3d_Node* new_node = new Plane3d_Node();    

    // initialize if possible
    if (msg->has_mu_o) {
        Plane3d mu (msg->mu_o[0], msg->mu_o[1], msg->mu_o[2], msg->mu_o[3], msg->mu_o[4], msg->mu_o[5]);
        new_node->init (mu);
    }

    m_slam->add_node(new_node);

    return new_node;
}

void
isamu_add_glc_factor (const perllcm_isam_glc_factor_t *glc_factor, isam::Slam *m_slam, vector<Pose3d_Node*> p3d_nodes){
    
    int rows  = glc_factor->m / glc_factor->n; 
    bool rs_inc_x_o_w = false;
    if (glc_factor->n == glc_factor->np * (6 - 1))
        rs_inc_x_o_w = false;
    else if (glc_factor->n == glc_factor->np * 6)
        rs_inc_x_o_w = true;
    else
        cout << "ERROR: can't understand U size in glc factor" << endl;
    
    Eigen::Map< Matrix<double, Dynamic, Dynamic, RowMajor> > U(glc_factor->U, rows, glc_factor->n);
    Eigen::Map< VectorXd > x(glc_factor->x, glc_factor->n);
    
    isam::Factor *f_add;
    if (glc_factor->is_root_shifted)
        f_add = new GLC_RS_Factor(p3d_nodes, x, U, rs_inc_x_o_w);
    else
        f_add = new GLC_Factor(p3d_nodes, x, U);
        
    m_slam->add_factor(f_add); 
    
}

void
isamu_add_factor (const perllcm_isam_vlink_t *vlink, isam::Slam *m_slam, Node* node1, Node* node2)
{

    if (!vlink->accept)
        return;

    if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_POSE2D) {
        // implement me!
    }
    else if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_POSE3D && vlink->n == 6) {
        Pose3d delta     = isamu_van2isam_pose3d (vlink->z);
        MatrixXd sqrtinf = isamu_van2isam_sqrtinf3d (vlink->R);
        Pose3d_Pose3d_Factor* constraint = new Pose3d_Pose3d_Factor (dynamic_cast<Pose3d_Node *>(node1), dynamic_cast<Pose3d_Node *>(node2), delta, SqrtInformation(sqrtinf));
        m_slam->add_factor(constraint);
    }
    else if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_POSE2DB) {
        // implement me!
    }
    else if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_POSE3DB) {
        Pose3db delta     = isamu_van2isam_pose3db (vlink->z);
        MatrixXd sqrtinf = isamu_van2isam_sqrtinf3db (vlink->R);
        if (vlink->dynamic_xvs) {
            Pose3d senxform1 = isamu_van2isam_pose3d (vlink->x_vs1);
            Pose3d senxform2 = isamu_van2isam_pose3d (vlink->x_vs2);
            Pose3db_Pose3db_Factor* constraint = new Pose3db_Pose3db_Factor (dynamic_cast<Pose3d_Node *>(node1), dynamic_cast<Pose3d_Node *>(node2), delta, SqrtInformation(sqrtinf), &senxform1, &senxform2);
            m_slam->add_factor(constraint);
        }
        else {
            std::cout << "WARNING: this camera link does not have x_vs. Adding the vehicle to vehicle bearing instead." << std::endl;
            Pose3db_Pose3db_Factor* constraint = new Pose3db_Pose3db_Factor (dynamic_cast<Pose3d_Node *>(node1), dynamic_cast<Pose3d_Node *>(node2), delta, SqrtInformation(sqrtinf));
            m_slam->add_factor(constraint);
        }
    }
    else if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_SONAR2D) {
        // a imaging sonar link
        double* m = vlink->z;
        Pose2d delta (m[0], m[1], m[2]);
        Matrix<double, 3,3> cov;
        double* s = vlink->R;
        for (int i=0;i<3; i++)
            for (int j=0;j<3; j++) cov(i,j) = s[i*3+j];

        if (vlink->dynamic_xvs) {
            Pose3d sonar_frame1 = isamu_van2isam_pose3d (vlink->x_vs1);
            Pose3d sonar_frame2 = isamu_van2isam_pose3d (vlink->x_vs2);
            Sonar2d_Factor* constraint = new Sonar2d_Factor (dynamic_cast<Pose3d_Node *>(node1), dynamic_cast<Pose3d_Node *>(node2), delta, Covariance(cov), sonar_frame1, sonar_frame2);
            m_slam->add_factor(constraint);
        }
        else {
            std::cout << "WARNING: this sonar2D link does not have x_vs. No factor added." << std::endl;
        }
    }
    else if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_PLANE3D) {
        Plane3dMeasurement meas = isamu_van2isam_plane3d_meas (vlink->z);
        MatrixXd sqrtinf = isamu_van2isam_plane3d_meas_sqrtinf (vlink->R);
        Plane3d_Node *plane = dynamic_cast<Plane3d_Node *>(node2);
        Pose3d_Plane3d_Factor* constraint = new Pose3d_Plane3d_Factor (dynamic_cast<Pose3d_Node *>(node1), plane, meas, SqrtInformation (sqrtinf));
        m_slam->add_factor (constraint);
    }
    else {
        std::cout << "ERROR in parsing vlink: Unknown link type" << std::endl;
    }    
}

void
isamu_add_partial (const perllcm_isam_vlink_t *vlink, isam::Slam *slam,
                   isam::Pose3d_Node* node, bool print_status) {
                    
    int64_t id = vlink->id2;    
    
    switch (vlink->link_type) {
        case PERLLCM_ISAM_VLINK_T_LINK_ZPR_PARTIAL: {  
            // depth, pitch, roll order
            VectorXd delta(3); delta << vlink->z[0], vlink->z[1], vlink->z[2];
        
            MatrixXd cov(3,3);
            for (int i=0;i<3; i++)
                for (int j=0;j<3; j++)
                    cov(i,j) = vlink->R[i*3+j];
            MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
        
            Pose3dPartial_Factor* constraint = new Pose3dPartial_Factor(node, delta, SqrtInformation(sqrtinf));
            slam->add_factor(constraint);
        
            //std::cout << "[server]\tzrp factor for " << id << " added." << std::endl;
            break;
        }
        case PERLLCM_ISAM_VLINK_T_LINK_XYZ_PARTIAL: {
            // x y z
            VectorXd delta(3); delta << vlink->z[0], vlink->z[1], vlink->z[2];
        
            MatrixXd cov(3,3);
            for (int i=0;i<3; i++)
                for (int j=0;j<3; j++)
                    cov(i,j) = vlink->R[i*3+j];
            MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
        
            Pose3d_xyz_Factor* constraint = new Pose3d_xyz_Factor(node, delta, SqrtInformation(sqrtinf));
            slam->add_factor(constraint);
        
            if (print_status) std::cout << "[server]\txyz factor for " << id << " added." << std::endl;
            break;
        }
        case PERLLCM_ISAM_VLINK_T_LINK_H_PARTIAL: {
            
            // h
            VectorXd delta(1); delta << vlink->z[0];
        
            MatrixXd cov(1,1);
            cov(0,0) = vlink->R[0];
            MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
        
            Pose3d_h_Factor* constraint = new Pose3d_h_Factor(node, delta, SqrtInformation(sqrtinf));
            slam->add_factor(constraint);
            
            if (print_status) std::cout << "[server]\th factor for " << id << " added." << std::endl;
            break;
        }
        case PERLLCM_ISAM_VLINK_T_LINK_RP_PARTIAL: {
            
            // rp
            VectorXd delta(2); delta << vlink->z[0], vlink->z[1];
        
            MatrixXd cov(2,2);
            for (int i=0;i<2; i++)
                for (int j=0;j<2; j++)
                    cov(i,j) = vlink->R[i*2+j];
            MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
        
            Pose3d_rp_Factor* constraint = new Pose3d_rp_Factor(node, delta, SqrtInformation(sqrtinf));
            slam->add_factor(constraint);
            
            if (print_status) std::cout << "[server]\trp factor for " << id << " added." << std::endl;
            break;
        }
        case PERLLCM_ISAM_VLINK_T_LINK_Z_PARTIAL: {
            
            // h
            VectorXd delta(1); delta << vlink->z[0];
        
            MatrixXd cov(1,1);
            cov(0,0) = vlink->R[0];
            MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
        
            Pose3d_z_Factor* constraint = new Pose3d_z_Factor(node, delta, SqrtInformation(sqrtinf));
            slam->add_factor(constraint);
            
            if (print_status) std::cout << "[server]\tz factor for " << id << " added." << std::endl;
            break;
        }
        case PERLLCM_ISAM_VLINK_T_LINK_XY_PARTIAL: {
            
            // rp
            VectorXd delta(2); delta << vlink->z[0], vlink->z[1];
        
            MatrixXd cov(2,2);
            for (int i=0;i<2; i++)
                for (int j=0;j<2; j++)
                    cov(i,j) = vlink->R[i*2+j];
            MatrixXd sqrtinf = cov.inverse().llt().matrixL().transpose();
        
            Pose3d_xy_Factor* constraint = new Pose3d_xy_Factor(node, delta, SqrtInformation(sqrtinf));
            slam->add_factor(constraint);
            
            if (print_status) std::cout << "[server]\txy factor for " << id << " added." << std::endl;
            break;
        }
        default:
            ERROR ("Unrecognized partial type.");
    }
}


void
isamu_state_fprintf (const char *filename, const perllcm_pose3d_collection_t *pc, GList *utimelist)
{
    FILE *stream = fopen (filename, "w");

    size_t n = pc->npose;

    for (size_t i=0; i<n; i++) {
        perllcm_pose3d_t *p = &pc->pose[i];

        // append utime if available. just idx o.w.
        if (utimelist) {
            GList *event = g_list_nth (utimelist, i);
            int64_t *utime = (int64_t*) event->data;
            fprintf (stream, "%ld ", *utime);
        }
        else
            fprintf (stream, "%zu ", i);
        
        // print mu
        for (size_t j=0; j<6; j++)
            fprintf (stream, "%g ", p->mu[j]);

        // print sigma
        for (size_t j=0; j<36; j++)
            fprintf (stream, "%g ", p->Sigma[j]);

        fprintf (stream, "\n");
    }

    fclose (stream);
}

void
isamu_cputime_fprintf (const char *filename, std::map<int64_t, int64_t> cputime)
{
    FILE *stream = fopen (filename, "w");

    std::map<int64_t, int64_t>::iterator it;
    for (it = cputime.begin(); it != cputime.end(); it++) {
        int64_t node_utime = it->first;
        int64_t dt = it->second; // the value in the key value pair

        fprintf (stream, "%ld %ld\n", node_utime, dt);
    }

    fclose (stream);
}


Eigen::MatrixXd
isamu_get_weighted_jacobian (isam::Factor *f) {

    Jacobian jac = f->jacobian();
    // get jacobian size
    std::vector<Node*>& f_nodes = f->nodes();
    int n_measure = f->dim();
    int n_var = 0;
    for (size_t i=0; i<f_nodes.size(); i++) {
        n_var += f_nodes[i]->dim();
    }
        
        
    // fill jacobian matrix
    MatrixXd H (n_measure, n_var);
    int offset = 0;
    for (Terms::const_iterator it=jac.terms().begin(); it!=jac.terms().end(); it++) {
        int ncols = it->term().cols();
        H.block(0, offset, n_measure, ncols) = it->term();
        offset += ncols;
    }
    
    return H;

}

Eigen::MatrixXd
isamu_get_jacobian (isam::Factor *f) {

    MatrixXd H = isamu_get_weighted_jacobian (f);
    
    // jacobian in ISAM is weighted by square root information
    // H = f->sqrtinf().inverse() * H;
    // try pinverse insted
    MatrixXd sqrtinf = f->sqrtinf();
    MatrixXd sqrtinf_inv = (sqrtinf.transpose() * sqrtinf).inverse() * sqrtinf.transpose();
    H = sqrtinf_inv * H;

double test = (sqrtinf_inv - f->sqrtinf().inverse()).array().abs().matrix().lpNorm<Infinity>();
if (test > 1e-6)
    cout << "Square root information not invertible: " << test << endl;    
    
    return H;
}


MatrixXd
isamu_conditional_information_recovery (Slam *slam, vector<int> inds) {
    const SparseSystem& R = slam->get_R();
    const int *a_to_r = R.a_to_r(); // column ordering
    MatrixXd Rsub(R.num_rows(), inds.size());
    // get colums of R for each of the inds
    for (int i=0; i<R.num_rows(); i++) {
        for (size_t j=0; j<inds.size(); j++) {
            Rsub(i,j) = R(a_to_r[i], a_to_r[inds[j]]);
        }
    }

    return Rsub.transpose()*Rsub;
}

void
isamu_root_shift (VectorXd &x, MatrixXd *J, vector<Node*> nodes, Selector s, bool inc_x_o_w) {
    
    int np = nodes.size();
   
    // allows root shifted glc to support world frame priors
    // other wise some graphs would need root shifted and regular GLC
    if (np == 1) {
        x = nodes[0]->vector(s);
        if (J != NULL)
            (*J) = Matrix<double, 6, 6>::Identity();
        return;
    }    
    
    int n = np * 6;
    int m;
    if (inc_x_o_w)
        m = np*6;
    else
        m = (np-1)*6;
    
    x = VectorXd(m);
    if (J != NULL) {
        (*J) = MatrixXd(m,n);
        J->setZero();
    }
    
    Pose3d_Node *node_i = dynamic_cast<Pose3d_Node *>(nodes[0]);
    double X_w_i[6] = {0};
    isamu_isam2van_pose3d (node_i->value(s), X_w_i);

    for (int j=1; j<np; j++) {
        
        Pose3d_Node *node_j = dynamic_cast<Pose3d_Node *>(nodes[j]);
        double X_w_j[6] = {0};
        isamu_isam2van_pose3d (node_j->value(s), X_w_j);
        
        double X_i_j[6] = {0};
        if (J != NULL) {
            Matrix<double, Dynamic, Dynamic, RowMajor> Jij (6,12);
            ssc_tail2tail (X_i_j, Jij.data(), X_w_i, X_w_j);
                
            J->block<6,6>((j-1)*6, 0)   = Jij.block<6,6>(0,0);
            J->block<6,6>((j-1)*6, j*6) = Jij.block<6,6>(0,6);
        } else {
            ssc_tail2tail (X_i_j, NULL, X_w_i, X_w_j);
        }
        
        Pose3d x_i_j = isamu_van2isam_pose3d (X_i_j);
        x.segment<6>((j-1)*6) = x_i_j.vector();
    }
    
    if (inc_x_o_w) {
        // include inverse of root node at end
        double X_i_w[6] = {0};
        if (J != NULL) {
            Matrix<double, Dynamic, Dynamic, RowMajor> Jiw (6,6);
            ssc_inverse (X_i_w, Jiw.data(), X_w_i);
            
            J->block<6,6>((np-1)*6, 0) = Jiw;
        } else {
            ssc_inverse (X_i_w, NULL, X_w_i);
        }
        Pose3d x_i_w = isamu_van2isam_pose3d (X_i_w);
        x.segment<6>((np-1)*6) = x_i_w.vector();
    }   
    
//cout << "J" << endl << *J << endl << endl;
//cout << "x" << endl << x << endl << endl;
    
    if (J != NULL) {
        (*J) = isamu_isam2van_eigen (*J);
    }
    
    
}
