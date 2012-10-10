#include <glib.h>

#include "perls-lcmtypes/perllcm_isam_vlink_t.h"

#include "isam_util.h"
#include "gparser.h"

#include "perls-math/gsl_util.h"

using namespace Eigen;

bool 
gParser::fread_graph (void) 
{
    string line;
    if (m_inFile.is_open ()) {
        while (m_inFile.good ()) {
            getline (m_inFile, line);
            if (line.length() > 0)
                _handleLine(line);
        }
        m_inFile.close ();
        m_done = true;

        std::cout << "[parser]\tFile loading done." << std::endl;

        return m_done;
    }
    else
        throw runtime_error ("Unable to open file");
}

// parser a line of string
void 
gParser::_handleLine (const string line) 
{
    //std::cout << line << std::endl;

    perllcm_isam_vlink_t *vlink = NULL;
    perllcm_isam_add_node_t *addnode = NULL;
    perllcm_isam_glc_factor_t *glc_factor = NULL;

    if (_is_commented_line (line))
        return;
    else if (_is_pose3d_prior_line (line))
        vlink = _handle_prior (line);
    else if (_is_node_line (line))
        addnode = _handle_node (line);
    else if (_is_odo_factor_line (line))
        vlink = _handle_odo_factor (line);
    else if (_is_cam_factor_line (line))
        vlink = _handle_cam_factor (line);
    else if (_is_sonar_factor_line (line))
        vlink = _handle_sonar_factor (line);
    else if (_is_z_factor_line (line))
        vlink = _handle_1_tuple_factor (line, PERLLCM_ISAM_VLINK_T_LINK_Z_PARTIAL);
    else if (_is_h_factor_line (line))
        vlink = _handle_1_tuple_factor (line, PERLLCM_ISAM_VLINK_T_LINK_H_PARTIAL);
    else if (_is_rp_factor_line (line))
        vlink = _handle_2_tuple_factor (line, PERLLCM_ISAM_VLINK_T_LINK_RP_PARTIAL);
    else if (_is_xy_factor_line (line))
        vlink = _handle_2_tuple_factor (line, PERLLCM_ISAM_VLINK_T_LINK_XY_PARTIAL);
    else if (_is_xyz_factor_line (line))
        vlink = _handle_3_tuple_factor (line, PERLLCM_ISAM_VLINK_T_LINK_XYZ_PARTIAL);
    else if (_is_zpr_factor_line (line))
        vlink = _handle_3_tuple_factor (line, PERLLCM_ISAM_VLINK_T_LINK_ZPR_PARTIAL);
    else if (_is_glc_factor_line (line)) 
        glc_factor = _handle_glc_factor (line, false);
    else if (_is_glc_rs_factor_line (line)) 
        glc_factor = _handle_glc_factor (line, true);
    else if (_is_plane_prior_line (line))
        vlink = _handle_plane_prior (line);
    else if (_is_plane_factor_line (line))
        vlink = _handle_plane_factor (line);
    else if (_is_plane_node_line (line))
        addnode = _handle_plane_node (line);
    else {
        std::cout << line << std::endl;
        throw runtime_error ("[parser]\tparser for this type has not been implemented.");
    }


    // put them in server's queue
    if (m_gq != NULL) {
        if (_is_node_line (line) || _is_plane_node_line (line)) { // is it node?
            queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (*qe));
            qe->msg = addnode;
            qe->utime = addnode->id;
            qe->type = QE_NODE;
            g_async_queue_push (m_gq, qe);
        }
        else if (_is_glc_factor_line (line) || _is_glc_rs_factor_line (line)) {
            queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (*qe));
            qe->msg = glc_factor;
            int64_t max_id = 0;
            for (int i=0; i<glc_factor->np; i ++)
                if (glc_factor->ids[i] > max_id) max_id = glc_factor->ids[i];
            qe->utime = max_id;
            qe->type = QE_GLC;
            g_async_queue_push (m_gq, qe);
        }
        else { // or factors?
            queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (*qe));
            qe->msg = vlink;
            qe->utime = vlink->id2;
            qe->type = QE_VLINK;
            g_async_queue_push (m_gq, qe);
        }
    }

}

// nodes
// ----------------------------------------------------------------
perllcm_isam_add_node_t *
gParser::_handle_node (const string line)
{
    int64_t idx = 0; int64_t utime = 0;
    isam::Pose3d node_mu; double offset = 0.0;
    bool has_mu;
    _parse_node (line, idx, node_mu, utime, offset, has_mu);

    perllcm_isam_add_node_t addnode = {0};
    addnode.utime = utime;
    addnode.id = idx;
    addnode.node_type = PERLLCM_ISAM_ADD_NODE_T_NODE_POSE3D;
    addnode.mu_o[0] = node_mu.x();
    addnode.mu_o[1] = node_mu.y();
    addnode.mu_o[2] = node_mu.z();
    addnode.mu_o[3] = node_mu.roll();
    addnode.mu_o[4] = node_mu.pitch();
    addnode.mu_o[5] = node_mu.yaw();
    addnode.has_mu_o = has_mu;
    addnode.sensor_id = PERLLCM_ISAM_ADD_NODE_T_NODE_NOSENSOR;

    if (offset > 0.0)
        addnode.sensor_id = PERLLCM_ISAM_ADD_NODE_T_NODE_CAMERA;

    perllcm_isam_add_node_t *addnode_alloc = perllcm_isam_add_node_t_copy (&addnode);

    return addnode_alloc;
}

perllcm_isam_add_node_t *
gParser::_handle_plane_node (const string line)
{
    int64_t idx = 0; int64_t utime = 0;
    isam::Pose3d node_mu; double offset = 0.0;
    bool has_mu;
    _parse_node (line, idx, node_mu, utime, offset, has_mu);

    perllcm_isam_add_node_t addnode = {0};
    addnode.utime = utime;
    addnode.id = idx;
    addnode.node_type = PERLLCM_ISAM_ADD_NODE_T_NODE_PLANE3D;

    /* HACK: treat node_mu (a Pose3d) as a Plane3d, since they both are 6-dimensional */
    addnode.mu_o[0] = node_mu.x();
    addnode.mu_o[1] = node_mu.y();
    addnode.mu_o[2] = node_mu.z();
    addnode.mu_o[3] = node_mu.roll();
    addnode.mu_o[4] = node_mu.pitch();
    addnode.mu_o[5] = node_mu.yaw();
    addnode.has_mu_o = has_mu;
    
    addnode.sensor_id = PERLLCM_ISAM_ADD_NODE_T_NODE_NOSENSOR;

    if (offset > 0.0)
        addnode.sensor_id = PERLLCM_ISAM_ADD_NODE_T_NODE_CAMERA;

    perllcm_isam_add_node_t *addnode_alloc = perllcm_isam_add_node_t_copy (&addnode);

    return addnode_alloc;
}

// factors
// ----------------------------------------------------------------
perllcm_isam_vlink_t *
gParser::_handle_prior (const string line)
{
    Pose3d Pose0; MatrixXd sqrtinf0 (6,6); 
    int64_t idx = 0; int64_t utime = 0;
    _parse_prior (line, idx, Pose0, sqrtinf0, utime);

    // publish parsed factor
    double z[6], R[36];
    isamu_isam2van_pose3d (Pose0, z);
    isamu_isam2van_sqrtinf3d (sqrtinf0, R);

    perllcm_isam_vlink_t vlink = {0};
    vlink.utime = utime;
    vlink.id1 = 0;
    vlink.id2 = idx; //(utime == 0)?idx:utime;;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = 0;
    vlink.n = 6;
    vlink.z = z;
    vlink.n2 = 6*6;
    vlink.R = R;
    vlink.link_type = PERLLCM_ISAM_VLINK_T_LINK_PRIOR;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_vlink_t *
gParser::_handle_odo_factor (string line)
{
    Pose3d delta; Eigen::MatrixXd sqrtinf (6,6); 
    int64_t idx1, idx2 = 0;
    int64_t utime1 = 0, utime2 = 0;

    _parse_odo_factor (line, idx1, idx2, delta, sqrtinf, utime1, utime2);

    if (idx1 == 0 && idx2 == 0) 
        throw runtime_error("ERROR: parsing odometry factor");
    
    // publish parsed factor
    double z[6], R[36];
    isamu_isam2van_pose3d (delta, z);
    isamu_isam2van_sqrtinf3d (sqrtinf, R);

    /* cout << "Pose3d sqrtinf" << endl; */
    /* cout << sqrtinf << endl; */
    /* cout << "Plane3d R" << endl; */
    /* for (int i=0; i<36; i++) */
    /*     printf ("%f ", R[i]); */
    /* printf("\n"); */

    perllcm_isam_vlink_t vlink = {0};
    vlink.utime = utime2;
    vlink.id1 = idx1; //(utime1 == 0)?idx1:utime1;
    vlink.id2 = idx2; //(utime2 == 0)?idx2:utime2;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = PERLLCM_ISAM_VLINK_T_SENSOR_ODOMETRY;
    vlink.n = 6;
    vlink.z = z;
    vlink.n2 = 6*6;
    vlink.R = R;
    vlink.link_type = PERLLCM_ISAM_VLINK_T_LINK_POSE3D;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_vlink_t *
gParser::_handle_cam_factor (string line)
{
    Pose3db delta; Eigen::MatrixXd sqrtinf (5,5); 
    int64_t idx1, idx2 = 0;
    int64_t utime1 = 0, utime2 = 0;
    Pose3d x_v1c, x_v2c;

    _parse_cam_factor (line, idx1, idx2, delta, sqrtinf, x_v1c, x_v2c, utime1, utime2);

    if (idx1 == 0 && idx2 == 0) 
        throw runtime_error("ERROR: parsing camera factor");

    // publish parsed factor
    double z[5], R[25];
    isamu_isam2van_pose3db (delta, z);
    isamu_isam2van_sqrtinf3db (sqrtinf, R);

    perllcm_isam_vlink_t vlink = {0};
    vlink.utime = utime2;
    vlink.id1 = idx1; //(utime1 == 0)?idx1:utime1;
    vlink.id2 = idx2; //(utime2 == 0)?idx2:utime2;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = PERLLCM_ISAM_VLINK_T_SENSOR_CAMERA;
    vlink.n = 5;
    vlink.z = z;
    vlink.n2 = 5*5;
    vlink.R = R;
    vlink.link_type = PERLLCM_ISAM_VLINK_T_LINK_POSE3DB;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    isamu_isam2van_pose3d (x_v1c, vlink.x_vs1);
    isamu_isam2van_pose3d (x_v2c, vlink.x_vs2);
    if (x_v1c.x() + x_v1c.y() + x_v1c.z() + + x_v1c.roll() + + x_v1c.pitch() + + x_v1c.yaw() > 0) {
        vlink.dynamic_xvs = 1;
    }

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_vlink_t *
gParser::_handle_sonar_factor (string line)
{
    Pose2d delta; Eigen::MatrixXd sqrtinf (3,3); 
    int64_t idx1, idx2 = 0;
    int64_t utime1 = 0, utime2 = 0;
    Pose3d x_v1s, x_v2s;

    _parse_sonar_factor (line, idx1, idx2, delta, sqrtinf, x_v1s, x_v2s, utime1, utime2);

    if (idx1 == 0 && idx2 == 0) 
        throw runtime_error("ERROR: parsing sonar factor");

    // publish parsed factor:
    double z[3], R[9];
    z[0] = delta.x();
    z[1] = delta.y();
    z[2] = delta.t();

    MatrixXd L = sqrtinf * sqrtinf.transpose();
    MatrixXd cov = L.inverse();
    
    for (int i=0;i<3; i++)
        for (int j=0;j<3; j++)
            R[i*3+j] = cov(i,j);

    perllcm_isam_vlink_t vlink ={0};
    vlink.utime = utime2;
    vlink.id1 = idx1; //(utime1 == 0)?idx1:utime1;
    vlink.id2 = idx2; //(utime2 == 0)?idx2:utime2;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = PERLLCM_ISAM_VLINK_T_SENSOR_SONAR;
    vlink.n = 3;
    vlink.z = z;
    vlink.n2 = 3*3;
    vlink.R = R;
    vlink.link_type = PERLLCM_ISAM_VLINK_T_LINK_SONAR2D;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    isamu_isam2van_pose3d (x_v1s, vlink.x_vs1);
    isamu_isam2van_pose3d (x_v2s, vlink.x_vs2);
    if (x_v1s.x() + x_v1s.y() + x_v1s.z() + + x_v1s.roll() + + x_v1s.pitch() + + x_v1s.yaw() > 0) {
        vlink.dynamic_xvs = 1;
    }

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_vlink_t *
gParser::_handle_plane_prior (string line)
{
    Plane3d prior; Eigen::MatrixXd sqrtinf (6, 6);
    int64_t idx = 0; int64_t utime = 0;
    _parse_plane_prior (line, idx, prior, sqrtinf, utime);

    double z[6], R[36];
    isamu_isam2van_plane3d (prior, z);
    isamu_isam2van_sqrtinf3d (sqrtinf, R);

    perllcm_isam_vlink_t vlink = {0};
    vlink.utime = utime;
    vlink.id1 = 0;
    vlink.id2 = idx; //(utime == 0)?idx:utime;;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = 0;
    vlink.n = 6;
    vlink.z = z;
    vlink.n2 = 6*6;
    vlink.R = R;
    vlink.link_type = PERLLCM_ISAM_VLINK_T_LINK_PLANE3D_PRIOR;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_vlink_t *
gParser::_handle_plane_factor (string line)
{
    Plane3dMeasurement meas; Eigen::MatrixXd sqrtinf (4, 4);
    int64_t idx1, idx2 = 0;
    int64_t utime1 = 0, utime2 = 0;
    double z[Plane3dMeasurement::dim];
    double R[Plane3dMeasurement::dim*Plane3dMeasurement::dim];

    _parse_plane_factor (line, idx1, idx2, meas, sqrtinf, utime1, utime2);
    isamu_isam2van_plane3d_meas (meas, z);
    isamu_isam2van_plane3d_meas_sqrtinf (sqrtinf, R);

    /* cout << "Plane3d sqrtinf" << endl; */
    /* std::cout << sqrtinf << std::endl; */
    /* /\* std::cout << meas << std::endl; *\/ */
    /* cout << "Plane3d R" << endl; */
    /* for (int i=0; i<16; i++) { */
    /*     printf ("%f ", R[i]); */
    /* } */
    /* printf ("\n"); */

    if (idx1 == 0 && idx2 == 0) 
        throw runtime_error("ERROR: parsing odometry factor");

    perllcm_isam_vlink_t vlink = {0};
    vlink.utime = utime2;
    vlink.id1 = idx1; //(utime1 == 0)?idx1:utime1;
    vlink.id2 = idx2; //(utime2 == 0)?idx2:utime2;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = PERLLCM_ISAM_VLINK_T_SENSOR_PLANE;
    vlink.n = Plane3dMeasurement::dim;
    vlink.z = z;
    vlink.n2 = vlink.n*vlink.n;
    vlink.R = R;
    vlink.link_type = PERLLCM_ISAM_VLINK_T_LINK_PLANE3D;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_vlink_t *
gParser::_handle_1_tuple_factor (string line, int32_t link_type)
{
    double delta = 0.0; 
    double sqrtinf = 0.0;
    int64_t idx, utime = 0;
    _parse_1_tuple_factor (line, idx, delta, sqrtinf, utime);

    double z[1], R[1];
    z[0] = delta;
    if (sqrtinf > 0.0)
        R[0] = 1.0/sqrtinf;
    else
        R[0] = 0.0;

    perllcm_isam_vlink_t vlink = {0};
    vlink.utime = utime;
    vlink.id1 = 0;
    vlink.id2 = idx; //(utime == 0)?idx:utime;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = 0;
    vlink.n = 1;
    vlink.z = z;
    vlink.n2 = 1;
    vlink.R = R;
    vlink.link_type = link_type;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_vlink_t *
gParser::_handle_2_tuple_factor (string line, int32_t link_type)
{

    double delta[2]; MatrixXd sqrtinf (2,2); 
    int64_t idx, utime = 0;
    _parse_2_tuple_factor (line, idx, delta, sqrtinf, utime);

    // publish parsed factor: (depth, pitch, roll)
    double z[2], R[4];
    z[0] = delta[0];
    z[1] = delta[1];

    MatrixXd L = sqrtinf * sqrtinf.transpose();
    MatrixXd cov = L.inverse();
    
    for (int i=0;i<2; i++)
        for (int j=0;j<2; j++)
            R[i*2+j] = cov(i,j);

    perllcm_isam_vlink_t vlink = {0};
    vlink.utime = utime;
    vlink.id1 = 0;
    vlink.id2 = idx; //(utime == 0)?idx:utime;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = 0;
    vlink.n = 2;
    vlink.z = z;
    vlink.n2 = 2*2;
    vlink.R = R;
    vlink.link_type = link_type;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_vlink_t *
gParser::_handle_3_tuple_factor (string line, int32_t link_type)
{

    double delta[3]; MatrixXd sqrtinf (3,3); 
    int64_t idx, utime = 0;
    _parse_3_tuple_factor (line, idx, delta, sqrtinf, utime);

    // publish parsed factor: (depth, pitch, roll)
    double z[3], R[9];
    z[0] = delta[0];
    z[1] = delta[1];
    z[2] = delta[2];

    MatrixXd L = sqrtinf * sqrtinf.transpose();
    MatrixXd cov = L.inverse();
    
    for (int i=0;i<3; i++)
        for (int j=0;j<3; j++)
            R[i*3+j] = cov(i,j);

    perllcm_isam_vlink_t vlink = {0};
    vlink.utime = utime;
    vlink.id1 = 0;
    vlink.id2 = idx; //(utime == 0)?idx:utime;
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = 0;
    vlink.n = 3;
    vlink.z = z;
    vlink.n2 = 3*3;
    vlink.R = R;
    vlink.link_type = link_type;
    vlink.accept = true;
    vlink.accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

    perllcm_isam_vlink_t *vlink_dup = perllcm_isam_vlink_t_copy (&vlink);
    return vlink_dup;
}

perllcm_isam_glc_factor_t *
gParser::_handle_glc_factor (const std::string line, bool is_root_shift) {
    
    perllcm_isam_glc_factor_t *f = (perllcm_isam_glc_factor_t *)calloc (1, sizeof(*f));
    f->utime = 0;
    f->is_root_shifted = is_root_shift; 
    
    char *linep = g_strdup(line.c_str());
        
    char *pch = strtok (linep, " "); // get name of factor
    pch = strtok (NULL,"("); //get list of nodes
    char *ids_str = g_strdup(pch);
    pch = strtok (NULL,")"); //get x string
    char *x_str = g_strdup(pch);
    pch = strtok (NULL,"{");
    pch = strtok (NULL,"}"); //get u string
    char *U_str = g_strdup(pch);
//cout << "linep: " << linep << endl;
//cout << "ids_str: " << ids_str << endl;
//cout << "z_str: " << z_str << endl;
//cout << "U_str: " << U_str << endl;    
    
    // get ids
    vector<int64_t> ids; 
    pch = strtok (ids_str, " "); 
    while (pch) {
        ids.push_back (atol (pch));
        pch = strtok (NULL, " "); 
    }
    f->np = ids.size();
    f->ids = (int64_t *) calloc(f->np,sizeof(int64_t));
    memcpy (f->ids, &ids[0], f->np*sizeof(int64_t));

    // get x
    vector<double> x; 
    pch = strtok (x_str, " "); 
    while (pch) {
        x.push_back (atof (pch));
        pch = strtok (NULL, " "); 
    }
    f->n = x.size();
    f->x = (double *) calloc(f->n, sizeof(double));
    memcpy (f->x, &x[0], f->n*sizeof(double));
    
    vector<double> U;
    pch = strtok (U_str, " "); 
    while (pch) {
        U.push_back (atof (pch));
        pch = strtok (NULL, " "); 
    }
    f->m = U.size();
    f->U = (double *) calloc(f->m, sizeof(double));
    memcpy (f->U, &U[0], f->m*sizeof(double));

    free(linep);
    free(ids_str);
    free(x_str);
    free(U_str);

    return f;
}

// line level gParser
// ----------------------------------------------------------------
void 
gParser::_parse_utime (std::string str_idxed, int64_t &idx_i, int64_t &idx_j)
{
    char factor_line[1024];
    strcpy (factor_line, str_idxed.c_str());
    char *pch = strtok (factor_line," ,;(){}");

    size_t token_counter = 0;
    std::string factor_name;
    bool pairwise = false;

    while (pch) {
        if (token_counter == 0)
            factor_name = pch;

        // case1: pose3d_factor
        if (factor_name.compare (str_node) == 0
            || factor_name.compare (str_pose_prior) == 0
            || factor_name.compare (str_zpr) == 0 ) {
            pairwise = false;
            //std::cout << factor_name << " " << pairwise << std::endl;
        }
        else if (factor_name.compare (str_odo) == 0
                 || factor_name.compare (str_cam) == 0
                 || factor_name.compare (str_sonar) == 0)
            {
                pairwise = true;
                //std::cout << factor_name << " " << pairwise << std::endl;
            }

        if (token_counter == 1) idx_i = atol (pch);
        if (pairwise && token_counter == 2) idx_j = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }
}

void
gParser::_parse_prior (std::string str, int64_t &idx, isam::Pose3d &delta, Eigen::MatrixXd &sqrtinf, int64_t &utime)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());
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
            idx = atol (pch);

        if (2 <= token_counter && token_counter < 2+dof) {
            mu[idx_mu] = atof (pch);
            idx_mu ++;
        }

        if (2+dof <= token_counter && token_counter < 2+dof+dim_inf) {
            inf[idx_inf] = atof (pch);
            idx_inf ++;
        }

        if (token_counter == 2+dof+dim_inf)
            utime = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set delta
    delta.set (mu[0],mu[1],mu[2],mu[3],mu[4],mu[5]);

    // set sqrt information matrix
    sqrtinf <<
        inf[0], inf[1],  inf[2],  inf[3],  inf[4],  inf[5],
        0.0,    inf[6],  inf[7],  inf[8],  inf[9],  inf[10],
        0.0,    0.0,     inf[11], inf[12], inf[13], inf[14],
        0.0,    0.0,     0.0,     inf[15], inf[16], inf[17],
        0.0,    0.0,     0.0,     0.0,     inf[18], inf[19],
        0.0,    0.0,     0.0,     0.0,     0.0,     inf[20];
}

void
gParser::_parse_plane_prior (std::string str, int64_t &idx, isam::Plane3d &delta, Eigen::MatrixXd &sqrtinf, int64_t &utime)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());
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
            idx = atol (pch);

        if (2 <= token_counter && token_counter < 2+dof) {
            mu[idx_mu] = atof (pch);
            idx_mu ++;
        }

        if (2+dof <= token_counter && token_counter < 2+dof+dim_inf) {
            inf[idx_inf] = atof (pch);
            idx_inf ++;
        }

        if (token_counter == 2+dof+dim_inf)
            utime = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set delta
    delta.set (mu[0],mu[1],mu[2],mu[3],mu[4],mu[5]);

    // set sqrt information matrix
    sqrtinf <<
        inf[0], inf[1],  inf[2],  inf[3],  inf[4],  inf[5],
        0.0,    inf[6],  inf[7],  inf[8],  inf[9],  inf[10],
        0.0,    0.0,     inf[11], inf[12], inf[13], inf[14],
        0.0,    0.0,     0.0,     inf[15], inf[16], inf[17],
        0.0,    0.0,     0.0,     0.0,     inf[18], inf[19],
        0.0,    0.0,     0.0,     0.0,     0.0,     inf[20];
}

void 
gParser::_parse_node (std::string str, int64_t &idx, 
                      isam::Pose3d &node_mu, int64_t &utime, double &offset, bool &has_mu)
{
    // format: Pose3d_Node idx (x, y, z; h, p, r) utime offset (x, y, z; h, p, r) {sqrtinf 36 elements}
    char factor_line[1024];
    strcpy (factor_line, str.c_str());
    char *pch = strtok (factor_line," ,;(){}");

    // input
    size_t dof = 6; size_t dim_inf = 21;
    double mu[6] = {0}; 
    // double x_vs[6];
    //double inf[21];
    size_t token_counter = 0;
    int idx_mu = 0, idx_x_vs = 0, idx_inf = 0;

    while (pch) {
        if (token_counter == 0)
            std::string factor_name = pch;

        if (token_counter == 1)
            idx = atol (pch);

        if (2 <= token_counter && token_counter < 2+dof) {
            mu[idx_mu] = atof (pch);
            idx_mu ++;
        }

        if (token_counter == 2+dof)
            utime = atol (pch);

        if (token_counter == 2+dof+1)
            offset = atof (pch);

        if (2+dof+2 <= token_counter && token_counter < 2+dof+2+dof) {
            //x_vs[idx_x_vs] = atof (pch);
            idx_x_vs ++;
        }

        if (2+dof+2+dof <= token_counter && token_counter < 2+dof+2+dof+dim_inf) {
            //inf[idx_inf] = atof (pch);
            idx_inf ++;
        }

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set delta
    if (idx_mu) {
        node_mu.set (mu[0],mu[1],mu[2],mu[3],mu[4],mu[5]);
        has_mu = true;
    }

    /*// set sqrt information matrix
    sqrtinf <<
    inf[0], inf[1],  inf[2],  inf[3],  inf[4],  inf[5],
    0.0,    inf[6],  inf[7],  inf[8],  inf[9],  inf[10],
    0.0,    0.0,     inf[11], inf[12], inf[13], inf[14],
    0.0,    0.0,     0.0,     inf[15], inf[16], inf[17],
    0.0,    0.0,     0.0,     0.0,     inf[18], inf[19],
    0.0,    0.0,     0.0,     0.0,     0.0,     inf[20];*/
}

void 
gParser::_parse_odo_factor (std::string str, int64_t &idx_i, int64_t &idx_j,
                           isam::Pose3d &delta, Eigen::MatrixXd &sqrtinf,
                           int64_t &utime1, int64_t &utime2)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());

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

        if (token_counter == 3+dof+dim_inf)
            utime1 = atol (pch);

        if (token_counter == 3+dof+dim_inf+1)
            utime2 = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set delta
    delta.set (mu[0],mu[1],mu[2],mu[3],mu[4],mu[5]);

    // set sqrt information matrix
    sqrtinf <<
    inf[0], inf[1],  inf[2],  inf[3],  inf[4],  inf[5],
    0.0,    inf[6],  inf[7],  inf[8],  inf[9],  inf[10],
    0.0,    0.0,     inf[11], inf[12], inf[13], inf[14],
    0.0,    0.0,     0.0,     inf[15], inf[16], inf[17],
    0.0,    0.0,     0.0,     0.0,     inf[18], inf[19],
    0.0,    0.0,     0.0,     0.0,     0.0,     inf[20];
}

void 
gParser::_parse_cam_factor (std::string str, int64_t &idx_i, int64_t &idx_j,
                           isam::Pose3db &delta, Eigen::MatrixXd &sqrtinf,
                           isam::Pose3d &x_v1c, isam::Pose3d &x_v2c,
                           int64_t &utime1, int64_t &utime2)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());

    char *pch = strtok (factor_line," ,;(){}");

    // input
    size_t dof = 5; size_t dim_inf = 15; size_t senxform_dof = 6;
    double mu[5]; double inf[15]; double v1c[6], v2c[6];
    int idx_mu = 0, idx_inf = 0, idx_v1c = 0, idx_v2c = 0;
    size_t token_counter = 0;

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

        if (3+dof+dim_inf <= token_counter && token_counter < 3+dof+dim_inf+senxform_dof) {
            v1c[idx_v1c] = atof (pch);
            idx_v1c ++;
        }

        if (3+dof+dim_inf+senxform_dof <= token_counter && token_counter < 3+dof+dim_inf+senxform_dof*2) {
            v2c[idx_v2c] = atof (pch);
            idx_v2c ++;
        }

        if (token_counter == 3+dof+dim_inf+senxform_dof*2)
            utime1 = atol (pch);

        if (token_counter == 3+dof+dim_inf+senxform_dof*2+1)
            utime2 = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set delta
    delta.set (mu[0], mu[1], mu[2], mu[3], mu[4]);

    // set sqrt information matrix
    sqrtinf <<
    inf[0], inf[1], inf[2], inf[3],  inf[4],
    0.0,    inf[5], inf[6], inf[7],  inf[8],
    0.0,    0.0,    inf[9], inf[10], inf[11],
    0.0,    0.0,    0.0,    inf[12], inf[13],
    0.0,    0.0,    0.0,    0.0,     inf[14];

    // set x_v1c & x_v2c
    if (idx_v1c == 0)
        x_v1c.set (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    else
        x_v1c.set (v1c[0],v1c[1],v1c[2],v1c[3],v1c[4],v1c[5]);

    if (idx_v2c == 0)
        x_v2c.set (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    else
        x_v2c.set (v2c[0],v2c[1],v2c[2],v2c[3],v2c[4],v2c[5]);
}

void 
gParser::_parse_sonar_factor (std::string str, int64_t &idx_i, int64_t &idx_j,
                             isam::Pose2d &delta, Eigen::MatrixXd &sqrtinf,
                             isam::Pose3d &x_v1c, isam::Pose3d &x_v2c,
                             int64_t &utime1, int64_t &utime2)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());

    char *pch = strtok (factor_line," ,;(){}");

    // prior is in order = [prior.z(), prior.pitch(), prior.roll()]
    // input
    size_t dof = 3; size_t dim_inf = 6;
    double meas[3];   double inf[6];
    double v1c[6], v2c[6];
    int idx_v1c = 0, idx_v2c = 0;
    size_t senxform_dof = 6;

    size_t token_counter = 0;
    int idx_meas = 0, idx_inf = 0;

    while (pch) {
        if (token_counter == 0)
            std::string factor_name = pch;

        if (token_counter == 1)
            idx_i = atol (pch);

        if (token_counter == 2)
            idx_j = atol (pch);

        if (3 <= token_counter && token_counter < 3+dof) {
            meas[idx_meas] = atof (pch);
            idx_meas ++;
        }

        if (3+dof <= token_counter && token_counter < 3+dof+dim_inf) {
            inf[idx_inf] = atof (pch);
            idx_inf ++;
        }

        if (3+dof+dim_inf <= token_counter && token_counter < 3+dof+dim_inf+senxform_dof) {
            v1c[idx_v1c] = atof (pch);
            idx_v1c ++;
        }

        if (3+dof+dim_inf+senxform_dof <= token_counter && token_counter < 3+dof+dim_inf+senxform_dof*2) {
            v2c[idx_v2c] = atof (pch);
            idx_v2c ++;
        }

        if (token_counter == 3+dof+dim_inf+senxform_dof*2)
            utime1 = atol (pch);

        if (token_counter == 3+dof+dim_inf+senxform_dof*2+1)
            utime2 = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set delta [z, pitch, roll]
    delta.set (meas[0], meas[1], meas[2]);

    // set sqrt information matrix
    sqrtinf <<
    inf[0], inf[1], inf[2],
    0.0,    inf[3], inf[4],
    0.0,    0.0,    inf[5];

    // set x_v1c & x_v2c
    if (idx_v1c == 0)
        x_v1c.set (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    else
        x_v1c.set (v1c[0],v1c[1],v1c[2],v1c[3],v1c[4],v1c[5]);

    if (idx_v2c == 0)
        x_v2c.set (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    else
        x_v2c.set (v2c[0],v2c[1],v2c[2],v2c[3],v2c[4],v2c[5]);
}

void 
gParser::_parse_plane_factor (std::string str, int64_t &idx_i, int64_t &idx_j,
                              isam::Plane3dMeasurement &meas, Eigen::MatrixXd &sqrtinf,
                              int64_t &utime1, int64_t &utime2)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());

    char *pch = strtok (factor_line," ,;(){}");

    // input
    size_t dof = 4; size_t dim_inf = 10;
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

        if (token_counter == 3+dof+dim_inf)
            utime1 = atol (pch);

        if (token_counter == 3+dof+dim_inf+1)
            utime2 = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set delta
    meas.set (mu[0],mu[1],mu[2],mu[3]);

    // set sqrt information matrix
    sqrtinf <<
        inf[0], inf[1], inf[2], inf[3],  
        0.0,    inf[4], inf[5], inf[6],  
        0.0,    0.0,    inf[7], inf[8], 
        0.0,    0.0,     0.0,   inf[9];

}

// absolute measurements
void
gParser::_parse_1_tuple_factor (std::string str, int64_t &idx,
                                double &delta, double &sqrtinf,
                                int64_t &utime)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());

    char *pch = strtok (factor_line," ,;(){}");

    // input
    size_t dim_meas = 1; size_t dim_inf = 1;
    size_t token_counter = 0;
    int idx_meas = 0, idx_inf = 0;

    while (pch) {
        if (token_counter == 0)
            std::string factor_name = pch;

        if (token_counter == 1)
            idx = atol (pch);

        if (2 <= token_counter && token_counter < 2+dim_meas) {
            delta = atof (pch);
            idx_meas ++;
        }

        if (2+dim_meas <= token_counter && token_counter < 2+dim_meas+dim_inf) {
            sqrtinf = atof (pch);
            idx_inf ++;
        }

        if (token_counter == 2+dim_meas+dim_inf)
            utime = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }
}

void
gParser::_parse_2_tuple_factor (std::string str, int64_t &idx,
                                double delta[2], Eigen::MatrixXd &sqrtinf,
                                int64_t &utime)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());

    char *pch = strtok (factor_line," ,;(){}");

    // input
    size_t dim_meas = 2; size_t dim_inf = 3;
    double inf[3];
    size_t token_counter = 0;
    int idx_meas = 0, idx_inf = 0;

    while (pch) {
        if (token_counter == 0)
            std::string factor_name = pch;

        if (token_counter == 1)
            idx = atol (pch);

        if (2 <= token_counter && token_counter < 2+dim_meas) {
            delta[idx_meas] = atof (pch);
            idx_meas ++;
        }

        if (2+dim_meas <= token_counter && token_counter < 2+dim_meas+dim_inf) {
            inf[idx_inf] = atof (pch);
            idx_inf ++;
        }

        if (token_counter == 2+dim_meas+dim_inf)
            utime = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set sqrt information matrix
    sqrtinf <<
       inf[0], inf[1],
       0.0,    inf[2];
}

void
gParser::_parse_3_tuple_factor (std::string str, int64_t &idx,
                                double delta[3], Eigen::MatrixXd &sqrtinf,
                                int64_t &utime)
{
    char factor_line[1024];
    strcpy (factor_line, str.c_str());

    char *pch = strtok (factor_line," ,;(){}");

    // prior is in order = [prior.z(), prior.pitch(), prior.roll()]
    // input
    size_t dim_meas = 3; size_t dim_inf = 6;
    double inf[6];

    size_t token_counter = 0;
    int idx_meas = 0, idx_inf = 0;

    while (pch) {
        if (token_counter == 0)
            std::string factor_name = pch;

        if (token_counter == 1)
            idx = atol (pch);

        if (2 <= token_counter && token_counter < 2+dim_meas) {
            delta[idx_meas] = atof (pch);
            idx_meas ++;
        }

        if (2+dim_meas <= token_counter && token_counter < 2+dim_meas+dim_inf) {
            inf[idx_inf] = atof (pch);
            idx_inf ++;
        }

        if (token_counter == 2+dim_meas+dim_inf)
            utime = atol (pch);

        pch = strtok (NULL," ,;(){}");
        token_counter++;
    }

    // set sqrt information matrix
    sqrtinf <<
        inf[0], inf[1], inf[2],
        0.0,    inf[3], inf[4],
        0.0,    0.0,    inf[5];
}

