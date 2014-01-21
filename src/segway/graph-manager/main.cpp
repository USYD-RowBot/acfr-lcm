#include "seg_graph_manager.h"

using namespace std;


/**
 * @brief SegGraphManager Constructor
 */
SegGraphManager::SegGraphManager ()
{
    // open param cfg file
    param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param)
        std::cout << "could not find " << BOTU_PARAM_DEFAULT_CFG << std::endl;

    std::cout << "Opened Param File [" << BOTU_PARAM_DEFAULT_CFG << "]" << std::endl;
    
    // read from param file
    channel_drop_ds = bot_param_get_str_or_fail (param, "navigator.channel_drop_ds");
    channel_drop_ds_ack = bot_param_get_str_or_fail (param, "navigator.channel_drop_ds_ack");
    channel_seg_navigator = bot_param_get_str_or_fail (param, "navigator.channel");
    channel_lb3_imagesync = bot_param_get_str_or_fail (param, "seg-graph-manager.channel_lb3_imagesync");
    channel_drop_scan = bot_param_get_str_or_fail (param, "velodyne-scan-matcher.drop_scan_channel");
    channel_laser_plink = bot_param_get_str_or_fail (param, "velodyne-scan-matcher.plink_channel");
    channel_isam_vlink = bot_param_get_str_or_fail (param, "isamServer.lcm_channels.VLINK_CHANNEL");
    channel_isam_request_state = bot_param_get_str_or_fail (param, "isamServer.lcm_channels.REQUEST_ST_CHANNEL");
    channel_isam_return_state = bot_param_get_str_or_fail (param, "isamServer.lcm_channels.RETURN_ST_CHANNEL");
    channel_gps = NULL;
    bot_param_get_str(param, "seg-graph-manager.channel_gps", &channel_gps);
    channel_ms_25 = NULL;
    bot_param_get_str(param, "seg-graph-manager.channel_ms_25", &channel_ms_25);
    channel_ms = NULL;
    bot_param_get_str(param, "seg-graph-manager.channel_ms", &channel_ms);
    
    drop_node_dtrans = 1;
    bot_param_get_double (param, "seg-graph-manager.drop_node_dtrans", &drop_node_dtrans);
    drop_node_dyaw = 30;
    bot_param_get_double (param, "seg-graph-manager.drop_node_dyaw", &drop_node_dyaw);
    drop_node_dyaw *= UNITS_DEGREE_TO_RADIAN;
        
    max_laser_plinks = 2;
    bot_param_get_int (param, "seg-graph-manager.max_laser_plinks", &max_laser_plinks);
    laser_plink_trans_roc = 2.0;
    bot_param_get_double (param, "seg-graph-manager.laser_plink_trans_roc", &laser_plink_trans_roc);
    laser_plink_yaw_roc = 360.0;
    bot_param_get_double (param, "seg-graph-manager.laser_plink_yaw_roc", &laser_plink_yaw_roc);
    laser_plink_yaw_roc *= UNITS_DEGREE_TO_RADIAN;
    p_roc_thresh = 0.9;
    bot_param_get_double (param, "seg-graph-manager.p_roc_thresh", &p_roc_thresh);
    
    // gps linearization
    double ll_deg[2] = {0};
    bot_param_get_double_array_or_fail (param, "site.orglatlon", ll_deg, 2);
    llxy = (BotGPSLinearize *)calloc(1, sizeof (*llxy));
    bot_gps_linearize_init (llxy, ll_deg);
    org_alt = 0;
    bot_param_get_double (param, "site.orgalt", &org_alt);

    // partial factor noises
    gsl_matrix_view R_gps_v = gsl_matrix_view_array (R_gps, 3, 3);
    botu_param_read_covariance (&R_gps_v.matrix, param, (char *)"seg-graph-manager.R_gps");
    gsl_matrix_view R_ms_h_v = gsl_matrix_view_array (&R_ms_h, 1, 1);
    botu_param_read_covariance (&R_ms_h_v.matrix, param, (char *)"seg-graph-manager.R_ms_h");
    gsl_matrix_view R_ms_rp_v = gsl_matrix_view_array (R_ms_rp, 2, 2);
    botu_param_read_covariance (&R_ms_rp_v.matrix, param, (char *)"seg-graph-manager.R_ms_rp");
    
    // partial factor x_vs
    bot_param_get_double_array_or_fail (param, "sensors.gpsd3-rtk-client.x_vs", x_vs_gpsd, 6);
    x_vs_gpsd[3] = x_vs_gpsd[3] * UNITS_DEGREE_TO_RADIAN;
    x_vs_gpsd[4] = x_vs_gpsd[4] * UNITS_DEGREE_TO_RADIAN;
    x_vs_gpsd[5] = x_vs_gpsd[5] * UNITS_DEGREE_TO_RADIAN;
    bot_param_get_double_array_or_fail (param, "sensors.ms-gx3.x_vs", x_vs_ms, 6);
    x_vs_ms[3] = x_vs_ms[3] * UNITS_DEGREE_TO_RADIAN;
    x_vs_ms[4] = x_vs_ms[4] * UNITS_DEGREE_TO_RADIAN;
    x_vs_ms[5] = x_vs_ms[5] * UNITS_DEGREE_TO_RADIAN;
    bot_param_get_double_array_or_fail (param, "sensors.ms-gx3-25.x_vs", x_vs_ms_25, 6);
    x_vs_ms_25[3] = x_vs_ms_25[3] * UNITS_DEGREE_TO_RADIAN;
    x_vs_ms_25[4] = x_vs_ms_25[4] * UNITS_DEGREE_TO_RADIAN;
    x_vs_ms_25[5] = x_vs_ms_25[5] * UNITS_DEGREE_TO_RADIAN;
    bot_param_get_double_array_or_fail (param, "sensors.ms-gx3.x_lr", x_lr_ms, 6);
    x_lr_ms[3] = x_lr_ms[3] * UNITS_DEGREE_TO_RADIAN;
    x_lr_ms[4] = x_lr_ms[4] * UNITS_DEGREE_TO_RADIAN;
    x_lr_ms[5] = x_lr_ms[5] * UNITS_DEGREE_TO_RADIAN;
    bot_param_get_double_array_or_fail (param, "sensors.ms-gx3-25.x_lr", x_lr_ms_25, 6);
    x_lr_ms_25[3] = x_lr_ms_25[3] * UNITS_DEGREE_TO_RADIAN;
    x_lr_ms_25[4] = x_lr_ms_25[4] * UNITS_DEGREE_TO_RADIAN;
    x_lr_ms_25[5] = x_lr_ms_25[5] * UNITS_DEGREE_TO_RADIAN;
    is_done = 0;
    memset (last_nav_pose, 0, 6*sizeof (double));
    memset (last_node_pose, 0, 6*sizeof (double));
    
    // noise use for scan matches
    gsl_matrix_view R_scan_match_v = gsl_matrix_view_array (R_scan_match, 6, 6);
    botu_param_read_covariance (&R_scan_match_v.matrix, param, (char *)"velodyne-scan-matcher.R");
    
    world_frame = false;
    bot_param_get_int (param, "seg-graph-manager.world_frame", &world_frame);
    
    full_3d = false;
    bot_param_get_int (param, "seg-graph-manager.full_3d", &full_3d);

    // check lcm connection
    if(!lcm.good()) {
        is_done = 1;
        ERROR ("lcm_create () failed!");
    }
}

SegGraphManager::~SegGraphManager ()
{
    
}

// ----------------------------------------------------------------------------
// Main Loop
// ----------------------------------------------------------------------------
void
SegGraphManager::run ()
{    
    // subscribe to lcm channels
    lcm.subscribe(channel_seg_navigator, &SegGraphManager::segway_navigator_t_cb, this);
    lcm.subscribe(channel_lb3_imagesync, &SegGraphManager::lb3_imagesync_cb, this);
    lcm.subscribe(channel_isam_return_state, &SegGraphManager::isam_return_state_cb, this);
    
    if (world_frame) {
        if (channel_gps)
            lcm.subscribe(channel_gps, &SegGraphManager::gps_cb, this);
        if (channel_ms_25)
            lcm.subscribe(channel_ms_25, &SegGraphManager::ms_25_cb, this);
        if (channel_ms)
            lcm.subscribe(channel_ms, &SegGraphManager::ms_cb, this);
    }
    
    while (!is_done)
        lcm.handle();
}

void
SegGraphManager::gps_cb (const lcm::ReceiveBuffer *rbuf,
                         const std::string &chan,
                         const senlcm::gpsd3_t *msg)
{        
    if (msg->status >= senlcm::gpsd3_t::STATUS_FIX &&
        msg->fix.mode >= senlcm::gpsd3_fix_t::MODE_3D) {
        gps_queue.push_front(*msg);
    }
    
    while (gps_queue.size() > GPS_QUEUE_LEN)
        gps_queue.pop_back();    
}

void
SegGraphManager::ms_25_cb (const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const senlcm::ms_gx3_25_t *msg)
{    
    if (msg->bitmask & senlcm::ms_gx3_25_t::EULER)
        ms_25_queue.push_front (*msg);
    
    while (ms_25_queue.size() > MS_QUEUE_LEN)
        ms_25_queue.pop_back();
}

void
SegGraphManager::ms_cb (const lcm::ReceiveBuffer *rbuf,
                        const std::string &chan,
                        const senlcm::ms_gx3_t *msg)
{    
    if (msg->bitmask & senlcm::ms_gx3_t::STAB_EULER)
        ms_queue.push_front (*msg);
    
    while (ms_queue.size() > MS_QUEUE_LEN)
        ms_queue.pop_back();    
}


int
SegGraphManager::find_close_ms (int64_t node_utime)
{    
    if (0 == ms_queue.size())
        return -1;
    
    int cls_ind = -1;
    int64_t min_dif = 1e5;
    for (size_t i=0; i<ms_queue.size(); i++) {
        if (min_dif > abs (ms_queue[i].utime - node_utime)) {
            cls_ind = i;
            min_dif = abs (ms_queue[i].utime - node_utime);
        }
    }

    if (min_dif >= 1e5)
        ERROR ("No MS_GX3 entry within %lf sec.", min_dif/1e6);

    return cls_ind;
}

int
SegGraphManager::find_close_ms_25 (int64_t node_utime)
{    
    if (0 == ms_25_queue.size())
        return -1;
    
    int cls_ind = -1;
    int64_t min_dif = 1e5;
    for (size_t i=0; i<ms_25_queue.size(); i++) {
        if (min_dif > abs (ms_25_queue[i].utime - node_utime)) {
            cls_ind = i;
            min_dif = abs (ms_25_queue[i].utime - node_utime);
        }
    }

    if (min_dif >= 1e5)
        ERROR ("No MS_GX3_25 entry within %lf sec.", min_dif/1e6);

    return cls_ind;
}

int
SegGraphManager::find_close_gps (int64_t node_utime)
{    
    if (0 == gps_queue.size())
        return -1;
    
    int cls_ind = -1;
    int64_t min_dif = 1e6;
    for (size_t i=0; i<gps_queue.size(); i++) {     
        if (min_dif > abs (gps_queue[i].utime - node_utime)) {
            cls_ind = i;
            min_dif = abs (gps_queue[i].utime - node_utime);
        }
    }

    if (min_dif >= 1e6) {
        // happens often becaues rate is only one hz and we dont get packets
        // when it cant resolve a solution, not a big deal
        //ERROR ("No GPS entry within %lf sec.", min_dif/1e6);
    }
    return cls_ind;    
}

int
SegGraphManager::get_wr_pos (int64_t node_utime, double x_wv_out[6])
{    
    // @TODO if we were tracking uvw in the state we would want to coast the gps
    // measurement based on the dt between the node time and the gps hit
    // to a lesser extent we could do this for rph as well, but the data rate is ~50hz so doesn't really matter
    
    memset (x_wv_out, 0, 6*sizeof (double));
    
    int ind_ms = find_close_ms (node_utime);
    int ind_ms_25 = find_close_ms_25 (node_utime);
    int ind_gps = find_close_gps (node_utime);
    
    int status = 0;
    
    double rph_s[3] = {0};
    double x_lv[6] = {0};
    double x_vs[6] = {0};
    double x_lr[6] = {0};
    if (-1 != ind_ms_25) {
        memcpy (rph_s, ms_25_queue[ind_ms_25].Euler, 3*sizeof (double));
        memcpy (x_vs, x_vs_ms_25, 6*sizeof (double));
        memcpy (x_lr, x_lr_ms_25, 6*sizeof (double));
    }
    else if (-1 != ind_ms) {
        memcpy (rph_s, &(ms_queue[ind_ms].sEuler), 3*sizeof (double));
        memcpy (x_vs, x_vs_ms, 6*sizeof (double));
        memcpy (x_lr, x_lr_ms, 6*sizeof (double));
    }
    
    // roll and pitch
    if (-1 != ind_ms || -1 != ind_ms_25) {
    
        // x_lv = ominus (x_lr oplus x_rs) oplus x_sv
        double x_rs[6] = {0, 0, 0, rph_s[0], rph_s[1], rph_s[2]};
        double x_ls[6] = {0};
        ssc_head2tail (x_ls, NULL, x_lr, x_rs);
        double x_sv[6] = {0};
        ssc_inverse (x_sv, NULL, x_vs);
        ssc_tail2tail (x_lv, NULL, x_ls, x_sv);
    
        x_wv_out[3] = x_lv[3];
        x_wv_out[4] = x_lv[4];
        x_wv_out[5] = x_lv[5];
        
        status = 1; // rph
    }
    
    if (-1 != ind_gps && (-1 != ind_ms || -1 != ind_ms_25)) {
        
        //convert lat lon rads to local xy
        double ll_deg[2] = {gps_queue[ind_gps].fix.latitude*UNITS_RADIAN_TO_DEGREE,
                            gps_queue[ind_gps].fix.longitude*UNITS_RADIAN_TO_DEGREE};
        double yx[2] = {0};
        // NOTE returns yx -> returns in ENU not NED
        bot_gps_linearize_to_xy (llxy, ll_deg, yx);
        
        // move by x_vs
        //x_wv = x_ws oplus x_sv
        double x_sv_gps[6] = {0};
        ssc_inverse (x_sv_gps, NULL, x_vs_gpsd);
        double x_ws[6] = {yx[1], yx[0], gps_queue[ind_gps].fix.altitude - org_alt,
                          x_lv[3], x_lv[4], x_lv[5]}; //vehicle assumes no rotation between vehicle and gps "frames"
        double x_wv[6] = {0};
        ssc_head2tail (x_wv, NULL, x_ws, x_sv_gps);
        
        x_wv_out[0] = x_wv[0];
        x_wv_out[1] = x_wv[1];
        x_wv_out[2] = -x_wv[2];
        
        status = 2; // rph & gps
        
    }
    
    return status;
}

void
SegGraphManager::pub_parfac (int64_t node_utime, double mu[6], double Sigma[6*6])
{    
    double x_wv[6] = {0};
    int status = get_wr_pos (node_utime, x_wv);
    
    // currently dont send roll and pitch
    //// roll and pitch
    //if (status >= 1) {
    //    
    //    perllcm::isam_vlink_t vlink = get_empty_vlink ();
    //    vlink.id1 = 0;
    //    vlink.id2 = node_utime;
    //    vlink.n = 2;
    //    vlink.z.resize(2);
    //    vlink.z[0] = x_wv[3];
    //    vlink.z[1] = x_wv[4];
    //    vlink.n2 = 2*2;
    //    vlink.R.resize(2*2);
    //    memcpy (&(vlink.R[0]), R_ms_rp, 2*2*sizeof (double));
    //    vlink.link_type = perllcm::isam_vlink_t::LINK_RP_PARTIAL;        
    //    
    //    //lcm.publish (channel_isam_vlink, &vlink);
    //}
    
    gsl_matrix_view Sigma_v = gsl_matrix_view_array (Sigma, 6, 6);
    
    // gps
    if (status >= 2) {
        
        if (full_3d) {
            
            // check mahl distance of inovation covariance before updating
            GSLU_MATRIX_VIEW (Sigma_gps_innov_v, 3,3, {0});
            gsl_matrix_view R_gps_v = gsl_matrix_view_array (R_gps, 3, 3);
            gsl_matrix_view Sigma_xyz_v = gsl_matrix_submatrix (&Sigma_v.matrix, 0, 0, 3, 3);
            gsl_matrix_add (&Sigma_gps_innov_v.matrix, &Sigma_xyz_v.matrix);
            gsl_matrix_add (&Sigma_gps_innov_v.matrix, &R_gps_v.matrix);
            GSLU_MATRIX_VIEW (invCov, 3, 3);
            gslu_matrix_inv (&invCov.matrix, &Sigma_gps_innov_v.matrix);
            GSLU_VECTOR_VIEW (mu_xyz, 3, {mu[0], mu[1], mu[2]});
            GSLU_VECTOR_VIEW (x_wv_xyz, 3, {x_wv[0], x_wv[1], x_wv[2]});
            double dist = gslu_vector_mahal_dist (&mu_xyz.vector, &x_wv_xyz.vector, &invCov.matrix);
            
            double mdist = gsl_cdf_chisq_Pinv (0.9, 3);
            
            if (dist < mdist) {
                cout << "[partial]\tGPS passed Mahalanobis distance check against prior: " << dist << " < " << mdist << endl;
            
                // send xyz factor
                perllcm::isam_vlink_t vlink = get_empty_vlink ();
                vlink.id1 = 0;
                vlink.id2 = node_utime;
                vlink.n = 3;
                vlink.z.resize(3);
                vlink.z[0] = x_wv[0];
                vlink.z[1] = x_wv[1]; 
                vlink.z[2] = x_wv[2];
                vlink.n2 = 3*3;
                vlink.R.resize(3*3);
                memcpy (&(vlink.R[0]), R_gps, 3*3*sizeof (double));
                vlink.link_type = perllcm::isam_vlink_t::LINK_XYZ_PARTIAL;
                lcm.publish (channel_isam_vlink, &vlink);
                
            }
            else
                cout << "[partial]\tGPS FAILED Mahalanobis distance check against prior: " << dist << " > " << mdist << endl;
            
        }
        else {    
            // check mahl distance of inovation covariance before updating
            GSLU_MATRIX_VIEW (Sigma_gps_innov_v, 2,2, {0});
            GSLU_MATRIX_VIEW (R_gps_v, 2, 2, {R_gps[0], R_gps[1], R_gps[3], R_gps[4]});
            gsl_matrix_view Sigma_xy_v = gsl_matrix_submatrix (&Sigma_v.matrix, 0, 0, 2, 2);
            gsl_matrix_add (&Sigma_gps_innov_v.matrix, &Sigma_xy_v.matrix);
            gsl_matrix_add (&Sigma_gps_innov_v.matrix, &R_gps_v.matrix);
            GSLU_MATRIX_VIEW (invCov, 2, 2);
            gslu_matrix_inv (&invCov.matrix, &Sigma_gps_innov_v.matrix);
            GSLU_VECTOR_VIEW (mu_xy, 2, {mu[0], mu[1]});
            GSLU_VECTOR_VIEW (x_wv_xy, 2, {x_wv[0], x_wv[1]});
            double dist = gslu_vector_mahal_dist (&mu_xy.vector, &x_wv_xy.vector, &invCov.matrix);
            
            double mdist = gsl_cdf_chisq_Pinv (0.9, 2);
            
            if (dist < mdist) {
                cout << "[partial]\tGPS passed Mahalanobis distance check against prior: " << dist << " < " << mdist << endl;
                
                // send xy factor
                perllcm::isam_vlink_t vlink = get_empty_vlink ();
                vlink.id1 = 0;
                vlink.id2 = node_utime;
                vlink.n = 2;
                vlink.z.resize(2);
                vlink.z[0] = x_wv[0];
                vlink.z[1] = x_wv[1]; 
                vlink.n2 = 2*2;
                vlink.R.resize(2*2);
                vlink.R[0] = R_gps[0];
                vlink.R[1] = R_gps[1];
                vlink.R[2] = R_gps[3];
                vlink.R[3] = R_gps[4];
                vlink.link_type = perllcm::isam_vlink_t::LINK_XY_PARTIAL;
                lcm.publish (channel_isam_vlink, &vlink);
                
            }
            else
                cout << "[partial]\tGPS FAILED Mahalanobis distance check against prior: " << dist << " > " << mdist << endl;
        }
        
        // currently dont send heading, too noisy near buildings
        //// only use heading when we have gps, this should prevent noisy indoor
        //// heading measurements for some respects
        //perllcm::isam_vlink_t vlink_h = get_empty_vlink ();
        //vlink_h.id1 = 0;
        //vlink_h.id2 = node_utime;
        //vlink_h.n = 1;
        //vlink_h.z.resize(1);
        //vlink_h.z[0] = x_wv[5];
        //vlink_h.n2 = 1;
        //vlink_h.R.resize(1);
        //vlink_h.R[0] = R_ms_h;
        //vlink_h.link_type = perllcm::isam_vlink_t::LINK_H_PARTIAL; 
        //
        ////lcm.publish (channel_isam_vlink, &vlink_h);
        
    }
}


void
SegGraphManager::segway_navigator_t_cb (const lcm::ReceiveBuffer *rbuf,
                                        const std::string &chan,
                                        const perllcm::segway_navigator_t *msg)
{ 
    if (msg->state_len > 0) {
        
        // save the last pose
        last_nav_pose[0] = (-1 == msg->index.x) ? 0 : msg->mu[msg->index.x];
        last_nav_pose[1] = (-1 == msg->index.y) ? 0 : msg->mu[msg->index.y];
        last_nav_pose[2] = (-1 == msg->index.z) ? 0 : msg->mu[msg->index.z];
        last_nav_pose[3] = (-1 == msg->index.r) ? 0 : msg->mu[msg->index.r];
        last_nav_pose[4] = (-1 == msg->index.p) ? 0 : msg->mu[msg->index.p];
        last_nav_pose[5] = (-1 == msg->index.h) ? 0 : msg->mu[msg->index.h];
    }

}


int 
SegGraphManager::check_drop_pose(void)
{    
    double dist = sqrt (pow(last_nav_pose[0]-last_node_pose[0], 2) +
                        pow(last_nav_pose[1]-last_node_pose[1], 2) +
                        pow(last_nav_pose[2]-last_node_pose[2], 2));
    double yaw_dist = fabs (gslu_math_minimized_angle (last_nav_pose[5]-last_node_pose[5])); 
    
    if (dist > drop_node_dtrans || yaw_dist > drop_node_dyaw) {
        memcpy (last_node_pose, last_nav_pose, 6*sizeof (double));
        return 1;    
    }
    else
        return 0;
}

void
SegGraphManager::lb3_imagesync_cb (const lcm::ReceiveBuffer *rbuf,
                                        const std::string &chan,
                                        const perllcm::heartbeat_t *msg)
{    
    int64_t utime = msg->utime;
    // hack: problem with no delay in seg navigator when using heartbeat
    // not normal mode of operation, so leave it
    if (chan.compare(0,13,"SEG_HEARTBEAT") == 0)
        utime -= 2e5; // fakes out camera lag
    
    // lb3 data rate is the slowest so it drives the delayed state dropping
    if (0 == graph_nodes.size() || check_drop_pose()) { // first node
        
        // send signal to seg_nav to drop
        DropDSAck drop_ds = DropDSAck (channel_drop_ds, channel_drop_ds_ack);
        if (drop_ds.drop_ds (utime)) {
            
            if (0 == graph_nodes.size()) {
                // publish a prior
                perllcm::isam_vlink_t vlink = get_empty_vlink ();
                vlink.id1 = 0;
                vlink.id2 = utime;
                vlink.sensor_id = perllcm::isam_vlink_t::SENSOR_ODOMETRY;
                vlink.n = 6;
                vlink.z.resize(vlink.n);
                if (world_frame) {
                    double x_wv[6] = {0};
                    int status = get_wr_pos (utime, x_wv);
                    if (status >= 2) { //have gps and rph
                        memcpy (&(vlink.z[0]), x_wv, 6*sizeof (double));
                        if (!full_3d) { // force z to zero
                            vlink.z[2] = 0.0;
                        }
                        std::cout << "[graph]\tInit graph in global frame" << std::endl;
                    }
                    else
                        return;
                }
                else {
                    memcpy (&(vlink.z[0]), drop_ds.mu_new, 6*sizeof (double));
                    if (!full_3d) // force z to zero
                            vlink.z[2] = 0.0;

                    std::cout << "[graph]\tInit graph in local frame" << std::endl;
                }
                vlink.n2 = 6*6;
                vlink.R.resize(vlink.n2);
                
                memset (&(vlink.R[0]), 0, 6*6*sizeof (double));
                
                if (world_frame) {
                    vlink.R[0*6+0] = 9;    // sigma^2_x
                    vlink.R[1*6+1] = 9;    // sigma^2_y
                    vlink.R[2*6+2] = 25;   // sigma^2_z
                    vlink.R[3*6+3] = 1;    // sigma^2_r
                    vlink.R[4*6+4] = 1;    // sigma^2_p
                    vlink.R[5*6+5] = 9;    // sigma^2_h
                }
                else {
                    vlink.R[0*6+0] = 0.0001;    // sigma^2_x
                    vlink.R[1*6+1] = 0.0001;    // sigma^2_y
                    vlink.R[2*6+2] = 0.0001;   // sigma^2_z
                    vlink.R[3*6+3] = 0.0001;    // sigma^2_r
                    vlink.R[4*6+4] = 0.0001;    // sigma^2_p
                    vlink.R[5*6+5] = 0.0001;    // sigma^2_h
                }
                if (!full_3d) // force z to zero
                    vlink.R[2*6+2] = 0.0001;

                vlink.link_type = perllcm::isam_vlink_t::LINK_PRIOR;
                lcm.publish (channel_isam_vlink, &vlink);
                
            }
            else {
                // publish a factor
                perllcm::isam_vlink_t vlink = get_empty_vlink ();
                vlink.id1 = graph_nodes.back()->utime;
                vlink.id2 = utime;
                vlink.sensor_id = perllcm::isam_vlink_t::SENSOR_ODOMETRY;
                vlink.n = 6;
                vlink.z.resize(vlink.n);
                memcpy (&(vlink.z[0]), drop_ds.mu_odo, 6*sizeof (double));
                vlink.n2 = 6*6;
                vlink.R.resize(vlink.n2);
                memcpy (&(vlink.R[0]), drop_ds.Sigma_odo, 6*6*sizeof (double));
                vlink.link_type = perllcm::isam_vlink_t::LINK_POSE3D;
                lcm.publish (channel_isam_vlink, &vlink);
                
                // drive solution to 2.5D world
                if (!full_3d) {
                    // send z factor to force z to zero
                    perllcm::isam_vlink_t vlink = get_empty_vlink ();
                    vlink.id1 = 0;
                    vlink.id2 = utime;
                    vlink.n = 1;
                    vlink.z.resize(1);
                    vlink.z[0] = 0.0;
                    vlink.n2 = 1;
                    vlink.R.resize(1);
                    vlink.R[0] = 1;
                    vlink.link_type = perllcm::isam_vlink_t::LINK_Z_PARTIAL;
                    lcm.publish (channel_isam_vlink, &vlink);
                }
                
                //// remind the segway that it cant be too pitched or rolled
                //// with pure odometry this can be a problem, as there is no external measurement
                //// would be better to ms but has slight bias which cause spaceship mode
                //perllcm::isam_vlink_t vlink_rp = get_empty_vlink ();
                //vlink_rp.id1 = 0;
                //vlink_rp.id2 = utime;
                //vlink_rp.n = 2;
                //vlink_rp.z.resize(2);
                //vlink_rp.z[0] = 0.0;
                //vlink_rp.z[1] = 0.0;
                //vlink_rp.n2 = 2*2;
                //vlink_rp.R.resize(2*2);
                //memcpy (&(vlink_rp.R[0]), R_ms_rp, 2*2*sizeof (double));
                //vlink_rp.link_type = perllcm::isam_vlink_t::LINK_RP_PARTIAL;        
                //lcm.publish (channel_isam_vlink, &vlink_rp);
                
            }
            
            // save the local copy
            GraphNode *new_graph_node = new GraphNode (utime);
            new_graph_node->set_odometry (drop_ds.mu_odo);
            new_graph_node->set_odometry_cov (drop_ds.Sigma_odo);
            new_graph_node->set_pose (drop_ds.mu_new);
            graph_nodes.push_back (new_graph_node);
            
            // tell scan matcher to save a pose
            perllcm::heartbeat_t drop_scan;
            drop_scan.utime = utime;
            lcm.publish (channel_drop_scan, &drop_scan);
            
            // estimate this pose's block diagonal uncertianty until we over write it with one returned from isam
            if (block_diag_cov.empty()) {
                vector <double> tmp;
                tmp.resize(36);
                memcpy (&tmp[0], drop_ds.Sigma_odo, 36*sizeof (double));
                block_diag_cov[utime] = tmp;
            }
            else {
                double Sigma_prev[36] = {0};
                memcpy (Sigma_prev, &(block_diag_cov.rbegin()->second)[0], 36*sizeof (double));
                double empty1[6] = {0};
                double empty2[6] = {0};
                double J[6*12] = {0};
                ssc_head2tail(empty1, J, empty2, drop_ds.mu_odo);
                gsl_matrix_view J_v = gsl_matrix_view_array (J, 6, 12);
                gsl_matrix_view J1 = gsl_matrix_submatrix (&J_v.matrix, 0, 0, 6, 6);
                gsl_matrix_view J2 = gsl_matrix_submatrix (&J_v.matrix, 0, 6, 6, 6);
                GSLU_MATRIX_VIEW (tmp1, 6, 6, {0});
                GSLU_MATRIX_VIEW (tmp2, 6, 6, {0});
                gsl_matrix_view Sigma_prev_v = gsl_matrix_view_array (Sigma_prev, 6, 6);
                gsl_matrix_view Sigma_odo_v = gsl_matrix_view_array (drop_ds.Sigma_odo, 6, 6);
                gslu_blas_mmmT (&tmp1.matrix, &J1.matrix, &Sigma_prev_v.matrix, &J1.matrix, NULL);
                gslu_blas_mmmT (&tmp2.matrix, &J2.matrix, &Sigma_odo_v.matrix, &J2.matrix, NULL);
                gsl_matrix_add (&tmp1.matrix, &tmp2.matrix);

                vector <double> tmp;
                tmp.resize(36);
                memcpy (&tmp[0], tmp1.matrix.data, 36*sizeof (double));
                block_diag_cov[utime] = tmp;
            }
            
            // kick off a link hypothesis cycle by requesting mu for the entire graph
            request_state (perllcm::isam_request_state_t::POSE);
            
            std::cout << "[graph]\tAdding Node w/ utime = " << utime << std::endl;
        }
        
    }
}

void
SegGraphManager::request_state (int16_t state_type)
{    
    std::vector<int64_t> utimes;
    utimes.resize (graph_nodes.size());
    // ask for all graph nodes
    for (size_t i=0; i<graph_nodes.size(); i++)
        utimes[i] = graph_nodes[i]->utime;
    
    request_state (state_type, utimes);
}

void
SegGraphManager::request_state (int16_t state_type, std::vector<int64_t> utimes)
{        
    perllcm::isam_request_state_t rs = {0};
    
    rs.utime = timestamp_now ();
    rs.n = utimes.size();
    rs.state_type = state_type;
    rs.variables = utimes;
    lcm.publish (channel_isam_request_state, &rs);   
}



void
SegGraphManager::isam_return_state_cb (const lcm::ReceiveBuffer *rbuf,
                                       const std::string &chan,
                                       const perllcm::isam_return_state_t *msg)
{    
    if ((msg->state_type & perllcm::isam_request_state_t::POSE) && 
        (msg->state_type & perllcm::isam_request_state_t::COV_RIGHTCOL)) {

        // current mu and right column of covariance
        
        // save the bottom block to build up our block diagonal covariance
        // between COV_BLOCK requests
        int i_end = msg->n-1;
        vector<double> tmp;
        tmp.resize(36);
        memcpy (&tmp[0], &(msg->covariance[36*i_end]), 36*sizeof (double));
        block_diag_cov[msg->timestamps[i_end]] = tmp;
        
        // find and publish partial factors for this node
        if (world_frame) {
            double mu[6] = {0};
            double Sigma[6*6] = {0};
            memcpy (Sigma, &(msg->covariance[36*i_end]), 36*sizeof (double));
            memcpy (mu, &(msg->mu[i_end][0]), 6*sizeof (double));            
            pub_parfac (msg->timestamps[i_end], mu, Sigma);
        }
        
        propose_laser_links (msg);
        
        //propose_camera_links (msg);

        
    }
    else if (msg->state_type & perllcm::isam_request_state_t::POSE) {
        
        // find nodes, based on ecludian distance (translation),
        // that we want to request cross covariance (COV_RIGHTCOL) for
        std::vector<int64_t> utimes;
        int64_t utime_i = msg->timestamps[msg->n-1];
        std::vector<double> mu_i = msg->mu[msg->n-1];
        if (block_diag_cov.find(utime_i) == block_diag_cov.end()) {
            ERROR ("Missing block covariance for ith pose!");
            return;
        }
        std::vector<double> S_ii =  block_diag_cov[utime_i];

        // if its even remotly possible
        double mdist = gsl_cdf_chisq_Pinv (0.999999, 2);
        for (int j=0; j<msg->n-1; j++) {
            std::vector<double> mu_j = msg->mu[j];
            int64_t utime_j = msg->timestamps[j];
            // get the block diagonal associated with the jth poses
            if (block_diag_cov.find(utime_j) == block_diag_cov.end()) {
                ERROR ("Missing block covariance for jth pose!");
                continue;
            }
            std::vector<double> S_jj =  block_diag_cov[utime_j];
            
            GSLU_VECTOR_VIEW(xy_i, 2, {mu_i[0], mu_i[1]});
            GSLU_VECTOR_VIEW(xy_j, 2, {mu_j[0], mu_j[1]});
            GSLU_MATRIX_VIEW(cov, 2, 2, {S_ii[0] + S_jj[0], S_ii[1] + S_jj[1],
                                         S_ii[6] + S_jj[6], S_ii[7] + S_jj[7]});
            
            GSLU_MATRIX_VIEW (inv_cov, 2, 2, {0});
            gslu_matrix_inv (&inv_cov.matrix, &cov.matrix);
            double dist = gslu_vector_mahal_dist (&xy_i.vector, &xy_j.vector, &inv_cov.matrix);
            
            if (dist < mdist)
                utimes.push_back (utime_j);
        }

        // add the most recent utime right at the end
        utimes.push_back (utime_i);
        
        // request right column state for the most recent node and all
        int16_t r_type = perllcm::isam_request_state_t::POSE | perllcm::isam_request_state_t::COV_RIGHTCOL;
        request_state (r_type, utimes);
        
        // link hypothesis generation
        cout << "[link]\t" << utimes.size() << " cov_rightcol nodes requested based on mahl distance." << std::endl;
    }
    else if (msg->state_type & perllcm::isam_request_state_t::COV_BLOCK) {
        // full block diagonal covariance if we have asked for it
        size_t n_nodes = msg->m / 36;
        for (size_t i=0; i<n_nodes; i++) {
            vector<double> tmp;
            tmp.resize(36);
            memcpy (&tmp[0], &(msg->covariance[36*i]), 36*sizeof (double));
            block_diag_cov[msg->timestamps[i]] = tmp;
        }
    }
    
}

bool
pair_sort (std::pair<double, perllcm::isam_plink_t> i, std::pair<double, perllcm::isam_plink_t>  j)
{
    return (i.first > j.first);
}

void
SegGraphManager::propose_laser_links (const perllcm::isam_return_state_t *msg)
{    
    if (msg->n < 2)
        return;
   
    int reject_cnt = 0;
    int accept_cnt = 0;
    
    std::vector< std::pair<double, perllcm::isam_plink_t> > plinks_pairs;
    int ind_i = msg->n-1;
    std::vector<double> mu_i = msg->mu[ind_i];
    int64_t utime_i = msg->timestamps[ind_i];
    
    // get the block diagonal associated with the current pose
    if (block_diag_cov.find(utime_i) == block_diag_cov.end()) {
        ERROR ("Missing block covariance for most recent pose!");
        return;
    }
    std::vector<double> S_ii =  block_diag_cov[utime_i];
    
    GSLU_MATRIX_VIEW (S_iijj, 12, 12);
    gsl_matrix_view S_sub_ii = gsl_matrix_submatrix (&S_iijj.matrix, 0, 0, 6, 6);
    gsl_matrix_view S_sub_ij = gsl_matrix_submatrix (&S_iijj.matrix, 0, 6, 6, 6);
    gsl_matrix_view S_sub_ji = gsl_matrix_submatrix (&S_iijj.matrix, 6, 0, 6, 6);
    gsl_matrix_view S_sub_jj = gsl_matrix_submatrix (&S_iijj.matrix, 6, 6, 6, 6);
    gsl_matrix_view S_ii_v = gsl_matrix_view_array (&(S_ii[0]), 6, 6);
    gsl_matrix_memcpy (&S_sub_ii.matrix, &S_ii_v.matrix);
    
    gsl_matrix_view R_sm_v = gsl_matrix_view_array (R_scan_match, 6, 6);
    double det_R_sm = gslu_matrix_det (&R_sm_v.matrix);
    
    // calculate information gain for each link
    // propose links sorted by information gain
    // also make sure that we think they are within GICP's region of convergence
    for (int j=0; j<ind_i; j++) {
        std::vector<double> mu_j = msg->mu[j];
        int64_t utime_j = msg->timestamps[j];
        
        // fill this poses covariance
        if (block_diag_cov.find(utime_j) == block_diag_cov.end()) {
            ERROR ("Missing block covariance for j pose!");
            continue;
        }
        std::vector<double> S_jj =  block_diag_cov[utime_j];
        gsl_matrix_view S_jj_v = gsl_matrix_view_array (&(S_jj[0]), 6, 6);
        gsl_matrix_memcpy (&S_sub_jj.matrix, &S_jj_v.matrix);
        
        // fill the corss covariance
        const double *cov = &(msg->covariance[0]);
        gsl_matrix_const_view cov_ij = gsl_matrix_const_view_array (cov+36*j, 6, 6);
        gsl_matrix_memcpy (&S_sub_ij.matrix, &cov_ij.matrix);
        gsl_matrix_transpose_memcpy (&S_sub_ji.matrix, &cov_ij.matrix);   
        
        // get the prior
        double x_ji[6] = {0};
        double Sigma_ji[6*6] = {0};
        
        relative_pose (x_ji, Sigma_ji, &(mu_i[0]), &(mu_j[0]), S_iijj.data);
        
        // create a 3D pose
        perllcm::pose3d_t p3d;
        memcpy (p3d.mu, x_ji, 6*sizeof (double));
        p3d.utime = utime_i;
        memcpy (p3d.Sigma, Sigma_ji, 6*6*sizeof (double));
        
        // create a plink
        perllcm::isam_plink_t pl;
        pl.utime_i = utime_i;
        pl.utime_j = utime_j;
        pl.prior = 1;        
        pl.x_ji = p3d;    
        pl.link_id = 1; //TODO MAKE REAL ID
        pl.sensor_id = perllcm::isam_vlink_t::SENSOR_LASER;
        
        // calcluate the expected information gain
        GSLU_MATRIX_VIEW (S, 6, 6, {0});
        gsl_matrix_view Sigma_ji_v = gsl_matrix_view_array (Sigma_ji, 6, 6);
        gsl_matrix_add (&S.matrix, &Sigma_ji_v.matrix);
        gsl_matrix_add (&S.matrix, &R_sm_v.matrix);
        double ig = 0.5 * log (gslu_matrix_det (&S.matrix) / det_R_sm);
        
        // we want x and y translation to be less than laser_plink_dtrans
        // and we want d rotation to be less than laser_plink_dyaw
        // this defines what we consider to be the convergence region for the algorithm
        // follows "Information-Based Compact Pose SLAM", Ila.
        double sigma_x = sqrt(S_ii[0] + S_jj[0]);
        double sigma_y = sqrt(S_ii[1*6+1] + S_jj[1*6+1]);
        double sigma_h = sqrt(S_ii[5*6+5] + S_jj[5*6+5]);
        double p_in_roc_x =  prob_in_roc (mu_i[0] - mu_j[0], laser_plink_trans_roc, sigma_x);
        double p_in_roc_y =  prob_in_roc (mu_i[1] - mu_j[1], laser_plink_trans_roc, sigma_y);
        double p_in_roc_h =  prob_in_roc (mu_i[5] - mu_j[5], laser_plink_yaw_roc, sigma_h, true);

        if (p_in_roc_x > p_roc_thresh && p_in_roc_y > p_roc_thresh && p_in_roc_h > p_roc_thresh) {
            plinks_pairs.push_back (make_pair (ig, pl));
            accept_cnt++;
        }
        else
            reject_cnt++;
    } 
    
    // sort based on information gain
    sort (plinks_pairs.begin(), plinks_pairs.end(), pair_sort);

    //for (size_t j=0; j<plinks_pairs.size(); j++){
    //    cout << plinks_pairs[j].first << " -- "  << plinks_pairs[j].second.utime_j << endl;
    //}
    
    perllcm::isam_plink_collection_t plc;
    plc.nlinks = 0;
    
    // publish the plinks
    int64_t cl_utime = 0;
    int num = min (max_laser_plinks, (int) plinks_pairs.size());
    for (int j=0; j<num; j++){
        
        perllcm::isam_plink_t pl = plinks_pairs[j].second;
            
        // most recent utime in this link
        int64_t mr_utime = (pl.utime_i >= pl.utime_j) ? pl.utime_i : pl.utime_j ;
        if (mr_utime > cl_utime)
            cl_utime = mr_utime;
        
        // add it to a collection
        plc.plink.push_back (pl);
        plc.nlinks++;
    }
    
    // publish the collection
    if (plc.nlinks > 0) {
        plc.utime = cl_utime; // the utime of the latest link in the collection
        lcm.publish (channel_laser_plink, &plc);
    }
    
    
    std::cout << "[link]\t" << plc.nlinks << " laser links proposed: " << accept_cnt << "/" << reject_cnt+accept_cnt << " passed roc check" << std::endl;
}




// ----------------------------------------------------------------------------
// Ctrl+C ching
// ----------------------------------------------------------------------------
SegGraphManager *g_sgm;
void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    std::cout << "Sigint caught. Quitting ..." << std::endl;
    if (g_sgm->is_done ) {
        std::cout << "Goodbye" << std::endl;
        exit (EXIT_FAILURE);
    } 
    else
        g_sgm->is_done = 1;
}

// ----------------------------------------------------------------------------
// Main 
// ----------------------------------------------------------------------------
int
main (int argc, char *argv[])
{    
    fasttrig_init ();
    
    SegGraphManager sgm = SegGraphManager ();
    g_sgm = &sgm;
    
    // install custom signal handler
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    // kick off the manager
    sgm.run ();
    
    return 0;   
}
