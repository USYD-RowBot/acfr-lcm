#include <iostream>
#include <stdexcept>
#include <lcm/lcm.h>

#include "perls-math/ssc.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"

#include "perls-lcmtypes++/hauv/wp_list_t.hpp"

#include "perls-lcmtypes++/perllcm/isam_info_t.hpp"
#include "perls-lcmtypes++/hauv/pl_rns_t.hpp"
#include "perls-lcmtypes++/hauv/pl_sus_t.hpp"
#include "perls-lcmtypes++/hauv/pl_ghp_t.hpp"

#include "HauvClient.h"

#define MAX_WP_CMD_HZ 3.0

using namespace Eigen;
using namespace std;
using namespace lcm;

HauvClient::HauvClient(LCM& lcm)
{
    this->m_lcm = &lcm;
    this->m_prev_node_utime = 0;
    this->m_param = bot_param_new_from_file(BOTU_PARAM_DEFAULT_CFG);
    this->m_active = false;
    this->m_has_rnv = false;
    this->m_has_cnv = false;
    this->m_initialized_prior = false;
    this->m_start_adding_nodes = false;
    this->m_resume_waypoint = 0;
    this->m_waypoint_goto_id = 0;

    if (0!=bot_param_get_str (this->m_param, "isamServer.lcm_channels.VLINK_CHANNEL", &m_vlink_channel))
        throw runtime_error("isamServer.lcm_channels.VLINK_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "hauv-client.lcm_channels.RTVAN_ADD_NODE_CHANNEL", &m_rtvan_add_node_channel))
        throw runtime_error("hauv-client.lcm_chanenls.RTVAN_ADD_NODE_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "hauv-client.lcm_channels.SONAR_ADD_NODE_CHANNEL", &m_sonar_add_node_channel))
        throw runtime_error("hauv-client.lcm_chanenls.SONAR_ADD_NODE_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "hauv-client.lcm_channels.SELF_ADD_NODE_CHANNEL", &m_self_add_node_channel))
        throw runtime_error("hauv-client.lcm_chanenls.SELF_ADD_NODE_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "isamServer.lcm_channels.CMD_CHANNEL", &m_isam_cmd_channel))
        throw runtime_error("isamServer.lcm_channels.CMD_CHANNEL not found in config file");

    // waypoint navigation
    if (0!=bot_param_get_str (this->m_param, "hauv-client.wp_nav.GOTO_STATE_CHANNEL", &m_goto_state_channel))
        throw runtime_error("hauv-client.wp_nav.GOTO_STATE_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "hauv-client.wp_nav.GOTO_CHANNEL", &m_goto_channel))
        throw runtime_error("hauv-client.wp_nav.GOTO_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "hauv-client.wp_nav.GOTO_RQ_CHANNEL", &m_rq_goto_channel))
        throw runtime_error("hauv-client.wp_nav.GOTO_RQ_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "hauv-client.wp_nav.WP_LIST_CHANNEL", &m_wp_list_channel))
        throw runtime_error("hauv-client.wp_nav.WP_LIST_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "hauv-client.wp_nav.SAVE_CMD_CHANNEL", &m_wp_save_channel))
        throw runtime_error("hauv-client.wp_nav.SAVE_CMD_CHANNEL not found in config file");

    if (0!=bot_param_get_str (this->m_param, "isamServer.lcm_channels.VIS_CHANNEL", &m_graph_vis_channel))
        throw runtime_error("isamServer.lcm_channels.VIS_CHANNEL not found in config file");

}

HauvClient::~HauvClient() 
{
    //Free strings alloc'ed with stdlib
    free(m_vlink_channel);
    free(m_rtvan_add_node_channel);
    free(m_self_add_node_channel);
}

void
HauvClient::run()
{
    // start lcm subscription
    while (0 == m_lcm->handle());

    return;
}

void
HauvClient::perllcm_isam_cmd_t_callback (const ReceiveBuffer *rbuf,
                                         const std::string& channel,
                                         const perllcm::isam_cmd_t *msg)
{
    if (msg->mode == perllcm::isam_cmd_t::MODE_START)
        m_active = true;
}

void
HauvClient::hauv_bs_rnv_2_t_callback (const ReceiveBuffer *rbuf, 
                                      const std::string& channel,
                                      const hauv::bs_rnv_2_t *msg)
{
    if (!m_has_rnv || !m_active) {
        m_origin[3] = msg->absroll;
        m_origin[4] = msg->abspitch;
        m_origin[5] = msg->absheading;
        m_has_rnv = true;
    }

    if (m_has_cnv && m_has_rnv && !m_initialized_prior && m_active) {
        this->init_slam(msg->time_received);
    }

    this->m_traj.add (msg);
}

void
HauvClient::hauv_bs_pit_t_callback (const ReceiveBuffer *rbuf, 
                                    const std::string& channel,
                                    const hauv::bs_pit_t *msg)
{
    this->m_traj.add (msg);
}

void
HauvClient::hauv_bs_cnv_t_callback (const ReceiveBuffer *rbuf, 
                                    const std::string& channel,
                                    const hauv::bs_cnv_t *msg)
{
    if (!m_has_cnv || !m_active) {
        m_origin[0] = msg->x;
        m_origin[1] = msg->y;
        m_origin[2] = msg->z;
        m_has_cnv = true;
    }
    
    if (m_has_cnv && m_has_rnv && !m_initialized_prior && m_active) {
        this->init_slam(msg->time_received);
    }

    this->m_traj.add (msg);
}

void
HauvClient::hauv_bs_dvl_2_t_callback (const ReceiveBuffer *rbuf, 
                                      const std::string& channel,
                                      const hauv::bs_dvl_2_t *msg)
{
    this->m_traj.add (msg);
}

//This should only be used for testing purposes
void
HauvClient::perllcm_heartbeat_t_2_callback (const ReceiveBuffer *rbuf, 
                                            const std::string& channel,
                                            const perllcm::heartbeat_t *msg)
{
    this->addNode(msg->utime - HAUV_UTIME_OFFSET);
}

void
HauvClient::perllcm_heartbeat_t_5_callback (const ReceiveBuffer *rbuf, 
                                            const std::string& channel,
                                            const perllcm::heartbeat_t *msg)
{
    // broadcast waypoints
    broadcast_wp();
    generateWaypoint();

    // update hauv-clinet
    this->update();

}

void
HauvClient::perllcm_rtvan_add_node_callback(const ReceiveBuffer *rbuf, 
                                            const std::string& channel,
                                            const perllcm::heartbeat_t *msg)
{
    // Add the new node to the incoming queue
    HCINFO("Queue node %lu from CAMERA", msg->utime);
    this->queueNode(msg, perllcm::isam_vlink_t::SENSOR_CAMERA);
}

void
HauvClient::perllcm_sonar_add_node_callback(const ReceiveBuffer *rbuf, 
                                            const std::string& channel,
                                            const perllcm::heartbeat_t *msg)
{
    // Add the new node to the incoming queue
    HCINFO("Queue node %lu from SONAR", msg->utime);
    this->queueNode(msg, perllcm::isam_vlink_t::SENSOR_SONAR);
}

//Used for interpolating nodes
void
HauvClient::perllcm_self_add_node_callback(const ReceiveBuffer *rbuf, 
                                            const std::string& channel,
                                            const perllcm::heartbeat_t *msg)
{
    // Add the new node to the incoming queue
    HCINFO("Queue node %lu from SELF", msg->utime);
    this->queueNode(msg, perllcm::isam_vlink_t::SENSOR_ODOMETRY);
}

/*------------------------------------------------------------

  BEGIN PUBLIC MEMBER FUNCTIONS

  ------------------------------------------------------------*/
void
HauvClient::queueNode(const perllcm::heartbeat_t *msg, int creator)
{
    // Add the new node to the incoming queue
    // We only add it to the estimator when there is at least one
    // available constraint.    

    map<int64_t, perllcm::heartbeat_t*>::iterator it;
    it = m_nodes.find(msg->utime);
    if (it != m_nodes.end()) 
        HCWARN("Node %lu already exists", msg->utime);
    else {
        //create a new add_node_t
        perllcm::isam_vlink_t *factorToAdd = new perllcm::isam_vlink_t;
        perllcm::heartbeat_t *msgCopy = new perllcm::heartbeat_t;
        *msgCopy = *msg;
        factorToAdd->utime = msg->utime;
        factorToAdd->creator_of_id2 = creator;
        m_nodes[msg->utime] = msgCopy;
        m_new_nodes.push_back(factorToAdd);
    }
}

void
HauvClient::addNode(int64_t utime) 
{
    if (!m_initialized_prior || utime < m_prev_node_utime)
        return;

    HCINFO("Add node %lu (%lu)", utime, m_prev_node_utime);
    perllcm::heartbeat_t add_node;
    add_node.utime = utime;
    m_lcm->publish(m_self_add_node_channel, &add_node);
}

void
HauvClient::broadcast_wp()
{
    hauv::wp_list_t list;
    list.n = m_saved_waypoints.size();
    
    // broadcast wp and let the renderer to draw the wp (wp = [id, utime])
    std::map<int32_t, int64_t>::iterator it_wp;
    for (it_wp=m_saved_waypoints.begin(); it_wp!=m_saved_waypoints.end(); it_wp++) {
        if (it_wp->second > 0) {
            list.id.push_back (it_wp->first);
            list.utime.push_back (it_wp->second);
        }
    }

    m_lcm->publish (m_wp_list_channel, &list);
}

void
HauvClient::update()
{
    if (!m_initialized_prior) {
        return;
    }

    if (m_new_nodes.empty()) {
        //std::cout << "No new nodes..." << std::endl;
        //std::cout << "--------------------------------------------------" << std::endl;
        return;
    }

    while (!m_new_nodes.empty()) {

        perllcm::isam_vlink_t* n = m_new_nodes.front();

        if (!m_traj.canInterpolate(n->utime)) {
            HCWARN("Can not interpolate %lu", n->utime);
            int64_t oldest = m_traj.getOldest();
            if (oldest == 0 || oldest > n->utime) {
                m_new_nodes.pop_front();
                delete n;
            }
            break;
        }

        State s = this->m_traj.getState(n->utime);
        if (!this->m_start_adding_nodes)
            this->initialize(s);
        else
            this->publishDeltas(s, n->creator_of_id2);

    }

    std::cout << "--------------------------------------------------" << std::endl;

}

/*------------------------------------------------------------

  BEGIN PRIVATE MEMBER FUNCTIONS

  ------------------------------------------------------------*/
void
HauvClient::init_slam(uint64_t utime)
{
    HCINFO("Initializing origin prior at %lu", utime);
    MatrixXd sigma(6,6);
    double s0_xyz = 0.00001*0.00001;
    double s0_rpz = 0.0001*DTOR*0.0001*DTOR;
    sigma <<
        s0_xyz, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, s0_xyz, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, s0_xyz, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, s0_rpz, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, s0_rpz, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, s0_rpz;

    this->sendLinkPrior(utime, m_origin, sigma);
    m_initialized_prior = true;
    m_prev_node_utime = utime;
}

void
HauvClient::initialize(const State& state) 
{
    HCINFO("Adding first odometry constraint");
    MatrixXd sigma(6,6);
    double s0_xyz = 0.00001*0.00001;
    double s0_rpz = 0.0001*DTOR*0.0001*DTOR;
    sigma <<
        s0_xyz, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, s0_xyz, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, s0_xyz, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, s0_rpz, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, s0_rpz, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, s0_rpz;
  
    //Pose3d next_pose = (s.utime==0)?Pose3d():Pose3d(s.x,s.y,s.z,s.heading,s.pitch,s.roll);
    double next_pose[6] = {state.x, state.y, state.z, state.roll, state.pitch, state.heading};
    double delta[6];

    ssc_tail2tail(delta, NULL, m_origin, next_pose);
    HCINFO("Delta 0 : (%f %f %f %f %f %f)", delta[0],delta[1],delta[2],delta[3],delta[4],delta[5]);

    this->sendLink(m_prev_node_utime, state.utime, delta, sigma, perllcm::isam_vlink_t::CODE_ACCEPTED);
    save_wp_nodes (state);

    memcpy(m_prev_pose, next_pose, 6*sizeof(*m_prev_pose));
    m_prev_node_utime = state.utime;
    delete m_new_nodes.front();
    m_new_nodes.pop_front();
    m_start_adding_nodes = true;
}

void
HauvClient::publishDeltas(const State& state, const int32_t& creator)
{

    State prev_state = m_traj.getState(m_prev_node_utime);

    double dt = (state.utime-prev_state.utime)*1E-6;

    bool caninterpolate = true;
    if (prev_state.utime == 0)
        caninterpolate = false;
          
    HCINFO("Add new constraint: %.5f %.1f", dt, HAUV_STEP_SIZE);

    if (dt > HAUV_STEP_SIZE && caninterpolate)
        this->interpolate(prev_state, dt, creator);

    if (dt<0.1) dt =0.1; //001) dt = 0.001;

    // Add a new node to the pose graph (squareroot information)
    double sxx, syy, szz; sxx=syy=szz=1./(odo_std_xyz*dt);
    double srr, spp, shh; srr=spp=shh=1./(odo_std_rph*dt);
    MatrixXd sigma(6, 6);
    sigma <<
        1/(sxx*sxx), 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1/(syy*syy), 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1/(szz*szz), 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1/(shh*shh), 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1/(spp*spp), 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1/(srr*srr);

    double pose[6], delta[6];
    pose[0] = state.x;
    pose[1] = state.y;
    pose[2] = state.z;
    pose[3] = state.roll;
    pose[4] = state.pitch;
    pose[5] = state.heading;

    this->setPrevPose(prev_state);

    ssc_tail2tail (delta, NULL, m_prev_pose, pose);

    // if delta is larger than 5m due to the error in HAUV message, ignore
    if (delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] > 5*5 || delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] < 0.1*0.1) {
        delete m_new_nodes.front();
        m_new_nodes.pop_front();
        return;
    }

    this->sendLink(prev_state.utime, state.utime, delta, sigma, creator);
    save_wp_nodes (state);

    // std::cout << "addNode - " << m_nodes.size() << " " << m_prev_node->value() << " " << node->value() << " " << dt << " " << sxx << " utime: " << s.utime << std::endl;
    // std::cout << "addNode - " << " from: " << prev_state.utime << " to " << s.utime <<  " node " << m_prev_node << std::endl;
    HCINFO("Delta odometry : (%f %f %f %f %f %f)", delta[0],delta[1],delta[2],delta[3],delta[4],delta[5]);

    MatrixXd sigma_zpr(3, 3);
    double zpr[3] = {pose[2], pose[4], pose[3]};
    szz=1./gbl_std_z; srr=1./gbl_std_r; spp=1./gbl_std_p;
    sigma_zpr <<
        1/(szz*szz),  0.0, 0.0,
        0.0,  1/(spp*spp), 0.0,
        0.0,  0.0, 1/(srr*srr);

    // Add global constraint using depth, pitch and roll
    sendLinkPartial(state.utime, zpr, sigma_zpr);

    m_prev_node_utime = state.utime;
    delete m_new_nodes.front();
    m_new_nodes.pop_front();

}

void
HauvClient::interpolate(State& prev_state, double& dt, const int32_t& creator)
{
    // add nodes up to new node
    double tmp_dt = dt;
    dt = HAUV_STEP_SIZE;

    // Get next state
    State next_state = m_traj.getState(m_prev_node_utime + dt*1E6);

    HCINFO("Interpolating from %lu to %lu", prev_state.utime, next_state.utime);

    HCINFO("%f %f", tmp_dt, dt);
    while (tmp_dt > dt) {
        // Add a new node to the pose graph 
        //Below are the squareroot information terms
        double sxx, syy, szz; sxx=syy=szz=1./(odo_std_xyz*dt);
        double srr, spp, shh; srr=spp=shh=1./(odo_std_rph*dt);
        MatrixXd sigma(6, 6);
        //Convert sqrtinf to covariance
        sigma <<
            1/(sxx*sxx), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1/(syy*syy), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1/(szz*szz), 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1/(shh*shh), 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1/(spp*spp), 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1/(srr*srr);

        double pose[6], delta[6];
        if (next_state.utime == 0)
            for (int i=0; i<6; i++)
                pose[i] = 0.0;
        else {
            pose[0] = next_state.x;
            pose[1] = next_state.y;
            pose[2] = next_state.z;
            pose[3] = next_state.roll;
            pose[4] = next_state.pitch;
            pose[5] = next_state.heading;
        }

        this->setPrevPose(prev_state);
        ssc_tail2tail (delta, NULL, m_prev_pose, pose);

        // if delta is larger than 5m due to the error in HAUV message, ignore
        if (delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] > 5*5 || delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] < 0.1*0.1) {
            tmp_dt -= dt;
            continue;
        }

        this->sendLink(prev_state.utime, next_state.utime, delta, sigma, perllcm::isam_vlink_t::SENSOR_ODOMETRY);
        save_wp_nodes (next_state);
        //this->addNode(next_state.utime);

        HCINFO("Add odometry %zu from: %lu to %lu %.2f %.2f %.2f", m_nodes.size(), prev_state.utime, next_state.utime, tmp_dt, dt, HAUV_STEP_SIZE);

        // Add global constraint using depth, pitch and roll
        double zpr[3] = {next_state.z, next_state.pitch, next_state.roll};
        szz=1./gbl_std_z; srr=1./gbl_std_r; spp=1./gbl_std_p;
        MatrixXd sigma_zpr(3, 3);
        sigma_zpr <<
            1/(szz*szz),  0.0, 0.0,
            0.0,  1/(spp*spp), 0.0,
            0.0,  0.0, 1/(srr*srr);

        this->sendLinkPartial(next_state.utime, zpr, sigma_zpr);

        m_prev_node_utime = next_state.utime;
        // m_prev_node = node;
        prev_state = next_state;
        next_state = m_traj.getState(m_prev_node_utime+dt*1E6);

        tmp_dt -= dt;
    }
    dt = tmp_dt;

}

void 
HauvClient::sendLinkPartial(int64_t utime_to,
                            double mu[3],
                            const MatrixXd& sigma)
{
    HCINFO("Sending partial link for node %lu", utime_to);
    double *doubleArray = (double *) calloc(9, sizeof (*doubleArray));
    this->MatrixXdToArray(sigma, &doubleArray);

    this->sendLinkPartial(utime_to, mu, doubleArray);

    free(doubleArray);
}


void
HauvClient::sendLink(int64_t utime_from, 
                     int64_t utime_to, 
                     double mu[6], 
                     const MatrixXd& sigma,
                     int32_t creator)
{
    HCINFO("Sending full link from %lu to %lu", utime_from, utime_to);
    double *doubleArray = (double *) calloc(36, sizeof (*doubleArray));
    this->MatrixXdToArray(sigma, &doubleArray);

    this->sendLink(utime_from, utime_to, mu, doubleArray, creator);

    free(doubleArray);
}

void
HauvClient::sendLinkPrior(int64_t utime_to,
                          double mu[6], 
                          const MatrixXd& sigma)
{
    HCINFO("Sending prior at %lu", utime_to);
    double *doubleArray = (double *) calloc(36, sizeof (*doubleArray));
    this->MatrixXdToArray(sigma, &doubleArray);

    this->sendLinkPrior(utime_to, mu, doubleArray);

    free(doubleArray);
}

void
HauvClient::sendLinkPartial(int64_t utime_to,
                            double mu[3],
                            double sigma[9])
{
    perllcm::isam_vlink_t vlink = this->getEmptyVlink();
    vlink.utime = timestamp_now();
    vlink.id2 = utime_to;
    vlink.n = 3;
    vlink.n2 = 9; //n squared
    vlink.link_type = perllcm::isam_vlink_t::LINK_ZPR_PARTIAL;
    vlink.sensor_id = perllcm::isam_vlink_t::SENSOR_ODOMETRY;

    std::copy(mu, mu+3, std::back_inserter(vlink.z));
    std::copy(sigma, sigma+9, std::back_inserter(vlink.R));

    m_lcm->publish(m_vlink_channel, &vlink);
}

void
HauvClient::sendLink(int64_t utime_from, 
                     int64_t utime_to, 
                     double mu[6], 
                     double sigma[36],
                     int32_t creator)
{

    perllcm::isam_vlink_t vlink = this->getEmptyVlink();
    vlink.utime = timestamp_now();
    vlink.id1 = utime_from;
    vlink.id2 = utime_to;
    vlink.n = 6;
    vlink.n2 = 36; //n squared
    vlink.link_type = perllcm::isam_vlink_t::LINK_POSE3D;
    vlink.sensor_id = perllcm::isam_vlink_t::SENSOR_ODOMETRY;
    vlink.creator_of_id2 = creator;

    std::copy(mu, mu+6, std::back_inserter(vlink.z));
    std::copy(sigma, sigma+36, std::back_inserter(vlink.R));

    m_lcm->publish(m_vlink_channel, &vlink);

}

void
HauvClient::sendLinkPrior(int64_t utime_to,
                          double mu[6], 
                          double sigma[36])
{
    perllcm::isam_vlink_t vlink = this->getEmptyVlink();
    vlink.id2 = utime_to;
    vlink.n = 6;
    vlink.n2 = 36; //n squared
    vlink.link_type = perllcm::isam_vlink_t::LINK_PRIOR;
    vlink.sensor_id = perllcm::isam_vlink_t::SENSOR_ODOMETRY;

    std::copy(mu, mu+6, std::back_inserter(vlink.z));
    std::copy(sigma, sigma+36, std::back_inserter(vlink.R));

    m_lcm->publish(m_vlink_channel, &vlink);
}

void
HauvClient::MatrixXdToArray(const MatrixXd& m, double **array)
{
    *array = (double *)malloc(m.rows() * m.cols() * sizeof(double));

    int ind = 0;
    for (int r=0; r<m.rows(); r++)
        for (int c=0; c<m.cols(); c++) {
            (*array)[ind] = m(r,c);
            ind++;
        }
  
}

void
HauvClient::setPrevPose(const State& state)
{
    double pose[6] = {state.x, state.y, state.z,
                      state.roll, state.pitch, state.heading};    
    memcpy(m_prev_pose, pose, 6*sizeof(*m_prev_pose));
}

perllcm::isam_vlink_t
HauvClient::getEmptyVlink()
{
    perllcm::isam_vlink_t vlink = {0};
    vlink.utime = timestamp_now();
    vlink.link_id = 0;
    vlink.creator_of_id2 = 0;
    vlink.sensor_id = 0;
    memset (vlink.x_vs1, 0, 6*sizeof (double));
    memset (vlink.x_vs2, 0, 6*sizeof (double));
    vlink.accept = true;
    vlink.accept_code = perllcm::isam_vlink_t::CODE_ACCEPTED;
    return vlink;
}

// WAYPOINT NAV
void
HauvClient::hauv_wp_save_t_callback (const ReceiveBuffer *rbuf,
                                     const std::string& channel,
                                     const hauv::wp_save_t *msg)
{
    //If waypoint_id is -1, then it's from a Delete Resume button click
    m_saved_waypoints [msg->waypoint_id] = msg->utime;
}

void 
HauvClient::hauv_wp_goto_request_t_callback (const lcm::ReceiveBuffer *rbuf,
                                             const std::string& channel,
                                             const hauv::wp_goto_request_t *msg)
{

    // on request goto msg, publish goto
    hauv::wp_goto_t g;
    g.mode = msg->mode;

    if (msg->mode == hauv::wp_goto_t::MODE_GOTO || msg->mode == hauv::wp_goto_t::MODE_GOTO_REL) {
        //Check that we actually have the waypoint
        if (m_saved_waypoints.count(msg->waypoint_id) == 0) return;
        g.utime_target = m_saved_waypoints[msg->waypoint_id];

        // local bookkeeping
        m_waypoint_goto_id = msg->waypoint_id;

        // save resume point
        save_resume_point ();
    }
    else if (msg->mode == hauv::wp_goto_t::MODE_RESUME) {
        g.utime_target = m_resume_waypoint;
        m_waypoint_goto_id = msg->waypoint_id;  // -1
        m_resume_waypoint = 0;
    }

    //@todo we should queue this up
    // command HAUV
    wpnav_cmd_vehicle (g.mode, g.utime_target);

    // publish goto message for viewer and other clients
    if (g.utime_target > 0) 
        m_lcm->publish(m_goto_channel, &g);
}

void
HauvClient::perllcm_isam_graph_vis_t_callback (const ReceiveBuffer *rbuf,
                                               const std::string& channel,
                                               const perllcm::isam_graph_vis_t *msg)
{
    m_isam_graph = *msg;
}

wpNode*
HauvClient::find_wpnode(int64_t utime)
{
    double min_dt = 10000.0;
    wpNode *cloesest_node = NULL;

    std::map<int64_t, wpNode*>::iterator it;
    for (it=m_wp_nodes.begin(); it!=m_wp_nodes.end(); it++) {
        int64_t node_time = it->first;
        int64_t dt = abs (node_time - utime);
        if (dt < min_dt) {
            min_dt = dt;
            cloesest_node = it->second;
        }
    }  

    return cloesest_node;
}

void 
HauvClient::save_wp_nodes (const State& state)
{
    wpNode *_wpnode = new wpNode;
    _wpnode->m_initial_cartesian[0] = state.x;
    _wpnode->m_initial_cartesian[1] = state.y;
    _wpnode->m_initial_cartesian[2] = state.z;
    _wpnode->m_initial_cartesian[3] = state.roll;
    _wpnode->m_initial_cartesian[4] = state.pitch;
    _wpnode->m_initial_cartesian[5] = state.heading;
    _wpnode->m_hull_relative[0] = state.rel_x;
    _wpnode->m_hull_relative[1] = state.rel_y;
    _wpnode->m_hull_relative[2] = state.rel_heading;

    m_wp_nodes[state.utime] = _wpnode;
}

// This used to be "goto_t_callback in hauv-viewer
void
HauvClient::wpnav_cmd_vehicle (int32_t mode, int64_t utime_target)
{
    // prepare message to viewer
    hauv::wp_goto_state_t goto_state;
    goto_state.waypoint_horizontal = 0;
    goto_state.waypoint_vertical = 0;
    goto_state.original_horizontal = 0;
    goto_state.original_vertical = 0;
    for (int i=0; i<6; i++) {goto_state.waypoint[i] = 0;}
    for (int i=0; i<6; i++) {goto_state.original[i] = 0;}

    // if switching from resume to off, correct the vehicle's belief of
    // where it is to where it should think it is in order to continue
    // the survey from where it actually left of
    if (m_waypoint.mode == hauv::wp_goto_t::MODE_RESUME && mode == hauv::wp_goto_t::MODE_OFF) {
        // how far are we from the target?
        double dh, dv;
        if (deltaToWaypoint (dh, dv) > 0) {
            // only if we are close to the target
            if (sqrt (dh*dh + dv*dv) < 3.) {
                // note: we correct the hull relative coordinates by the current
                // offset from the resume point
                wpNode* wpnode = find_wpnode (m_waypoint.utime_target);

                if (wpnode) {
                    double h = wpnode->m_hull_relative[1] - dh;
                    double v = wpnode->m_hull_relative[0] - dv;
                    
                    hauv::pl_rns_t rns;
                    rns.time = 0;
                    rns.x = h; // kaess: x/y is different from our internal x/y (tested on vehicle)
                    rns.y = v;
                    rns.bearing_offset = 0;
                    m_lcm->publish("HAUV_PL_RNS", &rns);
                }
                else {
                    HCINFO ("WARNING: Null wp node found. Skipping this command!\n"); 
                }
            }
        }
        else
            HCINFO ("WARNING: No isam node found for target utime. Skipping this command!\n"); 
    }

    // now process the actual mode
    if (mode == hauv::wp_goto_t::MODE_GOTO
        || mode == hauv::wp_goto_t::MODE_RESUME
        || mode == hauv::wp_goto_t::MODE_GOTO_REL) {
        m_waypoint.mode = mode;
        m_waypoint.utime = 0; // force immediate sending of new waypoint
        m_waypoint.utime_target = utime_target;
        // state will be sent to viewer by generateWaypoint()
    } else {
        m_waypoint.mode = mode;
        m_waypoint.utime = 0;
        m_waypoint.utime_target = 0;
        goto_state.mode = mode;
        m_lcm->publish (m_goto_state_channel, &goto_state);
    }

}

int 
HauvClient::deltaToWaypoint(double& dh, double& dv) 
{
    if (!m_active) return -1;

    // Cartesian estimate of target point
    int idx = this->utimeToIndex(m_waypoint.utime_target);
    if (idx < 0) return -1; // fail to find corresponding isam node

    vector<double> target_node = m_isam_graph.mu[idx];
    vector<double> current_node = m_isam_graph.mu[m_isam_graph.mu.size() - 1];

    // vector from current to target in 3D Cartesian
    vector<double> target_vector(3,0);
    for (int i=0; i<3; i++)
        target_vector[i] = target_node[i] - current_node[i];

    // for horizontal, omit the z component of the target vector, and
    // project the resulting vector onto a line through the vehicle
    // perpendicular to its x (forward) axis according to its
    // *Cartesian* estimate
    double theta = - current_node[5]; // yaw

    // dh = sin(theta)*target_vector.x() + cos(theta)*target_vector.y(); // to the right
    dh = sin(theta)*target_vector[0] + cos(theta)*target_vector[1]; // z

    // for vertical, project target vector onto plane given by its x
    // (forward) axis and the z axis according to its Cartesian estimate
    double down = target_vector[2]; // z

    // double in = cos(theta)*target_vector.x() + sin(theta)*target_vector.y();
    double in = cos(theta)*target_vector[0] + sin(theta)*target_vector[1];

    dv = sqrt(down*down + in*in);
    if ((down<0 && in<0) || (down<0 && -down>in) || (in<0 && -in>down)) {
        dv = -dv;
    }

    return 1;
}

void 
HauvClient::save_resume_point () 
{
    // only save if there is currently no resume point saved
    if (m_resume_waypoint == 0) {
        // Save the current position so we can later resume
        //m_resume_waypoint = m_traj.getLatest();
        m_resume_waypoint = m_isam_graph.node_id.back();
        // resume point has ID -1 (not available as a normal waypoint)
        hauv::wp_save_t s;
        s.utime = m_resume_waypoint;
        s.waypoint_id = -1;
        m_lcm->publish (m_wp_save_channel, &s);
    }
}

void
HauvClient::display_wpnav_status(int32_t mode, hauv::wp_goto_state_t *msg)
{
    if (mode != hauv::wp_goto_t::MODE_GOTO
        && mode != hauv::wp_goto_t::MODE_GOTO_REL
        && mode != hauv::wp_goto_t::MODE_RESUME)
    return;


    char str[1000];

    perllcm::isam_info_t entry;

    if (mode == hauv::wp_goto_t::MODE_GOTO) {
        sprintf(str, "goto wp%i: %.1f,%.1f (%.1f,%.1f)\n%.2f,%.2f,%.2f (%.2f,%.2f,%.2f)",
                m_waypoint_goto_id+1,
                msg->waypoint_horizontal, msg->waypoint_vertical,
                msg->original_horizontal, msg->original_vertical,
                msg->waypoint[0], msg->waypoint[1], msg->waypoint[2],
                msg->original[0], msg->original[1], msg->original[2]);

        entry.data = str;
    }
    else if (mode == hauv::wp_goto_t::MODE_GOTO_REL) {
        sprintf(str, "goto wp%i RELATIVE: %.1f,%.1f (%.1f,%.1f)",
	            m_waypoint_goto_id+1,
                msg->waypoint_horizontal, msg->waypoint_vertical,
                msg->original_horizontal, msg->original_vertical);
        entry.data = str;
    } 
    else if (mode == hauv::wp_goto_t::MODE_RESUME) {
        sprintf(str, "goto RESUME: %.1f,%.1f (%.1f,%.1f)\n%.2f,%.2f,%.2f (%.2f,%.2f,%.2f)",
                msg->waypoint_horizontal, msg->waypoint_vertical,
                msg->original_horizontal, msg->original_vertical,
                msg->waypoint[0], msg->waypoint[1], msg->waypoint[2],
                msg->original[0], msg->original[1], msg->original[2]);
        entry.data = str;
    }

    m_lcm->publish ("INFO", &entry);
}

void 
HauvClient::send_waypoint(double horizontal, double vertical) 
{
#if 0
    // sending commands through the java pilot (PilotWaypoint)
    hauv_vehicle_plan_t plan;
    plan.time = m_mapestimate->getLastTime();
    plan.stop = false; // our mode is broadcast
    plan.holdStation = false;
    plan.broadcast = true;
    plan.heading = false; // always false in hull-relative
    plan.headingOffset = 0.; // ignored
    plan.depth = 0.; // ignored
    plan.npoints = 2; // todo - should only be one waypoint, but problem on java side
    // strange convention on pilot side arising from surge=-vertical and
    // sway=horizontal
    double waypoints[2] = {-vertical, horizontal};
    double* wp[2] = {waypoints, waypoints};
    plan.waypoints = wp;
    hauv_vehicle_plan_t_publish(m_lcm, "HAUV_VEHICLE_PLAN", &plan);
#else
    // directly issue waypoint commands to the vehicle
    hauv::pl_sus_t sus;
    sus.time = 0;
    m_lcm->publish ("HAUV_PL_SUS", &sus);

    hauv::pl_ghp_t ghp;
    ghp.time = 0;
    ghp.horizontal = horizontal;
    ghp.vertical = vertical;
    ghp.distance = 1.5;
    m_lcm->publish ("HAUV_PL_GHP", &ghp);
#endif
}

// Frequently create a new waypoint to deal with the inconsistencies
// in the hull relative coordinate frame as well as drift and
// potential corrections along the way based on new loop
// closures. Close to the goal, the waypoint will be accurate, but at
// some distance there can be a significant error on a curved hull.
void 
HauvClient::generateWaypoint()
{

    // should be executed twice a second to make sure we keep vehicle control
    if (m_waypoint.mode != hauv::wp_goto_t::MODE_OFF
        && abs(m_waypoint.utime - m_traj.getLatest()) > int(1.0/ MAX_WP_CMD_HZ*1E6)) {

        //m_mapestimate->timing.tic("C generate waypoint");
        m_waypoint.utime = m_traj.getLatest();

        if (m_waypoint.mode == hauv::wp_goto_t::MODE_GOTO || m_waypoint.mode == hauv::wp_goto_t::MODE_RESUME) {
            double dh, dv;
            if (deltaToWaypoint(dh, dv) < 0) return;

            // obtain waypoint by adding current relative coordinates
            //      std::cout << "waypoint: delta for hull relative: dh=" << dh << " dv=" << dv << std::endl; 
            State s = m_traj.getState(m_traj.getLatest());
            double h = s.rel_y + dh;
            double v = s.rel_x + dv;
            send_waypoint(h, v);
         
            // send status to viewer
            hauv::wp_goto_state_t goto_state;
            goto_state.mode = m_waypoint.mode;
            // actual waypoint
            goto_state.waypoint_horizontal = h;
            goto_state.waypoint_vertical = v;
            // saved, incorrect waypoint
            State s_orig = m_traj.getState (m_waypoint.utime_target);
            goto_state.original_horizontal = s_orig.rel_y;
            goto_state.original_vertical = s_orig.rel_x;

            // actual Cartesian waypoint
            int idx = utimeToIndex(m_waypoint.utime_target);
            vector<double> target_node = m_isam_graph.mu[idx];
            double* wp = goto_state.waypoint;
            wp[0] = target_node[0];
            wp[1] = target_node[1];
            wp[2] = target_node[2];
            wp[3] = target_node[3];
            wp[4] = target_node[4];
            wp[5] = target_node[5];

            // original Cartesian waypoint before drift correction
            double *orig = goto_state.original;
            orig[0] = s_orig.x;
            orig[1] = s_orig.y;
            orig[2] = s_orig.z;
            orig[3] = s_orig.heading;
            orig[4] = s_orig.pitch;
            orig[5] = s_orig.roll;

            // send
            m_lcm->publish (m_goto_state_channel, &goto_state);
            display_wpnav_status(m_waypoint.mode, &goto_state);

        } else if (m_waypoint.mode == hauv::wp_goto_t::MODE_GOTO_REL) {

            wpNode *node = find_wpnode (m_waypoint.utime_target);
            double h = node->m_hull_relative[1];
            double v = node->m_hull_relative[0];
            send_waypoint(h, v);

            hauv::wp_goto_state_t goto_state;
            goto_state.mode = hauv::wp_goto_t::MODE_GOTO_REL;
            goto_state.waypoint_horizontal = h;
            goto_state.waypoint_vertical = v;
            goto_state.original_horizontal = h;
            goto_state.original_vertical = v;
            // no Cartesian involved, so we leave those entries 0
            for (int i=0; i<6; i++) goto_state.waypoint[i] = 0;
            for (int i=0; i<6; i++) goto_state.original[i] = 0;

            // send
            m_lcm->publish (m_goto_state_channel, &goto_state);
            display_wpnav_status(m_waypoint.mode, &goto_state);

        }
        //m_mapestimate->timing.toc("C generate waypoint");
    }
}

int
HauvClient::utimeToIndex(int64_t utime)
{
    //get the integer index for mu[]
    int64_t min_dt = HAUV_STEP_SIZE*1E6;   // should be less than a sec
    int min_idx = -1;

    for (size_t idx = 0; idx < m_isam_graph.mu.size(); idx++) {
        int64_t dt = fabs (m_isam_graph.node_id[idx] - m_waypoint.utime_target);
        
        if (dt < min_dt) {
            //std::cout << "dt = " << abs(m_isam_graph.node_id[idx] - m_waypoint.utime_target) << std::endl;
            min_dt = dt;
            min_idx = idx;
        }
    }
 
    return min_idx;
}
