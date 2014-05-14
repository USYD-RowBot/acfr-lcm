#ifndef _HAUV_HAUVCLIENT_H
#define _HAUV_HAUVCLIENT_H

#include <list>
#include <map>
#include <string>
#include <Eigen/Core>
#include <lcm/lcm-cpp.hpp>

#include "perls-common/units.h"
#include "perls-common/bot_util.h"

#include "perls-lcmtypes++/perllcm/isam_vlink_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/hauv/bs_rnv_2_t.hpp"
#include "perls-lcmtypes++/hauv/bs_cnv_t.hpp"
#include "perls-lcmtypes++/hauv/bs_dvl_2_t.hpp"
#include "perls-lcmtypes++/hauv/bs_pit_t.hpp"

#include "perls-lcmtypes++/perllcm/isam_add_node_t.hpp"

#include "perls-lcmtypes++/hauv/wp_save_t.hpp"
#include "perls-lcmtypes++/hauv/wp_goto_request_t.hpp"
#include "perls-lcmtypes++/hauv/wp_goto_t.hpp"
#include "perls-lcmtypes++/hauv/wp_goto_state_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_cmd_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_graph_vis_t.hpp"

#include "perls-hauv/vehicletrajectory.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)

#define HAUV_STEP_SIZE 1.0
#define HAUV_UTIME_OFFSET 1e5 //0.1 seconds

#define HCINFO(format, ...) \
    fprintf (stdout, "[hauv-client (info)]\t" format "\n", ## __VA_ARGS__)

#define HCWARN(format, ...) \
    fprintf (stderr, "[hauv-client (WARN)]\t" format "\n", ## __VA_ARGS__)

class Waypoint {
public:
    Waypoint() : utime(0), mode(hauv::wp_goto_t::MODE_OFF), utime_target(0) {}

    int64_t utime; // for timing of waypoint commands
    int32_t mode;
    int64_t utime_target;
};

class wpNode {
public:
    // initial cartesian and hull relative coordinate
    double  m_initial_cartesian[6]; // pose3d
    double  m_hull_relative[3];     // pose2d
};

class HauvClient
{
  public:
    HauvClient(lcm::LCM& lcm);
    ~HauvClient();
    void run();
    void perllcm_isam_cmd_t_callback (const lcm::ReceiveBuffer *rbuf,
                                      const std::string& channel,
                                      const perllcm::isam_cmd_t *msg);
    void hauv_bs_rnv_2_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                   const std::string& channel, 
                                   const hauv::bs_rnv_2_t *msg);
    void hauv_bs_pit_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                 const std::string& channel, 
                                 const hauv::bs_pit_t *msg);
    void hauv_bs_cnv_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                 const std::string& channel, 
                                 const hauv::bs_cnv_t *msg);
    void hauv_bs_dvl_2_t_callback (const lcm::ReceiveBuffer *rbuf,
                                   const std::string& channel, 
                                   const hauv::bs_dvl_2_t *msg);
    void perllcm_heartbeat_t_2_callback (const lcm::ReceiveBuffer *rbuf,
                                         const std::string& channel, 
                                         const perllcm::heartbeat_t *msg);
    void perllcm_heartbeat_t_5_callback (const lcm::ReceiveBuffer *rbuf,
                                         const std::string& channel, 
                                         const perllcm::heartbeat_t *msg);
    void perllcm_rtvan_add_node_callback(const lcm::ReceiveBuffer *rbuf,
                                         const std::string& channel, 
                                         const perllcm::heartbeat_t *msg);
    void perllcm_sonar_add_node_callback(const lcm::ReceiveBuffer *rbuf, 
                                         const std::string& channel,
                                         const perllcm::heartbeat_t *msg);
    void perllcm_self_add_node_callback(const lcm::ReceiveBuffer *rbuf,
                                        const std::string& channel, 
                                        const perllcm::heartbeat_t *msg);
    void hauv_wp_save_t_callback (const lcm::ReceiveBuffer *rbuf,
                                     const std::string& channel,
                                     const hauv::wp_save_t *msg);
    void hauv_wp_goto_request_t_callback (const lcm::ReceiveBuffer *rbuf,
                                          const std::string& channel,
                                          const hauv::wp_goto_request_t *msg);
    void perllcm_isam_graph_vis_t_callback (const lcm::ReceiveBuffer *rbuf,
                                            const std::string& channel,
                                            const perllcm::isam_graph_vis_t *msg);

    // slam channel
    char *m_vlink_channel, *m_rtvan_add_node_channel, *m_sonar_add_node_channel, *m_self_add_node_channel, *m_graph_vis_channel;
    
    // user interface
    char *m_isam_cmd_channel;

    // waypoint navigation channel
    char *m_goto_channel, *m_goto_state_channel, *m_rq_goto_channel, *m_wp_list_channel, *m_wp_save_channel;

  private:
    lcm::LCM *m_lcm;
    VehicleTrajectory   m_traj;
    BotParam *m_param;

    // process noise on odometry
    const static double odo_std_xyz = 0.005;        //  5 mm/s
    const static double odo_std_rph = 0.0230*DTOR;  // 80 deg/hr

    // observation noise on observables (for roll pitch depth abs measurement)
    const static double gbl_std_r = 0.1*DTOR;
    const static double gbl_std_p = 0.1*DTOR;
    const static double gbl_std_z = 0.01; // rads & m

    bool m_active;
    bool m_initialized_prior, m_start_adding_nodes;
    bool m_has_rnv, m_has_cnv;
    int64_t m_prev_node_utime;
    double m_origin[6]; //xyzrph
    double m_prev_pose[6]; //xyzrph
    std::list<perllcm::isam_vlink_t*> m_new_nodes;
    std::map<int64_t, perllcm::heartbeat_t*> m_nodes;
    perllcm::isam_graph_vis_t m_isam_graph;

    void update();
    void queueNode(const perllcm::heartbeat_t *msg, int creator);
    void addNode(int64_t utime);

    void sendLink(int64_t utime_from, int64_t utime_to, double mu[6], double sigma[36], int32_t creator);
    void sendLink(int64_t utime_from, int64_t utime_to, double mu[6], const Eigen::MatrixXd& sigma, int32_t creator);
    void sendLinkPartial(int64_t utime_to, double mu[3], double sigma[9]);
    void sendLinkPartial(int64_t utime_to, double mu[3], const Eigen::MatrixXd& sigma);
    void sendLinkPrior(int64_t utime_to, double mu[6], double sigma[36]);
    void sendLinkPrior(int64_t utime_to, double mu[6], const Eigen::MatrixXd& sigma);

    void init_slam(uint64_t utime);
    void initialize(const State& s);
    void publishDeltas(const State& s, const int32_t& creator);
    void interpolate(State& prev_state, double& dt, const int32_t& creator);

    void MatrixXdToArray(const Eigen::MatrixXd &m, double **array);
    void printSixDof(double *d);
    void setPrevPose(const State& s);
    perllcm::isam_vlink_t getEmptyVlink();

    /// Saved waypoints indexed by waypoint id. wp = [id, utime]
    //  the second value is the timestamp for the waypoint.
    std::map<int64_t, wpNode*> m_wp_nodes;
    Waypoint                   m_waypoint;
    int64_t                    m_resume_waypoint;
    std::map<int32_t, int64_t> m_saved_waypoints;
    int32_t                    m_waypoint_goto_id;

    wpNode* find_wpnode (int64_t utime);
    void save_wp_nodes (const State& state);
    void broadcast_wp();
    void save_resume_point ();
    void display_wpnav_status(int32_t mode, hauv::wp_goto_state_t *msg);
    void generateWaypoint();
    void send_waypoint(double horizontal, double vertical);
    int  deltaToWaypoint (double& dh, double& dv);
    void wpnav_cmd_vehicle (int32_t mode, int64_t utime_target);
    int utimeToIndex (int64_t utime);
};


#endif
