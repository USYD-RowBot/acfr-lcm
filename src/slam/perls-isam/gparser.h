#ifndef __ISAM_GPARSER_H__
#define __ISAM_GPARSER_H__

#include <stdexcept>

#include <glib.h>
#include <isam/isam.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_isam_vlink_t.h"
#include "perls-lcmtypes/perllcm_isam_add_node_t.h"
#include "perls-lcmtypes/perllcm_isam_glc_factor_t.h"

#include "user_nodes.h"
#include "user_factors.h"

#define str_comment_symbol "#"
#define str_pose_prior  "Pose3d_Factor"             // initial pose
#define str_node        "Pose3d_Node"               // nodes
#define str_odo         "Pose3d_Pose3d_Factor"      // odometry
#define str_cam         "Pose3db_Pose3db_Factor"    // camera measurement
#define str_sonar       "Sonar2d_Factor"            // sonar measurement
#define str_zpr         "Pose3dPartial_Factor"      // roll pitch depth abs meas
#define str_xyz         "Pose3d_xyz_Factor"
#define str_h           "Pose3d_h_Factor"
#define str_rp          "Pose3d_rp_Factor"
#define str_z           "Pose3d_z_Factor"
#define str_xy          "Pose3d_xy_Factor"
#define str_glc         "GLC_Factor"
#define str_glc_rs      "GLC_RS_Factor"
#define str_plane_prior  "Plane3d_Factor"
#define str_plane_factor "Pose3d_Plane3d_Factor"
#define str_plane_node   "Plane3d_Node"

using namespace isam;
using namespace std;


typedef enum _queue_element_type_t
{
    QE_VLINK = 0,
    QE_NODE = 1,
    QE_CMD = 2,
    QE_STATE = 3,
    QE_INIT = 4,
    QE_GLC = 5,
} queue_element_type_t;

typedef struct _queue_element_t queue_element_t;
struct _queue_element_t
{    
    int64_t utime;
    queue_element_type_t type;
    void *msg;
};

    
// @TODO: name should be changed for odo_factor. laser will use the same factor but not sequential!
class gParser {
  public:
    gParser (lcm_t *lcm_export, std::string filename) {
        m_inFile.open(filename.c_str());
        if (m_inFile.is_open())
            std::cout << "Loaded [" << filename.c_str() << "] to parse" << std::endl;
        else
            throw runtime_error ("Unable to open file");

        // hand a pointer to the server's lcm
        m_lcm = lcm_export;
        m_done = false;
        m_gq = NULL;
    }

    ~gParser () {
    }

    bool fread_graph (void);
    
    bool is_done (void) {
        return m_done;
    }

    void set_gq (GAsyncQueue *_gq) {
        m_gq = _gq;
    }
    
  private:  
    //data
    std::ifstream   m_inFile;
    lcm_t *m_lcm;

    bool m_done;

    // handle a line (parser / get / return)
    void _handleLine (const std::string line);

    // parse & publish a factor
    perllcm_isam_add_node_t *   _handle_node           (const std::string line);
    perllcm_isam_vlink_t *      _handle_prior          (const std::string line);
    perllcm_isam_vlink_t *      _handle_odo_factor     (const std::string line);
    perllcm_isam_vlink_t *      _handle_cam_factor     (const std::string line);
    perllcm_isam_vlink_t *      _handle_sonar_factor   (const std::string line);
    perllcm_isam_vlink_t *      _handle_1_tuple_factor (const std::string line, int32_t link_type);
    perllcm_isam_vlink_t *      _handle_2_tuple_factor (const std::string line, int32_t link_type);
    perllcm_isam_vlink_t *      _handle_3_tuple_factor (const std::string line, int32_t link_type);
    perllcm_isam_glc_factor_t * _handle_glc_factor     (const std::string line, bool is_root_shift);
    perllcm_isam_vlink_t *      _handle_plane_prior    (const std::string line);
    perllcm_isam_vlink_t *      _handle_plane_factor   (const std::string line);
    perllcm_isam_add_node_t *   _handle_plane_node     (const string line);

    // line header check: if isam provides factor name in public, this can be removed.
    bool    _is_commented_line         (const std::string line) {return line.find (str_comment_symbol) == 0;}
    bool    _is_pose3d_prior_line      (const std::string line) {return line.find (str_pose_prior) == 0;}
    bool    _is_node_line              (const std::string line) {return line.find (str_node) == 0;}
    bool    _is_odo_factor_line        (const std::string line) {return line.find (str_odo) == 0;}
    bool    _is_cam_factor_line        (const std::string line) {return line.find (str_cam) == 0;}
    bool    _is_sonar_factor_line      (const std::string line) {return line.find (str_sonar) == 0;}
    bool    _is_zpr_factor_line        (const std::string line) {return line.find (str_zpr) == 0;}
    bool    _is_xyz_factor_line        (const std::string line) {return line.find (str_xyz) == 0;}
    bool    _is_h_factor_line          (const std::string line) {return line.find (str_h) == 0;}
    bool    _is_rp_factor_line         (const std::string line) {return line.find (str_rp) == 0;}
    bool    _is_z_factor_line          (const std::string line) {return line.find (str_z) == 0;}
    bool    _is_xy_factor_line         (const std::string line) {return line.find (str_xy) == 0;}
    bool    _is_glc_factor_line        (const std::string line) {return line.find (str_glc) == 0;}
    bool    _is_glc_rs_factor_line     (const std::string line) {return line.find (str_glc_rs) == 0;}
    bool    _is_plane_prior_line       (const std::string line) {return line.find (str_plane_prior) == 0;}
    bool    _is_plane_factor_line      (const std::string line) {return line.find (str_plane_factor) == 0;}
    bool    _is_plane_node_line        (const std::string line) {return line.find (str_plane_node) == 0;}
    
    // isam graph line parsers
    void    _parse_utime (std::string str_idxed, int64_t &idx_i, int64_t &idx_j);
    void    _parse_prior (std::string str, int64_t &idx, isam::Pose3d &delta, Eigen::MatrixXd &sqrtinf, int64_t &utime);
    void    _parse_plane_prior (std::string str, int64_t &idx, isam::Plane3d &delta, Eigen::MatrixXd &sqrtinf, int64_t &utime);
    void    _parse_node (std::string str, int64_t &idx, isam::Pose3d &mu, int64_t &utime, double &offset, bool &has_mu);
    void    _parse_odo_factor (std::string str, int64_t &idx_i, int64_t &idx_j, 
                               isam::Pose3d &delta, Eigen::MatrixXd &sqrtinf,
                               int64_t &utime1, int64_t &utime2);
    void    _parse_cam_factor (std::string str, int64_t &idx_i, int64_t &idx_j,
                               isam::Pose3db &delta, Eigen::MatrixXd &sqrtinf,
                               isam::Pose3d &x_v1c, isam::Pose3d &x_v2c,
                               int64_t &utime1, int64_t &utime2);
    void    _parse_sonar_factor (std::string str, int64_t &idx_i, int64_t &idx_j,
                                 isam::Pose2d &delta, Eigen::MatrixXd &sqrtinf,
                                 isam::Pose3d &x_v1c, isam::Pose3d &x_v2c,
                                 int64_t &utime1, int64_t &utime2);
    void    _parse_plane_factor (std::string str, int64_t &idx_i, int64_t &idx_j,
                                 isam::Plane3dMeasurement &delta, Eigen::MatrixXd &sqrtinf,
                                 int64_t &utime1, int64_t &utime2);

    void    _parse_1_tuple_factor (std::string str, int64_t &idx,
                                   double &delta, double &sqrtinf,
                                   int64_t &utime);
    void    _parse_2_tuple_factor (std::string str, int64_t &idx,
                                   double delta[2], Eigen::MatrixXd &sqrtinf,
                                   int64_t &utime);
    void    _parse_3_tuple_factor (std::string str, int64_t &idx,
                                   double delta[3], Eigen::MatrixXd &sqrtinf,
                                   int64_t &utime);

    // it is just a pointer handed from isamServer
    GAsyncQueue *m_gq;    // glib asynchronous queue for nodes and vlinks
};

#endif // __ISAM_GPARSER_H__
