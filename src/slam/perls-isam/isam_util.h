#ifndef __ISAM_UTIL_H__
#define __ISAM_UTIL_H__

#include <map>
#include <stdexcept>

#include <isam/isam.h>
#include <Eigen/Core>
#include <glib.h>

#include "perls-common/bot_util.h"

#include "perls-lcmtypes/perllcm_isam_init_t.h"
#include "perls-lcmtypes/perllcm_isam_add_node_t.h"
#include "perls-lcmtypes/perllcm_isam_add_node_ack_t.h"
#include "perls-lcmtypes/perllcm_isam_vlink_t.h"
#include "perls-lcmtypes/perllcm_isam_glc_factor_t.h"
#include "perls-lcmtypes/perllcm_pose3d_collection_t.h"

#include "user_nodes.h"
#include "user_factors.h"

// conversion between rph to hpr
Eigen::MatrixXd          isamu_van2isam_eigen           (Eigen::MatrixXd in);
Eigen::MatrixXd          isamu_isam2van_eigen           (Eigen::MatrixXd in);
isam::Pose3d             isamu_van2isam_pose3d          (const double z[6]);
isam::Pose3db            isamu_van2isam_pose3db         (const double z[5]);
Eigen::MatrixXd          isamu_van2isam_sqrtinf3d       (const double R[36]);
Eigen::MatrixXd          isamu_van2isam_sqrtinf3db      (const double R[25]);
isam::Plane3d            isamu_van2isam_plane3d         (const double z[6]);
isam::Plane3dMeasurement isamu_van2isam_plane3d_meas    (const double z[4]);
Eigen::MatrixXd          isamu_van2isam_plane3d_sqrtinf (const double R[16]);

void isamu_isam2van_pose3d     (isam::Pose3d p, double z[6]);
void isamu_isam2van_pose3db    (isam::Pose3db p, double z[5]);
void isamu_isam2van_plane3d    (isam::Plane3d p, double z[4]);
void isamu_isam2van_plane3d_meas    (isam::Plane3dMeasurement p, double z[4]);
void isamu_isam2van_sqrtinf3d  (Eigen::MatrixXd sqrtinf, double R[36]);
void isamu_isam2van_sqrtinf3db (Eigen::MatrixXd sqrtinf, double R[25]);
void isamu_isam2van_plane3d_meas_sqrtinf (Eigen::MatrixXd sqrtinf, double R[isam::Plane3dMeasurement::dim*isam::Plane3dMeasurement::dim]);


// isam interface
isam::Properties isamu_init_slam (const perllcm_isam_init_t *init_t);
isam::Properties isamu_init_slam (BotParam *param);

isam::Pose3d_Node*
isamu_add_node_pose3d (const perllcm_isam_add_node_t *msg, isam::Slam *m_slam);

isam::Plane3d_Node*
isamu_add_node_plane3d (const perllcm_isam_add_node_t *msg, isam::Slam *m_slam);

void
isamu_add_factor (const perllcm_isam_vlink_t *vlink, isam::Slam *m_slam, 
                  isam::Node* node1, isam::Node* node2);

void
isamu_add_glc_factor (const perllcm_isam_glc_factor_t *glc_factor,
                      isam::Slam *m_slam, std::vector<isam::Pose3d_Node*> p3d_nodes);

void
isamu_add_partial (const perllcm_isam_vlink_t *vlink, isam::Slam *slam, 
                   isam::Pose3d_Node* node, bool print_status);

// isam state fprintf / publish
void
isamu_state_fprintf (const char *filename, const perllcm_pose3d_collection_t *pc, GList *utimelist);

void
isamu_cputime_fprintf (const char *filename, std::map<int64_t, int64_t> cputime);

Eigen::MatrixXd
isamu_get_weighted_jacobian (isam::Factor *f);

Eigen::MatrixXd
isamu_get_jacobian (isam::Factor *f);

Eigen::MatrixXd
isamu_conditional_information_recovery (isam::Slam *slam, std::vector<int> inds);

void
isamu_root_shift (Eigen::VectorXd &x, Eigen::MatrixXd *J, std::vector<isam::Node*> nodes,
                  isam::Selector s, bool inc_x_o_w); 

#endif //__ISAM_UTIL__

