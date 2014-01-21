#ifndef PERLS_SEG_GRAPH_MANAGER_H
#define PERLS_SEG_GRAPH_MANAGER_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include <map>
#include <deque>

#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_sf_erf.h>

#include "perls-math/gsl_util.h"
#include "perls-math/ssc.h"

#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/error.h"
#include "perls-common/units.h"

#include "perls-lcmtypes++/perllcm/segway_navigator_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/perllcm/position_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_plink_collection_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_plink_t.hpp"
#include "perls-lcmtypes++/perllcm/pose3d_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_vlink_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_request_state_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_return_state_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/ms_gx3_25_t.hpp"
#include "perls-lcmtypes++/senlcm/ms_gx3_t.hpp" 

#define DTOR UNITS_DEGREE_TO_RADIAN
#define RTOD UNITS_RADIAN_TO_DEGREE

#define GPS_QUEUE_LEN 10
#define MS_QUEUE_LEN 200


using namespace std;

static inline void
relative_pose (double x_ji[6], double S_ji[6*6],
               const double x_wi[6], const double x_wj[6],
               const double S_iijj[12*12]) {

    // tail to tail operation
    double x_ij[6] = {0};
    GSLU_MATRIX_VIEW (Jt2t, 6, 12);
    ssc_tail2tail (x_ij, Jt2t.matrix.data, x_wi, x_wj);

    // find covariance ij
    GSLU_MATRIX_VIEW (Sigma_ij, 6, 6);
    gsl_matrix_const_view S_iijj_v = gsl_matrix_const_view_array (S_iijj, 12, 12);
    gslu_blas_mmmT(&Sigma_ij.matrix, &Jt2t.matrix, &S_iijj_v.matrix, &Jt2t.matrix, NULL);
    
    GSLU_MATRIX_VIEW (Jinv, 6, 6);
    ssc_inverse (x_ji, Jinv.matrix.data, x_ij);
    
    // find covariance ji
    gsl_matrix_view S_ji_v = gsl_matrix_view_array (S_ji, 6, 6);
    gslu_blas_mmmT(&S_ji_v.matrix, &Jinv.matrix, &Sigma_ij.matrix, &Jinv.matrix, NULL);
}

static inline double
prob_in_roc (double mu, double vr, double sigma, bool angular = 0) {
    
    if (angular) {
        mu = bot_mod2pi (mu);
    }
    double numa = (vr - mu);
    double numb = (-vr - mu);
    // dont do this it works if it is unwraped
    //if (angular) {
    //    numa = bot_mod2pi_ref (mu, numa);
    //    numb = bot_mod2pi_ref (mu, numb);
    //}
    double dnom = sqrt(2)*sigma;
    double a = gsl_sf_erf ( numa / dnom);
    double b = gsl_sf_erf ( numb / dnom);
    return 0.5 * (a - b);

}


class DropDSAck {
    private:
        lcm::LCM lcm;
        bool data_ready;
        char  *channel_drop_ds;
        char  *channel_drop_scan;
        char  *channel_drop_ds_ack;
               
        void
        seg_nav_drop_ds_ack_cb (const lcm::ReceiveBuffer *rbuf,
                                const std::string &chan,
                                const perllcm::segway_navigator_t *msg) {
            
            if (0 == msg->state_len) {
                data_ready = 0;
                return;
            }
            
            // pull poses out of
            double x_w_p1[6] = {(-1 == msg->index.x) ? 0 : msg->mu[msg->index.x],
                                (-1 == msg->index.y) ? 0 : msg->mu[msg->index.y],
                                (-1 == msg->index.z) ? 0 : msg->mu[msg->index.z],
                                (-1 == msg->index.r) ? 0 : msg->mu[msg->index.r],
                                (-1 == msg->index.p) ? 0 : msg->mu[msg->index.p],
                                (-1 == msg->index.h) ? 0 : msg->mu[msg->index.h]};
                                
            memcpy (mu_new, x_w_p1, sizeof (double) * 6);
                                
            double x_w_p2[6] = {(-1 == msg->index.x) ? 0 : msg->mu[msg->index.x + msg->delayed_state_len],
                                (-1 == msg->index.y) ? 0 : msg->mu[msg->index.y + msg->delayed_state_len],
                                (-1 == msg->index.z) ? 0 : msg->mu[msg->index.z + msg->delayed_state_len],
                                (-1 == msg->index.r) ? 0 : msg->mu[msg->index.r + msg->delayed_state_len],
                                (-1 == msg->index.p) ? 0 : msg->mu[msg->index.p + msg->delayed_state_len],
                                (-1 == msg->index.h) ? 0 : msg->mu[msg->index.h + msg->delayed_state_len]};
            
            int inds[6] = {msg->index.x, msg->index.y, msg->index.z,
                              msg->index.r, msg->index.p, msg->index.h};
            
            GSLU_MATRIX_VIEW (Sigma_w, 12, 12, {0});                   
            for (int i=0;  i<12; i++) {
                for (int j=0;  j<12; j++) {
                    if (inds[i%6] == -1 || inds[j%6] == -1) {
                        gsl_matrix_set (&Sigma_w.matrix, i, j, 0);
                    } else {
                        int ii = 0;
                        if (i >= 6)
                            ii += (inds[i%6] + msg->delayed_state_len) * msg->state_len;
                        else
                            ii += inds[i%6] * msg->state_len;
                        if (j >= 6)
                            ii += (inds[j%6] + msg->delayed_state_len);
                        else
                            ii += inds[j%6];
                        gsl_matrix_set (&Sigma_w.matrix, i, j, msg->Sigma[ii]);
                    }
                }
            }
            // right now z isn't tracked in the navigation filter
            if (msg->index.z == -1) {
                // find displacement in xy, we know displacment in z is less that 1/3 of that due to max slope of 20 deg
                double tmp[6] = {0};
                ssc_tail2tail (tmp, NULL, x_w_p1, x_w_p2);
                double disp = sqrt (tmp[0]*tmp[0] + tmp[1]*tmp[1]);
// for most cases this is way too extreme
//double var_z = pow (disp*sin(20*DTOR)/3.0, 2);
double var_z = pow (disp*sin(0.2*DTOR)/3.0, 2);
                gsl_matrix_set (&Sigma_w.matrix, 2, 2, var_z);
                // given z_t+1 = z_t + dt*var_z
                //double dt = (msg->delayed_states_utime[0] - msg->delayed_states_utime[1])/1e6;
                //double var_z = 1.0/9.0;   // 1/3 m/s std dev for z velocity noise
                //gsl_matrix_set (&Sigma_w.matrix, 2, 2, var_z*dt);    
            } else {
                ERROR ("update seg-graph-manager to if z is now part of seg-navigator filter");
            }
            
            relative_pose (mu_odo, Sigma_odo, x_w_p1, x_w_p2, Sigma_w.data);
                           
            data_ready = 1;            
            
        }
        
    public:
        
        double mu_new[6];                 // new node mu
        double Sigma_new[6*6];            // new node Sigma
        double mu_odo[6];                 // odometry mu old node to new
        double Sigma_odo[6*6];            // odometry Sigma old node to new
        
        DropDSAck (char  *channel_drop_ds_in, char  *channel_drop_ds_ack_in) {
            channel_drop_ds = channel_drop_ds_in;
            channel_drop_ds_ack = channel_drop_ds_ack_in;
        };
        
        int
        drop_ds (int64_t utime) {
            
            // subscribe to call back
            lcm.subscribe(channel_drop_ds_ack, &DropDSAck::seg_nav_drop_ds_ack_cb, this);
            
            // send the message to navigator
            perllcm::heartbeat_t drop_ds;
            drop_ds.utime = utime;
            lcm.publish (channel_drop_ds, &drop_ds);  
            
            // wait for response
            struct timeval to;
            to.tv_sec = 0;
            to.tv_usec = 1e5;
            //int ret = lcm.handle();
            int ret = lcmu_handle_timeout (lcm.getUnderlyingLCM(), &to);
            if (ret > 0 && data_ready) {
                return 1;
            } else {
                return 0;
            }
        };
};

class GraphNode {
    private:
        
        
    public:
        int64_t utime;              // set by the image associated with this pose
        double pose[6];
        double odometry[6];
        double odometry_cov[6*6];
        
        GraphNode (int64_t utime_in) {
            utime = utime_in;
        };
    
        ~GraphNode () {
            
        };
        
        void
        set_pose (double pose_in[6]) {
            memcpy (pose, pose_in, 6*sizeof (double));
        }
        
        void
        set_odometry (double odometry_in[6]) {
            memcpy (odometry, odometry_in, 6*sizeof (double));
        }
        
        void
        set_odometry_cov (double odometry_cov_in[6*6]) {
            memcpy (odometry_cov, odometry_cov_in, 6*6*sizeof (double));
        }
    
};

class SegGraphManager {
    private:
        lcm::LCM lcm;
        
        BotParam *param;
        
        char  *channel_drop_ds;
        char  *channel_drop_ds_ack;
        char  *channel_seg_navigator;
        char  *channel_lb3_imagesync;
        char  *channel_drop_scan;
        char  *channel_laser_plink;
        char  *channel_isam_vlink;
        char  *channel_isam_request_state;
        char  *channel_isam_return_state;
        char  *channel_gps;
        char  *channel_ms_25;
        char  *channel_ms;
        
        double x_vs_gpsd[6];
        double x_vs_ms[6];
        double x_vs_ms_25[6];
        double x_lr_ms[6];
        double x_lr_ms_25[6];
        std::deque< senlcm::gpsd3_t >       gps_queue;
        std::deque< senlcm::ms_gx3_25_t >   ms_25_queue;
        std::deque< senlcm::ms_gx3_t >      ms_queue;
        
        int world_frame;  // should we init at 0,0,0,0,0,0 or in world frame based on gps and magnetic heading
        int full_3d;      // should we use full 3D or 2.5D (assumes z=0)
        
        double drop_node_dtrans;
        double drop_node_dyaw ;         
        
        int max_laser_plinks;
        double laser_plink_trans_roc;
        double laser_plink_yaw_roc;
        double p_roc_thresh;
        
        std::vector<GraphNode*> graph_nodes;                      // local pose history
        
        std::map<int64_t, std::vector<double> > block_diag_cov;    // local cache of graph covariance    
        
        double last_nav_pose[6];
        double last_node_pose[6];
        
        BotGPSLinearize *llxy;  //struct containing gps origin used to convert between local xy
        double org_alt;
        
        double R_gps[3*3];
        double R_ms_h;
        double R_ms_rp[2*2];
        
        double R_scan_match[6*6];
        
        // lcm callbacks
        void
        segway_navigator_t_cb (const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const perllcm::segway_navigator_t *msg);
        
        void
        lb3_imagesync_cb (const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const perllcm::heartbeat_t *msg);
        
        void
        isam_return_state_cb (const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const perllcm::isam_return_state_t *msg);
        
        void
        gps_cb (const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const senlcm::gpsd3_t *msg);
        void
        ms_25_cb (const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const senlcm::ms_gx3_25_t *msg);
        void
        ms_cb (const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const senlcm::ms_gx3_t *msg);
        
        int
        get_wr_pos (int64_t node_utime, double x_wv_out[6]);
        
        void
        pub_parfac (int64_t node_utime, double mu[6], double Sigma[6*6]);
        
        int
        find_close_ms (int64_t node_utime);
        
        int
        find_close_ms_25 (int64_t node_utime);

        int
        find_close_gps (int64_t node_utime);
        
        int 
        check_drop_pose (void);
        
        void
        propose_laser_links (const perllcm::isam_return_state_t *msg);
        
        // request state for all nodes
        void
        request_state (int16_t state_type);
        // request state for a subset of nodes
        void
        request_state (int16_t state_type, std::vector<int64_t> utimes);
        
        
        perllcm::isam_vlink_t
        get_empty_vlink (void) {
            perllcm::isam_vlink_t vlink = {0};
            vlink.utime = timestamp_now();
            vlink.link_id = 0;
            vlink.sensor_id = 0;
            memset (vlink.x_vs1, 0, 6*sizeof (double));
            memset (vlink.x_vs2, 0, 6*sizeof (double));
            vlink.accept = true;
            vlink.accept_code = perllcm::isam_vlink_t::CODE_ACCEPTED;
            return vlink;
        }
        
    public:
        
        bool is_done;      

        SegGraphManager ();
        ~SegGraphManager ();
        
        void
        run ();
        
};


#endif /* PERLS_SEG_GRAPH_MANAGER_H */
