#ifndef __LOCAL_VEHICLE_H__
#define __LOCAL_VEHICLE_H__

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>

#include "perls-common/bot_util.h"

#include "perls-lcmtypes++/senlcm/dstar_ssp1_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/rph_t.hpp"
#include "perls-lcmtypes++/senlcm/rdi_pd4_t.hpp"

#include "eigen_utils.h"
#include "perls-owtt-nav/estimator_kalman.h"
#include "perls-owtt-nav/estimator_info.h"
#include "perls-owtt-nav/index.h"
#include "perls-owtt-nav/meas_queue.h"
#include "perls-owtt-nav/process_model.h"
#include "perls-owtt-nav/utils.h"

using perls::operator<<;

class Local_manager
{
  public:
    Local_manager (BotParam *param, int64_t lag_usecs);
    virtual ~Local_manager () 
    {
        delete _estimator;
        delete _pm;
        delete _llxy;
    }

    void do_work () {_lcm.handle ();}

  protected:
    lcm::LCM _lcm; 
    std::string _chan_prefix;

    perls::Index _index;
    perls::Process_model *_pm;
    perls::If *_estimator;

    // user initializes filter 
    virtual void init_est (BotParam *param) = 0;
    //user calls this after initilizing filter AND adding any desired
    //non-local meas callbacks, e.g., toa/tol callbacks
    void add_local_callbacks (BotParam *param);

    // this really only works if we have 0 lag in the meas queue --- just fine
    // for the server
    void pred_aug_to_time (int64_t utime, const std::string& key);

    BotGPSLinearize *_llxy;
    double _depth;

    double _sig_uv;
    double _sig_heading;
    double _sig_gps;

    perls::Meas_queue _mqueue;

  private:
    void meas_callback (perls::Meas_ptr m);

    // static coordinate xforms
    Eigen::Matrix<double,6,1> _x_lr;
    Eigen::Matrix<double,6,1> _x_vs_dvl; 
    Eigen::Matrix<double,6,1> _x_vs_rph; 
    Eigen::Matrix<double,6,1> _x_vs_gps; 
    
    // current attitude
    Eigen::Matrix<double,6,1> _x_lv_rph;

    // lagged attitude/velocity measurement
    Eigen::Matrix<double,6,1> _uvwrph;

    // sensor switchyard
    bool _gps;
    bool _gps_odo;

    void dstar_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::dstar_ssp1_t *msg);
    void gpsd_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::gpsd3_t *msg);
    void rph_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::rph_t *msg);
    void rdi_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::rdi_pd4_t *msg);

    void process_lagged_measurements ();

    // should do this better --- opos only from the client
    std::string _uvc_opos_channel;
};

#endif // __LOCAL_VEHICLE_H__
