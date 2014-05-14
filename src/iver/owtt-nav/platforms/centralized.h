#ifndef __CENTRALIZED_H__
#define __CENTRALIZED_H__

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

class Vehicle
{
  public:
    Vehicle (BotParam *param);
    ~Vehicle () 
    {
        delete _llxy;
    }

    void bind_vehicle (lcm::LCM *lcm, perls::Index *index, perls::Meas_queue *mqueue)
    {
        _lcm = lcm;
        _index = index;
        _mqueue = mqueue;
    }

  private:
    perls::Index *_index;

    bool _gps;

    // static coordinate transforms
    Eigen::Matrix<double,6,1> _x_lr;
    Eigen::Matrix<double,6,1> _x_vs_dvl; 
    Eigen::Matrix<double,6,1> _x_vs_rph; 
    
    // current attitude
    Eigen::Matrix<double,6,1> _x_lv_rph;

    // lagged attitude/velocity measurement
    Eigen::Matrix<double,6,1> _uvwrph;

    perls::Meas_queue *_mqueue;

    BotGPSLinearize *_llxy; // SHOULD BE STATIC MEMBER 
    double _depth;

    double _sig_uv;
    double _sig_heading;
    double _sig_gps;

    lcm::LCM *_lcm;

    void dstar_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::dstar_ssp1_t *msg);
    void gpsd_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::gpsd3_t *msg);
    void rph_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::rph_t *msg);
    void rdi_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::rdi_pd4_t *msg);
};


class Centralized
{
  public:
    Centralized (BotParam *param, const std::string& log_path);
    ~Centralized ()
    {
        if (_lcm) delete lcm;
        if (_mqueue) delete _mqueue;
    }

    int do_work () {return _lcm->handle ();}

  private:
    Vehicle _server;
    Vehicle _client;

    perls::Index _index;
    perls::Process_model *_pm;
    perls::If *_estimator;

    perls::Meas_queue *_mqueue;
    void meas_callback (perls::Meas_ptr m);

    lcm::LCM *_lcm;
};



#endif // __CENTRALIZED_H__
