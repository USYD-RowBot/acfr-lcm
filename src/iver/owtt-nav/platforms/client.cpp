#include <iostream>
#include <boost/lexical_cast.hpp>
#include <cstdlib>

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>

#include "perls-common/bot_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-lcmtypes++/perllcm/auv_iver_state_t.hpp"
#include "perls-lcmtypes++/perllcm/owtt_nav_status_t.hpp"

#include "perls-lcmtypes++/senlcm/acomms_osp_recovery_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_two_osp_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_pose_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_range_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_request_t.hpp"

#include "eigen_utils.h"
#include "perls-owtt-nav/meas_sensors.h"
#include "perls-owtt-nav/origin_state.h"
#include "perls-owtt-nav/utils.h"

#include "local_manager.h"


const unsigned int MAX_DELAYED_STATES = 3;
const int64_t CLIENT_LAG = 20e6;

class Client_manager : public Local_manager
{
  public:
    enum Rx_rule
    {
        NONE,
        INDEPENDENT,
        OSM
    };

    Client_manager (BotParam *param)
        : Local_manager (param, CLIENT_LAG), 
        _rule (NONE), _accumulator (2), _new_no (0), _org_no (0), _last_delta_no (0)
    {
        _x_vs_owtt = Eigen::MatrixXd::Zero (6,1);
        _speed_of_sound = bot_param_get_int_or_fail (param, "site.speed_of_sound");
        _sig_owtt = 0.1875;

        senlcm::acomms_pose_t tmp_pose = {0};
        _server_pose = new senlcm::acomms_pose_t (tmp_pose);
        senlcm::acomms_two_osp_t tmp_osp = {0};
        _server_osp = new senlcm::acomms_two_osp_t (tmp_osp);
        
        init_est (param);
        add_meas_callbacks (param);
    }
    ~Client_manager () 
    {
        delete _server_pose;
        delete _server_osp;
    }

  protected:
    void init_est (BotParam *param);

  private:
    Eigen::Matrix<double,6,1> _x_vs_owtt;
    double _sig_owtt;
    double _speed_of_sound;

    Rx_rule _rule;

    perls::Osm_accumulator _accumulator;
    int _new_no;
    int _org_no;

    Eigen::MatrixXd _last_Lambda;
    Eigen::VectorXd _last_eta;
    int _last_delta_no;

    void add_meas_callbacks (BotParam *param);
    void gpsd_ini_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::gpsd3_t *msg);
    void state_ini_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const perllcm::auv_iver_state_t *msg);

    void owtt_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_range_t *msg);
    void rx_pose_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_pose_t *msg);
    void rx_osp_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_two_osp_t *msg);

    void rx_recovery_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_osp_recovery_t *msg);
    std::string _rrq_chan;

    senlcm::acomms_pose_t *_server_pose;
    senlcm::acomms_two_osp_t *_server_osp;
};

void Client_manager::gpsd_ini_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::gpsd3_t *msg)
{
    double latlon[2] = {msg->fix.latitude * UNITS_RADIAN_TO_DEGREE, msg->fix.longitude * UNITS_RADIAN_TO_DEGREE};
    double yx[2] = {0};
    bot_gps_linearize_to_xy (_llxy, latlon, yx);

    Eigen::VectorXd mu0(2); mu0 << yx[1], yx[0];
    Eigen::MatrixXd Sigma0(2,2); Sigma0 = (_sig_gps*_sig_gps)*Eigen::Matrix2d::Identity (); 

    _estimator = new perls::If (msg->utime, _index, mu0, Sigma0, _pm, true);
    std::cout << "{Client_manager} initialized filter with mu0 = " << std::endl << mu0 << std::endl;
    std::cout << "{Client_manager} initialized filter with Sigma0 = " << std::endl << Sigma0 << std::endl;
}

void Client_manager::state_ini_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const perllcm::auv_iver_state_t *msg)
{
    Eigen::VectorXd mu0(2);
    mu0 << msg->position.xyzrph[0], msg->position.xyzrph[1];
    Eigen::MatrixXd Sigma0(2,2); Sigma0 = (_sig_gps*_sig_gps)*Eigen::Matrix2d::Identity ();

    _estimator = new perls::If (msg->position.utime, _index, mu0, Sigma0, _pm, true);
    std::cout << "{Client_manager} initialized filter with mu0 = " << std::endl << mu0 << std::endl;
    std::cout << "{Client_manager} initialized filter with Sigma0 = " << std::endl << Sigma0 << std::endl;
}

void Client_manager::init_est (BotParam *param)
{
    std::cout << "{Client_manager} waiting to initialize filter..." << std::endl;
    std::string chan = bot_param_get_str_or_fail (param, "vehicle.state_channel");
    lcm::Subscription *ini_sub = _lcm.subscribe (chan.c_str (), &Client_manager::state_ini_callback, this);
    //std::string chan = bot_param_get_str_or_fail (param, "sensors.gpsd3-client.gsd.channel");
    //lcm::Subscription *ini_sub = _lcm.subscribe (chan.c_str (), &Client_manager::gpsd_ini_callback, this);
    while (!_estimator) {
        _lcm.handle ();
    }
    _lcm.unsubscribe (ini_sub);
}

void Client_manager::add_meas_callbacks (BotParam *param)
{
    std::string rx_rule = bot_param_get_str_or_fail (param, "owtt-nav.rx_rule");
    if (rx_rule == "none") _rule = NONE;
    else if (rx_rule == "independent") _rule = INDEPENDENT;
    else if (rx_rule == "origin-state") _rule = OSM;
    else {
        std::cerr << "{Client_manager} unrecognized update rule in param!" << std::endl;
        exit (EXIT_FAILURE);
    }

    // NOTE: data packet is sent *before* the range packet --- but the utimes
    // should still match 
    // .: store the data meas wait for the range packet then augment
    // state/push meas on queue 
    // additionally, not all range meas's will be to the server vehicle
    std::string chan = bot_param_get_str_or_fail (param, "sensors.modem.gsd.channel");
    if (_rule != NONE) {
        std::string range_chan = chan+"_RANGE";
        _lcm.subscribe (range_chan, &Client_manager::owtt_callback, this);
    }

    switch (_rule)
    {
        case INDEPENDENT:
            {
                std::string data_chan = chan+"_NAV_POSE";
                _lcm.subscribe (data_chan, &Client_manager::rx_pose_callback, this);
            }
            break;
        case OSM:
            {
                std::string data_chan = chan+"_NAV_OSP";
                _lcm.subscribe (data_chan, &Client_manager::rx_osp_callback, this);
                std::string recover_chan = chan+"_NAV_OSP_RECOVERY";
                _lcm.subscribe (recover_chan, &Client_manager::rx_recovery_callback, this);
                ////TMP
                //_lcm.subscribe ("IVER31_ACOMMS_NAV_OSP_REPLY", &Client_manager::rx_osp_callback, this);
                //_lcm.subscribe ("IVER31_ACOMMS_NAV_OSP_RECOVERY_REPLY", &Client_manager::rx_recovery_callback, this);
            }
            break;
        default:
            break;
    }
    _rrq_chan = chan+"_RECOVERY_REQUEST";

    add_local_callbacks (param);
}

void Client_manager::rx_pose_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::acomms_pose_t *msg)
{
    std::cout << "{Client_manager} received acomms server pose data" << std::endl;
    *_server_pose = *msg;
}

void Client_manager::rx_osp_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::acomms_two_osp_t *msg)
{
    std::cout << "{Client_manager} received acomms server osp data" << std::endl;
    *_server_osp = *msg;
}

void Client_manager::rx_recovery_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::acomms_osp_recovery_t *msg)
{
    std::cout << "{Client_manager} received acomms server osp recovery data" << std::endl;
    *_server_osp = msg->two_osp_msg;

    // catch up accumulator w/ msg->eta, msg->Lambda
    if (_new_no > msg->new_tol_no) return; // we are tracking a state beyond the recovery already
    if (_new_no != msg->org_tol_no) return; // this recovery isnt tailored for us
    std::cout << "{Client_manager} adding recovery information" << std::endl;
    Eigen::MatrixXd Lam_rc (4,4);
    Eigen::VectorXd eta_rc (4);
    std::copy (msg->Lambda, msg->Lambda+16, Lam_rc.data ());
    std::copy (msg->eta, msg->eta+4, eta_rc.data ());
    _new_no = msg->new_tol_no;
    _org_no = msg->org_tol_no;

    std::string new_key = boost::lexical_cast<std::string> (_new_no);
    _accumulator.add_block_state (Lam_rc, eta_rc, new_key);
}

void Client_manager::owtt_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::acomms_range_t *msg)
{
    // TODO: should check CLK_MODE 
    if (msg->type != senlcm::acomms_range_t::ONE_WAY_SYNCHRONOUS) return;

    std::cout << "{Client_manager} received owtt message" << std::endl;
    double slant_range = msg->owtt[0]*_speed_of_sound;
    double depth_rel = 0; 

    std::string tol_key;
    if (_rule == INDEPENDENT) {
        if (msg->utime != _server_pose->utime) {
            std::cout << "{Client_manager} data utime does not match range utime!" << std::endl;
            return;
        }

        tol_key = boost::lexical_cast<std::string> (msg->utime);
        depth_rel = _depth - _server_pose->depth;

        Eigen::MatrixXd Sigma = Eigen::Map<Eigen::MatrixXd>(_server_pose->xy_cov,2,2);
        Eigen::VectorXd mu = Eigen::Map<Eigen::VectorXd>(_server_pose->xy,2);
        Eigen::MatrixXd Lambda = Sigma.inverse ();
        Eigen::VectorXd eta = Lambda*mu;
        _estimator->augment_independent (Lambda, eta, tol_key);

    }
    else if (_rule == OSM) {
        if (msg->utime != _server_osp->utime) {
            std::cout << "{Client_manager} data utime does not match range utime!" << std::endl;
            return;
        }
        ////TMP
        //if ((msg->utime - _server_osp->utime) > 20*1e6 || msg->src != 3) {
        //    std::cout << "{Client_manager} data utime does not match range utime!" << std::endl;
        //    return;
        //}

        int Ns = 2;
        Eigen::MatrixXd osp_Lambda (2*Ns,2*Ns);
        Eigen::VectorXd osp_eta (2*Ns);

        if (_server_osp->current.org_tol_no > _new_no && _new_no != _org_no) {
            if (_server_osp->last.org_tol_no > _new_no) {
                std::cerr << "{Client_manager} have not received current or backup origin!" << std::endl;
                senlcm::acomms_request_t request = {0};
                request.utime = timestamp_now ();
                request.type = senlcm::acomms_request_t::OSP_RECOVERY;
                request.last_tol_no = _new_no;
                _lcm.publish (_rrq_chan, &request);
                return;
            }
            std::cout << "{Client_manager} adding backup osp" << std::endl;
            std::copy (_server_osp->last.Lambda, _server_osp->last.Lambda+16, osp_Lambda.data ());
            std::copy (_server_osp->last.eta, _server_osp->last.eta+4, osp_eta.data ());
            _new_no = _server_osp->last.new_tol_no;
            _org_no = _server_osp->last.org_tol_no;
            _accumulator.add_osp (osp_Lambda, osp_eta, _new_no, _org_no);
            //_added_intermediate = true;
        }
        std::cout << "{Client_manager} adding current osp" << std::endl;
        std::copy (_server_osp->current.Lambda, _server_osp->current.Lambda+16, osp_Lambda.data ());
        std::copy (_server_osp->current.eta, _server_osp->current.eta+4, osp_eta.data ());
        _new_no = _server_osp->current.new_tol_no;
        _org_no = _server_osp->current.org_tol_no;
        _accumulator.add_osp (osp_Lambda, osp_eta, _new_no, _org_no);

        Eigen::MatrixXd current_Lambda = _accumulator.Lambda ();
        Eigen::VectorXd current_eta = _accumulator.eta ();

        if (_last_delta_no != _accumulator.state_order_no (1) && _last_delta_no > 0) { // this >0 is a hack
            Eigen::MatrixXd cL = current_Lambda;
            Eigen::VectorXd ce = current_eta;
            int end_state_no = _accumulator.find_state_no (_last_delta_no);
            //std::cout << "last state no in acc = " << end_state_no << std::endl;
            int Nm = (end_state_no-1)*Ns;
            std::vector<int> d_i (Nm); 
            for (int i=0;i<Nm;i++) d_i[i] = Ns+i;
            std::cout << "{Client_manager} removing intermediate states " << d_i << std::endl;
            perls::marg_info_over (current_Lambda,current_eta,d_i,cL,ce);
        }
        
        tol_key = boost::lexical_cast<std::string>(_new_no);
        depth_rel = _depth - _server_osp->current.depth;

        // compute and add delta packet
        if (!_last_Lambda.size ()) {
            std::string org_key = boost::lexical_cast<std::string>(_org_no);
            _estimator->augment_independent (Eigen::MatrixXd::Zero (Ns,Ns),Eigen::VectorXd::Zero (Ns),org_key);
            _estimator->add_delta_state (current_Lambda, current_eta, tol_key);
        }
        else {
            Eigen::MatrixXd last_L = Eigen::MatrixXd::Zero (2*Ns,2*Ns);
            last_L.bottomRightCorner (Ns,Ns) = _last_Lambda.topLeftCorner (Ns,Ns);
            Eigen::MatrixXd delta_Lambda = current_Lambda.topLeftCorner (2*Ns,2*Ns) - last_L;

            Eigen::VectorXd last_e = Eigen::VectorXd::Zero (2*Ns);
            last_e.tail (Ns) = _last_eta.head (Ns);
            Eigen::VectorXd delta_eta = current_eta.head (2*Ns) - last_e;

            _estimator->add_delta_state (delta_Lambda,delta_eta,tol_key);
        }
        _last_delta_no = _new_no;
        _last_Lambda = _accumulator.Lambda ();
        _last_eta = _accumulator.eta ();

        /*
        Eigen::MatrixXd Sigma = _accumulator.cov ();
        Eigen::VectorXd mu = _accumulator.mean ();

        perllcm::position_t pose = {0};
        pose.utime = msg->utime;
        pose.xyzrph[0] = mu(0);
        pose.xyzrph[1] = mu(1);
        pose.xyzrph_cov[0] = Sigma(0,0);
        pose.xyzrph_cov[1] = Sigma(0,1);
        pose.xyzrph_cov[6] = Sigma(1,0);
        pose.xyzrph_cov[7] = Sigma(1,1);

        std::string pub_chan = _chan_prefix + "ACCUMULATOR";
        _lcm.publish (pub_chan.c_str (), &pose);
        */

        perllcm::owtt_nav_status_t stat = {0};
        stat.utime = msg->utime;
        stat.last_tol_no = _new_no;
        _lcm.publish (_chan_prefix+"NAV_STATUS", &stat);
    }

    std::cout << "TIME DIFF = " << (msg->utime - _estimator->utime ())*1e-6 << std::endl;

    double pseudo_range = std::sqrt (slant_range*slant_range - depth_rel*depth_rel);

    std::vector<std::string> keys (2);
    keys[0] = "xy";
    keys[1] = tol_key;
    Eigen::Matrix<double,1,1> Sig; Sig << (_sig_owtt*_sig_owtt);
    Eigen::Matrix<double,1,1> z; z << pseudo_range;
    perls::Meas_ptr meas (new perls::Owtt_meas (msg->utime, _x_vs_owtt, z, Sig, _index, keys));
    _mqueue.push_pop (meas);

    // roll up delayed states --- only keep up to MAX_DELAYED_STATES
    if (_index("extra").nchildren () > MAX_DELAYED_STATES) {
        std::string last_key = _index("extra").child_key (_index("extra").nchildren ()-1);
        _estimator->marginalize (_index("extra"), last_key);
    }
}

int main (int argc, char *argv[])
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        std::cerr << "could not find " << BOTU_PARAM_DEFAULT_CFG << "!"
            << std::endl;
        exit (EXIT_FAILURE);
    }

    Client_manager client (param);
    while (true) {
        client.do_work ();
    }

    bot_param_destroy (param);
    exit (EXIT_SUCCESS);
}
