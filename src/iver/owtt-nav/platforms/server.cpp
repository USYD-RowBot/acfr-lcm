#include <iostream>
#include <boost/lexical_cast.hpp>
#include <cstdlib>

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>

#include "perls-common/bot_util.h"
#include "perls-common/units.h"

#include "perls-lcmtypes++/perllcm/auv_iver_state_t.hpp"
#include "perls-lcmtypes++/perllcm/owtt_nav_status_t.hpp"

#include "perls-lcmtypes++/senlcm/acomms_osp_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_osp_recovery_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_two_osp_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_pose_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_request_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"

#include "eigen_utils.h"
#include "perls-owtt-nav/origin_state.h"

#include "local_manager.h"

const int64_t SERVER_LAG = 0;
const double RV_THRESHOLD = 5e-2;
const int ORG_THRESHOLD = 20;

class Server_manager : public Local_manager
{
  public:
    enum Tx_rule
    {
        NONE,
        INDEPENDENT,
        OSM
    };

    Server_manager (BotParam *param)
        : Local_manager (param, SERVER_LAG), 
        _rule (NONE), _accumulator (2), _org_no (0), _new_no (0),
        _org_count (0)
    {
        senlcm::acomms_osp_t tmp_osp = {0};
        _backup_osp = tmp_osp;
        _last_osp = tmp_osp;

        init_est (param);
        add_custom_callbacks (param);
    }
    ~Server_manager () {}

  protected:
    void init_est (BotParam *param);

  private:
    Tx_rule _rule;
    std::string _packet_reply_chan;
    std::string _recovery_reply_chan;
    senlcm::acomms_osp_t _backup_osp;
    senlcm::acomms_osp_t _last_osp;

    perls::Osm_accumulator _accumulator;
    int _org_no;
    int _new_no;
    Eigen::MatrixXd _osp_Lambda;
    Eigen::VectorXd _osp_eta;

    void compute_osp ();
    void tx_nav_osp (const senlcm::acomms_request_t *msg);
    ////TMP
    //void tx_nav_osp (const senlcm::acomms_request_old_t *msg);

    void add_custom_callbacks (BotParam *param);
    void gpsd_ini_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::gpsd3_t *msg);
    void state_ini_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const perllcm::auv_iver_state_t *msg);

    void tx_nav_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_request_t *msg);
    ////TMP
    //void tx_nav_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
    //        const senlcm::acomms_request_old_t *msg);

    ////TMP
    //void tmp_rc_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
    //        const senlcm::acomms_request_t *msg);
    //int _tmp_last_tol_no;

    int _org_count;
};

void Server_manager::gpsd_ini_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::gpsd3_t *msg)
{
    double latlon[2] = {msg->fix.latitude * UNITS_RADIAN_TO_DEGREE, msg->fix.longitude * UNITS_RADIAN_TO_DEGREE};
    double yx[2] = {0};
    bot_gps_linearize_to_xy (_llxy, latlon, yx);

    Eigen::VectorXd mu0(2); mu0 << yx[1], yx[0];
    Eigen::MatrixXd Sigma0(2,2); Sigma0 = (_sig_gps*_sig_gps)*Eigen::Matrix2d::Identity (); 

    _estimator = new perls::If (msg->utime, _index, mu0, Sigma0, _pm, true);
    std::cout << "{Server_manager} initialized filter with mu0 = " << std::endl << mu0 << std::endl;
    std::cout << "{Server_manager} initialized filter with Sigma0 = " << std::endl << Sigma0 << std::endl;
}

void Server_manager::state_ini_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const perllcm::auv_iver_state_t *msg)
{
    Eigen::VectorXd mu0(2);
    mu0 << msg->position.xyzrph[0], msg->position.xyzrph[1];
    Eigen::MatrixXd Sigma0(2,2); Sigma0 = (_sig_gps*_sig_gps)*Eigen::Matrix2d::Identity ();

    _estimator = new perls::If (msg->position.utime, _index, mu0, Sigma0, _pm, true);
    std::cout << "{Server_manager} initialized filter with mu0 = " << std::endl << mu0 << std::endl;
    std::cout << "{Server_manager} initialized filter with Sigma0 = " << std::endl << Sigma0 << std::endl;
}

void Server_manager::init_est (BotParam *param)
{
    std::cout << "{Server_manager} waiting to initialize filter..." << std::endl;

    std::string name = bot_param_get_str_or_fail (param, "vehicle.name");
    std::string key = "owtt-nav." + name + ".init";
    std::string init = bot_param_get_str_or_fail (param, key.c_str ());

    lcm::Subscription *ini_sub = 0;
    if (init == "gps") {
        std::string chan = bot_param_get_str_or_fail (param, "sensors.gpsd3-client.gsd.channel");
        ini_sub = _lcm.subscribe (chan.c_str (), &Server_manager::gpsd_ini_callback, this);
    }
    else if (init == "uvc") {
        std::string chan = bot_param_get_str_or_fail (param, "vehicle.state_channel");
        ini_sub = _lcm.subscribe (chan.c_str (), &Server_manager::state_ini_callback, this);
    }
    else {
        std::cerr << "{Server_manager} initialization channel specified incorrectly!" << std::endl;
        exit (EXIT_FAILURE);
    }

    while (!_estimator) {
        _lcm.handle ();
    }
    _lcm.unsubscribe (ini_sub);
}

void Server_manager::add_custom_callbacks (BotParam *param)
{
    std::string chan = bot_param_get_str_or_fail (param, "sensors.modem.gsd.channel");

    std::string tx_rule = bot_param_get_str_or_fail (param, "owtt-nav.tx_rule");
    if (tx_rule == "none") {
        _rule = NONE;
    }
    else if (tx_rule == "independent") {
        _packet_reply_chan = chan + "_NAV_POSE_REPLY";
        _rule = INDEPENDENT;
    }
    else if (tx_rule == "origin-state") {
        _packet_reply_chan = chan + "_NAV_OSP_REPLY";
        _recovery_reply_chan = chan + "_NAV_OSP_RECOVERY_REPLY";
        _rule = OSM;
    }
    else {
        std::cerr << "{Server_manager} unrecognized tx rule in param!" << std::endl;
        exit (EXIT_FAILURE);
    }

    if (_rule != NONE) {
        std::string request_chan = chan + "_NAV_REQUEST";
        _lcm.subscribe (request_chan.c_str (), &Server_manager::tx_nav_callback, this);

        ////TMP
        //std::string iver28_request_chan = "IVER28_ACOMMS_RECOVERY_REQUEST";
        //_lcm.subscribe (iver28_request_chan, &Server_manager::tmp_rc_callback, this);
    }

    add_local_callbacks (param);
}

void Server_manager::compute_osp ()
{
    int Ns = 2;
    std::vector<int> o_i = _index("extra")[boost::lexical_cast<std::string>(_org_no)];
    std::vector<int> k_i = _index["base"];

    // check rv coeff with origin state:
    // if below thresh advance origin to last tol state
    Eigen::MatrixXd Sigma = _estimator->cov ();
    double e_kk = Sigma.topLeftCorner (Ns,Ns).trace ();
    double e_oo = Sigma.block (o_i[0],o_i[0],Ns,Ns).trace ();
    double e_ko = Sigma.block (0,o_i[0],Ns,Ns).trace ();
    double rv_ko = e_ko / std::sqrt (e_kk*e_oo);

    std::cout << "{Server_manager} rv between newest and origin = " << rv_ko << std::endl;
    if (rv_ko < RV_THRESHOLD || _org_count>=ORG_THRESHOLD) {
        std::cout << "{Server_manager} shifting origin from " << _org_no << " to " << _new_no-1 << std::endl;
        _org_no = _new_no-1;
        o_i = _index("extra").child (0)[""];

        // roll up to behind the origin state if necessary
        for (int i=_index("extra").nchildren ()-1;i>1;--i) {
            _estimator->marginalize (_index("extra"), _index("extra").child (i).label ());
        }
        // could also marginalize out states between origin and current
        _org_count = 0;
    }
    _org_count++;

    std::vector<int> m_i (2*Ns);
    std::merge (k_i.begin (),k_i.end (),o_i.begin (),o_i.end (),m_i.begin ());
    perls::marg_info_exclude (_osp_Lambda, _osp_eta, m_i, _estimator->Lambda (), _estimator->eta ());
}

////TMP
//void Server_manager::tx_nav_osp (const senlcm::acomms_request_old_t *msg)
void Server_manager::tx_nav_osp (const senlcm::acomms_request_t *msg)
{
    std::cout << "{Server_manager} osp request" << std::endl;
    std::string tol_key = boost::lexical_cast<std::string>(_new_no);
    pred_aug_to_time (msg->utime, tol_key);

    std::cout << "{Server_manager} adding tol state" << std::endl;

    // compute origin state packet
    if (_index ("extra").empty ()) {
        std::string new_key = boost::lexical_cast<std::string>(_new_no);
        _accumulator.add_single_state (_estimator->Lambda (), _estimator->eta (), new_key);
    }
    else {
        compute_osp ();

        std::cout << "{Server_manager} origin no = " << _org_no << std::endl;
        std::cout << "{Server_manager} tol no    = " << _new_no << std::endl;

        std::cout << "{Server_manager} osp_Lambda = " << std::endl
            << _osp_Lambda << std::endl;
        std::cout << "{Server_manager} osp_eta = " << std::endl
            << _osp_eta << std::endl;

        // sign, seal, and deliver
        senlcm::acomms_two_osp_t osp_msg = {0};
        osp_msg.utime = _estimator->utime ();

        osp_msg.current.utime = _estimator->utime ();
        osp_msg.current.depth = _depth;
        osp_msg.current.new_tol_no = _new_no;
        osp_msg.current.org_tol_no = _org_no;
        std::copy (_osp_Lambda.data (), _osp_Lambda.data ()+_osp_Lambda.size (), osp_msg.current.Lambda);
        std::copy (_osp_eta.data (), _osp_eta.data ()+_osp_eta.size (), osp_msg.current.eta);

        if (_backup_osp.new_tol_no != _org_no) { // means we just shifted origin
            _backup_osp = _last_osp;
        }
        osp_msg.last = _backup_osp;
        _last_osp = osp_msg.current;

        ////TMP
        //if (msg->type == senlcm::acomms_request_t::OSP && !_tmp_last_tol_no) {
        if (msg->type == senlcm::acomms_request_t::OSP) {
            _lcm.publish (_packet_reply_chan, &osp_msg);
        }
        else if (msg->type == senlcm::acomms_request_t::OSP_RECOVERY) {
            senlcm::acomms_osp_recovery_t rc_msg;
            rc_msg.two_osp_msg = osp_msg;

            // compute recovery packet from accumulator
            ////TMP
            //int rc_tol_no = _tmp_last_tol_no;
            int rc_tol_no = msg->last_tol_no;
            Eigen::MatrixXd Lam_rc;
            Eigen::VectorXd eta_rc;
            _accumulator.block_state (Lam_rc,eta_rc,_org_no,rc_tol_no);

            std::cout << "RECOVERY " << std::endl << Lam_rc.inverse () << std::endl;
            std::cout << "RECOVERY " << std::endl << Lam_rc.inverse ()*eta_rc << std::endl;

            rc_msg.new_tol_no = _org_no;
            rc_msg.org_tol_no = rc_tol_no;
            std::copy (Lam_rc.data (), Lam_rc.data ()+Lam_rc.size (), rc_msg.Lambda);
            std::copy (eta_rc.data (), eta_rc.data ()+eta_rc.size (), rc_msg.eta);

            _lcm.publish (_recovery_reply_chan, &rc_msg);
            ////TMP
            //_tmp_last_tol_no = 0;
        }

        std::string new_key = boost::lexical_cast<std::string>(_new_no);
        _accumulator.add_block_state (_estimator->Lambda ().topLeftCorner (4,4), _estimator->eta ().head (4), new_key);
    }

    perllcm::owtt_nav_status_t stat = {0};
    stat.utime = msg->utime;
    stat.last_tol_no = _new_no;
    _lcm.publish (_chan_prefix+"NAV_STATUS", &stat);

    _new_no++;
}

////TMP
//void Server_manager::tmp_rc_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
//        const senlcm::acomms_request_t *msg)
//{
//    _tmp_last_tol_no = msg->last_tol_no;
//}

////TMP
//void Server_manager::tx_nav_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
//        const senlcm::acomms_request_old_t *msg)
void Server_manager::tx_nav_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::acomms_request_t *msg)
{
    std::cout << "{Server_manager} tx request" << std::endl;
    if (msg->type == senlcm::acomms_request_t::POSE) {
        std::cout << "{Server_manager} pose request" << std::endl;
        std::string tol_key = boost::lexical_cast<std::string>(msg->utime);
        pred_aug_to_time (msg->utime, tol_key);

        Eigen::MatrixXd cov = _estimator->cov ();
        Eigen::VectorXd mean = _estimator->mean ();
        std::vector<int> base_i = _index["base"];
        std::vector<int> extra_i = _index["extra"];

        // sign, seal, and deliver
        senlcm::acomms_pose_t pose_msg = {0};
        pose_msg.utime = _estimator->utime ();
        pose_msg.depth = _depth;

        pose_msg.xy[0] = mean (base_i[0]);
        pose_msg.xy[1] = mean (base_i[1]);
        pose_msg.xy_cov[0] = cov (base_i[0],base_i[0]);
        pose_msg.xy_cov[1] = cov (base_i[0],base_i[1]);
        pose_msg.xy_cov[2] = cov (base_i[1],base_i[0]);
        pose_msg.xy_cov[3] = cov (base_i[1],base_i[1]);
        _lcm.publish (_packet_reply_chan, &pose_msg);
    }
    else if (msg->type == senlcm::acomms_request_t::OSP || msg->type == senlcm::acomms_request_t::OSP_RECOVERY) {
        tx_nav_osp (msg);
    }
}

int main (int argc, char *argv[])
{
    ////TMP
    //BotParam *param = bot_param_new_from_file ("/home/jeff/perls/config/iver31.cfg");
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        std::cerr << "could not find " << BOTU_PARAM_DEFAULT_CFG << "!"
            << std::endl;
        exit (EXIT_FAILURE);
    }

    Server_manager server (param);
    while (true) {
        server.do_work ();
    }

    bot_param_destroy (param);
    exit (EXIT_SUCCESS);
}
