#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "perls-common/lcm_util.h"
#include "perls-common/units.h"

#include "perls-lcmtypes++/perllcm/position_t.hpp"
#include "perls-lcmtypes++/senlcm/uvc_opos_t.hpp"

#include "perls-owtt-nav/meas_sensors.h"

#include "platform_utils.h"
#include "local_manager.h"

Local_manager::Local_manager (BotParam *param, int64_t lag_usecs)
    : _index (), _pm (0), _estimator (0), _llxy (new BotGPSLinearize), _mqueue (lag_usecs)
    //: _lcm ("file:///home/jeff/data/UMBS_0812/iver31/2012-08-26-dive.027/lcmlog-2012-08-26-dive.027?speed=0"), 
    //: _lcm ("file:///home/jeff/data/UMBS_0812/iver28/2012-08-26-dive.023/lcmlog-2012-08-26-dive.023?speed=0"), 
{
    if (!_lcm.good ()) {
        std::cerr << "{Local_manager} lcm not good!" << std::endl;
        exit (EXIT_FAILURE);
    }
    // initialize index 
    int nbase = 2;
    _index.add ("base");
    _index("base").add ("xy", nbase);
    _index.add ("extra");
    
    // set up process model
    std::vector<std::string> pm_keys (1, "base");
    _pm = new perls::Const_pos_discrete (nbase, _index, pm_keys);

    // parse local sensor config params
    _chan_prefix = bot_param_get_str_or_fail (param, "vehicle.lcm_channel_prefix");
    std::string name = bot_param_get_str_or_fail (param, "vehicle.name");

    _x_lr = Eigen::MatrixXd::Zero (6,1);
    bot_param_get_double_array (param, "sensors.ms-gx3-25.x_vs", _x_vs_rph.data (), 6);
    _x_vs_rph.tail(3) = UNITS_DEGREE_TO_RADIAN*_x_vs_rph.tail (3).array ();

    bot_param_get_double_array (param, "sensors.rdi.x_vs", _x_vs_dvl.data (), 6);
    _x_vs_dvl.tail(3) = UNITS_DEGREE_TO_RADIAN*_x_vs_dvl.tail (3).array ();

    bot_param_get_double_array (param, "sensors.gpsd3-client.x_vs", _x_vs_gps.data (), 6);
    _x_vs_gps.tail(3) = UNITS_DEGREE_TO_RADIAN*_x_vs_gps.tail (3).array ();

    _x_lv_rph = Eigen::MatrixXd::Zero (6,1);
    _uvwrph = Eigen::MatrixXd::Zero (6,1);

    std::string key = "owtt-nav." + name + ".sensors.gps";
    _gps = bot_param_get_boolean_or_fail (param, key.c_str ());
    key = "owtt-nav." + name + ".sensors.gps_odo";
    _gps_odo = bot_param_get_boolean_or_fail (param, key.c_str ());
    if (_gps_odo) { // using gps
        _sig_uv = 2.0;
        _sig_heading = 10.0*UNITS_DEGREE_TO_RADIAN; 
    }
    else { // using rdi
        _sig_uv = 0.15;
        _sig_heading = 2.0*UNITS_DEGREE_TO_RADIAN; 
    }
    _sig_gps = 3.0;

    key = "owtt-nav." + name + ".sensors.depth";
    std::string depth_src = bot_param_get_str_or_fail (param, key.c_str ());
    if (depth_src == "desert-star") {
        std::string chan = bot_param_get_str_or_fail (param, "sensors.dstar-ssp1.gsd.channel");
        _lcm.subscribe (chan, &Local_manager::dstar_callback, this);
    }
    else if (depth_src == "static") {
        key = "owtt-nav." + name + ".sensors.static_depth";
        _depth = bot_param_get_double_or_fail (param, key.c_str ());
    }
    else {
        std::cerr << "{Local_manager} unrecognized depth option" << std::endl;
        exit (EXIT_FAILURE);
    }

    double orglatlon[2] = {0};
    bot_param_get_double_array_or_fail (param, "site.orglatlon", orglatlon, 2);
    bot_gps_linearize_init (_llxy, orglatlon);

    _uvc_opos_channel = lcmu_channel_get_os_conduit (param, LCMU_CHANNEL_OS_CONDUIT_OPOS);

    _mqueue.set_callback (&Local_manager::meas_callback, this);
}

void Local_manager::add_local_callbacks (BotParam *param)
{
    std::string chan;
    if (_gps) {
        chan = bot_param_get_str_or_fail (param, "sensors.gpsd3-client.gsd.channel");
        _lcm.subscribe (chan, &Local_manager::gpsd_callback, this);
    }
    if (!_gps_odo) {
        chan = bot_param_get_str_or_fail (param, "sensors.rdi.gsd.channel");
        _lcm.subscribe (chan, &Local_manager::rdi_callback, this);

        chan = bot_param_get_str_or_fail (param, "sensors.ms-gx3-25.rphcorr.channel");
        _lcm.subscribe (chan, &Local_manager::rph_callback, this);
    }
}

void Local_manager::pred_aug_to_time (int64_t utime, const std::string& key) 
{
    int64_t dt = utime - _estimator->utime ();
    perls::Input in = compute_uvwrph_odo (dt, _sig_uv, _sig_heading, _uvwrph);
    _estimator->predict_augment (dt, in, key);
}

void Local_manager::meas_callback (perls::Meas_ptr meas)
{
    int64_t dt = meas->utime () - _estimator->utime ();
    if (dt < 0) {
        std::cerr << "{Local_manager} dt < 0!!!" << std::endl;
        return;
    }

    if (meas->type () == "uvwrph") {
        perls::Input in = compute_uvwrph_odo (dt, _sig_uv, _sig_heading, meas->raw ());
        _estimator->compute_cycle (dt, in);
        _uvwrph = meas->raw ();
    }
    else if (meas->type () == "gps") {
        perls::Input in = compute_uvwrph_odo (dt, _sig_uv, _sig_heading, _uvwrph);
        _estimator->compute_cycle (in, meas);
    }
    else {
        std::cout << "recieved " << *meas << std::endl;
        perls::Input in = compute_uvwrph_odo (dt, _sig_uv, _sig_heading, _uvwrph);
        _estimator->compute_cycle (in, meas);

        if (meas->type () == "owtt") 
            process_lagged_measurements ();
    }

    Eigen::VectorXd mean = _estimator->mean ();
    Eigen::MatrixXd cov = _estimator->cov ();
    std::vector<int> base_i = _index["base"];
    std::vector<int> extra_i = _index["extra"];

    perllcm::position_t pose = {0};
    pose.utime = _estimator->utime ();
    pose.xyzrph[0] = mean(base_i[0]);
    pose.xyzrph[1] = mean(base_i[1]);
    pose.xyzrph[2] = _depth;
    pose.xyzrph[5] = _uvwrph(5);
    pose.xyzrph_cov[0] = cov(base_i[0],base_i[0]);
    pose.xyzrph_cov[1] = cov(base_i[0],base_i[1]);
    pose.xyzrph_cov[6] = cov(base_i[1],base_i[0]);
    pose.xyzrph_cov[7] = cov(base_i[1],base_i[1]);

    std::string pub_chan = _chan_prefix + "EST_TEST";
    _lcm.publish (pub_chan, &pose);
}

void Local_manager::process_lagged_measurements () {
    std::cout << "{Local_managaer} processing lagged measurements ..." << std::endl;
    int Ns = 2;
    perls::Index lag_ind;
    lag_ind.add ("base");
    lag_ind("base").add ("xy", Ns);
    lag_ind.add ("extra");

    std::vector<std::string> pm_keys (1, "base");
    perls::Process_model* lag_pm = new perls::Const_pos_discrete (Ns,lag_ind,pm_keys);

    Eigen::VectorXd mu0 = _estimator->mean ().head (Ns);
    Eigen::MatrixXd Sig0 = _estimator->cov ().topLeftCorner (Ns,Ns);

    perls::If *lag_est = new perls::If (_estimator->utime (), lag_ind, mu0, Sig0, lag_pm, true);
    
    std::cout << "{Local_manager} queue has " << _mqueue.size () << " measurements remaining" << std::endl;
    std::vector<perls::Meas_ptr> meas_vect = _mqueue.remaining_queue ();

    Eigen::MatrixXd uvwrph = Eigen::MatrixXd::Zero (6,1);
    for (unsigned int i=0;i<meas_vect.size ();++i) {
        if (!meas_vect[i]) {
            continue;
        }
        int64_t dt = meas_vect[i]->utime () - lag_est->utime ();
        if (dt < 0) {
            std::cerr << "{Local_manager} dt < 0 processing lagged measurements" << std::endl;
            continue;
        }

        if (meas_vect[i]->type () == "uvwrph") {
            perls::Input in = compute_uvwrph_odo (dt, _sig_uv, _sig_heading, meas_vect[i]->raw ());
            lag_est->compute_cycle (dt, in);
            uvwrph = meas_vect[i]->raw ();
        }
        else if (meas_vect[i]->type () == "gps") {
            perls::Input in = compute_uvwrph_odo (dt, _sig_uv, _sig_heading, uvwrph);
            lag_est->compute_cycle (in, meas_vect[i]);
        }
    }
    std::cout << "{Local_manager} completed processing remaining measurements" << std::endl;

    Eigen::VectorXd mean = lag_est->mean ();
    Eigen::MatrixXd cov = lag_est->cov ();

    double xy[2] = {mean[1],mean[0]};
    double latlon_deg[2] = {0};
    bot_gps_linearize_to_lat_lon (_llxy, xy, latlon_deg);

    senlcm::uvc_opos_t cmd = {0};
    cmd.utime = lag_est->utime ();
    cmd.latitude = latlon_deg[0]*UNITS_DEGREE_TO_RADIAN;        
    cmd.longitude = latlon_deg[1]*UNITS_DEGREE_TO_RADIAN;
    //cmd.speed = std::sqrt (uvwrph(0)*uvwrph(0) + uvwrph(1)*uvwrph(1));
    cmd.speed = std::fabs (uvwrph(0));
    _lcm.publish (_uvc_opos_channel, &cmd);

    /* 
    double yx[2] = {0};
    bot_gps_linearize_to_xy (_llxy, latlon_deg, yx);
    perllcm::position_t pose = {0};
    pose.utime = lag_est->utime ();
    pose.xyzrph[0] = yx[1];
    pose.xyzrph[1] = yx[0];
    pose.xyzrph[2] = _depth;
    _lcm.publish (_chan_prefix+"CURRENT_EST", &pose);
    */

    delete lag_est;
    delete lag_pm;
}

void Local_manager::dstar_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::dstar_ssp1_t *msg)
{
    _depth = msg->depth;
}

void Local_manager::gpsd_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::gpsd3_t *msg)
{
    double latlon[2] = {msg->fix.latitude * UNITS_RADIAN_TO_DEGREE, msg->fix.longitude * UNITS_RADIAN_TO_DEGREE};
    double yx[2] = {0};
    bot_gps_linearize_to_xy (_llxy, latlon, yx);

    Eigen::Vector2d xy; xy << yx[1], yx[0];
    Eigen::Matrix2d Sig = (_sig_gps*_sig_gps)*Eigen::Matrix2d::Identity ();
    
    std::vector<std::string> keys (1, "xy");
    perls::Meas_ptr meas (new perls::Gps_meas (msg->utime, _x_vs_gps, xy, Sig, _index("base"), keys));
    _mqueue.push_pop (meas);

    if (_gps_odo) {
        double h = msg->fix.track * UNITS_DEGREE_TO_RADIAN;
        double s = msg->fix.speed;
        Eigen::Vector3d rph; rph << 0, 0, h;
        Eigen::Vector3d uvw; uvw << cos (h)*s, sin (h)*s, 0; 
        Eigen::Matrix<double,6,1> raw;
        raw(uvw_i) = uvw;
        raw(rph_i) = rph;
        perls::Meas_ptr meas (new Dvl_rph_meas (msg->utime, raw, _index));
        _mqueue.push_pop (meas);
    }
}

void Local_manager::rph_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::rph_t *msg)
{
    Eigen::Matrix<double,6,6> J_inv;
    Eigen::Matrix<double,6,12> J_h2t;

    Eigen::Matrix<double,6,1> x_rs; x_rs << 0,0,0,msg->rph[0],msg->rph[1],msg->rph[2];
    Eigen::Matrix<double,6,1> x_sv;
    Eigen::Matrix<double,6,1> x_rv;

    ssc_inverse (x_sv.data (), J_inv.data (), _x_vs_rph.data ());
    ssc_head2tail (x_rv.data (), J_h2t.data (), x_rs.data (), x_sv.data ());
    ssc_head2tail (_x_lv_rph.data (), J_h2t.data (), _x_lr.data (), x_rv.data ());
}

void Local_manager::rdi_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::rdi_pd4_t *msg)
{
    if (msg->btv[0] > senlcm::rdi_pd4_t::BTV_SENTINAL &&
        msg->btv[1] > senlcm::rdi_pd4_t::BTV_SENTINAL &&
        msg->btv[2] > senlcm::rdi_pd4_t::BTV_SENTINAL) {

        Eigen::Vector3d uvw_s; uvw_s << msg->btv[0], msg->btv[1], msg->btv[2];
        Eigen::Matrix3d R_lv;
        Eigen::Matrix3d R_vs;
        Eigen::Vector3d rph_lv = _x_lv_rph (perls::Meas::ssc_rph_i);
        Eigen::Vector3d rph_vs = _x_vs_dvl (perls::Meas::ssc_rph_i);

        so3_rotxyz (R_lv.data (), rph_lv.data ());
        so3_rotxyz (R_vs.data (), rph_vs.data ());

        Eigen::Vector3d uvw_v = R_vs * uvw_s;
        Eigen::Vector3d uvw_l = R_lv * uvw_v;

        Eigen::Matrix<double,6,1> raw;
        raw(uvw_i) = uvw_l;
        raw(rph_i) = rph_lv;
        perls::Meas_ptr meas (new Dvl_rph_meas (msg->utime, raw, _index));

        _mqueue.push_pop (meas);
    }
}
