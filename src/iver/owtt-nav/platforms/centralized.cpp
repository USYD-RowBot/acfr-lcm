#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "perls-common/units.h"

#include "platform_utils.h"
#include "centralized.h"

const int64_t LAG_USECS = 20e6;

Vehicle::Vehicle (BotParam *param) 
    : _llxy (new BotGPSLinearize) 
{
}

void Vehicle::dstar_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::dstar_ssp1_t *msg)
{
    _depth = msg->depth;
}

void Vehicle::gpsd_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::gpsd3_t *msg)
{
    double latlon[2] = {msg->fix.latitude * UNITS_RADIAN_TO_DEGREE, msg->fix.longitude * UNITS_RADIAN_TO_DEGREE};
    double yx[2] = {0};
    bot_gps_linearize_to_xy (_llxy, latlon, yx);

    Eigen::Vector2d xy; xy << yx[1], yx[0];
    Eigen::Matrix2d Sig = (_sig_gps*_sig_gps)*Eigen::Matrix2d::Identity ();
    
    std::vector<std::string> keys (1, "xy");
    perls::Meas_ptr meas (new perls::Gps_meas (msg->utime, _x_vs_gps, xy, Sig, _index("base"), keys));
    _mqueue->push_pop (meas);
}

void Vehicle::rph_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
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

void Vehicle::rdi_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
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

        _mqueue->push_pop (meas);
    }
}


Centralized::Centralized (BotParam *param, const std::string& log_path)
    : _server (param), _client (param),
    _index (), _pm (0), _estimator (0), _mqueue (LAG_USECS)
    _lcm (0)
{
    _lcm = new lcm::LCM ("file://" + log_path + "?speed=0"); 
    if (!_lcm->good ()) {
        std::cerr << "{Centralized} lcm not good!" << std::endl;
        exit (EXIT_FAILURE);
    }
}

void Centralized::meas_callback (perls::Meas_ptr m)
{
}


int main (int argc, char *argv[])
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        std::cerr << "could not find " << BOTU_PARAM_DEFAULT_CFG << "!"
            << std::endl;
        exit (EXIT_FAILURE);
    }

    std::string path = "//home/jeff/data/UMBS_0812/iver31/2012-08-26-dive.027/lcmlog-2012-08-26-dive.027";
    Centralized central (param, path);
    while (central.do_work () == 0);

    bot_param_destroy (param);
    exit (EXIT_SUCCESS);
}
