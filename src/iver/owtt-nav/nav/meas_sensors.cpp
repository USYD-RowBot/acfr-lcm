#include <cmath>

#include "perls-math/gsl_util_math.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "meas_sensors.h"

void perls::Rph_meas::observation_model (const Eigen::VectorXd& state)
{
    std::cout << "ms-gx3 observation model called" << std::endl;
    /*
    std::vector<int> rph_i = index.find (_index_keys[0]);
    int zi[3] = {0,1,2}; 
    std::vector<int> z_i (zi,zi+3);

    Eigen::VectorXd x_lv (6);
    Eigen::VectorXd x_rl (6);
    Eigen::VectorXd x_ls (6);
    Eigen::VectorXd x_rs (6);
    Eigen::MatrixXd J_ls_plus (6,12);
    Eigen::MatrixXd J_rs_plus (6,12);

    Eigen::VectorXd rph = state(rph_i);
    x_lv (ssc_rph_i) = rph;
   
    ssc_head2tail (x_ls.data (), J_ls_plus.data(), x_lv.data (), _x_vs.data ());
    ssc_head2tail (x_rs.data (), J_rs_plus.data(), x_rl.data (), x_ls.data ());
    Eigen::MatrixXd J_plus = J_rs_plus (ssc_xyzrph_i,ssc_xjk_i) * J_ls_plus (ssc_xyzrph_i,ssc_xij_i);

    _Jacobian = Eigen::MatrixXd (_raw_size, index.size ());
    _Jacobian(z_i,rph_i) = J_plus (ssc_rph_i,ssc_rph_i);
    */
}

void perls::Rdi_meas::observation_model (const Eigen::VectorXd& state)
{
    std::cout << "rdi observation model called" << std::endl;
    /*
    std::vector<int> uvw_i = index.find (_index_keys[0]);
    std::vector<int> abc_i = index.find (_index_keys[1]);
    int zi[3] = {0,1,2};
    std::vector<int> z_i (zi,zi+3);

    Eigen::MatrixXd O_sv (3,3);

    Eigen::Vector3d t_vs = _x_vs (ssc_xyz_i);
    Eigen::Vector3d r_vs = _x_vs (ssc_rph_i);
    so3_rotxyz (O_sv.data (), r_vs.data ());

    _Jacobian = Eigen::MatrixXd (_raw_size, index.size ());
    _Jacobian(z_i,uvw_i) = O_sv;
    _Jacobian(z_i,abc_i) = -O_sv*skewSymmetric3d (t_vs);
    */
}

void perls::Gps_meas::observation_model (const Eigen::VectorXd& state)
{
    std::cout << "gps observation model called" << std::endl;
    std::vector<int> xy_i = _index[_index_keys[0]];
    int zi[2] = {0,1};
    std::vector<int> z_i (zi,zi+2);

    _prediction = state(xy_i);

    _Jacobian = Eigen::MatrixXd::Zero (_raw_size, state.size ());
    _Jacobian(z_i,xy_i) = Eigen::MatrixXd::Identity (_raw_size,_raw_size);
}

void perls::Owtt_meas::observation_model (const Eigen::VectorXd& state)
{
    std::cout << "owtt observation model called" << std::endl;
    std::vector<int> c_xy_i = _index("base")[_index_keys[0]];
    std::vector<int> s_xy_i = _index("extra")[_index_keys[1]];
    std::vector<int> xy_i (c_xy_i.size ()+s_xy_i.size ());
    std::merge (c_xy_i.begin (),c_xy_i.end (),s_xy_i.begin (),s_xy_i.end (),xy_i.begin ());

    std::vector<int> z_i (1,0);

    std::vector<int> c_i (2); 
    c_i[0] = 0;
    c_i[1] = 1;
    std::vector<int> s_i (2);
    s_i[0] = 2;
    s_i[1] = 3;

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero (2,4);
    M (c_i,c_i) = Eigen::MatrixXd::Identity (2,2);
    M (c_i,s_i) = (-1*(Eigen::MatrixXd::Identity (2,2).array ())).matrix ();

    Eigen::MatrixXd T = Eigen::MatrixXd::Zero (4,state.size ());
    T (c_i, c_xy_i) = Eigen::MatrixXd::Identity (2,2);
    T (s_i, s_xy_i) = Eigen::MatrixXd::Identity (2,2);

                        
    Eigen::VectorXd norm = state.transpose ()*T.transpose ()*M.transpose ()*M*T*state;
    _prediction = norm.array ().pow (0.5).matrix ();
    std::cout << "PRED = " << _prediction << std::endl;
    std::cout << "RAW  = " << _raw << std::endl;

    _Jacobian = (norm.array ().pow (-0.5).matrix ())*state.transpose ()*T.transpose ()*M.transpose ()*M*T;
    std::cout << "J = " << std::endl << _Jacobian << std::endl;
}
