#include <iostream>
#include <fstream>
#include <cmath>

#include <lcm/lcm-cpp.hpp>

#include "perls-lcmtypes++/perllcm/position_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_two_osp_t.hpp"

#include "eigen_utils.h"
#include "perls-owtt-nav/origin_state.h"
#include "perls-owtt-nav/utils.h"

class Test_accumulator
{
  public:
    Test_accumulator () : _accumulator (2)
    {
        _lcm.subscribe ("IVER31_ACOMMS_NAV_OSP_REPLY",&Test_accumulator::rx_osp_callback,this);
    }
    ~Test_accumulator () {};

    void compute_rv ();

    void do_work () {_lcm.handle ();}
    bool working () {return ((_accumulator.size () < 2 * 94) ? true : false);}

  private:
    perls::Osm_client_accumulator _accumulator;

    lcm::LCM _lcm;

    void rx_osp_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_two_osp_t *msg);
};

void Test_accumulator::rx_osp_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::acomms_two_osp_t *msg)
{
    Eigen::MatrixXd osp_Lambda (4,4);
    Eigen::VectorXd osp_eta (4);
    std::copy (msg->current.Lambda, msg->current.Lambda+16, osp_Lambda.data ());
    std::copy (msg->current.eta, msg->current.eta+4, osp_eta.data ());
    std::cout << "{Accumulator} osp_Lambda = " << std::endl
        << osp_Lambda << std::endl;
    std::cout << "{Accumulator} osp_eta = " << std::endl
        << osp_eta << std::endl;

    _accumulator.add_osp (osp_Lambda, osp_eta, msg->current.new_tol_no, msg->current.org_tol_no);

    Eigen::MatrixXd Sigma = _accumulator.cov ();
    Eigen::VectorXd mu = _accumulator.mean ();

    /*
    perllcm::position_t pose = {0};
    pose.utime = msg->utime;
    pose.xyzrph[0] = mu(0);
    pose.xyzrph[1] = mu(1);
    pose.xyzrph_cov[0] = Sigma(0,0);
    pose.xyzrph_cov[1] = Sigma(0,1);
    pose.xyzrph_cov[6] = Sigma(1,0);
    pose.xyzrph_cov[7] = Sigma(1,1);
    std::string pub_chan = "IVER31_ACCUMULATOR";

    _lcm.publish (pub_chan.c_str (), &pose);
    */

    std::cout << "{Test_accumulator} now tracking " << _accumulator.size ()/2 << " server states" << std::endl;
}

void Test_accumulator::compute_rv ()
{
    std::ofstream file;
    file.open ("rv_coeffs.txt");

    int Nb = 2;
    Eigen::MatrixXd Sigma = _accumulator.cov ();

    double e_bb = Sigma.topLeftCorner (Nb,Nb).trace ();

    for (int i=0;i<_accumulator.size ()/Nb - 1;++i) {
        std::cout << "testing correlation with " << i << "th state" << std::endl;
        double e_kk = Sigma.block (Nb*i,Nb*i,Nb,Nb).trace ();
        double e_bk = Sigma.block (0,Nb*i,Nb,Nb).trace ();

        double e_pp = Sigma.block (Nb*(i+1),Nb*(i+1),Nb,Nb).trace ();
        double e_kp = Sigma.block (Nb*i,Nb*(i+1),Nb,Nb).trace ();

        double rv_bk = e_bk/std::sqrt (e_bb*e_kk);
        double rv_kp = e_kp/std::sqrt (e_kk*e_pp);
        file << rv_bk << "\t" << rv_kp << std::endl;
    }

    file.close ();
}

int main (int argc, char *argv[])
{
    //Eigen::MatrixXd A = Eigen::MatrixXd::Zero (4,4);
    //A (2,2) = 3.3;
    //std::cout << "A is zero = " << ((A.array () == 0).all () ? "True" :"False") << std::endl;

    Test_accumulator client;
    while (client.working ())
        client.do_work ();

    std::cout << "{Test_accumulator} computing RV coefficient between current state and past states" << std::endl;
    client.compute_rv ();

    exit (EXIT_SUCCESS);
}
