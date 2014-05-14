#include <iostream>
#include <boost/lexical_cast.hpp>
#include <vector>

#include "perls-owtt-nav/eigen_utils.h"
#include "perls-owtt-nav/estimator_info.h"
#include "perls-owtt-nav/estimator_kalman.h"
#include "perls-owtt-nav/index.h"
#include "perls-owtt-nav/origin_state.h"
#include "perls-owtt-nav/utils.h"

using perls::operator<<;

/* ORIGIN STATE TEST
 * server_est tracks server state through several time steps
 *
 * client accumulates server graph via acoustic origin-state-packet
 * transmissions
 */

//#define RV_THRESHOLD 5e-2
#define RV_THRESHOLD 0.80

void compute_osp (int& org_no, perls::Index &index, 
        Eigen::MatrixXd& osp_Lambda, Eigen::VectorXd& osp_eta, perls::If *est)
{
    int Ns = 2;
    std::vector<int> o_i = index("extra")[boost::lexical_cast<std::string>(org_no)];
    std::vector<int> k_i = index["base"];

    // check rv coeff with origin state:
    // if below thresh advance origin to last tol state
    Eigen::MatrixXd Sigma = est->cov ();
    double e_kk = Sigma.topLeftCorner (Ns,Ns).trace ();
    double e_oo = Sigma.block (o_i[0],o_i[0],Ns,Ns).trace ();
    double e_ko = Sigma.block (0,o_i[0],Ns,Ns).trace ();
    double rv_ko = e_ko / std::sqrt (e_kk*e_oo);

    std::cout << "rv between newest and origin = " << rv_ko << std::endl;
    if (rv_ko < RV_THRESHOLD) {
        int new_no = boost::lexical_cast<int>(index("extra").child (0).label ());
        std::cout << "shifting origin from " << org_no << " to " << new_no << std::endl;
        org_no = new_no;
        o_i = index("extra").child (0)[""];
    
        for (int i=index("extra").nchildren ()-1;i>0;--i) {
            est->marginalize (index("extra"), index("extra").child (i).label ());
        }
    }
    std::cout << "have index " << index("extra") << std::endl;
    std::cout << "computing osp org_no = " << org_no << std::endl;

    std::vector<int> m_i (2*Ns);
    std::merge (k_i.begin (),k_i.end (),o_i.begin (),o_i.end (),m_i.begin ());
    perls::marg_info_exclude (osp_Lambda, osp_eta, m_i, est->Lambda (), est->eta ());
}

int main (int argc, char *argv[])
{
    // initialize server filter
    int Ns = 2, Nb = 4;
    perls::Index server_index;
    server_index.add ("base");
    server_index("base").add ("xy", Ns);
    server_index.add ("extra");
    
    std::vector<std::string> pm_keys (1, "base");
    perls::Process_model *pm = new perls::Const_pos_discrete (Ns, server_index, pm_keys);

    Eigen::MatrixXd Sigma0 (Ns,Ns); Sigma0 << 1, 0.7, 0.7, 1;
    Eigen::VectorXd mu0 (Ns); mu0 << 0,1;

    perls::Osm_accumulator acc_server (Ns);
    Eigen::MatrixXd osp_Lambda;
    Eigen::VectorXd osp_eta;
    int server_org_no = 0;
    int server_new_no = 0;
    std::string new_key;

    // initialize client accumulator --- this is the clients version of the
    // server state
    perls::Osm_accumulator acc_client (Ns);

    // kick server filter off
    perls::If *server_est = new perls::If (0, server_index, mu0, Sigma0, pm, true);
    std::cout << "{Server} [XI]" << std::endl
              << "{Server} mu =    " << std::endl << server_est->mean () << std::endl
              << "{Server} Lambda = " << std::endl << server_est->Lambda () << std::endl
              << std::endl;

    // T0 (origin) move to position (0,2) [XI] --> [X0]
    perls::Input in;
    in.input_raw = Eigen::VectorXd(2); in.input_raw << 0,1;
    in.input_cov = Eigen::MatrixXd(2,2); in.input_cov << 0.8,0.1,0.1,0.8;
    server_est->predict_augment (1e6, in, "0");
    std::cout << "{Server} [XI] ---> [X0]" << std::endl
              << "{Server} mu =    " << std::endl << server_est->mean () << std::endl
              << "{Server} Lambda = " << std::endl << server_est->Lambda () << std::endl
              << std::endl;
    new_key = boost::lexical_cast<std::string>(server_new_no);
    acc_server.add_single_state (server_est->Lambda (), server_est->eta (),new_key);
    server_new_no++;

    // T1 (tol) move to position (3,2) [X0] --> [X1 D0]
    in.input_raw << 3,0;
    in.input_cov << 0.8,0.1,0.1,0.8;
    server_est->predict_augment (1e6, in, "1");
    std::cout << "{Server} [X0] ---> [X1 0]" << std::endl
              << "{Server} mu =    " << std::endl << server_est->mean () << std::endl
              << "{Server} Lambda = " << std::endl << server_est->Lambda () << std::endl
              << std::endl;
    new_key = boost::lexical_cast<std::string>(server_new_no);
    acc_server.add_block_state (server_est->Lambda ().topLeftCorner (Nb,Nb), server_est->eta ().head (Nb),new_key);
    server_new_no++;

    // transmit osp ["X1" "D0"]
    std::cout << "TRANSMIT to client" << std::endl;
    compute_osp (server_org_no, server_index, osp_Lambda, osp_eta, server_est);

    std::cout << "RECEIVED from server" << std::endl;
    acc_client.add_osp (osp_Lambda, osp_eta, server_new_no-1, server_org_no);
    std::cout << "{Client} Lambda = " << std::endl << acc_client.Lambda () << std::endl
              << "{Client} mu     = " << std::endl << acc_client.mean () << std::endl;

    // T2 (tol) move to position (4,3) [X1 D0] --> [X2 D1 D0]
    in.input_raw << 1,1;
    in.input_cov << 0.8,0.1,0.1,0.8;
    server_est->predict_augment (1e6, in, "2");
    std::cout << "{Server} [X1 0] ---> [X2 1 0]" << std::endl
              << "{Server} mu =    " << std::endl << server_est->mean () << std::endl
              << "{Server} Lambda = " << std::endl << server_est->Lambda () << std::endl
              << std::endl;
    new_key = boost::lexical_cast<std::string>(server_new_no);
    acc_server.add_block_state (server_est->Lambda ().topLeftCorner (Nb,Nb), server_est->eta ().head (Nb),new_key);
    server_new_no++;

    // transmit osp ["X2" "D0"]
    std::cout << "TRANSMIT to client" << std::endl;
    compute_osp (server_org_no, server_index, osp_Lambda, osp_eta, server_est);
    std::cout << "RECEIVED from server" << std::endl;
    acc_client.add_osp (osp_Lambda, osp_eta, server_new_no-1, server_org_no);
    std::cout << "{Client} Lambda = " << std::endl << acc_client.Lambda () << std::endl
              << "{Client} mu     = " << std::endl << acc_client.mean () << std::endl;


    // T3 (tol) move to position (3,7) [X2 D1 D0] --> [X3 D2 D1 D0]
    in.input_raw << -1,4;
    in.input_cov << 0.8,0.1,0.1,0.8;
    server_est->predict_augment (1e6, in, "3");
    std::cout << "{Server} [X2 1 0] ---> [X3 2 1 0]" << std::endl
              << "{Server} mu =    " << std::endl << server_est->mean () << std::endl
              << "{Server} Lambda = " << std::endl << server_est->Lambda () << std::endl
              << std::endl;
    new_key = boost::lexical_cast<std::string>(server_new_no);
    acc_server.add_block_state (server_est->Lambda ().topLeftCorner (Nb,Nb), server_est->eta ().head (Nb),new_key);
    server_new_no++;

    // transmit osp ["X3" "D0"]
    std::cout << "TRANSMIT to client" << std::endl;
    compute_osp (server_org_no, server_index, osp_Lambda, osp_eta, server_est);
    std::cout << "RECEIVED from server" << std::endl;
    acc_client.add_osp (osp_Lambda, osp_eta, server_new_no-1, server_org_no);
    std::cout << "{Client} Lambda = " << std::endl << acc_client.Lambda () << std::endl
              << "{Client} mu     = " << std::endl << acc_client.mean () << std::endl;

    // T4 (tol) move to position (5,8) [X3 D2 D1 D0] --> [X4 D3 D2 D1 D0]
    in.input_raw << 2,1;
    in.input_cov << 0.8,0.1,0.1,0.8;
    server_est->predict_augment (1e6, in, "4");
    std::cout << "{Server} [X3 2 1 0] ---> [X4 3 2 1 0]" << std::endl
              << "{Server} mu =    " << std::endl << server_est->mean () << std::endl
              << "{Server} Lambda = " << std::endl << server_est->Lambda () << std::endl
              << std::endl;
    new_key = boost::lexical_cast<std::string>(server_new_no);
    acc_server.add_block_state (server_est->Lambda ().topLeftCorner (Nb,Nb), server_est->eta ().head (Nb),new_key);
    server_new_no++;

    // transmit osp ["X4" "D2"]
    std::cout << "TRANSMIT to client" << std::endl;
    compute_osp (server_org_no, server_index, osp_Lambda, osp_eta, server_est);
    std::cout << "RECEIVED from server" << std::endl;
    acc_client.add_osp (osp_Lambda, osp_eta, server_new_no-1, server_org_no);
    std::cout << "{Client} Lambda = " << std::endl << acc_client.Lambda () << std::endl
              << "{Client} mu     = " << std::endl << acc_client.mean () << std::endl;

    std::cout << "{Server} Lambda = " << std::endl << acc_server.Lambda () << std::endl
              << "{Server} mu     = " << std::endl << acc_server.mean () << std::endl;

    delete server_est;
    delete pm;

    return 0;
}
