#include <algorithm>

#include "perls-math/so3.h"

#include "index.h"
#include "utils.h"

Eigen::Matrix3d perls::skewSymmetric3d (const Eigen::Vector3d& v)
{
    Eigen::Matrix3d M;
    M << 0,    -v(2),  v(1),
         v(2),  0,    -v(0), 
        -v(1),  v(0),    0;
    return M;
}

Eigen::MatrixXd perls::ldlt_inverse (const Eigen::MatrixXd &A)
{
    return A.ldlt ().solve (Eigen::MatrixXd::Identity (A.rows (), A.cols ()));
}

void perls::marg_info_over (Eigen::MatrixXd& Lambda_p, Eigen::VectorXd& eta_p,
        const std::vector<int>& yi, const Eigen::MatrixXd& Lambda, const Eigen::VectorXd& eta) 
{
    std::vector<int> ai (eta.size ());
    for (int i=0;i<eta.size ();++i) ai[i] = i;

    std::vector<int>::iterator it;

    // assume indices are sorted
    std::vector<int> xi (ai.size () - yi.size ());
    it = std::set_difference (ai.begin (),ai.end (),yi.begin (),yi.end (),xi.begin ());

    Eigen::MatrixXd Lambda_xx = Lambda (xi,xi);
    Eigen::MatrixXd Lambda_xy = Lambda (xi,yi);
    Eigen::MatrixXd Lambda_yy = Lambda (yi,yi);
    //Eigen::MatrixXd invLambda_yy = Lambda_yy.inverse ();
    Eigen::MatrixXd invLambda_yy = ldlt_inverse (Lambda_yy);

    Eigen::VectorXd eta_x = eta (xi);
    Eigen::VectorXd eta_y = eta (yi);

    Lambda_p = Lambda_xx - Lambda_xy*invLambda_yy*Lambda_xy.transpose ();
    eta_p = eta_x - Lambda_xy*invLambda_yy*eta_y;
}

void perls::marg_info_exclude (Eigen::MatrixXd& Lambda_p, Eigen::VectorXd& eta_p,
        const std::vector<int>& xi, const Eigen::MatrixXd& Lambda, const Eigen::VectorXd& eta) 
{
    if (static_cast<int>(xi.size ()) == eta.size ()) { // user must not actually want to marginalize out any states
        Lambda_p = Lambda;
        eta_p = eta;
        return;
    }
    else if (static_cast<int>(xi.size ()) > eta.size ()) {
        std::cerr << "{marg_info_exclude} index mismatch!" << std::endl;
        return;
    }

    std::vector<int> ai (eta.size ());
    for (int i=0;i<eta.size ();++i) ai[i] = i;

    std::vector<int>::iterator it;

    std::vector<int> yi (ai.size () - xi.size ());
    it = std::set_difference (ai.begin (),ai.end (),xi.begin (),xi.end (),yi.begin ());

    Eigen::MatrixXd Lambda_xx = Lambda (xi,xi);
    Eigen::MatrixXd Lambda_xy = Lambda (xi,yi);
    Eigen::MatrixXd Lambda_yy = Lambda (yi,yi);
    //Eigen::MatrixXd invLambda_yy = Lambda_yy.inverse ();
    Eigen::MatrixXd invLambda_yy = ldlt_inverse (Lambda_yy);

    Eigen::VectorXd eta_x = eta (xi);
    Eigen::VectorXd eta_y = eta (yi);

    Lambda_p = Lambda_xx - Lambda_xy*invLambda_yy*Lambda_xy.transpose ();
    eta_p = eta_x - Lambda_xy*invLambda_yy*eta_y;
}
