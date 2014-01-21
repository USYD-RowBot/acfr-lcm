#ifndef __UTILS_H__
#define __UTILS_H__

#include <iostream>
#include <vector>

#include "eigen_utils.h"

namespace perls 
{
    Eigen::Matrix3d skewSymmetric3d (const Eigen::Vector3d& v);

    Eigen::MatrixXd ldlt_inverse (const Eigen::MatrixXd &A);

    void marg_info_over (Eigen::MatrixXd& Lambda_p, Eigen::VectorXd& eta_p,
            const std::vector<int>& yi, const Eigen::MatrixXd& Lambda, const Eigen::VectorXd& eta);

    void marg_info_exclude (Eigen::MatrixXd& Lambda_p, Eigen::VectorXd& eta_p,
            const std::vector<int>& xi, const Eigen::MatrixXd& Lambda, const Eigen::VectorXd& eta);
}

#endif // __UTILS_H__
