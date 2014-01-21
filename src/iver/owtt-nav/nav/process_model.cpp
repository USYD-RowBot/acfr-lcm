#include <cmath>

#include "process_model.h"

void perls::Const_pos_discrete::transition (int64_t ut, const Eigen::VectorXd& state, const Input& odo)
{
    std::vector<int> base_i = _index[_index_keys[0]];
    Eigen::VectorXd base = state (base_i);

    _raw = base + odo.input_raw;
    _cov = odo.input_cov;
    _Jacobian = Eigen::MatrixXd::Identity (_raw_size, _raw_size);
}

