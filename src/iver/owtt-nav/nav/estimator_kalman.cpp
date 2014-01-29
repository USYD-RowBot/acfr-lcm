#include <iostream>
#include <gsl/gsl_cdf.h>

#include "estimator_kalman.h"

perls::Kf::Kf (int64_t ut, Index& index, Eigen::VectorXd& mu0, Eigen::MatrixXd& Sigma0, Process_model *pm)
    : Estimator (ut, index, pm), 
    _mu (Eigen::VectorXd::Zero (MAX_STATE_SIZE)), 
    _Sigma (Eigen::MatrixXd::Zero (MAX_STATE_SIZE,MAX_STATE_SIZE)), 
    _augment (false), _aug_key ("")
{
    std::vector<int> base_i = _index["base"];
    _Sigma (base_i,base_i) = Sigma0;
    _mu (base_i) = mu0;
}

void perls::Kf::predict (int64_t dt, Input& in)
{
    if (_augment) {
        std::cout << "{Kalman Filter} ------------  AUGMENT  --------------------------" << std::endl;
        
        std::vector<int> vbase_i = _index["base"];
        int bsz = vbase_i.size ();
        Eigen::VectorXi base_i (bsz);
        std::copy (vbase_i.begin (), vbase_i.end (), base_i.data ());

        Eigen::VectorXi new_i = base_i.array () + bsz;

        std::vector<int> vextra_i = _index["extra"];
        int esz = vextra_i.size ();
        Eigen::VectorXi extra_i (esz);
        std::copy (vextra_i.begin (), vextra_i.end (), extra_i.data ());

        if (esz) {
            Eigen::VectorXi shift_i = extra_i.array () + bsz;

            Eigen::VectorXd m_e  = _mu (extra_i);
            Eigen::MatrixXd S_be = _Sigma (base_i, extra_i);
            Eigen::MatrixXd S_ee = _Sigma (extra_i, extra_i);

            _mu (shift_i) = m_e;
            _Sigma (shift_i,shift_i) = S_ee;
            _Sigma (new_i,shift_i)   = S_be;
            _Sigma (shift_i,new_i)   = S_be.transpose ();
            _Sigma (base_i,shift_i)  = S_be;
            _Sigma (shift_i,base_i)  = S_be.transpose ();
        }

        _mu (new_i) = _mu (base_i); //_mu (base_i);
        _Sigma (new_i,new_i)  = _Sigma (base_i,base_i);
        _Sigma (new_i,base_i) = _Sigma (base_i,base_i);
        _Sigma (base_i,new_i) = _Sigma (base_i,base_i);

        _index("extra").prepend (_aug_key);
        _index("extra")(_aug_key).add ("xy", 2);

        // reset flags
        _aug_key = "";
        _augment = false;
    }

    std::cout << "{Kalman Filter} predicting state forward" << std::endl;
    std::vector<int> base_i = _index["base"];
    std::vector<int> extra_i = _index["extra"];

    Eigen::VectorXd mu_b = _mu (base_i);
    Eigen::MatrixXd Sigma_bb = _Sigma (base_i,base_i);
    
    _pmodel->transition (_utime, mu_b, in);
    Eigen::MatrixXd J = _pmodel->Jacobian ();
    Eigen::MatrixXd Q = _pmodel->cov ();

    _mu (base_i) = _pmodel->raw ();
    _Sigma (base_i,base_i) = J*Sigma_bb*J.transpose () + Q;
    if (extra_i.size ()) {
        Eigen::MatrixXd Sigma_be = _Sigma (base_i,extra_i);
        _Sigma (base_i,extra_i) = J*Sigma_be;
        _Sigma (extra_i,base_i) = _Sigma (base_i,extra_i).transpose ();
    }
}

void perls::Kf::process_meas (Meas_ptr m)
{
    std::cout << "{Kalman Filter} processing measurement " << m->type () << std::endl;

    int sz = _index.size ();
    m->observation_model (_mu.head (sz));
    Eigen::MatrixXd H = m->Jacobian ();
    Eigen::MatrixXd R = m->cov ();

    Eigen::MatrixXd S = H*_Sigma.topLeftCorner (sz,sz)*H.transpose () + R;
    Eigen::MatrixXd K = _Sigma.topLeftCorner (sz,sz)*H.transpose ()*S.inverse ();
    Eigen::VectorXd y = m->raw () - m->prediction ();

    double mahl_dist = y.transpose ()*S.inverse ()*y;
    if (mahl_dist > gsl_cdf_chisq_Pinv (0.99,m->raw ().size ()) ) {
        std::cout << "{Kalman Filter} measurement failed mahalanobis distance = " << mahl_dist << std::endl;
        return;
    }

    _mu.head (sz) = _mu.head (sz) + (K*y);
    _Sigma.topLeftCorner (sz,sz) = _Sigma.topLeftCorner (sz,sz) - (K*H* _Sigma.topLeftCorner (sz,sz));
}

void perls::Kf::compute_cycle (int64_t dt, Input& in)
{
    std::cout << "{Kalman Filter} ---------------- PRED CYCLE -------------------------" << std::endl;
    // make sure we aren't going back in time
    if (dt < 0) { // should throw exception perhaps --- or silently ignore meas
        std::cerr << "measurement time occured before current estimator time"
            << std::endl;
        return;
    }
    else if (dt == 0)
        return;
    predict (dt, in);
    _utime += dt;
}

void perls::Kf::compute_cycle (Input &in, Meas_ptr m)
{
    std::cout << "{Kalman Filter} ---------------- MEAS CYCLE -------------------------" << std::endl;
    // make sure we aren't going back in time
    int64_t dt = m->utime () - _utime;
    if (dt < 0) { // should throw exception perhaps --- or silently ignore meas
        std::cerr << "measurement time occured before current estimator time"
            << std::endl;
        return;
    }
    else if (dt == 0) {
        process_meas (m);
    }
    else {
        predict (dt, in);
        process_meas (m);
    }

    _utime += dt;
}

void perls::Kf::predict_augment (int64_t dt, Input& in, const std::string& key)
{
    compute_cycle (dt, in);
    _aug_key = key;
    _augment = true;
}

void perls::Kf::predict_augment (Input& in, Meas_ptr m, const std::string& key)
{
    compute_cycle (in, m);
    _aug_key = key;
    _augment = true;
}


