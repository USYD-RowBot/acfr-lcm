#include <iostream>
#include <gsl/gsl_cdf.h>

#include "estimator_info.h"

perls::If::If (int64_t ut, Index& index, Eigen::VectorXd& mu0, Eigen::MatrixXd& Sigma0, 
        Process_model *pm, bool is_lin)
    : Estimator (ut, index, pm), 
    _eta (Eigen::VectorXd::Zero (MAX_STATE_SIZE)),  
    _Lambda (Eigen::MatrixXd::Zero (MAX_STATE_SIZE,MAX_STATE_SIZE)), 
    _is_linear (is_lin),
    _augment (false), _aug_key ("")
{
    int sz = _index.size ();
    Eigen::MatrixXd Lambda0 = Sigma0.inverse ();
    _Lambda.topLeftCorner (sz,sz) = Lambda0;
    _eta.head (sz) = Lambda0 * mu0;
}

void perls::If::predict (int64_t dt, Input& in)
{
    int sz = _index.size ();

    std::cout << "{Information Filter} predicting state forward" << std::endl;
    std::vector<int> base_i = _index["base"];
    std::vector<int> extra_i = _index["extra"];

    Eigen::VectorXd mu = Eigen::VectorXd::Zero (sz);
    if (!_is_linear) mu = mean ();

    _pmodel->transition (_utime, mu, in);
    Eigen::MatrixXd J = _pmodel->Jacobian ();
    Eigen::MatrixXd Q = _pmodel->cov ();
    Eigen::MatrixXd Q_inv = Q.inverse ();

    if (_augment) {
        std::cout << "{Information Filter} -------  AUGMENT  --------------------------" << std::endl;

        int bsz = base_i.size ();
        Eigen::VectorXi vbase_i (bsz);
        std::copy (base_i.begin (), base_i.end (), vbase_i.data ());

        Eigen::VectorXi new_i = vbase_i.array () + bsz;

        int esz = extra_i.size ();
        if (esz) {
            Eigen::VectorXi vextra_i (esz);
            std::copy (extra_i.begin (), extra_i.end (), vextra_i.data ());
            Eigen::VectorXi shift_i = vextra_i.array () + bsz;

            Eigen::VectorXd eta_e = _eta (vextra_i);
            Eigen::MatrixXd Lambda_be = _Lambda (vbase_i,vextra_i);
            Eigen::MatrixXd Lambda_ee = _Lambda (vextra_i,vextra_i);

            _eta (shift_i) = eta_e;
            _Lambda (shift_i,shift_i) = Lambda_ee;
            _Lambda (new_i,shift_i)   = Lambda_be;
            _Lambda (shift_i,new_i)   = Lambda_be.transpose ();

        }

        Eigen::VectorXd eta_b = _eta (vbase_i);
        Eigen::MatrixXd Lambda_bb = _Lambda (vbase_i,vbase_i);

        _eta (new_i) = eta_b - J.transpose ()*Q_inv*in.input_raw;
        _eta (vbase_i) = Q_inv*in.input_raw;
        _Lambda (new_i,new_i)   = J.transpose ()*Q_inv*J + Lambda_bb;
        _Lambda (vbase_i,new_i)  = -Q_inv*J;
        _Lambda (new_i,vbase_i)  = -J.transpose ()*Q_inv;
        _Lambda (vbase_i,vbase_i) = Q_inv;

        _index("extra").prepend (_aug_key);
        _index("extra")(_aug_key).add ("xy", 2);
        extra_i = _index["extra"];

        sz = _index.size ();

        // reset flags
        _aug_key = "";
        _augment = false;
    }
    else {
        Eigen::VectorXd eta_b = _eta (base_i);
        Eigen::MatrixXd Lambda_bb = _Lambda (base_i,base_i);

        //Eigen::MatrixXd Lambda_bb_inv = Lambda_bb.inverse ();
        Eigen::MatrixXd Lambda_bb_inv = ldlt_inverse (Lambda_bb);
        Eigen::MatrixXd Phi_inv = Q + J*Lambda_bb_inv*J.transpose ();
        //Eigen::MatrixXd Phi = Phi_inv.inverse ();
        Eigen::MatrixXd Phi = ldlt_inverse (Phi_inv);
        Eigen::MatrixXd Omega = Lambda_bb + J.transpose ()*Q_inv*J;
        //Eigen::MatrixXd Omega_inv = Omega.inverse ();
        Eigen::MatrixXd Omega_inv = ldlt_inverse (Omega);

        _eta (base_i) = Q_inv*J*Omega_inv*eta_b + Phi*in.input_raw;
        _Lambda (base_i,base_i) = Phi;

        if (extra_i.size ()) {
            Eigen::VectorXd eta_e = _eta (extra_i);
            Eigen::MatrixXd Lambda_be = _Lambda (base_i,extra_i);
            Eigen::MatrixXd Lambda_ee = _Lambda (extra_i,extra_i);

            _eta (extra_i) = eta_e - Lambda_be.transpose ()*Omega_inv*(eta_b - J.transpose ()*Q_inv*in.input_raw);
            _Lambda (base_i,extra_i) = Q_inv*J*Omega_inv*Lambda_be;
            _Lambda (extra_i,base_i) = Lambda_be.transpose ()*Omega_inv*J.transpose ()*Q_inv;
            _Lambda (extra_i,extra_i) = Lambda_ee - Lambda_be.transpose ()*Omega_inv*Lambda_be;
        }
    }
    //std::cout << "{Information Filter} Lambda = " << std::endl << _Lambda.topLeftCorner (sz,sz) << std::endl;
    //std::cout << "{Information Filter} eta = " << std::endl << _eta.head (sz) << std::endl;
}

void perls::If::process_meas (Meas_ptr m)
{
    std::cout << "{Information Filter} processing measurement " << m->type () << std::endl;

    int sz = _index.size ();
    Eigen::MatrixXd Sigma = ldlt_inverse (_Lambda.topLeftCorner (sz,sz));
    Eigen::VectorXd mu = Sigma*_eta.head (sz);
    m->observation_model (mu);
    Eigen::MatrixXd H = m->Jacobian ();
    Eigen::MatrixXd R = m->cov ();

    Eigen::VectorXd y = m->raw () - m->prediction ();

    // mahalanobis distance check
    Eigen::MatrixXd S = H*Sigma*H.transpose () + R;

    double mahl_dist = y.transpose ()*S.inverse ()*y;
    if (mahl_dist > gsl_cdf_chisq_Pinv (0.9999,m->raw ().size ()) ) {
        std::cout << "{Information Filter} measurement failed mahalanobis distance = " << mahl_dist << std::endl;
        return;
    }

    Eigen::VectorXd i_vect = H.transpose () * R.inverse () * (y + H*mu);
    Eigen::MatrixXd I_mat  = H.transpose () * R.inverse () * H;
    _eta.head (sz) = _eta.head (sz) + i_vect;
    _Lambda.topLeftCorner (sz,sz) = _Lambda.topLeftCorner (sz,sz) + I_mat;
}

void perls::If::compute_cycle (int64_t dt, Input& in)
{
    std::cout << "{Information Filter} ----------- PRED CYCLE -------------------------" << std::endl;
    if (dt < 0) {  // throw exception maybe
        std::cerr << "{Information Filter} pred time occured before current estimator time"
            << std::endl;
        return;
    }
    else if (dt == 0)
        return;
    predict (dt, in);
    _utime += dt;
}

void perls::If::compute_cycle (Input &in, Meas_ptr m)
{
    std::cout << "{Information Filter} ----------- MEAS CYCLE -------------------------" << std::endl;
    int64_t dt = m->utime () - _utime;
    if (dt < 0) { 
        std::cerr << "{Information Filter} meas time occured before current estimator time"
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

void perls::If::predict_augment (int64_t dt, Input& in, const std::string& key)
{
    compute_cycle (dt, in);
    _aug_key = key;
    _augment = true;
}

void perls::If::predict_augment (Input& in, Meas_ptr m, const std::string& key)
{
    compute_cycle (in, m);
    _aug_key = key;
    _augment = true;
}

void perls::If::augment_independent (const Eigen::MatrixXd& Lambda, const Eigen::VectorXd& eta, 
        const std::string& key)
{
    std::vector<int> vbase_i = _index["base"];
    int bsz = vbase_i.size ();
    Eigen::VectorXi base_i (bsz);
    std::copy (vbase_i.begin (), vbase_i.end (), base_i.data ());

    int nsz = eta.size ();
    Eigen::VectorXi new_i (nsz);
    for (int i=0;i<nsz;++i) new_i(i) = bsz+i;

    std::vector<int> vextra_i = _index["extra"];
    int esz = vextra_i.size ();
    Eigen::VectorXi extra_i (esz);
    std::copy (vextra_i.begin (), vextra_i.end (), extra_i.data ());

    if (esz) {
        Eigen::VectorXi shift_i = extra_i.array () + nsz;

        Eigen::VectorXd e_e  = _eta (extra_i);
        Eigen::MatrixXd L_be = _Lambda (base_i, extra_i);
        Eigen::MatrixXd L_ee = _Lambda (extra_i, extra_i);

        _eta (shift_i) = e_e;
        _Lambda (shift_i,shift_i) = L_ee;
        _Lambda (new_i,shift_i)   = Eigen::MatrixXd::Zero (nsz, esz);
        _Lambda (shift_i,new_i)   = Eigen::MatrixXd::Zero (esz, nsz);
        _Lambda (base_i,shift_i)  = L_be;
        _Lambda (shift_i,base_i)  = L_be.transpose ();
    }

    _eta (new_i) = eta; 
    _Lambda (new_i,new_i)  = Lambda;
    _Lambda (new_i,base_i) = Eigen::MatrixXd::Zero (nsz, bsz);
    _Lambda (base_i,new_i) = Eigen::MatrixXd::Zero (bsz, nsz);

    _index("extra").prepend (key, eta.size ());
}

void perls::If::add_delta_state (const Eigen::MatrixXd& delta_Lambda, const Eigen::VectorXd& delta_eta, 
        const std::string& key) 
{
    int sz = delta_eta.size ()/2;

    // first augment empty space for delta
    augment_independent (Eigen::MatrixXd::Zero (sz,sz) , Eigen::VectorXd::Zero (sz), key);

    std::vector<int> k_i = _index ("extra").child (0)[""];
    std::vector<int> p_i = _index ("extra").child (1)[""];

    Eigen::MatrixXd Lpp = _Lambda (p_i,p_i);
    _Lambda (k_i,k_i) = delta_Lambda.topLeftCorner (sz,sz);
    _Lambda (k_i,p_i) = delta_Lambda.topRightCorner (sz,sz);
    _Lambda (p_i,k_i) = delta_Lambda.bottomLeftCorner (sz,sz);
    _Lambda (p_i,p_i) = delta_Lambda.bottomRightCorner (sz,sz) + Lpp;

    Eigen::VectorXd ep = _eta (p_i);
    _eta (k_i) = delta_eta.head (sz);
    _eta (p_i) = delta_eta.tail (sz) + ep;
}

void perls::If::marginalize (Index& sub_index, const std::string& key)
{
    int sz = _index.size ();

    std::vector<int> o_i = sub_index[key];
    int osz = o_i.size ();
    // should check to be sure osz < sz
    std::cout << "{Information_filter} marginalizing out index: " << o_i << std::endl;

    sub_index.remove (key); 

    Eigen::MatrixXd Lam_p;
    Eigen::VectorXd eta_p;
    marg_info_over (Lam_p, eta_p, o_i, _Lambda.topLeftCorner (sz,sz), _eta.head (sz)); 

    _Lambda.topLeftCorner (sz-osz,sz-osz) = Lam_p;
    _eta.head (sz-osz) = eta_p;
}

