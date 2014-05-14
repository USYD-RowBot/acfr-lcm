#ifndef __ESTIMATOR_KALMAN_H__
#define __ESTIMATOR_KALMAN_H__

#include "estimator.h"

namespace perls
{
    class Kf : public Estimator
    {
      public:
        Kf (int64_t ut, Index& index, Eigen::VectorXd& mu0, Eigen::MatrixXd& Sigma0, Process_model *pm);
        ~Kf () {}

        void compute_cycle (int64_t dt, Input& in);
        void compute_cycle (Input& in, Meas_ptr m);
        void predict_augment (int64_t dt, Input& in, const std::string& key);
        void predict_augment (Input& in, Meas_ptr m, const std::string& key);

        Eigen::VectorXd mean () const {return _mu.head (_index.size ());}
        Eigen::MatrixXd cov () const {return _Sigma.topLeftCorner (_index.size (), _index.size ());}

      private:
        Eigen::VectorXd _mu;
        Eigen::MatrixXd _Sigma;

        void predict (int64_t dt, Input& in);
        void process_meas (Meas_ptr m);

        bool _augment;
        std::string _aug_key;

        Kf ();
    };
}

#endif // __ESTIMATOR_KALMAN_H__
