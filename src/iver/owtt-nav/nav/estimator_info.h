#ifndef __ESTIMATOR_INFORMATION_H__
#define __ESTIMATOR_INFORMATION_H__

#include "utils.h"
#include "estimator.h"

namespace perls
{
    class If : public Estimator
    {
      public:
        If (int64_t ut, Index& index, Eigen::VectorXd& mu0, Eigen::MatrixXd& Sigma0, Process_model *pm, bool is_lin);
        ~If () {}

        void compute_cycle (int64_t dt, Input& in);
        void compute_cycle (Input& in, Meas_ptr m);

        void predict_augment (int64_t dt, Input& in, const std::string& key);
        void predict_augment (Input& in, Meas_ptr m, const std::string& key);
        void augment_independent (const Eigen::MatrixXd& Lambda, const Eigen::VectorXd& eta, const std::string& key);

        void add_delta_state (const Eigen::MatrixXd& delta_Lambda, const Eigen::VectorXd& delta_eta, 
                const std::string& key); 

        void marginalize (Index& sub_index, const std::string& key);

        Eigen::VectorXd mean () const 
        {
            return ldlt_inverse (_Lambda.topLeftCorner (_index.size (),_index.size ()))*_eta.head (_index.size ());
        }
        Eigen::MatrixXd cov () const 
        {
            return ldlt_inverse (_Lambda.topLeftCorner (_index.size (),_index.size ()));
        }

        Eigen::MatrixXd Lambda () const {return _Lambda.topLeftCorner (_index.size (),_index.size ());}
        Eigen::VectorXd eta () const {return _eta.head (_index.size ());}

      private:
        Eigen::VectorXd _eta;
        Eigen::MatrixXd _Lambda;
        
        bool _is_linear;

        void predict (int64_t dt, Input& in);
        void process_meas (Meas_ptr m);

        bool _augment;
        std::string _aug_key;

        If ();
    };
}

#endif // __ESTIMATOR_INFORMATION_H__
