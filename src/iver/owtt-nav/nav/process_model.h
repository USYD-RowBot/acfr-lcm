#ifndef __PROCESS_MODEL_H__
#define __PROCESS_MODEL_H__

#include "eigen_utils.h"
#include "index.h"

namespace perls
{
    struct Input
    {
        Input () 
            : input_raw (Eigen::VectorXd ()), input_cov (Eigen::MatrixXd ())
        {}

        Eigen::VectorXd input_raw;        
        Eigen::MatrixXd input_cov;
    };

    class Process_model
    {
      public:
        Process_model (int sz, Index& index, std::vector<std::string> keys) 
            : _utime(0), _raw_size(sz), 
            _raw (Eigen::VectorXd ()), _cov (Eigen::VectorXd ()),
            _Jacobian (Eigen::VectorXd ()),
            _index (index), _index_keys (keys)
        {}

        ~Process_model () {}

        // transition simply computes Jacobian and fixes the covariance
        // (possibly for certain process models) --- let
        // estimator propagate the state forward
        // P(k+1) = Jacobian*P(k)*Jacobian' + Q(k)
        // x(k+1) = Jacobian*x(k) + u(k) 
        // Q(k) = in_cov
        // u(k) = in
        // raw = x(k+1), raw is simply an `observation' of the future state given current state 
        // cov = Q(k), covariance associated with the odometry
        // requires 1) state mean
        //          2) index
        // modifies 1) raw (depends on state, time, and input)
        //          2) cov (depends on state, time, and input) 
        //          3) Jacobian (depends on state, time, and input)  
        virtual void transition (int64_t ut, const Eigen::VectorXd& state, 
                const Input& odo = Input ()) = 0;

        int64_t utime () const {return _utime;}
        const Eigen::VectorXd& raw () const {return _raw;} 
        const Eigen::MatrixXd& cov () const {return _cov;}
        const Eigen::MatrixXd& Jacobian () const {return _Jacobian;}

      protected:
        int64_t _utime;

        int _raw_size;
        Eigen::VectorXd _raw;
        Eigen::MatrixXd _cov;
        Eigen::MatrixXd _Jacobian;

        Index& _index;
        std::vector<std::string> _index_keys;

      private:
        Process_model ();
    };

    // requires keys[0] --- state base that input is added to
    class Const_pos_discrete : public Process_model
    {
      public: 
        Const_pos_discrete (int sz, Index& index, std::vector<std::string> keys) 
            : Process_model (sz, index, keys)
        {}
        ~Const_pos_discrete () {}

        void transition (int64_t ut, const Eigen::VectorXd& state, const Input& odo = Input ());

      private:
        Const_pos_discrete ();
    };
}

#endif // __PROCESS_MODEL_H__
