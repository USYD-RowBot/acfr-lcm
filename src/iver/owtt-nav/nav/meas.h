#ifndef __MEAS_H__
#define __MEAS_H__

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <vector>

#include "eigen_utils.h"
#include "index.h"

#define XFRM_SIZE 6

namespace perls
{

    // shared pointers might be slow-ish, but this is probably the safest
    // way to handle pointers to meas objects
    class Meas;
    typedef boost::shared_ptr<Meas> Meas_ptr;

    class Meas 
    {
      public:
        // TODO: how to ensure index is packed correctly for index
        Meas (int64_t ut, double *x_vs, int raw_size, double *raw, double *cov, 
                Index& index, std::vector<std::string> keys, std::string type);
        Meas (int64_t ut, const Eigen::VectorXd& x_vs, 
                const Eigen::VectorXd& raw, const Eigen::MatrixXd& cov, 
                Index& index, std::vector<std::string> keys, std::string type);
        virtual ~Meas () { }

        int64_t utime () const {return _utime;}
        const Eigen::VectorXd& raw () const {return _raw;} 
        const Eigen::MatrixXd& cov () const {return _cov;}
        const Eigen::MatrixXd& Jacobian () const {return _Jacobian;}
        const Eigen::VectorXd& prediction () const {return _prediction;}

        // observation model simply computes Jacobian, prediction, and `fixes'
        // z,R --- let estimator compute innovation
        // z_predict = h(state)
        // innovation = z_fix - z_predict
        // requires 1) state mean
        //          2) index
        // modifies 1) meas (z_fix)
        //          2) meas cov (R_fix, may depend on state)
        //          3) Jacobian (size depends on Index)
        virtual void observation_model (const Eigen::VectorXd& state) = 0;

        const std::string& type () const {return _meas_type;}

        // index helpers
        static const std::vector<int> ssc_xyzrph_i;
        static const std::vector<int> ssc_xyz_i; 
        static const std::vector<int> ssc_rph_i; 
        static const std::vector<int> ssc_xij_i;
        static const std::vector<int> ssc_xjk_i;

      protected:
        int64_t _utime;

        Eigen::VectorXd _x_vs;

        int _raw_size;
        Eigen::VectorXd _raw;
        Eigen::MatrixXd _cov;
        Eigen::MatrixXd _Jacobian;
        Eigen::VectorXd _prediction;

        // how can we ensure that the index is packed correctly?
        // we could require that each measurement have a certain state index string,
        // e.g., Ms_gx3 requires "rph" index map key
        Index& _index;
        std::vector<std::string> _index_keys;
        std::string _meas_type;

    };

    std::ostream& operator<< (std::ostream& os, const Meas& meas);
}

#endif // __MEAS_H__
