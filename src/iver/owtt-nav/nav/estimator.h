#ifndef __ESTIMATOR_H__
#define __ESTIMATOR_H__

#include "eigen_utils.h"
#include "index.h"
#include "meas.h"
#include "process_model.h"

#define MAX_STATE_SIZE 100

namespace perls
{
    class Estimator // interface class for all estimator types
    {
      public:
        Estimator (int64_t ut, Index& index, Process_model *pm) 
            : _utime (ut), _index (index), _pmodel (pm) 
        {}
        virtual ~Estimator () {}

        virtual void compute_cycle (int64_t dt, Input& in) = 0;
        virtual void compute_cycle (Input& in, Meas_ptr m) = 0;
        virtual void predict_augment (int64_t dt, Input& in, const std::string& key) = 0;
        virtual void predict_augment (Input& in, Meas_ptr m, const std::string& key) = 0;

        virtual void marginalize (Index& sub_index, const std::string& key) {}

        virtual Eigen::VectorXd mean () const = 0;
        virtual Eigen::MatrixXd cov () const = 0;

        int64_t utime () const {return _utime;}

      protected:
        int64_t _utime;
        Index& _index;

        Process_model *_pmodel;

      private:
        Estimator (); 
    };
}

#endif // __ESTIMATOR_H__
