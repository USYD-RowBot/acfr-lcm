#ifndef __MEAS_SENSORS_H__
#define __MEAS_SENSORS_H__

#include "meas.h"

namespace perls
{
    // requires keys[0] --- rph state elements
    class Rph_meas : public Meas
    {
      public:
        Rph_meas (int64_t ut, double *x_vs, int raw_size, double *raw, double *raw_cov,
                Index& index, std::vector<std::string> keys)
            : Meas (ut, x_vs, raw_size, raw, raw_cov, index, keys, "ms-gx3")
        {}

        Rph_meas (int64_t ut, const Eigen::VectorXd& x_vs, 
                const Eigen::VectorXd& raw, const Eigen::MatrixXd& raw_cov,
                Index& index, std::vector<std::string> keys)
            : Meas (ut, x_vs, raw, raw_cov, index, keys, "ms-gx3")
        {}
        ~Rph_meas () {};

        void observation_model (const Eigen::VectorXd& state);
    };

    // requires keys[0] --- uvw state elements
    //          keys[1] --- abc state elements
    class Rdi_meas: public Meas
    {
      public:
        Rdi_meas (int64_t ut, double *x_vs, int raw_size, double *raw, double *raw_cov,
                Index& index, std::vector<std::string> keys)
            : Meas (ut, x_vs, raw_size, raw, raw_cov, index, keys, "rdi")
        {}

        Rdi_meas (int64_t ut, const Eigen::VectorXd& x_vs, 
                const Eigen::VectorXd& raw, const Eigen::MatrixXd& raw_cov,
                Index& index, std::vector<std::string> keys)
            : Meas (ut, x_vs, raw, raw_cov, index, keys, "rdi")
        {}
        ~Rdi_meas () {};

        void observation_model (const Eigen::VectorXd& state);
    };

    // requires keys[0] --- xy state elements
    class Gps_meas: public Meas
    {
      public:
        Gps_meas (int64_t ut, double *x_vs, int raw_size, double *raw, double *raw_cov,
                Index& index, std::vector<std::string> keys)
            : Meas (ut, x_vs, raw_size, raw, raw_cov, index, keys, "gps")
        {}

        Gps_meas (int64_t ut, const Eigen::VectorXd& x_vs, 
                const Eigen::VectorXd& raw, const Eigen::MatrixXd& raw_cov,
                Index& index, std::vector<std::string> keys)
            : Meas (ut, x_vs, raw, raw_cov, index, keys, "gps")
        {}
        ~Gps_meas () {};

        void observation_model (const Eigen::VectorXd& state);
    };

    class Owtt_meas: public Meas
    {
      public:
        Owtt_meas (int64_t ut, double *x_vs, int raw_size, double *raw, double *raw_cov,
                Index& index, std::vector<std::string> keys)
            : Meas (ut, x_vs, raw_size, raw, raw_cov, index, keys, "owtt")
        {}

        Owtt_meas (int64_t ut, const Eigen::VectorXd& x_vs, 
                const Eigen::VectorXd& raw, const Eigen::MatrixXd& raw_cov,
                Index& index, std::vector<std::string> keys)
            : Meas (ut, x_vs, raw, raw_cov, index, keys, "owtt")
        {}
        ~Owtt_meas () {};

        void observation_model (const Eigen::VectorXd& state);
    };
}

#endif // __MEAS_SENSORS_H__
