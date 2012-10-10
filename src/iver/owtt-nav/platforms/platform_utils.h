#include "eigen_utils.h"
#include "perls-owtt-nav/meas.h"
#include "perls-owtt-nav/process_model.h"

const double MICROSEC_TO_SEC = 1e-6;

class Dvl_rph_meas : public perls::Meas
{
  public:
    Dvl_rph_meas (int64_t ut, const Eigen::VectorXd& raw, perls::Index& index)
        : Meas (ut, Eigen::Matrix<double,6,1>::Zero (), 
        raw, Eigen::Matrix<double,6,6>::Zero (), 
        index, std::vector<std::string> (0), "uvwrph")
    {}
    ~Dvl_rph_meas () {};

    void observation_model (const Eigen::VectorXd& state) {}
};

static int uvw[3] = {0,1,2};
const std::vector<int> uvw_i (uvw,uvw+3);  
static int rph[3] = {3,4,5};
const std::vector<int> rph_i (rph,rph+3);  

perls::Input compute_uvwrph_odo (int64_t, double, double, const Eigen::Matrix<double,6,1>&);



