#include "perls-math/gsl_util_math.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "index.h"
#include "meas.h"
#include "utils.h"

static int ssc_xyzrph[6] = {0,1,2,3,4,5};
const std::vector<int> perls::Meas::ssc_xyzrph_i (ssc_xyzrph,ssc_xyzrph+6);
static int ssc_xyz[3] = {0,1,2};
const std::vector<int> perls::Meas::ssc_xyz_i (ssc_xyz,ssc_xyz+3);  
static int ssc_rph[3] = {3,4,5};
const std::vector<int> perls::Meas::ssc_rph_i (ssc_rph,ssc_rph+3);  
static int ssc_xij[6] = {0,1,2,3,4,5};
const std::vector<int> perls::Meas::ssc_xij_i (ssc_xij,ssc_xij+6);
static int ssc_xjk[6] = {6,7,8,9,10,11};
const std::vector<int> perls::Meas::ssc_xjk_i (ssc_xjk,ssc_xjk+6);

perls::Meas::Meas (int64_t ut, double *x_vs, int raw_size, double *raw, double *cov, 
        perls::Index& index, std::vector<std::string> keys, std::string type) 
    : _utime (ut), _raw_size (raw_size), _index (index), _index_keys (keys), _meas_type (type)
{
    _x_vs = Eigen::Map<Eigen::VectorXd>(x_vs, XFRM_SIZE);
    _raw = Eigen::Map<Eigen::VectorXd>(raw, raw_size);
    _cov = Eigen::Map<Eigen::MatrixXd>(cov, raw_size, raw_size);
}

perls::Meas::Meas (int64_t ut, const Eigen::VectorXd& x_vs, 
        const Eigen::VectorXd& raw, const Eigen::MatrixXd& cov,
        perls::Index& index, std::vector<std::string> keys, std::string type)
    : _utime (ut), _x_vs (x_vs), _raw_size (raw.size ()), _raw (raw), _cov (cov), 
    _index (index), _index_keys (keys), _meas_type (type)
{}

std::ostream& perls::operator<< (std::ostream& os, const perls::Meas& meas)
{
    os << "Meas (" << meas.type () << "): " << std::endl
        << "\tutime = " << meas.utime () << std::endl
        << "\traw = " << std::endl << meas.raw () << std::endl
        << "\tcov = " << std::endl << meas.cov () << std::endl;
    return os;
}

