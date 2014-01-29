#ifndef __LBL_H__
#define __LBL_H__

//#include <fstream>
#include <list>
#include <vector>
#include <utility>
#include <stdint.h>

#include <Eigen/Dense>

namespace perls
{
    typedef std::pair<int64_t,double> range_pair;

    class Lbl_nav
    {
      public:
        Lbl_nav (int64_t utime_win, double med_thresh);
        ~Lbl_nav () {} 

        void add_beacon (const std::vector<double>& xy, double z);

        std::vector<double> compute_fix (int64_t utime, const std::vector<double>& ranges, 
                double* seed_xy, double depth);

      private:
        unsigned int _nbeacons;
        Eigen::MatrixXd _xbeacons;
        Eigen::VectorXd _zbeacons;
        std::vector<double> least_squares_fix (int ngood, const std::vector<bool>& good, 
                const std::vector<double>& ranges, double* seed_xy);
        std::vector<double> two_range_fix (int ngood, const std::vector<bool>& good, 
                const std::vector<double>& ranges, double* seed_xy);

        std::vector<std::list<range_pair> > _ranges;
        bool median_filt (range_pair new_pair, std::list<range_pair>& pairs);

        int _utime_window;
        double _med_thresh;

        Lbl_nav ();
    };
}

#endif // __LBL_H__
