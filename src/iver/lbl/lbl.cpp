#include <iostream>
#include <algorithm>
#include <cmath>

#include "lbl.h"

#define PERLS_LBL_MIN_RANGES_REQUIRED 3
#define PERLS_LBL_FIX_MAX_ITERS 10
#define PERLS_LBL_FIX_TOL 1e-3
#define PERLS_LBL_SOS 1500.0

perls::Lbl_nav::Lbl_nav (int64_t utime_win, double med_thresh)
    :_nbeacons (0), _xbeacons (), 
    _zbeacons (),
    _ranges (0), 
    _utime_window (utime_win), _med_thresh (med_thresh)
{}

void perls::Lbl_nav::add_beacon (const std::vector<double>& xy, double z)
{
    _nbeacons++;
    _ranges.push_back (std::list<range_pair> (0) );

    _xbeacons.conservativeResize (2,_nbeacons);
    _zbeacons.conservativeResize(_nbeacons);
    
    _xbeacons(0,_nbeacons-1) = xy[0];
    _xbeacons(1,_nbeacons-1) = xy[1];
    _zbeacons(_nbeacons-1) = z;  // add depth
    //std::cout << "depth for beacon" << (_nbeacons-1) << ": " << _zbeacons(_nbeacons-1) << std::endl;
}

double median (std::list<perls::range_pair>& pairs)
{
    size_t size = pairs.size ();

    std::vector<double> ranges (size);
    std::list<perls::range_pair>::iterator it;
    unsigned int i = 0;
    for (it = pairs.begin (); it!=pairs.end ();++i,++it) ranges[i] = it->second;

    std::sort (ranges.begin (), ranges.end ());
    double med;
    if (size%2 == 0) med = (ranges[size/2 - 1] + ranges[size/2])/2;
    else med = ranges[size/2];

    return med;
}

bool perls::Lbl_nav::median_filt (range_pair new_pair, std::list<range_pair>& pairs)
{
    while (true) {
        if (pairs.size () <= 0) return true;
        std::pair<int64_t,double>& elem = pairs.front ();
        if (_utime_window < new_pair.first - elem.first) pairs.pop_front ();
        else break;
    }

    double med = median (pairs);
    if (std::fabs (new_pair.second - med) > _med_thresh) return false;

    return true;
}

void predict_meas_Jacob (Eigen::VectorXd& h, Eigen::MatrixXd& J, const Eigen::VectorXd& xv, const Eigen::MatrixXd &Xb)
{
    for (int i=0;i<h.size ();++i) {
        Eigen::Vector2d xy_diff = xv - Xb.col(i); //is this 2D? looks like yes, so no need to change
        h(i) = xy_diff.norm ();  //predicted range based on vehicle and beacon positions
        J.row (i) = -std::pow (xy_diff.squaredNorm (),-0.5) * xy_diff.transpose ();
    }
}

std::vector<double> perls::Lbl_nav::least_squares_fix (int ngood, const std::vector<bool>& good, 
        const std::vector<double>& ranges, double* seed_xy)
{
    Eigen::Vector2d xv;
    xv = Eigen::Map<Eigen::VectorXd>(seed_xy, 2);
    std::vector<double> fix;

    Eigen::MatrixXd Xb (2, ngood);
    Eigen::MatrixXd J (ngood, 2);
    Eigen::VectorXd h (ngood);
    Eigen::VectorXd r (ngood);
    for (unsigned int i=0,j=0;i<ranges.size ();++i)
        if (good[i]) {
            Xb.col(j) = _xbeacons.col(i);
            r(j) = ranges[i];
            ++j;
        }
   
    for (int iter_count=0;;++iter_count) {
        predict_meas_Jacob (h, J, xv, Xb);
        Eigen::Vector2d step = (J.transpose () * J).inverse () * J.transpose () * (PERLS_LBL_SOS*r - h);
        xv = xv - step;

        if (step.norm () < PERLS_LBL_FIX_TOL) {
            std::cout << "found nav least-squares solution, xv = " << std::endl
                << xv << std::endl;
            break;
        }

        if (iter_count > PERLS_LBL_FIX_MAX_ITERS) {
            std::cout << "exceeded PERLS_LBL_FIX_MAX_ITERS!" << std::endl;
            return fix;
        }
    }

    fix.resize (2);
    std::copy (fix.begin (), fix.end (), xv.data ());
    return fix;
}

std::vector<double> perls::Lbl_nav::two_range_fix (int ngood, const std::vector<bool>& good, 
        const std::vector<double>& ranges, double* seed_xy)
{
    Eigen::Vector2d xv;
    xv = Eigen::Map<Eigen::VectorXd>(seed_xy, 2);

    Eigen::MatrixXd Xb (2, ngood);
    Eigen::VectorXd r (ngood);
    for (unsigned int i=0,j=0;i<ranges.size ();++i)
        if (good[i]) {
            Xb.col(j) = _xbeacons.col(i);
            r(j) = ranges[i];
            ++j;
        }
        
    Eigen::Vector2d beac_diff = Xb.col(0) - Xb.col(1);
    double d = beac_diff.norm (); //distance between beacon positions

    //TODO: check to make sure circles intersect

    double a = (std::pow (PERLS_LBL_SOS*r(0),2) - std::pow(PERLS_LBL_SOS*r(1),2) + std::pow(d,2)) / (d*2.0);
    double b = std::sqrt (std::pow (PERLS_LBL_SOS*r(0),2) - std::pow (a,2));
    
    Eigen::Vector2d beac_between = Xb.col(0) + a * (Xb.col(1) - Xb.col(0)) / d;

    Eigen::Vector2d sol1, sol2;
    
    // compute the two possible intersections
    sol1(0) = beac_between(0) + b * (Xb(1,1) - Xb(1,0)) / d;
    sol2(0) = beac_between(0) - b * (Xb(1,1) - Xb(1,0)) / d;
    sol1(1) = beac_between(1) - b * (Xb(0,1) - Xb(0,0)) / d;
    sol2(1) = beac_between(1) + b * (Xb(0,1) - Xb(0,0)) / d;
    
    // return which intersection is closer to the seed_xy
    Eigen::Vector2d sol1_err = xv - sol1;
    Eigen::Vector2d sol2_err = xv - sol2;
    //std::cout << "sol1 err = " << sol1_err.norm () << std::endl;
    //std::cout << "sol2 err = " << sol2_err.norm () << std::endl;
    
    std::vector<double> fix (2);
    if (sol1_err.norm () <= sol2_err.norm ()){
        std::copy (fix.begin (), fix.end (), sol1.data ());
        //std::cout << "two range: solution 1" << std::endl;
        std::cout << "found nav two-range solution, xv = " << std::endl
                << sol1 << std::endl;
    }
    else {
        std::copy (fix.begin (), fix.end (), sol2.data ());
        //std::cout << "two range: solution 2" << std::endl;
        std::cout << "found nav two-range solution, xv = " << std::endl
                << sol2 << std::endl;
    }
    return fix;
}


std::vector<double> perls::Lbl_nav::compute_fix (int64_t utime, const std::vector<double>& ranges, 
        double* seed_xy, double depth)
{
    std::vector<double> fix;

    std::vector<double> ranges_d (ranges.size ()); //vector of depth-adjusted ranges
    if (ranges.size () < _nbeacons) {
        std::cerr << "recieved " << ranges.size () 
            << " when we should have at least " << _nbeacons << "!" 
            << std::endl;
        return fix;
    }

    std::cout << "received raw ranges: ";
    for (unsigned int i=0;i<ranges.size ();++i) std::cout << ranges[i] << "\t";
    std::cout << std::endl;
    
    std::vector<bool> good_returns (_nbeacons, false);
    int ngood = 0;
    for (unsigned int i=0;i<_nbeacons;++i) {
        if (std::isnan (ranges[i])) continue;
        
        //make depth-adjusted ranges
        double depth_diff = std::fabs(depth - _zbeacons[i]);
        ranges_d[i] = std::sqrt (std::pow (ranges[i],2) - std::pow (depth_diff/PERLS_LBL_SOS,2));
        //std::cout << "depth-adjusted range" << i << " = " << ranges_d[i] << std::endl;
        
        //median filter
        range_pair new_pair = std::make_pair (utime, ranges_d[i]);
        if (median_filt (new_pair, _ranges[i])) {
            _ranges[i].push_back (new_pair);
            if (_ranges[i].size () > PERLS_LBL_MIN_RANGES_REQUIRED) {
                good_returns[i] = true;
                ++ngood;
            }
        }
        
        //Yoerger: surface bounce and wrap tests here
    }

    //Yoerger: if >2 good ranges, do least-squares, then residual test
    //Yoerger: if 2 good ranges, choose baseline, then compute fix directly
    if (ngood == 2) 
        fix = two_range_fix (ngood, good_returns, ranges_d, seed_xy);      
    else if (ngood > 2) 
        fix = least_squares_fix (ngood, good_returns, ranges_d, seed_xy);      

    if (!fix.empty ()) 
        std::cout << utime << "\t" << fix[0] << "\t" << fix[1] << "\t" 
            << seed_xy[0] << "\t" << seed_xy[1] << std::endl;

    return fix;
}
