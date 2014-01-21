/**
 * @file timing.h
 * @brief Timing of code segments with display in viewer window
 * @author Michael Kaess
 * @version $Id: $
 *
 * (c) 2011 Massachusetts Institute of Technology
 *
 */

#ifndef _HAUV_TIMING_H
#define _HAUV_TIMING_H

#include <lcm/lcm.h>

#include <cmath>
#include <map>
#include <sys/time.h>

// simple class for accumulating execution timing information by name
class Timing {

  class Stats {
  public:
    double t0;
    double t_last;
    double t;
    double t_max;
    double t_min;
    int n;
  };

  std::map<std::string, Stats> stats;

public:

  void add_t0(std::string id, double t0);

  double get_t0(std::string id);

  void add_dt(std::string id, double dt);

  void print();

  double tic();

  double tic(std::string id);

  double toc(double t);

  double toc(std::string id);

  void send(lcm_t* lcm);

};

#endif
