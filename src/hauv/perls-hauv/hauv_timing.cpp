/**
 * @file timing.cpp
 * @brief Timing of code segments with display in viewer window
 * @author Michael Kaess
 * @version $Id: $
 *
 * (c) 2011 Massachusetts Institute of Technology
 *
 * @note ported to perls-hauv by Ayoung Kim 2012.02.10
 */

#include <string>

#include <iostream>
#include <iomanip>
#include <sstream>

#include "perls-lcmtypes/perllcm_logbook_t.h"

#include "perls-common/timestamp.h"

#include "hauv_timing.h"


void sendInfo(lcm_t* lcm, std::string key, std::string data) {
    // one would think that modern libraries use strings to avoid such uglyness...
    /*  se_info_t info;
    char ckey[key.size()+1];
    strcpy(ckey, key.c_str());
    info.key = ckey;
    char cdata[data.size()+1];
    strcpy(cdata, data.c_str());
    info.data = cdata;
    se_info_t_publish(lcm, "SE_INFO", &info);*/

    char ckey[key.size()+1];
    strcpy(ckey, key.c_str());
    char cdata[data.size()+1];
    strcpy(cdata, data.c_str());

    // the message will appear in "utime username@hostname: message" format
    perllcm_logbook_t entry;
    entry.utime = 0;
    entry.username = strdup("");
    entry.hostname = ckey;
    entry.message = cdata;
    perllcm_logbook_t_publish (lcm, "LOGBOOK", &entry);
    free (entry.username);
}


#include <cmath>

void Timing::add_t0(std::string id, double t0) {
  stats[id].t0 = t0;
}

double Timing::get_t0(std::string id) {
  return stats[id].t0;
}

void Timing::add_dt(std::string id, double dt) {
  Stats& s = stats[id];
  s.t_last = dt;
  s.t += dt;
  s.n++;
  if (s.n==1 || s.t_max < dt) s.t_max = dt;
  if (s.n==1 || s.t_min > dt) s.t_min = dt;
}

void Timing::print() {
  std::map<std::string, Stats>::iterator it;
  for(it = stats.begin(); it!=stats.end(); it++) {
    Stats& s = it->second;
    printf("%s: %g (%i times, min: %g, max: %g)\n",
           it->first.c_str(), s.t, s.n, s.t_min, s.t_max);
  }
}

double Timing::tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

double Timing::tic(std::string id) {
  double t0 = tic();
  this->add_t0(id, t0);
  return t0;
}

double Timing::toc(double t) {
  double s = this->tic();
  return (std::max(0., s-t));
}

double Timing::toc(std::string id) {
  double dt = this->toc(get_t0(id));
  add_dt(id, dt);
  return dt;
}

void Timing::send(lcm_t* lcm) {
  std::map<std::string, Stats>::iterator it;
  for (it = stats.begin(); it!=stats.end(); it++) {
    Stats& st = it->second;
    std::stringstream s;
    s << std::setiosflags(std::ios::fixed) << std::setprecision(4);
    s << st.t_last;
    s << std::setiosflags(std::ios::fixed) << std::setprecision(0);
    s << " (num: " << st.n;
    s << std::setiosflags(std::ios::fixed) << std::setprecision(4);
    s << ", sum: " << st.t << ", avg: " << st.t/(double)st.n << ", max: " << st.t_max << ")";
    sendInfo(lcm, it->first, s.str());
  }
}
