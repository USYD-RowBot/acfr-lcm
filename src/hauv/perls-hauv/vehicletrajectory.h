#ifndef _HAUV_VEHICLETRAJECTORY_H
#define _HAUV_VEHICLETRAJECTORY_H

#include <vector>
#include <lcm/lcm.h>

#include "perls-lcmtypes++/hauv/bs_rnv_2_t.hpp"
#include "perls-lcmtypes++/hauv/bs_cnv_t.hpp"
#include "perls-lcmtypes++/hauv/bs_dvl_2_t.hpp"
#include "perls-lcmtypes++/hauv/bs_pit_t.hpp"
#include "perls-lcmtypes++/bot_core/image_t.hpp"

//#include "pointcloud.h"

struct State
{
    int64_t utime;
    int64_t img_utime;

    double x,y,z;
    double roll,pitch,heading;

    double rel_x,rel_y,rel_heading;

    double pitch_dvl, pitch_sonar;
    double sonar_range;    
    
    double altitude;
    
    bool   valid_normal;
    double nx,ny,nz;
    double ndist;
};

/**
 * Listens to incoming HAUV messages and provides a 
 * continuous trajectory view of the messages.
 * Linear interpolation is used to combine the different 
 * messages.
 */
class VehicleTrajectory
{
public:
    /**
     * Returns a combined state at time utime.
     */
    State getState(int64_t utime);

    int64_t getOldest() const;
    int64_t getLatest() const;
    
    bool canInterpolate(int64_t utime) const;
    
    void add(const hauv::bs_rnv_2_t* rnv);
    void add(const hauv::bs_dvl_2_t* dvl);
    void add(const hauv::bs_cnv_t* cnv);
    void add(const hauv::bs_pit_t* pit);
private:    
    template<class T>
    void add(std::vector<T> & v, T m); 
  
    template<class T>
    int find(std::vector<T> & v, int64_t utime);

    // Helper functions to read of time from the different HAUV messages
    static int64_t getTime(const hauv::bs_rnv_2_t & v) {return v.time_received;}
    static int64_t getTime(const hauv::bs_dvl_2_t & v) {return v.time_received;}
    static int64_t getTime(const hauv::bs_cnv_t & v) {return v.time_received;}
    static int64_t getTime(const hauv::bs_pit_t & v) {return v.time_received;}
    static int64_t getTime(const int64_t & v) {return v;}

    /** Stores the HAUV messages in order */
    std::vector<hauv::bs_rnv_2_t> m_rnv_list;
    std::vector<hauv::bs_dvl_2_t> m_dvl_list;
    std::vector<hauv::bs_cnv_t> m_cnv_list;
    std::vector<hauv::bs_pit_t> m_pit_list;
};

#endif
