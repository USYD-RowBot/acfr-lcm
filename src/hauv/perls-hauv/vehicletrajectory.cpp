#include "vehicletrajectory.h"
#include <iostream>
#include <cmath>
#include <limits>

using std::numeric_limits;

bool VehicleTrajectory::canInterpolate(int64_t utime) const
{
    if (   m_rnv_list.size() == 0 
        && m_cnv_list.size() == 0 
        && m_pit_list.size() == 0) return false;

    int64_t minrnv, mincnv, minpit;

    if (m_rnv_list.size() == 0) 
        minrnv = 0;
    else
        minrnv = getTime(m_rnv_list.front());

    if (m_cnv_list.size() == 0) 
        mincnv = 0;
    else
        mincnv = getTime(m_cnv_list.front());
    
    if (m_pit_list.size() == 0) 
        minpit = 0;
    else
        minpit = getTime(m_pit_list.front());

    return     getLatest()>= utime 
            && utime>minrnv
            && utime>mincnv
            && utime>minpit;
}

int64_t VehicleTrajectory::getOldest() const
{
    if (   m_rnv_list.size() == 0
        && m_cnv_list.size() == 0
        && m_pit_list.size() == 0) return 0;

    int64_t minrnv, mincnv, minpit;

    if (m_rnv_list.size() == 0) 
        minrnv = 0;
    else
        minrnv = getTime(m_rnv_list.front());

    if (m_cnv_list.size() == 0) 
        mincnv = 0;
    else
        mincnv = getTime(m_cnv_list.front());
    
    if (m_pit_list.size() == 0) 
        minpit = 0;
    else
        minpit = getTime(m_pit_list.front());
    
    int64_t maxtime = std::max(minrnv,
                      std::max(mincnv,
                               minpit));

    int64_t maxrnv, maxcnv, maxpit;

    if (m_rnv_list.size() == 0) 
        maxrnv = numeric_limits<int64_t>::max();
    else
        maxrnv = getTime(m_rnv_list.back());

    if (m_cnv_list.size() == 0) 
        maxcnv = numeric_limits<int64_t>::max();
    else
        maxcnv = getTime(m_cnv_list.back());
    
    if (m_pit_list.size() == 0) 
        maxpit = numeric_limits<int64_t>::max();
    else
        maxpit = getTime(m_pit_list.back());

    if (maxtime > maxrnv || maxtime > maxcnv || maxtime > maxpit) return 0;

    return maxtime;
}

int64_t VehicleTrajectory::getLatest() const
{
    if (   m_rnv_list.size() == 0 
        && m_cnv_list.size() == 0 
        && m_pit_list.size() == 0 ) return 0;

    int64_t maxrnv, maxcnv, maxpit;

    if (m_rnv_list.size() == 0) 
        maxrnv = numeric_limits<int64_t>::max();
    else
        maxrnv = getTime(m_rnv_list.back());

    if (m_cnv_list.size() == 0) 
        maxcnv = numeric_limits<int64_t>::max();
    else
        maxcnv = getTime(m_cnv_list.back());
    
    if (m_pit_list.size() == 0) 
        maxpit = numeric_limits<int64_t>::max();
    else
        maxpit = getTime(m_pit_list.back());  

    int64_t mintime = std::min(maxrnv,
                      std::min(maxcnv,
                               maxpit));
                               
    int64_t minrnv, mincnv, minpit;

    if (m_rnv_list.size() == 0) 
        minrnv = 0;
    else
        minrnv = getTime(m_rnv_list.front());

    if (m_cnv_list.size() == 0) 
        mincnv = 0;
    else
        mincnv = getTime(m_cnv_list.front());
    
    if (m_pit_list.size() == 0) 
        minpit = 0;
    else
        minpit = getTime(m_pit_list.front());
                  
    if (mintime < minrnv || mintime < mincnv || mintime < minpit) return 0;
    
    return mintime;
}

template<class T>
int VehicleTrajectory::find(std::vector<T> & v, int64_t utime)
{
    int a = 0;
    int b = v.size();
    int s = b;
    int64_t x = utime;
    while(a!=b)
    {
        //  |  <x  | ?? | >x  |
        //   0      a    b     n        
        int m = (a+b)/2;
	//	assert(m>=0 && m<v.size());
        if (getTime(v[m]) > x) b = m;
        else if (getTime(v[m]) < x) a = m+1;
        else return m;
    }
    //  |  <x  |=x| ?? | >x  |
    //   0       a      b     n

    // kaess: above function can return value beyond end of vector:
    // e.g. if v.size==1 and time of single entry is less than
    // requested utime, a=m+1, which will be equal to v.size
    if (s!=0 && a>=s) a = a-1; 

    return a;
}

State VehicleTrajectory::getState(int64_t utime)
{
    State s;
    s.valid_normal = false;
    s.utime = 0;
    
    if (!canInterpolate(utime)) 
      {
	// kaess: initialize to avoid unpredictable behavior if
	// utime==0 is not checked by calling function; disable to
	// find such places using valgrind
	s.rel_x = 0;
	s.rel_y = 0;
	s.rel_heading = 0;
	s.x = 0;
	s.y = 0;
	s.z = 0;
	s.roll = 0;
	s.pitch = 0;
	s.heading = 0;
	s.altitude = 0;
      }
    else
      {
        s.utime = utime;
        int idx_rnv = find(m_rnv_list, utime);
        if (getTime(m_rnv_list[idx_rnv]) == utime) {
            // @todo rotate with DVL          
            s.rel_x = m_rnv_list[idx_rnv].vertical;
            s.rel_y = m_rnv_list[idx_rnv].horizontal;
            s.rel_heading = m_rnv_list[idx_rnv].heading;
            s.z = m_rnv_list[idx_rnv].depth;
            s.roll = m_rnv_list[idx_rnv].absroll;
            s.pitch = m_rnv_list[idx_rnv].abspitch;
            s.heading = m_rnv_list[idx_rnv].absheading;            
            s.altitude = m_rnv_list[idx_rnv].distance;
        } else {
            hauv::bs_rnv_2_t a = m_rnv_list[idx_rnv-1];
            hauv::bs_rnv_2_t b = m_rnv_list[idx_rnv];
            double t = (double)(utime-getTime(a))/(getTime(b)-getTime(a));

            s.rel_x = a.vertical*(1.0-t) + b.vertical*t;
            s.rel_y = a.horizontal*(1.0-t) + b.horizontal*t;
            s.rel_heading = a.heading*(1.0-t) + b.heading*t;
            s.z = a.depth*(1.0-t) + b.depth*t;
            s.roll = a.absroll*(1.0-t) + b.absroll*t;
            s.pitch = a.abspitch*(1.0-t) + b.abspitch*t;
            s.heading = a.absheading*(1.0-t) + b.absheading*t;                        
            s.altitude = a.distance*(1.0-t) + b.distance*t;
        }

        int idx_cnv = find(m_cnv_list, utime);
        if (getTime(m_cnv_list[idx_cnv]) == utime) {
            s.x = m_cnv_list[idx_cnv].x;
            s.y = m_cnv_list[idx_cnv].y;
        } else {
            hauv::bs_cnv_t a = m_cnv_list[idx_cnv-1];
            hauv::bs_cnv_t b = m_cnv_list[idx_cnv];
            double t = (double)(utime-getTime(a))/(getTime(b)-getTime(a));
          
            s.x = a.x*(1.0-t) + b.x*t;
            s.y = a.y*(1.0-t) + b.y*t;
        }

        int idx_pit = find(m_pit_list, utime);
        if (m_pit_list[idx_pit].time == utime) {
            s.pitch_sonar = m_pit_list[idx_pit].pitch_sonar;          
            s.pitch_dvl = m_pit_list[idx_pit].pitch_dvl;          
        } else {
            hauv::bs_pit_t a = m_pit_list[idx_pit-1];
            hauv::bs_pit_t b = m_pit_list[idx_pit];
            double t = (double)(utime-getTime(a))/(getTime(b)-getTime(a));
            
            s.pitch_sonar = a.pitch_sonar*(1.0-t) + b.pitch_sonar*t;
            s.pitch_dvl = a.pitch_dvl*(1.0-t) + b.pitch_dvl*t;
        }
#if 0 // todo
        // populate the normal if we can (not included in
        // canInterpolate() because often not available)
        int idx_normal = find(m_normal_list, utime);
        // warning: idx might be beyond end of array
        if ( m_normal_list.size() > 0 && idx_normal > 0
          && abs (getTime(m_normal_list[idx_normal-1]) - utime) < 1e6) {
            s.nx = m_normal_list[idx_normal].nx;
            s.ny = m_normal_list[idx_normal].ny;
            s.nz = m_normal_list[idx_normal].nz;
            s.ndist = m_normal_list[idx_normal].ndist;
            s.valid_normal = m_normal_list[idx_normal].valid;
        } else {
            // or interpolate normal??
            s.valid_normal = false;
        }
#endif
    } // if can interpolate  
    
    return s;
}

void VehicleTrajectory::add(const hauv::bs_rnv_2_t* rnv)
{
    add(m_rnv_list, *rnv);
}

void VehicleTrajectory::add(const hauv::bs_dvl_2_t* dvl)
{
    add(m_dvl_list, *dvl);
}

void VehicleTrajectory::add(const hauv::bs_cnv_t* cnv)
{
    add(m_cnv_list, *cnv);
}

void VehicleTrajectory::add(const hauv::bs_pit_t* pit)
{
    add(m_pit_list, *pit);
}

template<class T>
void VehicleTrajectory::add(std::vector<T> & v, T m)
{
    // We are assuming the messages are received in time order
    v.push_back(m);
}
