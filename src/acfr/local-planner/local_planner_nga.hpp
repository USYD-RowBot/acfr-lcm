#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <fstream>

#include "local_planner.hpp"

#pragma once


#define DOUBLE_MAX std::numeric_limits< double >::max()
#define ITERATION_NUM 3


class LocalPlannerTunnel : public LocalPlanner
{
public:
    LocalPlannerTunnel();
    virtual ~LocalPlannerTunnel();
    virtual int loadConfig(char *programName);
    virtual int calculateWaypoints();
    virtual int processWaypoints();
    
    virtual int onPathCommand(const acfrlcm::auv_path_command_t *pc);
    virtual int onNav(const acfrlcm::auv_acfr_nav_t *nav);
    virtual int init();
    
};
