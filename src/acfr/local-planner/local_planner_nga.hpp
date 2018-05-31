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
};
