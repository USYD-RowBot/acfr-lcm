#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <fstream>

#include "local_planner.hpp"
#include "acfr-common/socket.h"

#include "perls-lcmtypes++/senlcm/micron_ping_t.hpp"

#pragma once


#define DOUBLE_MAX std::numeric_limits< double >::max()
#define ITERATION_NUM 3

#define AUV_MSG_PORT 12345

// This is what is used by Sirius to represent no value
#define NO_VALUE -98765

class LocalPlannerSirius : public LocalPlanner
{
public:
    LocalPlannerSirius();
    virtual ~LocalPlannerSirius() = default;
    
    virtual int onPathCommand(const acfrlcm::auv_path_command_t *pc);
    virtual int onNav(const acfrlcm::auv_acfr_nav_t *nav);
    virtual int init();
    virtual int execute_abort();
    
private:
    udp_info_t auv_udp;
    void heartbeat_callback(const lcm::ReceiveBuffer*rbuf, const std::string& channel,
                            const perllcm::heartbeat_t *hb);
                            
};


