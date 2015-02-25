#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <bot_param/param_client.h>
#include "perls-common/serial.h"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_status_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_command_t.hpp"
#include "evologics.hpp"

#ifndef EVOLOGICS_MODEM_H
#define EVOLOGICS_MODEM_H

#define MAX_TARGETS 8

using namespace senlcm;
using namespace perllcm;
using namespace acfrlcm;
using namespace std;


class Evologics_Modem
{
    public:
        Evologics_Modem();
        lcm::LCM *lcm;
        int load_config(char *program_name);
        int init();
        int process();
        int keep_alive();
        int send_status(const auv_status_t *status);
        int get_usbl_address();
        Evologics *evo;
        
    private:
        
        // serial io
        char *parity;
        int baud;
        char *device;
        bool use_serial_comm;
        // ip io
        int open_port();
        char *ip;
        char *port;
        bool use_ip_comm;
        char term;

        int evo_fd;
        int keep_alive_count;
        char **lcm_channels;
        int usbl_address;
        int local_address;
        int gain;
        int source_level;
        bool auto_gain;
        
        int targets[MAX_TARGETS];
        int num_targets;
        int ping_period;
        int ping_counter;
        int ping_timeout;
};

#endif
