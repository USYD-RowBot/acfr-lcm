#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <deque>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <cmath>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <zlib.h>
#include <bot_param/param_client.h>
#include <small/Pose3D.hh>
//#include "evologics.hpp"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/senlcm/ahrs_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
//#include "perls-lcmtypes++/acfrlcm/auv_global_planner_t.hpp"

using namespace std;
using namespace senlcm;
using namespace perllcm;
//using namespace acfrlcm;

#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI
#define MAX_BUF_LEN 1024
#define AHRS_PORT "10000"

#define MAX_TARGETS 8

class Evologics_AHRS
{
    public:
        Evologics_AHRS();
        ~Evologics_AHRS();
        int load_config(char *program_name);
        int init();
        int process();
        int chop_string(char *data, char **tokens, int nTokens);
        int process_ahrs_message(char *buf);
        //int on_lcm_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel, bool use_pbm = false);
        
        // data holders
        //gpsd3_t gpsd;
        //deque<novatel_t *> novatelq;
        //queue<evologics_usbl_t *> fixq;
        //novatel_t novatel;
        ahrs_t ahrs;
        
        //Evologics *evo;
        
        int reopen_port();
        int open_port(const char *ip, const char *port);

        int process_lcm_data(unsigned char *d, int len);
        int process_ahrs_data(char *d, int len, int64_t timestamp);
        int clear_ahrs();

        // LCM handlers
        int handle_heartbeat();
        int start_handlers();
        
        int ahrs_fd;
        
        int thread_exit;
        
    private:
        // ip io
        int open_port();
        char *ip;
        char *port;
        bool use_ip_comm;
        char term;

        lcm::LCM *lcm;

        SMALL:Pose3d attitude_offset;
};
         
