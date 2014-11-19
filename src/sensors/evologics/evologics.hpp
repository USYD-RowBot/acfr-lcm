#include <lcm/lcm-cpp.hpp>
#include <small/Pose3D.hh>
#include <proj_api.h>
#include <pthread.h>
#include <unistd.h>
#include <zlib.h>
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/senlcm/evologics_usbl_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_usbl_angles_t.hpp"

#ifndef EVOLOGICS_H
#define EVOLOGICS_H

using namespace senlcm;
using namespace std;


#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI
#define MAX_BUF_LEN 1024

int chop_string(char *data, char **tokens);

class Evologics
{
    public:
        Evologics(int _fd, char _term, lcm::LCM *lcm);
        ~Evologics();
        int send_lcm_data(unsigned char *d, int size, int target, char *dest_channel);
        int send_command(const char *d);
        int send_ping(int target);
        int parse_lcm_data(unsigned char *d, int len);
        int parse_modem_data(char *d, int len, int64_t timestamp);
        int clear_modem();
        bool sending_im;
        bool sending_data;
        bool sending_command;
        int thread_exit;
        int fd;
        char term;
                
    private:
        pthread_t read_thread_id;
        lcm::LCM *lcm;
        // channel flags
        pthread_mutex_t flags_lock;
        int current_target;
        
        
        // message parsers
        int parse_usbllong(char *d, int64_t timestamp);
        int parse_usblangles(char *d, int64_t timestamp);
        int parse_im(char *d);
};
        
#endif
