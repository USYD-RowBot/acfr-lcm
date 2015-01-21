#include <lcm/lcm-cpp.hpp>
#include <small/Pose3D.hh>
#include <proj_api.h>
#include <pthread.h>
#include <unistd.h>
#include <queue>
#include <zlib.h>
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/senlcm/evologics_usbl_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_usbl_angles_t.hpp"

#ifndef EVOLOGICS_H
#define EVOLOGICS_H

using namespace senlcm;
using namespace std;

#define DEBUG_PRINTF(x) printf x


#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI
#define MAX_BUF_LEN 1024
#define MAX_DATA_AGE 10e6

int chop_string(char *data, char **tokens);

enum {evo_command, evo_data, evo_im};

class Evo_Data_Out
{
    public:
        int64_t timestamp;
        unsigned char *data;
        int size;
        int target;
        int type;
};

class Evologics
{
    public:
//        Evologics(int _fd, char _term, lcm::LCM *lcm);
        Evologics(int _fd, char _term, lcm::LCM *lcm, queue<evologics_usbl_t *> *q);
        ~Evologics();
        int send_lcm_data(unsigned char *d, int size, int target, char *dest_channel);
        int send_command(const char *d);
        int send_ping(int target);
        int parse_lcm_data(unsigned char *d, int len);
        int parse_modem_data(char *d, int len, int64_t timestamp);
        int clear_modem();
        int clear_queues();
        
        // channel flags
        pthread_mutex_t flags_lock;
        bool sending_im;
        bool sending_data;
        bool sending_command;
        int current_target;
        
        int thread_exit;
        int fd;
        char term;
        
        // Out data FIFO
        queue<Evo_Data_Out *> command_queue;
        queue<Evo_Data_Out *> data_queue;
        queue<evologics_usbl_t *> *fixq;
        int send_data(unsigned char *d, int size, int type, int target);
                
    private:
        pthread_t read_thread_id;
        pthread_t write_thread_id;
        lcm::LCM *lcm;
        
        
        // message parsers
        int parse_usbllong(char *d, int64_t timestamp);
        int parse_usblangles(char *d, int64_t timestamp);
        int parse_im(char *d);
        
        // The queue that we use for fix messages being passed to the USBL code
        
};
        
#endif
