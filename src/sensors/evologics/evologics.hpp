#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <pthread.h>
#include <unistd.h>
#include <deque>
#include <queue>
#include <vector>
#include <zlib.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include "perls-common/serial.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/senlcm/evologics_usbl_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_usbl_angles_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_range_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"

#ifndef EVOLOGICS_H
#define EVOLOGICS_H

using namespace senlcm;
using namespace perllcm;
using namespace std;

#define DEBUG_PRINTF(x) printf x


#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI
#define MAX_BUF_LEN 1024
#define MAX_DATA_AGE 5e6

int chop_string(char *data, char **tokens, int nTokens);

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
        Evologics(char *device, int baud, char *parity, lcm::LCM *lcm, queue<evologics_usbl_t *> *q, int _ping_timeout);
        Evologics(char *ip, char *port, lcm::LCM *lcm, queue<evologics_usbl_t *> *q, int _ping_timeout);
        ~Evologics();
        int init(lcm::LCM *lcm, queue<evologics_usbl_t *> *q, int _ping_timeout);
        int start_threads();
        int reopen_port();
        int open_serial_port();
        int open_port(const char *ip, const char *port);

        int send_lcm_data(unsigned char *d, int size, int target, const char *dest_channel, bool use_pbm = false);
        int send_command(const char *d);
        //int send_command_front(const char *d);
        int send_ping(int target);
        int parse_lcm_data(unsigned char *d, int len);
        int parse_modem_data(char *d, int len, int64_t timestamp);
        int clear_modem();
        int disconnect_modem();

        // LCM handlers
        int handle_heartbeat();
        int start_handlers();
        
        // channel flags
        pthread_mutex_t flags_lock;
        //pthread_mutex_t command_queue_lock;
        //pthread_mutex_t data_queue_lock;
        pthread_mutex_t write_lock;

        bool sending_im;
        bool sending_data;
        bool sending_command;
	string command_sent;
        int current_target;
        
        // Variables for ranging
        int last_im_target;     // Used for ranging
        int64_t last_im_timestamp;
        int local_address;
        
        
        int thread_exit;
        int fd;
        char term;
        
        // Out data FIFO
        //vector<Evo_Data_Out *> command_queue;
        //vector<Evo_Data_Out *> data_queue;
        queue<evologics_usbl_t *> *fixq;
        //int send_data(unsigned char *d, int size, int type, int target, int push_data_front);
        
        // Queue management commands
        int clear_queues();
        int wait_for_command_response();

        // counters
        int drop_counter;
        int drop_at_send;
        int im_sent;
        int last_im_sent;
        int im_counter;
        int ping_timeout;
        int  command_timeout_counter;
        
                
    private:
        string device;
        int baud;
        string parity;
        bool use_serial_port;

        string ip;
        string port;
        bool use_ip_port;

        pthread_t read_thread_id;
        pthread_t command_thread_id;
        pthread_t data_thread_id;
        lcm::LCM *lcm;
        
        
        // message parsers
        int parse_usbllong(char *d, int64_t timestamp);
        int parse_usblangles(char *d, int64_t timestamp);
        int parse_im(char *d);
        int parse_pbm(char *d);
        int parse_burst_data(char *d);
        
        
        // The queue that we use for fix messages being passed to the USBL code
        
};
        
#endif
