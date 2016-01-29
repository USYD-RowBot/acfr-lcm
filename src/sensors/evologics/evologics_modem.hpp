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
//#include "evologics.hpp"
#include "perls-common/timestamp.h"
#include "perls-common/serial.h"
//#include "perls-lcmtypes++/senlcm/novatel_t.hpp"
//#include "perls-lcmtypes++/senlcm/usbl_fix_t.hpp"
//#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_modem_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_usbl_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_usbl_angles_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_command_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_range_t.hpp"
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

class Evologics_Modem
{
    public:
        Evologics_Modem();
        ~Evologics_Modem();
        int load_config(char *program_name);
        int init();
        int process();
        //int calc_position(const evologics_usbl_t *evo);
        int ping_targets();
        int get_target_channel(const string target_name);
        int get_target_name(const int target_channel, string &target_name);
        int on_lcm_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel, bool use_pbm = false);
        
        bool send_fixes;
        bool sent_usbl_fix;
         
        int start_threads();
        int reopen_port();
        int open_serial_port();
        int open_port(const char *ip, const char *port);

        int send_lcm_data(unsigned char *d, int size, int target, const char *dest_channel, bool use_pbm = false);
        int send_command(const char *d);
        //int send_command_front(const char *d);
        int send_ping(int target);
        int process_lcm_data(unsigned char *d, int len);
        int process_modem_data(char *d, int len, int64_t timestamp);
        int clear_modem();
        int disconnect_modem();

        void publish_modem_response(int64_t timestamp, vector<unsigned char> buf);

        // LCM handlers
        int handle_heartbeat();
        
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
        
        int fd;
        
        int thread_exit;
        
        // Out data FIFO
        //vector<Evo_Data_Out *> command_queue;
        //vector<Evo_Data_Out *> data_queue;
        //queue<evologics_usbl_t *> *fixq;
        //int send_data(unsigned char *d, int size, int type, int target, int push_data_front);
        
        int wait_for_command_response();

        // counters
        int drop_counter;
        int drop_at_send;
        int im_sent;
        int last_im_sent;
        int im_counter;
        int  command_timeout_counter;

        
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

        // logging of received modem data
        int received_logging; // 0 == off, 1 == RECV*, 2 == all
        char *vehicle_name;

        int gain;
        int source_level;
        bool auto_gain;
        int sound_speed;
        
        //int open_port(const char *port);
        
        // targets
        int targets[MAX_TARGETS];
        char **target_names;
        int num_targets;
        int current_ping_target;
        char **lcm_channels;
        char **lcm_pbm_channels;
    
       
        int ping_period;
        int ping_counter; 
        int ping_time;
        int ping_timeout;
        
        int usbl_send_counter[MAX_TARGETS];    
        int usbl_send[MAX_TARGETS]; 
        
        pthread_t read_thread_id;
        lcm::LCM *lcm;
        
        
        // message processrs
        int process_usbllong(char *d, int64_t timestamp);
        int process_usblangles(char *d, int64_t timestamp);
        int process_im(char *d);
        int process_pbm(char *d);
        int process_burst_data(char *d);
       
        int chop_string(char *data, char **tokens, int nTokens);
        vector<string> chop_string(char *data, int nTokens);
        
};
         
