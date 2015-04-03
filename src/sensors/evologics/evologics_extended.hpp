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
#include "perls-common/timestamp.h"
#include "perls-common/serial.h"
#include "perls-lcmtypes++/senlcm/evologics_modem_t.hpp"

using namespace std;
using namespace senlcm;

#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI
#define MAX_BUF_LEN 1024
#define DEBUG_PORT "9201"

class Evologics_Extended
{
    public:
        Evologics_Extended();
        ~Evologics_Extended();
        int load_config(char *program_name);
        int init();
        int process();

        int reopen_port();
        int open_serial_port();
        int open_port(const char *ip, const char *port);

        int process_modem_data(char *d, int len, int64_t timestamp);

        // channel flags
        pthread_mutex_t flags_lock;

        int local_address;


        int fd;

        int thread_exit;

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

        lcm::LCM *lcm;

        // log all lines, or just SEND/RECV lines?
        char *vehicle_name;
        int complete;

};
