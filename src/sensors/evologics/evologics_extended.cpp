#include "evologics_extended.hpp"

int loop_exit;
int pipe_broken;

Evologics_Extended::Evologics_Extended()
{
    lcm = new lcm::LCM();
}

Evologics_Extended::~Evologics_Extended()
{
}

int Evologics_Extended::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm->getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;

    char rootkey[64];
    char key[128];
    sprintf (rootkey, "sensors.%s", program_name);

    sprintf(key, "%s.complete", rootkey);
    complete = bot_param_get_boolean_or_fail(param, key);
    cout << "Logging complete output." << endl;

    // check if we are using an serial connection
    sprintf(key, "%s.device", rootkey);
    if (bot_param_has_key(param, key))
    {
        device = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.baud", rootkey);
        baud = bot_param_get_int_or_fail(param, key);

        sprintf(key, "%s.parity", rootkey);
        parity = bot_param_get_str_or_fail(param, key);
        use_serial_comm = true;
    }

    // check if we are using an IP connection
    sprintf(key, "%s.ip", rootkey);
    if (bot_param_has_key(param, key))
    {
        ip = bot_param_get_str_or_fail(param, key);
        sprintf(key, "%s.port", rootkey);
        port = bot_param_get_str_or_fail(param, key);
        use_ip_comm = true;
    }
    if (use_serial_comm == false && use_ip_comm == false) {
        cout << "Missing config setting for ip or serial comms" << endl;
        exit(1);
    }

    return 1;

}

int Evologics_Extended::init()
{
    // Open the comm ports
    if (use_serial_comm)
    {
        // really can't be used without displacing the actual modem process
       fd = open_serial_port();
       term = '\r';
    } else if (use_ip_comm) {
       fd = open_port(ip, port);
       term = '\n';
    } else {
       printf("Unknown communications protocol.  Can't initialise Evologics object.\n");
       exit(1);
    }

    return 1;
}

int Evologics_Extended::reopen_port()
{
    close(fd);

    if (use_serial_comm)
    {
       fd = open_serial_port();
    } else if (use_ip_comm) {
       fd = open_port(ip, port);
    }
    // if the reconnection fails, wait for a while before reattempting
    if (fd == -1)
       sleep(1);
    return fd;
}

int Evologics_Extended::open_serial_port()
{
   int evo_fd = serial_open(device, serial_translate_speed(baud), serial_translate_parity(parity), 1);
   serial_set_noncanonical(evo_fd, 1, 0);

   tcflush(evo_fd,TCIOFLUSH);

   return evo_fd;
}


int Evologics_Extended::open_port(const char *ip, const char *port)
{
    // In a seperate function so we can reconnect if the pipe breaks
    printf("Attempting to connect to %s on port %s\n", ip, port);

    int evo_fd;

    struct addrinfo hints, *evo_addr, *result;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_TCP;
    //int s = getaddrinfo(ip, port, &hints, &evo_addr);
    int s = getaddrinfo(ip, port, &hints, &result);
    if (s != 0) {
       fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
               exit(EXIT_FAILURE);
    }
    for (evo_addr = result; evo_addr != NULL; evo_addr = evo_addr->ai_next)
    {
       evo_fd = socket(evo_addr->ai_family, evo_addr->ai_socktype, evo_addr->ai_protocol);
       if (evo_fd == -1)
       {
           perror("Could not create socket\n");
           continue;
       }

       if(connect(evo_fd, evo_addr->ai_addr, evo_addr->ai_addrlen) != -1)
       {
           break; // Success
       }
       perror("Failed to connect.  Trying next address");
       close(evo_fd);
       evo_fd = -1;
    }

    freeaddrinfo(result);

    if (evo_addr == NULL)
    {
       printf("Could not connect to %s on port %s\n", ip, port);
    } else {
       printf("Successfully connected to %s on port %s\n", ip, port);
    }
    return evo_fd;
}


int Evologics_Extended::process()
{
    // reading the port
    fd_set rfds;
    int64_t timestamp;
    vector<unsigned char> buf;
    buf.reserve(1024);
    size_t bytes;

    while(!loop_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (this->fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        timestamp = timestamp_now();
        if(ret > 0)
        {
            buf.clear();
            bytes = 0;
            while (true) {
                // for now - just grab the whole packet, it can be parsed later
                char byte;
                bytes += read(this->fd, &byte, 1);
                buf.push_back(byte);
                // presumably hit the end of the message - this may not be the case
                // if the data is binary, or the content contains a new line pairing...
                if((buf[bytes-2] == 0x0D) || (buf[bytes-1] == 0x0A))
                {
                    // if we are taking complete logs, then publish it, else only do so for SEND and RECV
                    if (complete || strncmp((char *)buf.data(), "SEND", 4) == 0 || strncmp((char *)buf.data(), "RECV", 4) == 0) {
                        cout << "Logging message: " << string((char *)buf.data(), 4) << endl;
                        evologics_modem_t msg;
                        msg.utime = timestamp;
                        msg.size = bytes;
                        msg.data = buf;
                        lcm->publish("EVOLOGICS_LOG", &msg);
                    }
                    break;
                }
            }
        } else if (ret == -1) {
            char buf[64];
            sprintf(buf, "Select failed on read thread %d", this->fd);
            perror(buf);
            this->reopen_port();
        }
    }
    return 1;
}

void signal_handler(int sig)
{
    cout << "WARNING: Caught signal: " << sig << endl;
    if(sig == SIGPIPE)
        pipe_broken = 1;
    else
        loop_exit = 1;
}

// Main entry point
int main(int argc, char **argv)
{
    // install the exit handler
    loop_exit = 0;
    if (signal(SIGINT, signal_handler) == SIG_ERR)
         perror("ERROR: Can't register signal_handler for SIGINT");

    if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
         perror("ERROR: Can't register signal_handler for SIGPIPE");

    Evologics_Extended *modem = new Evologics_Extended;

    modem->load_config(basename((argv[0])));
    modem->init();
    modem->process();

    delete modem;

    return 1;
}
