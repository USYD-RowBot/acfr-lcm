#include "evologics_ahrs.hpp"

int loop_exit;
int pipe_broken;

// LCM handlers
void on_heartbeat(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *hb, Evologics_AHRS* ev) 
{
    ev->handle_heartbeat(); 
}

// a callback with no automatic decoding
void on_lcm(const lcm::ReceiveBuffer* rbuf, const std::string& channel, Evologics_AHRS* ev) 
{
    //ev->on_lcm_data(rbuf, channel, false);
}
    
// Used by the AHRS routine
int readline(int fd, char *buf, int max_len)
{
    int i=0;
    do
    {
        if(recv(fd, &buf[i++], 1, 0) == -1)
            break;
    } while((buf[i-2] != 0x0D) && (buf[i-1] != 0x0A));
    return i;
}

Evologics_AHRS::Evologics_AHRS()
{
    lcm = new lcm::LCM();
}

Evologics_AHRS::~Evologics_AHRS()
{
    thread_exit = 1;
}


int Evologics_AHRS::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm->getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
        
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "sensors.%s", program_name);

    // check if we are using an IP connection
    sprintf(key, "%s.ip", rootkey);
    if (bot_param_has_key(param, key))
    {
        ip = bot_param_get_str_or_fail(param, key);
        sprintf(key, "%s.port", rootkey);
        port = bot_param_get_str_or_fail(param, key);
    }

    return 1;    

}

// Cut a string up
int Evologics_AHRS::chop_string(char *data, char **tokens, int nTokens)
{
    char *token;
    int i = 0;

    token = strtok(data, " ,:*?!");
    while(token != NULL && i < nTokens)
    {
        tokens[i++] = token;
        token = strtok(NULL, " ,:*?!");
    }
    return i;
}


int Evologics_AHRS::process_ahrs_message(char *buf)
{
    // decode the AHRS message
    if(strstr(buf, "AHRS") != NULL)
    {
        char *tokens[6];
        if(chop_string(buf, tokens, 6) != 6)
            return 0;

        ahrs.mtime = (int64_t)(atof(tokens[1]) * 1e6);
        ahrs.utime = timestamp_now();
        ahrs.roll = atof(tokens[4]) * DTOR;
        ahrs.pitch = atof(tokens[3]) * DTOR;
        ahrs.heading = atof(tokens[5]) * DTOR;
        return 1;
    }
    else
        return 0;
}
        
int Evologics_AHRS::init()
{
    cout << "Preparing to open ahrs connection on ip address " << ip << " port " << port << endl;
    if((ahrs_fd = open_port(ip, port)) == -1)
    {
        cout << "Failed to open port on ip address " << ip << " on port " << port << endl;
        return 0;
    }
    
    thread_exit = 0;
   
    start_handlers();
 
    return 1;       
}


int Evologics_AHRS::reopen_port()
{
    close(ahrs_fd);

    ahrs_fd = open_port(ip, port);

    // if the reconnection fails, wait for a while before reattempting
    if (ahrs_fd == -1)
       sleep(1);
    return ahrs_fd;
}

int Evologics_AHRS::open_port(const char *ip, const char *port)
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

int Evologics_AHRS::start_handlers()
{
    // LCM subscriptions
    lcm->subscribeFunction("HEARTBEAT_5HZ", on_heartbeat, this);
    
    return 1;
}

int Evologics_AHRS::handle_heartbeat()
{
    lcm->publish("AHRS", &ahrs);
    return 1;
}

int Evologics_AHRS::process()
{
    int lcm_fd = lcm->getFileno();
    fd_set rfds;
    char buf[MAX_BUF_LEN];
    
    while(!loop_exit)
    {
        // check the port status, broekn pipes
        /*
        if(pipe_broken)
        {
            cout << "Evologics_AHRS: process found pipe_broken.  Reconnecting." << endl;
            if (open_port(ip, port) > 0)
	            pipe_broken = false;
        }*/
        
        FD_ZERO (&rfds);
        FD_SET (lcm_fd, &rfds);
        FD_SET (ahrs_fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        memset(buf, 0, MAX_BUF_LEN);
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm->handle();
                        
            if(FD_ISSET(ahrs_fd, &rfds))
            {
                // data to be read       
                memset(buf, 0, MAX_BUF_LEN);
                readline(ahrs_fd, buf, MAX_BUF_LEN);
                process_ahrs_message(buf);
            }
        } else if (ret == -1) {
            cout << "select timeout with return: " << ret << endl;
            if (reopen_port() > 0)
                pipe_broken = false;
            sleep(1);
        } else {
        }
    }
    //delete evo;
    
    return 1;
}

void signal_handler(int sig)
{
    cout << "WARNING: Caught signal: " << sig << endl;
    if(sig == SIGPIPE)
        // the connection to the Evologics has failed for some reason,
        // we will just reconnect it
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
    //if (signal(SIGPIPE, signal_handler) == SIG_ERR)
    if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
         perror("ERROR: Can't register signal_handler for SIGPIPE");
    Evologics_AHRS *ahrs = new Evologics_AHRS;
    ahrs->load_config(basename((argv[0])));
    ahrs->init();
    
    ahrs->process();
    
    delete ahrs;
    return 1;
}
