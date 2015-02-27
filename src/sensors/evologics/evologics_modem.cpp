#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>

#include "evologics_modem.hpp"


int loop_exit;


void on_heartbeat(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *hb, Evologics_Modem* ev) 
{
    ev->keep_alive(); 
}

void on_auv_status(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const auv_status_t *status, Evologics_Modem* ev) 
{
    cout << "Received lcm message on channel " << channel << ".  Sending auv_status to modem on address " << ev->get_usbl_address() << endl;
    ev->send_status(status);    
}

// a callback with no automatic decoding
void on_lcm(const lcm::ReceiveBuffer* rbuf, const std::string& channel, Evologics_Modem* ev) 
{
    cout << "Received lcm message on channel " << channel << ".  Sending to modem on address " << ev->get_usbl_address() << endl;
    char dest_channel[64];
    char buf[32];
    memset(dest_channel, 0, 64);
    strcpy(dest_channel, channel.c_str());
    sprintf(buf, ".%d", ev->get_usbl_address()); 
    strcat(dest_channel, buf);
    ev->evo->send_lcm_data((unsigned char *)rbuf->data, rbuf->data_size, ev->get_usbl_address(), dest_channel);
}

void on_evologics_command(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const evologics_command_t *ec, Evologics_Modem* ev) 
{
    if(ec->command == evologics_command_t::CLEAR)
        ev->evo->clear_modem();
}


Evologics_Modem::Evologics_Modem()
{
    lcm = new lcm::LCM();
    use_serial_comm = false;
    use_ip_comm = false;
}

int Evologics_Modem::keep_alive()
{
    if(keep_alive_count == 15)
    {
        keep_alive_count = 0;
        evo->send_command("+++AT?S");
    }
    else
        keep_alive_count++;

    
    if(ping_period > -1)
    {
        if(ping_counter == ping_period)
        {
            ping_counter = 0;
            for(int i=0; i<num_targets; i++)
                evo->send_ping(targets[i]);
        }
        else
            ping_counter++;
        
    }

    return 1;
}

int Evologics_Modem::send_status(const auv_status_t *status)
{
    // the message has been decoded so we will need to re encode it
    // we will also update the id field

    auv_status_t s;
    memcpy(&s, status, sizeof(auv_status_t));
    
    s.target_id = evo->local_address;
    
    char target_channel[16];
	memset(target_channel, 0, 16);
    sprintf(target_channel, "AUV_STATUS.%d", evo->local_address);
    int d_size = s.getEncodedSize();
    unsigned char *d = (unsigned char *)malloc(d_size);
    s.encode(d, 0, s.getEncodedSize());
    evo->send_lcm_data(d, d_size, usbl_address, target_channel);
    free(d);
    
    return 1;
}
    

int Evologics_Modem::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm->getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
        
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "sensors.%s", program_name);


    // read the initial config
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
    
	sprintf(key, "%s.usbl_address", rootkey);
    usbl_address = bot_param_get_int_or_fail(param, key);

    // Ping information
    sprintf(key, "%s.targets", rootkey);
    if(bot_param_has_key(param, key))
        num_targets = bot_param_get_int_array(param, key, targets, 8);
    else
        num_targets = 0;

    sprintf(key, "%s.ping_period", rootkey);
    if(bot_param_has_key(param, key))
        ping_period = bot_param_get_int_or_fail(param, key);
    else
        ping_period = -1;

    sprintf(key, "%s.ping_timeout", rootkey);
    if(bot_param_has_key(param, key))
        ping_timeout = bot_param_get_int_or_fail(param, key);
    else
        ping_timeout = 15;

    
    // LCM messages to push through the tunnel
    sprintf(key, "%s.lcm", rootkey);
    lcm_channels = NULL;
    lcm_channels = bot_param_get_str_array_alloc(param, key);
    
    sprintf(key, "%s.gain", rootkey);
    gain = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.source_level", rootkey);
    source_level = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.auto_gain", rootkey);
    auto_gain = bot_param_get_boolean_or_fail(param, key);
    
    return 1;
}        

int Evologics_Modem::open_port()
{
    // In a seperate function so we can reconnect if the pipe breaks

    int fd;
   
    struct addrinfo hints, *evo_addr, *result;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = 0;
    //int s = getaddrinfo(ip, port, &hints, &evo_addr);
    int s = getaddrinfo(ip, port, &hints, &result);
    if (s != 0) {
       fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
               exit(EXIT_FAILURE);
    }
    for (evo_addr = result; evo_addr != NULL; evo_addr = evo_addr->ai_next)
    {
       fd = socket(evo_addr->ai_family, evo_addr->ai_socktype, evo_addr->ai_protocol);
       if (fd == -1)
       {
           perror("Could not create socket\n");
           continue;
       }

       if(connect(fd, evo_addr->ai_addr, evo_addr->ai_addrlen) != -1)
       {
           break; // Success
       }
       perror("Failed to connect.  Trying next address");
       close(fd);
    }

    if (evo_addr == NULL)
    {
       printf("Could not connect to %s on port %s\n", ip, port);
       return -1;
    }

    freeaddrinfo(result);

    struct timeval tv;
    tv.tv_sec = 1;  // 1 Secs Timeout
    tv.tv_usec = 000;  // Not init'ing this can cause strange errors
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    // flush the port
    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
    flag = 0;
    write(fd, &flag, 1);
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));


    return fd;
}


int Evologics_Modem::init()
{
    // Open the comm port
    char term;
    char msg[32];
    if (use_serial_comm)
    {
       evo_fd = serial_open(device, serial_translate_speed(baud), serial_translate_parity(parity), 1);    
       //serial_set_canonical(state.fd, '\r', '\n');
       serial_set_noncanonical(evo_fd, 1, 0);
   
       tcflush(evo_fd,TCIOFLUSH);
       term = '\r';
    } else if (use_ip_comm) {
       evo_fd = open_port(); 
       term = '\n';
    }
    
    // Create the Evologics object
    evo = new Evologics(evo_fd, term, lcm, NULL, ping_timeout);
    
    // subscribe to the relevant streams
    
    // put the modem in a known state
    evo->send_command("+++ATZ4");
    evo->wait_for_commands();
    evo->send_command("+++ATZ1");
    evo->wait_for_commands();
    
    char cmd[64];
    memset(cmd, 0, 64);
    sprintf(cmd, "+++AT!L%d", source_level);
    evo->send_command(cmd);
    sprintf(cmd, "+++AT!G%d", gain);
    evo->send_command(cmd);

    // Get the local address
    evo->send_command("+++AT?AL");

    if(auto_gain)
        evo->send_command("+++AT!LC1");
    
    evo->send_command("+++ATZ1");
    
    // now to force the settings that require a listen mode
    evo->wait_for_commands();
    evo->send_command("+++ATN");      // noise mode
    evo->wait_for_commands();
    usleep(3e6);
    evo->send_command("+++ATA");      // listen state
    evo->wait_for_commands();
    
    keep_alive_count = 0;
    ping_counter = 0;
    
    // Subscribe to LCM messages
    lcm->subscribeFunction("HEARTBEAT_1HZ", on_heartbeat, this);
    lcm->subscribeFunction("AUV_STATUS", on_auv_status, this);
    lcm->subscribeFunction("EVOLOGICS_CONTROL", on_evologics_command, this);
    
    int i = 0;
    while(lcm_channels[i] != NULL)
    {
        lcm->subscribeFunction(lcm_channels[i], on_lcm, this);
        i++;
      
    }
    
    evo->start_handlers();
    
    return 1;
}

int Evologics_Modem::get_usbl_address()
{
    return usbl_address;
}

int Evologics_Modem::process()
{
    int lcm_fd = lcm->getFileno();
    fd_set rfds;
    char buf[MAX_BUF_LEN];
    
    while(!loop_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (lcm_fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        memset(buf, 0, MAX_BUF_LEN);
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm->handle();
        }
        
    }
    delete evo;
    
    return 1;
}


void signal_handler(int sig)
{
    loop_exit = 1;
}

// Main entry point
int main(int argc, char **argv)
{
    // install the exit handler
    loop_exit = 0;
    signal(SIGINT, signal_handler);
    Evologics_Modem *modem = new Evologics_Modem;
    modem->load_config(basename((argv[0])));
    modem->init();
    
    modem->process();
    
    delete modem;
    return 1;
}
