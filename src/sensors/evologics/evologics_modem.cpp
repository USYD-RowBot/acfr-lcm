#include "evologics_modem.hpp"

int loop_exit;
int pipe_broken;

// LCM handlers
void on_heartbeat(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *hb, Evologics_Modem* ev) 
{
    ev->handle_heartbeat(); 
}

void on_evo_control(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const evologics_command_t *ec, Evologics_Modem* ev) 
{
    if(ec->command == evologics_command_t::SEND_FIXES) 
        ev->send_fixes = ec->d;
    cout << "SF:" << ev->send_fixes << endl;
    
    if(ec->command == evologics_command_t::CLEAR)
        ev->clear_modem();
}


// a callback with no automatic decoding
void on_lcm(const lcm::ReceiveBuffer* rbuf, const std::string& channel, Evologics_Modem* ev) 
{
    ev->on_lcm_data(rbuf, channel, false);
}
    
// a callback with no automatic decoding for messages that should be registered as PiggyBack messages
void on_lcm_pbm(const lcm::ReceiveBuffer* rbuf, const std::string& channel, Evologics_Modem* ev) 
{
    ev->on_lcm_data(rbuf, channel, true);
}
    

Evologics_Modem::Evologics_Modem()
{
    lcm = new lcm::LCM();
    current_ping_target = 0;
//    pthread_create(&fix_thread_id, NULL, fix_thread, this);
//    pthread_detach(fix_thread_id);
}

Evologics_Modem::~Evologics_Modem()
{
    // free up the resources of the string arrays
    if (lcm_channels != NULL)
       bot_param_str_array_free(lcm_channels);
    if (lcm_pbm_channels != NULL)
       bot_param_str_array_free(lcm_pbm_channels);
    bot_param_str_array_free(target_names);

    disconnect_modem();
    thread_exit = 1;
    pthread_join(read_thread_id, NULL);
}


int Evologics_Modem::ping_targets()
{
    // check the state of the link if we are sending data
    //cout << "sending_data=" << state.sending_data << endl;
//    if(sending_data)
//        send_command("AT?S");
    
    if (ping_period > -1)
    {
        if(ping_counter == ping_period)
        {
            string targetName;
            get_target_name(targets[current_ping_target], targetName);
            cout << "Preparing to ping target " << targetName << " at address " << targets[current_ping_target] << endl;
            ping_counter = 0;
            // try to ping the current target.  If we are already waiting for
            // a ping reply don't change target.
            if (send_ping(targets[current_ping_target]) == 1)
            {
                if (++current_ping_target >= num_targets)
                    current_ping_target = 0;
            }
        }
        else
            ping_counter++;
    }
    
    for(int i=0; i<num_targets; i++)    
    {
        // limit the usbl transmit rate to the ping period
        if(!usbl_send[i] && usbl_send_counter[i] < ping_period)
            usbl_send_counter[i]++;
        else
        {
            usbl_send[i] = 1;
            usbl_send_counter[i] = 0;
        }
    }
            
    return 1;
}

int Evologics_Modem::get_target_channel(const string target_name)
{
    int target_index = 0;
    while (target_names[target_index] != NULL)
    {
       //if (!strcmp(target_names[target_index], target_name.c_str()))
       if (target_name == target_names[target_index])
       {
          break;
       }
       target_index++;
    }
    if (target_names[target_index] == NULL)
    {
       cout << "Target " << target_name << " not found" << endl;
       return -1;
    } else {
       return targets[target_index];
    }
}

int Evologics_Modem::get_target_name(const int target_channel, string &target_name)
{
    for (int target_index = 0; target_index < num_targets; target_index++)
    {
       if (targets[target_index] == target_channel)
       {
         target_name = target_names[target_index];
         return 1;
       }
    } 
    return 0;
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

    use_serial_comm = false;
    use_ip_comm = false;

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

    sprintf(key, "%s.vehicle_name", rootkey);
    vehicle_name = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.term", rootkey);
    if (bot_param_has_key(param, key))
    {
	int terms[4];
	int num_term = bot_param_get_int_array(param, key, terms, 4);
	term = (char *)malloc(num_term);
	for(int i=0; i<num_term; i++)
	    term[i] = terms[i] & 0xFF;
	term_len = num_term;

//	bot_param_get_str(param, key, &term);
//	term_len = strlen(term);
    }
    else
    {
	term = (char *)malloc(1);
        if (use_serial_comm)
        {
	    *term = '\r';
            term_len = 1;
        }
            
        if (use_ip_comm)
        {
	    *term = '\n';
            term_len = 1;
        }
    }

    // check if we are using an IP connection
    sprintf(key, "%s.logging_level", rootkey);
    received_logging = 0; // set default to off
    if (bot_param_has_key(param, key))
    {
       received_logging =  bot_param_get_int_or_fail(param, key);
        if (received_logging > 0) {
            sprintf(key, "%s.vehicle_name", rootkey);
            vehicle_name = bot_param_get_str_or_fail(param, key);
        }
    }
    
    // Ping information
    sprintf(key, "%s.targets", rootkey);
    num_targets = bot_param_get_int_array(param, key, targets, 8);

    sprintf(key, "%s.target_names", rootkey);
    target_names = NULL;
    target_names = bot_param_get_str_array_alloc(param, key);
    int i = 0;
    cout << "Read in target names: ";
    while (target_names[i] != NULL)
    {
        cout << targets[i] << ":" << target_names[i] << "; ";
        i++;
    }
    cout << endl;
    if (i != num_targets)
    {
        cout << "Target IDs and name arrays do not match" << endl;
        exit(1);
    }

    sprintf(key, "%s.ping_period", rootkey);
    ping_period = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.ping_timeout", rootkey);
    ping_timeout = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.gain", rootkey);
    gain = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.source_level", rootkey);
    source_level = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.auto_gain", rootkey);
    auto_gain = bot_param_get_boolean_or_fail(param, key);

    sprintf(key, "%s.lcm", rootkey);
    lcm_channels = NULL;
    lcm_channels = bot_param_get_str_array_alloc(param, key);

    sprintf(key, "%s.lcm_pbm", rootkey);
    lcm_pbm_channels = NULL;
    lcm_pbm_channels = bot_param_get_str_array_alloc(param, key);

    sprintf(key, "%s.sound_speed", rootkey);
    sound_speed = bot_param_get_int_or_fail(param, key);

    return 1;    

}

int Evologics_Modem::on_lcm_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel, bool use_pbm) 
{
    // extract the platform id
    int channel_pos = channel.find_last_of('.');
    std::string target_name = channel.substr(channel_pos + 1);
    // figure out which channel to send the data on
    int target_channel = get_target_channel(target_name.c_str());
    cout << "Got LCM message on channel " << channel;
    if (target_channel != -1) 
    {
       cout << ".  Sending to target name: " << target_name << " on channel " << target_channel << endl;
       send_lcm_data((unsigned char *)rbuf->data, rbuf->data_size, target_channel, channel.c_str(), use_pbm);
   } else {
       cout << ".  Target " << target_name << " not found in channel list.  Dropping lcm message." << endl;
    }

    return 1;
}

int Evologics_Modem::init()
{
    // set flags
    sending_im = false;
    sending_data = false;
    sending_command = false;
    command_sent = "";
    current_target = 0;
    drop_counter = 0;
    im_sent = 0;
    last_im_sent = 0;
    im_counter = 0;

    // open the ports

    // Open the comm ports
    if (use_serial_comm)
    {
       //evo = new Evologics(device, baud, parity, lcm, ping_timeout);
       fd = open_serial_port();
//       if(term == NULL)
//       {
//           term_char = '\r';
//           term = &term_char;
//           term_len = 1;
//       }
//       term = '\r';
    } else if (use_ip_comm) {
       //evo = new Evologics(ip, inet_port, lcm, ping_timeout);
       fd = open_port(ip, port);
//       term = '\n';
//       if(term == NULL)
//       {
//           term_char = '\n';
//           term = &term_char;
//           term_len = 1;
//      }
    } else {
       printf("Unknown communications protocol.  Can't initialise Evologics object.\n");
       exit(1);
    } 
printf("Term char = 0x%02X\n", (unsigned int)*term);
    thread_exit = 0;
    pthread_mutex_init(&flags_lock, NULL);
    pthread_mutex_init(&write_lock, NULL);

    start_threads();

    // put the USBL in a known state
    //send_command("ATZ4");
    //send_command("ATZ1");

    char cmd[64];
    memset(cmd, 0, 64);

	// As of firmware version 1.8 you need to enable global settings control before
	// being able to change things like the gain
    sprintf(cmd, "AT@CTRL");
    send_command(cmd);
	
    sprintf(cmd, "AT!L%d", source_level);
    send_command(cmd);
    sprintf(cmd, "AT!G%d", gain);
    send_command(cmd);

	
    sprintf(cmd, "AT!CA%d", sound_speed);
    send_command(cmd);

    if(auto_gain)
    {
        send_command("AT!LC1");
    }

    // Get the local address
    //send_command("AT?AL");
    send_command("AT?S");

    /*//set up to the first remote address in the list for data communications.  This will be switched when messages arrive as necessary.
    if (num_targets > 0)
    {
        memset(cmd, 0, 64);
        sprintf(cmd, "AT!AR%d", targets[0]); 
        send_command(cmd);
    }
    // now to force the settings that require a listen mode
    // we need to wait for the modem to catch up before the next two commands
    send_command("AT?AR");
    */ 
    send_command("AT@ZU1");      // request USBL positioning data
    send_command("AT?ZU");      // request USBL positioning data
    
    //send_command("ATN");      // noise mode
    
    //send_command("ATA");      // listen state
    //send_command("ATD");      // establish an acoustic connection

    send_command("AT!RT1500");     // set the retry count on burst data
    send_command("AT!RC1");     // set the retry timeout on burst data


    usleep(1e6);
    
    lcm->subscribeFunction("HEARTBEAT_1HZ", on_heartbeat, this);
    lcm->subscribeFunction("EVOLOGICS_CONTROL", on_evo_control, this);
    
    int lcm_channel_ndx = 0;
    while(lcm_channels != NULL && lcm_channels[lcm_channel_ndx] != NULL)
    {
        cout << "Subscribing to LCM channel: " << lcm_channels[lcm_channel_ndx] << endl;
        lcm->subscribeFunction(lcm_channels[lcm_channel_ndx], on_lcm, this);
        lcm_channel_ndx++;
    }

    lcm_channel_ndx = 0;
    while(lcm_pbm_channels != NULL && lcm_pbm_channels[lcm_channel_ndx] != NULL)
    {
        cout << "Subscribing to LCM PBM channel: " << lcm_pbm_channels[lcm_channel_ndx] << endl;
        lcm->subscribeFunction(lcm_pbm_channels[lcm_channel_ndx], on_lcm_pbm, this);
        lcm_channel_ndx++;
    }
    
    ping_counter = 0;
    send_fixes = true;
    sent_usbl_fix = false;
    
    return 1;       
}

void Evologics_Modem::publish_modem_response(int64_t timestamp, vector<unsigned char> buf)
{
    switch (received_logging) {
        case 0:
            return;
        case 1:
            if (strncmp((const char *)buf.data(), "RECV", 4) != 0) return;
    }

    cout << "Logging message:\n>" << string((char *)buf.data(), buf.size() - 3) << endl;
    cout << "Message complete." << endl;
    evologics_modem_t msg;
    msg.utime = timestamp;
    msg.size = buf.size() - 1;
    msg.data = buf;

    char channel_name[128];
    snprintf(channel_name, 128, "%s.EVOLOGICS_LOG", vehicle_name);
    this->lcm->publish(channel_name, &msg);
}

// read thread
static void *read_thread(void *u)
{
    Evologics_Modem *evo = (Evologics_Modem *)u;
 
    cout << "Read thread started\n";
    
    fd_set rfds;
    vector<unsigned char> buf;
    //buf.reserve(1024);
    int bytes;
    int64_t timestamp;
    
    while(!evo->thread_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (evo->fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        timestamp = timestamp_now();
        if(ret > 0)
        {
            // we need to do single character reads as the termination character can change
            bytes = 0;
            buf.clear();
            while(true)
            {
               char byte;
               bytes += read(evo->fd, &byte, 1);
               buf.push_back(byte);
               if(bytes > 1 && ((buf[bytes-2] == 0x0D) && (buf[bytes-1] == 0x0A)))
               {
                  // a potential modem message
                  char *ptr = reinterpret_cast<char *>(buf.data());
                  cout << "Received potential modem data: " << bytes << " bytes:\n>" << string(ptr, bytes-2) << endl;
                  cout << "End of message." << endl;

                  buf.push_back(0); // add null termination
                  // check if we've received an LCM message in the string
                  if(strstr(ptr, "RECV") != NULL &&
                     strstr(ptr, "LCM") != NULL)
                  {
                     // check for the 'LE' characters at the end of the string
                     if (buf[bytes-4] == 0x4C && buf[bytes-3] == 0x45)
                     {
                        evo->publish_modem_response(timestamp, buf);
                        if (evo->process_modem_data(ptr, bytes, timestamp))
                           break;
                     } else {
                        cout << "Failed to find LE at end of potential LCM message.  Continuing to look for end of line character." << endl;
                     }
                  } else {
                     // otherwise let's try to process the message
                     evo->publish_modem_response(timestamp, buf);
                     if (evo->process_modem_data(ptr, bytes, timestamp))
                        break;
                  }
                  // wasn't the end of a packet... drop the null char
                  buf.pop_back();
               }
            }
            
        } else if (ret == -1) {
            char tmpbuf[64];
            sprintf(tmpbuf, "Select failed on read thread %d", evo->fd);
            perror(tmpbuf);
            evo->reopen_port();
        } else {
            printf("Select timeout\n");
        }
    }
    cout << "Read thread exit\n";
    return NULL;
}

               
// Helper functions

vector<string> Evologics_Modem::chop_string(char *data, int nTokens)
{
   std::string s(data);
   std::string delimiter = " ,:*?!";
   vector<string> tokens;

   std::string token;
   size_t last = 0;
   size_t next = 0;
   int i = 0;
   while (((next = s.find_first_of(delimiter, last)) != std::string::npos) && (i < nTokens-1)) {
       token = s.substr(last, next-last);
       tokens.push_back(token);
       last = next + 1;
       i++;
   }
   tokens.push_back(s.substr(last));
   return tokens;
}

// Cut a string up
int Evologics_Modem::chop_string(char *data, char **tokens, int nTokens)
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

/*
Evologics_Modem::Evologics(char *_device, int _baud, char *_parity, lcm::LCM *_lcm, int _ping_timeout)
{
    device = _device;
    baud = _baud;
    parity = _parity;
    use_serial_port = true;
    use_ip_port = false;

    init(_lcm, _ping_timeout);
    fd = open_serial_port();
    term = NULL;
    start_threads();
}

Evologics_Modem::Evologics(char *_ip, char *_port, lcm::LCM *_lcm, int _ping_timeout)
{
    ip = _ip;
    port = _port;
    use_serial_port = false;
    use_ip_port = true;

    init(_lcm, _ping_timeout);
    fd = open_port(ip.c_str(), port.c_str());
    term = NULL;
    start_threads();
}

Evologics_Modem::~Evologics()
{
    disconnect_modem();
    thread_exit = 1;
    pthread_join(read_thread_id, NULL);
}
int Evologics_Modem::init(lcm::LCM *_lcm, int _ping_timeout)
{
    lcm = _lcm;
    //fixq = q;
    ping_timeout = _ping_timeout;
    
    sending_im = false;
    sending_data = false;
    sending_command = false;
    command_sent = "";
    current_target = 0;
    drop_counter = 0;
    im_sent = 0;
    last_im_sent = 0;
    im_counter = 0;
    
    
    thread_exit = 0;
    pthread_mutex_init(&flags_lock, NULL);
    pthread_mutex_init(&write_lock, NULL);
            
    return 1; 
}
*/

int Evologics_Modem::start_threads()
{
    pthread_create(&read_thread_id, NULL, read_thread, this);
    pthread_detach(read_thread_id);
    return 1;
}

int Evologics_Modem::reopen_port()
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

int Evologics_Modem::open_serial_port()
{
   pthread_mutex_lock(&write_lock);
   int evo_fd = serial_open(device, serial_translate_speed(baud), serial_translate_parity(parity), 1);
   serial_set_noncanonical(evo_fd, 1, 0);

   tcflush(evo_fd,TCIOFLUSH);
   pthread_mutex_unlock(&write_lock);

   return evo_fd;
}


int Evologics_Modem::open_port(const char *ip, const char *port)
{
    // In a seperate function so we can reconnect if the pipe breaks
    printf("Attempting to connect to %s on port %s\n", ip, port);
    pthread_mutex_lock(&write_lock);

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


       //struct timeval tv;
       //tv.tv_sec = 1;  // 1 Secs Timeout
       //tv.tv_usec = 0;  // Not init'ing this can cause strange errors
       //setsockopt(evo_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

       // flush the port
       //int flag = 1;
       //setsockopt(evo_fd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
       //flag = 0;
       //write(evo_fd, &flag, 1);
       //setsockopt(evo_fd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
    }
    pthread_mutex_unlock(&write_lock);
    return evo_fd;
}


int Evologics_Modem::handle_heartbeat()
{
    // If we are sending data we need to check to make sure it has gone through
    // we will send an AT?S one a second until it has completed.
    //if(sending_data)
    //{
    //    send_command("AT?S");
    //}
    

    // if we have gotten stuck sending an IM we need to clear the buffer
    if(sending_im == true && im_sent == last_im_sent)
        im_counter++;
    else
    {
        im_counter = 0;
        last_im_sent = im_sent;
    }
    if(im_counter > ping_timeout)
    {
        printf("Instant message reply timeout\n");
        //send_command_front("ATZ3");
        //send_command("ATZ3");
        
        im_counter = 0;
        
        // request the size of the current IM queue
        //send_command_front("AT?DI");
        //send_command("AT?DI");
        pthread_mutex_lock(&flags_lock);
        sending_im = false;
        pthread_mutex_unlock(&flags_lock);
    }
    
    // if we have gotten stuck sending a command then clear the flag
    if(sending_command)
    {
        if(command_timeout_counter > 1)
        {
            pthread_mutex_lock(&flags_lock);
            sending_command = false;
            command_sent = "";
            pthread_mutex_unlock(&flags_lock);
            command_timeout_counter = 0;
            cout << "Sending command timed out." << endl;
        }
        else
            command_timeout_counter++;
    }
    else
        command_timeout_counter = 0;
        
    // ping the targets
    ping_targets();
    
    cout << "************* Sending C: " << sending_command << " Sending IM: " << sending_im << " Sending data: " << sending_data << " sent usbl fix: " << sent_usbl_fix << endl; 

    return 1;
}

int Evologics_Modem::process_modem_data(char *d, int len, int64_t timestamp)
{
    // if we got this far then the first three charaters are +++
    //pthread_mutex_lock(&flags_lock);
    // work out what the message was
    
    // USBL message
    if(strstr((const char *)d, "USBLLONG") != NULL)
    {
        return process_usbllong(d, timestamp);
    }
    // Short USBL message
    else if(strstr((const char *)d, "USBLANGLES") != NULL)
    {
        return process_usblangles(d, timestamp);
    }    
    // Received an IM
    else if(strstr((const char *)d, "RECVIM") != NULL)
    {
        return process_im(d);
    }
    // Received a piggy back IM
    else if(strstr((const char *)d, "RECVPBM") != NULL)
    {
        return process_pbm(d);
    }
    // Receive a burst data message 
    else if(strstr((const char *)d, "RECV") != NULL)
    {
        return process_burst_data(d);
    }
    // Sent IM information    
    else if(strstr((const char *)d, "FAILEDIM") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_im = false;
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
        im_sent++;
    }
    else if(strstr((const char *)d, "FAILED") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_data = false;
        pthread_mutex_unlock(&flags_lock);
    }
    // Received a cancellation notice for an IM.  This happens if we send beforethe last message was finished sending. The message may still have been received.
    else if (strstr((const char *)d, "CANCELEDIM") != NULL)
    {
    }
    else if (strstr((const char *)d, "CANCELEDPBM") != NULL)
    {
    }
    // Special case of instant messages, we can get the propagation time
    else if(strstr((const char *)d, "DELIVEREDIM") != NULL)
    {
        // Find out who the target was
        char *tokens[2];
        if(chop_string(d, tokens, 2) == 2)
            last_im_target = atoi(tokens[1]);
        last_im_timestamp = timestamp;
        pthread_mutex_lock(&flags_lock);
        sending_im = false;
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
        im_sent++;
        send_command("AT?T");
    }
    else if(strstr((const char *)d, "DELIVERED") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_data = false;
        // reset the ping counter to avoid sending too soon after burst data exchange
        cout << "RECEIVED Burst data reply.  Resetting ping counter." << endl;
        ping_counter = 0;
        pthread_mutex_unlock(&flags_lock);
    }
    else if(strstr((const char *)d, "AT?T") != NULL || command_sent == "AT?T")
    {
        cout << "Evologics: Received AT?T reply " << d << endl;
//        char *tokens[4];
        evologics_range_t er;
	er.time = atoi(d);
/*
        if(chop_string(d, tokens, 4) == 1)
        {
            er.time = atoi(tokens[3]);
        } else if(chop_string(d, tokens, 1) == 1) {
            er.time = atoi(tokens[0]);
        } else {
            return 0;
        }
*/
        er.target = last_im_target;
        er.source = local_address;
        er.utime = last_im_timestamp;

        string target_name;
        char usbl_range_channel_name[64];
        get_target_name(er.target, target_name);
        sprintf(usbl_range_channel_name, "%s.EVO_RANGE.%s", vehicle_name, target_name.c_str());

        lcm->publish(usbl_range_channel_name, &er);
        
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    // Data sent, channel ready
    else if((strstr((const char *)d, "LISTEN") != NULL) ||
        (strstr((const char *)d, "NOISE") != NULL) ||
        (strstr((const char *)d, "ESTABLISH") != NULL))
    {
        pthread_mutex_lock(&flags_lock);
        if(strstr((const char *)d, "LISTEN") != NULL)
        {
            sending_data = false;
            if(drop_counter > drop_at_send)
                cerr << "Dropped data, last message not sent" << endl;
        }
            
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    else if(strstr((const char *)d, "ONLINE") != NULL) 
    {
        pthread_mutex_lock(&flags_lock);
        sending_data = false;
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    // Command response
    else if(strstr((const char *)d, "OK") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    // Command response
    else if(strstr((const char *)d, "DISCONNECT") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    // Target change
    else if(strstr((const char *)d, "?AR") != NULL)
    {
        char *tokens[4];
        if(chop_string(d, tokens, 4) == 4)
            current_target = atoi(tokens[3]);
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    // Request positioning information
    else if(strstr((const char *)d, "?ZU") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    else if(strstr((const char *)d, "OUT_OF_CONTEXT") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    // Get the local address
    else if(strstr((const char *)d, "?AL") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
        char *tokens[4];
        if(chop_string(d, tokens, 4) == 4)
        {
            local_address = atoi(tokens[3]);
            cout << "Local address: " << local_address << endl;
        }
    }
    else if(strstr((const char *)d, "Local Address") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
        char *tokens[3];
        if(chop_string(d, tokens, 3) == 3)
        {
            local_address = atoi(tokens[2]);
            cout << "Local address: " << local_address << endl;
        }
    }
    // Get the drop counter
    else if(strstr((const char *)d, "?ZD") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
        char *tokens[4];
        if(chop_string(d, tokens, 4) == 4)
        {
            drop_counter = atoi(tokens[3]);
        }
    }
    // Get the IM counter
    else if(strstr((const char *)d, "?DI") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        char *tokens[4];
        if(chop_string(d, tokens, 4) == 4)
        {
            if (strstr(tokens[3], "EMPTY") != NULL)
            {
              sending_im = false;
            }
        }
        pthread_mutex_unlock(&flags_lock);
    }    
    // Someting went wrong
    else if(strstr((const char *)d, "ERROR") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        command_sent = "";
        pthread_mutex_unlock(&flags_lock);
    }
    else
    {
        cerr << "Unknown modem message: " << d;
    }
    
    //pthread_mutex_unlock(&flags_lock);
    return 1;   
}

int Evologics_Modem::process_lcm_data(unsigned char *d, int size)
{
    // first chack the crc
    unsigned long crc = crc32(0, &d[3], size - 9);
    unsigned long data_crc = *(unsigned long *)&d[size - 6];
    
    if((data_crc & 0xFFFFFFFF) != (crc & 0xFFFFFFFF))
    {
        cerr << "LCM data CRC error\n";
        //printf("0x%X 0x%X\n", crc, data_crc);
        return 0;
    }

    // get the channel name
    char *channel = (char *)malloc(d[3] + 1);
    memset(channel, 0, d[3] + 1);
    memcpy(channel, &d[4], d[3]);

    void *lcm_data_start = &d[d[3] + 4];
    int lcm_data_length = size - d[3] - 10;
    
    cout << "Publishing lcm data on channel name: " << channel << endl;
    
    // publish the LCM data
    lcm->publish(channel, lcm_data_start, lcm_data_length);
        
    
    return 1;
}

int Evologics_Modem::process_usbllong(char *d, int64_t timestamp)
{
    char *tokens[17];
    int ret;
    ret = chop_string(d, tokens, 17);
    
    if(ret != 17)
        return 0;
    
    // Work out the actual time that the measurment was taken
    double measurement_time = atof(tokens[2]);
    double current_time = atof(tokens[1]);
    
    // we will correct the time as it may not be sync'd with the computer running this code
    //double time_diff = current_time - measurement_time;
    
    evologics_usbl_t ud;
    ud.utime = timestamp;// + (int64_t)(time_diff * 1e6);
    ud.mtime = (int64_t)(measurement_time * 1e6);
    ud.ctime = (int64_t)(current_time * 1e6);
    ud.remote_id = atoi(tokens[3]);
    ud.x = atof(tokens[4]);
    ud.y = atof(tokens[5]);
    ud.z = atof(tokens[6]);
    ud.e = atof(tokens[7]);
    ud.n = atof(tokens[8]);
    ud.u = atof(tokens[9]);
    ud.r = atof(tokens[10]);
    ud.p = atof(tokens[11]);
    ud.h = atof(tokens[12]);
    ud.prop_time = atof(tokens[13]);
    ud.rssi = atoi(tokens[14]);
    ud.integrity = atoi(tokens[15]);
    ud.accuracy = atof(tokens[16]);
    
    string target_name;
    char usbl_fix_channel_name[128];
    get_target_name(ud.remote_id, target_name);
    snprintf(usbl_fix_channel_name, 128, "%s.EVO_USBLFIX.%s", vehicle_name, target_name.c_str());

    lcm->publish(usbl_fix_channel_name, &ud);
    
    // put it in the queue for position calculation
    
    /*if(fixq != NULL)
    {
        evologics_usbl_t *fq = new evologics_usbl_t;
        memcpy(fq, &ud, sizeof(evologics_usbl_t));
        fixq->push(fq);
    }*/
    
    return 1;
}

int Evologics_Modem::process_usblangles(char *d, int64_t timestamp)
{
    char *tokens[14];
    int ret;
    ret = chop_string(d, tokens, 14);
    
    if(ret != 14)
        return 0;
    
    // Work out the actual time that the measurment was taken
    double measurement_time = atof(tokens[2]);
    double current_time = atof(tokens[1]);
    
    // we will correct the time as it may not be sync'd with the computer running this code
    //double time_diff = current_time - measurement_time;
    
    evologics_usbl_angles_t ud;
    ud.utime = timestamp;// + (int64_t)(time_diff * 1e6);
    ud.mtime = (int64_t)(measurement_time * 1e6);
    ud.ctime = (int64_t)(current_time * 1e6);
    ud.remote_id = atoi(tokens[3]);
    ud.lbearing = atof(tokens[4]);
    ud.lelevation = atof(tokens[5]);
    ud.bearing = atof(tokens[6]);
    ud.elevation = atof(tokens[7]);
    ud.r = atof(tokens[8]);
    ud.p = atof(tokens[9]);
    ud.h = atof(tokens[10]);
    ud.rssi = atoi(tokens[11]);
    ud.integrity = atoi(tokens[12]);
    ud.accuracy = atof(tokens[13]);

    string target_name;
    char angles_channel_name[128];
    get_target_name(ud.remote_id, target_name);
    snprintf(angles_channel_name, 128, "%s.EVO_ANGLES.%s", vehicle_name, target_name.c_str());

    lcm->publish(angles_channel_name, &ud);
    
    // put it in the queue for position calculation
    
    /*if(fixq != NULL)
    {
        evologics_usbl_t *fq = new evologics_usbl_t;
        memcpy(fq, &ud, sizeof(evologics_usbl_t));
        fixq->push(fq);
    }*/
    
    return 1;
}

int Evologics_Modem::process_im(char *d)
{
    vector<string> tokens = chop_string(d, 10);
    if (tokens.size() != 10)
       return 0;

    int size = atoi(tokens[1].c_str());
    int source = atoi(tokens[2].c_str());
    int target = atoi(tokens[3].c_str());
    const char *data = strstr(d, tokens[9].c_str()); 

    cout << "*** EVOLOGICS modem " << local_address << " received instant message from " << source << " to " << target << " of size " << size << " with data " << data << endl;
    
    // check if the IM data contains LCM data
    if (!strncmp(data, "LCM", 3) && target == local_address)
       process_lcm_data((unsigned char *)data, size);

    return 1;
}

int Evologics_Modem::process_pbm(char *d)
{
    vector<string> tokens = chop_string(d, 9);
    if (tokens.size() != 9)
       return 0;

    int size = atoi(tokens[1].c_str());
    int source = atoi(tokens[2].c_str());
    int target = atoi(tokens[3].c_str());
    const char *data = strstr(d, tokens[8].c_str()); 

    cout << "*** EVOLOGICS modem " << local_address << " received piggy back instant message from " << source << " to " << target << " of size " << size << " with " << tokens[8].length() << " bytes of data " << data << endl;
    
    // check if the IM data contains LCM data
    if (!strncmp(data, "LCM", 3) && target == local_address)
       process_lcm_data((unsigned char *)data, size);

    return 1;
}

int Evologics_Modem::process_burst_data(char *d)
{
    vector<string> tokens = chop_string(d, 10);
    if (tokens.size() != 10)
       return 0;

    int size = atoi(tokens[1].c_str());
    int source = atoi(tokens[2].c_str());
    int target = atoi(tokens[3].c_str());
    const char *data = strstr(d, tokens[9].c_str());

    cout << "*** EVOLOGICS modem " << local_address << " received piggy back instant message from " << source << " to " << target << " of size " << size << " with data " << data << endl;

    // check if the IM data contains LCM data
    if (!strncmp(data, "LCM", 3) && target == local_address)
       process_lcm_data((unsigned char *)data, size);

    return 1;
}


int Evologics_Modem::send_lcm_data(unsigned char *d, int size, int target, const char *dest_channel, bool use_pbm)
{
    // first clear the modem of any pending data messages
    //clear_modem();
   
    // we will be adding 9 extra bytes in addition to the name and the name size
    int data_size = strlen(dest_channel) + 10 + size;
    unsigned char *dout = (unsigned char *)malloc(data_size);
    
    // Put it together    
    strncpy((char *)dout, "LCM", 3);                                                // Prefix
    dout[3] = strlen(dest_channel);                                                 // Size of channel name
    memcpy(&dout[4], dest_channel, strlen(dest_channel));                           // Channel name
    memcpy(&dout[4 + strlen(dest_channel)], d, size);                               // data
    
    // the CRC is of everthing other then the LCM and LE tags
    unsigned long crc = crc32(0, &dout[3], size + strlen(dest_channel) + 1);

    memcpy(&dout[4 + strlen(dest_channel) + size], &crc, 4);                       // CRC
    strncpy((char *)&dout[8 + strlen(dest_channel) + size], "LE", 2);              // Suffix
    
   cout << "Preparing to send data of size " << data_size << " for target " << target << " on channel " << dest_channel << " with data " << dout << endl;
   // check if we can send this as an instant message
   if (data_size <= 64)
   { 
       char im_msgbuf[128];
       memset(im_msgbuf, 0, 128);
       if (use_pbm == true)
          sprintf(im_msgbuf, "AT*SENDPBM,%d,%d,",data_size, target);
       else
          sprintf(im_msgbuf, "AT*SENDIM,%d,%d,ack,",data_size, target);
       pthread_mutex_lock(&(write_lock));
       write(fd, im_msgbuf, strlen(im_msgbuf));
       write(fd, dout, data_size);
       write(fd, term, term_len);
       //write(fd, &term, 1);
       pthread_mutex_lock(&flags_lock);
       sending_command = true;
       sending_im = true;
       pthread_mutex_unlock(&flags_lock);
       pthread_mutex_unlock(&(write_lock));
   } else {
       // FIXME: Lock out sending of more than one USBL_FIX per IM ping.  This
       //        could be more elegantly handled with a state machine or
       //        some other mechanism.
       // Send the data.  If this is a USBL_FIX, check if it should be sent.
       if(strstr(dest_channel, "USBL_FIX") == NULL || 
          (sent_usbl_fix == false && send_fixes == true))
       {
           char im_msgbuf[128];
           memset(im_msgbuf, 0, 128);
           sprintf(im_msgbuf, "AT*SEND,%d,%d,", data_size, target);
           cout << im_msgbuf << endl;
           pthread_mutex_lock(&(write_lock));
           write(fd, im_msgbuf, strlen(im_msgbuf));
           write(fd, dout, data_size);
       //    write(fd, &term, 1);
           write(fd, term, term_len);
           pthread_mutex_lock(&flags_lock);
           sending_command = true;
           sending_data = true;
           if (strstr(dest_channel, "USBL_FIX") != NULL)
               sent_usbl_fix = true;
           pthread_mutex_unlock(&flags_lock);
           pthread_mutex_unlock(&(write_lock));
       } else {
           cout << "Not sending USBL_FIX" << endl;
       }
   }

   return 1;
}

int Evologics_Modem::send_command(const char *d)
{
    char msg[32];
    memset(msg, 0, 32);
    memcpy(msg, d, strlen(d));
    memcpy(&msg[strlen(d)], term, term_len);
  //  sprintf(msg, "%s%s", d, term);
    cout << "Queuing command " << msg;
    //send_data((unsigned char *)msg, strlen(msg), evo_command, 0, 0);
    //DEBUG_PRINTF(("Sending command: %.*s\n", strlen(msg), &msg));
    pthread_mutex_lock(&(write_lock));
    int bytes = write(fd, msg, strlen(msg));
    pthread_mutex_unlock(&(write_lock));
    if( bytes == -1)
    {
        perror("Failed to send command to modem:");
        sending_command = false;
        reopen_port();
    }
    else
    {
        sending_command = true;
        command_sent = d;
        cout << "Evologics_Modem: send_command sent command " << command_sent << ". Waiting for response." << endl;
    }
    wait_for_command_response();
    return 1;
}

int Evologics_Modem::send_ping(int target)
{
    char msg[32];
    if(!sending_im)
    {
        memset(msg, 0, 32);
        sprintf(msg, "AT*SENDIM,5,%d,ack,%05d", target, target);
        send_command(msg);
        pthread_mutex_lock(&flags_lock);
        sending_im = true;
        sent_usbl_fix = false;
        pthread_mutex_unlock(&flags_lock);

        return 1;
    }

    return 0;
}

int Evologics_Modem::clear_modem()
{
    send_command("ATZ4");
    return 1;
}

int Evologics_Modem::disconnect_modem()
{
    send_command("ATZ1");
    return 1;
}

int Evologics_Modem::wait_for_command_response()
{
    int64_t timestamp;
    timestamp = timestamp_now();
    while(sending_command && ((timestamp_now() - timestamp)/1e6 < 1.0))
    {
        usleep(100e3);
    }
    
    if ((timestamp_now() - timestamp)/1e6 >= 1.0)
    {
        cout << "Timed out waiting for command response" << endl;
        return 0;
    }

    return 1;
}


int Evologics_Modem::process()
{
    int lcm_fd = lcm->getFileno();
    fd_set rfds;
    char buf[MAX_BUF_LEN];
    
    while(!loop_exit)
    {
        // check the port status, broekn pipes
        if(pipe_broken)
        {
            cout << "Evologics_Modem: process found pipe_broken.  Reconnecting." << endl;
            if (open_port(ip, port) > 0)
	        pipe_broken = false;
        }
        
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
        //else
        //    cout << "select timeout\n";
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
    Evologics_Modem *modem = new Evologics_Modem;
    modem->load_config(basename((argv[0])));
    modem->init();
    
    modem->process();
    
    delete modem;
    return 1;
}
