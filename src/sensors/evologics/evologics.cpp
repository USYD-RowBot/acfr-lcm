#include "evologics.hpp"


// read thread
static void *read_thread(void *u)
{
    Evologics *evo = (Evologics *)u;
 
    cout << "Read thread started\n";
    
    fd_set rfds;
    char buf[MAX_BUF_LEN];
    int bytes;
    int data_type;
    bool data_good;
    int64_t timestamp;
    
    while(!evo->thread_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (evo->fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        memset(buf, 0, MAX_BUF_LEN);
        data_good = false;
        
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        timestamp = timestamp_now();
        if(ret > 0)
        {
            // we need to do single character reads as the termination character can change
            // this minimum return is going to be 3 characters
            bytes = 0;
            char *d = &buf[0];
            data_type = 0;
            
            bytes += read(evo->fd, &d[bytes], 1);
            if(buf[0] != '+' && buf[0] != 'L')
                continue;
            
            
            while(bytes < 3)
                bytes += read(evo->fd, &buf[bytes], 3-bytes); 
                 
            // check for the preamble, either +++ or LCM
            if(!strncmp(buf, "LCM", 3))
            {
                while(bytes < MAX_BUF_LEN)
                {
                    bytes += read(evo->fd, &buf[bytes], 1);
                    if((buf[bytes-2] == 'L') && (buf[bytes-1] == 'E'))
                    {
                        data_good = true;
                        data_type = 2;
                        break;
                    }
                    
                    // for the strange case where we do not see the termination due to error
                    // but we do get a +++, copy the first 3 bytes of the message, set the counter to 3
                    // and continue
                    if((buf[bytes-3] == '+') && (buf[bytes-2] == '+') && (buf[bytes-1] == '+'))
                    {
                        memcpy(buf, &buf[bytes-3], 3);
                        memset(&buf[3], 0, MAX_BUF_LEN-3);
                        bytes = 3;
                        cout << "***** Found a +++ in the LCM data\n";
                        break;
                    }
                    
                }
            }
            
            
            if(!strncmp(buf, "+++", 3) && data_type == 0)
            {
                while(bytes < (MAX_BUF_LEN))
                {
                    bytes += read(evo->fd, &buf[bytes], 1);
                    if((buf[bytes-2] == 0x0D) || (buf[bytes-1] == 0x0A))
                        break;
                    
                    data_good = true;
                    data_type = 1;
                }
            }    

            
            // only proceed if the data is good and we have more then just the header
            if(data_good && bytes > 3)
            {
            
                if(data_type == 1)
                {   
                    // a modem message
                    cout << "Received modem data: " << bytes << " bytes. " << buf << endl;
                    evo->parse_modem_data(buf, bytes, timestamp);
                }
                else if(data_type == 2)
                {
                    // LCM data
                    cout << "Received LCM data: " << bytes << " bytes" << endl;
                    evo->parse_lcm_data((unsigned char *)buf, bytes);
                }
            }
        } else if (ret == -1) {
            char buf[64];
            sprintf(buf, "Select failed on read thread %d", evo->fd);
            perror(buf);
            evo->reopen_port();
        }
    }
    cout << "Read thread exit\n";
    return NULL;
}

               
// Helper functions

vector<string> chop_string(char *data, int nTokens)
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
int chop_string(char *data, char **tokens, int nTokens)
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

// LCM handlers
void on_heartbeat(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *hb, Evologics* ev) 
{
    ev->handle_heartbeat(); 
}


Evologics::Evologics(char *_device, int _baud, char *_parity, lcm::LCM *_lcm, queue<evologics_usbl_t *> *q, int _ping_timeout)
{
    device = _device;
    baud = _baud;
    parity = _parity;
    use_serial_port = true;
    use_ip_port = false;

    init(_lcm, q, _ping_timeout);
    fd = open_serial_port();
    term = '\r';
    start_threads();
}

Evologics::Evologics(char *_ip, char *_port, lcm::LCM *_lcm, queue<evologics_usbl_t *> *q, int _ping_timeout)
{
    ip = _ip;
    port = _port;
    use_serial_port = false;
    use_ip_port = true;

    init(_lcm, q, _ping_timeout);
    fd = open_port(ip.c_str(), port.c_str());
    term = '\n';
    start_threads();
}

Evologics::~Evologics()
{
    disconnect_modem();
    thread_exit = 1;
    pthread_join(read_thread_id, NULL);
}

int Evologics::init(lcm::LCM *_lcm, queue<evologics_usbl_t *> *q, int _ping_timeout)
{
    lcm = _lcm;
    fixq = q;
    ping_timeout = _ping_timeout;
    
    sending_im = false;
    sending_data = false;
    sending_command = false;
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

int Evologics::start_threads()
{
    pthread_create(&read_thread_id, NULL, read_thread, this);
    pthread_detach(read_thread_id);
}

int Evologics::reopen_port()
{
    close(fd);

    if (use_serial_port)
    {
       fd = open_serial_port();
    } else if (use_ip_port) {
       fd = open_port(ip.c_str(), port.c_str());
    }
    // if the reconnection fails, wait for a while before reattempting
    if (fd == -1)
       sleep(1);
    return fd;
}

int Evologics::open_serial_port()
{
   pthread_mutex_lock(&write_lock);
   int evo_fd = serial_open(device.c_str(), serial_translate_speed(baud), serial_translate_parity(parity.c_str()), 1);
   serial_set_noncanonical(evo_fd, 1, 0);

   tcflush(evo_fd,TCIOFLUSH);
   pthread_mutex_unlock(&write_lock);

   return evo_fd;
}


int Evologics::open_port(const char *ip, const char *port)
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

    if (evo_addr == NULL)
    {
       printf("Could not connect to %s on port %s\n", ip, port);
    } else {
       printf("Successfully connected to %s on port %s\n", ip, port);

       freeaddrinfo(result);

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

int Evologics::start_handlers()
{
    // LCM subscriptions
    lcm->subscribeFunction("HEARTBEAT_1HZ", on_heartbeat, this);
    
    return 1;
}

int Evologics::handle_heartbeat()
{
    // If we are sending data we need to check to make sure it has gone through
    // we will send an AT?S one a second until it has completed.
    if(sending_data)
    {
        send_command("+++AT?S");
    }
    

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
        //send_command_front("+++ATZ3");
        //send_command("+++ATZ3");
        
        im_counter = 0;
        
        // request the size of the current IM queue
        //send_command_front("+++AT?DI");
        //send_command("+++AT?DI");
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
            pthread_mutex_unlock(&flags_lock);
            command_timeout_counter = 0;
            cout << "Sending command timed out." << endl;
        }
        else
            command_timeout_counter++;
    }
    else
        command_timeout_counter = 0;
        
    
    cout << "************* Sending C: " << sending_command << " Sending IM: " << sending_im << " Sending data: " << sending_data << endl; 

    return 1;
}

int Evologics::parse_modem_data(char *d, int len, int64_t timestamp)
{
    // if we got this far then the first three charaters are +++
    //pthread_mutex_lock(&flags_lock);
    // work out what the message was
    
    // USBL message
    if(strstr((const char *)d, "USBLLONG") != NULL)
    {
        parse_usbllong(d, timestamp);
    }
    // Short USBL message
    else if(strstr((const char *)d, "USBLANGLES") != NULL)
    {
        parse_usblangles(d, timestamp);
    }    
    // Received an IM
    else if(strstr((const char *)d, "RECVIM") != NULL)
        parse_im(d);
    // Received a piggy back IM
    else if(strstr((const char *)d, "RECVPBM") != NULL)
        parse_pbm(d);
    // Sent IM information    
    else if(strstr((const char *)d, "FAILEDIM") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_im = false;
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
        im_sent++;
    }
    // Received a cancellation notice for an IM.  This happens if we send beforethe last message was finished sending. The message may still have been received.
    else if (strstr((const char *)d, "CANCELEDIM") != NULL)
    {
    }
    // Special case of instant messages, we can get the propagation time
    else if(strstr((const char *)d, "DELIVEREDIM") != NULL)
    {
        // Find out who the target was
        char *tokens[4];
        if(chop_string(d, tokens, 4) == 4)
            last_im_target = atoi(tokens[3]);
        last_im_timestamp = timestamp;
        pthread_mutex_lock(&flags_lock);
        sending_im = false;
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
        im_sent++;
        send_command("+++AT?T");
    }
    else if(strstr((const char *)d, "AT?T") != NULL)
    {
        sending_command = false;
        char *tokens[4];
        if(chop_string(d, tokens, 4) == 4)
        {
            evologics_range_t er;
            er.time = atoi(tokens[3]);
            er.target = last_im_target;
            er.source = local_address;
            er.utime = last_im_timestamp;
            lcm->publish("EVOLOGICS_RANGE", &er);
        }
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
        pthread_mutex_unlock(&flags_lock);
    }
    else if(strstr((const char *)d, "ONLINE") != NULL) 
    {
        pthread_mutex_lock(&flags_lock);
        sending_data = false;
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
    }
    // Command response
    else if(strstr((const char *)d, "OK") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
    }
    // Command response
    else if(strstr((const char *)d, "DISCONNECT") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
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
        pthread_mutex_unlock(&flags_lock);
    }
    // Request positioning information
    else if(strstr((const char *)d, "?ZU") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
    }
    else if(strstr((const char *)d, "OUT_OF_CONTEXT") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
    }
    // Get the local address
    else if(strstr((const char *)d, "?AL") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
        char *tokens[4];
        if(chop_string(d, tokens, 4) == 4)
        {
            local_address = atoi(tokens[3]);
            cout << "Local address: " << local_address << endl;
        }
    }
    // Get the drop counter
    else if(strstr((const char *)d, "?ZD") != NULL)
    {
        pthread_mutex_lock(&flags_lock);
        sending_command = false;
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
        pthread_mutex_unlock(&flags_lock);
    }
    else
        cerr << "Unknown modem message: " << d;
    
    //pthread_mutex_unlock(&flags_lock);
    return 1;   
}

int Evologics::parse_lcm_data(unsigned char *d, int size)
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

int Evologics::parse_usbllong(char *d, int64_t timestamp)
{
    char *tokens[19];
    int ret;
    ret = chop_string(d, tokens, 19);
    
    if(ret != 19)
        return 0;
    
    // Work out the actual time that the measurment was taken
    double measurement_time = atof(tokens[4]);
    double current_time = atof(tokens[3]);
    
    // we will correct the time as it may not be sync'd with the computer running this code
    //double time_diff = current_time - measurement_time;
    
    evologics_usbl_t ud;
    ud.utime = timestamp;// + (int64_t)(time_diff * 1e6);
    ud.mtime = (int64_t)(measurement_time * 1e6);
    ud.ctime = (int64_t)(current_time * 1e6);
    ud.remote_id = atoi(tokens[5]);
    ud.x = atof(tokens[6]);
    ud.y = atof(tokens[7]);
    ud.z = atof(tokens[8]);
    ud.e = atof(tokens[9]);
    ud.n = atof(tokens[10]);
    ud.u = atof(tokens[11]);
    ud.r = atof(tokens[12]);
    ud.p = atof(tokens[13]);
    ud.h = atof(tokens[14]);
    ud.prop_time = atof(tokens[15]);
    ud.rssi = atoi(tokens[16]);
    ud.integrity = atoi(tokens[17]);
    ud.accuracy = atof(tokens[18]);
    
    lcm->publish("EVOLOGICS_USBL", &ud);
    
    // put it in the queue for position calculation
    
    if(fixq != NULL)
    {
        evologics_usbl_t *fq = new evologics_usbl_t;
        memcpy(fq, &ud, sizeof(evologics_usbl_t));
        fixq->push(fq);
    }
    
    return 1;
}

int Evologics::parse_usblangles(char *d, int64_t timestamp)
{
    char *tokens[16];
    int ret;
    ret = chop_string(d, tokens, 16);
    
    if(ret != 16)
        return 0;
    
    // Work out the actual time that the measurment was taken
    double measurement_time = atof(tokens[4]);
    double current_time = atof(tokens[3]);
    
    // we will correct the time as it may not be sync'd with the computer running this code
    //double time_diff = current_time - measurement_time;
    
    evologics_usbl_angles_t ud;
    ud.utime = timestamp;// + (int64_t)(time_diff * 1e6);
    ud.mtime = (int64_t)(measurement_time * 1e6);
    ud.ctime = (int64_t)(current_time * 1e6);
    ud.remote_id = atoi(tokens[5]);
    ud.lbearing = atof(tokens[6]);
    ud.lelevation = atof(tokens[7]);
    ud.bearing = atof(tokens[8]);
    ud.elevation = atof(tokens[9]);
    ud.r = atof(tokens[10]);
    ud.p = atof(tokens[11]);
    ud.h = atof(tokens[12]);
    ud.rssi = atoi(tokens[13]);
    ud.integrity = atoi(tokens[14]);
    ud.accuracy = atof(tokens[15]);

    lcm->publish("EVOLOGICS_USBL_ANGLES", &ud);
    
    return 1;
}


int Evologics::parse_im(char *d)
{
    /*char *tokens[12];
    int ret;
    ret = chop_string(d, tokens, 12);
    
    if(ret != 12)
    {
        cout << "*** EVOLOGICS modem got IM with " << ret << " fields.  Expecting 12." << endl;
        return 0;
    }
    */
    vector<string> tokens = chop_string(d, 12);

    int size = atoi(tokens[3].c_str());
    int source = atoi(tokens[4].c_str());
    int target = atoi(tokens[5].c_str());
    const char *data = strstr(d, tokens[11].c_str()); 

    cout << "*** EVOLOGICS modem " << local_address << " received instant message from " << source << " to " << target << " of size " << size << " with data " << data << endl;
    
    // check if the IM data contains LCM data
    if (!strncmp(data, "LCM", 3) && target == local_address)
       parse_lcm_data((unsigned char *)data, size);

    return 1;
}

int Evologics::parse_pbm(char *d)
{
    vector<string> tokens = chop_string(d, 11);

    int size = atoi(tokens[3].c_str());
    int source = atoi(tokens[4].c_str());
    int target = atoi(tokens[5].c_str());
    const char *data = strstr(d, tokens[10].c_str()); 

    cout << "*** EVOLOGICS modem " << local_address << " received piggy back instant message from " << source << " to " << target << " of size " << size << " with data " << data << endl;
    
    // check if the IM data contains LCM data
    if (!strncmp(data, "LCM", 3) && target == local_address)
       parse_lcm_data((unsigned char *)data, size);

    return 1;
}

int Evologics::send_lcm_data(unsigned char *d, int size, int target, const char *dest_channel)
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
       sprintf(im_msgbuf, "+++AT*SENDIM,%d,%d,ack,",data_size, target);
       //sprintf(im_msgbuf, "+++AT*SENDPBM,%d,%d,",data_size, target);
       //cout << im_msgbuf << endl;
       pthread_mutex_lock(&(write_lock));
       write(fd, im_msgbuf, strlen(im_msgbuf));
       write(fd, dout, data_size);
       write(fd, &term, 1);
       pthread_mutex_lock(&flags_lock);
       sending_command = true;
       sending_im = true;
       pthread_mutex_unlock(&flags_lock);
       pthread_mutex_unlock(&(write_lock));
   } else {
       if(target != current_target)
       {
           // we need to change targets, this is the only place we are doing
           // out of queue command writes to the modem
           send_command("+++ATZ4");
       
           int retry = 0;
           //while(evo->current_target != (*od)->target && retry < 5)
           {
               cout << "Target, current: " << current_target << "   destination: " << target << endl;
               char msg[32];
               sprintf(msg, "+++AT!AR%d", target);
               send_command(msg);

               send_command("++AT?AR");

               retry++;
           }
       }     

       if(target != current_target)
       {
           cerr << "Could not change the modem data target\n";
       } else {
           pthread_mutex_lock(&flags_lock);
           if(!sending_data)
           {
               pthread_mutex_lock(&(write_lock));
               int bytes = write(fd, dout, size);
               pthread_mutex_unlock(&(write_lock));
               if( bytes == -1)
               {
                   perror("Failed to send data to modem:");
                   sending_data = false;
                   reopen_port();
               }
               else
               {
                   DEBUG_PRINTF(("Sending %d bytes of LCM data to channel: %.*s\n", data_size, dout[3], &dout[4]));
                   drop_at_send = drop_counter;
               }
           }
           pthread_mutex_unlock(&flags_lock);    
       }
   }

   return 1;
}

int Evologics::send_command(const char *d)
{
    char msg[32];
    sprintf(msg, "%s%c", d, term);
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
        //command_sent = true;
        sending_command= true;
    }
    wait_for_command_response();
    return 1;
}

int Evologics::send_ping(int target)
{
    char msg[32];
    if(!sending_im)
    {
        memset(msg, 0, 32);
        sprintf(msg, "+++AT*SENDIM,5,%d,ack,%05d", target, target);
        send_command(msg);
        pthread_mutex_lock(&flags_lock);
        sending_im = true;
        pthread_mutex_unlock(&flags_lock);

        return 1;
    }

    return 0;
}

int Evologics::clear_modem()
{
    send_command("+++ATZ4");
    return 1;
}

int Evologics::disconnect_modem()
{
    send_command("+++ATZ1");
    return 1;
}

int Evologics::clear_queues()
{
    return 1;
}

int Evologics::wait_for_command_response()
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
    }

    return 1;
}
