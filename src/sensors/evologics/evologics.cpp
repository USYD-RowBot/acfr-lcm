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
            bytes += read(evo->fd, &buf[bytes], 1);
            if(buf[0] != '+' && buf[0] != 'L')
                continue;
            
            
            while(bytes < 3)
                bytes += read(evo->fd, &buf[bytes], 3-bytes); 
                 
            // check for the preamble, either +++ or LCM
            if(!strncmp(buf, "+++", 3))
            {
                while(bytes < MAX_BUF_LEN)
                {
                    bytes += read(evo->fd, &buf[bytes], 1);
                    if((buf[bytes-2] == 0x0D) || (buf[bytes-1] == 0x0A))
                        break;
                    
                    data_good = true;
                    data_type = 1;
                }
            }    
            else if(!strncmp(buf, "LCM", 3))
            {
                while(bytes < MAX_BUF_LEN)
                {
                    bytes += read(evo->fd, &buf[bytes], 1);
                    if((buf[bytes-2] == 'L') && (buf[bytes-1] == 'E'))
                        break;
                    data_good = true;
                    data_type = 2;
                }
            }

            // only proceed if the data is good and we have more then just the header
            if(data_good && bytes > 3)
            {
            
                if(data_type == 1)
                {   
                    // a modem message
                    cout << "Received modem data: " << bytes << " bytes. " << buf;
                    evo->parse_modem_data(buf, bytes, timestamp);
                }
                else if(data_type == 2)
                {
                    // LCM data
                    cout << "Received LCM data: " << bytes << " bytes" << endl;
                    evo->parse_lcm_data((unsigned char *)buf, bytes);
                }
            }
        }
    }
    cout << "Read thread exit\n";
    return NULL;
}

// write thread
static void *write_thread(void *u)
{
    Evologics *evo = (Evologics *)u;
 
    cout << "Write thread started\n";
    
    bool data_sent;
    
    // wait for data to appear in the write queue, when it does check to see how old
    // it is, if it is too old don't send it, otherwise send it
    while(!evo->thread_exit)
    {
        //cout << "Command Q size: " << evo->command_queue.size() << endl;
        data_sent = false;
        if(!evo->command_queue.empty())
        {
            // TODO: impletement modem locking here
        
            Evo_Data_Out *od = evo->command_queue.front();
            
            if((timestamp_now() - od->timestamp) < MAX_DATA_AGE)
            {
                pthread_mutex_lock(&evo->flags_lock);
                if(!evo->sending_command)
                {
//                    DEBUG_PRINTF(("Command age: %ld\n", timestamp_now() - od->timestamp));
                    DEBUG_PRINTF(("Sending command: %.*s, Age: %ld\n", od->size - 1, od->data, timestamp_now() - od->timestamp));
                    int bytes = write(evo->fd, od->data, od->size);
                    if( bytes == -1)
                    {
                        cerr << "Failed to send data to modem\n";
                        evo->sending_command = false;
                    }
                    else
                    {
                        data_sent = true;
                        evo->sending_command= true;
                        if(od->type == evo_im)
                            evo->sending_im = true;
                    }
                }
                pthread_mutex_unlock(&evo->flags_lock);
            }
            else
            {
                DEBUG_PRINTF(("Deleteing command from queue due to age: %.*s\n", od->size-1, od->data));
                data_sent = true;   // Data age timeout
                evo->sending_command = false;
                evo->sending_im = false;
            }
                
            if(data_sent)
            {   
                // remove it from the queue
                delete od->data;
                delete od;
                evo->command_queue.pop();
            }
        }


        data_sent = false;
        if(!evo->data_queue.empty())
        {
            Evo_Data_Out *od = evo->data_queue.front();
            // check the timestamp
            if((timestamp_now() - od->timestamp) < MAX_DATA_AGE)
            {
                if(od->target != evo->current_target)
                {
                    char msg[32];
                    // we need to change targets, this is the only place we are doing
                    // out of queue command writes to the modem
                    sprintf(msg, "+++ATZ4%c", evo->term);
                    usleep(100e3);
                    write(evo->fd, msg, strlen(msg));
                    
                    int retry = 0;
                    while(evo->current_target != od->target && retry < 5)
                    {
                        cout << "Target, current: " << evo->current_target << "   destination: " << od->target << endl;
                        sprintf(msg, "+++AT!AR%d%c", od->target, evo->term);
                        write(evo->fd, msg, strlen(msg));
                        usleep(1e5);
                        sprintf(msg, "+++AT?AR%c", evo->term); 
                        write(evo->fd, msg, strlen(msg));
                        retry++;
                    }
                }
        
                if(od->target != evo->current_target)
                {
                    cerr << "Could not change the modem data target\n";
                    data_sent = true;
                }
            
            
                pthread_mutex_lock(&evo->flags_lock);
                if(!evo->sending_data)
                {
                    int bytes = write(evo->fd, od->data, od->size);
                    if( bytes == -1)
                    {
                        cerr << "Failed to send data to modem\n";
                        evo->sending_data = false;
                    }
                    else
                    {
                        data_sent = true;
                        evo->sending_data = true;
                    }
                }
                pthread_mutex_unlock(&evo->flags_lock);    
            }
            
            if(data_sent)
            {   
                // remove it from the queue
                delete od->data;
                delete od;
                evo->data_queue.pop();
            }
        }


        // if the queue is empty, delay a little so not to use so many CPU cycles
        if(evo->data_queue.empty() && evo->command_queue.empty())
            usleep(100e3);  // 100ms
    }
                           
 
    cout << "Write thread exit\n";
    return NULL;   
}

                
// Helper functions

// Cut a string up
int chop_string(char *data, char **tokens)
{
    char *token;
    int i = 0;
    
    token = strtok(data, " ,:*?!");
    while(token != NULL) 
    {
        tokens[i++] = token;            
        token = strtok(NULL, " ,:*?!");            
    }
    return i;        
}                    


Evologics::Evologics(int _fd, char _term, lcm::LCM *_lcm, queue<evologics_usbl_t *> *q)
{
    fd = _fd;
    term = _term;
    lcm = _lcm;
    fixq = q;
    
    sending_im = false;
    sending_data = false;
    sending_command = false;
    current_target = 0;
    
    thread_exit = 0;
    pthread_mutex_init(&flags_lock, NULL);    
    pthread_create(&read_thread_id, NULL, read_thread, this);
    pthread_detach(read_thread_id);

    pthread_create(&write_thread_id, NULL, write_thread, this);
    pthread_detach(write_thread_id);
}

Evologics::~Evologics()
{
    thread_exit = 1;
    pthread_join(read_thread_id, NULL);
    pthread_join(write_thread_id, NULL);
}

int Evologics::parse_modem_data(char *d, int len, int64_t timestamp)
{
    // if we got this far then the first three charaters are +++
    pthread_mutex_lock(&flags_lock);
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
    // Sent IM information    
    else if((strstr((const char *)d, "FAILEDIM") != NULL) ||
        (strstr((const char *)d, "DELIVEREDIM") != NULL) ||
        (strstr((const char *)d, "CANCELEDIM") != NULL))
    {
        sending_im = false;
    }    
    // Data sent, channel ready
    else if((strstr((const char *)d, "LISTEN") != NULL) ||
        (strstr((const char *)d, "NOISE") != NULL) || 
        (strstr((const char *)d, "ESTABLISH") != NULL))
    {
        sending_data = false;
        sending_command = false;
    }
    else if(strstr((const char *)d, "ONLINE") != NULL) 
    {
        sending_data = false;
        sending_command = false;
    }
    // Command response
    else if(strstr((const char *)d, "OK") != NULL)
    {
        sending_command = false;
    }
    // Command response
    else if(strstr((const char *)d, "DISCONNECT") != NULL)
    {
        sending_command = false;
    }
    // Target change
    else if(strstr((const char *)d, "?AR") != NULL)
    {
        char *tokens[4];
        if(chop_string(d, tokens) == 4)
            current_target = atoi(tokens[3]);
        sending_command = false;
    }
    pthread_mutex_unlock(&flags_lock);
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
    
    cout << "Channel name: " << channel << endl;
    
    // publish the LCM data
    lcm->publish(channel, lcm_data_start, lcm_data_length);
        
    
    return 1;
}

int Evologics::parse_usbllong(char *d, int64_t timestamp)
{
    char *tokens[19];
    int ret;
    ret = chop_string(d, tokens);
    
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
    ret = chop_string(d, tokens);
    
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
    return 1;
}

int Evologics::send_lcm_data(unsigned char *d, int size, int target, char *dest_channel)
{
    /*
    // Check to see what the current target is and if its different change it
    char msg[64];
    if(target != current_target)
    {
        
        sprintf(msg, "+++ATZ4%c", term);
        send_command(msg);
        
        int retry = 0;
        while(current_target != target && retry < 5)
        {
            sprintf(msg, "+++AT!AR%d%c", target, term);
            send_command(msg);
            usleep(1000);
            sprintf(msg, "+++AT?AR%c", term); 
            send_command(msg);
            retry++;
        }
        
        if(target != current_target)
        {
            cerr << "Could not change the modem data target\n";
            return 0;
        }
    }    
    */
    
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
    
    send_data(dout, data_size, evo_data, target);
    
    /*
    //cout << "Data size: " << data_size << endl;
    // make sure we can send
    pthread_mutex_lock(&flags_lock);
    int count = 0;
    while(sending_data && count < 5)
    {
        pthread_mutex_unlock(&flags_lock);
        usleep(5e5);
        count++;
        pthread_mutex_lock(&flags_lock);
    }
    
    // send the data
    int bytes = write(fd, dout, data_size);
    if( bytes == -1)
    {
        cerr << "Failed to send data to modem\n";
        return 0;
    }
    cout << "Sending LCM message: " << dest_channel << endl;
    sending_data = true;
    pthread_mutex_unlock(&flags_lock);    
    */
    return 1;
    
}

int Evologics::send_data(unsigned char *d, int size, int type, int target)
{
    // allocate the memory
    Evo_Data_Out *od = new Evo_Data_Out;
    od->data = new unsigned char[size];
    memset(od->data, 0, size);
    memcpy(od->data, d, size);
    od->target = target;
    od->size = size;
    od->type = type;
    od->timestamp = timestamp_now();
    
    // put it in the queue
    if(type == evo_data)
        data_queue.push(od);
    else if(type == evo_command || type == evo_im) 
        command_queue.push(od);
    return 1;
}

int Evologics::send_command(const char *d)
{
    send_data((unsigned char *)d, strlen(d), evo_command, 0);
    /*
    pthread_mutex_lock(&flags_lock);
    int count = 0;
    while(sending_command && count < 30)
    {
        pthread_mutex_unlock(&flags_lock);
        usleep(5e5);
        count++;
        pthread_mutex_lock(&flags_lock);
    }
    if(sending_command)
    {
        cerr << "Could not send command, modem busy\n";
        cerr << "Clearing flag anyway\n";
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
        return 0;
    }
        
    cerr << "Sending command: " << d;    
    
    int bytes = write(fd, d, strlen(d));
    if( bytes == -1)
    {
        cerr << "Failed to send data to modem\n";
        sending_command = false;
        pthread_mutex_unlock(&flags_lock);
        return 0;
    }
    
    sending_command = true;
    pthread_mutex_unlock(&flags_lock);    
    */
    return 1;
}

int Evologics::send_ping(int target)
{
    char msg[32];
    if(!sending_im)
    {
        memset(msg, 0, 32);
        sprintf(msg, "+++AT*SENDIM,1,%d,ack,%d000%c", target, target, term);
        send_data((unsigned char *)msg, strlen(msg), evo_im, target);
/*        
        pthread_mutex_lock(&flags_lock);
        sending_im = true;
        pthread_mutex_unlock(&flags_lock);
        if(!send_command((const char *)msg))
        {
            pthread_mutex_lock(&flags_lock);
            sending_im = false;
            pthread_mutex_unlock(&flags_lock);
        }
*/
        return 1;
    }

    return 0;
}

int Evologics::clear_modem()
{
    char msg[32];
    sprintf(msg, "+++ATZ4%c", term);
    send_command(msg);
    return 1;
}

int Evologics::clear_queues()
{
    queue<Evo_Data_Out *> empty;
    swap(command_queue, empty);
    
//    data_queue.clear();
//    fixq.clear();
    return 1;
}

