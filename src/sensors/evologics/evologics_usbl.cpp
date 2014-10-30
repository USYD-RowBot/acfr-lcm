#include "evologics_usbl.hpp"

int loop_exit;

// Handle a GPS message, this will only work for Version 3 of the GPSD software
// which everything should run by now
void on_gpsd(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const gpsd3_t *gps, Evologics_Usbl* ev) 
{
	// Accept GPS only under the following conditions:
	// 1) Status is good (1=GPS; 2=DGPS)
	// 2) Mode is >= 2 (2D or 3D fix).  
	// 3) Satellites used >= 3. 
	// (2) and (3) are redundant but are generated 
	// asynchronously --- a mode 2 fix may still be reported after
	// the number of satellites has dropped to 0.

    if((gps->status >= 1) && (gps->fix.mode >= 2)) // & (gps->satellites_used >= 3 ))
    {
        memcpy(&ev->gpsd, gps, sizeof(gpsd3_t));
	}
}

void on_novatel(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const novatel_t *nov, Evologics_Usbl* ev) 
{
	// Accept GPS only under the following conditions:
	// nov->status is INS_SOLUTION_GOOD 

    //if(nov->status == "INS_SOLUTION_GOOD" || nov->status == "INS_ALIGNMENT_COMPLETE")
    {
        memcpy(&ev->novatel, nov, sizeof(novatel_t));
	}
}

void on_heartbeat(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *hb, Evologics_Usbl* ev) 
{
    ev->ping_targets(); 
}

void on_evo_usbl(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const evologics_usbl_t *evo, Evologics_Usbl* ev) 
{
    ev->calc_position(evo->x, evo->y, evo->z, evo->accuracy, evo->remote_id); 
}

// a callback with no automatic decoding
void on_lcm(const lcm::ReceiveBuffer* rbuf, const std::string& channel, Evologics_Usbl* ev) 
{
    char dest_channel[64];
    memset(dest_channel, 0, 64);
    strcpy(dest_channel, channel.c_str());
    //strcat(dest_channel, ".3");
    ev->evo->send_lcm_data((unsigned char *)rbuf->data, rbuf->data_size, 3, dest_channel);
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


Evologics_Usbl::Evologics_Usbl()
{
    lcm = new lcm::LCM();
}

int Evologics_Usbl::ping_targets()
{
    // check the state of the link if we are sending data
    //cout << "sending_data=" << state.sending_data << endl;
    if(evo->sending_data)
        evo->send_command("+++AT?S\n");
    
    if(ping_counter == ping_period)
    {
        ping_counter = 0;
        for(int i=0; i<num_targets; i++)
            evo->send_ping(targets[i]);
    }
    else
        ping_counter++;
    
    // check to make sure we haven't gotten stuck sending a ping
    // this can happen if we don't get a response, lets wait 10 seconds
    if(evo->sending_im && ping_time == 10)
        evo->sending_im = false;
        
    if(evo->sending_im && ping_time < 10)
        ping_time++;
        
    if(!evo->sending_im)
        ping_time = 0;
        
    
    for(int i=0; i<num_targets; i++)    
    {
        if(!usbl_send[i] && usbl_send_counter[i] < 5)
            usbl_send_counter[i]++;
        else
        {
            usbl_send[i] = 1;
            usbl_send_counter[i] = 0;
        }
    }
            
    return 1;
}


// The Evologics reference frame is Y forward, X right, Z down
int Evologics_Usbl::calc_position(double xt, double yt, double zt, double accuracy, int remote_id)
{
    //cout << "Calc position\n";
    // We have everything we need to work out where the target is
    SMALL::Pose3D target;
    target.setPosition(xt, yt, zt);
    target.setRollPitchYawRad(0, 0, 0);
    
    SMALL::Pose3D ship;
    ship.setPosition(0, 0, 0);
    if(attitude_source == ATT_NOVATEL)
        ship.setRollPitchYawRad(novatel.roll, novatel.pitch, novatel.heading);  
    else if (attitude_source == ATT_EVOLOGICS)
        ship.setRollPitchYawRad(ahrs.roll, ahrs.pitch, ahrs.heading);
    
    
    SMALL::Pose3D sensor2world = ship.compose(usbl_ins_pose);
    SMALL::Pose3D repro_target = sensor2world.compose(target);
    
    // set up the coordinate reprojection
    char proj_str[64];
    if(gps_source == GPS_NOVATEL)
        sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", novatel.longitude * RTOD, novatel.latitude * RTOD);
    else if(gps_source == GPS_GPSD)
        sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", gpsd.fix.longitude, gpsd.fix.latitude);
    
    cout << proj_str << endl;
    
    //sprintf(proj_str, "+proj=tmerc +lon_0=150.0 +lat_0=-33.5 +units=m");
    
    projPJ pj_tmerc;
    if (!(pj_tmerc = pj_init_plus(proj_str)))
    {
       cerr << "Error creating Proj4 transform\n";
       return 0;
    }
    
    double x = repro_target.getX();
    double y = repro_target.getY();           
    pj_transform(pj_tmerc, pj_latlong, 1, 1, &x, &y, NULL);
    
    usbl_fix_t uf;
    uf.utime = timestamp_now();
    uf.remote_id = remote_id;
    uf.latitude = y;
    uf.longitude = x;
    uf.depth = repro_target.getZ();
    uf.accuracy = accuracy;
    lcm->publish("USBL_FIX", &uf);
    
    
    printf("USBL FIX: target: %d Lat: %3.5f Lon: %3.5f Depth %3.1f Accuracy %2.2f\n", remote_id, uf.latitude * RTOD, uf.longitude *RTOD, uf.depth, uf.accuracy);
    
    
    // Get the target index
    int target_index;
    for(target_index=0; target_index<num_targets; target_index++)
        if(targets[target_index] == remote_id)
            break;
    
    // We will limit how often we send the USBL messages through the modem to once every 5 seconds
    if(usbl_send[target_index])
    {    
        // generate the channel name for the targets LCM message
        char target_channel[10];
        //sprintf(target_channel, "USBL_FIX.%d", remote_id);
        sprintf(target_channel, "USBL_FIX");
        
        int d_size = uf.getEncodedSize();
        unsigned char *d = (unsigned char *)malloc(d_size);
        uf.encode(d, 0, uf.getEncodedSize());
        evo->send_lcm_data(d, d_size, remote_id, target_channel);
        free(d);
        
        usbl_send[target_index] = 0;
    }
    return 1;
    
}


int Evologics_Usbl::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm->getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
        
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "sensors.%s", program_name);

    sprintf(key, "%s.ip", rootkey);
    ip = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.port", rootkey);
    inet_port = bot_param_get_str_or_fail(param, key);
    
    // Ping information
    sprintf(key, "%s.targets", rootkey);
    num_targets = bot_param_get_int_array(param, key, targets, 8);

    sprintf(key, "%s.ping_period", rootkey);
    ping_period = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.gain", rootkey);
    gain = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.source_level", rootkey);
    source_level = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.auto_gain", rootkey);
    auto_gain = bot_param_get_boolean_or_fail(param, key);
    
    sprintf(key, "%s.lcm", rootkey);
    lcm_channels = NULL;
    lcm_channels = bot_param_get_str_array_alloc(param, key);

    double d[6];
    sprintf(key, "%s.usbl_ins", rootkey);
    bot_param_get_double_array_or_fail(param, key, d, 6);
    usbl_ins_pose.setPosition(d[0], d[1], d[2]);
    usbl_ins_pose.setRollPitchYawRad(d[3], d[4], d[5]);

    sprintf(key, "%s.ins_ship", rootkey);
    bot_param_get_double_array_or_fail(param, key, d, 6);
    ins_ship_pose.setPosition(d[0], d[1], d[2]);
    ins_ship_pose.setRollPitchYawRad(d[3], d[4], d[5]);


    // Attitude source
    sprintf(key, "%s.attitude_source", rootkey);
    char *att_source_str = bot_param_get_str_or_fail(param, key);
    if(!strncmp(att_source_str, "NOVATEL", 4))
        attitude_source = ATT_NOVATEL;
    else if(!strncmp(att_source_str, "EVOLOGICS", 9))
        attitude_source = ATT_EVOLOGICS;
    
    // GPS source
    sprintf(key, "%s.gps_source", rootkey);
    char *gps_source_str = bot_param_get_str_or_fail(param, key);
    if(!strncmp(gps_source_str, "NOVATEL", 7))
        gps_source = GPS_NOVATEL;
    else if(!strncmp(gps_source_str, "GPSD", 4))
        gps_source = GPS_GPSD;
    else
    {
        cerr << "Invalid GPS source: " << gps_source_str << endl;
        return 0;
    }
        
    cout << "GPS source: " << gps_source << endl;
        
    
        
    return 1;    

}

int Evologics_Usbl::parse_ahrs_message(char *buf)
{
    // decode the AHRS message
    if(strstr(buf, "AHRS") != NULL)
    {
        char *tokens[10];
        if(chop_string(buf, tokens) != 5)
            return 0;

        ahrs.utime = (int64_t)(atof(tokens[1]) * 1e6);
        ahrs.roll = atof(tokens[3]) * DTOR;
        ahrs.pitch = atof(tokens[2]) * DTOR;
        ahrs.heading = atof(tokens[4]) * DTOR;
        lcm->publish("AHRS", &ahrs);
        return 1;
    }
    else
        return 0;
}
        
        

int Evologics_Usbl::init()
{
    // open the ports
    
    struct addrinfo hints, *evo_addr;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(ip, inet_port, &hints, &evo_addr);
	evo_fd = socket(evo_addr->ai_family, evo_addr->ai_socktype, evo_addr->ai_protocol);
    if(connect(evo_fd, evo_addr->ai_addr, evo_addr->ai_addrlen) < 0) 
    {
        printf("Could not connect to %s on port %s\n", ip, inet_port);
		return 1;
    }
    
    struct timeval tv;
    tv.tv_sec = 1;  // 1 Secs Timeout 
    tv.tv_usec = 000;  // Not init'ing this can cause strange errors
    setsockopt(evo_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));
    
    // open the AHRS port
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(ip, AHRS_PORT, &hints, &evo_addr);
	ahrs_fd = socket(evo_addr->ai_family, evo_addr->ai_socktype, evo_addr->ai_protocol);
    if(connect(ahrs_fd, evo_addr->ai_addr, evo_addr->ai_addrlen) < 0) 
    {
        printf("Could not connect to %s on port %s\n", ip, AHRS_PORT);
		return 1;
    }
    
    tv.tv_sec = 1;  // 1 Secs Timeout 
    tv.tv_usec = 0000;  // Not init'ing this can cause strange errors
    setsockopt(ahrs_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));
    
    evo = new Evologics(evo_fd, '\n', lcm);

    
    
    // put the USBL in a known state
    //send_evologics_command("ATC\n", NULL, 256, &state);
    evo->send_command("+++ATZ1\n");

    char cmd[64];
    sprintf(cmd, "+++AT!L%d\n", source_level);
    evo->send_command(cmd);
    sprintf(cmd, "+++AT!G%d\n", gain);
    evo->send_command(cmd);

    if(auto_gain)
        evo->send_command("+++AT!LC1\n");

    evo->send_command("+++ATH1\n");
    evo->send_command("+++ATZ1\n");
    
    // now to force the settings that require a listen mode
    evo->send_command("+++ATN\n");      // noise mode
    evo->send_command("+++ATA\n");      // listen state
    
    if(gps_source == GPS_GPSD)
        lcm->subscribeFunction("GPSD_CLIENT", on_gpsd, this);
        
    if((gps_source == GPS_NOVATEL) || (attitude_source == ATT_NOVATEL))
        lcm->subscribeFunction("NOVATEL", on_novatel, this);
    
    lcm->subscribeFunction("HEARTBEAT_1HZ", on_heartbeat, this);
    lcm->subscribeFunction("EVOLOGICS_USBL", on_evo_usbl, this);
    
    int i = 0;
    while(lcm_channels[i] != NULL)
    {
        lcm->subscribeFunction(lcm_channels[i], on_lcm, this);
        i++;
      
    }
    
    ping_counter = 0;
    
    if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84")))
    {
       cerr <<  "Error creating Proj4 transform (WGS84)\n";
       return 0;
    }
    return 1;       
}

int Evologics_Usbl::process()
{
    int lcm_fd = lcm->getFileno();
    fd_set rfds;
    char buf[MAX_BUF_LEN];
    
    while(!loop_exit)
    {
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
                parse_ahrs_message(buf);
            }
                
        }
        else
            cout << "select timeout\n";
        
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
    Evologics_Usbl *usbl = new Evologics_Usbl;
    usbl->load_config(basename((argv[0])));
    usbl->init();
    
    usbl->process();
    
    delete usbl;
    return 1;
}
