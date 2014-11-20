#include "evologics_usbl.hpp"

int loop_exit;
int pipe_broken;

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
        //cout << "nov size " << ev->novatelq.size() << endl;
        
        novatel_t *n = (novatel_t *)malloc(sizeof(novatel_t));
        memcpy(n, nov, sizeof(novatel_t));
        ev->novatelq.push_front(n);

        //for(int i=0; i<ev->novatelq.size(); i++)
        //      cout << ev->novatelq[i]->utime << endl; 
        
        
        if(ev->novatelq.size() > 10)
        {
            free(ev->novatelq.back());
            ev->novatelq.pop_back();
        }
        
    memcpy(&ev->novatel, nov, sizeof(novatel_t));
	}
}

void on_heartbeat(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *hb, Evologics_Usbl* ev) 
{
    ev->ping_targets(); 
}

void on_evo_usbl(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const evologics_usbl_t *evo, Evologics_Usbl* ev) 
{
    ev->calc_position(evo); 
}

void on_evo_control(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const evologics_command_t *ec, Evologics_Usbl* ev) 
{
    if(ec->command == evologics_command_t::SEND_FIXES) 
        ev->send_fixes = ec->d;
    cout << "SF:" << ev->send_fixes << endl;
    
    if(ec->command == evologics_command_t::CLEAR)
        ev->evo->clear_modem();
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

// fix processing thread
// we need this because there is no way to trigger an event for select() on a STL queue
static void *fix_thread(void *u)
{
    Evologics_Usbl *evo = (Evologics_Usbl *)u;
    
    while(!loop_exit)
    {
        if(!evo->fixq.empty())
        {
            evologics_usbl_t *ef = evo->fixq.front();
            evo->calc_position(ef);
            delete ef;
            evo->fixq.pop();
        }
        if(evo->fixq.empty())
            usleep(100e3);  // 100 ms sleep
    }
    return NULL;
}
        
           
Evologics_Usbl::Evologics_Usbl()
{
    lcm = new lcm::LCM();
    pthread_create(&fix_thread_id, NULL, fix_thread, this);
    pthread_detach(fix_thread_id);
}

Evologics_Usbl::~Evologics_Usbl()
{
    pthread_join(fix_thread_id, NULL);
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
int Evologics_Usbl::calc_position(const evologics_usbl_t *ef)
{
    //cout << "Calc position\n";
    // We have everything we need to work out where the target is
    //SMALL::Pose3D target;
    //target.setPosition(xt, yt, zt);
    //target.setRollPitchYawRad(0, 0, 0);
    SMALL::Vector3D target;
    target = ef->y, ef->x, ef->z;
    
    
    SMALL::Pose3D ship;
    ship.setPosition(0, 0, 0);
    double ship_roll;
    double ship_pitch;
    double ship_heading;
    
    int nov_index = 0;
    if(attitude_source == ATT_NOVATEL || gps_source == GPS_NOVATEL)
    {
        // find the closest novatel message in the queue
        int time_diff;
        for(unsigned int i=0; i<novatelq.size(); i++)
        {
            time_diff = ef->utime - novatelq[i]->utime;
            if(time_diff > 0)
            {
                nov_index = i;
                break;
            }
            nov_index = i;
        }
    }
    
    printf("ET: %ld, NT: %ld, %d\n", ef->utime, novatelq[nov_index]->utime, nov_index);  
    
    if(attitude_source == ATT_NOVATEL)
    {
            
        ship_roll = novatelq[nov_index]->roll;
        ship_pitch = novatelq[nov_index]->pitch;
        ship_heading = novatelq[nov_index]->heading;
/*
        ship_roll = novatel.roll;
        ship_pitch = novatel.pitch;
        ship_heading = novatel.heading;
*/

    }
    else if (attitude_source == ATT_EVOLOGICS)
    {
        ship_roll = ahrs.roll;
        ship_pitch = ahrs.pitch;
        ship_heading = ahrs.heading;
    }
    ship.setRollPitchYawRad(ship_roll, ship_pitch, ship_heading);
    
    
    cout << "ship" << ship.getPosition() <<  ship.getAxisAngle() * RTOD << endl;
    //cout << "target" << target.getPosition() <<  target.getAxisAngle()  * RTOD << endl;
    cout << "target" << target << endl;
    
    
    SMALL::Vector3D target_ship = usbl_ins_pose.transformFrom(target);
    cout << "target_ship" << target_ship << endl;
    
    
    SMALL::Vector3D target_world = ship.transformFrom(target_ship);
    cout << "target_world" << target_world << endl;
    
    
    
    
    //SMALL::Pose3D sensor2world = ship.compose(usbl_ins_pose);
    //cout << "U2W" << sensor2world.getPosition() <<  sensor2world.getAxisAngle()  * RTOD << endl;
    
    //SMALL::Pose3D repro_target = sensor2world.compose(target);
    
    //SMALL::Vector3D repro_target = sensor2world.transformFrom(target);
    
    //cout << "T2W" << repro_target.getPosition() <<  repro_target.getAxisAngle()   * RTOD << endl;
    //cout << "target" << repro_target << endl;
    
    // set up the coordinate reprojection
    char proj_str[64];
    double ship_latitude;
    double ship_longitude;
    if(gps_source == GPS_NOVATEL)
    {
    
        ship_latitude = novatelq[nov_index]->latitude * RTOD;
        ship_longitude = novatelq[nov_index]->longitude * RTOD;
    /*
        ship_latitude = novatel.latitude * RTOD;
        ship_longitude = novatel.longitude * RTOD;
    */
    } 
    else if(gps_source == GPS_GPSD)
    {
        ship_latitude = gpsd.fix.latitude;
        ship_longitude = gpsd.fix.longitude;
    } 
     
    sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", ship_longitude, ship_latitude);
    
    cout << proj_str << endl;
    
    projPJ pj_tmerc;
    if (!(pj_tmerc = pj_init_plus(proj_str)))
    {
       cerr << "Error creating Proj4 transform\n";
       return 0;
    }
    
    double x = target_world[1]; 
    double y = target_world[0];           
    pj_transform(pj_tmerc, pj_latlong, 1, 1, &x, &y, NULL);
    
    usbl_fix_t uf;
    uf.utime = timestamp_now();
    uf.remote_id = ef->remote_id;
    uf.latitude = y;
    uf.longitude = x;
    uf.depth = target_world[2];
    uf.accuracy = ef->accuracy;
    uf.ship_longitude = ship_longitude * DTOR;
    uf.ship_latitude = ship_latitude * DTOR;
    uf.ship_roll = (float)ship_roll;
    uf.ship_pitch = (float)ship_pitch;
    uf.ship_heading = (float)ship_heading;
    
    
    // given the SD of the novatels position in degrees, convert to meters, calculate the DRMS and add it to the Evologics accuracy
    projUV sd;

    sd.u = novatelq[nov_index]->latitude_sd * RTOD;
    sd.v = novatelq[nov_index]->longitude_sd * RTOD;
/*.
    sd.u = novatel.latitude_sd * RTOD;
    sd.v = novatel.longitude_sd * RTOD;
*/
printf("nov sd: %f, %f\n", sd.u, sd.v);

    sd = pj_inv(sd, pj_tmerc);
    
    double nov_drms = sqrt(sd.u * sd.u + sd.v * sd.v);
    printf("Nov error: %f, %f, DRMS: %f\n", sd.u, sd.u, nov_drms);
    
    uf.accuracy += nov_drms;
    
    lcm->publish("USBL_FIX", &uf);
    
    
    printf("USBL FIX: target: %d Lat: %3.5f Lon: %3.5f Depth %3.1f Accuracy %2.2f\n", ef->remote_id, uf.latitude * RTOD, uf.longitude *RTOD, uf.depth, uf.accuracy);
  
/*  
    libplankton::Local_WGS84_TM_Projection *map_projection = new libplankton::Local_WGS84_TM_Projection(-33.869475, 151.182582);
    double ship_n, ship_e;
    map_projection->calc_map_coords(ship_latitude, ship_longitude, ship_n, ship_e);
    SMALL::Pose3D ship_map;
    ship_map.setPosition(ship_n, ship_e, 0);
    ship_map.setRollPitchYawRad(ship_roll, ship_pitch, ship_heading);
    SMALL::Vector3D target_map = ship_map.transformFrom(target_ship);
    cout << "target_map" << target_map << endl;
*/    
    
    // Get the target index
    int target_index;
    for(target_index=0; target_index<num_targets; target_index++)
        if(targets[target_index] == ef->remote_id)
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
        if(send_fixes)
        {
            cout << "Sending fix of acoustic modem" << endl;
            evo->send_lcm_data(d, d_size, ef->remote_id, target_channel);
        }
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

        ahrs.mtime = (int64_t)(atof(tokens[1]) * 1e6);
        ahrs.utime = timestamp_now();
        ahrs.roll = atof(tokens[3]) * DTOR;
        ahrs.pitch = atof(tokens[2]) * DTOR;
        ahrs.heading = atof(tokens[4]) * DTOR;
        lcm->publish("AHRS", &ahrs);
        return 1;
    }
    else
        return 0;
}
        

int Evologics_Usbl::open_port(const char *port)
{
    // In a seperate function so we can reconnect if the pipe breaks    
    
    int fd;
    
    struct addrinfo hints, *evo_addr;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(ip, port, &hints, &evo_addr);
	fd = socket(evo_addr->ai_family, evo_addr->ai_socktype, evo_addr->ai_protocol);
    if(connect(fd, evo_addr->ai_addr, evo_addr->ai_addrlen) < 0) 
    {
        printf("Could not connect to %s on port %s\n", ip, inet_port);
		return -1;
    }
    
    struct timeval tv;
    tv.tv_sec = 1;  // 1 Secs Timeout 
    tv.tv_usec = 000;  // Not init'ing this can cause strange errors
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    return fd;
}

    

int Evologics_Usbl::init()
{
    // open the ports
    /*
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
    */
    
    if((evo_fd = open_port(inet_port)) == -1)
        return 0;
    if((ahrs_fd = open_port(AHRS_PORT)) == -1)
        return 0;
    
    evo = new Evologics(evo_fd, '\n', lcm, &fixq);

    
    
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
    lcm->subscribeFunction("EVOLOGICS_CONTROL", on_evo_control, this);
    
    int i = 0;
    while(lcm_channels[i] != NULL)
    {
        lcm->subscribeFunction(lcm_channels[i], on_lcm, this);
        i++;
      
    }
    
    ping_counter = 0;
    send_fixes = true;
    
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
        // check the port status, broekn pipes
        if(pipe_broken)
            open_port(inet_port);
        
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
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);
    Evologics_Usbl *usbl = new Evologics_Usbl;
    usbl->load_config(basename((argv[0])));
    usbl->init();
    
    usbl->process();
    
    delete usbl;
    return 1;
}
