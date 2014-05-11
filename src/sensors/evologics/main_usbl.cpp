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


    if(nov->status == "INS_SOLUTION_GOOD")
    {
        memcpy(&ev->novatel, nov, sizeof(novatel_t));
	}
}

Evologics_Usbl::Evologics_Usbl()
{
    lcm = new lcm::LCM();
    state.lcm = lcm->getUnderlyingLCM();
    state.usbl = (senlcm_evologics_usbl_t *)malloc(sizeof(senlcm_evologics_usbl_t));
}

int Evologics_Usbl::calc_position()
{
    // We have everything we need to work out where the target is
    SMALL::Pose3D target;
    target.setPosition(state.usbl->x, state.usbl->y, state.usbl->z);
    target.setRollPitchYawRad(0, 0, 0);
    
    SMALL::Pose3D ship;
    ship.setPosition(0, 0, 0);
    if(attitude_source == ATT_NOVATEL)
        ship.setRollPitchYawRad(novatel.roll * DTOR, novatel.pitch * DTOR, novatel.heading * DTOR);  
    else if (attitude_source == ATT_EVOLOGICS)
        ship.setRollPitchYawRad(state.usbl->r, state.usbl->p, state.usbl->h);
    
    
    SMALL::Pose3D sensor2world = ship.compose(usbl_ins_pose);
    SMALL::Pose3D repro_target = sensor2world.compose(target);
    
    // set up the coordinate reprojection
    char proj_str[64];
    if(gps_source == GPS_NOVATEL)
        sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", novatel.longitude, novatel.latitude);
    else if(gps_source == GPS_GPSD)
        sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", gpsd.fix.longitude, gpsd.fix.latitude);
    
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
    uf.latitude = y;
    uf.longitude = x;
    uf.depth = repro_target.getZ();
    uf.accuracy = state.usbl->accuracy;
    lcm->publish("USBL_FIX", &uf);
    
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
    state.num_targets = bot_param_get_int_array(param, key, state.targets, 8);
    for(int i=0; i<state.num_targets; i++)
        state.ping_sem[i] = 1;

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
    if(!strcmp(att_source_str, "NOVATEL"))
        attitude_source = ATT_NOVATEL;
    else if(!strcmp(att_source_str, "EVOLOGICS"))
        attitude_source = ATT_EVOLOGICS;
    
    // GPS source
    sprintf(key, "%s.gps_source", rootkey);
    char *gps_source_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(gps_source_str, "NOVATEL"))
        gps_source = GPS_NOVATEL;
    else if(!strcmp(gps_source_str, "GPSD"))
        gps_source = GPS_GPSD;
        
    return 1;    

}

int Evologics_Usbl::init()
{
    // open the ports
    struct addrinfo hints, *evo_addr;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(ip, inet_port, &hints, &evo_addr);
	state.fd = socket(evo_addr->ai_family, evo_addr->ai_socktype, evo_addr->ai_protocol);
    if(connect(state.fd, evo_addr->ai_addr, evo_addr->ai_addrlen) < 0) 
    {
        printf("Could not connect to %s on port %s\n", ip, inet_port);
		return 1;
    }
    
    struct timeval tv;
    tv.tv_sec = 0;  // 1 Secs Timeout 
    tv.tv_usec = 1000;  // Not init'ing this can cause strange errors
    setsockopt(state.fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
    
    // put the USBL in a known state
    send_evologics_command("+++ATZ1\n", NULL, 256, &state);
    
    if(gps_source == GPS_GPSD)
        lcm->subscribeFunction("GPSD_CLIENT", on_gpsd, this);
        
    if((gps_source == GPS_NOVATEL) || (attitude_source == ATT_NOVATEL))
        lcm->subscribeFunction("NOVATEL", on_novatel, this);
    
    if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=wgs84")))
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
    int64_t timestamp;
    int len;
    
    while(!loop_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (lcm_fd, &rfds);
        FD_SET (state.fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select (state.fd + 1, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm->handle();
            else
            {
                // data to be read       
                memset(buf, 0, MAX_BUF_LEN);
                len = readline(state.fd, buf, MAX_BUF_LEN);
                timestamp = timestamp_now();
                
                // parsing the meaasge will also set all the channel control flags
                parse_message_t ret = parse_evologics_message(buf, len, &state, timestamp);
                if(ret == PARSE_USBL)
                    calc_position();
            }
        }
    }
    
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
 
 
 
