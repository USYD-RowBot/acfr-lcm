#include "lqusbl.hpp"

// Handle a GPS message, this will only work for Version 3 of the GPSD software
// which everything should run by now
void on_gpsd(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const gpsd3_t *gps, Lq_Usbl* ev) 
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

void on_novatel(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const novatel_t *nov, Lq_Usbl* ev) 
{
	// Accept GPS only under the following conditions:
	// nov->status is INS_SOLUTION_GOOD 


    if(nov->status == "INS_SOLUTION_GOOD")
    {
        memcpy(&ev->novatel, nov, sizeof(novatel_t));
	}
}

void on_rt3202(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const rt3202_t *rt, Lq_Usbl* ev) 
{
	// Accept GPS only under the following conditions:
	// nov->status is INS_SOLUTION_GOOD 


    //if(nov->status == "INS_SOLUTION_GOOD")
    {
        memcpy(&ev->rt3202, rt, sizeof(rt3202_t));
	}
}



int readline(int fd, char *buf, int max_len)
{
    int i=0;
    do
    {
        if(recv(fd, &buf[i++], 1, 0) == -1)
            break;
    } while((buf[i-1] != 0x0A));
    return i;
}

int loop_exit;

Lq_Usbl::Lq_Usbl()
{
    lcm = new lcm::LCM();    
}


// open the named pipe created by vmplayer, code lifted from seabed_gui
int Lq_Usbl::open_lq_socket(char *name)
{
	struct sockaddr_un sa;
  
	if ((strlen(name)+1) > sizeof(sa.sun_path))
	{
		cerr << "Path name is too long for a Unix socket." << endl;
		return 0;
	}
	sa.sun_family = AF_UNIX;
  	strcpy(sa.sun_path, name);
  
  	if ((lq_fd = socket(PF_UNIX, SOCK_STREAM, 0)) < 3) 
 	{
		perror("Couldn't open socket");
    	return 0;
  	}
  	if (connect(lq_fd, (struct sockaddr *)&sa, sizeof(sa)) != 0)
  	{
		perror("Couldn't connect socket");
    	return 0;
  	}


	// instead of setting the socket to non blocking set it to a recv timeout of 1 second
	// this may not work  
	struct timeval tv;
	tv.tv_sec = 1;  // 1 Secs Timeout 
    tv.tv_usec = 0000;  // Not init'ing this can cause strange errors
    setsockopt(lq_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

  	return 1;
}


int Lq_Usbl::calc_position(double x, double y, double z, int id, int64_t timestamp)
{
    //cout << "Calc position\n";
    // We have everything we need to work out where the target is
    SMALL::Pose3D target;
    target.setPosition(y, x, z);
    target.setRollPitchYawRad(0, 0, 0);
    
    cout << "Before transform:  " << target << endl;
    
    SMALL::Pose3D ship;
    ship.setPosition(0, 0, 0);
    
    if(attitude_source == ATT_NOVATEL)
        ship.setRollPitchYawRad(novatel.roll * DTOR, novatel.pitch * DTOR, novatel.heading * DTOR);  
    else if (attitude_source == ATT_RT3202)
        ship.setRollPitchYawRad(rt3202.r, rt3202.p, rt3202.h);
    
    SMALL::Pose3D sensor2world = ship.compose(usbl_ins_pose);
    SMALL::Pose3D repro_target = sensor2world.compose(target);
    
    cout << "After transform:  " << repro_target << endl;
    
    // set up the coordinate reprojection
    char proj_str[64];
    if(gps_source == GPS_NOVATEL)
        sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", novatel.longitude * RTOD, novatel.latitude * RTOD);
    else if(gps_source == GPS_GPSD)
        sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", gpsd.fix.longitude, gpsd.fix.latitude);
    else if(gps_source == GPS_RT3202)
        sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", rt3202.lon * RTOD, rt3202.lat * RTOD);
    
    
    
    //sprintf(proj_str, "+proj=tmerc +lon_0=151.0 +lat_0=-33.5 +units=m");
    
    projPJ pj_tmerc;
    if (!(pj_tmerc = pj_init_plus(proj_str)))
    {
       cerr << "Error creating Proj4 transform\n";
       return 0;
    }
        
    double target_x = repro_target.getX();
    double target_y = repro_target.getY();
    double target_z = 0;
    
    printf("Target before projection: %f, %f, %f\n", target_x, target_y, target_z);
              
    pj_transform(pj_tmerc, pj_latlong, 1, 1, &target_x, &target_y, NULL);
    
    printf("target ll: %f, %f\n", target_x * RTOD, target_y * RTOD);
    
    usbl_fix_t uf;
    uf.utime = timestamp;
    uf.latitude = target_y;
    uf.longitude = target_x;
    uf.depth = repro_target.getZ();
//    uf.accuracy = usbl->accuracy;
    uf.remote_id = id;
    lcm->publish("USBL_FIX", &uf);
    
    return 1;
}


vector<string> tokenize(const string &str, const string &delimeters)
{
  vector<string> tokens;
  // skip delimeters at beginning
  string::size_type lastPos = str.find_first_not_of(delimeters, 0);
  // find first "non-delimeter"
  string::size_type pos = str.find_first_of(delimeters, lastPos);
  while (string::npos != pos || string::npos != lastPos)
  {
    // found a token add it to the vector
    tokens.push_back(str.substr(lastPos, pos - lastPos));
    // skip delimeters.  Note the "not_of"
    lastPos = str.find_first_not_of(delimeters, pos);
    // find next "non-delimeter"
    pos = str.find_first_of(delimeters, lastPos);
  }
  return tokens;
}


// parse a LinkQuest TP2 message from Tracklink then calculate the position of the 
// target and send the usblfix_t message to the tracker also publish the lq lcm
// message for the processing chain
int Lq_Usbl::parse_tp2(char *d, int64_t timestamp)
{
    vector<string> tokens;

    tokens = tokenize( d, " \r\n" );

    if ( tokens.size() != 9 )
    { 
        cerr << "Error parsing usblfix message " << d << " got " << tokens.size() << " tokens" << endl;
        return 0;
    }

    cout << d << endl;
    // this is the TP2 message format sent from the Tracklink software
        
    int target_num = atoi(tokens[0].c_str());
//    string time = tokens[1];
//    double bearing = atof(tokens[3].c_str());
//    double slant_range = atof(tokens[4].c_str());
  
    double x = atof(tokens[6].c_str());
    double y = atof(tokens[5].c_str());
    double z = atof(tokens[7].c_str());
    cout << "Got position : " << x << "," << y << "," << z << endl;
    return calc_position(x, y, z, target_num, timestamp);  
}



int Lq_Usbl::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm->getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
        
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "sensors.%s", program_name);

    sprintf(key, "%s.pipe", rootkey);
    pipename = bot_param_get_str_or_fail(param, key);

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
    else if(!strcmp(att_source_str, "RT3202"))
        attitude_source = ATT_RT3202;
    
    // GPS sourcey
    sprintf(key, "%s.gps_source", rootkey);
    char *gps_source_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(gps_source_str, "NOVATEL"))
        gps_source = GPS_NOVATEL;
    else if(!strcmp(gps_source_str, "GPSD"))
        gps_source = GPS_GPSD;
    else if(!strcmp(gps_source_str, "RT3202"))
        gps_source = GPS_RT3202;
        
    return 1;    

}

int Lq_Usbl::process()
{
    int lcm_fd = lcm->getFileno();
    fd_set rfds;
    char buf[MAX_BUF_LEN];
    int64_t timestamp;
    
    while(!loop_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (lcm_fd, &rfds);
        FD_SET (lq_fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        memset(buf, 0, MAX_BUF_LEN);
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm->handle();
            
            if(FD_ISSET(lq_fd, &rfds))
            {
                // data to be read       
                memset(buf, 0, MAX_BUF_LEN);
                readline(lq_fd, buf, MAX_BUF_LEN);
                timestamp = timestamp_now();
                cout << "Data in: " << buf;
                // parsing the meaasge will also set all the channel control flags
                parse_tp2(buf, timestamp);
            }
                            
        }
        else
            cout << "select timeout\n";
        
    }
    
    return 1;
}
        
int Lq_Usbl::init()
{
    if(gps_source == GPS_GPSD)
        lcm->subscribeFunction("GPSD_CLIENT", on_gpsd, this);
        
    if((gps_source == GPS_NOVATEL) || (attitude_source == ATT_NOVATEL))
        lcm->subscribeFunction("NOVATEL", on_novatel, this);
    
    if((gps_source == GPS_RT3202) || (attitude_source == ATT_RT3202))
        lcm->subscribeFunction("RT3202", on_rt3202, this);
    
     if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84")))
    {
       cerr <<  "Error creating Proj4 transform (WGS84)\n";
       return 0;
    }
        
    // open the named pipe for the usbl
    return open_lq_socket(pipename);
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
    Lq_Usbl *usbl = new Lq_Usbl;
    usbl->load_config(basename((argv[0])));
    usbl->init();
    
    
    usbl->process();
    
    delete usbl;
    return 1;
}

