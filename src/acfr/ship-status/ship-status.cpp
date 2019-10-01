#include "ship-status.hpp"


// LCM callbacks
void on_gps(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const gpsd3_t *gps, Ship_Status* ss) 
{
    memcpy(&ss->gpsd3, gps, sizeof(gpsd3_t));
}

void on_novatel(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const novatel_t *nov, Ship_Status* ss) 
{
    memcpy(&ss->novatel, nov, sizeof(novatel_t));
}

void on_ahrs(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const ahrs_t *ahrs, Ship_Status* ss) 
{
    memcpy(&ss->ahrs, ahrs, sizeof(ahrs_t));
}

void on_rt3202(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const rt3202_t *rt, Ship_Status* ss) 
{
    memcpy(&ss->rt3202, rt, sizeof(rt3202_t));
}

void on_posmv(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const posmv_t *pmv, Ship_Status* ss) 
{
    memcpy(&ss->posmv, pmv, sizeof(posmv_t));
}

void on_heartbeat(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *hb, Ship_Status* ss) 
{
    ss->send_status();
}

Ship_Status::Ship_Status(char *rootkey)
{
    // Clear the sensors to start with
    memset(&ahrs, 0, sizeof(ahrs_t));
    memset(&novatel, 0, sizeof(novatel_t));
    memset(&gpsd3, 0, sizeof(gpsd3_t));
    memset(&rt3202, 0, sizeof(rt3202_t));
    
    lcm = new lcm::LCM();
    
    // read the config file to see what we are listening for
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm->getUnderlyingLCM(), 1);
    if(param == NULL)
        return;
        
    char key[128];

    // Attitude source
    sprintf(key, "%s.attitude_source", rootkey);
    char *att_source_str = bot_param_get_str_or_fail(param, key);
    if(!strcasecmp(att_source_str, "novatel"))
        att_source = ATT_NOVATEL;
    else if(!strcasecmp(att_source_str, "rt3202"))
        att_source = ATT_RT3202;
    else if(!strcasecmp(att_source_str, "ahrs"))
        att_source = ATT_AHRS;
    else if(!strcasecmp(att_source_str, "posmv"))
        att_source = ATT_POSMV;

    // GPS source
    sprintf(key, "%s.gps_source", rootkey);
    char *gps_source_str = bot_param_get_str_or_fail(param, key);
    if(!strcasecmp(gps_source_str, "novatel"))
        gps_source = GPS_NOVATEL;
    else if(!strcasecmp(gps_source_str, "rt3202"))
        gps_source = GPS_RT3202;
    else if(!strcasecmp(gps_source_str, "gpsd"))
        gps_source = GPS_GPSD;
	else if(!strcasecmp(gps_source_str, "static"))
        gps_source = GPS_STATIC;
	else if(!strcasecmp(gps_source_str, "posmv"))
        gps_source = GPS_POSMV;

        
	// depending on the attitude source we need a mag declination
	if(att_source == ATT_AHRS)
	{
		sprintf(key, "%s.mag_dec", rootkey);
		mag_dec = bot_param_get_double_or_fail(param, key) * DTOR;
	}
	else
		mag_dec = 0.0;

	if(gps_source == GPS_STATIC)
	{
		sprintf(key, "%s.static_longitude", rootkey);
		static_lon = bot_param_get_double_or_fail(param, key);
		sprintf(key, "%s.static_latitude", rootkey);
		static_lat = bot_param_get_double_or_fail(param, key);
	}



    // ID
    sprintf(key, "%s.ship_name", rootkey);
    ship_name = bot_param_get_str_or_fail(param, key);
    
    sprintf(key, "%s.ship_id", rootkey);
    ship_id = bot_param_get_int_or_fail(param, key);


    char channel_name[128];


    // subscribe to the LCM channels
    if(gps_source == GPS_NOVATEL || att_source == ATT_NOVATEL)
    {
        snprintf(channel_name, 128, "%s.NOVATEL", ship_name);
        lcm->subscribeFunction(channel_name, on_novatel, this);
    }
        
    if(gps_source == GPS_RT3202 || att_source == ATT_RT3202)
    {
        snprintf(channel_name, 128, "%s.RT3202", ship_name);
        lcm->subscribeFunction(channel_name, on_rt3202, this);
    }
        
    if(gps_source == GPS_POSMV || att_source == ATT_POSMV)
    {
        snprintf(channel_name, 128, "%s.POSMV", ship_name);
        lcm->subscribeFunction(channel_name, on_posmv, this);
    }
   
    if(gps_source == GPS_GPSD)
    {
        snprintf(channel_name, 128, "%s.GPSD_CLIENT", ship_name);
        lcm->subscribeFunction(channel_name, on_gps, this);
    }
    
    if(att_source == ATT_AHRS)
    {
        snprintf(channel_name, 128, "%s.AHRS", ship_name);
        lcm->subscribeFunction(channel_name, on_ahrs, this);
    }

    lcm->subscribeFunction("HEARTBEAT_10HZ", on_heartbeat, this);

}

Ship_Status::~Ship_Status()
{
}

int Ship_Status::send_status()
{
    ship_status_t ss;
    ss.utime = timestamp_now();
    ss.ship_id = ship_id;
    ss.name = ship_name;
    
    switch(gps_source)
    {
        case GPS_NOVATEL:
            ss.latitude = novatel.latitude;
            ss.longitude = novatel.longitude;
            break;
        case GPS_RT3202:
            ss.latitude = rt3202.lat;
            ss.longitude = rt3202.lon;
            break;
        case GPS_POSMV:
            ss.latitude = posmv.latitude;
            ss.longitude = posmv.longitude;
            break;
        case GPS_GPSD:
            ss.latitude = gpsd3.fix.latitude;
            ss.longitude = gpsd3.fix.longitude;
            break;
        case GPS_STATIC:
            ss.latitude = static_lat * DTOR;
            ss.longitude = static_lon * DTOR;
            break;
    }
    
    switch(att_source)
    {
        case ATT_NOVATEL:
            ss.roll = novatel.roll;
            ss.pitch = novatel.pitch;
            ss.heading = novatel.heading;
            break;
        case ATT_RT3202:
            ss.roll = rt3202.r;
            ss.pitch = rt3202.p;
            ss.heading = rt3202.h;
            break;
        case ATT_POSMV:
            ss.roll = posmv.roll;
            ss.pitch = posmv.pitch;
            ss.heading = posmv.heading;
            break;

        case ATT_AHRS:
            ss.roll = ahrs.roll;
            ss.pitch = ahrs.pitch;
            ss.heading = ahrs.heading + mag_dec;
            break;
    }
    char channel[64] = {0};
    snprintf(channel, 64, "%s.SHIP_STATUS", ship_name);
    lcm->publish(channel, &ss);

    //cout << "Published ship_status on chan " << channel << " with lat:" << ss.latitude << " lon:" << ss.longitude << " r:" << ss.roll << " p:" << ss.pitch << " h:" << ss.heading << endl; 
    
    return 1;
}
            
int loop_exit;
int Ship_Status::process()
{
    int fd = lcm->getFileno();
    fd_set rfds;
    while(!loop_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select (fd + 1, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
            lcm->handle();
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
    if(argc != 2)
    {
        cerr << "Usage: " << basename(argv[0]) << " [config key]" << endl << "eg.  " << basename(argv[0]) << " ship.bombora" << endl;
        return 0;
    }
    
    // install the exit handler
    loop_exit = 0;
    signal(SIGINT, signal_handler);
    
    
    Ship_Status ship_status(argv[1]);
    ship_status.process();
    
    return 1;
}
    
    
