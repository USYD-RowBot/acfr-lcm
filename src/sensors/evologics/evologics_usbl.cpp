#include "evologics_usbl.hpp"

int loop_exit;
int pipe_broken;

void on_ship_status(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const ship_status_t *nov, Evologics_Usbl* ev) 
{
    //cout << "nov size " << ev->ship_statusq.size() << endl;
        
    ship_status_t *n = (ship_status_t *)malloc(sizeof(ship_status_t));
    memcpy(n, nov, sizeof(ship_status_t));
    ev->ship_statusq.push_front(n);        
    
    if(ev->ship_statusq.size() > 40)
    {
        free(ev->ship_statusq.back());
        ev->ship_statusq.pop_back();
    }
        
    memcpy(&ev->ship_status, nov, sizeof(ship_status_t));
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

void on_usblfix(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const evologics_usbl_t *ef, Evologics_Usbl* ev) 
{
    ev->process_usblfix(channel, ef);
}
        
           
Evologics_Usbl::Evologics_Usbl()
{
    lcm = new lcm::LCM();
}

Evologics_Usbl::~Evologics_Usbl()
{
}


// The Evologics reference frame is Y forward, X right, Z down
int Evologics_Usbl::process_usblfix(const std::string& channel, const evologics_usbl_t *ef)
{
    // convert to vehicle local coordinate frame (x fwd, y stdb, z down)
    SMALL::Vector3D target;
    target = ef->y, ef->x, -ef->z;
    
    cout << "Evologics_Usbl got local fix: x:" << ef->x << " y: " << ef->y << " z: " << ef->z << " n:" << ef->n << " e:" << ef->e << " r:" << ef->r << " p:" << ef->p << " h:" << ef->h << endl;
    
    SMALL::Pose3D ship;
    ship.setPosition(0, 0, 0);
    double ship_roll;
    double ship_pitch;
    double ship_heading;
    
    int ship_index = 0;
    SMALL::Vector3D target_world;

    if (ship_statusq.size() < 1)
    {
        cout << "WARNING: Evologics_Usbl expecting ship_status data.  Not found." << endl;
        return 0;
    }
    // find the closest ship_status message in the queue
    int time_diff;
    for(unsigned int i=0; i<ship_statusq.size(); i++)
    {
        time_diff = ef->utime - ship_statusq[i]->utime;
        if(time_diff > 0)
        {
            ship_index = i;
            break;
        }
        ship_index = i;
    }
    cout << "ship_index:" << ship_index << endl;
    printf("ET: %ld, NT: %ld, %d\n", ef->utime, ship_statusq[ship_index]->utime, ship_index);  
        
    ship_roll = ship_statusq[ship_index]->roll;
    ship_pitch = ship_statusq[ship_index]->pitch;
    ship_heading = ship_statusq[ship_index]->heading;
    ship.setRollPitchYawRad(ship_roll, ship_pitch, ship_heading);
    
    
        cout << "ship attitude" <<  ship.getAxisAngle() * RTOD << endl;
        cout << "target xyz (sensor)" << target << endl;
        
    
        SMALL::Vector3D target_ship = usbl_ins_pose.transformFrom(target);
        cout << "target xyz (ship)" << target_ship << endl;
    
    
        target_world = ship.transformFrom(target_ship);
        cout << "target xyz (world)" << target_world << endl;
    
        cout << "target_world[0]: " << target_world[0] << " [1]: " << target_world[1] << endl;
    // set up the coordinate reprojection
    char proj_str[64];
    double ship_latitude;
    double ship_longitude;
    ship_latitude = ship_statusq[ship_index]->latitude * RTOD;
    ship_longitude = ship_statusq[ship_index]->longitude * RTOD;
     
    sprintf(proj_str, "+proj=tmerc +lon_0=%f +lat_0=%f +units=m", ship_longitude, ship_latitude);
        
    projPJ pj_tmerc;
    if (!(pj_tmerc = pj_init_plus(proj_str)))
    {
       cerr << "Error creating Proj4 transform\n";
       return 0;
    }
    
    double x = (double)target_world[1]; 
    double y = (double)target_world[0];           
    pj_transform(pj_tmerc, pj_latlong, 1, 1, &x, &y, NULL);

cout << "Ship lat: " << ship_latitude << " lon:" << ship_longitude << endl;
cout << "Target lat: " << y << " lon:" << x << endl;

    usbl_fix_t uf;
    uf.utime = timestamp_now();
    uf.remote_id = ef->remote_id;
    uf.latitude = y ;
    uf.longitude = x ;
    uf.depth = target_world[2];
    uf.accuracy = ef->accuracy;
    uf.ship_longitude = ship_longitude * DTOR;
    uf.ship_latitude = ship_latitude * DTOR;
    uf.ship_roll = (float)ship_roll;
    uf.ship_pitch = (float)ship_pitch;
    uf.ship_heading = (float)ship_heading;
    uf.target_x = ef->x;
    uf.target_y = ef->y;
    uf.target_z = ef->z;
    
    
    // Novatel errors are in meters so we need to convert the standard deviations to a DRMS error so it can be
    // added to the Evologics error.
    /*double ship_drms = sqrt(ship_statusq[ship_index]->latitude_sd * ship_statusq[ship_index]->latitude_sd + ship_statusq[ship_index]->longitude_sd * ship_statusq[ship_index]->longitude_sd);
    printf("Nov error: %f, %f, DRMS: %f\n", ship_statusq[ship_index]->latitude_sd, ship_statusq[ship_index]->longitude_sd, ship_drms);
    
    uf.accuracy += ship_drms;
    */

    // Add uncertainty proporitional to the reported range. Assume a 1 degree
    // accuracy in the fix. FIXME: do a better job of estimating uncertainty.
    double range = sqrt(ef->x*ef->x + ef->y*ef->y + ef->z*ef->z);  
    uf.accuracy += 2*range*sin(0.5*DTOR);
    
    // extract the platform id from the received channel name
    int channel_pos = channel.find_last_of('.');
    std::string target_name = channel.substr(channel_pos + 1);

    char usbl_fix_channel_name[64]; 
    sprintf(usbl_fix_channel_name, "USBL_FIX.%s", target_name.c_str());
    lcm->publish(usbl_fix_channel_name, &uf);
    
    printf("%s: target: %d Lat: %3.5f Lon: %3.5f Depth %3.1f Accuracy %2.2f\n", usbl_fix_channel_name, ef->remote_id, uf.latitude * RTOD, uf.longitude *RTOD, uf.depth, uf.accuracy);
      
/*
    int d_size = uf.getEncodedSize();
    unsigned char *d = (unsigned char *)malloc(d_size);
    uf.encode(d, 0, uf.getEncodedSize());
    if(send_fixes)
    {
        cout << "Sending fix of acoustic modem with size " << d_size << " and data " << d << endl;
        evo->send_lcm_data(d, d_size, ef->remote_id, usbl_fix_channel_name);
    }
    free(d);
    
    //usbl_send[target_index] = 0;
*/
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

    double d[6];
    sprintf(key, "%s.usbl_ins", rootkey);
    bot_param_get_double_array_or_fail(param, key, d, 6);
    usbl_ins_pose.setPosition(d[0], d[1], d[2]);
    usbl_ins_pose.setRollPitchYawRad(d[3], d[4], d[5]);

    sprintf(key, "%s.ins_ship", rootkey);
    bot_param_get_double_array_or_fail(param, key, d, 6);
    ins_ship_pose.setPosition(d[0], d[1], d[2]);
    ins_ship_pose.setRollPitchYawRad(d[3], d[4], d[5]);


    // ship status source
    sprintf(key, "%s.ship_status_channel", rootkey);
    ship_status_channel_str = bot_param_get_str_or_fail(param, key);
        
    return 1;    

}

int Evologics_Usbl::init()
{
    usleep(1e6);
    
    lcm->subscribeFunction(ship_status_channel_str, on_ship_status, this);

    lcm->subscribeFunction("EVO_USBL.*", on_usblfix, this);
    
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
    
    return 1;
}

// Main entry point
int main(int argc, char **argv)
{
    // install the exit handler
    loop_exit = 0;
    Evologics_Usbl *usbl = new Evologics_Usbl;
    usbl->load_config(basename((argv[0])));
    usbl->init();
    
    usbl->process();
    
    delete usbl;
    return 1;
}
