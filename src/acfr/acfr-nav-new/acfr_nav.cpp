#include "acfr_nav.hpp"


int loop_exit;
string vehicle_name = "DEFAULT";

acfr_nav::acfr_nav()
{
    state = new(state_c);
    state->mode = NAV;
    state->lcm = new lcm::LCM();
    state->altitude = 0;
    state->oas_altitude = 0;
    state->depth = 0;
    state->bottomLock = false;
    state->broken_iver_alt = false;
}

acfr_nav::~acfr_nav()
{
    delete state->slam;
    delete state;
}

int acfr_nav::initialise()
{

    //Config_File *slam_config_file = new Config_File(slam_config_filename);
	//poseAugOptions  = new Pose_Aug_Options(*slam_config_file);
	//lastPoseAugmentationTime = 0;

    // create the SLAM object
	state->slam = new Seabed_Interface();
    state->slam->configure_interface_without_mag_variation(slam_config_filename);
    calculate_mag();   
    
    if(depth_source == DEPTH_YSI)
        state->slam->set_tare_depth("/home/auv/mission.cfg");    
    
    // subscribe to the relevant LCM channel based on our configuration
    
    // we always subscribe to the GPS
    state->lcm->subscribeFunction(vehicle_name+".GPSD_CLIENT", on_gps, state);
    
    // Are we using the OS_COMPASS, attitude or depth
    if(attitude_source == ATT_OS || DEPTH_OS_COMPASS)
        state->lcm->subscribeFunction(vehicle_name+".OS_COMPASS", on_os_compass, state);
    
    // Are we using the TCM compass
    if(attitude_source == ATT_TCM)
        state->lcm->subscribeFunction(vehicle_name+".TCM", on_tcm_compass, state);
    else if(attitude_source == ATT_UVC)
	state->lcm->subscribeFunction(vehicle_name+".UVC_RPH", on_uvc_rph, state);
        
    // Which depth sensor are we using
    if(depth_source == DEPTH_YSI)
        state->lcm->subscribeFunction(vehicle_name+".YSI", on_ysi, state);
    else if(depth_source == DEPTH_PAROSCI)
        state->lcm->subscribeFunction(vehicle_name+".PAROSCI", on_parosci, state);
    else if(depth_source == DEPTH_SEABIRD)
        state->lcm->subscribeFunction(vehicle_name+".SEABIRD", on_seabird_depth, state);
    
    // We always subscribe to this as our velocity source
    if(velocity_source == VEL_RDI)
	state->lcm->subscribeFunction(vehicle_name+".RDI", on_rdi, state);
    else if(velocity_source == VEL_UVC)
	    state->lcm->subscribeFunction(vehicle_name+".UVC_DVL", on_uvc_dvl, state);

    // Always subscribe to the IMU
    state->lcm->subscribeFunction(vehicle_name+".IMU", on_imu, state);


    // Always subscribe to the DWN oas nimbus
    state->lcm->subscribeFunction(vehicle_name+".MICRON_SOUNDER_DWN", on_dwn, state);
    
    // Subscribe to the Evologics USBL
    if(evologics_channel != NULL)
	    state->lcm->subscribeFunction(evologics_channel, on_evologics, state);
    
    state->lowRateCount = 0;
    
    return 1;
}    

int acfr_nav::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (state->lcm->getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
        
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "nav.%s", program_name);

    sprintf(key, "%s.slam_config", rootkey);
    slam_config_filename = bot_param_get_str_or_fail(param, key);

    // Attitude source
    sprintf(key, "%s.attitude_source", rootkey);
    char *att_source_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(att_source_str, "TCM"))
        attitude_source = ATT_TCM;
    else if(!strcmp(att_source_str, "RDI"))
        attitude_source = ATT_RDI;
    else if(!strcmp(att_source_str, "OS"))
        attitude_source = ATT_OS;
    else if(!strcmp(att_source_str, "UVC"))
	attitude_source = ATT_UVC;

    
    // Depth source
    sprintf(key, "%s.depth_source", rootkey);
    char *depth_source_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(depth_source_str, "YSI"))
        depth_source = DEPTH_YSI;
    else if(!strcmp(depth_source_str, "PAROSCI"))
        depth_source = DEPTH_PAROSCI;
    else if(!strcmp(depth_source_str, "SEABIRD"))
        depth_source = DEPTH_SEABIRD;
     else if(!strcmp(depth_source_str, "UVC"))
        depth_source = DEPTH_OS;   
    else if(!strcmp(depth_source_str, "OS_COMPASS"))
        depth_source = DEPTH_OS_COMPASS;   
    
    // Velocity source
    sprintf(key, "%s.velocity_source", rootkey);
    if(bot_param_has_key(param, key))
    {	    
	    
	char *velocity_source_str;
	bot_param_get_str( param, key, &velocity_source_str);
	 
	if(!strcmp(velocity_source_str, "RDI"))
            velocity_source = VEL_RDI;
        else if(!strcmp(velocity_source_str, "UVC"))
            velocity_source = VEL_UVC;
    }
    else
	    velocity_source = VEL_RDI;

    evologics_channel = NULL;
    sprintf(key, "%s.evologics_channel", rootkey);
    if(bot_param_has_key(param, key))
	    bot_param_get_str( param, key, &evologics_channel);
    
    
    return 1;    

}

void publish_nav(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *heartbeat, state_c* state)
{
    if(state->slam->is_initialised() == true) {
	    libflounder::Vehicle_Estimate estimate = state->slam->get_vehicle();

		acfrlcm::auv_acfr_nav_t nav;
		//printf("%f, %f, %f, %f, %f\n", estimate.x[SB_VEHICLE_X], estimate.x[SB_VEHICLE_Y], estimate.x[SB_VEHICLE_Z], fmod(estimate.x[SB_VEHICLE_PSI]*180/M_PI,360), estimate.x[SB_VEHICLE_X_VEL]);

		state->slam->calc_geo_coords(estimate.x[SB_VEHICLE_X], estimate.x[SB_VEHICLE_Y], nav.latitude, nav.longitude);
		nav.latitude = nav.latitude * M_PI / 180.0;
		nav.longitude = nav.longitude * M_PI / 180.0;
		nav.x = estimate.x[SB_VEHICLE_X];
		nav.y = estimate.x[SB_VEHICLE_Y];
		nav.depth = estimate.x[SB_VEHICLE_Z];
		nav.roll = estimate.x[SB_VEHICLE_PHI];
		nav.pitch = estimate.x[SB_VEHICLE_THETA];
		nav.heading = estimate.x[SB_VEHICLE_PSI];
		nav.vx = estimate.x[SB_VEHICLE_X_VEL];
		nav.vy = estimate.x[SB_VEHICLE_Y_VEL];
		nav.vz = estimate.x[SB_VEHICLE_Z_VEL];
		nav.rollRate = estimate.x[SB_VEHICLE_PHI_RATE];
		nav.pitchRate = estimate.x[SB_VEHICLE_THETA_RATE];
		nav.headingRate = estimate.x[SB_VEHICLE_PSI_RATE];
		nav.utime = (int64_t)(estimate.timestamp*1e6);
		double oas_transformed_alt = state->oas_altitude*cos(nav.pitch) - 1.25*sin(nav.pitch);
		double sea_floor_est = nav.depth + min(state->altitude, oas_transformed_alt);
         if(((((timestamp_now() - state->oas_utime) < 2e6)) && state->oas_altitude > 0.5))
		  nav.altitude = min(state->altitude, oas_transformed_alt);
         else
		  nav.altitude = min(state->altitude, sea_floor_est - nav.depth);
		nav.fwd_obstacle_dist = state->fwd_obs_dist;
		
		
		//printf("%ld\r", (long int)nav.utime);

        state->lcm->publish(vehicle_name+".ACFR_NAV", &nav);   

    	if(state->lowRateCount++ == 9) {
    		state->lowRateCount = 0;
            state->lcm->publish(vehicle_name+".ACFR_NAV_TOP", &nav);   

    	}
/*        }

		if((poseAugOptions->use_max_time_option) &&
			((estimate.timestamp - lastPoseAugmentationTime) >= poseAugOptions->max_time)) {
            Seabed_Pose_Data *pd = new Seabed_Pose_Data(estimate.timestamp, altitude);
			slam->slam->augment_current_pose(pd);
			lastPoseAugmentationTime = estimate.timestamp;
		}

		if(savePoses) {
			Vehicle_Pose_Cov slamData;

			slamData.pose_id = ++savedPoseId;
			slamData.pose_time = estimate.timestamp;
			estimate.x.resize(AUV_NUM_POSE_STATES,true);
			estimate.P.resize(AUV_NUM_POSE_STATES, AUV_NUM_POSE_STATES, true);
			slamData.pose_est.assign( estimate.x );
			slamData.pose_cov.assign( estimate.P );
			slamPoseCov.push_back(slamData);
		}

*/		
    }
}

int acfr_nav::process()
{
    state->lcm->subscribeFunction("HEARTBEAT_10HZ", publish_nav, state);
    int fd = state->lcm->getFileno();
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
            state->lcm->handle();
    }
    
    return 1;
}
    
    
void acfr_nav::calculate_mag()
{
    // get the origin for the magnetic variation
    SLAM_Params slam_params = state->slam->get_slam_params();
    double orig_lat = slam_params.origin_latitude;
    double orig_long = slam_params.origin_longitude;
        
    time_t time_now = time(NULL);
    struct tm *t = gmtime(&time_now);

    // Use the function from the magnetic variation lib to calulate what we need.
    double field[6];
    int model = 8; // WMM2005
    double mag_var_radians = 
        SGMagVar(orig_lat * DTOR, orig_long * DTOR,  0, // height
             yymmdd_to_julian_days(t->tm_year - 100, t->tm_mon, t->tm_mday), model, field); 
        // note:- gmttime year is report as years since 1900.  yymmdd... expects 07 for 2007 etc...

    // Finish setting up the interface. Then write the mag_var file for later use.
    state->slam->set_mag_var_rad(mag_var_radians);

    cout << "Calculated magnetic variation for AUV dive as ";
    cout << fixed << setprecision(2) << mag_var_radians * RTOD << endl << endl;

    // Now write the mag variation to a file for later use.
    char file_name[200];
    snprintf(file_name, sizeof(file_name), "/tmp/magnetic_variation.cfg");
    ofstream mag_var_file(file_name, ios_base::trunc);
    if(!mag_var_file){
        cerr << endl << "ERR:- Unable to open the file " << file_name << " for writing. Aborting." <<endl;
        exit(-1);
    }
    
    mag_var_file << "#" << endl << "# WARNING:- Auto generated file. DO NOT EDIT! " <<endl;
    mag_var_file << "# File is written by vehicle code when it is starting up"<<endl; 
    mag_var_file << "# Magnetic variation for AUV dive. Dive origin and date appear below."<< endl;
    mag_var_file << "# Note:- 6 d.p. for the origin gives position to better than 10cm." << endl << "#" << endl;
     
    mag_var_file.fill('0');
    mag_var_file << "MAG_VAR_DATE \"" << t->tm_year + 1900 << "/"
           << setw(2) << t->tm_mon + 1 << "/" << setw(2) << t->tm_mday << "\"" << endl;  
    mag_var_file.precision(6);
    mag_var_file << "MAG_VAR_LAT " << fixed << showpoint << orig_lat << endl;
    mag_var_file << "MAG_VAR_LNG " << orig_long << endl;
    mag_var_file << "MAGNETIC_VAR_DEG " << setprecision(2) << mag_var_radians * RTOD << endl << endl;

    mag_var_file.close();
}
    



void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv)
{
    int opt;

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            vehicle_name = (char*)optarg;
            break;
         }
    }
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
    parse_args(argc, argv);
    acfr_nav *nav = new acfr_nav;
    nav->load_config(basename((argv[0])));
    nav->initialise();
    
    nav->process();
    
    
    delete nav;
    return 1;
}
    
    
