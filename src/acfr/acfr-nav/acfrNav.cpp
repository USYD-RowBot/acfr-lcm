#include "acfrNav.hpp"

// FIXME: Should be defined somewhere central
#define NO_VALUE ((long)(-98765))

// stubs used for the callbacks
void heartBeatHandler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleHeartBeat(hb);
}

void HMR3600Handler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_honeywell_hmr3600_t *hmr, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleHMR3600(hmr);
}

void OSCompassHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_os_compass_t *osc, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleOSCompass(osc);
}
#ifdef GPS3
void gpsHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_gpsd3_t *gps, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleGps(gps);
}
#else
void gpsHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_gpsd_t *gps, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleGps(gps);
}
#endif // GPS

void motorsHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_os_motors_t *m, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleMotors(m);
}

void altitudeHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_os_altimeter_t *alt, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleAltitude(alt);
}

void imuHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_IMU_t *imu, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleIMU(imu);
}
void msGx1Handler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_ms_gx1_t *ms, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleMsGx1(ms);
}

void parosciHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_parosci_t *parosci, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleParosci(parosci);
}

void lqModemHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_lq_modem_t *lqm, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleLQModem(lqm);
}

void ctHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_seabird_t *ct, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleCT(ct);
}

void seabirdDepthHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_seabird_depth_t *sd, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleSeabirdDepth(sd);
}

void rdiHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_rdi_pd5_t *rdi, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleRDI(rdi);
}

void visHandler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_vis_rawlog_t *vis, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleVis(vis);
}

void ysiHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_ysi_t *ysi, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleYsi(ysi);
}

void oasHandler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_oas_t *oas, void *u) {
    acfrNav *nav = (acfrNav *)u;
    nav->handleOAS(oas);
}

acfrNav::acfrNav() : R_gps(3,3), altitude(0) {
}

acfrNav::~acfrNav() {
	if(!processToRaw)
	    delete slam;
}

int acfrNav::initialise(lcm_t *_lcm, char *slamConfigFileName, char *depthTareFileName,
        char *motorSpeedsFileName, int motorSpeedsNumber, int updateRate, bool _savePoses,
        int _rphSource, bool _lcmPublish, bool _useIMU, bool _useParosci, bool _useYsi, 
        bool _useRdi, bool _useIverPropCount, bool _processToRaw) {

    savePoses = _savePoses;
    rphSource = _rphSource;
    lcmPublish = _lcmPublish;
    useIMU = _useIMU;
    useParosci = _useParosci;
    useYsi = _useYsi;
    useRdi = _useRdi;
    useIverPropCount = _useIverPropCount;
    processToRaw = _processToRaw;
    lcm = _lcm;
	
    // load the motor speeds
    if(useIverPropCount)
        if(!loadMotorSpeeds(motorSpeedsFileName, motorSpeedsNumber))
            return 0;
        
	// load the depth variation
	if(depthTareFileName == NULL)
	    depthTareYsi = 0;
	else {
	    Config_File *depthTareFile;
	    depthTareFile = new Config_File(depthTareFileName);
	    bool have_depth_tare = depthTareFile->get_value( "DEPTH_TARE_YSI"    , depthTareYsi );
	    if (!have_depth_tare){
	        cout << "\nUnable to get depth tare from file [" << depthTareFileName <<"]. Unable to continue.\n\n";
	        return 0;
	    }
    }
    	
	// we don't need to load these options or create the slam object if we are just going to process to RAW
	Config_File *slamConfigFile;
	//Config_File *magVariationFile;
	lowRateCount = 0;
	
	if(!processToRaw) {
		// load the pose augmentation options
		slamConfigFile = new Config_File(slamConfigFileName);
		poseAugOptions  = new Pose_Aug_Options(*slamConfigFile);
		lastPoseAugmentationTime = 0;

		// load the GPS velocity standard deviations
		R_gps.clear();
		bool haveGpsStd = slamConfigFile->get_value( "VEL_GPS_X_STD" ,R_gps(0, 0));
		if (!haveGpsStd){
		    cout << "\nUnable to get GPS velocity STDs from file [" << slamConfigFileName <<"]. Unable to continue.\n\n";
		    return 0;
		}
		slamConfigFile->get_value( "VEL_GPS_Y_STD" , R_gps(1, 1));
		slamConfigFile->get_value( "VEL_GPS_Z_STD" , R_gps(2, 2));

		// create the SLAM object
		slam = new Seabed_Interface(); //(slamConfigFileName, slamMagFileName);
        slam->configure_interface_without_mag_variation(slamConfigFileName);
        
        // get the origin for the magnetic variation
        SLAM_Params slamParams = slam->get_slam_params();
        double origLat = slamParams.origin_latitude;
        double origLng = slamParams.origin_longitude;
        
        time_t time_now = time(NULL);
        struct tm *t = gmtime(&time_now);

        // Use the function from the magnetic variation lib to calulate what we need.
        double field[6];
        int model = 8; // WMM2005
        double mag_var_radians = 
            SGMagVar(origLat * DTOR, origLng * DTOR,  0, // height
                 yymmdd_to_julian_days(t->tm_year - 100, t->tm_mon, t->tm_mday), model, field); 
        // note:- gmttime year is report as years since 1900.  yymmdd... expects 07 for 2007 etc...

        // Finish setting up the interface. Then write the mag_var file for later use.
        slam->set_mag_var_rad(mag_var_radians);

        cout << "Calculated magnetic variation for AUV dive as ";
        cout << fixed << setprecision(2) << mag_var_radians * RTOD << endl << endl;

        // Now write the mag variation to a file for later use.
        char file_name[200];
        snprintf(file_name, sizeof(file_name), "/tmp/magnetic_variation.cfg");
        ofstream MagVarFile(file_name, ios_base::trunc);
        if(!MagVarFile){
            cerr << endl << "ERR:- Unable to open the file " << file_name << " for writing. Aborting." <<endl;
            exit(-1);
        }
        
        MagVarFile << "#" << endl << "# WARNING:- Auto generated file. DO NOT EDIT! " <<endl;
        MagVarFile << "# File is written by vehicle code when it is starting up"<<endl; 
        MagVarFile << "# Magnetic variation for AUV dive. Dive origin and date appear below."<< endl;
        MagVarFile << "# Note:- 6 d.p. for the origin gives position to better than 10cm." << endl << "#" << endl;
         
        MagVarFile.fill('0');
        MagVarFile << "MAG_VAR_DATE \"" << t->tm_year + 1900 << "/"
               << setw(2) << t->tm_mon + 1 << "/" << setw(2) << t->tm_mday << "\"" << endl;  
        MagVarFile.precision(6);
        MagVarFile << "MAG_VAR_LAT " << fixed << showpoint << origLat << endl;
        MagVarFile << "MAG_VAR_LNG " << origLng << endl;
        MagVarFile << "MAGNETIC_VAR_DEG " << setprecision(2) << mag_var_radians * RTOD << endl << endl;

        MagVarFile.close();


		savedPoseId = 0;
	}
	
    // subscriptions required by the nav filter
    senlcm_os_altimeter_t_subscribe(lcm, "OS_ALTIMETER", &altitudeHandler, this);
#ifdef GPS3   
    senlcm_gpsd3_t_subscribe(lcm, "GPSD_CLIENT", &gpsHandler, this);
#else
	senlcm_gpsd_t_subscribe(lcm, "GPSD_CLIENT", &gpsHandler, this);
#endif //GPS3
    if(useIverPropCount)
        senlcm_os_motors_t_subscribe(lcm, "OS_MOTORS", &motorsHandler, this);
    senlcm_os_compass_t_subscribe (lcm, "OS_COMPASS", &OSCompassHandler, this);
    senlcm_lq_modem_t_subscribe(lcm, "LQ_MODEM", &lqModemHandler, this);
    senlcm_seabird_t_subscribe(lcm, "SEABIRD", &ctHandler, this);
    senlcm_seabird_depth_t_subscribe(lcm, "SEABIRD_DEPTH", &seabirdDepthHandler, this);
    acfrlcm_auv_vis_rawlog_t_subscribe(lcm, "ACFR_AUV_VIS_RAWLOG", &visHandler, this);
    senlcm_ms_gx1_t_subscribe(lcm, "3DM", &msGx1Handler, this);
    
    if(useIMU)
	    senlcm_IMU_t_subscribe (lcm, "IMU", &imuHandler, this);
    if(rphSource == rph_hmr3600)
        senlcm_honeywell_hmr3600_t_subscribe (lcm, "HONEYWELL_HMR3600", &HMR3600Handler, this);
    if(useParosci)
	    senlcm_parosci_t_subscribe (lcm, "PAROSCI", &parosciHandler, this);
	if(useYsi)
	    senlcm_ysi_t_subscribe (lcm, "YSI", &ysiHandler, this);
	if(useRdi)
	    senlcm_rdi_pd5_t_subscribe (lcm, "RDI", &rdiHandler, this);

    
	// we don't subscribe to any on the heartbeats if we are just going to create a RAW file
	if(!processToRaw) {
		switch(updateRate) {
			case 1:
				perllcm_heartbeat_t_subscribe(lcm, "HEARTBEAT_1HZ", &heartBeatHandler, this);
				break;
			case 5:
				perllcm_heartbeat_t_subscribe(lcm, "HEARTBEAT_5HZ", &heartBeatHandler, this);
				break;
			case 10:
				perllcm_heartbeat_t_subscribe(lcm, "HEARTBEAT_10HZ", &heartBeatHandler, this);
				break;
			case 100:
				perllcm_heartbeat_t_subscribe(lcm, "HEARTBEAT_100HZ", &heartBeatHandler, this);
				break;
			default:
				perllcm_heartbeat_t_subscribe(lcm, "HEARTBEAT_1HZ", &heartBeatHandler, this);
				cout << "Invalid update rate, defaulting to 1Hz\n";
				break;
		}
	}
    else
    {
    }

    // a bit of cleaning
    if(!processToRaw) {
		delete slamConfigFile;
	}

	return 1;
}

int acfrNav::saveData() {
    // save the data files
    unsigned int numPoses  = slam->slam->get_num_poses();
    unsigned int stateSize = slam->slam->get_state_size();
    stringstream stats;
    stats << "SLAM statistics: " << endl;
    stats << "   Number of augmented poses: " << numPoses << endl;
    stats << "   State vector size        : " << stateSize << endl;

    stringstream fileHeader;
    fileHeader << "Produced by acfrNav\n\n";
    fileHeader << stats.str();

    saveVehiclePoses(fileHeader.str());
    if(savePoses)
        saveVehicleCov(fileHeader.str());
    return 1;
}

void acfrNav::handleHeartBeat(const perllcm_heartbeat_t *heartbeat) {
    if(slam->is_initialised() == true) {
	    libflounder::Vehicle_Estimate estimate = slam->get_vehicle();

		acfrlcm_auv_acfr_nav_t nav;
		//printf("%f, %f, %f, %f, %f\n", estimate.x[SB_VEHICLE_X], estimate.x[SB_VEHICLE_Y], estimate.x[SB_VEHICLE_Z], fmod(estimate.x[SB_VEHICLE_PSI]*180/M_PI,360), estimate.x[SB_VEHICLE_X_VEL]);

		slam->calc_geo_coords(estimate.x[SB_VEHICLE_X], estimate.x[SB_VEHICLE_Y], nav.latitude, nav.longitude);
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
		//nav.altitude = min(altitude, oasAltitude);
		nav.altitude = altitude;
		nav.fwd_obstacle_dist = fwdObstacleDist;
		
		
		printf("%ld\r", (long int)nav.utime);

        if(lcmPublish) {
            acfrlcm_auv_acfr_nav_t_publish(lcm, "ACFR_NAV", &nav);   
        	if(lowRateCount == 9) {
        		lowRateCount = 0;
        		acfrlcm_auv_acfr_nav_t_publish(lcm, "ACFR_NAV.TOP", &nav);
        	}
        	else
        		lowRateCount++;        
        }

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

		
    }
}

void acfrNav::handleHMR3600(const senlcm_honeywell_hmr3600_t *hmr3600) {
    if(rphSource == rph_hmr3600) {
    	if(!useParosci && !useYsi) {
		    // depth data from the OS compass
		    auv_data_tools::Depth_Data depth;
		    depth.depth = osCompass.depth;
		    depth.set_raw_timestamp((double)osCompass.utime/1e6);
		    if(processToRaw) {
			    depth.print(rawOut);
    		    rawOut << endl;
			}
			else
			    slam->handle_depth_data(depth);			
		}
		
        // store the heading for the velocity calculation when on the surface
        vehicleHeading = hmr3600->rph[2]; // + (magneticVariation/180*M_PI);
		if(!useIMU) {
		    auv_data_tools::HMR3600_Data rph;
		    rph.roll = hmr3600->rph[0];
		    rph.pitch = hmr3600->rph[1];
		    rph.heading = hmr3600->rph[2];
		    rph.set_raw_timestamp((double)hmr3600->utime/1e6);
		    if(processToRaw) {
			    rph.print(rawOut);
    		    rawOut << endl;
			}
			else
		    	slam->handle_HMR3600_data(rph);
		}
    }
}

void acfrNav::handleAltitude(const senlcm_os_altimeter_t *alt) {
    // store the altitude for use with pose augmentation
    altitude = alt->metres;
}

#ifdef GPS3
void acfrNav::handleGps(const senlcm_gpsd3_t *gps) {
    // used for velocity when we are on the surface
	gpsSpeed = gps->fix.speed;
	gpsHeading = gps->fix.track;

	auv_data_tools::GPS_Data gpsData;
	gpsData.latitude = gps->fix.latitude * RTOD;
	gpsData.longitude = gps->fix.longitude * RTOD;
	gpsData.speed = gps->fix.speed;
	gpsData.course = gps->fix.track;
	gpsData.mag_var = NAN;
	gpsData.set_raw_timestamp((double)gps->utime/1e6);
	if(gps->status > 0)
		gpsData.warning = "A";
	else
		gpsData.warning = "V";

	// Accept GPS only under the following conditions:
	// 1) Status is good (1=GPS; 2=DGPS)
	// 2) Mode is >= 2 (2D or 3D fix).  
	// 3) Satellites used >= 3. 
	// (2) and (3) are redundant but are generated 
	// asynchronously --- a mode 2 fix may still be reported after
	// the number of satellites has dropped to 0.
	if ( ( gps->status >= 1 )
	     & ( gps->fix.mode >= 2 )
	     & ( gps->satellites_used >= 3 ) )
	  {
		if(processToRaw) {
			gpsData.print(rawOut);
		    rawOut << endl;
		}
		else
		    slam->handle_gps_data(gpsData);
	  }
}
#else
void acfrNav::handleGps(const senlcm_gpsd_t *gps) {
    // used for velocity when we are on the surface
	gpsSpeed = gps->speed;
	gpsHeading = gps->track;

	auv_data_tools::GPS_Data gpsData;
	gpsData.latitude = gps->latitude * RTOD;
	gpsData.longitude = gps->longitude * RTOD;
	gpsData.speed = gps->speed;
	gpsData.course = gps->track;
	gpsData.set_raw_timestamp((double)gps->utime/1e6);
	if(gps->status > 0)
		gpsData.warning = "A";
	else
		gpsData.warning = "V";

	// Ignore GPS unless reported horizontal position accuracy is
	// less than assumed cov.  The eph field is the 95% (2*sigma) 
	// confidence radius (see gpsd man page).
	SLAM_Params slam_params = slam->get_slam_params();
	if ( gps->eph/2.0 < sqrt(slam_params.R_gps(0,0)))
	  {
	    if(processToRaw) {
		    gpsData.print(rawOut);
		    rawOut << endl;
		}
		else
			slam->handle_gps_data(gpsData);
	  }
}
#endif //GPS3

void acfrNav::handleOSCompass(const senlcm_os_compass_t *osc) {
    // first make a copy of the compass data for use if we are using the honeywell compass
    memcpy(&osCompass, osc, sizeof(osCompass));
    if(rphSource == rph_osCompass) {
    	if(!useParosci && !useYsi) {
		    // depth data
		    auv_data_tools::Depth_Data depth;
		    depth.depth = osc->depth;
		    
		    //---@@@TODO: nasty hack to calibrate out depth using first depth reading
		    static double FirstDepth = depth.depth;
		    depth.depth -= FirstDepth;
		    //---

		    depth.set_raw_timestamp((double)osc->utime/1e6);
   		    if(processToRaw) {
			    depth.print(rawOut);
    		    rawOut << endl;
			}
			else
			    slam->handle_depth_data(depth);
		}
		
        // store the heading for the velocity calculation when on the surface
        vehicleHeading = osc->rph[2]; // + (magneticVariation/180*M_PI);
		if(rphSource == rph_osCompass) {

		    auv_data_tools::OS_Compass_Data rph;
//			auv_data_tools::Generic_RPH_Data rph;
		    rph.roll = osc->rph[0];
		    rph.pitch = osc->rph[1];
		    rph.heading = osc->rph[2];
		    rph.set_raw_timestamp((double)osc->utime/1e6);
		    if(processToRaw) {
			    rph.print(rawOut);
    		    rawOut << endl;
			}
			else
			    slam->handle_OS_compass_data(rph);
			    //slam->handle_generic_rph_data(rph);
		}
    }
}

void acfrNav::handleMotors(const senlcm_os_motors_t *m) {
    unsigned int key = (unsigned int)m->motor[4] & 0xFF;
    /*
    // 2011-Feb-20  mvj  Commented out this block.  This needs to be rethought.  GPS speed
    //                   may not be valid (NaN).  On Iver GPS, speed is computed from fixes and not 
    //                   a measurement.
    //
    // check to see if we are on the surface, and if we are then send the GPS velocity instead
    if(osCompass.depth < 0.3) {
        velocity.vx = cos((vehicleHeading) - gpsHeading/180*M_PI) * gpsSpeed;
        velocity.vy = sin((vehicleHeading) - gpsHeading/180*M_PI) * gpsSpeed;
        slam->handle_generic_velocity_data(velocity, R_gps);
    }
    else
    */

    auv_data_tools::Iver_Prop_Data velocity;
    velocity.vx = motorSpeed[key] * 0.51444; // in m/s
    velocity.set_raw_timestamp((double)m->utime/1e6);

	if(processToRaw) {
		velocity.print(rawOut);
	    rawOut << endl;
	}
	else
    	slam->handle_Iver_prop_data(velocity);
}

int acfrNav::loadMotorSpeeds(char *filename, int number) {
    // load the motor speeds from the file and interpolate them over the entire range
    ifstream fs(filename);
	string line;

	if(!fs.is_open()) {
		cout << "Could not open motor speed file: " << filename << endl;
		return 0;
	}
	char numberStr[3];
	sprintf(numberStr, "%d", number);

	int count = 0;
	double speeds[256], motorCount[256];
	while(getline(fs, line))
		if(line[0] == numberStr[0]) {
			int i;
			sscanf(line.c_str(), "%d;%lf;%lf", &i, &speeds[count], &motorCount[count]);
			count++;
		}
	fs.close();

	// sort the motorCount and speeds as the interp function needs them in order
	// get the index vector
	size_t p[256];
	gsl_sort_index(p, motorCount, 1, count);

	// now sort the data
	double newSpeeds[256], newMotorCount[256];
	for(int i=0; i<count; i++) {
		newSpeeds[i] = speeds[p[i]];
		newMotorCount[i] = motorCount[p[i]];
	}

	// assuming there are no duplicates this will work, otherwise GSL throws and error
	gsl_interp *sI = gsl_interp_alloc(gsl_interp_linear, count);
	gsl_interp_init(sI, newMotorCount, newSpeeds, count);
	gsl_interp_accel *accel =  gsl_interp_accel_alloc();
	for(int i=0; i<256; i++)
		motorSpeed[i] = gsl_interp_eval(sI, newMotorCount, newSpeeds, (double)i, accel);

	return 1;
}

void acfrNav::handleIMU(const senlcm_IMU_t *imu) {
	// handle IMU messages
	
	if(useIMU) {
		imuTimeStamp = imu->utime;
		// work out the roll and pitch from the acclerometers
		double roll = atan2(imu->accel[1], sqrt(pow(imu->accel[0], 2) + pow(imu->accel[2], 2)));
		double pitch = atan2(imu->accel[0], sqrt(pow(imu->accel[1], 2) + pow(imu->accel[2], 2)));
	
		// we are not working out the heading from the IMU, use the compass for that
		auv_data_tools::Generic_RPH_Data rph;
		rph.roll = roll;
		rph.pitch = pitch;
		rph.heading = vehicleHeading;
		rph.set_raw_timestamp((double)imu->utime/1e6);
		if(processToRaw) {
			rph.print(rawOut);
	    	rawOut << endl;
		}
		else
			slam->handle_generic_rph_data(rph);
		
		// handle the angular rates
		auv_data_tools::IMU_Data imuData;
		imuData.angXrate_rad_persec = imu->angRate[0];
		imuData.angYrate_rad_persec = imu->angRate[1];
		imuData.angZrate_rad_persec = imu->angRate[2];
		imuData.linXacc_m_persec_sqr = imu->accel[0];
		imuData.linYacc_m_persec_sqr = imu->accel[1];
		imuData.linZacc_m_persec_sqr = imu->accel[2];
		imuData.deltaXang_rad = imu->deltaAngle[0];
		imuData.deltaYang_rad = imu->deltaAngle[1];
		imuData.deltaZang_rad = imu->deltaAngle[2];
		imuData.deltaXvel_m_persec = imu->deltaVelocity[0];
		imuData.deltaYvel_m_persec = imu->deltaVelocity[1];
		imuData.deltaZvel_m_persec = imu->deltaVelocity[2];
		imuData.set_raw_timestamp((double)imu->utime/1e6);
	    if(processToRaw) {
		    imuData.print(rawOut);
   		    rawOut << endl;
		}
		else
			slam->handle_imu_data(imuData);
	}
}


void acfrNav::handleMsGx1(const senlcm_ms_gx1_t *ms) {
	// handle ms gx1 messages
	auv_data_tools::ThreeDM_Data threeDm;	

    // check ms->bitmask
	// static const int ValidMask = (SENLCM_MS_GX1_T_STAB_EULER | SENLCM_MS_GX1_T_INST_ACCEL | SENLCM_MS_GX1_T_STAB_ANGRATE);
	//bool valid = ((ms->bitmask & ValidMask) == ValidMask);
	bool valid=1;
    if(valid) {
	    // rpy
	    threeDm.roll = ms->iEuler[0]*180/M_PI;
	    threeDm.pitch = ms->iEuler[1]*180/M_PI;
	    threeDm.yaw = ms->iEuler[2]*180/M_PI;

        // coverted Accelerations X, Y, Z
	    threeDm.accel_x = ms->iAccel[0];
	    threeDm.accel_y = ms->iAccel[1];
	    threeDm.accel_z = ms->iAccel[2];

        // converted Rates X, Y, Z
	    threeDm.rate_x = ms->iAngRate[0];
	    threeDm.rate_y = ms->iAngRate[1];
	    threeDm.rate_z = ms->iAngRate[2];

        // todo: use the rest of info avail in senlcm_ms_gx1_t ()
		
	    threeDm.set_raw_timestamp((double)ms->utime/1e6);
	    if(processToRaw) {
		    threeDm.print(rawOut);
        	rawOut << endl;
	    }
	    else
		    slam->handle_threedm_data(threeDm);
    }
}



void acfrNav::handleParosci(const senlcm_parosci_t *parosci) {
	// use the parosci depth sensor
	auv_data_tools::Depth_Data depth;
    depth.depth = parosci->depth;
    depth.set_raw_timestamp((double)parosci->utime/1e6);
    if(processToRaw) {
	    depth.print(rawOut);
	    rawOut << endl;
	}
	else
	    slam->handle_depth_data(depth);
}

void acfrNav::handleLQModem(const senlcm_lq_modem_t *lqm) {
	if(lqm->messageType == LQM_FIX) {
		auv_data_tools::USBLFix_Data usblfixData;
		usblfixData.set_raw_timestamp((double)lqm->utime/1e6);
		usblfixData.ship_timestamp = lqm->time;
		usblfixData.latitude = lqm->lat;
		usblfixData.longitude = lqm->lon;
		usblfixData.heading = lqm->heading;
		usblfixData.roll = lqm->roll;
		usblfixData.pitch = lqm->pitch;
		usblfixData.bearing = lqm->bearing;
		usblfixData.range = lqm->slantRange;
	    if(processToRaw) {
		    usblfixData.print(rawOut);
  		    rawOut << endl;
		}
		else
			slam->handle_usblfix_data(usblfixData);
	}
}

void acfrNav::handleCT(const senlcm_seabird_t *ct) {
	auv_data_tools::Seabird_Data seaBird;
	seaBird.cond = ct->conductivity;
	seaBird.temp = ct->temperature;
	seaBird.sal = ct->salinity;
	seaBird.pres = ct->pressure;
	seaBird.sos = ct->speed_of_sound;
    if(processToRaw) {
	    seaBird.print(rawOut);
	    rawOut << endl;
	}

	else
		slam->handle_salinitytemp_data(seaBird);
}

void acfrNav::handleSeabirdDepth(const senlcm_seabird_depth_t *sd) {
    auv_data_tools::Seabird_Depth_Data depth;
    depth.depth = sd->depth;
    depth.set_raw_timestamp((double)sd->utime/1e6);
    if(processToRaw) {
        depth.print(rawOut);
	    rawOut << endl;
    }
    else
        slam->handle_depth_data(depth);			
}

void acfrNav::handleRDI(const senlcm_rdi_pd5_t *rdi) {
	// handle a message from the RDI DVL, the old code used the PD0 message
	// we are using the PD5 message as the LCM module was already written
	
	if((rdi->pd4.btv[0] > -32.786) && (rdi->pd4.btv[1] > -32.786) && (rdi->pd4.btv[2] > -32.786) && (rdi->pd4.btv[3])) {
	    auv_data_tools::RDI_Data rdiD;
	    rdiD.set_raw_timestamp((double)rdi->utime/1e6);
	    rdiD.alt = rdi->pd4.altitude;
	    rdiD.r1 = rdi->pd4.range[0];
	    rdiD.r2 = rdi->pd4.range[1];
	    rdiD.r3 = rdi->pd4.range[2];
	    rdiD.r4 = rdi->pd4.range[3];
	    rdiD.h = rdi->heading * RTOD;
	    rdiD.p = rdi->pitch * RTOD;
	    rdiD.r = rdi->roll * RTOD;
	    rdiD.vx = rdi->pd4.btv[0];
	    rdiD.vy = rdi->pd4.btv[1];
	    rdiD.vz = rdi->pd4.btv[2];
	    rdiD.COG = 0;   // FIXME
	    rdiD.SOG = 0;   // FIXME
	    rdiD.bt_status = rdi->pd4.btv_status;
	    rdiD.h_true = 0;   // FIXME
	    rdiD.p_gimbal = 0;   // FIXME
	    rdiD.sv = rdi->pd4.speed_of_sound;
	    rdiD.depth_rate = 0;   // FIXME
        
        if(useRdi) {
            // for the moment, filter the altitude. A median filter may be
            // more appropriate
	    double last_altitude = altitude;
            altitude = 0.05*rdi->pd4.altitude + 0.95*last_altitude;
        }

        if(processToRaw) {
		    rdiD.print(rawOut);
   		    rawOut << endl;
		}
		else
			slam->handle_dvl_data(rdiD);
	}	

	// This block has been commented out as the DVL on the Iver does not cointain
	// a RPH sensor.  CL

	// as this is where out compass is the is where we will also submit the
	// RPH data, the R and P come from the IMU
/*	if(useIMU) {
		// check the timestamp to make sure the IMU data is close
		if((rdi->utime - imuTimeStamp) < 10000) {
			// with in 10ms
			auv_data_tools::Generic_RPH_Data rph;
			rph.roll = rdi->roll;
			rph.pitch = rdi->pitch;
			rph.heading = vehicleHeading;
			rph.set_raw_timestamp((double)rdi->utime/1e6);
		    if(processToRaw) {
			    rph.print(rawOut);
    		    rawOut << endl;
			}
			else
				slam->handle_generic_rph_data(rph);			
		}
	}
	else {
		// use the RDI RPH sensor
		auv_data_tools::Generic_RPH_Data rph;
	    rph.roll = rdi->roll;
	    rph.pitch = rdi->pitch;
	    rph.heading = rdi->heading;
	    rph.set_raw_timestamp((double)rdi->utime/1e6);
	    if(processToRaw) {
		    rph.print(rawOut);
		    rawOut << endl;
		}
		else
		    slam->handle_generic_rph_data(rph);
	}
*/
}
	
void acfrNav::handleVis(const acfrlcm_auv_vis_rawlog_t *vis) {
	// the only thing we do here at the moment is process the message for the raw log
	auv_data_tools::Vision_Data visData;

	visData.image_name = string(vis->image_name) + ".tif";
	visData.set_raw_timestamp((double)vis->utime/1e6);
	visData.set_vis_timestamp((double)vis->utime/1e6);
	//fix to shift timestamps on RAW log file for 20110603 cam was 3.61 sec ahead
	//visData.set_vis_timestamp((double)vis->utime/1e6-3.61);
	visData.exposure = vis->exp_time;
	if(processToRaw) {
		visData.print(rawOut);
	    rawOut << endl;
	}
}

void acfrNav::handleYsi(const senlcm_ysi_t *ysi) {
	// use to`he YSI sensor to get depth
/*	auv_data_tools::Depth_Data depth;
	depth.depth = ysi->depth - depthTareYsi;
	depth.set_raw_timestamp((double)ysi->utime/1e6);
	if(processToRaw) {
		depth.print(rawOut);
		rawOut << endl;
	}
	else
		slam->handle_depth_data(depth);
*/		

   	auv_data_tools::YSI_Data ysiD;
	ysiD.temperature = ysi->temperature;
	ysiD.depth = ysi->depth - depthTareYsi;
    ysiD.turbidity = ysi->turbidity;
    ysiD.chlorophyl = ysi->chlorophyl;
    ysiD.conductivity = ysi->conductivity;
    ysiD.oxygen = ysi->oxygen;
    ysiD.battery = ysi->battery;
    ysiD.salinity = ysi->salinity;
		
	ysiD.set_raw_timestamp((double)ysi->utime/1e6);
	if(processToRaw) {
		ysiD.print(rawOut);
		rawOut << endl;
	}
	else
		slam->handle_YSI_data(ysiD);
		
}		

void acfrNav::handleOAS(const senlcm_oas_t *oas) {
    auv_data_tools::OAS_Data oasD;
    
    oasD.prof_range = oas->fProfRange;
    oasD.pseudo_alt = oas->fPseudoAlt;
    oasD.psedo_forward_distance = oas->fPseudoFwdDistance;
    
    
    fwdObstacleDist = oas->fPseudoFwdDistance;
    oasAltitude = oas->fPseudoAlt;

	if(processToRaw) {
		oasD.print(rawOut);
		rawOut << endl;
	}       
}




void acfrNav::saveVehiclePoses(string fileHeader) {
    if(poseAugOptions->use_max_time_option) {
        cout << "Preparing to write out pose cov for " << slam->slam->get_num_poses() << " states" << endl;
        Vehicle_Pose_File data;
        data.origin_latitude = slam->get_slam_params().origin_latitude;
        data.origin_longitude = slam->get_slam_params().origin_longitude;
        data.poses.resize(slam->slam->get_num_poses());

        for(unsigned int i=0; i<slam->slam->get_num_poses(); i++) {
            Seabed_Pose_Data *pd = slam->slam->get_pose_data_by_index(i);

            data.poses[i].pose_time = pd->time;
            data.poses[i].altitude = pd->altitude;
            data.poses[i].pose_id = slam->slam->get_pose_id(i);
            data.poses[i].pose_est = convert_vector(slam->slam->get_pose_est_by_index(i));

            slam->calc_geo_coords( data.poses[i].pose_est[AUV_POSE_INDEX_X],
                data.poses[i].pose_est[AUV_POSE_INDEX_Y],
                data.poses[i].latitude,
                data.poses[i].longitude );;
        }
        write_vehicle_pose_est_file(VEH_POSE_EST_FILE_NAME, fileHeader, data );
    }
}
#define VEH_TRAJ_SLAM_FILE_NAME "veh_traj_slam.data"
void acfrNav::saveVehicleCov(string fileHeader) {
    write_vehicle_pose_cov_file(VEH_TRAJ_SLAM_FILE_NAME, fileHeader, slamPoseCov);
}
