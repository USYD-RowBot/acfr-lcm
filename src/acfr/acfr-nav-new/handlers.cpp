/*
    ACFR navigation LCM handler routines (C++)
    
    Christian Lees
    ACFR
    16/10/12

    The state class that is passed into the handler functions must contain
    - a mode flag (defined in handlers.hpp)
    - a pointer to the slam object
    - an altitude variable
    - an OAS altitude valiable

*/

#include "handlers.hpp"



// Handle a GPS message, this will only work for Version 3 of the GPSD software
// which everything should run by now
void on_gps(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const gpsd3_t *gps, state_c* state) 
{
    auv_data_tools::GPS_Data gps_data;
    gps_data.latitude = gps->fix.latitude * RTOD;
    gps_data.longitude = gps->fix.longitude * RTOD;
    gps_data.speed = gps->fix.speed;
    gps_data.course = gps->fix.track;
    gps_data.mag_var = NAN;
    gps_data.set_raw_timestamp((double)gps->utime/1e6);
    
    if(gps->status > 0)
	    gps_data.warning = "A";
    else
	    gps_data.warning = "V";

	// Accept GPS only under the following conditions:
	// 1) Status is good (1=GPS; 2=DGPS)
	// 2) Mode is >= 2 (2D or 3D fix).  
	// 3) Satellites used >= 3. 
	// (2) and (3) are redundant but are generated 
	// asynchronously --- a mode 2 fix may still be reported after
	// the number of satellites has dropped to 0.

    if((gps->status >= 1) && (gps->fix.mode >= 2))// && (gps->dop.hdop == gps->dop.hdop)) // & (gps->satellites_used >= 3 ))
    {
        if(state->mode == NAV)
        {
            state->slam->handle_gps_data(gps_data);
        }
        else if(state->mode == RAW)
        {
            gps_data.print(state->raw_out);
	    	state->raw_out << endl;
		}
	}
}

// Handle a Parosci depth sensor message
void on_parosci(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const parosci_t *parosci, state_c* state) 
{
	// use the parosci depth sensor
	auv_data_tools::Depth_Data depth;
    depth.depth = parosci->depth;
    depth.set_raw_timestamp((double)parosci->utime/1e6);
    if(state->mode == NAV)
    	state->slam->handle_depth_data(depth);
    else if(state->mode == RAW)
    {
        depth.print(state->raw_out);
	    state->raw_out << endl;
	}
}


void on_lq_modem(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const lq_modem_t *lqm, state_c* state) 
{
	if(lqm->messageType == lq_modem_t::FIX) {
		auv_data_tools::USBLFix_Data usbl_fix_data;
		usbl_fix_data.set_raw_timestamp((double)lqm->utime/1e6);
		usbl_fix_data.ship_timestamp = lqm->time;
		usbl_fix_data.latitude = lqm->lat;
		usbl_fix_data.longitude = lqm->lon;
		usbl_fix_data.heading = lqm->heading;
		usbl_fix_data.roll = lqm->roll;
		usbl_fix_data.pitch = lqm->pitch;
		usbl_fix_data.bearing = lqm->bearing;
		usbl_fix_data.range = lqm->slantRange;
        if(state->mode == NAV)
    		state->slam->handle_usblfix_data(usbl_fix_data);
	    else if(state->mode == RAW)
        {
            usbl_fix_data.print(state->raw_out);
    	    state->raw_out << endl;
    	}

	}
}

void on_seabird_ct(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const seabird_t *ct, state_c* state) 
{
	auv_data_tools::Seabird_Data seabird;
	seabird.set_raw_timestamp((double)ct->utime/1e6);
	seabird.cond = ct->conductivity;
	seabird.temp = ct->temperature;
	seabird.sal = ct->salinity;
	seabird.pres = ct->pressure;
	seabird.sos = ct->speed_of_sound;
    if(state->mode == NAV)
    	state->slam->handle_salinitytemp_data(seabird);
    else if(state->mode == RAW)
    {
        seabird.print(state->raw_out);
	    state->raw_out << endl;
	}

}

void on_seabird_depth(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const seabird_depth_t *sd, state_c* state) 
{
    auv_data_tools::Seabird_Depth_Data depth;
    depth.depth = sd->depth;
    depth.set_raw_timestamp((double)sd->utime/1e6);
    if(state->mode == NAV)
        state->slam->handle_depth_data(depth);			
    else if(state->mode == RAW)
    {
        depth.print(state->raw_out);
	    state->raw_out << endl;
	}

}

// handle a message from the RDI DVL, the old code used the PD0 message
// we are using the PD5 message as the LCM module was already written
void on_rdi(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const rdi_pd5_t *rdi, state_c* state) 
{		
	const double bad_value = -32.768;

	/*
	 * Check the status of the bottom tracking
	 * - If the values are close to bad value or
	 * - If we have no bottom lock (status != 0)
	 */
	bool btv_ok = true;
	if( (fabs(rdi->pd4.btv[0] - bad_value) < 1e-3) ||
		(fabs(rdi->pd4.btv[0] - bad_value) < 1e-3) ||
		(fabs(rdi->pd4.btv[0] - bad_value) < 1e-3) )
	{
		btv_ok = false;

	}
	if( rdi->pd4.btv_status != 0x00) {
		btv_ok = false;
	}

	state->bottomLock = btv_ok;

	if( btv_ok )
	{
	    auv_data_tools::RDI_Data rdi_data;
	    rdi_data.set_raw_timestamp((double)rdi->utime/1e6);
	    rdi_data.alt = rdi->pd4.altitude;
	    rdi_data.r1 = rdi->pd4.range[0];
	    rdi_data.r2 = rdi->pd4.range[1];
	    rdi_data.r3 = rdi->pd4.range[2];
	    rdi_data.r4 = rdi->pd4.range[3];
	    rdi_data.h = rdi->heading * RTOD;
	    rdi_data.p = rdi->pitch * RTOD;
	    rdi_data.r = rdi->roll * RTOD;
	    rdi_data.vx = rdi->pd4.btv[0];
	    rdi_data.vy = rdi->pd4.btv[1];
	    rdi_data.vz = rdi->pd4.btv[2];
	    rdi_data.COG = 0;   // FIXME
	    rdi_data.SOG = 0;   // FIXME
	    rdi_data.bt_status = rdi->pd4.btv_status;
	    rdi_data.h_true = 0;   // FIXME
	    rdi_data.p_gimbal = 0;   // FIXME
	    rdi_data.sv = rdi->pd4.speed_of_sound;
	    rdi_data.depth_rate = 0;   // FIXME
        
        double last_altitude = state->altitude;
        state->altitude = 0.05*rdi->pd4.altitude + 0.95*last_altitude;

        if(state->mode == NAV)
    		state->slam->handle_dvl_data(rdi_data);
        else if(state->mode == RAW)
        {
            rdi_data.print(state->raw_out);
	        state->raw_out << endl;
	    }
    }	
}

void on_ysi(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const ysi_t *ysi, state_c* state) 
{
	// use the YSI sensor to get depth
   	auv_data_tools::YSI_Data ysi_data;
	ysi_data.temperature = ysi->temperature;
	ysi_data.depth = ysi->depth;// FIXME - depthTareYsi;
    ysi_data.turbidity = ysi->turbidity;
    ysi_data.chlorophyl = ysi->chlorophyl;
    ysi_data.conductivity = ysi->conductivity;
    ysi_data.oxygen = ysi->oxygen;
    ysi_data.battery = ysi->battery;
    ysi_data.salinity = ysi->salinity;		
	ysi_data.set_raw_timestamp((double)ysi->utime/1e6);

    if(state->mode == NAV)
    	state->slam->handle_YSI_data(ysi_data);
    else if(state->mode == RAW)
    {
        ysi_data.print(state->raw_out);
        state->raw_out << endl;
    }

}		

void on_oas(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const oas_t *oas, state_c* state) 
{
    auv_data_tools::OAS_Data oas_data;
    
    oas_data.prof_range = oas->fProfRange;
    oas_data.pseudo_alt = oas->fPseudoAlt;
    oas_data.psedo_forward_distance = oas->fPseudoFwdDistance;
    
    
    state->fwd_obs_dist = oas->fPseudoFwdDistance;
    state->oas_altitude = oas->fPseudoAlt;
    
    if(state->mode == RAW)
    {
        oas_data.print(state->raw_out);
        state->raw_out << endl;
    }
}

void on_tcm_compass(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const tcm_t *tcm, state_c* state) 
{
    auv_data_tools::TCM_Data tcm_data;
    tcm_data.roll = tcm->roll;
    tcm_data.pitch = tcm->pitch;
    tcm_data.heading = tcm->heading;
    tcm_data.set_raw_timestamp((double)tcm->utime/1e6);

    if(state->mode == NAV)
       	state->slam->handle_TCM_data(tcm_data);
    else if(state->mode == RAW)
    {
        tcm_data.print(state->raw_out);
        state->raw_out << endl;
    }
}

void on_vis(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const auv_vis_rawlog_t *vis, state_c* state) 
{
	// the only thing we do here at the moment is process the message for the raw log
	auv_data_tools::Vision_Data vis_data;
	
	//vis_data.image_name = string(vis->image_name) + ".tif";
	vis_data.image_name = string(vis->image_name) ;
	if (vis_data.image_name.find(".tif") == std::string::npos)
		   vis_data.image_name = string(vis->image_name) + ".tif";
	vis_data.set_raw_timestamp((double)vis->utime/1e6);
	vis_data.set_vis_timestamp((double)vis->utime/1e6);
	//fix to shift timestamps on RAW log file for 20110603 cam was 3.61 sec ahead
	//visData.set_vis_timestamp((double)vis->utime/1e6-3.61);
	vis_data.exposure = vis->exp_time;
    if(state->mode == RAW)
    {
		vis_data.print(state->raw_out);
	    state->raw_out << endl;
	}
}

void on_ms_gx1(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const ms_gx1_t *ms, state_c* state) 
{
	// handle ms gx1 messages
	auv_data_tools::ThreeDM_Data ms_data;	

    // check ms->bitmask
	// static const int ValidMask = (SENLCM_MS_GX1_T_STAB_EULER | SENLCM_MS_GX1_T_INST_ACCEL | SENLCM_MS_GX1_T_STAB_ANGRATE);
	//bool valid = ((ms->bitmask & ValidMask) == ValidMask);
	bool valid=1;
    if(valid) 
    {
	    // rpy
	    ms_data.roll = ms->iEuler[0]*180/M_PI;
	    ms_data.pitch = ms->iEuler[1]*180/M_PI;
	    ms_data.yaw = ms->iEuler[2]*180/M_PI;

        // coverted Accelerations X, Y, Z
	    ms_data.accel_x = ms->iAccel[0];
	    ms_data.accel_y = ms->iAccel[1];
	    ms_data.accel_z = ms->iAccel[2];

        // converted Rates X, Y, Z
	    ms_data.rate_x = ms->iAngRate[0];
	    ms_data.rate_y = ms->iAngRate[1];
	    ms_data.rate_z = ms->iAngRate[2];

        // todo: use the rest of info avail in senlcm_ms_gx1_t ()
		
	    ms_data.set_raw_timestamp((double)ms->utime/1e6);
	    
	    if(state->mode == NAV)
           	state->slam->handle_threedm_data(ms_data);
        else if(state->mode == RAW)
        {
            ms_data.print(state->raw_out);
            state->raw_out << endl;
        }
		    
    }
}

void on_os_compass(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const os_compass_t *osc, state_c* state)
{
    auv_data_tools::OS_Compass_Data osc_data;
    osc_data.roll = osc->rph[0];
    osc_data.pitch = osc->rph[1];
    osc_data.heading = osc->rph[2];
    osc_data.depth = osc->rph[4];
    osc_data.set_raw_timestamp((double)osc->utime/1e6);

    if(state->mode == NAV)
       	state->slam->handle_OS_compass_data(osc_data);
    else if(state->mode == RAW)
    {
        osc_data.print(state->raw_out);
        state->raw_out << endl;
    }
}

void on_imu(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const IMU_t *imu, state_c* state)
{
    auv_data_tools::IMU_Data imu_data;

    //cout << "IMU reading recieved" << endl;

    imu_data.set_raw_timestamp((double)imu->utime/1e6);

    imu_data.angXrate_rad_persec = imu->angRate[0];    // Rotation rate (radians/s)
    imu_data.angYrate_rad_persec = imu->angRate[1];
    imu_data.angZrate_rad_persec = imu->angRate[2];
    imu_data.linXacc_m_persec_sqr = imu->accel[0];   // Linear acceleration (m/s^2)
    imu_data.linYacc_m_persec_sqr = imu->accel[1];
    imu_data.linZacc_m_persec_sqr = imu->accel[2];

    state->slam->handle_imu_data(imu_data);
}

void on_evologics(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const usbl_fix_t *usbl, state_c* state)
{
    auv_data_tools::Evologics_Fix_Data usbl_data;

    //cout << "IMU reading recieved" << endl;

    usbl_data.set_raw_timestamp((double)usbl->utime/1e6);
    usbl_data.target_lat = usbl->latitude * RTOD;    
    usbl_data.target_lon = usbl->longitude * RTOD;
    usbl_data.target_depth = usbl->depth;
    usbl_data.accuracy = usbl->accuracy;
    usbl_data.ship_lat = usbl->ship_latitude * RTOD;
    usbl_data.ship_lon = usbl->ship_longitude * RTOD;
    usbl_data.ship_roll = usbl->ship_roll * RTOD;
    usbl_data.ship_pitch = usbl->ship_pitch * RTOD;
    usbl_data.ship_heading = usbl->ship_heading * RTOD;
    usbl_data.target_x = usbl->target_x;
    usbl_data.target_y = usbl->target_y;
    usbl_data.target_z = usbl->target_z;
    
    if(state->mode == NAV)
       	state->slam->handle_evologicsfix_data(usbl_data);
    else if(state->mode == RAW)
    {
        usbl_data.print(state->raw_out);
        state->raw_out << endl;
    }
    
}

void on_uvc_dvl(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const uvc_dvl_t *dvl, state_c* state)
{
    auv_data_tools::Evologics_Fix_Data usbl_data;

    const double bad_value = 999.99;

	/*
	 * Check the status of the bottom tracking
	 * - If the values are close to bad value or
	 * - If we have no bottom lock (status != 0)
	 */
	bool btv_ok = true;
	if( (fabs(dvl->vx - bad_value) < 1e-3) ||
		(fabs(dvl->vy - bad_value) < 1e-3) ||
		(fabs(dvl->vz - bad_value) < 1e-3) ||
		(fabs(dvl->alt - bad_value) < 1e-3))
	{
		btv_ok = false;

	}

	state->bottomLock = btv_ok;
	
	//if( btv_ok )
	{
	    auv_data_tools::RDI_Data rdi_data;
	    rdi_data.set_raw_timestamp((double)dvl->utime/1e6);
	    rdi_data.alt = dvl->alt;
	    rdi_data.r1 = 0.0;
	    rdi_data.r2 = 0.0;
	    rdi_data.r3 = 0.0;
	    rdi_data.r4 = 0.0;
	    rdi_data.h = 0.0;
	    rdi_data.p = 0.0;
	    rdi_data.r = 0.0;
	    rdi_data.nx = 0.0;
	    rdi_data.ny = 0.0;
	    rdi_data.nz = 0.0;
	    rdi_data.vx = dvl->vx;
	    rdi_data.vy = dvl->vy;
	    rdi_data.vz = dvl->vz;
	    rdi_data.COG = 0;   // FIXME
	    rdi_data.SOG = 0;   // FIXME
	    rdi_data.bt_status = btv_ok ? 0 : 1;
	    rdi_data.h_true = 0;   // FIXME
	    rdi_data.p_gimbal = 0;   // FIXME
	    rdi_data.sv = 1500;
	    rdi_data.depth_rate = 0;   // FIXME
        

        if(state->mode == NAV)
    		state->slam->handle_dvl_data(rdi_data);
        else if(state->mode == RAW)
        {
            rdi_data.print(state->raw_out);
	        state->raw_out << endl;
	    }
	
    }
}

void on_uvc_rph(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const uvc_rphtd_t *osc, state_c* state)
{
    auv_data_tools::OS_Compass_Data osc_data;
    osc_data.roll = osc->rph[0];
    osc_data.pitch = osc->rph[1];
    osc_data.heading = osc->rph[2];
    
    // The handler in seabed interface assumes the depth of this message is in feet.
    osc_data.depth = osc->rph[4] / UNITS_FEET_TO_METER;
    
    osc_data.set_raw_timestamp((double)osc->utime/1e6);

    if(state->mode == NAV)
       	state->slam->handle_OS_compass_data(osc_data);
    else if(state->mode == RAW)
    {
        osc_data.print(state->raw_out);
        state->raw_out << endl;
    }
}

void on_seabotix_sensors(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const seabotix_sensors_t *ss, state_c* state)
{
    // spoof an OS compass
    auv_data_tools::OS_Compass_Data osc_data;
    osc_data.roll = ss->roll;
    osc_data.pitch = ss->pitch;
    osc_data.heading = ss->heading;
    
    // The handler in seabed interface assumes the depth of this message is in feet.
    osc_data.depth = ss->depth / UNITS_FEET_TO_METER;
    
    osc_data.set_raw_timestamp((double)ss->utime/1e6);

    if(state->mode == NAV)
       	state->slam->handle_OS_compass_data(osc_data);
    else if(state->mode == RAW)
    {
        osc_data.print(state->raw_out);
        state->raw_out << endl;
    }
}

// This is not complete, it spoofs the RDI DVL and only fills in the altitude variable to be used with processing the images 
void on_micron_sounder(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const micron_sounder_t *ms, state_c* state)
{
    if(fabs(ms->altitude) < 0.000001)
        return;
    auv_data_tools::RDI_Data rdi_data;
    memset(&rdi_data, 0, sizeof(auv_data_tools::RDI_Data));
    rdi_data.set_raw_timestamp((double)ms->utime/1e6);
    rdi_data.alt = ms->altitude;
    if(state->mode == RAW)
    {
        rdi_data.print(state->raw_out);
        state->raw_out << endl;
    }
}


