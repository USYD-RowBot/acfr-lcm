/*
    ACFR navigation LCM handler routines
    
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

    if((gps->status >= 1) & (gps->fix.mode >= 2) & (gps->satellites_used >= 3 ))
        if(state->mode == NAV)
            state->slam->handle_gps_data(gps_data);
        else if(state->mode == RAW)
        {
            gps_data.print(state->raw_out);
		    state->raw_out << endl;
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
	if((rdi->pd4.btv[0] != -32.786) && (rdi->pd4.btv[1] != -32.786) && (rdi->pd4.btv[2] != -32.786)) {
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
        
        state->altitude = rdi->pd4.altitude;
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

