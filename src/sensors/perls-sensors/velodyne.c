#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <glib.h>
#include <bot_core/bot_core.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include "perls-math/so3.h"
#include "perls-math/ssc.h"
#include "perls-math/gsl_util.h"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/magic.h"
#include "perls-common/units.h"

#include "velodyne.h"


// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

int
velodyne_calib_precompute (velodyne_calib_t *calib);

void
velodyne_read_intrinsic_calibration_file (velodyne_calib_t *calib, const char *db_xml_file_path);


// -----------------------------------------------------------------------------
// create the calibration structure
// -----------------------------------------------------------------------------
velodyne_calib_t *
velodyne_calib_create (velodyne_sensor_type_t sensor_type, const char *db_xml_file_path)
{
    velodyne_calib_t *calib = calloc (1, sizeof (*calib));
      
    // get the number of lasers and sensor type
    if (sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E)
	calib->num_lasers = VELODYNE32_NUM_LASERS;
    else if (sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S1 ||
	     sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S2)
	calib->num_lasers = VELODYNE64_NUM_LASERS;

    calib->sensor_type = sensor_type;
    
    calib->lasers = calloc (calib->num_lasers, sizeof (*calib->lasers)); 
    calib->physical2logical = calloc (calib->num_lasers, sizeof (*calib->physical2logical)); 
    calib->logical2physical = calloc (calib->num_lasers, sizeof (*calib->logical2physical)); 
    calib->va_sin = calloc (calib->num_lasers, sizeof (*calib->va_sin)); 
    calib->va_cos = calloc (calib->num_lasers, sizeof (*calib->va_cos));
    
    velodyne_read_intrinsic_calibration_file (calib, db_xml_file_path);
    
    calib->calibrated_intensity = calloc (calib->num_lasers, sizeof (int*));
    for (int i=0; i<(calib->num_lasers); i++)
	calib->calibrated_intensity[i] = calloc (256, sizeof(int));
    
    velodyne_calib_precompute (calib);
    
    bot_fasttrig_init ();

    return calib;
}

void
velodyne_calib_free (velodyne_calib_t * calib) {
    free (calib->lasers);
    free (calib->physical2logical);
    free (calib->logical2physical);
    free (calib->va_sin);
    free (calib->va_cos);
    for (int i=0; i<(calib->num_lasers); i++)
	free (calib->calibrated_intensity[i]);
    free (calib->calibrated_intensity);
    free (calib);
}


perllcm_velodyne_laser_return_collection_t *
_decode_data_packet (velodyne_calib_t *calib, uint8_t *data, int data_len, int64_t utime, uint8_t subsamp, double subsamp_pct)
{    
    int return_cnt = 0;
  
    assert (data_len == VELODYNE_DATA_PACKET_LEN);
    
    int num_returns = 0;
    uint8_t mask[VELODYNE_NUM_LASER_RETURNS_PER_PACKET] = {0};
    if (subsamp) {
	num_returns = ceil ((double)VELODYNE_NUM_LASER_RETURNS_PER_PACKET*subsamp_pct);
	if (num_returns == VELODYNE_NUM_LASER_RETURNS_PER_PACKET)
	    subsamp = 0; 
	else {
	    gsl_rng *r = gslu_rand_rng_alloc ();
	    uint16_t set[VELODYNE_NUM_LASER_RETURNS_PER_PACKET];
	    uint16_t subset[num_returns];
	    for (uint16_t i=0; i<VELODYNE_NUM_LASER_RETURNS_PER_PACKET; i++) {set[i] = i;}
	    gsl_ran_choose (r, subset, num_returns, set, VELODYNE_NUM_LASER_RETURNS_PER_PACKET, sizeof (uint16_t));
	    gsl_rng_free (r);
	    for (int i=0; i<num_returns; i++)
		mask[subset[i]] = 1;
	}
    }
    else
	num_returns = VELODYNE_NUM_LASER_RETURNS_PER_PACKET;
    
    perllcm_velodyne_laser_return_collection_t *lrc = velodyne_calloc_laser_return_collection (num_returns, 0);
    
    // in a packet there are 12 firing blocks each with 32 laser returns
    int i_f = 0;	//firing index = 0..11 
    int i_l = 0;	//laser index index = 0..31
    
    // TODO read status messages (including gps timestamp)
    
    // timestamp for collection based on first laser in packet
    // utime passed in represents last firing in packet
    // if we were all synced up time wise using GPS we could calculate using
    // timestamp parsed from packet which would be better, but now we are doing
    // timestamp sync in the driver and passing that utime in
    if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E)
	lrc->utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET(0, 0);
    else
	lrc->utime = utime; // TODO: get timing table implementted for velodyne 64s
    
    int laser_offset = 0;

    // loop over each firing in this packet
    for (i_f = 0; i_f<VELODYNE_NUM_FIRING_PER_PACKET ; i_f++) {
	
	uint16_t start_id = VELODYNE_GET_START_IDENTIFIER (data, VELODYNE_DATA_FIRING_START (i_f));

        if (start_id == VELODYNE_UPPER_START_IDENTIFIER) {
            //Upper block lasers are numbered 1-32 in the db.xml file
            //but they are 32-63 in the velodyne-uncalib.h file
            laser_offset = 0;
        }
        else if (start_id == VELODYNE_LOWER_START_IDENTIFIER)
            laser_offset = 32; 
        else
            ERROR ("ERROR: Unknown Velodyne start identifier %4x\n", start_id);
      
        // position of velodyne head, constant for all 32 measurements that follow
        double ctheta = VELODYNE_GET_ROT_POS (data, VELODYNE_DATA_FIRING_START (i_f));

	if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S1) 
            ctheta = 2*M_PI - ctheta;

        if (ctheta >= 2*M_PI)
	    ctheta = 0;
        
	// cache the cos and sin of the constant heading measurement for the next 32 meas to follow
	double sin_ctheta, cos_ctheta;
        fsincos (ctheta, &sin_ctheta, &cos_ctheta);
        
	int64_t fire_start_utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET (i_f, 0);

	// loop over each laser in this firing
	for (i_l = 0; i_l<VELODYNE_NUM_LASERS_PER_FIRING; i_l++) {
	    
	    //fill this structure
	    perllcm_velodyne_laser_return_t *lr;
	    if (subsamp) {
		if (mask[i_f*VELODYNE_NUM_LASERS_PER_FIRING + i_l]) {
		    lr = &(lrc->laser_returns[return_cnt]);
		    return_cnt++;
		}
                else
		    continue;
	    } 
            else
		lr = &(lrc->laser_returns[i_f*VELODYNE_NUM_LASERS_PER_FIRING + i_l]);
	
	    if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S1) {
		//ERROR ("ERROR: decoding not implemented for VELODYNE_SENSOR_TYPE_HDL_64E_S1");
		lr->utime = utime; // + VELODYNE_32_LASER_FIRING_TIME_OFFSET (i_f, i_l);
		// compensate for intershot yaw change
		if (i_l%4 == 0) { //yaw changes by SPIN_RATE*4 between the groups of four firings
		    ctheta += VELODYNE_SPIN_RATE * VELODYNE_INTRA_SHOT_USEC;
		    if (ctheta >= 2*M_PI){
			ctheta = 0; //seems weird, shouldnt it roll over instead of going to zero? --NCB
		    }
		    fsincos (ctheta, &sin_ctheta, &cos_ctheta);
		}
		   
		lr->physical = laser_offset + i_l;
		lr->logical  = velodyne_physical_to_logical (calib, lr->physical);
		   
		velodyne_laser_calib_t *params = &calib->lasers[lr->physical];
		   
		lr->raw_range = VELODYNE_GET_RANGE(data, VELODYNE_DATA_LASER_START(i_f, i_l));
		lr->range     = (lr->raw_range + params->range_offset) * (1.0 + params->range_scale_offset);
		lr->ctheta    = ctheta;
		lr->theta     = bot_mod2pi_ref (M_PI, ctheta + params->rcf);
		lr->phi       = params->vcf;
		lr->intensity = VELODYNE_GET_INTENSITY(data, VELODYNE_DATA_LASER_START(i_f, i_l));
		   
		double sin_theta, cos_theta;
		fsincos (lr->theta, &sin_theta, &cos_theta);
		double sin_phi = calib->va_sin[lr->physical];
		double cos_phi = calib->va_cos[lr->physical];
	
                lr->theta = ctheta; 	
                double Dxy;
		double Z;
		//Here following the MIT coordinate convention for the laser scanner.
		//MIT coordinate system ===> X forward, Y left and Z up. 
		//Our MATLAB coordinate system ===> Y forward, X right and Z up. 
		if(laser_offset == 0){
		    //Upper block
		    // The numbers and the equation to calculate Dxy and Z are taken from
		    // the presentation send by velodyne people: refer diagram.ppt.
		    Dxy = lr->range*cos_phi - 2.85*2.54*0.01*cos_phi;
		    Z = params->voffset + 2.8309*2.54*0.01*sin_phi/cos_phi + lr->range*sin_phi - 2.85*2.54*0.01*sin_phi;
		}
		else{
		    //Lower block
		    Dxy = lr->range*cos_phi - 1.8*2.54*0.01*cos_phi;
		    Z = params->voffset + 2.4464*2.54*0.01*sin_phi/cos_phi + lr->range*sin_phi - 1.8*2.54*0.01*sin_phi;
		}
		   
		lr->xyz[0] = Dxy*cos_theta - params->hcf*cos_ctheta;
		lr->xyz[1] = Dxy*sin_theta + params->hcf*sin_ctheta;
		lr->xyz[2] = Z;   
	    }
	    else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S2) {
		ERROR ("ERROR: decoding not implemented for VELODYNE_SENSOR_TYPE_HDL_64E_S2");
	    }
	    else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E) {
		
		// according to velodyne the 32E shouldn't need any calibration
		// beyond the vertical correction, so curently no other corrections
		// are applied
		
		lr->utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET (i_f, i_l);
	
		// compensate for intershot yaw change
		// at 100m this can be up to 1/4 of a meter of correction
		int64_t usec_since_first = lr->utime - fire_start_utime;
		double ctheta_yaw_cmp = ctheta + (VELODYNE_SPIN_RATE * usec_since_first);
	    
		lr->physical = laser_offset + i_l;
		lr->logical  = velodyne_physical_to_logical (calib, lr->physical);
	    
		velodyne_laser_calib_t *params = &calib->lasers[lr->physical];
		lr->raw_range = VELODYNE_GET_RANGE (data, VELODYNE_DATA_LASER_START (i_f, i_l));
		lr->range     = lr->raw_range;
		lr->ctheta    = ctheta;
		lr->theta     = ctheta_yaw_cmp;
		lr->phi       = params->vcf;
		lr->intensity = VELODYNE_GET_INTENSITY (data, VELODYNE_DATA_LASER_START (i_f, i_l));
	    
		double sin_theta, cos_theta;
		fsincos (lr->theta, &sin_theta, &cos_theta);
		double sin_phi = calib->va_sin[lr->physical];
		double cos_phi = calib->va_cos[lr->physical];
		
		lr->xyz[0] = lr->range * cos_theta * cos_phi;
		lr->xyz[1] = -lr->range * sin_theta * cos_phi;
		lr->xyz[2] = lr->range * sin_phi;
	
	    }
	}
    }
    
    // remove points too close to the velodyne
    int num_remove = 0;
    uint8_t *remove = calloc(num_returns, sizeof (uint8_t));
    for (int i=0; i<num_returns; i++){
	if (lrc->laser_returns[i].range < VELODYNE_MIN_RANGE) {
	    num_remove++;
	    remove[i] = 1;
	}
    }

    perllcm_velodyne_laser_return_collection_t *lrc_out = velodyne_calloc_laser_return_collection (num_returns-num_remove, 0);
    lrc_out->num_lr = num_returns-num_remove;
    lrc_out->num_lrl = 0;
    lrc_out->utime = lrc->utime;
    int j=0;
    for (int i=0; i<num_returns; i++){
	if (!remove[i]) {
	    lrc_out->laser_returns[j] = lrc->laser_returns[i];
	    j++;
	}
    }
    velodyne_free_laser_return_collection (lrc);
    free (remove);
    return lrc_out;
}


perllcm_velodyne_laser_return_collection_t *
velodyne_decode_data_packet(velodyne_calib_t* calib, uint8_t *data, int data_len, int64_t utime)
{
    return _decode_data_packet (calib, data, data_len, utime, 0, 100);
}

perllcm_velodyne_laser_return_collection_t *
velodyne_decode_data_packet_subsamp(velodyne_calib_t* calib, uint8_t *data, int data_len, int64_t utime, double subsamp_pct)
{
    return _decode_data_packet (calib, data, data_len, utime, 1, subsamp_pct);
}

perllcm_velodyne_laser_return_collection_t *
velodyne_calloc_laser_return_collection (int num_samples, int num_lite_samples)
{
    perllcm_velodyne_laser_return_collection_t *lrc = calloc (1, sizeof (*lrc));
    lrc->num_lr = num_samples;
    if (lrc->num_lr > 0) {
	lrc->laser_returns = calloc (lrc->num_lr, sizeof (*(lrc->laser_returns)));
    } else {
	lrc->laser_returns = NULL;
    }
    lrc->num_lrl = num_lite_samples;
    if (lrc->num_lrl > 0) {
	lrc->laser_returns_lite = calloc (lrc->num_lrl, sizeof (*(lrc->laser_returns_lite)));
    } else {
	lrc->laser_returns_lite = NULL;
    }
    
    return lrc;
}

void
velodyne_free_laser_return_collection (perllcm_velodyne_laser_return_collection_t *lrc)
{    
    perllcm_velodyne_laser_return_collection_t_destroy (lrc);
}


velodyne_laser_return_collector_t *
velodyne_laser_return_collector_create (uint8_t whole_scan, double start_angle, double end_angle, double x_vs[6])
{    
    velodyne_laser_return_collector_t *collector = calloc (1, sizeof (*collector));
    
    memcpy (collector->x_vs, x_vs, 6*sizeof (double));
    collector->whole_scan = whole_scan;
    collector->start_angle = start_angle;
    collector->end_angle = end_angle;
    collector->num_lr = 0;
    collector->laser_returns = g_array_new (FALSE, TRUE, sizeof (perllcm_velodyne_laser_return_t));
    collector->state.utime = 0;
    collector->collection_ready = 0;
    collector->utime_collection = 0;                 
    collector->collecting = 0;
    collector->packet_cnt = 0;
    
    return collector;
}

// clean up to start a new collection
void
velodyne_reset_collector (velodyne_laser_return_collector_t *collector)
{    
    g_array_free (collector->laser_returns, TRUE);
    collector->num_lr = 0;
    collector->laser_returns = g_array_new (FALSE, TRUE, sizeof (perllcm_velodyne_laser_return_t));
    collector->utime_collection = 0; 
    collector->collection_ready = 0;
    collector->first_laser_has_pose = 0;
    collector->packet_cnt = 0;
}

void
velodyne_laser_return_collector_free (velodyne_laser_return_collector_t *collector)
{    
    g_array_free (collector->laser_returns, TRUE);
    free (collector);
}

uint8_t
in_collecting_range (double theta, double start, double end)
{
    // two cases
    if (start <= end) { // no rollover
	if (theta <= end && theta >= start)
	    return 1;
	else
	    return 0;
    }
    else { // rollover at 2PI
	if (theta >= start || theta <= end)
	    return 1;
	else
	    return 0;
    }
}

void
motion_compensate (velodyne_laser_return_collector_t *collector,
		   perllcm_velodyne_laser_return_collection_t *new_returns)
{    
    // data packet rate 1.8 Khz and pose rate max is about 0.1 Khz
    // so for each data packet we only need to find the first laser to current pose once
    double x_flp_cp[6] = {0}; //first laser pose to current pose
    ssc_tail2tail (x_flp_cp, NULL, collector->first_laser_pose, collector->state.xyzrph);
    
    // NOTE if you don't care about the difference in dt within a datapacket (approx 500 usec)
    // (for anything less than a car moving on the highway you probably don't)
    // you can save a lot of CPU time by motion compensating the entire datapacket at once
    // #if switches between this and motion compensating each individual laser return
    
#if 1 // compensate the entire data packet to at once based on timestamp of first laser in packet

    double dt = (new_returns->utime - collector->state.utime)/1e6;
    double x_flp_clp[6] = {0}; // pose at this laser firing (current pose + velocities*dt)
    x_flp_clp[0] = x_flp_cp[0] + dt*collector->state.xyzrph_dot[0];
    x_flp_clp[1] = x_flp_cp[1] + dt*collector->state.xyzrph_dot[1];
    x_flp_clp[2] = x_flp_cp[2] + dt*collector->state.xyzrph_dot[2];
    x_flp_clp[3] = x_flp_cp[3] + dt*collector->state.xyzrph_dot[3];
    x_flp_clp[4] = x_flp_cp[4] + dt*collector->state.xyzrph_dot[4];
    x_flp_clp[5] = x_flp_cp[5] + dt*collector->state.xyzrph_dot[5];
    
    double R_clp_flp[9];
    double rph_flp_clp[3] = {x_flp_clp[3], x_flp_clp[4], x_flp_clp[5]};
    so3_rotxyz (R_clp_flp, rph_flp_clp);
    gsl_matrix_view R_clp_flp_v = gsl_matrix_view_array (R_clp_flp, 3, 3);
    
    // stack all laser returns into matrix
    gsl_matrix *xyz_stacked = gsl_matrix_calloc (3, new_returns->num_lr);
    gsl_matrix *xyz_stacked_r = gsl_matrix_calloc (3, new_returns->num_lr);
    for (int i=0 ; i<(new_returns->num_lr) ; i++) {
	perllcm_velodyne_laser_return_t *lr = &(new_returns->laser_returns[i]);
	gsl_matrix_set (xyz_stacked, 0, i, lr->xyz[0]);
	gsl_matrix_set (xyz_stacked, 1, i, lr->xyz[1]);
	gsl_matrix_set (xyz_stacked, 2, i, lr->xyz[2]);
    }
    // rotate laser returns
    gslu_blas_mm (xyz_stacked_r, &R_clp_flp_v.matrix, xyz_stacked);

    // translate and pull data out of matrix
    for (int i=0 ; i<(new_returns->num_lr) ; i++) {
	perllcm_velodyne_laser_return_t *lr = &(new_returns->laser_returns[i]);
	lr->xyz[0] = gsl_matrix_get (xyz_stacked_r, 0, i) + x_flp_clp[0];
	lr->xyz[1] = gsl_matrix_get (xyz_stacked_r, 1, i) + x_flp_clp[1];
	lr->xyz[2] = gsl_matrix_get (xyz_stacked_r, 2, i) + x_flp_clp[2];
    }
    
    gsl_matrix_free (xyz_stacked);
    gsl_matrix_free (xyz_stacked_r);

#else // compensate each laser return based on its individual timestamp
    
    for (int i=0 ; i<(new_returns->num_lr) ; i++) {
	
	perllcm_velodyne_laser_return_t *lr = &(new_returns->laser_returns[i]);
	
	// compensate laser
	double dt = (lr->utime - collector->state.utime)/1e6;
	double x_flp_clp[6] = {0}; // pose at this laser firing (current pose + velocities*dt)
	x_flp_clp[0] = x_flp_cp[0] + dt*collector->state.xyzrph_dot[0];
	x_flp_clp[1] = x_flp_cp[1] + dt*collector->state.xyzrph_dot[1];
	x_flp_clp[2] = x_flp_cp[2] + dt*collector->state.xyzrph_dot[2];
	x_flp_clp[3] = x_flp_cp[3] + dt*collector->state.xyzrph_dot[3];
	x_flp_clp[4] = x_flp_cp[4] + dt*collector->state.xyzrph_dot[4];
	x_flp_clp[5] = x_flp_cp[5] + dt*collector->state.xyzrph_dot[5];
    
	double R_clp_flp[9];
	double rph_flp_clp[3] = {x_flp_clp[3], x_flp_clp[4], x_flp_clp[5]};
	so3_rotxyz (R_clp_flp, rph_flp_clp);
	GSLU_VECTOR_VIEW (xyz, 3, {lr->xyz[0], lr->xyz[1], lr->xyz[2]});
	GSLU_VECTOR_VIEW (xyz_r, 3, {0});
	// rotate into first laser pose frame
	gsl_matrix_view R_clp_flp_v = gsl_matrix_view_array (R_clp_flp, 3, 3);
	gslu_blas_mv (&xyz_r.vector, &R_clp_flp_v.matrix, &xyz.vector);
	// include tranlation between first laser pose and current laser pose
	lr->xyz[0] = gsl_vector_get (&xyz_r.vector, 0) + x_flp_clp[0];
	lr->xyz[1] = gsl_vector_get (&xyz_r.vector, 1) + x_flp_clp[1];
	lr->xyz[2] = gsl_vector_get (&xyz_r.vector, 2) + x_flp_clp[2];
	
	lr->motion_compensated = 1;
    }
    
#endif
}

velodyne_collector_push_return_t
velodyne_collector_push_laser_returns (velodyne_laser_return_collector_t *collector,
                                       perllcm_velodyne_laser_return_collection_t *new_returns)
{
    if (new_returns->num_lr == 0) {
	return VELODYNE_COLLECTION_PUSH_OK;
    }
    
    if (!collector->collecting) {
	
	// start collecting again if we are collecting whole scans or if we have re-entered
	if (collector->whole_scan || 
            in_collecting_range (new_returns->laser_returns[0].theta, collector->start_angle, collector->end_angle)) {
	    // reset collector
	    velodyne_reset_collector (collector);
	    if (collector->whole_scan)
		collector->start_angle = new_returns->laser_returns[0].theta;

	    collector->utime_first_laser = new_returns->laser_returns[0].utime;
	    // do we have a pose for the start of this collection?
	    if (collector->state.utime != 0) {
		double dt = (collector->state.utime - collector->utime_first_laser)/1e6;
		collector->first_laser_pose[0] = collector->state.xyzrph[0] + dt*collector->state.xyzrph_dot[0];
		collector->first_laser_pose[1] = collector->state.xyzrph[1] + dt*collector->state.xyzrph_dot[1];
		collector->first_laser_pose[2] = collector->state.xyzrph[2] + dt*collector->state.xyzrph_dot[2];
		collector->first_laser_pose[3] = collector->state.xyzrph[3] + dt*collector->state.xyzrph_dot[3];
		collector->first_laser_pose[4] = collector->state.xyzrph[4] + dt*collector->state.xyzrph_dot[4];
		collector->first_laser_pose[5] = collector->state.xyzrph[5] + dt*collector->state.xyzrph_dot[5];
		collector->first_laser_has_pose = 1;
	    }
	    
	    // start collecting
	    collector->collecting = 1;
	}
    }
    
    if (collector->collecting) {
	// motion compensation 
	// we have a pose to compensate into and the current pose ins't stale
	if (collector->first_laser_has_pose && abs(new_returns->utime - collector->state.utime) < 2e5)
	    motion_compensate (collector, new_returns);
    
	// push the new returns onto the collection
	for (int i=0 ; i<(new_returns->num_lr) ; i++) {
	    g_array_append_val (collector->laser_returns, new_returns->laser_returns[i]);
	    collector->num_lr++; 
	}
	
	// check for collection finish
	if (collector->whole_scan) {
            
            collector->packet_cnt++;
            
	    if (collector->packet_cnt > VELODYNE_PACKETS_PER_REV) {
		// done collecting, return
		collector->collecting = 0;
		collector->collection_ready = 1;
		return VELODYNE_COLLECTION_READY; 
	    }
	    
	}
        else {    
	    if (!in_collecting_range (new_returns->laser_returns[new_returns->num_lr-1].theta,
                                      collector->start_angle, collector->end_angle)) {
		    // done collecting, return
		    collector->collecting = 0;
		    collector->collection_ready = 1;
		    return VELODYNE_COLLECTION_READY;
	    }
	}
    }
    
    return VELODYNE_COLLECTION_PUSH_OK;
    //return VELODYNE_COLLECTION_PUSH_ERROR;
}

velodyne_collector_push_return_t
velodyne_collector_push_state (velodyne_laser_return_collector_t *collector, perllcm_position_t state_in)
{    
    gsl_vector_view v = gsl_vector_view_array (collector->x_vs, 6);
    double norm = gslu_vector_norm (&v.vector);
    if (norm <= 0.000001 ) { // x_vs is identity
	memcpy (&(collector->state), &state_in, sizeof (perllcm_position_t));
	return VELODYNE_COLLECTION_PUSH_OK;
    } 
    else {
	perllcm_position_t state = {0};
    
        state.utime = state_in.utime;

        // find sensor pose in local/world frame
        double *x_lr = state_in.xyzrph;
        ssc_head2tail (state.xyzrph, NULL, x_lr, collector->x_vs);

	// calculate velocities and rates at sensor head =======================
	
	//rotate world frame velocities into body frame, for vehicle -----------
        double R_vw[9];
        so3_rotxyz (R_vw, &(state_in.xyzrph[3]));
        gsl_matrix_view R_wv_v = gsl_matrix_view_array (R_vw, 3, 3);
	gsl_matrix_transpose (&R_wv_v.matrix); // now R_wv
        GSLU_VECTOR_VIEW (xyz_dot_v, 3, {state_in.xyzrph_dot[0],
					 state_in.xyzrph_dot[1],
					 state_in.xyzrph_dot[2]});
	double uvw_v[3] = {0};
        gsl_vector_view uvw_v_v = gsl_vector_view_array (uvw_v, 3);
        gslu_blas_mv (&uvw_v_v.vector, &R_wv_v.matrix, &xyz_dot_v.vector);
	
	// get body rates of the vehicle ---------------------------------------
	double abc_v[3];
	so3_euler2body (&(state_in.xyzrph_dot[3]), &(state_in.xyzrph[3]), abc_v, NULL);
        
	// find body velocitites of sensor -------------------------------------
	double R_sv[9];
	double rph_vs[3] = {collector->x_vs[3], collector->x_vs[4], collector->x_vs[5]};
	so3_rotxyz (R_sv, rph_vs);
	double t_vs[3] = {collector->x_vs[0], collector->x_vs[1], collector->x_vs[2]};
    
	double skewsym[9] = { 0,       -t_vs[2],  t_vs[1],
			      t_vs[2],  0,       -t_vs[0],
			     -t_vs[1],  t_vs[0],  0        };
	
	double uvw_s[3] = {0};   
	gsl_vector_view uvw_s_v = gsl_vector_view_array (uvw_s, 3);
	gsl_vector_view abc_v_v = gsl_vector_view_array (abc_v, 3);
	gsl_matrix_view R_sv_v = gsl_matrix_view_array (R_sv, 3, 3);
	gsl_matrix_view skewsym_v = gsl_matrix_view_array (skewsym, 3, 3);
	//skewsym(t_vs)*abc;
	gslu_blas_mv (&uvw_s_v.vector, &skewsym_v.matrix, &abc_v_v.vector);
	//[uvw - skewsym(t_vs)*abc]
	gsl_vector_sub (&uvw_v_v.vector, &uvw_s_v.vector);
	//z_predict = O_sv * [uvw - skewsym(t_vs)*abc];
	gslu_blas_mv (&uvw_s_v.vector, &R_sv_v.matrix, &uvw_v_v.vector);
	
	// find body rates of sensor -------------------------------------------
	double abc_s[3] = {0}; 
	gsl_vector_view abc_s_v = gsl_vector_view_array (abc_s, 3);
	gsl_matrix_view R_vs_v = gsl_matrix_view_array (R_sv, 3, 3);
	gsl_matrix_transpose (&R_vs_v.matrix);
	gslu_blas_mv (&abc_s_v.vector, &R_vs_v.matrix, &abc_v_v.vector);
	
	//rotate body velocities into world frame for sensor -------------------
        double R_sw[9];
        so3_rotxyz (R_sw, &(state.xyzrph[3])); 
        gsl_matrix_view R_sw_v = gsl_matrix_view_array (R_sw, 3, 3);
        gsl_vector_view xyz_dot_s = gsl_vector_view_array (&(state.xyzrph_dot[0]), 3);
        gslu_blas_mv (&xyz_dot_s.vector, &R_sw_v.matrix, &uvw_s_v.vector);
        
        // set euler rates for sensor -----------------------------------------
        so3_body2euler (abc_s, &(state.xyzrph[3]), &(state.xyzrph_dot[3]), NULL);
	
	// good way to verify
	//double delta = 0.001;	
	//// way 3
	//double x_lr2[6] = {state_in.xyzrph[0] + state_in.xyzrph_dot[0] *  delta,
	//		   state_in.xyzrph[1] + state_in.xyzrph_dot[1] *  delta,
	//		   state_in.xyzrph[2] + state_in.xyzrph_dot[2] *  delta,
	//		   state_in.xyzrph[3] + state_in.xyzrph_dot[3] *  delta,
	//		   state_in.xyzrph[4] + state_in.xyzrph_dot[4] *  delta,
	//		   state_in.xyzrph[5] + state_in.xyzrph_dot[5] *  delta};
	//		   
	//double x_ls1[6] = {0};
	//double x_ls2[6] = {0};
	//ssc_head2tail (x_ls1, NULL, x_lr, collector->x_vs);
	//ssc_head2tail (x_ls2, NULL, x_lr2, collector->x_vs);
	//
	//double xyzrph_dot_3[6] = {(x_ls2[0] - x_ls1[0]) / delta,
	//			  (x_ls2[1] - x_ls1[1]) / delta,
	//			  (x_ls2[2] - x_ls1[2]) / delta,
	//			  (x_ls2[3] - x_ls1[3]) / delta,
	//			  (x_ls2[4] - x_ls1[4]) / delta,
	//			  (x_ls2[5] - x_ls1[5]) / delta};


	memcpy (&(collector->state), &state, sizeof (perllcm_position_t));
	return VELODYNE_COLLECTION_PUSH_OK;
    }
}

perllcm_velodyne_laser_return_collection_t *
velodyne_collector_pull_collection (velodyne_laser_return_collector_t *collector)
{    
    if (!collector->collection_ready)
	return NULL;
    
    perllcm_velodyne_laser_return_collection_t *lrc = velodyne_calloc_laser_return_collection (collector->num_lr, 0);
    
    lrc->utime = collector->utime_first_laser;
    memcpy (lrc->x_vs, collector->x_vs, 6*sizeof (double));
    
    // copy the laser returns to the output
    for (int i=0; i<(lrc->num_lr); i++)
	lrc->laser_returns[i] = g_array_index (collector->laser_returns, perllcm_velodyne_laser_return_t, i);

    // reset collector
    velodyne_reset_collector (collector);
    
    return lrc;
}


static velodyne_calib_t *__v;

static int 
laser_phi_compare (const void *_a, const void *_b)
{
    int a = *((int*) _a);
    int b = *((int*) _b);

    if (__v->lasers[a].vcf < __v->lasers[b].vcf) 
        return -1;
    return 1;
}

// NOT REENTRANT
int
velodyne_calib_precompute (velodyne_calib_t *calib)
{
    assert (!__v); // check for reentrancy...

    __v = calib;

    for (int i = 0; i < calib->num_lasers; i++)
        calib->logical2physical[i] = i;
    qsort (calib->logical2physical, calib->num_lasers, sizeof (int), laser_phi_compare);
    
    for (int logical = 0; logical < calib->num_lasers; logical++)
        calib->physical2logical[calib->logical2physical[logical]] = logical;

    for (int physical = 0; physical < calib->num_lasers; physical++)
        sincos (calib->lasers[physical].vcf, &calib->va_sin[physical], & calib->va_cos[physical]);
    
    __v = NULL;

    return 0;
}

void
parse_current_item (xmlDocPtr doc, xmlNodePtr cur, velodyne_calib_t* calib, int index)
{ 
    //Get the node px 
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"px")))
            break;

        cur = cur->next;
    }
    //printf("%s\n", cur->name);
 
    //Get all the data for current laser 
    xmlChar *key;
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"id_"))) {
            key = xmlNodeListGetString (doc, cur->xmlChildrenNode, 1);
            index = atoi ((char*)key);
            //printf ("id: %s\n", key);
            xmlFree (key);
        }
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"rotCorrection_"))) {
            key = xmlNodeListGetString (doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].rcf = atof ((char*)key)*M_PI/180;
            //printf ("rotCorrection: %f\n", calib->lasers[index].rcf);
            xmlFree (key);
        }
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"vertCorrection_"))) {
            key = xmlNodeListGetString (doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].vcf = atof ((char*)key)*M_PI/180;
            //printf ("vertCorrection: %f\n", calib->lasers[index].vcf);
            xmlFree (key);
        }
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"distCorrection_"))) {
            key = xmlNodeListGetString (doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].range_offset = atof ((char*)key)*0.01; 
            //printf ("distCorrection: %f\n", calib->lasers[index].range_offset);
            xmlFree (key);
        }
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"vertOffsetCorrection_"))) {
            key = xmlNodeListGetString (doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].voffset = atof ((char*)key)*0.01; 
            //printf ("vertOffsetCorrection: %f\n", calib->lasers[index].voffset);
            xmlFree (key);
        }
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"horizOffsetCorrection_"))) {
            key = xmlNodeListGetString (doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].hcf = atof ((char*)key)*0.01;
            //printf ("horizOffsetCorrection: %f\n", calib->lasers[index].hcf);
            xmlFree (key);
        }
        cur = cur->next;
    }
}
 
void 
velodyne_read_intrinsic_calibration_file (velodyne_calib_t *calib, const char *db_xml_file_path)
{ 
    xmlDocPtr doc;
    xmlNodePtr cur;
    doc = xmlParseFile (db_xml_file_path);
    if (doc == NULL ) {
        fprintf (stderr, "Calibration file not parsed successfully. \n");
        return;
    }  
    cur = xmlDocGetRootElement (doc);
    if (cur == NULL) {
        fprintf (stderr, "empty document\n");
        xmlFreeDoc (doc);
        return;
    }
    if (xmlStrcmp (cur->name, (const xmlChar *) "boost_serialization")) {
        fprintf (stderr, "document of the wrong type, root node != story");
        xmlFreeDoc (doc);
        return;
    }
    //printf ("%s\n", cur->name);

    //Get the node DB
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"DB"))){
            break;
        }
        cur = cur->next;
    }
    //printf ("%s\n", cur->name);

    // in the new db.xml they have a enabled node that tells you which of the 64
    // lasers are enabled

    //Get the node points_ 
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"points_"))){
            break;
        }
        cur = cur->next;
    }
    //printf ("%s\n", cur->name);
  
    xmlNodePtr curPoints = cur;
  
    xmlChar *key;
    //int numLasers;
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"count"))) {
            key = xmlNodeListGetString (doc, cur->xmlChildrenNode, 1);
            //printf("Count: %s\n", key);
            //numLasers = atof ((char*)key);
            xmlFree (key);
            break;
        }
        cur = cur->next;
    }
   
    //printf ("%s\n", cur->name);
    //printf ("%s %d\n", curPoints->name, numLasers);
  
    //Loop over all the item of node points_
    cur = curPoints->xmlChildrenNode;
    int index = 0;
    while (cur != NULL &&  index < calib->num_lasers) {
        if ((!xmlStrcmp (cur->name, (const xmlChar *)"item"))) {
            //populate the calibration data in velodyne_calib_t struct.
            parse_current_item (doc, cur, calib, index) ;
            calib->lasers[index].range_scale_offset = 0;
            index++;
        }
        cur = cur->next;
    }
}

void
velodyne_read_calibrated_intensity (velodyne_calib_t *calib, const char *db_xml_file_path)
{  
    FILE *fptr = fopen (db_xml_file_path, "r");    
    char temp;
    int i, j;
    for (i = 0; i<(calib->num_lasers); i++) {
        for (j = 0; j < 256; j++)
            fscanf (fptr,"%d", &calib->calibrated_intensity[i][j]);
        fscanf (fptr,"%c",&temp);
    }
  
}

void
velodyne_calib_dump (velodyne_calib_t *calib)
{
    printf ("velodyne_laser_calib_t[] = {\n");
    for (int i = 0; i < (calib->num_lasers); i++) {
        velodyne_laser_calib_t *params = &calib->lasers[i];
        printf ("   { %11.7f, %11.7f, %8.4f, %8.4f, %10.6f }, // laser %2d\n", 
                params->rcf, params->vcf, params->hcf, params->range_offset, params->range_scale_offset, i+1);
    }
    printf ("};\n\n");
}
