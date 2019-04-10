/*
 * OccupancyMap.cpp
 *
 *  Created on: Mar 23, 2012
 *      Author: s4155021
 */

#include "OccupancyMap.hpp"

int programExit;


/**
 * Print the contect of the map.
 * Can print to file if the filestream is pointing at the fstream element
 */
void printMap( state_t * state, std::ostream* fp = &cout ) {
	if( fp == &cout )
		*fp << "Map contains " << state->map.size() << " cells:" << endl;

	for( state->iter = state->map.begin(); state->iter < state->map.end(); state->iter++ ) {
		if( fp == &cout )
			*fp << "\t";
		*fp << (*state->iter).toString() << endl;
	}
}

/**
 * Checks if the point is out of bounds of the map
 */
bool boundaryCheck( state_t * state, Vector3D & p ) {
	for( int i = 0; i < 3; i++ ) {
		if( fabs( p[i] ) > state->mapSize[i] / 2 ) {
			//cout << "Point out of map range: " << p.toString() << endl;
			return false;
		}
	}
	return true;
}

/**
 * UPDATE MAP by moving the location of the occupied cells according to the shift in vehicle pose
 */
void updateMap( state_t * state, Matrix44 & T ) {
	Vector3D pN;
	Vector4D p4;
	for( state->iter = state->map.begin(); state->iter < state->map.end(); state->iter++ ) {

		// convert to the coordinates of the new pose
		pN = (*state->iter).loc;
		p4 = pN[0], pN[1], pN[2], 1;
		p4 = T * p4;

		// boundary check
		for( int j = 0; j < 3; j++ ) {
			// voxelize
			pN[j] = (int) (p4[j] / state->voxelSize) * state->voxelSize;
		}

		if( !boundaryCheck( state, pN ) ) {
//			cout << "Point is now outside of map " << (*state->iter).toString() << endl;
			state->map.erase( state->iter );
		}
		else {
			(*state->iter).loc = pN;
		}
	}
}

/**
 * DECREMENT NON-EMPTY CELLS
 * We decrement by an exponential function so it resembles half-life similar
 * 	to radio active particles :)
 */
//
void decMap( state_t * state ) {

    double t = timestamp_now()/1.e6;
	for( state->iter = state->map.begin(); state->iter < state->map.end(); state->iter++ ) {
		//[a b c] = getCellIndices( cells );
		//state(a,b,c) = state(a,b,c) .* 2^(-dt/MAP_HALF_TIME);
		(*state->iter).votes = (*state->iter).votes * pow( 2, -(t-(*state->iter).timestamp) / state->MAP_HALF_TIME );

		// TODO: we could use a flag instead of setting the value to zero
		if( (*state->iter).votes < state->MAP_MIN_VAL ) {
//			cout << "Deleting " << (*state->iter).loc << endl;
			state->map.erase( state->iter );
		}
	}
}

/**
 * INCREMENT MAP FROM the 3D point p
 * TODO: Should we change this to call by reference? Current implentation might be expensive
 * 	in creating new vectors all the time
 */
void incMap( state_t * state, vector<Vector4D> pVec ) {

	Vector3D pN;
	double t = timestamp_now() / 1.e6;
	
	for( unsigned int i = 0; i < pVec.size(); i++ ) {
		// voxelize
		for( int j = 0; j < 3; j++ ) {
			pN[j] = (int) round( pVec[i][j] / state->voxelSize ) * state->voxelSize;
		}

		// BoundaryCheck
		if( !boundaryCheck( state, pN ) ) {
			continue;
		}

		// check if this cell already exists
		bool exists = false;
		for( state->iter = state->map.begin(); state->iter < state->map.end();
					state->iter++ ) {
			// increment voxel
			if( ((*state->iter).loc - pN).normSq() < 1e-3 ) {
				(*state->iter).votes += state->MAP_HIT_VAL;
				(*state->iter).timestamp = t;
				exists = true;
				break;
			}
		}

		// add voxel
		if( !exists ) {
			state->map.push_back( OccMapCell( pN, state->MAP_HIT_VAL, t ) );
		}
	}
}

/**
 * GET SONAR CONE points from the range and cone angle
 */
vector<Vector4D> getSonarPoints( double range, double coneAngleH, double coneAngleV, double voxelSize ) {

	vector<Vector4D> pVec;
	Vector4D p;

	double coneWidthH = sin( coneAngleH / 2.0 ) * range;
	double coneWidthV = sin( coneAngleV / 2.0 ) * range;
	
	if( coneWidthH < voxelSize && coneWidthV < voxelSize ) {
		p = range, 0, 0, 1;
		pVec.push_back( p );
		return pVec;
	}

	for( double j = -coneWidthV; j <= coneWidthV; j += voxelSize ) {
		for( double i = -coneWidthH; i <= coneWidthH; i += voxelSize ) {
			double dp = hypot( i, j );
			double l = sqrt( range * range - dp * dp );
			if( l > 1e-6 && dp - coneWidthH < 1e-6 ) {
				p = l, i, j, 1;
				pVec.push_back( p );
			}
		}
	}
	return pVec;
}

/**
 * Read RDI range from one of the beams and return 3D point
 */
void senlcm_rdi_t_handle(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const senlcm::rdi_pd5_t *msg, state_t* state) {

    state->time = (double)(msg->utime)/1.0e6;

    Vector4D p4;
    vector<Vector4D> pVec;
    
	for( int beam = 0; beam < 4; beam++ ) {
	    double range = msg->pd4.range[beam];    	
	    if( range > state->RDI_MAX_RANGE || range < state->RDI_MIN_RANGE ) {
    		return;
	    }

    	// Convert to 3D point seen from RDI
	    double rot = 30.0 / 180.0 * M_PI;
	    double offset = 0.1315;
	    double a = M_PI / 2.0 * (beam + 1);// NOTE: THIS IS THE OFFSET TO BEAM zero! + M_PI / 4.0;
	    Matrix44 R1, R2;
	    R1 = cos( rot ), 0, sin( rot ), 0, 0, 1, 0, 0, -sin( rot ), 0, cos( rot ), 0, 0, 0, 0, 1;
	    R2 = 1, 0, 0, 0, 0, cos( a ), -sin( a ), 0, 0, sin( a ), cos( a ), 0, 0, 0, 0, 1;

    	vector<Vector4D> pVecTemp = getSonarPoints(
	    			range + offset, state->RDI_CONEANGLE, state->RDI_CONEANGLE, state->voxelSize );
	    for( unsigned int i = 0; i < pVecTemp.size(); i++ ) {
		    p4 = R2 * R1 * pVecTemp[i];

    		// Convert 3D point seen from RDI beam
	    	// to 3D point seen from vehicle
	    	p4 = state->RDI_T * p4;
            pVec.push_back( p4 );	    	
	    }// For every point in this beam
	}// For every beam

    incMap( state, pVec );
}

void senlcm_micron_ping_t_handle(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const senlcm::micron_ping_t *msg, state_t* state) {

    // TODO: Put this somewhere sensible
    uint8_t minReturnThresh = 0;
    
    Vector4D p4;
    vector<Vector4D> pVec;
    
    // Find the largest return
    uint8_t maxReturn = 0;
    int maxReturnIdx = 0;
    for( unsigned int i = 0; i < msg->bins.size(); i++ ) {
        if( msg->bins[i] > maxReturn ) {
            maxReturn = msg->bins[i];
            maxReturnIdx = i;
        }
    }
    
    // If below threshold skip
    if( maxReturn < minReturnThresh ) 
        return;
    
    // Calculate range
    double range = 1.0; //maxReturnIdx*msg->range_resolution;
    // Get sonar head angle
    double a = msg->angle-M_PI; // TODO: what is up and what is down?

    // Sonar angle rotation matrix    
    Matrix44 Rsonar; Rsonar = cos(a), 0, sin(a), 0, 
               0, 1, 0, 0,
               -sin(a), 0, cos(a), 0,
               0, 0, 0, 1;

    // Current vehicle pose
    Matrix44 Tpose; state->pose.get4x4TransformationMatrix();
    
    // Convert to world frame
    vector<Vector4D> pVecTemp = getSonarPoints( range, 30./180.*M_PI, 3./180.*M_PI, state->voxelSize );
	for( unsigned int i = 0; i < pVecTemp.size(); i++ ) {
        p4 = Tpose * state->MICRON_T * Rsonar * pVecTemp[i];
        cout << range << ", " << a/M_PI*180 << "," << p4[0] << "," << p4[1] << "," << p4[2] << ";" << endl;
    	
        pVec.push_back( p4 );	    	
	}// For every point in this beam
    
    // Update map
    incMap( state, pVec );
    
}

/**
 * READ OAS SENSOR and transform the range to 3D point relative to the vehicle centre-of-motion
 */
/* 
static void senlcm_oas_t_handle( const lcm_recv_buf_t *rbuf, const char * channel,
			const senlcm_oas_t * msg, void * user ) {

    state_t * state = (state_t *)user;
    vector<Vector4D> pVec;
    Vector4D p4;
*/
    /*
	printf ("Received OAS message on channel \"%s\":\n", channel);
	printf ("  timestamp   = %ld\n", msg->utime);
	
	printf( "  range       = %f [m]", msg->fProfRange );

    // READ OAS
	double range = msg->fProfRange;
	if( range > state->OAS_MAX_RANGE || range < state->OAS_MIN_RANGE ) {
		return;
	}

	// Convert 3D point seen from OAS
	//   to 3D point seen from vehicle
	vector<Vector4D> pVecTemp = getSonarPoints( range, state->OAS_CONEANGLE, state->voxelSize );
	for( unsigned int i = 0; i < pVec.size(); i++ ) {
		p4 = state->OAS_T * pVec[i];
        pVec.push_back( p4 );
		//	Vector3D pOAST;
		//	pOAST = oas1[loopCount], oas2[loopCount], oas3[loopCount];
		//	cout << "OAS: calculated: " << pOAS.toString() << ", TRUE=" << pOAST.toString() << endl;
	}
	
	incMap( state, pVec );
*/
/*
	return;
}
*/


/**
 * Get VEHICLE POSE
 */
void acfrlcm_auv_acfr_nav_t_handle(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_acfr_nav_t *msg, state_t* state) {

    state->posePrev = state->pose;
    
	state->pose.setPosition( msg->x, msg->y, msg->depth );
	state->pose.setRollPitchYawRad( msg->roll, msg->pitch, msg->heading );
}

/**
 *
 */
void perllcm_heartbeat_t_handle(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const perllcm::heartbeat_t *msg, state_t* state) {

    state->time = (double)(msg->utime)/1.0e6;
    Pose3D pose;
    Pose3D posePrev;

	// decay with the delta time
    decMap( state );
	
	// update map
	// move with the vehicle pose
	pose = state->pose;
	posePrev = state->posePrev;
	
	Matrix44 T = pose.get4x4TransformationMatrix();
	Matrix44 dT = (pose.delta( posePrev ).get4x4TransformationMatrix());
	updateMap( state, dT );
	

    // publish to LCM
    acfrlcm::auv_occmap_t lcmMap;
    lcmMap.utime = timestamp_now();
    lcmMap.num_elements = state->map.size();
    for( register unsigned int i = 0; i < state->map.size(); i++ ) {
        acfrlcm::auv_occmap_element_t e;
        e.x = state->map[i].loc[0];
        e.y = state->map[i].loc[1];
        e.z = state->map[i].loc[2];
        e.votes = state->map[i].votes;
        e.timestamp = state->map[i].timestamp;
        lcmMap.elements.push_back(e);
    }
    state->lcm.publish(state->vehicle_name + ".OCC_MAP", &lcmMap);

	return ;
}



void signalHandler(int sig_num) 
{
    programExit = 1;
}
/**
 *
 */

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, state_t *state)
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
            state->vehicle_name = optarg;
            break;
         }
    }
}

int main( int argc, char ** argv ) {

    programExit = 0;
    signal(SIGINT, signalHandler);

	state_t state;
	parse_args (argc, argv, &state);
	// Read the config variables from the param server
	BotParam *param = NULL;
    param = bot_param_new_from_server (state.lcm.getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
    
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "nav.%s", basename(argv[0]));

	// Map variables
	sprintf(key, "%s.map_half_time", rootkey);
    state.MAP_HALF_TIME = bot_param_get_double_or_fail(param, key);
	
	sprintf(key, "%s.map_hit_val", rootkey);
    state.MAP_HIT_VAL = bot_param_get_double_or_fail(param, key);
	
	sprintf(key, "%s.map_max_val", rootkey);
    state.MAP_MAX_VAL = bot_param_get_double_or_fail(param, key);
    
	sprintf(key, "%s.map_min_val", rootkey);
    state.MAP_MIN_VAL = bot_param_get_double_or_fail(param, key);
    
	sprintf(key, "%s.map_size", rootkey);
    bot_param_get_double_array_or_fail(param, key, state.mapSize, 3);
	
	sprintf(key, "%s.voxel_size", rootkey);
    state.voxelSize = bot_param_get_double_or_fail(param, key);

    // Sensor positions relative to vehicle nav center
    double pos[6];
    
    sprintf(key, "sensors.micron.position");
    bot_param_get_double_array(param, key, pos, 6);	
	state.MICRON_T = Pose3D(makeVector(pos[0], pos[1], pos[2]), Rotation3D(pos[3]*DTOR, pos[4]*DTOR, pos[5]*DTOR)).get4x4TransformationMatrix();
	
    sprintf(key, "sensors.rdi.position");
    bot_param_get_double_array_or_fail(param, key, pos, 6);	
	state.RDI_T = Pose3D(makeVector(pos[0], pos[1], pos[2]), Rotation3D(pos[3]*DTOR, pos[4]*DTOR, pos[5]*DTOR)).get4x4TransformationMatrix();
	
//	sprintf(key, "sensors.oas.position");
//    bot_param_get_double_array_or_fail(param, key, pos, 6);	
//	state.OAS_T = Pose3D(makeVector(pos[0], pos[1], pos[2]), Rotation3D(pos[3]*DTOR, pos[4]*DTOR, pos[5]*DTOR)).get4x4TransformationMatrix();
	
	// RDI specific variables
	sprintf(key, "%s.rdi_min_range", rootkey);
    state.RDI_MIN_RANGE = 0.1;
    
    sprintf(key, "sensors.rdi.range");
    state.RDI_MAX_RANGE = bot_param_get_double_or_fail(param, key);
    
    sprintf(key, "sensors.rdi.cone_angle");
    state.RDI_CONEANGLE = bot_param_get_double_or_fail(param, key) * DTOR;
	
/*	// Imagenex OAS specific variables
	sprintf(key, "%s.oas_min_range", rootkey);
    state.OAS_MIN_RANGE = bot_param_get_double_or_fail(param, key);
    
    sprintf(key, "%s.oas_max_range", rootkey);
    state.OAS_MAX_RANGE = bot_param_get_double_or_fail(param, key);
    
    sprintf(key, "%s.oas_cone_angle", rootkey);
    state.OAS_CONEANGLE = bot_param_get_double_or_fail(param, key) * DTOR;
*/
	
	//senlcm_oas_t_subscribe( state.lcm, "OAS", &senlcm_oas_t_handle, &state );
	state.lcm.subscribeFunction(state.vehicle_name + ".RDI", senlcm_rdi_t_handle, &state);
    state.lcm.subscribeFunction(state.vehicle_name + ".ACFR_NAV", acfrlcm_auv_acfr_nav_t_handle, &state );
	state.lcm.subscribeFunction("HEARTBEAT_1HZ", perllcm_heartbeat_t_handle, &state );
	state.lcm.subscribeFunction(state.vehicle_name + ".MICRON", senlcm_micron_ping_t_handle, &state );

    int fd = state.lcm.getFileno();
    fd_set rfds;
    while(!programExit)
    {
        FD_ZERO (&rfds);
        FD_SET (fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select (fd + 1, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
            state.lcm.handle();
    }
    
	// Clean up
	//print map
	ofstream printmap;
	printmap.open("/tmp/"+state.vehicle_name+"_occ_map.txt");
 	printMap(&state, &printmap);
 	printmap.close();
	return 0;
}
