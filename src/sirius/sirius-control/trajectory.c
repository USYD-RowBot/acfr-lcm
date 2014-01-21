#include "control.h"
#include "control_utils.h"


static void generate_speed_trajectory(double speed_goal, double max_acc, double tau, traj_fields_t *tf) 
{ 
    double new_ref;
    double alpha = exp(- CONTROL_DT / tau); 
    double dx = speed_goal - tf->reference;
    tf->velocity = alpha*tf->velocity + (1-alpha)*max_acc*sgn(dx);
    
    if (fabs(dx/tau) > max_acc)
        new_ref = tf->reference + tf->velocity*CONTROL_DT;
    else
    { 
        new_ref = alpha*tf->reference + (1-alpha)*speed_goal ;
        tf->velocity = (new_ref - tf->reference) / CONTROL_DT ;
    }
  
   tf->reference      = new_ref;
   tf->rate_reference = tf->velocity;

} /* END: generate_speed_trajectory() */  


static void generate_depth_trajectory(double depth_goal, double max_vel, double tau, traj_fields_t *tf)
{ 
    double new_ref;
    double alpha = exp(- CONTROL_DT / tau); 
    double old_rate = tf->rate_reference;  
    double dx = depth_goal - tf->reference;
    tf->velocity = alpha*tf->velocity + (1-alpha)*max_vel*sgn(dx);
    
    if (fabs(dx/tau) > max_vel)
        new_ref = tf->reference + tf->velocity*CONTROL_DT;
    else
    { 
        new_ref = alpha*tf->reference + (1-alpha)*depth_goal ;
        tf->velocity = (new_ref - tf->reference) / CONTROL_DT ;
    }

    tf->reference      = new_ref;
    tf->rate_reference = tf->velocity;
    tf->acc = (tf->velocity - old_rate)/CONTROL_DT;
} /* END: generate_depth_trajectory */


static void generate_heading_trajectory(double heading_goal, double max_vel, double tau, traj_fields_t *tf)
{ 
    double alpha = exp(-CONTROL_DT / tau);   
    double dx = delta_heading(heading_goal - tf->reference);
    double last_ref = heading_goal - dx;
    double old_rate = tf->rate_reference;
    double new_ref;

    tf->velocity = alpha*tf->velocity + (1-alpha)*max_vel*sgn(dx);
  
    if(fabs(dx/tau ) > max_vel)
        new_ref = last_ref + tf->velocity*CONTROL_DT;
    else
    { 
        new_ref = alpha*last_ref + (1-alpha)*heading_goal;
        tf->velocity = delta_heading( new_ref - tf->reference)/CONTROL_DT;
    }
  
    wrap_heading(&new_ref);
    tf->reference = new_ref;
    tf->rate_reference = tf->velocity;
    tf->acc = (tf->velocity - old_rate)/CONTROL_DT;

} /* END: generate_heading_trajectory */

static void tl_path_plan(goal_setpoint_t *gs, acfrlcm_auv_acfr_nav_t *es, double *dvv_x, double *dvv_y, trajectory_config_t *tc)
{
    double xi, yi;
    double del_xie, del_yie, del_xes, del_yes;

    double del_xts = gs->xpos2 - gs->xpos1;
    double del_yts = gs->ypos2 - gs->ypos1;

    double del_xte = es->x - gs->xpos2;
    double del_yte = es->y - gs->ypos2;

    double del_x = gs->xpos2 - es->x;
    double del_y = gs->ypos2 - es->y;

    if( del_xte*del_xts + del_yte*del_yts < 0 ) //follow the line, have not passed end point
    {
        del_xes = es->x - gs->xpos1; 
        del_yes = es->y - gs->ypos1;

        // intersect point (orthogonal projection)
        xi = (es->x)*pow(del_xts,2) + (gs->xpos1)*pow(del_yts,2)+del_yes*del_xts*del_yts;
	    xi = xi/(pow(del_xts,2)+pow(del_yts,2));

        //if (gs->xpos1 != gs->xpos2)
        if (fabs(gs->xpos2 - gs->xpos1) > 0.001)
	        yi = del_yts/del_xts*(xi-gs->xpos1) + gs->ypos1;
	    else
	        yi = es->y;

	    del_xie = xi - es->x;
	    del_yie = yi - es->y;

	    if(norm(del_xie,del_yie) > tc->tl_band )  
	    {
            *dvv_x = tc->tl_gain*del_xie/norm(del_xie,del_yie) + del_x/norm(del_x,del_y);  
            *dvv_y = tc->tl_gain*del_yie/norm(del_xie,del_yie) + del_y/norm(del_x,del_y);  
        }
	    else
	    {
            *dvv_x = tc->tl_gain*del_xie/tc->tl_band + del_x/norm(del_x,del_y); 
	        *dvv_y = tc->tl_gain*del_yie/tc->tl_band + del_y/norm(del_x,del_y); 
	    }  
    }
    else //desired velocity directly at the end point (past the endpoint)
    { 
        *dvv_x = del_x/norm(del_x,del_y);
        *dvv_y = del_y/norm(del_x,del_y);
    }

    *dvv_x = *dvv_x/norm(*dvv_x,*dvv_y);
    *dvv_y = *dvv_y/norm(*dvv_x,*dvv_y);

}



//main function
//------------------------------------------------------------------------------------
void trajectory(goal_setpoint_t *gs, trajectory_state_t *ts, acfrlcm_auv_acfr_nav_t *es, trajectory_config_t *tc , trajectory_mode_t *tm, controller_state_t *cs, debug_t *debug)
{ 

    double goal_depth, goal_altitude, goal_heading, goal_speed_surge, goal_speed_sway;
  
    //trackline 2P variable and gains 
    double del_x, del_y;
    double dvv_x, dvv_y;
    double des_angle, vel_angle, diff_angle;
    double sway;
    double RADIAL_VEL, ROA;

    RADIAL_VEL = .4;
    ROA = 1;

    /*=========================== DEPTH TRAJECTORY ==================================*/
    if (tm->depth.mode == TRAJ_OFF)
    { // keep the reference at the current depth, avoid jumps when starts back up.
        ts->depth.reference = es->depth;  
        ts->depth.goal      = NO_VALUE;
        ts->depth.velocity  = 0;
        cs->depth_int = 0;
    }
    else  // TRAJ_ON
    { 
        if(tm->depth.mode == AUTO_DEPTH)
	        goal_depth = es->depth + (es->altitude - tm->depth.value); // tm->depth.value is actually desired alt
        else if (tm->depth.mode == TRACKLINE_1P)
	        goal_depth = gs->zpos2;
        else if (tm->depth.mode == TRACKLINE_2P)
	        goal_depth = gs->zpos2;
        else
        {
            //fprintf(stderr,"***ERROR - Unknown depth mode in trajectory.c ***\n");
            goal_depth = 0;
        }  
      
        generate_depth_trajectory(goal_depth, tc->depth_rate_limit, tc->depth_tau, &(ts->depth));
        ts->depth.goal = goal_depth;
    } 

    /*=========================== ALTITUDE TRAJECTORY ==================================*/
    // 2010-04-20    mvj    Added this block for when controlling directly off altitude.
    //                      Altitude trajectory is computed whenever depth traj is computed.
    if (tm->depth.mode == TRAJ_OFF)
    { 
        ts->altitude.reference = es->altitude;  
        ts->altitude.goal      = NO_VALUE;
        ts->altitude.velocity  = 0;
        cs->altitude_int = 0;
    }
    else  // TRAJ_ON
    { 
        if (tm->depth.mode == AUTO_DEPTH)
	        goal_altitude = tm->depth.value; // tm->depth.value is actually desired alt
        else
        {
            //fprintf(stderr,"***ERROR - Unknown altitude mode in trajectory.c ***\n");
            goal_altitude = 0;
        }  
      
        generate_depth_trajectory(goal_altitude, tc->depth_rate_limit, tc->depth_tau, &(ts->altitude));
        ts->altitude.goal = goal_altitude;
    } 

    /*=========================== HEADING TRAJECTORY =================================*/
    if (tm->heading == TRAJ_OFF)
    { // keep it at the current heading, avoid jumps   
        ts->heading.reference = es->heading;
        ts->heading.goal      = NO_VALUE;
        ts->heading.velocity  = 0;  
        cs->heading_int       = 0;
    }
    else // TRAJ_ON
    { 
        if (tm->heading == MAN_HEADING)
	        goal_heading = gs->heading; 
        else if (tm->heading == TRACKLINE_1P) 
        { 
            del_x = gs->xpos2 - es->x;
            del_y = gs->ypos2 - es->y;
            goal_heading = atan2(del_y,del_x);
            wrap_heading(&goal_heading);
        }        
        else if(tm->heading == TRACKLINE_2P)
        { 
            tl_path_plan(gs, es, &dvv_x, &dvv_y, tc);


            // original code worked out desired change in velocity vector to
            // achieve desired velocity vector.  Change this to set a heading
            // setpoint to bring the vehicle onto the line and let the 
            // heading controller sort it out.  Hold that thought...
            //goal_heading = atan2(dvv_y, dvv_x);
//printf("es_h: %f goal_h: %f\n", es->heading*RTOD,  goal_heading*RTOD);

            double vx_wld = es->vx*cos(es->heading) - es->vy*sin(es->heading);
            double vy_wld = es->vx*sin(es->heading) + es->vy*cos(es->heading);

            vel_angle = atan2(vy_wld, vx_wld);
            des_angle = atan2(dvv_y,dvv_x);
    
            diff_angle = des_angle - vel_angle;
            diff_angle = delta_heading(diff_angle);

            if( fabs(diff_angle) > tc->two_point_max_step )  
	            diff_angle = sgn(diff_angle)*tc->two_point_max_step;
	    
            goal_heading = tc->two_point_gain*diff_angle + es->heading;

	        wrap_heading(&goal_heading);
//printf("vel_angle: %f des_angle: %f diff_angle: %f g_h: %f\n", vel_angle*RTOD, des_angle*RTOD, diff_angle*RTOD,  goal_heading*RTOD);
        }
        else if(tm->heading == TRACKLINE_2P_H)
            goal_heading = gs->heading;

            ts->heading.goal = goal_heading;
            generate_heading_trajectory(goal_heading, tc->heading_rate_limit, 
				  tc->heading_tau, &(ts->heading));
    }

    /*========================= SPEED TRAJECTORY ====================================*/


    if (tm->transit == TRAJ_OFF)
    { 
        ts->surge.reference = es->vx;
        ts->surge.goal      = NO_VALUE;
        ts->surge.velocity  = 0;
        cs->surge_int          = 0;
        ts->sway.reference = es->vy;
        ts->sway.goal      = NO_VALUE;
        ts->sway.velocity  = 0;
        cs->sway_int          = 0;
    }
    else // TRAJ_ON
    { 
        double desired_xy_vel = gs->xy_vel;
        if(tc->fwd_distance_min > 0 && es->fwd_obstacle_dist != NO_VALUE)
        {
        /*
      	Obstacle avoidance slowdown.  The desired velocity will
	    be ramped down linearly between the slowdown distance and
	    the minimum distance.

        speed
         ^         ____________ default speed
         |        /
         |       /
        0|______/____________________________>  dist
		^  ^
	      min   slowdown
        */
            if (es->fwd_obstacle_dist < tc->fwd_distance_min)
	        {
		        desired_xy_vel = 0;
	        } 
	        else if (es->fwd_obstacle_dist < tc->fwd_distance_slowdown) 
	        {
		        double speedRampSlope = gs->xy_vel/(tc->fwd_distance_slowdown - tc->fwd_distance_min);
		        desired_xy_vel = speedRampSlope*(es->fwd_obstacle_dist - tc->fwd_distance_min);
	        }
        } 
      
        if(tm->transit == CLOOP)
        {
       	    sway = es->vy;

	        if( fabs(sway) < desired_xy_vel )
 	            goal_speed_surge = sqrt( pow(desired_xy_vel,2) - pow(sway,2));
	        else
	            goal_speed_surge = 0;
	
	        generate_speed_trajectory(goal_speed_surge, tc->surge_rate_limit, tc->surge_tau, &(ts->surge)); 
            ts->surge.goal = goal_speed_surge;
            ts->sway.reference = es->vy;
            ts->sway.goal      = NO_VALUE;
            ts->sway.velocity  = 0;
	        cs->sway_int = 0;
        }
        else if(tm->transit == CLOOP_CRAB)
        {
	        tl_path_plan(gs, es, &dvv_x, &dvv_y, tc);
	
	        goal_speed_surge = desired_xy_vel*(cos(es->heading)*dvv_x + sin(es->heading)*dvv_y);
	        goal_speed_sway = desired_xy_vel*(sin(es->heading)*dvv_x - cos(es->heading)*dvv_y);
	
	        generate_speed_trajectory(goal_speed_surge, tc->surge_rate_limit, tc->surge_tau, &(ts->surge)); 
	        generate_speed_trajectory(goal_speed_sway, tc->sway_rate_limit, tc->sway_tau, &(ts->sway)); 
        }
        else if(tm->transit == CLOOP_HOLD)
        {
	        del_x = gs->xpos2 - es->x;
	        del_y = gs->ypos2 - es->y;

	        dvv_x = del_x/norm(del_x,del_y);
	        dvv_y = del_y/norm(del_x,del_y);

	        if( norm(del_x,del_y) > ROA )
	        {
                goal_speed_surge = RADIAL_VEL*(cos(es->heading)*dvv_x + sin(es->heading)*dvv_y);
	            goal_speed_sway = RADIAL_VEL*(sin(es->heading)*dvv_x - cos(es->heading)*dvv_y);
	
                generate_speed_trajectory(goal_speed_surge, tc->surge_rate_limit, tc->surge_tau, &(ts->surge)); 
	            generate_speed_trajectory(goal_speed_sway, tc->sway_rate_limit, tc->sway_tau, &(ts->sway)); 
	        }
	        else
	        {
	            goal_speed_surge = RADIAL_VEL*norm(del_x,del_y)/ROA*(cos(es->heading)*dvv_x + sin(es->heading)*dvv_y);
	            goal_speed_sway = RADIAL_VEL*norm(del_x,del_y)/ROA*(sin(es->heading)*dvv_x - cos(es->heading)*dvv_y);

                generate_speed_trajectory(goal_speed_surge, tc->surge_rate_limit, tc->surge_tau, &(ts->surge)); 
	            generate_speed_trajectory(goal_speed_sway, tc->sway_rate_limit, tc->sway_tau, &(ts->sway)); 
	        }
        } 
        else if (tm->transit == OLOOP)
        {
            //printf("Setting oloop reference to %f\n", gs->xy_vel);
            ts->surge.reference = gs->xy_vel;
        }
    }
} /* END: trajectory */     




