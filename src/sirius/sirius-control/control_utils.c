#include "control_utils.h"

double sgn(double x)
{ 
    double result;
    result = 1.0 ;
    if (x < 0.0)  result = -1.0;
    return result;
} // end sgn

double delta_heading(double dh)
{
    double result;
    result = dh ;
    while (result >  PI)  result -= TWOPI;
    while (result < -PI)  result += TWOPI;
    return result;
}

void wrap_heading(double *h)
{ 
    while (*h > TWOPI)  *h -= TWOPI;
    while (*h < 0)      *h += TWOPI;
}

double norm(double x, double y)
{
    double result;
    result = sqrt(pow(x,2) + pow(y,2));
    return result;
} 

double comp_to_cart(double x)
{ 
    double result;
    result = -(x - PI/2);
    return result;
}
 
double cart_to_comp(double x)
{ 
    double result;
    result = -x + PI/2;
    return result;
}

int rovtime2dsltime_str(double rov_time, char *str)
{
    int    num_chars;

    // for min, sec
    double total_secs;
    double secs_in_today;

    //double day;
    double hour;
    double min;
    double sec;

    // for date and hours
    struct tm  *tm;
    time_t      current_time;

    // read gettimeofday() clock and compute min, and
    // sec with microsecond precision. Use rov_time if
    // specified (rov_time >= 0)
    if (rov_time >= 0) 
    { 
        total_secs = rov_time;
        current_time = (time_t)total_secs;
        tm = gmtime(&current_time);
    }
    else 
    {
        //use current time
        total_secs = (double)timestamp_now() / 1e6;
        current_time = time(NULL);
        tm = gmtime(&current_time);
    }
		
    secs_in_today = fmod(total_secs,86400.0); //86400=24.0*60.0*60.0
    hour = secs_in_today/3600.0;
    min  = fmod(secs_in_today/60.0,60.0);
    sec  = fmod(secs_in_today,60.0);

    num_chars = sprintf(str,"%04d/%02d/%02d %02d:%02d:%05.2f",
        (int) tm->tm_year+1900,
        (int) tm->tm_mon+1,
        (int) tm->tm_mday,
        (int) tm->tm_hour,
        (int) min,
        sec);

   
  return num_chars;
}

int check_mp_goals(ctl_status_t *cs, goal_setpoint_t *gs, trajectory_mode_t *tm, acfrlcm_auv_acfr_nav_t *es, debug_t *debug)
{
    double rad;
    
    //if (cs->goal_completed != gs->goal_id)
    { 
        cs->at_goal = 0; 
        if((tm->heading == TRACKLINE_1P) && (tm->depth.mode == TRACKLINE_1P))
        {
	 	    rad = sqrt( pow(gs->xpos2 - es->x,2) + pow(gs->ypos2 - es->y,2));
	 	    if(rad < GOAL_PT_RADIUS && fabs( gs->zpos2 - es->depth) < DEPTH_BAND)
   		    {
   		        cs->at_goal = 1;
	   		    cs->goal_completed = gs->goal_id;
	   		    debug_printf(debug->debug_mp, "MP: Completed goal %d, rad = %.3f desired %.3f %.3f actual %.3f %.3f, depth = %.3f depth_err = %.3f \n",
	   		        gs->goal_id, rad, gs->xpos2, gs->ypos2, es->x, es->y, es->depth, fabs( gs->zpos2 - es->depth));
	   		}
        }
     
        else if (tm->heading == TRACKLINE_1P && tm->depth.mode != TRACKLINE_1P)
        {
	 	    rad = sqrt(pow(gs->xpos2 - es->x,2) + pow(gs->ypos2 - es->y,2));
	 	    if(rad < GOAL_PT_RADIUS)
	   		{
	   		    cs->at_goal = 1;
	   		    cs->goal_completed = gs->goal_id;
	   		    debug_printf(debug->debug_mp, "MP: Completed Goal %d, rad = %.3f desired %.3f %.3f actual %.3f %.3f\n",
	   		        gs->goal_id, rad ,gs->xpos2, gs->ypos2, es->x, es->y);
	   		}
        }

        else if (tm->heading == TRACKLINE_2P_H && tm->depth.mode != TRACKLINE_2P)
        {
 	        rad = sqrt(pow(gs->xpos2 - es->x,2) + pow(gs->ypos2 - es->y,2));
	 	    if( rad < GOAL_PT_RADIUS )
	   		{
	   		    cs->at_goal = 1;
	   		    cs->goal_completed = gs->goal_id;
	   		    debug_printf(debug->debug_mp, "MP: Completed Goal %d, rad = %.3f desired %.3f %.3f actual %.3f %.3f\n",
	   		        gs->goal_id, rad ,gs->xpos2, gs->ypos2, es->x, es->y);
	   		}
        }
     
        else if(tm->heading == TRACKLINE_2P_H && tm->depth.mode == TRACKLINE_2P)
        {
	 	    rad = sqrt(pow(gs->xpos2 - es->x,2) + pow(gs->ypos2 - es->y,2));
	 	    if( rad < GOAL_PT_RADIUS && fabs( gs->zpos2 - es->depth ) < DEPTH_BAND )
	   		{
	   		    cs->at_goal = 1;
	   		    cs->goal_completed = gs->goal_id;
	   		    debug_printf(debug->debug_mp, "MP: Completed Goal %d, rad = %.3f desired %.3f %.3f actual %.3f %.3f, depth = %.3f depth_err = %.3f \n",
	   		        gs->goal_id, rad, gs->xpos2, gs->ypos2, es->x, es->y, es->depth, fabs( gs->zpos2 - es->depth));
	   		}
        }
     
        else if(tm->heading == TRACKLINE_2P && tm->depth.mode == TRACKLINE_2P)
        {
	 	    rad = sqrt(pow(gs->xpos2 - es->x,2) + pow(gs->ypos2 - es->y,2));
	 	    if(rad < GOAL_PT_RADIUS && fabs(gs->zpos2 - es->depth) < DEPTH_BAND)
	   		{
	   		    cs->at_goal = 1;
	   		    cs->goal_completed = gs->goal_id;
	   		    debug_printf(debug->debug_mp, "MP: Completed Goal %d, rad = %.3f desired %.3f %.3f actual %.3f %.3f, depth = %.3f depth_err = %.3f \n",
	   		        gs->goal_id, rad, gs->xpos2, gs->ypos2, es->x, es->y, es->depth, fabs( gs->zpos2 - es->depth));
	   		}
        }

        else if(tm->heading == TRACKLINE_2P && tm->depth.mode != TRACKLINE_2P)
        {
	 	    rad = sqrt(pow(gs->xpos2 - es->x,2) + pow(gs->ypos2 - es->y,2));
	 	    if(rad < GOAL_PT_RADIUS)
	   		{
	   		    cs->at_goal = 1;
	   		    cs->goal_completed = gs->goal_id;
	   		    debug_printf(debug->debug_mp, "MP: Completed Goal %d, rad = %.3f desired %.3f %.3f actual %.3f %.3f\n",
	   		        gs->goal_id, rad, gs->xpos2, gs->ypos2, es->x, es->y); 
	   		}
        }
     
        else if(tm->heading == TRAJ_OFF && tm->depth.mode == TRACKLINE_1P)
        {
	 	    if(fabs(gs->zpos2 - es->depth) < DEPTH_BAND)
	   		{
	   		    cs->at_goal = 1;
	   		    cs->goal_completed = gs->goal_id;
	   		    debug_printf(debug->debug_mp, "MP: Completed Goal %d, depth = %.3f depth_err = %.3f\n",
		   	        gs->goal_id, es->depth, fabs( gs->zpos2 - es->depth));
	   		}
        }
     
        else if (tm->heading == MAN_HEADING && tm->transit == TRAJ_OFF)
        {
//printf("Checking manual heading goal %f vs current heading %f, delta %f\n", gs->heading, es->heading, delta_heading(gs->heading - es->heading));
            if(fabs(delta_heading(gs->heading - es->heading)) < HEADING_BAND)
            {
                cs->at_goal = 1;
                cs->goal_completed = gs->goal_id;
                debug_printf(debug->debug_mp, "MP: Completed Goal %d, heading = %.3f heading_err = %.3f\n",
                    gs->goal_id, es->heading, fabs( gs->heading - es->heading) * RTOD);
            }
        }

        else if(tm->heading != TRACKLINE_1P && tm->heading != TRACKLINE_2P && tm->depth.mode == AUTO_DEPTH)
        {
	        if(fabs(tm->depth.value - es->altitude) < DEPTH_BAND)
	        {
	            cs->at_goal = 1;
    	        cs->goal_completed = gs->goal_id;
	            debug_printf(debug->debug_mp, "MP: Completed Goal %d, Altitude = %.3f altitude_err = %.3f\n",
	    	        gs->goal_id, es->altitude, fabs( tm->depth.value - es->altitude));
            }
        }
        else
            cs->at_goal = 0;
     

    } // end of goal_complete if statement
    //else 
    //    cs->at_goal = 0;
            
    return 0;
        
}        

