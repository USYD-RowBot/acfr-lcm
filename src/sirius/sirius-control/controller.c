/*
 *  Motor control routine for Seabed AUV Sirius
 *  The routine contains the PID controllers and takes commands from
 *  the main control loop, it returns the desired thruster RPMs
 *
 *  Christian Lees
 *  ACFR
 *  5/1/12
 */
 
#include "control.h"
#include "control_utils.h"
#include "timer.h"


double ol_thrust(double speed)
{ 
    double u; 
    u = speed * 1.0;
    return(u);
}


// low level controller
// inputs are the controller config which contains the gains, the estimator output, the trajectory state and the servo mode
// it outputs prop RPMs
int controller(controller_config_t *cc, acfrlcm_auv_acfr_nav_t *es, trajectory_state_t *ts, servo_mode_t *mode, thruster_cmd_t *tc, debug_t *debug)
{
    double h_u_port, h_u_stbd;
    double t_u_port, t_u_stbd;
    double u_vert;
    
    // heading control
    if(mode->heading == ON)
    {
        double dh = delta_heading(ts->heading.reference - es->heading);
        double torque = pid(&cc->gains.heading, ts->heading.reference - dh, es->headingRate, ts->heading.reference, ts->heading.velocity, CONTROL_DT);
//        printf("href: %f h_es: %f\n", delta_heading(ts->heading.reference)*RTOD, delta_heading(es->heading)*RTOD); 
        h_u_port = torque / 2;
        h_u_stbd = -torque / 2;
    }
    else
    {
        h_u_port = 0;
        h_u_stbd = 0;
        cc->gains.heading.integral = 0;
    }
    
    // depth control
    if(mode->depth == DEPTH_ON)
        u_vert = pid(&cc->gains.depth, es->depth, es->vz, ts->depth.reference, ts->depth.velocity, CONTROL_DT);    
    else if(mode->depth == DEPTH_ALT)
        u_vert = -pid(&cc->gains.depth, es->altitude, -es->vz, ts->altitude.reference, ts->altitude.velocity, CONTROL_DT);
    else
    {
        u_vert = 0;
        cc->gains.depth.integral = 0;
    }
    
    // transit control
    if((mode->transit == TRANSIT_ON) || (mode->transit == TRANSIT_CLOOP))
    {
        if(mode->transit == TRANSIT_CLOOP)
        {
            double u_speed = pid(&cc->gains.surge, es->vx, 0.0, ts->surge.reference, 0.0, CONTROL_DT);
            t_u_port = u_speed / 2;
            t_u_stbd = u_speed / 2;
        }
        else
        {
            t_u_port = ol_thrust(ts->surge.reference);
            t_u_stbd = ol_thrust(ts->surge.reference);
            cc->gains.surge.integral = 0;
            cc->gains.sway.integral = 0;
        }
    }
    else
    {
        t_u_port = 0;
        t_u_stbd = 0;
        cc->gains.surge.integral = 0;
        cc->gains.sway.integral = 0;
    }
    
    // combine heading and transit signals
    double u_port = h_u_port + t_u_port;
    double u_stbd = h_u_stbd + t_u_stbd;
    
        
    if(u_vert < 0)
        u_vert = u_vert*cc->prop_factor;
    if(u_stbd < 0)
        u_stbd = u_stbd*cc->prop_factor;
    if(u_port < 0)
        u_port = u_port*cc->prop_factor;

    //convert what were motor current references for the 
    //original thrusters into RPM references. 
    u_stbd = TO_RPM*sgn(u_stbd)*sqrt(fabs(u_stbd));
    u_port = TO_RPM*sgn(u_port)*sqrt(fabs(u_port));
    u_vert = TO_RPM*sgn(u_vert)*sqrt(fabs(u_vert));

    // limit the max motor current limit 
    if (u_stbd > cc->max_prop_rpm)  
        u_stbd = cc->max_prop_rpm;
    if (u_stbd < cc->min_prop_rpm)  
        u_stbd = cc->min_prop_rpm; 
    if (u_port > cc->max_prop_rpm)  
        u_port = cc->max_prop_rpm; 
    if (u_port < cc->min_prop_rpm)  
        u_port = cc->min_prop_rpm; 
    if (u_vert > cc->max_prop_rpm)  
        u_vert = cc->max_prop_rpm; 
    if (u_vert < cc->min_prop_rpm)  
        u_vert = cc->min_prop_rpm; 

    // output the thurster commands
    tc->vert = u_vert; 
    tc->stbd = u_stbd;
    tc->port = u_port;
    
    return 1;
} //end controller

    
