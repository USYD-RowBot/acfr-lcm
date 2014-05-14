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

double delta_heading(double dh)
{ 
    double result;
    result = dh ;
    if (dh >  PI)  result -= TWOPI;
    if (dh < -PI)  result += TWOPI;
    return result;
}

double ol_thrust(double speed)
{ 
    double u; 
    u = speed * 1.0;
    return(u);
}

int init_controller(sirius_pid_gains_t *gains)
{
    gains->heading.integral = 0;
    gains->depth.integral = 0;
    gains->sway.integral = 0;
    gains->surge.integral = 0;
    
    return 1;
}
// low level controller
// inputs are the controller config which contains the gains, the estimator output, the trajectory state and the servo mode
// it outputs prop RPMs
int controller(controller_config_t *cc, acfrlcm_auv_acfr_nav_t *es, trajectory_state_t *ts, servo_mode_t *mode, thruster_cmd_t *tc)
{
    double h_u_port, h_u_strb;
    double t_u_port, t_u_strb;
    double u_vert;
    double u_lat;
    
    // heading control
    if(mode->heading == ON)
    {
        double dh = delta_heading(ts->heading.reference - es->heading);
        double torque = pid(&cc->gains.heading, ts->heading.reference - dh, es->headingRate, ts->heading.reference, ts->heading.velocity, CONTROL_DT);
        
        h_u_port = torque / 2;
        h_u_strb = -torque / 2;
    }
    else
    {
        h_u_port = 0;
        h_u_strb = 0;
        gains->heading.intergral = 0;
    }
    
    // depth control
    if(mode->depth == ON)
        u_vert = pid(&cc->gains.depth, es->depth, es->vz, ts->depth.reference, ts->depth.velocity, CONTROL_DT);    
    else if(mode->depth == ALT)
        u_vert = -pid(&cc->gains.depth, es->alt, -es->vz, ts->altitude.reference, ts->altitude.velocity, CONTROL_DT);
    else
    {
    `   u_vert = 0;
        gains->depth.integral = 0;
    }
    
    // transit control
    if((mode->transit == ON) || (mode->transit == CLOOP))
        if(mode->transit == CLOOP)
            double u_speed = pid(&cc->gains.surge, es->vx, 0.0, ts->vx, 0.0, CONTROL_DT);
            t_u_port = u_speed / 2;
            t_u_strb = u_speed / 2;
            
            u_lat = pid((&cc->gains.sway, es->vy, 0.0, ts->vy, 0.0, CONTROL_DT);
        }
        else
        {
            t_u_port = ol_thrust(ts->vx);
            t_u_strb = ol_thrust(ts->vx);
            gains->surge.intergral = 0;
            gains->sway.intergral = 0;
        }
    else
        t_u_port = 0;
        t_u_strb = 0;
        gains->surge.intergral = 0;
        gains->sway.intergral = 0;
    
    // combine heading and transit signals
    double u_port = h_u_port + t_u_port;
    double u_strb = h_u_strb + t_u_strb;
    
        
    if(u_lat < 0)
        u_lat = u_lat*cc->prop_factor;
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
    u_lat =  TO_RPM*sgn(u_lat)*sqrt(fabs(u_lat));

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
    if (u_lat  > cc->max_prop_rpm)  
        u_lat  = cc->max_prop_rpm; 
    if (u_lat  < cc->min_prop_rpm)  
        u_lat  = cc->min_prop_rpm; 

    // output the thurster commands
    tc->lat  = u_lat;
    tc->vert = u_vert; 
    tc->stbd = u_stbd;
    tc->port = u_port;
    
    return 1;
} //end controller

    
