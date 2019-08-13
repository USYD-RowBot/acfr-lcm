/*  Impletements PID controllers for use in AUVs

    Christian Lees
    ACFR
    9/11/12
*/


#include "pid.h"

// Classical PID controller, nothing fancy here, can be used as a P, PI or type A PD
// by setting the unused gains to zero
double
pid(pid_gains_t *gains, double value, double goal, double dt)
{
    double u;

    double error = goal - value;
    double derivative = (error - gains->prev_error) / dt;
    gains->integral += error * dt;
    gains->prev_error = error;


    // rail the integral
    if(gains->ki * gains->integral > gains->sat)
        gains->integral = gains->sat / gains->ki;
    else if(gains->ki * gains->integral < -gains->sat)
        gains->integral = -gains->sat / gains->ki;


    u = gains->kp * error + gains->ki * gains->integral + gains->kd * derivative;

    // rail the output
    if(u > gains->sat)
        u = gains->sat;
    else if(u < -gains->sat)
        u = -gains->sat;

    return u;
}

double
pid(pid_gains_t *gains, double value, double goal, double dt, acfrlcm::auv_base_pid_t *msg)
{
    double u;

    double error = goal - value;
    double derivative = (error - gains->prev_error) / dt;
    gains->integral += error * dt;
    gains->prev_error = error;


    // rail the integral
    if(gains->ki * gains->integral > gains->sat)
        gains->integral = gains->sat / gains->ki;
    else if(gains->ki * gains->integral < -gains->sat)
        gains->integral = -gains->sat / gains->ki;


    u = gains->kp * error + gains->ki * gains->integral + gains->kd * derivative;

    // rail the output
    if(u > gains->sat)
        u = gains->sat;
    else if(u < -gains->sat)
        u = -gains->sat;

    msg->kp = gains->kp;
    msg->ki = gains->ki;
    msg->kd = gains->kd;
    msg->integral = gains->integral;
    msg->derivative = derivative;
    msg->prev_error = gains->prev_error;
    msg->error = error;
    msg->sat = gains->sat;

    return u;
}

void
reset_pid(pid_gains_t *gains)
{
    gains->integral = 0;
    gains->prev_error = 0;
}

