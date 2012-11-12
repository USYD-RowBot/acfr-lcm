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
    
    u = gains->kp * error + gains->ki * gains->integral + gains->kd * derivative;
    
    if(fabs(u) < gains->sat)
        gains->integral += error * dt;
    
    gains->prev_error = error;
    
    return u;
}

