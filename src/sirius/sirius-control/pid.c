/*  Impletements PID controllers for use in AUVs

    Christian Lees
    ACFR
    1/8/11
*/


#include "pid.h"

// Classical PID controller, nothing fancy here, can be used as a P, PI or type A PD
// by setting the unused gains to zero
double
pid(pid_gains_t *gains, double pos, double vel, double pos_goal, double vel_goal, double dt)
{
    double u;
    double perror = pos_goal - pos;
    double verror = vel_goal - vel;

    u = gains->kp * perror + gains->ki * gains->integral + gains->kd * verror;

    if(fabs(u) < gains->sat)
        gains->integral += perror * dt;

    return u;
}

