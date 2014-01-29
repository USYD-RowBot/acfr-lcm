
#ifndef PID_H
#define PID_H

#include <math.h>

typedef struct
{
    // controller parameters
    double kp;
    double ki;
    double kd;
    double sat;
    
    // controller variables
    double integral;
} pid_gains_t;

// function declarations
extern double
pid(pid_gains_t *gains, double pos, double vel, double pos_goal, double vel_goal, double dt);


#endif // PID_H
