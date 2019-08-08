
#ifndef PID_H
#define PID_H

#include "perls-lcmtypes++/acfrlcm/auv_base_pid_t.hpp"

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
    double prev_error;
} pid_gains_t;

// function declarations
extern double
pid(pid_gains_t *gains, double value, double goal, double dt);
extern double
pid(pid_gains_t *gains, double value, double goal, double dt, acfrlcm::auv_base_pid_t *msg);
extern void
reset_pid(pid_gains_t *gains);

#endif // PID_H
