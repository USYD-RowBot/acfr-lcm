
#ifndef CONTROL_UTILS_H
#define CONTROL_UTILS_H

#include "control.h"

extern double sgn(double x);
extern double delta_heading(double dh);
extern void wrap_heading(double *h);
extern double norm(double x, double y);
extern double comp_to_cart(double x);
extern double cart_to_comp(double x);
extern int rovtime2dsltime_str(double rov_time, char *str);
extern int check_mp_goals(ctl_status_t *cs, goal_setpoint_t *gs, trajectory_mode_t *tm, acfrlcm_auv_acfr_nav_t *es, debug_t *debug);

#endif
