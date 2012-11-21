/*
 *  Local trajectory planner
 * 
 *  Takes in either one or two way points and sends the AUV along the
 *  line.  It is the responsibility of the module above to determine
 *  if we have reached out goal.
 *
 *  Christian Lees
 *  Navid Nourani-Vatani
 *
 *  ACFR
 *  21/11/12
 */

#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <bot_param/param_client.h>
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_iver_motor_command_t.hpp"
