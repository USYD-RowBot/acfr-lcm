#include <csignal>
#include <unistd.h>

#include <iostream>

#include "controller.hpp"
#include "pid.h"

#include "acfr-common/timestamp.h"
#include "acfr-common/spektrum-control.h"

#include "perls-lcmtypes++/acfrlcm/auv_sirius_motor_command_t.hpp"

// set the delta T to 0.1s, 10Hz loop rate
//#define W_BEARING 0.95 //amount to weight the velocity bearing (slip angle) in the heading controller, to account for water currents
//#define W_HEADING 0.05 //amount to weight the heading in the heading controller


// RC constants
#define RC_OFFSET 1024
#define RC_THROTTLE_OFFSET 1024     // Testing 16092016 JJM
#define RC_HALF_RANGE 685
//#define RC_TO_RAD (12*M_PI/180)/RC_HALF_RANGE
#define RC_TO_RPM 8           
#define RC_MAX_PROP_RPM 130
#define RC_DEADZONE 80 // Testing 160902016 JJM
#define RC_HEADING_MULTI RC_MAX_PROP_RPM/(RC_HALF_RANGE-RC_DEADZONE)
#define RC_THROTTLE_MULTI RC_MAX_PROP_RPM/(RC_HALF_RANGE-RC_DEADZONE)


class SiriusController: public ControllerBase
{
public:
    SiriusController(std::string const &process_name, std::string const &vehicle_name);
    virtual ~SiriusController() = default;

protected:
    virtual void manual_control(acfrlcm::auv_spektrum_control_command_t sc);
    virtual void automatic_control(acfrlcm::auv_control_t ac, acfrlcm::auv_acfr_nav_t nav);
    virtual void dead_control();

    virtual void init();

    void reset_integrals();

private:
    pid_gains_t gains_vel;
    pid_gains_t gains_depth;
    //pid_gains_t gains_altitude;
    pid_gains_t gains_heading;
};

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

std::string
parse_args (int argc, char **argv)
{
    int opt;

    std::string vehicle_name = "DEFAULT";

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            vehicle_name = std::string((char*)optarg);
            break;
         }
    }

    return vehicle_name;
}

int main(int argc, char **argv)
{
    // super hacky!
    std::string vehicle_name = parse_args(argc, argv);
    SiriusController ic(basename(argv[0]), vehicle_name);

    ic.run();

    return 0;
}

SiriusController::SiriusController(std::string const &process_name, std::string const &vehicle_name)
    : ControllerBase(process_name, vehicle_name)
{
}

void SiriusController::init()
{
    this->gains_vel = this->get_pid("velocity");
    this->gains_depth = this->get_pid("depth");
    //this->gains_altitude = this->get_pid("altitude");
    this->gains_heading = this->get_pid("heading");
}

void SiriusController::automatic_control(acfrlcm::auv_control_t cmd, acfrlcm::auv_acfr_nav_t nav)
{
    //std::cout << "Automatic\n";
    acfrlcm::auv_sirius_motor_command_t mc;
    memset(&mc, 0, sizeof(mc));
    mc.utime = timestamp_now();

    double thrust_rpm = 0.0;
    double thrust_port = 0.0, thrust_strb = 0.0;
    double heading_rpm = 0.0;
    double heading_port = 0.0, heading_strb = 0.0;
    double combined_port = 0.0, combined_strb = 0.0;
    double vert_rpm = 0.0;

    const double dt = this->dt();
    if (cmd.run_mode == acfrlcm::auv_control_t::RUN)
    {
            /************************************************************
        * Heading calculation
        * Calculate the diff between desired heading and actual heading.
        * Ensure this diff is between +/-PI
        *************************************************************/

        // to properly do this current should be accounted for
        // so it could be moving towards the target without actually
        // facing it
        // To stop the PID doing anything stupid we'll check for a NO_VALUE heading condition here
        if(cmd.heading != cmd.heading)
 	       heading_rpm = 0;
       	else
       	{ 	
		    while (nav.heading < -M_PI)
		        nav.heading += 2 * M_PI;
		    while (nav.heading > M_PI)
		        nav.heading -= 2 * M_PI;

		    while (cmd.heading < -M_PI)
		        cmd.heading += 2 * M_PI;
		    while (cmd.heading > M_PI)
		        cmd.heading -= 2 * M_PI;

		    double diff_heading = nav.heading - cmd.heading;
		    while( diff_heading < -M_PI )
		        diff_heading += 2*M_PI;
		    while( diff_heading > M_PI )
		        diff_heading -= 2*M_PI;


		    // Account for side slip by making the velocity bearing weighted
		    // on the desired heading
		    heading_rpm = pid(&this->gains_heading, diff_heading, 0.0, dt);
	    }
	    
	    heading_port = +heading_rpm / 2;
        heading_strb = -heading_rpm / 2;
    
        // X Velocity, as above
        if(cmd.vx != cmd.vx)
        	cmd.vx = 0;
        else
	        thrust_rpm = pid(&this->gains_vel, nav.vx, cmd.vx, dt);
	        
        thrust_port = thrust_rpm / 2;
        thrust_strb = thrust_rpm / 2;
        
        // Combine the heading and thrust values
        combined_port = thrust_port + heading_port;
        combined_strb = thrust_strb + heading_strb;
        
        // Depth control
        if(cmd.depth != cmd.depth)
        	vert_rpm = 0;
    	else if ((fabs(cmd.depth - 1.5) < 0.2) && (fabs(nav.depth - 1.5) < 0.2))
    		vert_rpm = 0;
    	else
	        vert_rpm = pid(&this->gains_depth, nav.depth, cmd.depth, dt);
                
        // Clipping has been moved to the animatics controller to be the same as all the other AUVs
        
        // Set motor controller values
        mc.port = combined_port;
        mc.starboard = combined_strb;
        mc.vertical = vert_rpm;
    }

    this->lc().publish(this->get_vehicle_name() + ".THRUSTER", &mc);
}

void SiriusController::manual_control(acfrlcm::auv_spektrum_control_command_t sc)
{
    this->reset_integrals();
    acfrlcm::auv_sirius_motor_command_t mc;
    memset(&mc, 0, sizeof(mc));
    mc.utime = timestamp_now();

    // Lateral tunnel thrusters
    double port = 0.0;
    double strb = 0.0;
    double thrust = 0.0;


    port -= (double)(sc.values[RC_AILERON] - RC_OFFSET) * RC_HEADING_MULTI;
    strb += (double)(sc.values[RC_AILERON] - RC_OFFSET) * RC_HEADING_MULTI;
    int rcval = sc.values[RC_THROTTLE] - RC_THROTTLE_OFFSET;
    
    
    // if in deadzone at centre
    if (abs(rcval) < RC_DEADZONE)
        thrust = 0.0;
    else
        thrust = (double)(rcval - RC_DEADZONE) * RC_THROTTLE_MULTI;

	// Add the values together
	port += thrust;
	strb += thrust;
	
	mc.port= port;
	mc.starboard = strb;
	mc.vertical = 0.0;

    this->lc().publish(this->get_vehicle_name() + ".THRUSTER", &mc);
}

void SiriusController::dead_control()
{
    this->reset_integrals();
    acfrlcm::auv_sirius_motor_command_t mc;
    memset(&mc, 0, sizeof(mc));
    mc.utime = timestamp_now();

    mc.port = 0.0;
    mc.starboard = 0.0;
    mc.vertical = 0.0;

    this->lc().publish(this->get_vehicle_name() + ".THRUSTER", &mc);
}

void SiriusController::reset_integrals()
{
    gains_vel.integral = 0;
    gains_depth.integral = 0;
    //gains_altitude.integral = 0;
    gains_heading.integral = 0;
}
