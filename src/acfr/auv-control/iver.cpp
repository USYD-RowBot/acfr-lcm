#include <csignal>

#include "controller.hpp"
#include "pid.h"

#include "acfr-common/timestamp.h"
#include "acfr-common/spektrum-control.h"

#include "perls-lcmtypes++/acfrlcm/auv_iver_motor_command_t.hpp"

#define RC_OFFSET 505
#define RC_THROTTLE_OFFSET 493      // Testing 16092016 JJM
#define RC_TO_RAD (0.071875*M_PI/180)*2
#define RC_TO_RPM 8              // Mitch
#define RC_MAX_PROP_RPM 1500
//#define RC_DEADZONE 40 // Testing 160902016 JJM // this is defined in the spektrum header
#define RCMULT 4.8 //RC_MAX_PROP_RPM/(RC_HALF_INPUT_RANGE-RC_DEADZONE)

class IverController: public ControllerBase
{
public:
    IverController(std::string const &process_name, std::string const &vehicle_name);
    virtual ~IverController() = default;

protected:
    virtual void manual_control(acfrlcm::auv_spektrum_control_command_t sc);
    virtual void automatic_control(acfrlcm::auv_control_t ac, acfrlcm::auv_acfr_nav_t nav);
    virtual void dead_control();

    virtual void init();

    void reset_integrals();

private:
    pid_gains_t gains_vel;
    pid_gains_t gains_roll;
    pid_gains_t gains_depth;
    pid_gains_t gains_altitude;
    pid_gains_t gains_pitch;
    pid_gains_t gains_pitch_r;
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
}

int main(int argc, char **argv)
{
    std::string vehicle_name = parse_args(argc, argv);
    IverController ic("iver-control", vehicle_name);

    ic.run();

    return 0;
}

IverController::IverController(std::string const &process_name, std::string const &vehicle_name)
    : ControllerBase(process_name, vehicle_name)
{
}

void IverController::init()
{
    this->gains_vel = this->get_pid("velocity");
    this->gains_roll = this->get_pid("roll");
    this->gains_pitch = this->get_pid("pitch");
    this->gains_pitch_r = this->get_pid("pitch_r");
    this->gains_depth = this->get_pid("depth");
    this->gains_altitude = this->get_pid("altitude");
    this->gains_heading = this->get_pid("heading");
}

void IverController::automatic_control(acfrlcm::auv_control_t cmd, acfrlcm::auv_acfr_nav_t nav)
{
    double prop_rpm = 0.0;
    double roll_offset = 0.0;
    double pitch = 0.0, plane_angle = 0.0, rudder_angle = 0.0;

    double dt = this->dt();

    prop_rpm = pid(&this->gains_vel, nav.vx, cmd.vx, dt);

    // Pitch to fins
    if (cmd.depth_mode == acfrlcm::auv_control_t::PITCH_MODE)
    {
        pitch = cmd.pitch;
    }
    // Altitude to pitch
    // Invert sign of pitch reference to reflect pitch
    // orientation
    else if (cmd.depth_mode == acfrlcm::auv_control_t::ALTITUDE_MODE)
        pitch = -pid(&this->gains_altitude, nav.altitude, cmd.altitude,
                        dt);
    // Depth to pitch mode
    else
        pitch = -pid(&this->gains_depth, nav.depth, cmd.depth,
                        dt);



    if ((nav.vx > -0.05) || (prop_rpm > -100))
        plane_angle = pid(&this->gains_pitch, nav.pitch, pitch,
                            dt);
    else
        plane_angle = pid(&this->gains_pitch_r, nav.pitch, pitch,
                            dt);

    /*
        * Heading calculation
        * Calculate the diff between desired heading and actual heading.
        * Ensure this diff is between +/-PI
        */
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

    if (cmd.run_mode == acfrlcm::auv_control_t::DIVE)
    {
        rudder_angle = 0;
        roll_offset = 0;
    }
    else
    {

        // Account for side slip by making the velocity bearing weighted
        // 	on the desired heading
        rudder_angle = pid(&this->gains_heading, diff_heading, 0.0, dt);

        // Roll compenstation
        // We try to keep the AUV level, ie roll = 0
        roll_offset = pid(&this->gains_roll, nav.roll, 0.0, dt);
    }

    // Add in the roll offset
    double top       = rudder_angle - roll_offset;
    double bottom    = rudder_angle + roll_offset;
    double port      = plane_angle  - roll_offset;
    double starboard = plane_angle  + roll_offset;

    //	printf("prop_rpm: %f\n",prop_rpm);
    // Reverse all the fin angles for reverse direction (given rpm is
    // 	negative and so is velocity, so water relative should be
    //	negative, or soon will be). May not be enough due to completely
    //	different dynamics in reverse, hence there are new gains for the
    //	reverse pitch control now.
    if ((nav.vx < -0.05) && (prop_rpm < -100))
    {
        printf("reversing, flipping fin control\n");
        top       = -top;
        bottom    = -bottom;
        port      = -port;
        starboard = -starboard;
    }

    //printf("hnav:%f, hcmd:%f, rangle:%f t:%.1f b:%.1f p:%.1f s:%.1f\n",
    // state->nav.heading, state->command.heading, rudder_angle, top, bottom, port, starboard);

    // Set motor controller values
    acfrlcm::auv_iver_motor_command_t mc;
    mc.main = prop_rpm;
    mc.top = top;
    mc.bottom = bottom;
    mc.port = port;
    mc.starboard = starboard;
    
    this->lc().publish(this->get_vehicle_name() + ".IVER_MOTOR", &mc);

    // Print out and publish IVER_MOTOR.TOP status message every 10 loops
    /*if( loopCount % 10 == 0 )
    {
        this->lc().publish(this->get_vehicle_name() + ".IVER_MOTOR.TOP", &mc);
        printf( "Velocity: curr=%2.2f, des=%2.2f, diff=%2.2f\n",
                nav.vx, cmd.vx, (cmd.vx - nav.vx) );
        printf( "Heading : curr=%3.2f, des=%3.2f, diff=%3.2f\n",
                nav.heading/M_PI*180, cmd.heading/M_PI*180, diff_heading/M_PI*180 );
        printf( "Pitch : curr=%3.2f, des=%3.2f, diff=%3.2f\n",
                nav.pitch/M_PI*180, pitch/M_PI*180, (pitch - nav.pitch)/M_PI*180 );
        printf( "Roll: curr=%3.2f, des=%3.2f, diff=%3.2f offset: %3.2f\n",
                nav.roll/M_PI*180, 0.0, -nav.roll/M_PI*180, roll_offset/M_PI*180 );
        printf( "Motor   : main=%4d\n", (int)mc.main);
        printf( "Fins    : top=%.2f, bot=%.2f, port=%.2f star=%.2f\n",
                mc.top, mc.bottom, mc.port, mc.starboard);
        printf( "\n" );
    }*/
}

void IverController::manual_control(acfrlcm::auv_spektrum_control_command_t sc)
{
    this->reset_integrals();
    acfrlcm::auv_iver_motor_command_t rc;
    rc.utime = timestamp_now();
    rc.top = -(sc.values[RC_AILERON] - RC_OFFSET) * RC_TO_RAD;
    rc.bottom = rc.top;
    rc.port = -(sc.values[RC_ELEVATOR] - RC_OFFSET) * RC_TO_RAD;
    rc.starboard = rc.port;
    
    int16_t rcval = sc.values[RC_THROTTLE] - RC_THROTTLE_OFFSET;

    // if in deadzone at centre
    if (abs(rcval) < RC_DEADZONE)
    {
        rc.main = 0;
    }
    else
    {
       rc.main = (rcval - RC_DEADZONE) * RCMULT;
        if (rc.main > RC_MAX_PROP_RPM)
            rc.main = RC_MAX_PROP_RPM;
        else if (rc.main < -RC_MAX_PROP_RPM)
            rc.main = -RC_MAX_PROP_RPM;
    }        

    rc.source = acfrlcm::auv_iver_motor_command_t::REMOTE;
    this->lc().publish(this->get_vehicle_name() + ".IVER_MOTOR", &rc);
}

void IverController::dead_control()
{
    this->reset_integrals();
    acfrlcm::auv_iver_motor_command_t rc;
    rc.utime = timestamp_now();
    rc.top = 0;
    rc.bottom = 0;
    rc.port = 0;
    rc.starboard = 0;
    rc.main = 0;

    rc.source = acfrlcm::auv_iver_motor_command_t::REMOTE;
    this->lc().publish(this->get_vehicle_name() + ".IVER_MOTOR", &rc);
}

void IverController::reset_integrals()
{
    gains_vel.integral = 0;
    gains_roll.integral = 0;
    gains_depth.integral = 0;
    gains_altitude.integral = 0;
    gains_pitch.integral = 0;
    gains_pitch_r.integral = 0;
    gains_heading.integral = 0;
}
