#include <csignal>
#include <unistd.h>

#include <iostream>

#include "controller.hpp"
#include "pid.h"

#include "acfr-common/timestamp.h"
#include "acfr-common/spektrum-control.h"

#include "perls-lcmtypes++/acfrlcm/auv_nga_motor_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_control_pid_t.hpp"

// For power limiting
#include "perls-lcmtypes++/senlcm/acfr_psu_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_bluefin_tail_status_t.hpp"
#include "perls-lcmtypes++/acfrlcm/tunnel_thruster_power_t.hpp"

// set the delta T to 0.1s, 10Hz loop rate
//#define W_BEARING 0.95 //amount to weight the velocity bearing (slip angle) in the heading controller, to account for water currents
//#define W_HEADING 0.05 //amount to weight the heading in the heading controller


// RC constants
#define RC_OFFSET 1024
#define RC_THROTTLE_OFFSET 1024     // Testing 16092016 JJM
#define RC_HALF_RANGE 685.0
#define RC_TO_RAD (12*M_PI/180)/RC_HALF_RANGE //12 multipier because full rudder ROM is 24 degrees
#define RC_TO_RPM 8              // Mitch
#define RC_MAX_PROP_RPM 700.0
#define RC_MIN_PROP_RPM -700.0
#define RC_DEADZONE 80 // Testing 160902016 JJM
#define RCMULT RC_MAX_PROP_RPM/(RC_HALF_RANGE-RC_DEADZONE)
#define RC_TUNNEL_MULTI 2047/(RC_HALF_RANGE)

#define RUDDER_DELTA 0.2094*2*0.1 // assuming full 24 degrees motion in 1 second at 10 Hz

class NGAController: public ControllerBase
{
public:
    NGAController(std::string const &process_name, std::string const &vehicle_name);
    virtual ~NGAController() = default;

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
    pid_gains_t gains_tunnel_depth;
    pid_gains_t gains_tunnel_descent;
    pid_gains_t gains_tunnel_pitch;
    pid_gains_t gains_tunnel_heading;


    // Callbacks for power monitoring
    void psu_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                              const senlcm::acfr_psu_t *psu);
    void tunnel_power_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                              const acfrlcm::tunnel_thruster_power_t *ttp);
    void tail_power_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                              const acfrlcm::auv_bluefin_tail_status_t *tp);

    // power limiting values
    double psu[15];
    double tail_power;
    double tunnel_power[15];
    double total_power();
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
    NGAController ic("nga-control", vehicle_name);

    ic.run();

    return 0;
}

NGAController::NGAController(std::string const &process_name, std::string const &vehicle_name)
    : ControllerBase(process_name, vehicle_name)
{
    prev_rudder_angle = 0.0;
    prev_elev_angle = 0.0;
}

void NGAController::init()
{
    this->gains_vel = this->get_pid("velocity");
    this->gains_roll = this->get_pid("roll");
    this->gains_pitch = this->get_pid("pitch");
    this->gains_pitch_r = this->get_pid("pitch_r");
    this->gains_depth = this->get_pid("depth");
    this->gains_altitude = this->get_pid("altitude");
    this->gains_heading = this->get_pid("heading");
    this->gains_tunnel_depth = this->get_pid("tunnel_depth");
    this->gains_tunnel_descent = this->get_pid("tunnel_descent");
    this->gains_tunnel_pitch = this->get_pid("tunnel_pitch");
    this->gains_tunnel_heading = this->get_pid("tunnel_heading");
    std::string vehicle_name = this->get_vehicle_name();
    this->lc().subscribe(vehicle_name + ".PSU_.*", &NGAController::psu_callback, this);
    this->lc().subscribe(vehicle_name + ".TUNNEL_THRUSTER_POWER.*", &NGAController::tunnel_power_callback, this);
    this->lc().subscribe(vehicle_name + ".BLUEFIN_STATUS", &NGAController::tail_power_callback, this);
    
    for(int i=0; i<15; i++)
    {	
        this->tunnel_power[i] = 0.0;
	this->psu[i] = 0.0;
    }
    
    this->tail_power = 0;
}

void NGAController::automatic_control(acfrlcm::auv_control_t cmd, acfrlcm::auv_acfr_nav_t nav)
{
	std::cout << "Total Power = " << this->total_power() << std::endl; 
    std::cout << "Automatic\n";
    acfrlcm::auv_nga_motor_command_t mc;
    acfrlcm::auv_control_pid_t cp;
    memset(&cp, 0, sizeof(cp));
    memset(&mc, 0, sizeof(mc));
    cp.utime = mc.utime = timestamp_now();

    bool thruster_flow_dependant = false, elevator_disabled = false;
    double prop_rpm = 0.0;
    double pitch = 0.0, plane_angle = 0.0, rudder_angle = 0.0;

    const double dt = this->dt();
    if (cmd.run_mode == acfrlcm::auv_control_t::RUN)// ||
        //state->run_mode == acfrlcm::auv_control_t::DIVE)
    {
        // X Velocity
        prop_rpm = pid(&this->gains_vel, nav.vx, cmd.vx, dt, &cp.velocity);

        if((fabs(prop_rpm - prev_rpm) < 20))
            prev_rpm = prop_rpm;
        else{
            prop_rpm = prev_rpm + fabs(prop_rpm - prev_rpm)/(prop_rpm - prev_rpm)*20;
            prev_rpm = prop_rpm;
        }
        
        //limit reverse in auto control also
        if (prop_rpm < RC_MIN_PROP_RPM)
            prop_rpm = RC_MIN_PROP_RPM;

        //std::cout << "Velocity: rpm= " << prop_rpm << ", curr= "<< nav.vx << ", des= " << cmd.vx << std::endl;
        /************************************************************
        * Depth calculation
        * Diving with NGA is a little special. we have tunnel thrusters
        * to reach a target depth
        *************************************************************/
        double target_pitch = 0.0;

        ///double target_descent = pid(&this->gains_tunnel_depth, nav.depth, cmd.depth, dt);

        std::cout << "nav depth: " << nav.depth << " cmd depth: " << cmd.depth <<  std::endl;
        // if ((fabs(nav.depth) > 0.5)&&(fabs(nav.depth - cmd.depth) < 1.5)) {
            // printf("pid calc\n");
        if (cmd.depth_mode == acfrlcm::auv_control_t::PITCH_MODE)
            {
                pitch = cmd.pitch;
                std::cout << "PITCH_MODE" << std::endl; 
            }
        else if (cmd.depth_mode == acfrlcm::auv_control_t::ALTITUDE_MODE)
            {
                //pitch = pid(&this->gains_altitude, nav.altitude, cmd.altitude, dt, &cp.altitude);
                pitch = -pid(&this->gains_depth, nav.depth, cmd.depth, dt, &cp.altitude);
                std::cout << "ALTITUDE_MODE" << std::endl;         
            }
        else
            {
                pitch = -pid(&this->gains_depth, nav.depth, cmd.depth, dt, &cp.depth);
                std::cout << "DEPTH_MODE" << std::endl; 
            }

        if ((nav.vx > -0.05) || (prop_rpm > -100))
            plane_angle = pid(&this->gains_pitch, nav.pitch, pitch, dt, &cp.pitch);
        else
            plane_angle = pid(&this->gains_pitch_r, nav.pitch, pitch, dt, &cp.pitch_r);

        //std::cout << "pitch: " << pitch/ M_PI * 180 << " nav pitch: " << nav.pitch/ M_PI * 180 << " plane_angle: " << plane_angle/ M_PI * 180 << std::endl;

        if((fabs(plane_angle - prev_elev_angle) < RUDDER_DELTA))
            prev_elev_angle = plane_angle;
        else{
            plane_angle = prev_elev_angle + (fabs(plane_angle - prev_elev_angle)/(plane_angle - prev_elev_angle))*RUDDER_DELTA;
            prev_elev_angle = plane_angle;
        }
    // }
    // else{
    //     reset_pid(&this->gains_depth);
    //     reset_pid(&this->gains_pitch);
    // }

        double differential_vert = pid(&this->gains_tunnel_pitch,
                nav.pitch, target_pitch, dt, &cp.tunnel_pitch);

        double transitional_diff_vert = pid(&this->gains_tunnel_pitch,
                nav.pitch, cmd.pitch, dt, &cp.tunnel_pitch);

        double mutual_vert = pid(&this->gains_tunnel_descent,
                nav.depth, cmd.depth, dt, &cp.tunnel_descent);

        // testing variable pitch rpm based of mutual value saturation
        double mutual_percent = abs(mutual_vert/gains_tunnel_descent.sat);
        double differential_vert_corrected = differential_vert *(cos(mutual_percent) - sin(mutual_percent)/2);

        // Set motor controller values
        mc.vert_fore = (mutual_vert - differential_vert_corrected);  
        mc.vert_aft = (mutual_vert + differential_vert_corrected);

    	//std::cout << "Vertical control mutual: " << mutual_vert << std::endl;
        //std::cout << " diff: " << differential_vert << " c_diff: " << differential_vert_corrected << std::endl;

        /************************************************************
        * Heading calculation
        * Calculate the diff between desired heading and actual heading.
        * Ensure this diff is between +/-PI
        *************************************************************/

        // to properly do this current should be accounted for
        // so it could be moving towards the target without actually
        // facing it

        nav.heading = fmod((nav.heading + M_PI),(2*M_PI));
        if (nav.heading < 0.0)
            nav.heading += 2*M_PI;
        nav.heading -= M_PI;

        cmd.heading = fmod((cmd.heading + M_PI),(2*M_PI));
        if (cmd.heading < 0.0)
            cmd.heading += 2*M_PI;
        cmd.heading -= M_PI;

        double diff_heading = fmod((nav.heading - cmd.heading + M_PI),(2*M_PI));
        if (diff_heading < 0.0)
            diff_heading += 2*M_PI;
        diff_heading -= M_PI;
        // Account for side slip by making the velocity bearing weighted
        // 	on the desired heading
        rudder_angle = pid(&this->gains_heading, diff_heading, 0.0, dt, &cp.heading);

        // std::cout << "nav heading: " << nav.heading/ M_PI * 180 << " cmd heading: " << cmd.heading/ M_PI * 180 << " diff heading: " << diff_heading/ M_PI * 180 << " rudder_angle: " << rudder_angle/ M_PI * 180 << std::endl;

        // checks for impossible rudder motions
        if((fabs(rudder_angle - prev_rudder_angle) < RUDDER_DELTA))
            prev_rudder_angle = rudder_angle;
        else{
            rudder_angle = prev_rudder_angle + (fabs(rudder_angle - prev_rudder_angle)/(rudder_angle - prev_rudder_angle))*RUDDER_DELTA;
            prev_rudder_angle = rudder_angle;
        }

        //std::cout <<"rudder_angle: " << rudder_angle/ M_PI * 180 << " prev rudder_angle: " << prev_rudder_angle/ M_PI * 180 << std::endl;

        double differential_lat = pid(&this->gains_tunnel_heading, diff_heading, 0, dt, &cp.tunnel_heading);

    	//std::cout << "Diff lat: " << differential_lat << std::endl;
        mc.lat_fore = differential_lat;
        mc.lat_aft = -differential_lat;

        // FIXME: Might consider adding some lat tunnel thruster here
        
        //	printf("prop_rpm: %f\n",prop_rpm);
        // Reverse all the fin angles for reverse direction (given rpm is
        // 	negative and so is velocity, so water relative should be
        //	negative, or soon will be). May not be enough due to completely
        //	different dynamics in reverse, hence there are new gains for the
        //	reverse pitch control now.
        if ((nav.vx < -0.05) && (prop_rpm < -100))
        {
            printf("reversing, flipping fin control\n");
            rudder_angle       = -rudder_angle;
            plane_angle      = -plane_angle;
        }

        // Set motor controller values
        mc.tail_thruster = prop_rpm;
        mc.tail_rudder = rudder_angle;
        if (elevator_disabled || (fabs(cmd.depth) <1e-3))//(fabs(nav.depth - cmd.depth) > 1.5))
            mc.tail_elevator = 0.0; // mc.tail_elevator = pid(&this->gains_pitch, nav.pitch, (5*M_PI)/180, dt); //target pitch of 5 degrees to make tail more efficient
        else
            mc.tail_elevator = plane_angle;

        double threshold = M_PI/18;
        double bias = 1.0;
        double dive_goal_threshold = 1.5; //m
        double tail_goal_threshold = 0.5; //m
        double distance_to_depth_goal = fabs(nav.depth - cmd.depth);
        double transition_percentage = (distance_to_depth_goal-tail_goal_threshold)/(dive_goal_threshold-tail_goal_threshold);
        int tail_transition_value = 200;
        int tunnel_transition_value = 1000;
        int heading_correction_limit = 500;
        // power management
        // dive with thrusters if large dive
        if (distance_to_depth_goal > dive_goal_threshold   || cmd.depth < 0){
            if(fabs(mc.lat_fore) > heading_correction_limit) //fabs this
		      mc.lat_fore = (heading_correction_limit*fabs(mc.lat_fore))/(mc.lat_fore);
            if(fabs(mc.lat_aft) > heading_correction_limit) //fabs this
                mc.lat_fore = (heading_correction_limit*fabs(mc.lat_fore))/(mc.lat_fore);
            mc.tail_thruster = 1.0; // don't trigger the idle reset on tail during a dive
            mc.tail_elevator = 0.0;
            mc.tail_rudder = 0.0;
            // std::cout << " vert thrusters only";
        }
        else if (distance_to_depth_goal <= dive_goal_threshold && distance_to_depth_goal > tail_goal_threshold){
            // differential_vert = transitional_diff_vert;
            // double differential_vert_corrected = differential_vert *(cos(mutual_percent) - sin(mutual_percent)/2);
            // mc.vert_fore = (mutual_vert - differential_vert_corrected);  
            // mc.vert_aft = (mutual_vert + differential_vert_corrected);
            mc.lat_fore = 0.0;
            mc.lat_aft = 0.0;
            // limit floor value of tail to 200 RPM when in transition zone
            mc.tail_thruster = mc.tail_thruster*(1-transition_percentage);
            if(mc.tail_thruster < tail_transition_value && mc.tail_thruster/(1-transition_percentage) > tail_transition_value)
                mc.tail_thruster = tail_transition_value;
            // limit ceiling value of tunnels to 1000 mean value for fore and aft
            mc.vert_fore = transition_percentage*mc.vert_fore;
            if (mc.vert_fore < tunnel_transition_value && mc.vert_fore/transition_percentage > tunnel_transition_value)
                mc.vert_fore = tunnel_transition_value - differential_vert_corrected; //added in the differential values for pitch control during transition
            mc.vert_aft = transition_percentage*mc.vert_aft;
            if (mc.vert_aft < tunnel_transition_value && mc.vert_aft/transition_percentage > tunnel_transition_value)
                mc.vert_aft = tunnel_transition_value + differential_vert_corrected;
        }
        //turn on spot if large turn angle
        else if (fabs(diff_heading) > M_PI/6){
            mc.vert_fore = 0.0;
            mc.vert_aft = 0.0;
	        mc.tail_thruster = 0.0;
            // std::cout << "tunnel turning";
            // adding lat tunnel efficiency code here for tests
            if (thruster_flow_dependant)
            {
                if (nav.heading < -threshold && diff_heading > threshold){
                    mc.lat_fore = bias*mc.lat_fore;
                    mc.lat_aft = 0.0;
                    // std::cout << " lat aft thruster off";
                } 
                else if (nav.heading < -threshold && diff_heading < -threshold){
                    mc.lat_fore = 0.0;
                    mc.lat_aft = bias*mc.lat_aft;
                    // std::cout << " lat fore thruster off";
                } 
                else if (nav.heading > threshold && diff_heading > threshold){
                    mc.lat_fore = 0.0;
                    mc.lat_aft = bias*mc.lat_aft;
                    // std::cout << " lat fore thruster off";
                } 
                else if (nav.heading > threshold && diff_heading < -threshold){
                    mc.lat_fore = bias*mc.lat_fore;
                    mc.lat_aft = 0.0;
                    // std::cout << " lat aft thruster off";
                } 
            }
        }
        else {
            //all else use tail thruster plus elevator and rudder
            mc.vert_fore = 0.0;
            mc.vert_aft = 0.0;
            mc.lat_fore = 0.0;
            mc.lat_aft = 0.0;
            // std::cout << " tail thruster only";
        }
        // std::cout << " " << std::endl;
    }
    // safety hard codes
    //mc.tail_elevator = 0.0;
    this->lc().publish(this->get_vehicle_name() + ".NEXTGEN_MOTOR", &mc);
    this->lc().publish(this->get_vehicle_name() + ".PID_TUNING", &cp);
}

void NGAController::manual_control(acfrlcm::auv_spektrum_control_command_t sc)
{
    std::cout << "Total Power = " << this->total_power() << std::endl; 
    this->reset_integrals();
    acfrlcm::auv_nga_motor_command_t mc;
    memset(&mc, 0, sizeof(mc));

    mc.utime = timestamp_now();

    // Lateral tunnel thrusters
    int fore = 0;
    int aft = 0;
    double rudder, elevator_hc, prop_rpm;

    //Strafe - this is side to side on left stick - always available
    fore = (sc.values[RC_RUDDER] - RC_OFFSET) * 1500/RC_HALF_RANGE;
    aft = fore;
        
    // Check the steering mode switch - top switch on right side
    if(sc.values[RC_GEAR] > REAR_POS_CUTOFF)
    {

        //tunnel turning when switch is back
        fore -= (sc.values[RC_AILERON] - RC_OFFSET) * 1500/RC_HALF_RANGE;
        aft += (sc.values[RC_AILERON] - RC_OFFSET) * 1500/RC_HALF_RANGE;
        if (fore > 1500)
            fore = 1500;
        if (aft > 1500)
            aft = 1500;
        rudder = 0;
    }
    else 
    {
        //or rudder turning when switch is forward
        rudder = -(sc.values[RC_AILERON] - RC_OFFSET) * RC_TO_RAD;
    }
    if((fabs(rudder - prev_rudder_angle) < RUDDER_DELTA))
        prev_rudder_angle = rudder;
    else{
        rudder = prev_rudder_angle + (fabs(rudder - prev_rudder_angle)/(rudder - prev_rudder_angle))*RUDDER_DELTA;
        prev_rudder_angle = rudder;
    }
    elevator_hc = -(sc.values[RC_ELEVATOR] - RC_OFFSET) * RC_TO_RAD;
    if((fabs(elevator_hc - prev_elev_angle) < RUDDER_DELTA))
            prev_elev_angle = elevator_hc;
    else{
        elevator_hc = prev_elev_angle + (fabs(elevator_hc - prev_elev_angle)/(elevator_hc - prev_elev_angle))*RUDDER_DELTA;
        prev_elev_angle = elevator_hc;
    }
    // elevator is always available on right stick forward and back
    mc.tail_elevator = elevator_hc;
    mc.tail_rudder = rudder; 
    mc.lat_aft = aft;
    mc.lat_fore = fore; 
    
    
    int rcval = sc.values[RC_THROTTLE] - RC_THROTTLE_OFFSET;
        
    // if in deadzone at centre
    if (abs(rcval) < RC_DEADZONE)
    {
        prop_rpm = 0;
    }
    else
    {
        prop_rpm = (rcval)* RC_MAX_PROP_RPM/RC_HALF_RANGE;
        if (prop_rpm > RC_MAX_PROP_RPM)
            prop_rpm = RC_MAX_PROP_RPM;
        if (prop_rpm < RC_MIN_PROP_RPM)
            prop_rpm = RC_MIN_PROP_RPM;
    }
    if((fabs(prop_rpm - prev_rpm) < 20))
        prev_rpm = prop_rpm;
    else{
        prop_rpm = prev_rpm + fabs(prop_rpm - prev_rpm)/(prop_rpm - prev_rpm)*20;
        prev_rpm = prop_rpm;
    }
    mc.tail_thruster = prop_rpm;

    //brown out limiting
    if (abs(rcval) > RC_DEADZONE)
    {
        // if we have a tail rpm then don't run lat tunnels at same time, but allow elevator and rudder commands
        // if no tail rpm then look for tunnel rpm, can no longer strafe and diff steer at same time. Steer has priority
        mc.lat_fore = 0.0;
        mc.lat_aft = 0.0;
    }

    this->lc().publish(this->get_vehicle_name() + ".NEXTGEN_MOTOR", &mc);
}

void NGAController::dead_control()
{
	std::cout << "Total Power = " << this->total_power() << std::endl; 
    this->reset_integrals();
    prev_rudder_angle=0;
    prev_elev_angle=0;
    prev_rpm=0;
    acfrlcm::auv_nga_motor_command_t rc;
    memset(&rc, 0, sizeof(rc));
    rc.utime = timestamp_now();

    rc.tail_thruster = 0;
    rc.tail_elevator = 0;
    rc.tail_rudder = 0;

    rc.lat_aft = 0;
    rc.lat_fore = 0;

    rc.vert_fore = 0;
    rc.vert_aft = 0;

    this->lc().publish(this->get_vehicle_name() + ".NEXTGEN_MOTOR", &rc);
}

void NGAController::reset_integrals()
{
    gains_vel.integral = 0;
    gains_roll.integral = 0;
    gains_depth.integral = 0;
    gains_altitude.integral = 0;
    gains_pitch.integral = 0;
    gains_pitch_r.integral = 0;
    gains_heading.integral = 0;
}


void NGAController::psu_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                              const senlcm::acfr_psu_t *psu)
{
    this->psu[psu->address] = psu->voltage * psu->current;
}

void NGAController::tunnel_power_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                              const acfrlcm::tunnel_thruster_power_t *ttp)
{
    this->tunnel_power[ttp->addr] = (ttp->voltage[0] * ttp->current[0]) + (ttp->voltage[1] * ttp->current[1]);
}

void NGAController::tail_power_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                              const acfrlcm::auv_bluefin_tail_status_t *tp)
{
    this->tail_power = tp->voltage * tp->current;
}

double NGAController::total_power()
{
    double tp = 0.0;
    for(int i=0; i<15; i++)
    {	
        tp += this->psu[i];
        tp += this->tunnel_power[i];
    }
    tp += this->tail_power;

    return tp;
}
