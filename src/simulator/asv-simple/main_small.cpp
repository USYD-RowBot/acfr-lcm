// ASV Simple simulator


#include <iostream>
#include <fstream>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <boost/numeric/odeint.hpp>
#include <small/Pose3D.hh>
#include <bot_param/param_client.h>
#include "acfr-common/auv_map_projection.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/asv_torqeedo_motor_command_t.hpp"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/senlcm/tcm_t.hpp"
#include "perls-lcmtypes++/senlcm/ysi_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/rdi_pd5_t.hpp"
#include "perls-lcmtypes++/senlcm/IMU_t.hpp"


using namespace std;
using namespace boost::numeric::odeint;
using namespace acfrlcm;
using namespace perllcm;
using namespace senlcm;
using namespace libplankton;

#define Kdp 0.01
#define CURRENT 0.1
#define WATER_DEPTH 30

#define GPS_CHANNELS 4
#define MAG_VAR 12.67*M_PI/180
#define TAU_A 500
#define TAU_G 500
#define STD_A 0.0372
#define STD_G 0.00010732
#define BIAS_A 0.0196
#define BIAS_G 9.6963e-06

#define ASV_WIDTH 2.5   // m (~8')
#define ASV_LENGTH 5.0  // m (~16')
#define ASV_MASS 200    // kg
#define ASV_INERTIA ASV_MASS*ASV_LENGTH*ASV_LENGTH/12 // Eq'n 4.14 of Physics of Sailing

typedef boost::numeric::ublas::vector< double > state_type;

// state vectors
state_type state(12);
runge_kutta_cash_karp54<state_type> stepper;

SMALL::Vector3D accel;

ofstream fp, fp_nav;
Local_WGS84_TM_Projection *map_projection_sim;

int64_t last_print_time = 4.611686e+18; // 2^62
int64_t last_obs_time = 4.611686e+18;

// the state vector is X Y h v w
#define XNDX 0
#define YNDX 1
#define PSINDX 2
#define VELNDX 3
#define PSIDOTNDX 4
// the control vector is RPM_port, RPM_std 
double in[2];
#define PORTNDX 0
#define STBDNDX 1

string vehicle_name = "DEFAULT";

void asv( const state_type &x , state_type &dxdt , const double /* t */ )
{
    double psi;
    double v, w;
    //Xa = x[XNDX];
    //Ya = x[YNDX];
    psi = x[PSINDX];
    v = x[VELNDX];
    w = x[PSIDOTNDX];
    
    // couple the velocity of the two pontoons to the velocity and rotation rate of the vessel
    double v_port = v + w*ASV_WIDTH/2; 
    double v_stbd = v - w*ASV_WIDTH/2;
/* for now ignore currents 
    // adjustments for a constant water current (or wind)

    SMALL::Rotation3D Cbn;
    Cbn.setRollPitchYawRad(phi, theta, psi);

    // current in the navigation frame
    SMALL::Vector3D vc_n;
    vc_n = CURRENT,0,0;
    // current in the body frame
    SMALL::Vector3D vc_b;
    vc_b = (Cbn.i() * vc_n);

    // vehicle velocity in the current
    nuLinear(1) = nuLinear(1) - vc_b(1);
    nuLinear(2) = nuLinear(2) - vc_b(2);
    nuLinear(3) = nuLinear(3) - vc_b(3);

    u = u - vc_b(1);
    v = v - vc_b(2);
    w = w - vc_b(3);
*/

    // make heading 0 to 2pi
    while(psi < 0)
        psi += 2 * M_PI;
    while(psi > (2 * M_PI))
        psi -= 2 * M_PI;

    // Where we are at, ie the asv pose
    //SMALL::Pose2D asvPose;
    //asvPose.set(Xa, Ya, psi);

    // Prop force calculations
    //double prop_alpha = 0.02290; //0.01; // advance ratio
    double prop_diameter = 0.3;
    double rho = 1030;              // Water density
    double J0 = 0; // open water advance coefficient
    double Kt = 0; // propeller torque coefficient
    
   
    // prop force parameters as per Fossen
    double alpha1 = 0.5;
    double alpha2 = -4.0/11.0;
    double alpha3 = (0.45 - alpha1)/(-0.2);
    double alpha4 = 0.45 -(-0.2* (0.95-0.45) / (-0.5-(-0.2)) );
    double alpha5 = (0.95-0.45) / (-0.5-(-0.2));
    double omega = 0.1;

    // ***************************
    // Prop force on the port side
    // ***************************
    // Sanity check on input
    if(in[PORTNDX] != in[PORTNDX])
    {
        in[PORTNDX] = 0;
        cerr << "In 0 NaN" << endl;
    }

    // propeller revolutions per second (rps)
    // converting desired rpm to rps
    double n_port = in[PORTNDX]/60;
    
    // limit the max rpm to 1300 ~= 21 rps
    if(fabs(n_port) > 21)
        n_port = n_port / fabs(n_port) * 21;
    
    // advance velocity as per Fossen eq 4.6
    double Va_port = (1 - omega) * v_port;
    
    if(fabs(n_port) > 1e-3)
        J0 = Va_port / (n_port * prop_diameter);      // as per Fossen eq 6.107

 
    if (J0 > 0)
        Kt = alpha1 + alpha2 * J0;   // Fossen eq 6.113
    else if (J0 > -0.2)
        Kt = alpha1 + alpha3 * J0;			// Fossen eq 6.113
    else
        Kt = alpha4 + alpha5 * J0; // Fossen eq 6.113

    double prop_force_port;
    prop_force_port = rho * pow(prop_diameter,4) * Kt * fabs(n_port) * n_port;     // As per Fossen eq 4.2
    cout << "n_port = " << n_port << " Va_port = " << Va_port << " J0 = " << J0 << " Kt = " << Kt << " F_port = " << prop_force_port << endl;
    if(prop_force_port !=  prop_force_port)
    {
        cerr << "Prop force error" << endl;
        prop_force_port = 0;
    }
    //if(fabs(prop_force_port) > 10.0)
    //    prop_force_port = prop_force_port / fabs(prop_force_port) * 10;

    // ********************************
    // Prop force on the starboard side
    // ********************************
    // Sanity check on input
    if(in[STBDNDX] != in[STBDNDX])
    {
        in[STBDNDX] = 0;
        cerr << "In 0 NaN" << endl;
    }
    
    // propeller revolutions per second (rps)
    // converting desired rpm to rps
    double n_stbd = in[STBDNDX]/60;
    
    // limit the max rpm to 1300 = 21 rps
    if(fabs(n_stbd) > 21)
        n_stbd = n_stbd / fabs(n_stbd) * 21;
    
    // advance velocity as per Fossen eq 4.6
    double Va_stbd = (1 - omega) * v_stbd;
    
    if(fabs(n_stbd) > 1e-3)
        J0 = Va_stbd / (n_stbd * prop_diameter);      // as per Fossen eq 6.107

    if (J0 > 0)
        Kt = alpha1 + alpha2 * J0;   // Fossen eq 6.113
    else if (J0 > -0.2)
        Kt = alpha1 + alpha3 * J0;			// Fossen eq 6.113
    else
        Kt = alpha4 + alpha5 * J0; // Fossen eq 6.113

    double prop_force_stbd;
    prop_force_stbd = rho * pow(prop_diameter,4) * Kt * fabs(n_stbd) * n_stbd;     // As per Fossen eq 4.2

    cout << "n_stbd = " << n_stbd << " Va_stbd = " << Va_stbd << " J0 = " << J0 << " Kt = " << Kt << " F_stbd = " << prop_force_stbd << endl;
    if(prop_force_stbd !=  prop_force_stbd)
    {
        cerr << "Prop force error" << endl;
        prop_force_stbd = 0;
    }
    //if(fabs(prop_force_stbd) > 10.0)
    //    prop_force_stbd = prop_force_stbd / fabs(prop_force_stbd) * 10;
    
    // ***********
    // Drag forces
    // ***********
    double Cd = 0.82; // modelled on a long cylinder
    double r_cyl = 0.3; // model cylinder as 30cm diameter
    double A = 0.5 * M_PI * pow(r_cyl,2); // half submerged

    double Fd_port = 0.5 * rho * A * Cd * fabs(v_port) * v_port ;
    double Fd_stbd = 0.5 * rho * A * Cd * fabs(v_stbd) * v_stbd ;

    // external applied forces
    double F_port, F_stbd;
    F_port = prop_force_port - Fd_port;
    F_stbd = prop_force_stbd - Fd_stbd;

    cout << "Fp_port: " << prop_force_port << " Fp_stbd:" << prop_force_stbd << " Fd_port:" << Fd_port << " Fd_stbd:" << Fd_stbd << " F_port:" << F_port << " F_stbd:" << F_stbd << endl;

    // update the rates of change of the system states
    dxdt[XNDX] = v*cos(psi);
    dxdt[YNDX] = v*sin(psi);
    dxdt[PSINDX] = w;
    dxdt[VELNDX] = (F_port + F_stbd)/ASV_MASS;
    dxdt[PSIDOTNDX] = (F_port - F_stbd)*(ASV_WIDTH/2)/ASV_INERTIA;
    cout << "dx:" << dxdt[XNDX] << " dy:" << dxdt[YNDX] << " dpsi:" << dxdt[PSINDX] << " dv:" << dxdt[VELNDX] << " dpsidot:" << dxdt[PSIDOTNDX] << endl;
}

// torqeedo port command callback
void on_torqeedo_port_command(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const asv_torqeedo_motor_command_t *mc, lcm::LCM *lcm) 
{
    cout << "Port command: " << mc->command_speed << endl;
    in[PORTNDX] = mc->command_speed;
}

// torqeedo stbd command callback
void on_torqeedo_stbd_command(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const asv_torqeedo_motor_command_t *mc, lcm::LCM *lcm) 
{
    cout << "Stbd command: " << mc->command_speed << endl;
    in[STBDNDX] = mc->command_speed;
}


double rand_n(void) // generate normally distributed variable given uniformly distributed variable using the Box-Muller method
{
    double u,v;
    u = (double)rand()/(double)RAND_MAX;
    v = (double)rand()/(double)RAND_MAX;

    return pow(-2*log(u),0.5)*sin(2*M_PI*v);
}

// Calculate the path, this is where the magic happens
void calculate(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *heartbeat, lcm::LCM *lcm) 
{
    // call the solver
    //    integrate_n_steps(stepper, asv, state , 0.0 , 0.001 , 100 );
    integrate_const(stepper, asv, state , 0.0 , 0.1 , 0.01 ); // 1x speed simulation

    int64_t timeStamp = timestamp_now();

    //construct the nav message
    auv_acfr_nav_t nav;
    nav.utime = timeStamp;
    nav.x = state(XNDX);
    nav.y = state(YNDX);
    nav.depth = 0;
    nav.roll = 0;
    nav.pitch = 0;
    nav.heading = state(PSINDX);
    nav.vx = state(VELNDX);
    nav.vy = 0;
    nav.vz = 0;
    nav.rollRate = 0;
    nav.pitchRate = 0;
    nav.headingRate = state(PSIDOTNDX);

    nav.altitude = WATER_DEPTH - nav.depth;

    double latitude_fix, longitude_fix;
    map_projection_sim->calc_geo_coords( nav.x + rand_n()*0.1, nav.y + rand_n()*0.1, latitude_fix, longitude_fix );
    nav.latitude = latitude_fix;
    nav.longitude = longitude_fix;

    if (timeStamp - last_print_time > 0.1*1e6) // 10 Hz
    {
        last_print_time = timeStamp;

        cout << "Truth: ";

        if(! fp.is_open() )
            fp.open("/tmp/log.txt",
        			ios::out|ios::app);
        for(int i=0; i<5; i++) {
            printf("%2.3f ", state(i));
            fp << state(i) << " ";
        }
        for(int i=0; i<2; i++) {
            printf("%2.3f ", in[i]);
            fp << in[i] << " ";
        }
        fp << "\n";
        fp.close();
        printf("\n");
        fflush(NULL);



        // publish the nav message.  FIXMED: This should be the NOVATEL message.
        lcm->publish(vehicle_name+".ACFR_NAV", &nav);

    }

    last_obs_time = timeStamp;
}

void on_nav_store(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const auv_acfr_nav_t *nav, lcm::LCM *lcm) {
    cout << "Nav: ";

    double nav_state[13];

    nav_state[0] = nav->x;
    nav_state[1] = nav->y;
    nav_state[2] = nav->depth;
    nav_state[3] = nav->roll;
    nav_state[4] = nav->pitch;
    nav_state[5] = nav->heading;
    nav_state[6] = nav->vx;
    nav_state[7] = nav->vy;
    nav_state[8] = nav->vz;
    nav_state[9] = nav->rollRate;
    nav_state[10] = nav->pitchRate;
    nav_state[11] = nav->headingRate;

    nav_state[12] = nav->altitude;

    if( !fp_nav.is_open() )
        fp_nav.open( "/tmp/log_nav.txt",
    			ios::out | ios::app);
    for(int i=0; i<13; i++) {
        printf("%2.3f ", nav_state[i]);
        fp_nav << nav_state[i] << " ";
    }
    fp_nav << "\n";
    fp_nav.close();
    printf("\n");
    fflush(NULL);

}

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv)
{
    int opt;

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            vehicle_name = (char*)optarg;
            break;
         }
    }
}

int main_exit;
void signal_handler(int sig)
{
    main_exit = 1;
}

int main(int argc, char **argv)
{
    cout << "Starting sim..." << endl;
    parse_args(argc, argv);

    // install the signal handler
    main_exit = 0;
    signal(SIGINT, signal_handler);

    lcm::LCM lcm;

    //srand(time(NULL));
    srand(1234); // seeding the same way every time to allow filter comparisons

    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm.getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;

    char rootkey[64];
    char key[128];
    sprintf (rootkey, "nav.acfr-nav-new");

    //sprintf(key, "%s.slam_config", rootkey);
    //char *slamConfigFileName = bot_param_get_str_or_fail(param, key);
    //Config_File *slamConfigFile;
    //slamConfigFile = new Config_File(slamConfigFileName);

    double latitude_sim, longitude_sim;
    sprintf(key, "%s.latitude", rootkey);
    latitude_sim = bot_param_get_double_or_fail(param, key);
    sprintf(key, "%s.longitude", rootkey);
    longitude_sim = bot_param_get_double_or_fail(param, key);
    
    map_projection_sim = new Local_WGS84_TM_Projection(latitude_sim, longitude_sim);

    int64_t timeStamp = timestamp_now();
    last_print_time = timeStamp;
    last_obs_time = timeStamp;

    lcm.subscribeFunction(vehicle_name+".PORT_MOTOR_CONTROL", on_torqeedo_port_command, &lcm);
    lcm.subscribeFunction(vehicle_name+".STBD_MOTOR_CONTROL", on_torqeedo_stbd_command, &lcm);
    lcm.subscribeFunction("HEARTBEAT_10HZ", calculate, &lcm);
    //lcm.subscribeFunction("HEARTBEAT_100HZ", calculate, &lcm); // needs to happen at 100 Hz due to IMU

    // initial conditions
    state(XNDX) = -10;
    state(YNDX) = -5;
    state(PSINDX) = 0;  
    state(VELNDX) = 0;
    state(PSIDOTNDX) = 0;

    in[PORTNDX] = 0;
    in[STBDNDX] = 0;

    fp_nav.open( "/tmp/log_nav.txt", ios::out);
    fp.open("/tmp/log.txt", ios::out);

    int fd = lcm.getFileno();
    fd_set rfds;
    while(!main_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select (fd + 1, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
            lcm.handle();
    }
    //delete slamConfigFile;
    delete map_projection_sim;
    if( fp.is_open())
    	fp.close();
    if( fp_nav.is_open() )
    	fp_nav.close();
    return 0;
}



