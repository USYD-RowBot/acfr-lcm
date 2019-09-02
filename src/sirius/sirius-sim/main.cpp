/*
    Listens to the motor command message and drives a simulated vehicle.
    Synthetic sensor data is also sent out reflecting the vehicle state.
    A failsafe is implemented using the 1Hz heart beat
    
    Stefan B. Williams
    ACFR
    06/01/12
*/
#define GPS3

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <sys/select.h>
#include <math.h>
#include <gps.h> // from debian package libgps-dev

//#include "perls-common/serial.h"
#include "acfr-common/timestamp.h"
//#include "perls-common/lcm_util.h"
#include "acfr-common/units.h"

#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_status_t.h"
#include "perls-lcmtypes/senlcm_parosci_t.h"
#ifdef GPS3
    #include "perls-lcmtypes/senlcm_gpsd3_t.h"
#else
    #include "perls-lcmtypes/senlcm_gpsd_t.h"
#endif //GPS3
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_os_power_system_t.h"
#include "perls-lcmtypes/senlcm_os_power_cont_t.h"



#include <libplankton/auv_map_projection.hpp>
#include <libplankton/auv_config_file.hpp>


using namespace std;
using namespace libplankton;

#define MAX_MESSAGE_SIZE 128
#define MOTOR_TIMEOUT 100000000  // this is in microseconds



// FIXME: this is duplicated from the gpsd_client tool.  Should be common.
static void
init_lcm_gpsd (senlcm_gpsd3_t *gd)
{
    // MAXCHANNELS defined in gps.h
    gd->used = (int16_t*) malloc (MAXCHANNELS * sizeof (*(gd->used)));
    gd->PRN = (int16_t*) malloc (MAXCHANNELS * sizeof (*(gd->PRN)));
    gd->elevation = (int16_t*) malloc (MAXCHANNELS * sizeof (*(gd->elevation)));
    gd->azimuth = (int16_t*) malloc (MAXCHANNELS * sizeof (*(gd->azimuth)));
    gd->ss = (int16_t*) malloc (MAXCHANNELS * sizeof (*(gd->ss)));
}

static void
free_lcm_gpsd (senlcm_gpsd3_t *gd)
{
    free (gd->used);
    free (gd->PRN);
    free (gd->elevation);
    free (gd->azimuth);
    free (gd->ss);
}



class veh_sim_state
{
public:
    veh_sim_state();
    void update_motor_command(double vertical, double port, double starboard);
    void update_motor_command_vert(double vertical);
    void update_motor_command_lat(double port, double starboard);
    void update_vehicle_state(double dt);
    void send_vehicle_sensor_obs(double t);
    int load_config(char *name);

    double vert_command;
    double port_command;
    double stbd_command;
    pthread_mutex_t port_lock;
    
    // lcm stuff
    lcm_t *lcm;
    
    // failsafe
    int64_t last_motor_command_t;
    pthread_mutex_t time_lock;

    // veh state updates
    int64_t last_vehicle_update_t;

    // battery status
    int32_t avg_charge_p;

    // thrust
    double vert;
    double port;
    double stbd;

	// body speeds
    double surge;
    double sway;

	// origin
	double origin_lat;
	double origin_lon;

	// local coordinates
	double x, y;
	double x_rate, y_rate;
    double curr_lat, curr_lon;

    // depth
    double depth, depth_rate;
 
    // attitude
	double heading, heading_rate;
	double roll, pitch;

    // altitude and oas observations
	double alt;
	double oas_range;

	// vehicle parameters (should be read in from a config file)
    double M;                   //Kg
    double M33;                 //Kg
    double Buoy;                  //Newtons pos up
    double M11;                  //Kg
    double M22;                 //Kg
    double current_speed;//.01; // 0.11;//.1;              //m/sec
    double current_dir;       //degrees (compass) flowing to 
    double thruster_tau;
    double rollpitch_tau;

    double J;
    double J11;
    double yaw_damping;

    // map projection
   libplankton::Local_WGS84_TM_Projection *map_projection;


};    

veh_sim_state::veh_sim_state()
{
    vert_command = 0.0;
    port_command = 0.0; 
    stbd_command = 0.0;

    depth = 0.0;
    depth_rate = 0.0;
    roll = 0.0;
    pitch = 0.0;
    x = 0.0;
    y = 0.0;
    x_rate = y_rate = 0.0;
    heading = 0.0;
    heading_rate = 0.0;
    surge = 0.0;
    sway = 0.0;
    port = stbd = vert = 0;
    avg_charge_p = 100;
/*
	// vehicle parameters (FIXME: should be read in from a config file)
    M = 250;                   //Kg
    M33 = 0.9112*M; //added mass of prolate elipsoid (Kg)
    Buoy = 8;                  //Newtons pos up
    M11 = 0.0639*M; //added mass of prolate elipsoid (Kg)
    M22 = 0.9112*M; //added mass of prolate elipsoid (Kg)
    current_speed = 0.0;//.01; // 0.11;//.1;              //m/sec
    current_dir = 0;       //degrees (compass) flowing to 
    thruster_tau = .4;
    rollpitch_tau = 0.1;

    J = 25;
    J11 = 0.7*J; // added mass of prolate elipsoid
    yaw_damping = 165;
*/
    last_vehicle_update_t = timestamp_now();


    pthread_mutex_init(&port_lock, NULL);
    pthread_mutex_init(&time_lock, NULL);
}

int
veh_sim_state::load_config(char *name)
{
   // read the config file
    BotParam *cfg;
	char rootkey[64];
	char key[64];

    
    cfg = bot_param_new_from_server (lcm, 1);
    
    sprintf(rootkey, "acfr.%s", name);

    // read the parameters from the config file
    sprintf(key, "%s.M", rootkey);
    M = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.M11", rootkey);
    M11 = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.M22", rootkey);
    M22 = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.M33", rootkey);
    M33 = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.Buoy", rootkey);
    Buoy = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.current_speed", rootkey);
    current_speed = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.current_dir", rootkey);
    current_dir = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.thruster_tau", rootkey);
    thruster_tau = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.rollpitch_tau", rootkey);
    rollpitch_tau = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.J", rootkey);
    J = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.J11", rootkey);
    J11 = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.yaw_damping", rootkey);
    yaw_damping = bot_param_get_double_or_fail(cfg, key);

	
	// get the slam config filenames from the master LCM config for the acfr-nav module
	// we are using this to get the origin lat and long
	char *slam_config_filename;
	slam_config_filename = bot_param_get_str_or_fail(cfg, "nav.acfr-nav-new.slam_config");
	
	Config_File *slam_config_file;
	slam_config_file = new Config_File(slam_config_filename);
	slam_config_file->get_value("LATITUDE", origin_lat);
	slam_config_file->get_value("LONGITUDE", origin_lon);
	
	cout << "Simulator origin Lat: " << origin_lat << " Lon: " << origin_lon << endl;

    // setup the global configuration
    map_projection = new libplankton::Local_WGS84_TM_Projection( origin_lat,
                                 origin_lon );
                                 
    return 1;
}
void
veh_sim_state::update_motor_command(double vertical, double port, double starboard)
{
    vert_command = vertical;
    port_command = port;
    stbd_command = starboard;
}

void
veh_sim_state::update_motor_command_vert(double vertical)
{
    vert_command = vertical;
}

void
veh_sim_state::update_motor_command_lat(double port, double starboard)
{
    port_command = port;
    stbd_command = starboard;
}

void
motor_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_sirius_motor_command_t *mc, void *u) 
{
    veh_sim_state *state = (veh_sim_state *)u;
    
    pthread_mutex_lock(&state->time_lock);
    state->last_motor_command_t = mc->utime;
    pthread_mutex_unlock(&state->time_lock);
    
    pthread_mutex_lock(&state->port_lock);
    state->update_motor_command(mc->vertical, mc->port, mc->starboard);
    pthread_mutex_unlock(&state->port_lock);
}
/*
void
motor_handler_vert(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_sirius_motor_command_t *mc, void *u)
{
    veh_sim_state *state = (veh_sim_state *)u;

    pthread_mutex_lock(&state->time_lock);
    state->last_motor_command_t = mc->utime;
    pthread_mutex_unlock(&state->time_lock);

    pthread_mutex_lock(&state->port_lock);
    state->update_motor_command_vert(mc->vertical);
    pthread_mutex_unlock(&state->port_lock);
}

void
motor_handler_lat(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_sirius_motor_command_t *mc, void *u)
{
    veh_sim_state *state = (veh_sim_state *)u;

    pthread_mutex_lock(&state->time_lock);
    state->last_motor_command_t = mc->utime;
    pthread_mutex_unlock(&state->time_lock);

    pthread_mutex_lock(&state->port_lock);
    state->update_motor_command_lat(mc->port, mc->starboard);
    pthread_mutex_unlock(&state->port_lock);
}
*/

void
veh_sim_state::update_vehicle_state(double dt)
{
  // FIXME: scale the motor commands to 'real' forces in Newtons 
  double desired_port = port_command; //0.05*fabs(port_command)*(port_command)/TO_RPM;
  double desired_stbd = stbd_command; //0.05*fabs(stbd_command)*(stbd_command)/TO_RPM;
  double desired_vert = vert_command; //0.05*fabs(vert_command)*(vert_command)/TO_RPM;

  // model inefficiency in reverse thrust
  if(desired_port < 0)
      desired_port = desired_port*.9;
  if(desired_stbd < 0)
    desired_stbd = desired_stbd*.9;
  if(desired_vert < 0)
    desired_vert = desired_vert*.9;

  double alpha = exp(-dt / thruster_tau );

  // FIXME: model 'slow' dynamics of prop/motor.  Should probably build a better
  // model of this.  Could also design whole control system using a proper
  // higher order model.
  port = alpha*(port) + (1.0-alpha)*(desired_port);
  stbd = alpha*(stbd) + (1.0-alpha)*(desired_stbd);
  vert = alpha*(vert) + (1.0-alpha)*(desired_vert);
/*
  port = desired_port;
  stbd = desired_stbd;
  vert = desired_vert;
*/

  // roll/pitch
  double rollpitch_alpha = exp(-dt / rollpitch_tau );
  roll =   rollpitch_alpha*(roll) + (1.0-rollpitch_alpha)*(0.01*rand()/RAND_MAX - 0.005);
  pitch =   rollpitch_alpha*(pitch) + (1.0-rollpitch_alpha)*(0.01*rand()/RAND_MAX - 0.005);

  //depth
  double depth_Cd = 0.75; // modelled as a cylinder of L/D 6.67
  double rho = 1000; // kg/m^3
  double depth_A = 2*2*0.3; // very rough - two long cylinders of 2m x 0.3m
  double depth_acc = (1/(M + M33))*(-Buoy - 0.5*rho*depth_Cd*depth_A*depth_rate*
    fabs(depth_rate) + vert);

  depth_rate += depth_acc * dt;
  depth += depth_rate * dt;

  if(depth < 0) //stop at the water's surface and bob
  {
    depth = 0;
    depth_rate = .02;
  }

  // project the speed and current onto the vehicle body frame to give
  // the relative water speed for drag
  double rel_water_vel_surge = surge - current_speed*cos(current_dir - heading);
  double rel_water_vel_sway = sway - current_speed*sin(current_dir - heading);

  // lateral dynamics
  double surge_Cd = 0.2; // modelled as an ellipsoid L/D 6.67
  double surge_A = 2*M_PI*(0.09); // approx. frontal area

  double surge_acc = (1/(M+M11))*(-0.5*rho*surge_Cd*surge_A*rel_water_vel_surge*fabs(rel_water_vel_surge) - 10*rel_water_vel_surge + (port+stbd)/2);
  surge += surge_acc*dt;

  double sway_Cd = 0.75; // modelled as a cylinder of L/D 6.67
  double sway_A = 2*2*0.3; // very rough - tow long cylinders of 2m x 0.3m
  
  double sway_acc = (1/(M+M33))*(-0.5*rho*sway_Cd*sway_A*rel_water_vel_sway*fabs(rel_water_vel_sway) - 10*rel_water_vel_sway);
  sway += sway_acc*dt;

  
  // heading
  double thruster_lever = 0.5; // m
  double heading_acc = (1/(J+J11))*(-yaw_damping*heading_rate + (port - stbd)*thruster_lever);
    
  int truc = 1;
  if (heading_acc != heading_acc)
  {
    cout << "attach " ;
    while(truc);
  }
  
  if(heading_acc != heading_acc)
  {
  	cout << "Heading Acc Nan\n";
  	heading_acc = 0.0;
  } 
  heading_rate += heading_acc*dt;
  heading += heading_rate*dt;

  // world velocity 
  x_rate = cos(heading)*surge - sin(heading)*sway;
  y_rate = sin(heading)*surge + cos(heading)*sway;
  
  // position update
  x += x_rate*dt;
  y += y_rate*dt;


  // project into geographic coordinates
  map_projection->calc_geo_coords( x, y, curr_lat, curr_lon);
}

void
veh_sim_state::send_vehicle_sensor_obs(double t)
{
  // publish PAROSCI data
  senlcm_parosci_t parosci;
  parosci.utime = t;
  parosci.raw = depth;
  parosci.depth = depth;
  senlcm_parosci_t_publish(lcm, "SIRIUS.PAROSCI", &parosci);

  // publish GPS data if not submerged
  if ( depth < 1.5 )
  {
    #ifdef GPS3
      senlcm_gpsd3_t gd = {0};
      init_lcm_gpsd( &gd );

      gd.utime = t;
      gd.fix.latitude = curr_lat * DTOR;
      gd.fix.longitude = curr_lon * DTOR;
      gd.fix.speed = sqrt(x_rate*x_rate + y_rate*y_rate);
      gd.fix.track = atan2(y_rate, x_rate);
      gd.fix.mode = 2;
      gd.status = 2; 
      gd.satellites_used = 11;
      gd.satellites_visible = 11;
//      char tag[1];
      gd.tag = strdup ("");
      gd.dev.path = strdup ("");
      gd.dev.driver = strdup ("");
      gd.dev.subtype = strdup ("");
      for (int i=0; i<gd.satellites_used; i++)
        gd.used[i] = 0;

      //FIXME: this seems to make the simulator crash.
      senlcm_gpsd3_t_publish(lcm, "SIRIUS.GPSD_CLIENT", &gd);

      free_lcm_gpsd( &gd );
    #else
      senlcm_gpsd_t gd;
      gd.utime = t;
      gd.latitude = curr_lat * DTOR;
      gd.longitude = curr_lon * DTOR;
      gd.speed = sqrt(x_rate*x_rate + y_rate*y_rate);
      gd.track = atan2(y_rate, x_rate);
      gd.mode = 2;
      gd.status = 2; 
      senlcm_gpsd_t_publish(lcm, "SIRIUS.GPSD_CLIENT", &gd);
    #endif
  }

  // publish RDI data
  senlcm_rdi_pd5_t rdi;
  rdi.utime = t;
  rdi.heading = heading;
  rdi.pitch = pitch;
  rdi.roll = roll;
  rdi.pd4.btv[0] = surge;
  rdi.pd4.btv[1] = sway;
  rdi.pd4.btv[2] = depth_rate;
  rdi.pd4.altitude = 30-depth; // FIXME
  rdi.pd4.speed_of_sound = 0;
  rdi.pd4.btv_status = 0;

  senlcm_rdi_pd5_t_publish(lcm, "SIRIUS.RDI", &rdi);

  // publish battery data
  senlcm_os_power_system_t battery_pack;
  battery_pack.utime = t;
  battery_pack.avg_charge_p = avg_charge_p;
  // need to have number of controllers set or you get a seg fault when publishing sometimes.
  battery_pack.num_controllers = 0;
  senlcm_os_power_system_t_publish(lcm, "SIRIUS.BATTERY", &battery_pack);

  // publish Thruster data

  // publish OAS data

  // publish Ecopuck data

  // publish multibeam data

  // publish stereo data
}

void
heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    // this is used to drive the vehicle model and to make sure some process 
    // up stream doesn't die and leave us in a bad
    // state.  If a timeout occurs then we will shut the motors down

    veh_sim_state *state = (veh_sim_state *)u;
        
    pthread_mutex_lock(&state->time_lock);
    int64_t time_diff = hb->utime - state->last_motor_command_t;
    if(time_diff > MOTOR_TIMEOUT)
    {
        pthread_mutex_lock(&state->port_lock);
        //state->update_motor_command(0, 0, 0);
        pthread_mutex_unlock(&state->port_lock);
    }

    double dt = (hb->utime - state->last_vehicle_update_t)*1e-6;
    state->last_vehicle_update_t = hb->utime;

    // update the current vehicle state
    state->update_vehicle_state(dt);

    // send data out over the appropriate lcm channels
    state->send_vehicle_sensor_obs(hb->utime);

    pthread_mutex_unlock(&state->time_lock);
}


int program_exit;
void 
signal_handler(int sigNum) 
{
    // do a safe exit
    program_exit = 1;
}

int 
main(int argc, char **argv)
{
    veh_sim_state state;
    // lets start LCM
	state.lcm = lcm_create(NULL);
    if(!state.load_config(basename(argv[0])))
        return 0; 
 
    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);   
    
 	
	
	
	acfrlcm_auv_sirius_motor_command_t_subscribe(state.lcm, "SIRIUS.THRUSTER", &motor_handler, &state);
//	acfrlcm_auv_sirius_motor_command_t_subscribe(state.lcm, "SIRIUS_MOTOR_VERT", &motor_handler_vert, &state);
//	acfrlcm_auv_sirius_motor_command_t_subscribe(state.lcm, "SIRIUS_MOTOR_LAT", &motor_handler_lat, &state);
	perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_10HZ", &heartbeat_handler, &state);
	
	// listen to LCM and all three serial ports
	while(!program_exit) 
	{
        lcm_handle_timeout(state.lcm, 1000);
//cout << "X: " << state.x << " Y: " << state.y << " Hdg: " << state.heading*180/M_PI <<  " Depth: " << state.depth << " surge: " << state.surge << " sway: " << state.sway << endl;
	}
cout << "Done!" << endl;
	
	return 0;
}
