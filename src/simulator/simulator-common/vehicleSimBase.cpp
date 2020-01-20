// nga AUV simulator


#include <iostream>
#include <fstream>
#include <thread>
#include <sys/timerfd.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <boost/numeric/odeint.hpp>
#include <small/Pose3D.hh>
#include <bot_param/param_client.h>
#include "acfr-common/auv_map_projection.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_nga_motor_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_bluefin_tail_status_t.hpp"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/senlcm/tcm_t.hpp"
#include "perls-lcmtypes++/senlcm/ysi_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/rdi_pd5_t.hpp"
#include "perls-lcmtypes++/senlcm/IMU_t.hpp"
#include "perls-lcmtypes++/senlcm/micron_sounder_t.hpp"
#include "simulator-common/vehicleSimBase.hpp"
#include "perls-lcmtypes++/senlcm/os_power_system_t.hpp"
#include "perls-lcmtypes++/senlcm/os_power_cont_t.hpp"

using namespace std;
using namespace boost::numeric::odeint;
using namespace acfrlcm;
using namespace perllcm;
using namespace senlcm;
using namespace libplankton;
using namespace vehicleSimSimple;

double rand_n(void) // generate normally distributed variable given uniformly distributed variable using the Box-Muller method
{
    double u,v;
    u = (double)rand()/(double)RAND_MAX;
    v = (double)rand()/(double)RAND_MAX;

    return pow(-2*log(u),0.5)*sin(2*M_PI*v);
}

static volatile bool exit_signalled;

// Calculate the path, this is where the magic happens
void VehicleSimBase::calculate() 
{
    // call the solver
    integrate_const(stepper, std::bind(&VehicleSimBase::updateState, this, placeholders::_1 , placeholders::_2 , placeholders::_3 ), state , 0.0 , time_step , time_step/10 ); // 1x speed simulation

    // populate the nav structure with the vehicle states
    int64_t timeStamp = timestamp_now();

    nav.utime = timeStamp;
    nav.x = state(XNDX);
    nav.y = state(YNDX);
    nav.depth = state(ZNDX);
    nav.roll = state(ROLLNDX);
    nav.pitch = state(PITCHNDX);
    nav.heading = state(HDGNDX);
    nav.vx = state(XDOTNDX);
    nav.vy = state(YDOTNDX);
    nav.vz = state(ZDOTNDX);
    nav.rollRate = state(ROLLDOTNDX);
    nav.pitchRate = state(PITCHDOTNDX);
    nav.headingRate = state(HDGDOTNDX);

    nav.altitude = WATER_DEPTH- nav.depth;//*(1-nav.y/50) - nav.depth;

    double latitude_fix, longitude_fix;
    map_projection_sim->calc_geo_coords( nav.x + rand_n()*0.1, nav.y + rand_n()*0.1, latitude_fix, longitude_fix );
    nav.latitude = latitude_fix * M_PI/180;
    nav.longitude = longitude_fix * M_PI/180;

}

void VehicleSimBase::publishACFRNav()
{
   int64_t timeStamp = timestamp_now();
   if (timeStamp - last_print_time > 1*1e6) // 10 Hz
    {
        last_print_time = timeStamp;

        // publish the nav message
        lcm.publish(vehicle_name+".ACFR_NAV", &nav);

    }
}
    // for simulating the sensors, acfr_nav_new should be publishing the ACFR_NAV

void VehicleSimBase::publishIMU()
{
    int64_t timeStamp = timestamp_now();
    //IMU
    //rotate gravity into local frame
    SMALL::Vector3D grav_b;

    SMALL::Rotation3D Cbn;
    Cbn.setRollPitchYawRad(state(3), state(4), state(5));

    grav_b = (Cbn.i() * grav); // look to local_planner.cpp implementation if this doesn't work

    //rotate earth rotation into the local frame
    SMALL::Vector3D earth_rot_b;
    earth_rot_b = (Cbn.i() * earth_rot);

    //compute acceleration at current time step
//    SMALL::Vector3D accel;
//    auv_accel(accel);

    //construct IMU message (biases to come later)
    senlcm::IMU_t imu;
    imu.utime = timeStamp;

    double dt = (timeStamp - last_obs_time)/1e6;

    if (dt > 0)
    {
        ba_x = ba_x + ( -1/TAU_A * ba_x + pow( (2*pow(BIAS_A,2)) / (TAU_A*dt) , 0.5 )*rand_n())*dt;
        ba_y = ba_y + ( -1/TAU_A * ba_y + pow( (2*pow(BIAS_A,2)) / (TAU_A*dt) , 0.5 )*rand_n())*dt;
        ba_z = ba_z + ( -1/TAU_A * ba_z + pow( (2*pow(BIAS_A,2)) / (TAU_A*dt) , 0.5 )*rand_n())*dt;
        bg_x = bg_x + ( -1/TAU_G * bg_x + pow( (2*pow(BIAS_G,2)) / (TAU_G*dt) , 0.5 )*rand_n())*dt;
        bg_y = bg_y + ( -1/TAU_G * bg_y + pow( (2*pow(BIAS_G,2)) / (TAU_G*dt) , 0.5 )*rand_n())*dt;
        bg_z = bg_z + ( -1/TAU_G * bg_z + pow( (2*pow(BIAS_G,2)) / (TAU_G*dt) , 0.5 )*rand_n())*dt;
    }

    imu.angRate[0] = state(9) + earth_rot_b[0] + bg_x + STD_G*rand_n();
    imu.angRate[1] = state(10) + earth_rot_b[1] + bg_y + STD_G*rand_n();
    imu.angRate[2] = state(11) + earth_rot_b[2] + bg_z + STD_G*rand_n();
    imu.accel[0] = accel[0] + grav_b[0] + ba_x + STD_A*rand_n();
    imu.accel[1] = accel[1] + grav_b[1] + ba_y + STD_A*rand_n();
    imu.accel[2] = accel[2] + grav_b[2] + ba_z + STD_A*rand_n();
    lcm.publish(vehicle_name+".IMU", &imu);

    //    if (timeStamp - last_print_time > 0.1*1e6) // 10 Hz
    //    {
    //        last_print_time = timeStamp;
    //        cout << "IMU: " << imu.angRate[0] << " " << imu.angRate[1] << " "<< imu.angRate[2] << " "<< imu.accel[0] << " "<< imu.accel[1] << " "<< imu.accel[2] << endl;
    //        cout << "accel: " << accel[0] << " " << accel[1] << " "<< accel[2] << endl;

    //    }
}

void VehicleSimBase::publishTCM()
{
    int64_t timeStamp = timestamp_now();
    //TCM compass
    if (timeStamp - last_tcm_time > 0.1*1e6) // 10 Hz
    {
        last_tcm_time = timeStamp;
        senlcm::tcm_t tcm;
        tcm.utime = timeStamp;
        tcm.heading = state(5) - MAG_VAR + rand_n()*0.5*M_PI/180;
        tcm.roll= state(3) + rand_n()*0.25*M_PI/180;
        tcm.pitch= state(4) + rand_n()*0.25*M_PI/180;
        tcm.temperature = 20;
        lcm.publish(vehicle_name+".TCM", &tcm);
    }
}

void VehicleSimBase::publishParosci()
{
    int64_t timeStamp = timestamp_now();
    // parosci depth
    if (timeStamp - last_parosci_time > 1.0*1e6) // 1 Hz
        //if(0)
    {
        last_parosci_time = timeStamp;
        senlcm::parosci_t parosci;
        parosci.utime = timeStamp;
        parosci.depth = state(2) + rand_n()*0.01;
	parosci.raw = parosci.depth;

        lcm.publish(vehicle_name+".PAROSCI", &parosci);
    }
}

void VehicleSimBase::publishYSI()
{
    int64_t timeStamp = timestamp_now();
    // YSI depth
    if (timeStamp - last_ysi_time > 0.1*1e6) // 10 Hz
        //if(0)
    {
        last_ysi_time = timeStamp;
        senlcm::ysi_t ysi;
        ysi.utime = timeStamp;
        ysi.salinity = 35; // ppt
        ysi.temperature = 20;
        ysi.depth = state(2) + rand_n()*0.01;
        ysi.turbidity = 0;
        ysi.chlorophyl = 0;
        ysi.conductivity = 0;
        ysi.oxygen = 0;
        ysi.battery = 0;
        lcm.publish(vehicle_name+".YSI", &ysi);
    }
}

void VehicleSimBase::publishGPS()
{
    int64_t timeStamp = timestamp_now();
    // GPS
    if (timeStamp - last_gps_time > 1*1e6) // 1 Hz
    {
        last_gps_time = timeStamp;
        if (nav.depth < 2) // 1m depth is where GPS cuts out, but simulation starts at 1m depth, so make it 2
        {
            SMALL::Vector4D nuLinear;
            nuLinear = nav.vx, nav.vy, nav.vz, 0.0;
            SMALL::Pose3D auvPose;
            auvPose.setPosition(nav.x, nav.y, nav.depth);
            auvPose.setRollPitchYawRad(nav.roll, nav.pitch, nav.heading);
            SMALL::Vector3D etaDotLin;
            etaDotLin = (auvPose.get3x4TransformationMatrix() * nuLinear);

            senlcm::gpsd3_t * gpsd3;
            gpsd3 = new senlcm::gpsd3_t;
            std::vector< int16_t > gused;
            std::vector< int16_t > gPRN;
            std::vector< int16_t > gelevation;
            std::vector< int16_t > gazimuth;
            std::vector< int16_t > gss;

            gpsd3->online = 0;
            gpsd3->fix.ept = 0;
            gpsd3->fix.epy = 0;
            gpsd3->fix.epx = 0;
            gpsd3->fix.epv = 0;
            gpsd3->fix.epd = 0;
            gpsd3->fix.eps = 0;
            gpsd3->fix.climb = 0;
            gpsd3->fix.epc = 0;
            gpsd3->geoidal_separation = 0;
            gpsd3->satellites_used = GPS_CHANNELS;
            for (int i=0; i<gpsd3->satellites_used; i++)
                gused.push_back(0);

            gpsd3->used = gused;
            gpsd3->dop.pdop = 0;
            gpsd3->dop.hdop = 0;
            gpsd3->dop.tdop = 0;
            gpsd3->dop.gdop = 0;
            gpsd3->dop.xdop = 0;
            gpsd3->dop.ydop = 0;
            gpsd3->epe    = 0;
            gpsd3->skyview_utime = 0;
            gpsd3->satellites_visible = GPS_CHANNELS;
            for (int i=0; i<gpsd3->satellites_visible; i++) {
                gPRN.push_back(0);
                gelevation.push_back(0);
                gazimuth.push_back(0);
                gss.push_back(0);
            }
            gpsd3->PRN = gPRN;
            gpsd3->elevation = gelevation;
            gpsd3->azimuth = gazimuth;
            gpsd3->ss = gss;
            gpsd3->dev.path = strdup("");
            gpsd3->dev.flags = 0;
            gpsd3->dev.driver = strdup("");
            gpsd3->dev.subtype = strdup("");
            gpsd3->dev.activated = 0;
            gpsd3->dev.baudrate = 0;
            gpsd3->dev.stopbits = 0;
            gpsd3->dev.cycle = 0;
            gpsd3->dev.mincycle = 0;
            gpsd3->dev.driver_mode = 0;
            gpsd3->fix.track = atan2( etaDotLin[1], etaDotLin[0] ) * 180/M_PI;
            gpsd3->fix.speed = pow(pow(etaDotLin[0],2) + pow(etaDotLin[1],2),0.5);
            double latitude_fix, longitude_fix;
            map_projection_sim->calc_geo_coords( nav.x + rand_n()*2, nav.y + rand_n()*2, latitude_fix, longitude_fix );
            gpsd3->fix.latitude = latitude_fix * M_PI/180;
            gpsd3->fix.longitude = longitude_fix * M_PI/180;
            gpsd3->fix.altitude = -nav.depth;
            gpsd3->fix.utime = timeStamp;
            gpsd3->utime = timeStamp;
            gpsd3->fix.mode = 3;
            gpsd3->status = 1;
            gpsd3->tag = strdup("");
            lcm.publish(vehicle_name+".GPSD_CLIENT", gpsd3);
            delete gpsd3;
        }
    }
}

void VehicleSimBase::publishBattery()
{
    int64_t timeStamp = timestamp_now();
    // POWER
    if (timeStamp - last_battery_time > 0.3*1e6) // once every 0.3 seconds
    {
        last_battery_time = timeStamp;

        // publish battery data
        senlcm::os_power_system_t battery_pack;
        memset(&battery_pack, 0, sizeof(battery_pack));
        battery_pack.utime = timestamp_now();
        battery_pack.avg_charge_p = 100;
        // need to have number of controllers set or you get a seg fault when publishing sometimes.
        battery_pack.num_controllers = 0;
        lcm.publish(vehicle_name+".BATTERY", &battery_pack);
    }

}

void VehicleSimBase::publishBluefinTail()
{
    int64_t timeStamp = timestamp_now();
    // Tail telemetry for decktest
    if (timeStamp - last_bluefin_time > 0.3*1e6) // once every 0.3 seconds
    {
        last_bluefin_time = timeStamp;

        // publish bluefin tail data
        acfrlcm::auv_bluefin_tail_status_t BFstatus;
        memset(&BFstatus, 0, sizeof(BFstatus));
        BFstatus.utime = timestamp_now();
        BFstatus.tail_utime = timestamp_now();

        BFstatus.voltage = 30;
        BFstatus.current = 5; 
        BFstatus.psu_temp = 30;

        BFstatus.tail_temp = 30;
        BFstatus.comp1 = 1;
        BFstatus.comp2 = 1;
        BFstatus.leak = 1;

        BFstatus.current_rpm = 0;
        BFstatus.target_rpm = 0;

        BFstatus.current_rudder = 0;
        BFstatus.target_rudder = 0;

        BFstatus.current_elevator = 0;
        BFstatus.target_elevator = 0;

        lcm.publish(vehicle_name+".BLUEFIN_STATUS", &BFstatus);
    }

}

void VehicleSimBase::publishDWN_OAS()
{
    int64_t timeStamp = timestamp_now();

   if (timeStamp - last_dwn_oas_time > 0.3*1e6) // once every 0.3 seconds
    {
        last_dwn_oas_time = timeStamp;
        senlcm::micron_sounder_t oas;
        memset(&oas, 0, sizeof(oas));

        oas.utime = timeStamp;
        oas.altitude = nav.altitude + rand_n()*0.003;
        lcm.publish(vehicle_name+".MICRON_SOUNDER_DWN", &oas);
    }
}

void VehicleSimBase::publishDVL()
{
    int64_t timeStamp = timestamp_now();
    // DVL
    if (timeStamp - last_dvl_time > 0.3*1e6) // once every 0.3 seconds
    {
        last_dvl_time = timeStamp;
        senlcm::rdi_pd5_t rdi;
        memset(&rdi, 0, sizeof(rdi));

        rdi.utime = timeStamp;
        rdi.pd4.utime = timeStamp;
        rdi.pd4.xducer_head_temp = 20;
        const double cos30 = 0.866025403784439; // cos(30*DTOR)
        rdi.pd4.range[0] = nav.altitude/cos30 + rand_n()*0.003;
        rdi.pd4.range[1] = nav.altitude/cos30 + rand_n()*0.003;
        rdi.pd4.range[2] = nav.altitude/cos30 + rand_n()*0.003;
        rdi.pd4.range[3] = nav.altitude/cos30 + rand_n()*0.003;
        rdi.pd4.altitude = ((rdi.pd4.range[0]+rdi.pd4.range[1]+rdi.pd4.range[2]+rdi.pd4.range[3])/4)*cos30;
        rdi.pd4.btv[0] = nav.vx + rand_n()*0.003;
        rdi.pd4.btv[1] = nav.vy + rand_n()*0.003;
        rdi.pd4.btv[2] = nav.vz + rand_n()*0.003;
        rdi.pitch = nav.pitch + rand_n()*0.25*M_PI/180;
        rdi.roll = nav.roll + rand_n()*0.25*M_PI/180;
        rdi.heading = nav.heading  - MAG_VAR + rand_n()*2*M_PI/180;

        if (nav.altitude < 50)
            rdi.pd4.btv_status = 0;
        else
            rdi.pd4.btv_status = 1; // beyond dvl bl range

        rdi.pd4.speed_of_sound = 1521.495;
        lcm.publish(vehicle_name+".RDI", &rdi);
    }

    last_obs_time = timeStamp;

}

static void signalHandler(int signal)
{
	exit_signalled = true;
}

VehicleSimBase::VehicleSimBase() :
    state(12), vehicle_name("DEFAULT"), interval_us(100000), time_step(interval_us*1.0e-6)
{
	exit_signalled = false;
	std::signal(SIGINT, signalHandler);
	
    //srand(time(NULL));
    srand(1234); // seeding the same way every time to allow filter comparisons

    param = bot_param_new_from_server (lcm.getUnderlyingLCM(), 1);
    if(param == NULL)
        return;

    // BotParam *cfg;
    
    // cfg = bot_param_new_from_server (lcm, 1);
    char rootkey[64];
    char key[128];
    sprintf (rootkey, "nav.acfr-nav-new");

    double latitude_sim, longitude_sim;
    // sprintf(key, "%s.latitude", rootkey);
    // latitude_sim = bot_param_get_double_or_fail(param, key);
    // sprintf(key, "%s.longitude", rootkey);
    // longitude_sim = bot_param_get_double_or_fail(param, key);

        // get the slam config filenames from the master LCM config for the acfr-nav module
    // we are using this to get the origin lat and long
    char *slam_config_filename;
    slam_config_filename = bot_param_get_str_or_fail(param, "nav.acfr-nav-new.slam_config");
    
    Config_File *slam_config_file;
    slam_config_file = new Config_File(slam_config_filename);
    slam_config_file->get_value("LATITUDE", latitude_sim);
    slam_config_file->get_value("LONGITUDE", longitude_sim);
    
    cout << "Simulator origin Lat: " << latitude_sim << " Lon: " << longitude_sim << endl;

    map_projection_sim = new Local_WGS84_TM_Projection(latitude_sim, longitude_sim);

    //Earth rotation vector in the navigation frame
    earth_rot = cos(latitude_sim*M_PI/180)*7.292115e-5,0,-sin(latitude_sim*M_PI/180)*7.292115e-5;

    //Set gravity
    grav = 0.0, 0.0, -9.81;

    int64_t timeStamp = timestamp_now();
    last_dvl_time = timeStamp;
    last_gps_time = timeStamp;
    last_ysi_time = timeStamp;
    last_parosci_time = timeStamp;
    last_tcm_time = timeStamp;
    last_print_time = timeStamp;
    last_obs_time = timeStamp;

    //initialise drifting biases
    ba_x = BIAS_A*rand_n();
    ba_y = BIAS_A*rand_n();
    ba_z = BIAS_A*rand_n();
    bg_x = BIAS_G*rand_n();
    bg_y = BIAS_G*rand_n();
    bg_z = BIAS_G*rand_n();


    // Interial matrices
    /*longM = m - Xudot, 			-Xwdot, 			m * zG - Xqdot,
            -Xwdot, 			m - Zwdot, 			-m * xG - Zqdot,
            m * zG - Xqdot, 	-m * xG - Zqdot, 	Iy - Mqdot;

    latM = 	m - Yvdot,			-m * zG - Ypdot,	m * xG - Yrdot,
            -m * zG - Ypdot,	Ix - Kpdot,			-Izx - Krdot,
            m * xG - Yrdot,		-Izx - Krdot,		Iz - Nrdot;
    */

    // initial conditions
    state(XNDX) = -10;
    state(YNDX) = 0;
    state(ZNDX) = 1.0;  //Z = 1;
    state(ROLLNDX) = 0;
    state(PITCHNDX) = 0.05;
    state(HDGNDX) = 0;
    state(XDOTNDX) = 0;
    state(YDOTNDX) = 0;
    state(ZDOTNDX) = 0;
    state(ROLLDOTNDX) = 0;
    state(PITCHDOTNDX) = 0;
    state(HDGDOTNDX) = 0;

}


VehicleSimBase::~VehicleSimBase()
{
    delete map_projection_sim;
}



void VehicleSimBase::setVehicleName(string vehName)
{
    vehicle_name = vehName;
}

string const VehicleSimBase::getVehicleName() const
{
    return vehicle_name;
}

double VehicleSimBase::prop_force( double prop_diameter, double prop_rpm, double water_velocity)
{
    double rho = 1030;              // Water density
    double J0 = 0; // open water advance coefficient
    double Kt = 0; // propeller thrust coefficient
    double Kq = 0; // properler torque coefficient
    double n = prop_rpm/60;
    
        // advance velocity as per Fossen eq 4.6
    double omega = 0.1;
    double Va = (1 - omega) * water_velocity;
    
    if(fabs(n) > 1e-3)
        J0 = Va / (n * prop_diameter);      // as per Fossen eq 6.107

    double alpha1 = 0.1574*4;
    double alpha2 = (-0.0144*4)/1.0;
    double alpha3 = (-0.1*alpha1)/(-0.2);
    double alpha4 = alpha1*1.9;
    double alpha5 = (alpha1) / (-0.5-(-0.2));

    // these need to be determined
    // but relate to the torque from the rear thruster
    // not sure but it may result in a torque not around
    // the x-axis of the vehicle
    double beta1 = 0.0;
    double beta2 = -0.0;

    if (J0 > 0)
    {
        Kt = alpha1 + alpha2 * J0;   // Fossen eq 6.113
        Kq = beta1 + beta2 * J0;
    }
    else if (J0 > -0.2)
    {
        Kt = alpha1 + alpha3 * J0;          // Fossen eq 6.113
        Kq = beta1 + beta2 * J0;
    }
    else
    {
        Kt = alpha4 + alpha5 * J0; // Fossen eq 6.113
        Kq = beta1 + beta2 * J0;
    }

    double prop_force;//, prop_torque;
    prop_force = rho * pow(prop_diameter,4) * Kt * fabs(n) * n;     // As per Fossen eq 4.2
    return prop_force;
}


void VehicleSimBase::lcm_thread()
{
    while (!exit_signalled)
    {
        this->lcm.handleTimeout(1000);
    }
}

void VehicleSimBase::run()
{
    // we need to spin off the LCM thread
    std::thread lcm_thread_handle(&VehicleSimBase::lcm_thread, this);

    subscribeLCMChannels();


    // now we set up the interval timers
    int fd;
    struct itimerspec itval;
                 
    fd = timerfd_create(CLOCK_MONOTONIC, 0);
    unsigned long long wakeups_missed = 0;
    
    if (fd == -1)
    {
        std::cerr << "Couldn't create timer." << std::endl;
        perror("create timer");
        exit_signalled = true;
        lcm_thread_handle.join();

        return;
    }

    std::cout << "Interval: " << (long)interval_us * 1000 << "us\n";
    std::cout << "Time step: " << time_step << "s\n";

    itval.it_interval.tv_sec = 0;
    itval.it_interval.tv_nsec = (long)interval_us * 1000;
    itval.it_value.tv_sec = 0;
    itval.it_value.tv_nsec = (long)interval_us * 1000;

    int ret;

    ret = timerfd_settime(fd, 0, &itval, NULL);

    if (ret)
    {
        perror("Failed to set time.");
        return;
    }

    //if (ret) check for failure
          
    // now the period waits
    while (!exit_signalled)
    {
        unsigned long long missed;
        // this handles the timer waiting
        ret = read(fd, &missed, sizeof(missed));
        if (ret == -1)
        {
            perror("read timer");

            exit_signalled = true;
            lcm_thread_handle.join();

            return;
        }

        if (missed > 1)
        {
            wakeups_missed += (missed - 1);
            std::cerr << "Missed a wakeup" << std::endl;
        }
        
        // update the vehicle state estimate.  This will call the
        // derived function to update the vehicle state and will also
        // populate the nav structure
        calculate();

        // call the derived function to publish the sensor data
        publishSensorData();
    }
}

void VehicleSimBase::publishSensorData()
{
    publishACFRNav();
}

//void VehicleSimBase::subscribeLCMChannels()
//{
//}

