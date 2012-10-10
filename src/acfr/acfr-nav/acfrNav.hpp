#define GPS3


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <gsl/gsl_sort_double.h>	// used to sort the motor speeds
#include <gsl/gsl_interp.h>			// used to interpolate the motor speeds
#include <pthread.h>

// Seabed includes
#include "adt_raw_file.hpp"
#include "seabed_interface.hpp"
#include "seabed_slam_file_io.hpp"
#include "auv_seabed_slam_models.hpp"

extern "C"
{
#include "magfield.h"
}
// LCM includes
#include "perls-common/units.h"
#include "perls-common/nmea.h"
#include "perls-lcmtypes/senlcm_raw_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include "perls-lcmtypes/senlcm_os_altimeter_t.h"
#include "perls-lcmtypes/senlcm_os_compass_t.h"
#include "perls-lcmtypes/senlcm_ms_gx1_t.h"
#include "perls-lcmtypes/senlcm_honeywell_hmr3600_t.h"
#include "perls-lcmtypes/senlcm_oas_t.h"

#ifdef GPS3
	#include "perls-lcmtypes/senlcm_gpsd3_t.h"
#else
	#include "perls-lcmtypes/senlcm_gpsd_t.h"
#endif //GPS3

#include "perls-lcmtypes/senlcm_os_motors_t.h"
#include "perls-lcmtypes/senlcm_IMU_t.h"
#include "perls-lcmtypes/senlcm_parosci_t.h"
#include "perls-lcmtypes/senlcm_lq_modem_t.h"
#include "perls-lcmtypes/senlcm_seabird_t.h"
#include "perls-lcmtypes/senlcm_seabird_depth_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/senlcm_ysi_t.h"

#include "perls-lcmtypes/acfrlcm_auv_os_conduit_t.h"
#include "perls-lcmtypes/acfrlcm_auv_os_conduit_raw_t.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/acfrlcm_auv_vis_rawlog_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
//#include "perls-lcmtypes/perllcm_auv_os_conduit_osi_t.h"
//#include "perls-common/generic_sensor_driver.h"


#define RTOD (UNITS_RADIAN_TO_DEGREE)
#define DTOR (UNITS_DEGREE_TO_RADIAN)

using namespace std;
using namespace libflounder;
using namespace libplankton;

// class to hold the pose options from the config file
class Pose_Aug_Options {
public:

   Pose_Aug_Options( const Config_File &config_file )
   {
      config_file.get_value( "POSE_AUG_USE_MAX_TIME_OPTION", use_max_time_option );
      config_file.get_value( "POSE_AUG_MAX_TIME"           , max_time  );
      config_file.get_value( "POSE_AUG_VIS_MIN_DIST"       , vis_min_dist );
   }

   bool use_max_time_option;
   double max_time;
   double vis_min_dist;
};

class acfrNav {
public:
    acfrNav();
    ~acfrNav();

    int initialise(lcm_t *_lcm, char *_slamConfigFileName, char *depthTareFileName,
        char *_motorSpeedsFileName, int _motorSpeedsNumber, int _updateRate, bool _savePoses,
        int _rphSource, bool _lcmPublish, bool _useIMU, bool _useParosci, bool _useYsi, 
        bool _useRdi, bool _useIverPropCount, bool _processToRaw);
    int saveData();

    lcm_t *lcm;

    // LCM handlers
    void handleHeartBeat(const perllcm_heartbeat_t *heartbeat);
//    void handleOpos(const perllcm_heartbeat_t *heartbeat);
//    void handleIverStatus(const perllcm_auv_os_conduit_osi_t*);
    void handleHMR3600(const senlcm_honeywell_hmr3600_t *hmr3600);
    void handleAltitude(const senlcm_os_altimeter_t *alt);
#ifdef GPS3    
    void handleGps(const senlcm_gpsd3_t *gps);
#else
    void handleGps(const senlcm_gpsd_t *gps);
#endif // GPS3
    void handleOSCompass(const senlcm_os_compass_t *osc);
    void handleMotors(const senlcm_os_motors_t *m);
    void handleIMU(const senlcm_IMU_t *imu);
    void handleMsGx1(const senlcm_ms_gx1_t *ms);
    void handleParosci(const senlcm_parosci_t *parosci);
    void handleLQModem(const senlcm_lq_modem_t *lqm);
    void handleCT(const senlcm_seabird_t *lqm);
    void handleSeabirdDepth(const senlcm_seabird_depth_t *sd);
    void handleRDI(const senlcm_rdi_pd5_t *rdi);
    void handleVis(const acfrlcm_auv_vis_rawlog_t *vis);
    void handleYsi(const senlcm_ysi_t *ysi);
    void handleNavOutTop(const perllcm_heartbeat_t *hb);
    void handleOAS(const senlcm_oas_t *oas);

private:
    // file names etc
/*    char *lcmUrl;
    char *slamConfigFileName;
    char *motorSpeedsFileName;
    int motorSpeedsNumber;
    int updateRate;
*/
    // stored data
    double altitude;
    double oasAltitude;
    senlcm_os_compass_t osCompass;
    double gpsSpeed, gpsHeading;
    double vehicleHeading;
    long imuTimeStamp;
    double fwdObstacleDist;

    // look up table of motor speeds
    double motorSpeed[256];

    // helper functions
    int loadMotorSpeeds(char *filename, int number);

    // config vaiables
    bool processToRaw;
    bool savePoses;
    int rphSource;
    bool lcmPublish;
    bool useIMU;
    bool useParosci;
    bool useYsi;
    bool useRdi;
    bool useIverPropCount;
    Pose_Aug_Options *poseAugOptions;
    double lastPoseAugmentationTime;
    SymMatrix R_gps;
    //double magneticVariation;
    double depthTareYsi;

    // slam object
    Seabed_Interface *slam;
	int lowRateCount;
	
    // saved data
    vector<Vehicle_Pose_Cov> slamPoseCov;
    int savedPoseId;
    void saveVehiclePoses(string fileHeader);
    void saveVehicleCov(string fileHeader);

    int oposCount;
    
    // output stream for processing to RAW
    #define rawOut cout
//    ostream rawOut;
};

enum LQM_MSG_TYPE {
	LQM_FIX,
	LQM_PING,
	LQM_ABORT
	};

// vehicle type
enum {vt_sirius, vt_iver};

// rph source
enum {rph_osCompass, rph_rdi, rph_hmr3600, rph_ms};

