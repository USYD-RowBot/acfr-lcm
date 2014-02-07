#include "acfrNav.hpp"
#include "perls-common/lcm_util.h"

#include <bot_param/param_client.h>
#include <signal.h>


int navExit;

void signalHandler(int sigNum) {
    // do a safe exit
    navExit = 1;
}

int
main (int argc, char *argv[]) {
    // set up the signal handler
    navExit = 0;
    signal(SIGINT, signalHandler);

    // this is needed by bot_param, for some reason it code that does this 
    // doesn't work so we do it here, I'll fix this later
//    if (!g_thread_supported ())
//        g_thread_init (NULL);

    lcm_t *lcm = lcm_create(NULL);
    BotParam *param = NULL;
    
    if(argc < 1)
        param = bot_param_new_from_server (lcm, 1);
    else
        param = bot_param_new_from_named_server (lcm, argv[1], 1);


	char rootkey[64];        
    sprintf (rootkey, "nav.%s", basename (argv[0]));

    // compass type
    char key[128];
    int rphSource;
    char *rphTypeStr;
    sprintf(key, "%s.rph", rootkey);
    rphTypeStr = bot_param_get_str_or_fail(param, key);
    if(!strcmp(rphTypeStr, "os"))
        rphSource = rph_osCompass;
    else if(!strcmp(rphTypeStr, "hmr3600"))    
        rphSource = rph_hmr3600;
    else if(!strcmp(rphTypeStr, "rdi"))    
        rphSource = rph_rdi;
    else if(!strcmp(rphTypeStr, "ms-gx1"))    
        rphSource = rph_ms;        
    else
	{
        printf("Invalid rphSource %s\n", rphTypeStr);
	    return 0;
	}
	
	char *vehicleTypeStr;
	int vehicleType;
	sprintf(key, "%s.vehicleType", rootkey);
	vehicleTypeStr = bot_param_get_str_or_fail(param, key);
	if(!strcmp(vehicleTypeStr, "sirius"))
	    vehicleType = vt_sirius;
	else if(!strcmp(vehicleTypeStr, "iver"))
	    vehicleType = vt_iver;
	else
	{
	    printf("Invalid vehicle type %s\n", vehicleTypeStr);
	    return 0;
	}

    // get the slam config filenames from the master LCM config
	char *slamConfigFileName;
	sprintf(key, "%s.slamConfig", rootkey);
	slamConfigFileName = bot_param_get_str_or_fail(param, key);
	
	char *depthTareFileName = NULL;
	sprintf(key, "%s.depthTare", rootkey);
	bot_param_get_str(param, key, &depthTareFileName);

    // Iver specific config variables
    char *motorSpeedsFileName;
    int motorSpeedNumber;
    if(vehicleType == vt_iver)
    {
        // get the file name for the motor speeds	    
	    sprintf(key, "%s.motorSpeeds", rootkey);
	    motorSpeedsFileName = bot_param_get_str_or_fail(param, key);

	    // get the speed set number
	    sprintf(key, "%s.motorSpeedNumber", rootkey);
	    motorSpeedNumber = bot_param_get_int_or_fail(param, key);
    }
    
    int useIverPropCount;
	sprintf(key, "%s.useIverPropCount", rootkey);
	if(bot_param_get_boolean(param, key, &useIverPropCount) == -1)
		useIverPropCount = 0;
    
    int useYsi;
    sprintf(key, "%s.useYsi", rootkey);
    if(bot_param_get_boolean(param, key, &useYsi) == -1)
        useYsi = 0;
        
	// check to see if we are going to save the covariances
	int saveCov;
	sprintf(key, "%s.saveCov", rootkey);
	if(bot_param_get_boolean(param, key, &saveCov) == -1)
	    saveCov = 0;

	// check to see if we are going to use an IMU
	int useIMU;
	sprintf(key, "%s.useIMU", rootkey);
	if(bot_param_get_boolean(param, key, &useIMU) == -1)
	    useIMU = 0;
	
	// check to see if we are going to use the parosci
	int useParosci;
	sprintf(key, "%s.useParosci", rootkey);
	if(bot_param_get_boolean(param, key, &useParosci) == -1)
	    useParosci = 0;

	
	int useRdi;
	sprintf(key, "%s.useRdi", rootkey);
	if(bot_param_get_boolean(param, key, &useRdi) == -1)
		useRdi = 0;
	
    int updateRate;
	sprintf(key, "%s.updateRate", rootkey);
	if(bot_param_get_int(param, key, &updateRate) == -1)
		updateRate = 1;	// if its not there set it to 1
	// check to see if its 1, 10 or 100 which are the only valid values
	if((updateRate != 1) && (updateRate != 5) && (updateRate != 10) && (updateRate != 100))
		updateRate = 1;
/*
    int oposRate;
	sprintf(key, "%s.oposRate", rootkey);
	if(bot_param_get_int(param, key, &oposRate) == -1)
		oposRate = 1;	// if its not there set it to 1
	// check to see if its 1, 10 or 100 which are the only valid values
	if((oposRate != 1) && (oposRate != 5) && (oposRate != 10) && (oposRate != 100))
		oposRate = 1;
*/
    // create the nav object and initialise it
    acfrNav nav;
    nav.initialise(lcm, slamConfigFileName, depthTareFileName, motorSpeedsFileName, motorSpeedNumber,
        updateRate, saveCov, rphSource, true, useIMU, useParosci, useYsi, useRdi, 
        useIverPropCount, false);

    // time out of 1 second so we can Ctrl-C to exit
    while (!navExit)
      {
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	lcmu_handle_timeout(lcm, &tv);
      }
	
    nav.saveData();
    return 0;
}

