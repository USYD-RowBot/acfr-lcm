#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <libgen.h>

#include "perls-lcmtypes/senlcm_micron_sounder_t.h"
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"

int program_exit;
void signal_handler(int sig_num)
{
   // do a safe exit
    program_exit = 1;
}

int
main (int argc, char *argv[])
{
    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);
    
    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);
                
	char rootkey[64];    
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;
        
    acfr_sensor_canonical(sensor, '\r', '\n');
    
    fd_set rfds;
    char buf[16];
    char value[16];
    senlcm_micron_sounder_t micron;
    
    while(!program_exit)
    { 
        memset(buf, 0, sizeof(buf));
        memset(&micron, 0, sizeof(senlcm_micron_sounder_t));
        
        FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);
	
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
	    
	    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            micron.utime = timestamp_now();
            if(acfr_sensor_read(sensor, buf, sizeof(buf)) > 0)
            {
                // parse out the voltage value leaving the mV off then end
                memset(value, 0, sizeof(value));
                strncpy(value, buf, strlen(buf) - 2);
                micron.altitude = atof(value);
                
                senlcm_micron_sounder_t_publish(lcm, "MICRON_SOUNDER", &micron);
            }
        }
    }
    
    return 1;
}
                 
                
            
            

