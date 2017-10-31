#include <signal.h>
#include <stdio.h>
#include <bot_param/param_client.h>


#include "perls-lcmtypes/senlcm_aanderaa_4319_t.h"
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
#include "acfr-common/units.h"
#include "acfr-common/nmea.h"


int chop_string(char *data, char **tokens)
{
    char *token;
    int i = 0;

    token = strtok(data, " \t");
    while(token != NULL)
    {
        tokens[i++] = token;
        token = strtok(NULL, " \t");
    }
    return i;
}

int parse_a4319(lcm_t *lcm, char *d, int bytes)
{
    char *tok[16];
    int num_tok = chop_string(d, tok);
    if(num_tok == 7)
    {
	    if(!strncmp(tok[0], "4319", 4))
	    {

		senlcm_aanderaa_4319_t a4319;
		a4319.utime = timestamp_now();
    		a4319.conductivity = atof(tok[2]);
		a4319.temperature = atof(tok[3]);
		a4319.salinity = atof(tok[4]);
		a4319.density = atof(tok[5]);
		a4319.speed = atof(tok[6]);

		senlcm_aanderaa_4319_t_publish(lcm, "4319", &a4319);
	
		return 1;
	    }
	    else
	    {
	    	return 0;
	    }

    }
    else
	return 0;
    
}





int program_exit;
void signal_handler(int sig_num)
{
    // do a safe exit
    program_exit = 1;
}

int main(int argc, char *argv[])
{
    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    lcm_t *lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    acfr_sensor_canonical(sensor, '\r', '\n');
    
    char a4319_str[128];
    int bytes;
    
    while(!program_exit)
    {
	memset(a4319_str, 0, sizeof(a4319_str));
        bytes = acfr_sensor_read_timeout(sensor, a4319_str, sizeof(a4319_str), 1);
        if(bytes > 0)
        {
	    parse_a4319(lcm, a4319_str, bytes);
        }
    }
    
    acfr_sensor_destroy(sensor);
    
    return 1;
}
    
    
    

