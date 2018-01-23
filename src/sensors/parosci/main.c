/* 	Paroscientific Depth sensor LCM module
	uses code lifted from the original vehicle code, SIO_parosci.c

	30/9/10
	Christian Lees
	
	Moved to the ACFR sensor driver 23/01/18 CL
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
#include "acfr-common/units.h"
//#include "perls-common/units.h"
//#include "perls-common/generic_sensor_driver.h"
#include "perls-lcmtypes/senlcm_parosci_t.h"

#define PAROSCI_START_CMD    "*0100P4\r\n" /* continuously sample and send      */
#define PAROSCI_STOP_CMD     "*0100P3\r\n" /* sample & send one measurement     */
#define INVALID_PARO_DEPTH   -1000.0       /* return value if bad read          */
#define ATMOSPHERE_OFFSET    14.7        /* surface pressure (psia)                  */
#define ATMOSPHERE_CONVERSION	0.689476	/* conversion psia to db/m */

//---------------------------------------------------------------------------
//  int validate_parosci -- Private:  validates character string grabbed
//                                    from PAROSCI port
//                          Returns 0-success  -1-error
//---------------------------------------------------------------------------
static int validateParosci(char *msg_ptr)
{
    int msgValid = 0;
    if (*msg_ptr++ != '*')  msgValid = -1;
    if (*msg_ptr++ != '0')  msgValid = -1;
    if (*msg_ptr++ != '0')  msgValid = -1;
    if (*msg_ptr++ != '0')  msgValid = -1;
    if (*msg_ptr++ != '1')  msgValid = -1;

    while ( ((*msg_ptr >= '0') && (*msg_ptr <= '9')) || (*msg_ptr == '.'))
        msg_ptr++;

    if ((*msg_ptr != '\n') && (*msg_ptr != '\r'))  msgValid = -1;

    return msgValid;
}

//---------------------------------------------------------------------------
//  double parse_parosci -- Private: converts PAROSCI string into a
//                                   depth measurement
//                          Returns 0-success  -1-error
//---------------------------------------------------------------------------
static int parseParosci(char *msg_ptr,double *depth)
{
    double my_depth, x;

    if (validateParosci(msg_ptr) == 0)
    {
        my_depth = 0.0;
        msg_ptr += 5; /* point to first digit */
        while (*msg_ptr != '.')
            my_depth = 10.0 * my_depth + (double) (*msg_ptr++ - '0');

        msg_ptr++;    /* skip decimal point */
        x = 1.0;
        while ((*msg_ptr >= '0') && (*msg_ptr <= '9'))
            my_depth = my_depth + ( (double) (*msg_ptr++ - '0') / pow(10.0, x++) );

        *depth = my_depth;

        return 1;
    }
    else
        return 0;
}

/*
static int myopts(generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Paroscientific depth sensor driver.");
    return 0;
}
*/


void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, char **vehicle_name)
{
    int opt;

    const char *default_name = "DEFAULT";
    *vehicle_name = malloc(strlen(default_name)+1);
    strcpy(*vehicle_name, default_name);
    
    int n;
    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            n = strlen((char *)optarg);
            free(*vehicle_name);
            *vehicle_name = malloc(n);
            strcpy(*vehicle_name, (char *)optarg);
            break;
         }
    }
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
    
    char *vehicle_name;
    parse_args(argc, argv, &vehicle_name);
    char sensor_channel[100];
    snprintf(sensor_channel, 100, "%s.PAROSCI", vehicle_name);

    lcm_t *lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    acfr_sensor_canonical(sensor, '\r', '\n');
    acfr_sensor_write(sensor, PAROSCI_START_CMD, strlen(PAROSCI_START_CMD));

    char parosci_str[256];
    int bytes;
    
    while(!program_exit)
    {
	memset(parosci_str, 0, sizeof(parosci_str));
        bytes = acfr_sensor_read_timeout(sensor, parosci_str, sizeof(parosci_str), 1);
        if(bytes > 0)
        {
        	senlcm_parosci_t parosci;
        	parosci.utime = timestamp_now();
	    	if(parseParosci(parosci_str, &parosci.raw))
            	if(validateParosci(parosci_str) == 0)
            	{
				    parosci.depth = (parosci.raw - ATMOSPHERE_OFFSET) * ATMOSPHERE_CONVERSION;
				    senlcm_parosci_t_publish (lcm, sensor_channel, &parosci);
				}	    
        }
    }
    
    acfr_sensor_destroy(sensor);
    
    return 1;
}
    


/*

int main (int argc, char *argv[])
{

    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical (gsd, '\r','\n');
    gsd_launch (gsd);

    gsd_flush (gsd);
    gsd_reset_stats (gsd);

    // send the start command
    gsd_write(gsd, PAROSCI_START_CMD, strlen(PAROSCI_START_CMD));

    // loop to collect, parse and publish
    while(!gsd->done)
    {
        char buf[256];
        int64_t timestamp;
        gsd_read (gsd, buf, 256, &timestamp);

        senlcm_parosci_t parosci;
        parosci.utime = timestamp;
        if(parseParosci(buf, &parosci.raw))
            if(validateParosci(buf) == 0)
            {
                parosci.depth = (parosci.raw - ATMOSPHERE_OFFSET) * ATMOSPHERE_CONVERSION;
                senlcm_parosci_t_publish (gsd->lcm, gsd->channel, &parosci);
                gsd_update_stats (gsd, 1);
            }
            else
                gsd_update_stats (gsd, -1);
        else
            gsd_update_stats (gsd, -1);
    }
    gsd_write(gsd, PAROSCI_STOP_CMD, strlen(PAROSCI_STOP_CMD));
}
*/
