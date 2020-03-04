#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <math.h>
#include <termios.h>
#include <libudev.h>

#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_spektrum_control_command_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#define BUFSIZE 100


typedef struct {
    char vehicle_name[BUFSIZE];
    char channel_name[BUFSIZE];
    char rootkey[BUFSIZE];

    int channels;
    short channel_values[16];

    char send_next;
    int dsm_level;
    unsigned char dsm_preamble;


    lcm_t *lcm;
    acfrlcm_auv_spektrum_control_command_t sc;
} state_t;

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;

    state->send_next = 1;
}

// Takes the type of device we are looking for, eg /dev/ttyUSB*, the serial number of the device we are looking for
// and will return the actual device eg /dev/ttyUSB0
int find_device(char *serial_number, char *port)
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	
	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		fprintf(stderr, "Can't create udev\n");
		return 0;
	}
	
	/* Create a list of the devices in the 'tty' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "tty");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);
	
	udev_list_entry_foreach(dev_list_entry, devices) 
	{
		const char *path;
		
		/* Get the filename of the /sys entry for the device
		   and create a udev_device object (dev) representing it */
		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);
		// Get the /dev entry
		const char *device_node = udev_device_get_devnode(dev);
		
		// Weed out the none USB devices
		dev = udev_device_get_parent_with_subsystem_devtype(
		       dev,
		       "usb",
		       "usb_device");
		       
		if(!dev)
			continue;
		
		// Get the devices serial number	
		const char *device_serial_number = udev_device_get_sysattr_value(dev, "serial");
		printf("%s, %s\n", device_node, device_serial_number);
		if(!strcmp(device_serial_number, serial_number))
		{
			printf("Found serial device at %s\n", device_node);
			strcpy(port, device_node);
			udev_device_unref(dev);
			udev_enumerate_unref(enumerate);
			udev_unref(udev);
			return 1;
		}

		udev_device_unref(dev);
	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	udev_unref(udev);
	return 0;
}
// Parse the 16 bytes that come from the RC controller
static int
parse_rc(char *buf, state_t *state)
{

    unsigned short channel_value;
    char channel_id;
    // short channel_values[state->channels];


    // packet size is always 16, first two bytes are status/flags
    // and data isn't necessarily ordered
    // so read the last 7 byte pairs
    for(int i=1; i<8; i++)
    {
        // we have an offset of 2 as the first two bytes are status/flags
        if(state->dsm_preamble == 0x12)
        {
            channel_id = (buf[i*2] & 0xF8) >> 3;
            channel_value = ((buf[i*2] & 0x07) << 8) | (buf[(i*2)+1] & 0xFF);
        }
        else
        {
            channel_id = (buf[i*2] & 0xFC) >> 2;
            channel_value = ((buf[i*2] & 0x03) << 8) | (buf[(i*2)+1] & 0xFF);
            // compensate for the limited range - so it better matches the
            // range of the DX6
            channel_value <<= 1;
        }
        

        
        //printf("%02x-%02x \t", (unsigned char)buf[i*2], (unsigned char)buf[i*2+1]);
        if (channel_id < state->channels)
        {
            //printf("%i-%i \t", (int)channel_id, (int)channel_value);
            state->channel_values[(int)channel_id] = channel_value;
        }
    }
    //printf("\n");



    acfrlcm_auv_spektrum_control_command_t sc;
    sc.utime = timestamp_now();
    sc.channels = state->channels;
    sc.values = state->channel_values;

    if (state->send_next)
    {
        state->send_next = 0;
        acfrlcm_auv_spektrum_control_command_t_publish(state->lcm, state->channel_name, &sc);
    }

    return 1;
}

int main_exit;
int broken_pipe;
void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else
        main_exit = 1;
}

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    printf("  -k config key                    param file config key\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, state_t *state)
{
    int opt;

    const char *def = "DEFAULT";
    strncpy(state->vehicle_name, def, BUFSIZE);
    snprintf(state->channel_name, BUFSIZE, "%s.SPEKTRUM_CONTROL", state->vehicle_name);

    while ((opt = getopt (argc, argv, "hn:k:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            strncpy(state->vehicle_name, (char *)optarg, BUFSIZE);
            snprintf(state->channel_name, BUFSIZE, "%s.SPEKTRUM_CONTROL", state->vehicle_name);
            break;
        case 'k':
            strncpy(state->rootkey, (char *)optarg, BUFSIZE);
            break;
         }
    }
}

static void *
lcm_thread (void *context)
{
    state_t *state = (state_t *) context;
    printf("LCM thread starting\n");
    while (!main_exit)
    {
        lcm_handle_timeout(state->lcm, 1000);
    }

    return 0;
}

void realign(acfr_sensor_t *sensor, state_t *state)
{
    // this is fragile - and may fail arbitrarily
    // but we need to find the start of the packet
    // based on decoding with an oscilloscope
    // the first two bytes of the message is 0xE1 0xA2
    fd_set rfds;
    char buf[16];
    uint8_t aligned = 0;

    //printf("Aligning to message.\n");
    while(!aligned && !main_exit)
    {
        if(broken_pipe)
        {
            sensor->port_open = 0;
            fprintf(stderr, "Pipe broken\n");
            continue;
        }

        memset(buf, 0, sizeof(buf));

        tcflush(sensor->fd, TCIOFLUSH);
        tcflush(sensor->fd, TCIOFLUSH);

        FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 2000;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
        {
            perror("Select failure: ");
        }
        else if(ret != 0)
        {
            int len;
            // deliberately don't align with expected packet size of 16 bytes
            int bytes = 16;
            len = acfr_sensor_read(sensor, buf, bytes);
            //printf("%i\n", len);
            for(int i=0;i<16; i++)
                printf("0x%02X ", buf[i] & 0xFF);
            printf("\n");
            
            
            if(len == bytes && (unsigned char)buf[1] == state->dsm_preamble)
            {
                aligned = 1;
            }
            else
            {
                len = acfr_sensor_read(sensor, buf, 1);
                //fprintf(stderr, "Shifting alignment, 0x%02X, 0x%02X\n", state->dsm_preamble& 0xFF, buf[1] & 0xFF);
            }
            
        }
        //else
        //{
        //    fprintf(stderr, "Timeout: Checking connection\n");
        //}
    }

    //printf("Aligned packet.\n");
}

int
main(int argc, char **argv)
{
    // install the signal handler
    main_exit = 0;
    broken_pipe = 0;
    signal(SIGINT, signal_handler);
    //signal(SIGPIPE, signal_handler);

    state_t state;
    parse_args(argc, argv, &state);

    state.lcm = lcm_create(NULL);
    BotParam *param = bot_param_new_from_server (state.lcm, 1);

    pthread_t tid;
    pthread_create(&tid, NULL, lcm_thread, &state);
    pthread_detach(tid);

	perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_10HZ", &heartbeat_handler, &state);

    if(param == NULL)
        return 0;

    // Read the config params
    char key[64];

    sprintf(key, "%s.channels", state.rootkey);
    state.channels = bot_param_get_int_or_fail(param, key);
	
    sprintf(key, "%s.dsm", state.rootkey);
    char *dsm_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(dsm_str, "DX5"))
        state.dsm_preamble = 0x01;
    else
        state.dsm_preamble = 0x12;

    printf("Setting DSM premable to 0x%02X\n", state.dsm_preamble & 0xFF);

	// As we want to over write the serial_device we can't use the acfr_sensor_create reoutine
	acfr_sensor_t *sensor = (acfr_sensor_t *)malloc(sizeof(acfr_sensor_t));
	if(acfr_sensor_load_config(state.lcm, sensor, state.rootkey))
	{
		if(!strcmp(sensor->serial_dev, "by_serial_number"))
		{
		    // Load the serial number of the device we are looking for
		    sprintf(key, "%s.device_serial_number", state.rootkey);
            char *device_serial_number = bot_param_get_str_or_fail(param, key);
            char device[128];
            if(!find_device(device_serial_number, device))
            {
                fprintf(stderr, "Could not find serial device based on serial number %s\n", device_serial_number);
                return 0;
            }
            // update the port in the sensor structure
            sensor->serial_dev = device;
        }
        
        if(!acfr_sensor_open(sensor))
            return 0;
   }    

    //acfr_sensor_t *sensor = acfr_sensor_create(state.lcm, rootkey);

    if(sensor == NULL)
        return 0;

    acfr_sensor_noncanonical(sensor, 1, 0);

    // Create the UDP listener
    realign(sensor, &state);

    fd_set rfds;
    char buf[16];

    while(!main_exit)
    {
        if(broken_pipe)
        {
            sensor->port_open = 0;
            fprintf(stderr, "Pipe broken\n");
            continue;
        }
        memset(buf, 0, sizeof(buf));

        FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
        {
            perror("Select failure: ");
        }
        else if(ret != 0)
        {
            int len;
            int bytes = 16;
            len = acfr_sensor_read(sensor, buf, bytes);
            if(len == bytes)
            {
                if ((unsigned char)buf[1] != state.dsm_preamble)
                {
                    realign(sensor, &state);
                }
                parse_rc(buf, &state);
            }
            else
            {
                fprintf(stderr, "Wrong number of bytes (%i)\n", len);
            }
        }
        //else
        //{
           // fprintf(stderr, "Timeout: Checking connection\n");
        //}
    }
    acfr_sensor_destroy(sensor);
    pthread_join(tid, NULL);

    return 0;
}

