#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>

#include "perls-common/timestamp.h"
#include <bot_param/param_client.h>
#include "perls-common/serial.h"

#include "perls-lcmtypes/senlcm_tcm_t.h"

#include "tcm.h"

#define update_rate 0.1

#define DTOR 3.141592/180

enum {io_socket, io_serial};


int parse_tcm(char *buf, senlcm_tcm_t *tcm)
{
    // decode the tcm frame
    char frame_id = buf[2];
    char *current_pos;
    
    
    if(frame_id == kDataResp)
    {
            int count = (int)buf[3];
            current_pos = &buf[4];
            int index = 0;
            while(1)
            {
                int data_id = (int)*current_pos++;
         
                switch(data_id)
                {
                    case kHeading:
                        tcm->heading = (*(float *)current_pos) * DTOR;
                        current_pos += 4;
                        index++;
                        break;    
                    case kPAngle:
                        tcm->pitch = (*(float *)current_pos) * DTOR;
                        current_pos += 4;
                        index++;
                        break;
                    case kRAngle:
                        tcm->roll = (*(float *)current_pos) * DTOR;
                        current_pos += 4;
                        index++;
                        break;
                    case kTemperature:
                        tcm->temperature = *(float *)current_pos;
                        current_pos += 4;
                        index++;
                        break;
                        
//                    case kXAligned	:
//                        tcm->mag_x = (*(float *)current_pos);
//                        current_pos += 4;
//                        index++;
//                        break;
//                    case kYAligned:
//                        tcm->mag_y = (*(float *)current_pos);
//                        current_pos += 4;
//                        index++;
//                        break;
//                    case kZAligned:
//                        tcm->mag_z = (*(float *)current_pos);
//                        current_pos += 4;
//                        index++;
//                        break;

                }
                if(index == count)
                    break;
		
	    }
    }
     
    return 1;            
             
}

int tcm_form_message(char *in, int data_len, char *out)
{
    unsigned short len = data_len + 4;
    out[0] = ((char *)&len)[1];
    out[1] = ((char *)&len)[0];

    memcpy(&out[2], in, data_len);
    unsigned short crc = tcm_crc(out, data_len + 2);
    out[data_len+3] = crc & 0xff;
    out[data_len+2] = crc >> 8;
    
    return len;
}
    


int program_tcm(int fd)
{
    char data[128];
    char out[128];
    int len;
    
    // set the return types
    memset(data, 0, sizeof(data));
    data[0] = kSetDataComponents;
    data[1] = 4;
    data[2] = kHeading;
    data[3] = kPAngle;
    data[4] = kRAngle;
    data[5] = kTemperature;
//    data[6] = kXAligned;
//    data[7] = kYAligned;
//    data[8] = kZAligned;
    len = tcm_form_message(data, 6, out);
//    len = tcm_form_message(data, 9, out);
    write(fd, out, len);
    
    
    // set the acquisition mode
    memset(data, 0, sizeof(data));
    data[0] = kSetAcqParams;
    data[1] = 0;     // data push mode
    data[2] = 0;     // flush filter is false
    float interval = update_rate;
    memcpy(&data[7], &interval, sizeof(float));
    len = tcm_form_message(data, 11, out);    
    write(fd, out, len);

    // set the filter mode
    double c4[4] = {4.6708657655334e-2, 4.5329134234467e-1, 4.5329134234467e-1, 4.6708657655334e-2};
    double c8[8] = {01.9875512449729e-2, 06.4500864832660e-2, 01.6637325898141e-1, 02.4925036373620e-1, 
                02.4925036373620e-1, 01.6637325898141e-1, 06.4500864832660e-2, 01.9875512449729e-2};
    char filter_length = 8;
    memset(data, 0, sizeof(data));
    data[0] = 12; //kSetFIRFilters;
    data[1] = 3;
    data[2] = 1;
    data[3] = filter_length;
    if( filter_length == 4 ) {
        memcpy(&data[4], c4, sizeof(double)*filter_length);
    }
    else {
        memcpy(&data[4], c8, sizeof(double)*filter_length);
    }
    len = tcm_form_message(data, sizeof(double)*filter_length + 4, out);
    write(fd, out, len);


    // start
    memset(data, 0, sizeof(data));
    data[0] = kStartIntervalMode;
    len = tcm_form_message(data, 1, out);    
    write(fd, out, len);

    return 1;
}

int program_exit;
void
signal_handler(int sigNum)
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
                
    // Read the LCM config file
    BotParam *param;
	char rootkey[64];
	char key[64];
	
    param = bot_param_new_from_server (lcm, 1);
    
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    // read the config file
    int io;
	sprintf(key, "%s.io", rootkey);
	char *io_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(io_str, "serial"))
        io = io_serial;
    else if(!strcmp(io_str, "socket"))
        io = io_socket;
    
    char *serial_dev, *parity;
    int baud;
    char *ip, *inet_port;
    
    if(io == io_serial)
    {
        sprintf(key, "%s.serial_dev", rootkey);
        serial_dev = bot_param_get_str_or_fail(param, key);

    	sprintf(key, "%s.baud", rootkey);
	    baud = bot_param_get_int_or_fail(param, key);

	    sprintf(key, "%s.parity", rootkey);
	    parity = bot_param_get_str_or_fail(param, key);
    }
    
    if(io == io_socket)
    {
        sprintf(key, "%s.ip", rootkey);
        ip = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.port", rootkey);
        inet_port = bot_param_get_str_or_fail(param, key);
    }

    // Open either the serial port or the socket
    struct addrinfo hints, *spec_addr;
    int fd;
    if(io == io_serial)
    {
        fd = serial_open(serial_dev, serial_translate_speed(baud), serial_translate_parity(parity), 1);
        if(fd < 0)
        {
            printf("Error opening port %s\n", serial_dev);
            return 0;
        }
    }        
    else if(io == io_socket)
    {
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo(ip, inet_port, &hints, &spec_addr);
    	fd = socket(spec_addr->ai_family, spec_addr->ai_socktype, spec_addr->ai_protocol);
        if(connect(fd, spec_addr->ai_addr, spec_addr->ai_addrlen) < 0) 
        {
	        printf("Could not connect to %s on port %s\n", ip, inet_port);
    		return 1;
        }
    
    }
    
    int len;
    char buf[256];
    int64_t timestamp;
    senlcm_tcm_t tcm;
    
    program_tcm(fd);
    
    fd_set rfds;
    
    while(!program_exit)
    {
        memset(buf, 0, sizeof(buf));
        
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
	
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
	    
	    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            timestamp = timestamp_now();
            len = read(fd, buf, 1);
            if(len == 1 && buf[0] == 0)
            //    continue;
            //else
            {
                // read another byte
                len += read(fd, &buf[1], 1);
                unsigned short data_len = (buf[0] << 8) + buf[1];   
                if(data_len > 5)
                {
                    // read the rest of the data
                    while(len < data_len)
                        len += read(fd, &buf[len], data_len - len);

                    // check the checksum
                    unsigned short crc = *(unsigned short *)&buf[len - 2];
                    if(tcm_crc(&buf[0], data_len-2) == (((crc >> 8) | (crc & 0xff) << 8)))
                    {
                        memset(&tcm, 0, sizeof(senlcm_tcm_t));
                        tcm.utime = timestamp;
                        // its good data, lets parse it

                        if(parse_tcm(&buf[0], &tcm))
                            senlcm_tcm_t_publish(lcm, "TCM", &tcm);
                    }
        	        else
            	        printf("Bad CRC\n");
    	        }
                else
                    printf("\nBad data_len %d\n", data_len);
            }
        }
    }        
        
    close(fd);
    
    return 0;
    
    	
}
