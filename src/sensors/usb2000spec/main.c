//
//  main.c
//  USB2000Driver
//
//  Created by Daniel Bongiorno on 5/09/13.
//  Copyright (c) 2013 Daniel Bongiorno. All rights reserved.
//

#include <stdio.h>
#include <lcm/lcm.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>

#include "USB2000Lib.h"
#include "perls-lcmtypes/senlcm_usb2000_spec_t.h"
#include <unistd.h>
#include "perls-common/timestamp.h"
#include <bot_param/param_client.h>
#include "perls-common/serial.h"


//#define PORT_ADDR "/dev/tty.PL2303-0010121A"
enum {io_socket, io_serial};
int program_exit;
void 
signal_handler(int sigNum) 
{
    // do a safe exit
    program_exit = 1;
}

int main(int argc, const char * argv[])
{
    printf("Spectrometer Starting");
    
    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);   
    

    /*
    int serialPort = initSerial(PORT_ADDR,9600);
    
    write(serialPort, "S", 1);
    usleep(1000);
    
    struct specHeader head;
    
    readHeaderPacket(head, serialPort);
    
    
    unsigned char buf[128];
    while(1) {
        ssize_t n = read(serialPort, buf, 128);
        if(n < 0) {
            break;
        } else if (n == 0) {
            printf(".");
            usleep(1000);
        } else {
            printf("%d\n", n);
            for(int i = 0; i < n; i++) {
                printf("%d ", buf[i]);
            }
            printf("\n");
        }
    
    }
    
    return 0;
}
*/
    int error = 0;
    long INTTIME = 1000;
    unsigned long specData[2048];
    
    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);
    
    
    
    senlcm_usb2000_spec_t specReading = {
        .timestamp = timestamp_now(),
        .id = "DownwardSpec",
        .numSamples = 2048,
        .startEndIdx = {0,0},
        .sixteenBitData = 1,
    };
    int32_t dd[10] = {0,1,2,3,4};
    specReading.specData = dd;
    
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
    int spec_fd;
    if(io == io_serial)
    {
        spec_fd = serial_open(serial_dev, serial_translate_speed(baud), serial_translate_parity(parity), 1);
        if(spec_fd < 0)
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
    	spec_fd = socket(spec_addr->ai_family, spec_addr->ai_socktype, spec_addr->ai_protocol);
        if(connect(spec_fd, spec_addr->ai_addr, spec_addr->ai_addrlen) < 0) 
        {
	        printf("Could not connect to %s on port %s\n", ip, inet_port);
    		return 1;
        }
    
    }

    
    //Setup serial
    //int serialPort = initSerial(PORT_ADDR,115200);
    
    //flushBuffer(serialPort);
    //error = getSpectra(serialPort, specData);

    
    
    //Initalise spectrometer with correct Baud rate
    //error = setBaud(serialPort, PORT_ADDR, 6);
    //Set to Binary Data mode
    error = setDataMode(spec_fd, TRUE);
    printf("Data mode set");
    //init int time
    //serialPort = initSerial(PORT_ADDR,115200);
    error = setIntTime(spec_fd, INTTIME);
    printf("Int time set");
  
    //Set trigger mode
    error = setTriggerMode(spec_fd, 4);
     printf("Trigger mode set");
    //error = setTriggerMode(serialPort, 0); //free running mode
    flushBuffer(spec_fd);
    //Need to tune these parameters for Auto Gain
    unsigned long thresholds[3] = {25000, 1000, 10000};
    
    //flushBuffer(serialPort);
    //long rubbish[3000];
    //receivePacket(rubbish, 3000, serialPort, 2);
    //Acquisition Loop
    while (!program_exit) {
        for (int i = 0; i < 3; i++) {
            //fp = fopen("/Users/daniel/Desktop/logFile.txt", "a");
            error = getSpectra(spec_fd, specData);

            for (int j = 0; j < 50; j++){
                printf("%lu\n", specData[j]);
            }
            specReading.timestamp = timestamp_now();
            specReading.specData = specData;
            specReading.numSamples = 2048;
            specReading.intTime = 1000;
            senlcm_usb2000_spec_t_publish(lcm, "SPEC_DOWN", &specReading);
            
            //fprintf(fp, "%llu\t%u\t", specReading.timestamp, specReading.intTime);

            //for (int j = 0; j < 2048; j++) {
            //    fprintf(fp, "%lu\t",specData[j]);
            //}
            
            //fprintf(fp, "\n");
            //fclose(fp);
        }
        //check for correct gain every 3 samples
        /*
        long newIntTime = checkIntTime(specData, 2048, INTTIME, thresholds);
        if (newIntTime != INTTIME) {
            //we have a new Int time
            setIntTime(spec_fd, newIntTime);
            INTTIME = newIntTime;
        }
        */
    }
    
    close(spec_fd);
    lcm_destroy(lcm);


    
    
    
    
    return 0;
}

