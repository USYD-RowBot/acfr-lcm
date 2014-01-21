//
//  main.c
//  Learning C
//
//  Created by Daniel Bongiorno on 21/08/13.
//  Copyright (c) 2013 Daniel Bongiorno. All rights reserved.
//

#include <stdio.h>
#include "STSspecLibC.h"
#include <time.h>
#include <unistd.h>
#include <lcm/lcm.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>

#include "perls-lcmtypes/senlcm_sts_spec_t.h"
#include <unistd.h>
#include "perls-common/timestamp.h"
#include <bot_param/param_client.h>
#include "perls-common/serial.h"

//Extra Serial headers
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <errno.h>


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
    printf("STS Spectrometer Starting\n");
    
    int CHECKRATE = 10;
    double SAMPLE_INTERVAL = 1; //time in seconds
    int meanThres_autoGain = 3000;
    int meanStartIdx = 350;
    int meanWidth = 300;
    int autoGainOn = 0;
    long INTTIME = 500000;
    
    
    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);
    
    
    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);
    
    
    senlcm_sts_spec_t specReading = {
        .timestamp = timestamp_now(),
        .id = "UpwardSpec",
        .numSamples = 1024,
        .boardTemp = 0.0,
        .detectTemp = 0.0,
        .newTemps = 0,
        .intTime = 0,
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
    
    sprintf(key, "%s.checkRate", rootkey);
    CHECKRATE = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.sampleInterval", rootkey);
    SAMPLE_INTERVAL = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.meanThres_autoGain", rootkey);
    meanThres_autoGain = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.meanStartIdx", rootkey);
    meanStartIdx = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.meanWidth", rootkey);
    meanWidth = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.autoGainOn", rootkey);
    autoGainOn = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.intTimeIntial", rootkey);
    INTTIME = (long) bot_param_get_int_or_fail(param, key) * 1000;
    
    printf("Serial Device: %s, IO: %u, (serial: %u), baud: %u\n",serial_dev,io, io_serial, baud);

    // Open either the serial port or the socket
    struct addrinfo hints, *spec_addr;
    int spec_fd;
    if(io == io_serial)
    {
        spec_fd = serial_open(serial_dev, serial_translate_speed(baud), serial_translate_parity(parity), 0);
        serial_set_noncanonical (spec_fd, 1, 0);
        
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
    
    
    

    //----------------
    
    unsigned short specData[1024];
    

    unsigned long thresholds[5] = {15500, 1500, 5000, 350, 300};
    thresholds[2] = meanThres_autoGain;
    thresholds[3] = meanStartIdx;
    thresholds[4] = meanWidth;
    
    
    float temps[2];

    char serialNum[16];
    getSerialNum(spec_fd, serialNum);
    printf("Serial Number :%s\n",serialNum);
    specReading.id = serialNum;
    
    flushBuffer(spec_fd);
    
    //get wavelength coeff
    float waveCoeff[5];
    getWaveCoeff(spec_fd, waveCoeff);
    for (int i = 0; i < 4; i++) {
        printf("Wcoeff: %u = %.50f\n", i, waveCoeff[i]);
    }
    
    float nonLinCoeff[8];
    getNonlinCoeff(spec_fd, nonLinCoeff);
    for (int i = 0; i < 8; i++) {
        printf("NonLinCoeff: %u = %.50f\n", i, nonLinCoeff[i]);
    }
    
    setIntTime(spec_fd, INTTIME);
    getSpectra(spec_fd, specData);

    //timer stuff
    time_t lastT,curT;
    double diffT;
    time(&lastT);
    time(&curT);
    

    int i = 0;
    while (!program_exit) {
    
        time(&curT);
        diffT = difftime(curT, lastT);
        usleep(250000);
        
        if (diffT > (double) SAMPLE_INTERVAL) {
            time(&lastT);
            //printf(".");
            getSpectra(spec_fd, specData);
            specReading.timestamp = timestamp_now();
            //for (int i = 0; i < 10; i++) {
            //    printf("%lu\t",specData[i]);
            //    specReading.specData[i] = specData[i];
            //    printf("\n");
            //}
            specReading.specData = &specData;
            specReading.numSamples = 1024;
            specReading.intTime = INTTIME / 1000;
            senlcm_sts_spec_t_publish(lcm, "SPEC_UP", &specReading);
            
            i++;
            if (i > CHECKRATE && autoGainOn == 1) {
                //check for correct gain every 3 samples
                
                long newIntTime = checkIntTime(specData, 1024, INTTIME, thresholds);
                newIntTime = (newIntTime /1000) * 1000;
                
                printf("Old Int: %luus, New Int: %luus\n", INTTIME, newIntTime);
                if (newIntTime != INTTIME) {
                    //we have a new Int time
                    setIntTime(spec_fd, newIntTime);
                    INTTIME = newIntTime;
                }
                
                getTemp(spec_fd, temps);
                specReading.detectTemp = temps[0];
                specReading.boardTemp = temps[1];
                specReading.newTemps = 1;
                           
                i = 0;
            }
            else {
                specReading.newTemps = 0;
            }
        }
    }
    
    close(spec_fd);
    lcm_destroy(lcm);

    return 0;
}

