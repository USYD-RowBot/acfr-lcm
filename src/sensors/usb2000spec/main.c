//
//  main.c
//  USB2000Driver
//
//  Created by Daniel Bongiorno on 5/09/13.
//  Copyright (c) 2013 Daniel Bongiorno. All rights reserved.
//

#include <stdio.h>
#include <lcm/lcm.h>
#include "USB2000Lib.h"
#include "exlcm_usb2000Spec_t.h"
#include <unistd.h>
#include "timestamp.h"

#define PORT_ADDR "/dev/tty.PL2303-0010121A"


int main(int argc, const char * argv[])
{
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
    FILE *fp;
    int error = 0;
    long INTTIME = 1000;
    unsigned long specData[2048];
    
    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);
    
    exlcm_usb2000Spec_t specReading = {
        .timestamp = timestamp_now(),
        .id = "DownwardSpec",
        .numSamples = 2048,
        .startEndIdx = {0,0},
        .sixteenBitData = 1,
    };
    int32_t dd[10] = {0,1,2,3,4};
    specReading.specData = dd;
    
    
    //Setup serial
    int serialPort = initSerial(PORT_ADDR,115200);
    
    flushBuffer(serialPort);
    error = getSpectra(serialPort, specData);

    
    
    //Initalise spectrometer with correct Baud rate
    //error = setBaud(serialPort, PORT_ADDR, 6);
    //Set to Binary Data mode
    //error = setDataMode(serialPort, TRUE);
    //init int time
    //serialPort = initSerial(PORT_ADDR,115200);
    //error = setIntTime(serialPort, INTTIME);
    //Set trigger mode
//    error = setTriggerMode(serialPort, 4);
    //error = setTriggerMode(serialPort, 0); //free running mode
    
    //Need to tune these parameters for Auto Gain
    unsigned long thresholds[3] = {25000, 1000, 10000};
    
    //flushBuffer(serialPort);
    //long rubbish[3000];
    //receivePacket(rubbish, 3000, serialPort, 2);
    //Acquisition Loop
    while (1) {
        for (int i = 0; i < 3; i++) {
            fp = fopen("/Users/daniel/Desktop/logFile.txt", "a");
            error = getSpectra(serialPort, specData);


            specReading.timestamp = timestamp_now();
            specReading.specData = specData;
            specReading.numSamples = 2048;
            specReading.intTime = INTTIME;
            exlcm_usb2000Spec_t_publish(lcm, "downSpec", &specReading);
            
            fprintf(fp, "%llu\t%u\t", specReading.timestamp, specReading.intTime);

            for (int j = 0; j < 2048; j++) {
                fprintf(fp, "%lu\t",specData[j]);
            }
            
            fprintf(fp, "\n");
            fclose(fp);
        }
        //check for correct gain every 3 samples
        long newIntTime = checkIntTime(specData, 2048, INTTIME, thresholds);
        if (newIntTime != INTTIME) {
            //we have a new Int time
            setIntTime(serialPort, newIntTime);
            INTTIME = newIntTime;
        }
        
        //Need some break condition
    }
    
    lcm_destroy(lcm);
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
//    for (int i = 0; i < 100; i++) {
//        specReading.timestamp = timestamp_now();
//        specReading.id = "Hello";
//        
//        exlcm_usb2000Spec_t_publish(lcm, "CHANNEL1", &specReading);
//        usleep(100000);
//        printf("LCM packet: %u time: %lld\n",i+1,specReading.timestamp );
//    }
    
    
    
    //start acq. for each sample
    //timestamp
    //record int time
    //acq spectra: ask for spectra and wait for h/w trigger (need timeout)
    //publish LCM packet
    //check signal level: if maxing out or too small, adjust int time
    
    
    //Clean up
    //delete lcm object

    
    
    
    
    return 0;
}

