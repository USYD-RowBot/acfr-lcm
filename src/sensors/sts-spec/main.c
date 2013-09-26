//
//  main.c
//  Learning C
//
//  Created by Daniel Bongiorno on 21/08/13.
//  Copyright (c) 2013 Daniel Bongiorno. All rights reserved.
//

#include <stdio.h>
//#include "STSspecLibC.h"
#include <time.h>
#include "arduino-serial-lib.h"
#include <unistd.h>
#include "USB2000Lib.h"

int main(int argc, const char * argv[])
{
    char packet[64];
    long specData[1024];
    FILE *fp;

    //Testing Zone
    char inPack[] = {0x00,0x01,0x02,0x03,0x04,0x05};
    unsigned long *returnValue = 0;
    convertBytesToLong(&inPack[0], 2, returnValue);
    
    /*
    // insert code here...
    printf("Hello, World!\n");
    print_yay();
    
    makePacket(10, 0, packet);
    for (int i = 0; i<64; i++) {
        printf("%x\n",packet[i]);
    }
    
    //spectral packet
    char incomingP[] ={0xc1,0xc0,0x00,0x10,0x01,0x00,0x00,0x00,0x00,0x10,0x10,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        
        0x14,0x08,0x00,0x00,
        
        0x05,0x05,0x40,0x40,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,
        0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,0x40,0x05,

    
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xc5,0xc4,0xc3,0xc2};

    printf("%c\n",incomingP[44]);
    processPacket(incomingP,specData);
    
    //Debug
    for (int i=0; i<1024; i++) {
        printf("%u\t%lu\n",i,specData[i]);
    }

    */
    //try talking to a real spectrometer
    
    const char *port = "/dev/tty.PL2303-00001014";
    //char buffer[2112];
    
    int serialPort = serialport_init(port, 9600);
    
    //setBaud(serialPort, 9600);
    
    //usleep(500000);
    
    //serialPort = initSerial(port, 115200);
    
    /*
    uint8_t b = 0x10;
    serialport_writebyte(serialPort, b);
    
    serialport_writebyte(serialPort, 0x00);
    serialport_writebyte(serialPort, b);
    */
    
    flushBuffer(serialPort);
    //flushBuffer(serialPort);
    //flushBuffer(serialPort);
    //flushBuffer(serialPort);
    
    
    
     
    //setIntTime(serialPort, 100);
     /*
    usleep(500);
    char serialNum[16];
    //getSerialNum(serialPort, serialNum);
    //printf("Serial: %s\n",serialNum);
    //usleep(500);
    
    getSpectra(serialPort, specData);
    for (int i=0; i<1024; i++) {
        printf("%u\t%lu\n",i,specData[i]);
    }
    usleep(100);
    
    float temps[2];
    getTemp(serialPort, temps);
    for (int i = 0; i<2; i++) {
        printf("%u\t%f\n",i,temps[i]);
    }
      */
    //usleep(500);
    
    
    
    
    
    //float nonlinCoeff[8];
    //getNonlinCoeff(serialPort, nonlinCoeff);

    setIntTime(serialPort, 500000);
    usleep(100000);
    
    float temps[2];
    
    for (int i = 0; i < 15; i++) {
        fp = fopen("/Users/daniel/Documents/C Programming/Learning C/logFile.txt", "a");
        
        getTemp(serialPort, temps);
        fprintf(fp, "%.3f\t%.3f\t",temps[0],temps[1]);
        
        getSpectra(serialPort, specData);
        for (int j = 490; j < 510; j++) {
            fprintf(fp, "%lu\t",specData[j]);
        }
        fprintf(fp, "\n");
        printf("Just finished: %u\n",i);
        fclose(fp);
    }
    
//    
//    getSpectra(serialPort, specData);
//    printf("Mark");
//    getSpectra(serialPort, specData);
//    printf("Mark");
//    
    //fclose(fp);
    return 0;
}

