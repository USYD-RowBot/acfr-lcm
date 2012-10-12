#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/timestamp.h"
#include "perls-common/error.h"
#include "perls-common/generic_sensor_driver.h"

#include "perls-lcmtypes/senlcm_tcm_t.h"

#include "tcm.h"

#define update_rate 1.0


static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "PNI TCM Fieldforce compass driver.");
    return 0;
}


int parse_tcm(char *buf, senlcm_tcm_t *tcm)
{
    // decode the tcm frame
    char frame_id = buf[2];
    
    switch frame_id
    {
        case kDataResp:
            int count = (int)buf[3];
            current_pos = &buf[4];
            index = 0;
            while(1)
            {
                int data_id = (int)buf[current_pos++];
         
                switch data_id
                {
                    case kHeading:
                        tcm->heading = *(float *)buf[current_pos];
                        current_pos += 4;
                        index++;
                        break;    
                    case kPAngle:
                        tcm->pitch = *(float *)buf[current_pos];
                        current_pos += 4;
                        index++;
                        break;
                    case kRAngle:
                        tcm->roll = *(float *)buf[current_pos];
                        current_pos += 4;
                        index++;
                        break
                    case kTemperature:
                        tcm->temp = *(float *)buf[current_pos];
                        current_pos += 4;
                        index++;
                        break
                }
                if(index == count)
                    break;
    } 
    return 1;            
             
}

int tcm_form_message(char *in, int data_len, char *out)
{
    unsigned short len = data_len + 2;
    memcpy(out, len, 2);
    memcpy(in, out, data_length);
    unsigned short crc = tcm_crc(out, len);
    memcpy(&out[data_len], crc, 2);
    return len + 2;
}
    

int program_tcm(generic_sensor_driver_t *gsd)
{
    char data[32];
    char out[32];
    int len;
    
    // set the return types
    memset(data, 0, sizeof(buf));
    data[0] = kSetDataComponents;
    data[1] = 4;
    data[2] = kHeading;
    data[3] = kPAngle;
    data[4] = kRAngle;
    data[5] = kTemperature;
    len = tcm_form_message(data, 6, out);    
    gsd_write(gsd, out, len)
    
    
    // set the acquisition mode
    memset(data, 0, sizeof(buf));
    data[0] = kSetAcqParams;
    data[1] = 0;     // data push mode
    data[2] = 0;     // flush filter is false
    float interval = update_rate;
    memcpy(&data[7], interval, sizeof(float));
    len = tcm_form_message(data, 11, out);    
    gsd_write(gsd, out, len);

    return 1;
}

int
main (int argc, char *argv[])
{
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_launch (gsd);
    
    int len;
    char buf[256];
    int64_t timestamp;
    senlcm_tcm_t tcm;
    
    program_tcm(gsd);
    
    while(!gsd->done)
    {
        // get two bytes
        len = 0; 
        while(len < 2)
    	    len += gsd_read(gsd, &buf[len], 2 - len, &timestamp);
    	    
    	int data_len = *(unsigned short *)buf;
    	
    	// read the rest of the data
        len = 0; 
        while(len < data_len)
    	    len += gsd_read(gsd, &buf[len + 2], data_len - len, NULL);
    	    
    	// check the checksum
    	unsigned short crc = *(unsigned short *)&buf[data_len];
    	if(tcm_crc(buf, data_len) == crc)
    	{
    	    memcpy(tcm, 0, sizeof(senlcm_tcm_t))
    	    tcm.utime = timestamp;
    	    // its good data, lets parse it
    	    if(parse_tcm(buf, tcm))
        	    senlcm_tcm_t_publish(gsd->lcm, gsd->channel, &tcm);
    	}
    }
    return 0;
}
    	
