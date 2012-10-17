#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "perls-common/timestamp.h"
#include "perls-common/error.h"
#include "perls-common/generic_sensor_driver.h"

#include "perls-lcmtypes/senlcm_tcm_t.h"

#include "tcm.h"

#define update_rate 0.5

#define DTOR 3.141592/180
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
    
    for(int i=0; i< len; i++)
    	printf("%02X ", out[i] & 0xFF);
    printf("\n");
    return len;
}
    

int program_tcm(generic_sensor_driver_t *gsd)
{
    char data[32];
    char out[32];
    int len;
    
    // set the return types
    memset(data, 0, sizeof(data));
    data[0] = kSetDataComponents;
    data[1] = 4;
    data[2] = kHeading;
    data[3] = kPAngle;
    data[4] = kRAngle;
    data[5] = kTemperature;
    len = tcm_form_message(data, 6, out);    
    gsd_write(gsd, out, len);
    
    
    // set the acquisition mode
    memset(data, 0, sizeof(data));
    data[0] = kSetAcqParams;
    data[1] = 0;     // data push mode
    data[2] = 0;     // flush filter is false
    float interval = update_rate;
    memcpy(&data[7], &interval, sizeof(float));
    len = tcm_form_message(data, 11, out);    
    gsd_write(gsd, out, len);

    // start
    memset(data, 0, sizeof(data));
    data[0] = kStartIntervalMode;
    len = tcm_form_message(data, 1, out);    
    gsd_write(gsd, out, len);


    return 1;
}

int
main (int argc, char *argv[])
{
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_launch (gsd);
    gsd_noncanonical(gsd, 1024, 1);
    gsd_flush(gsd);
    gsd_reset_stats(gsd);
    
    int len;
    char buf[256];
    int64_t timestamp;
    senlcm_tcm_t tcm;
    
    program_tcm(gsd);
    
    while(1)
    {
    	
    
        // get two bytes
    	gsd_read(gsd, &buf[0], 1, &timestamp);
    	if(buf[0] != 0)
    	    continue;
    	gsd_read(gsd, &buf[1], 1, NULL);
    	    
   	    unsigned short data_len = (buf[0] << 8) + buf[1];

   	    // read the rest of the data
   	    if(data_len > 200)
            continue;

        len = 0; 
        while(len < data_len - 2)
            len += gsd_read(gsd, &buf[len + 2], data_len - 2 - len, NULL);

/*
	for(int i=0; i<data_len ; i++)
    	    printf("%02X ", buf[i] & 0xFF);
    unsigned short crc2 = tcm_crc(buf, data_len - 2);
    printf(", crc = %02X %02X\n", (crc2 >> 8) & 0xff, crc2 & 0xff);
    printf("\n");
*/

    	    
    	// check the checksum
    	unsigned short crc = *(unsigned short *)&buf[data_len - 2];
    	if(tcm_crc(buf, data_len-2) == (((crc >> 8) | (crc & 0xff) << 8)))
    	{
    	    memset(&tcm, 0, sizeof(senlcm_tcm_t));
    	    tcm.utime = timestamp;
    	    // its good data, lets parse it
            printf("%02X\n", buf[2]);
    	    if(parse_tcm(buf, &tcm))
            {
        	    senlcm_tcm_t_publish(gsd->lcm, gsd->channel, &tcm);
                gsd_update_stats (gsd, true);
            }
            else
                gsd_update_stats (gsd, false);
    	}
	    else
        {
	       printf("Bad CRC\n");
           gsd_update_stats (gsd, false);
        }
    }
    return 0;
    
    	
}
