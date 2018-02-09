
//This program is a receving test program for LCM channel 'bmsCAN', it prints out whatever received

#include <stdio.h>
#include <stdlib.h>
#include <libpcan.h>
#include <fcntl.h>
#include <lcm/lcm.h>
#include <string.h>

#include "bmsCAN_data.h"


static void printout(const lcm_recv_buf_t *rbuf, const char * channel, 
        const bmsCAN_data * msg, void * user)
{
	printf("bms: %d\n", msg->bms);

	printf("error: %s\n", msg->error);

	printf("state of charge: %d\n", msg->soc);

	printf("total voltage: %f\n", msg->voltage);

	printf("current: %f\n", msg->current);

	printf("temperature: %dC\n", msg->temp);

	printf("cell voltage 1: %f\n", msg->cvolt1);
	
	printf("cell voltage 2: %f\n", msg->cvolt2);
	
	printf("cell voltage 3: %f\n", msg->cvolt3);
	
	printf("cell voltage 4: %f\n", msg->cvolt4);
	
	printf("cell voltage 5: %f\n", msg->cvolt5);
	
	printf("cell voltage 6: %f\n", msg->cvolt6);
	
	printf("cell voltage 7: %f\n", msg->cvolt7);
	
	printf("cell voltage 8: %f\n", msg->cvolt8);
	
	printf("cell voltage 9: %f\n", msg->cvolt9);
	
	printf("cell voltage 10: %f\n", msg->cvolt10);
	
	printf("cell voltage 11: %f\n", msg->cvolt11);
	
	printf("cell voltage 12: %f\n", msg->cvolt12);
	
	printf("cell voltage 13: %f\n", msg->cvolt13);
	
	printf("cell voltage 14: %f\n", msg->cvolt14);
	

}


int main()
{
	lcm_t * lcm = lcm_create(NULL);
	if(!lcm){
	    return 1;
	}

	bmsCAN_data_subscribe(lcm, "bmsCAN", &printout, NULL);


	while(1)
	{
        	lcm_handle(lcm);
	}

        lcm_destroy(lcm);
        return 0;
}
