/*	Commands		Can ID
	bms_status		500
	cell_voltages		501 502 503
	receive_config1		504
	receive_config2 	505
	battery_limits		849
	soc_and_health		853
	battery_details		854
	error_flags		858
*/

//This program takes input from bms through USB/CAN Adaptor and uploads to LCM channel 'bmsCAN'
//You may need to change the device nodes for USB/CAN Adaptor

#define		BMS_STATUS		500
#define		CELL_VOLTAGES1		501
#define		CELL_VOLTAGES2		502
#define		CELL_VOLTAGES3		503
#define		RECEIVE_CONFIG1		504
#define		RECEIVE_CONFIG2		505
#define		BATTERY_LIMITS		849
#define		SOC_AND_HEALTH		853
#define		BATTERY_DETAILS		854
#define		ERROR_FLAGS		858

#define		OFFSET1			0
#define		OFFSET2			10
#define		OFFSET3			20
#define		OFFSET4			3


#include <stdio.h>
#include <stdlib.h>
#include <libpcan.h>
#include <fcntl.h>
#include <lcm/lcm.h>
#include <string.h>

#include "bmsCAN_data.h"

int translate(bmsCAN_data *trans_msg, TPCANMsg *raw_msg)
{
	int offset = 0;
	int bmsNum = 0;

	if( (raw_msg->ID / 100 == 5 && raw_msg->ID % 100 < (10 + OFFSET1)) || (raw_msg->ID / 100 == 8 && raw_msg->ID % 100 < (59 + OFFSET1)) )
	{
		//printf("From BMS CAN BUS 1\n");
		offset = OFFSET1;
		bmsNum = 0;
	}
	else if( (raw_msg->ID / 100 == 5 && raw_msg->ID % 100 < (10 + OFFSET2)) || (raw_msg->ID / 100 == 8 && raw_msg->ID % 100 < (59 + OFFSET2)) )
	{
		//printf("From BMS CAN BUS 2\n");
		offset = OFFSET2;
		bmsNum = 1;
	}
	else if( (raw_msg->ID / 100 == 5 && raw_msg->ID % 100 < (10 + OFFSET3)) || (raw_msg->ID / 100 == 8 && raw_msg->ID % 100 < (59 + OFFSET3)) )
	{
		//printf("From BMS CAN BUS 3\n");
		offset = OFFSET3;
		bmsNum = 2;
	}
	else if( (raw_msg->ID / 100 == 5 && raw_msg->ID % 100 < (10 + OFFSET4)) || (raw_msg->ID / 100 == 8 && raw_msg->ID % 100 < (59 + OFFSET4)) )
	{
		//printf("From BMS CAN BUS 4\n");
		offset = OFFSET4;
		bmsNum = 3;
	}
	
	if(raw_msg->ID - offset == BMS_STATUS)
	{
		if(raw_msg->DATA[0] == 0)
		{
			trans_msg[bmsNum].error = "No error";
		}
		else if(raw_msg->DATA[0] == 1)
		{
			trans_msg[bmsNum].error = "Corrupt EEPROM";
		}
		else if(raw_msg->DATA[0] == 2)
		{
			trans_msg[bmsNum].error = "Undervoltage cell";
		}
		else if(raw_msg->DATA[0] == 3)
		{
			trans_msg[bmsNum].error = "Overvoltage cell";
		}
		else if(raw_msg->DATA[0] == 4)
		{
			trans_msg[bmsNum].error = "Low SoC";
		}
		else if(raw_msg->DATA[0] == 5)
		{
			trans_msg[bmsNum].error = "Overcurrent warning";
		}
		else if(raw_msg->DATA[0] == 6)
		{
			trans_msg[bmsNum].error = "Overcurrent shutdown";
		}
		else if(raw_msg->DATA[0] == 7)
		{
			trans_msg[bmsNum].error = "Over temperature";
		}
		else if(raw_msg->DATA[0] == 8)
		{
			trans_msg[bmsNum].error = "Critical Voltage";
		}

		trans_msg[bmsNum].soc = raw_msg->DATA[1];
		
		int voltage = raw_msg->DATA[2] + (raw_msg->DATA[3] << 8);
		trans_msg[bmsNum].voltage = (float)voltage/10;
		
		int current = raw_msg->DATA[4] + (raw_msg->DATA[5] << 8) - 32768;
		trans_msg[bmsNum].current = (float)current/10;

		trans_msg[bmsNum].temp = raw_msg->DATA[6] - 100;

	}
	else if(raw_msg->ID - offset == CELL_VOLTAGES1)
	{
		int cvolt = 0;

		cvolt = raw_msg->DATA[0] + ((raw_msg->DATA[6] & 0x01) << 8);
		trans_msg[bmsNum].cvolt1 = (float)cvolt/100;

		cvolt = raw_msg->DATA[1] + ((raw_msg->DATA[6] & 0x02) << 7);
		trans_msg[bmsNum].cvolt2 = (float)cvolt/100;

		cvolt = raw_msg->DATA[2] + ((raw_msg->DATA[6] & 0x04) << 6);
		trans_msg[bmsNum].cvolt3 = (float)cvolt/100;

		cvolt = raw_msg->DATA[3] + ((raw_msg->DATA[6] & 0x08) << 5);
		trans_msg[bmsNum].cvolt4 = (float)cvolt/100;

		cvolt = raw_msg->DATA[4] + ((raw_msg->DATA[6] & 0x10) << 4);
		trans_msg[bmsNum].cvolt5 = (float)cvolt/100;

		cvolt = raw_msg->DATA[5] + ((raw_msg->DATA[6] & 0x20) << 3);
		trans_msg[bmsNum].cvolt6 = (float)cvolt/100;
	}
	else if(raw_msg->ID - offset == CELL_VOLTAGES2)
	{
		int cvolt = 0;

		cvolt = raw_msg->DATA[0] + ((raw_msg->DATA[6] & 0x01) << 8);
		trans_msg[bmsNum].cvolt7 = (float)cvolt/100;

		cvolt = raw_msg->DATA[1] + ((raw_msg->DATA[6] & 0x02) << 7);
		trans_msg[bmsNum].cvolt8 = (float)cvolt/100;

		cvolt = raw_msg->DATA[2] + ((raw_msg->DATA[6] & 0x04) << 6);
		trans_msg[bmsNum].cvolt9 = (float)cvolt/100;

		cvolt = raw_msg->DATA[3] + ((raw_msg->DATA[6] & 0x08) << 5);
		trans_msg[bmsNum].cvolt10 = (float)cvolt/100;

		cvolt = raw_msg->DATA[4] + ((raw_msg->DATA[6] & 0x10) << 4);
		trans_msg[bmsNum].cvolt11 = (float)cvolt/100;

		cvolt = raw_msg->DATA[5] + ((raw_msg->DATA[6] & 0x20) << 3);
		trans_msg[bmsNum].cvolt12 = (float)cvolt/100;
	}
	else if(raw_msg->ID - offset == CELL_VOLTAGES3)
	{
		int cvolt = 0;

		cvolt = raw_msg->DATA[0] + ((raw_msg->DATA[6] & 0x01) << 8);
		trans_msg[bmsNum].cvolt13 = (float)cvolt/100;

		cvolt = raw_msg->DATA[1] + ((raw_msg->DATA[6] & 0x02) << 7);
		trans_msg[bmsNum].cvolt14 = (float)cvolt/100;
	}

	return bmsNum;

}

//----------------------------------------------------------------------------

int publish(bmsCAN_data *upload)
{
	lcm_t * lcm = lcm_create(NULL);
	if(!lcm)
	{
		return 1;
	}

	bmsCAN_data_publish(lcm, "bmsCAN", upload);

	lcm_destroy(lcm);
	return 0;
}



int main(void)
{
	DWORD status_Init , status_Read;
	TPCANMsg *raw_msg = malloc(sizeof(TPCANMsg));
	int bmsIdx;

	bmsCAN_data* trans_msg = (bmsCAN_data*)malloc(4 * sizeof(bmsCAN_data));

	//Initialize all contents to avoid segmentation fault
	trans_msg[0].bms = 500;
	trans_msg[0].error = "NA";
	trans_msg[0].soc = 0;
	trans_msg[0].voltage = 0;
	trans_msg[0].current = 0;
	trans_msg[0].temp = 0;
	trans_msg[0].cvolt1 = 0;
	trans_msg[0].cvolt2 = 0;
	trans_msg[0].cvolt3 = 0;
	trans_msg[0].cvolt4 = 0;
	trans_msg[0].cvolt5 = 0;
	trans_msg[0].cvolt6 = 0;
	trans_msg[0].cvolt7 = 0;
	trans_msg[0].cvolt8 = 0;
	trans_msg[0].cvolt9 = 0;
	trans_msg[0].cvolt10 = 0;
	trans_msg[0].cvolt11 = 0;
	trans_msg[0].cvolt12 = 0;
	trans_msg[0].cvolt13 = 0;
	trans_msg[0].cvolt14 = 0;

	trans_msg[1].bms = 510;
	trans_msg[1].error = "NA";
	trans_msg[1].soc = 0;
	trans_msg[1].voltage = 0;
	trans_msg[1].current = 0;
	trans_msg[1].temp = 0;
	trans_msg[1].cvolt1 = 0;
	trans_msg[1].cvolt2 = 0;
	trans_msg[1].cvolt3 = 0;
	trans_msg[1].cvolt4 = 0;
	trans_msg[1].cvolt5 = 0;
	trans_msg[1].cvolt6 = 0;
	trans_msg[1].cvolt7 = 0;
	trans_msg[1].cvolt8 = 0;
	trans_msg[1].cvolt9 = 0;
	trans_msg[1].cvolt10 = 0;
	trans_msg[1].cvolt11 = 0;
	trans_msg[1].cvolt12 = 0;
	trans_msg[1].cvolt13 = 0;
	trans_msg[1].cvolt14 = 0;

	trans_msg[2].bms = 520;
	trans_msg[2].error = "NA";
	trans_msg[2].soc = 0;
	trans_msg[2].voltage = 0;
	trans_msg[2].current = 0;
	trans_msg[2].temp = 0;
	trans_msg[2].cvolt1 = 0;
	trans_msg[2].cvolt2 = 0;
	trans_msg[2].cvolt3 = 0;
	trans_msg[2].cvolt4 = 0;
	trans_msg[2].cvolt5 = 0;
	trans_msg[2].cvolt6 = 0;
	trans_msg[2].cvolt7 = 0;
	trans_msg[2].cvolt8 = 0;
	trans_msg[2].cvolt9 = 0;
	trans_msg[2].cvolt10 = 0;
	trans_msg[2].cvolt11 = 0;
	trans_msg[2].cvolt12 = 0;
	trans_msg[2].cvolt13 = 0;
	trans_msg[2].cvolt14 = 0;

	trans_msg[3].bms = 530;
	trans_msg[3].error = "NA";
	trans_msg[3].soc = 0;
	trans_msg[3].voltage = 0;
	trans_msg[3].current = 0;
	trans_msg[3].temp = 0;
	trans_msg[3].cvolt1 = 0;
	trans_msg[3].cvolt2 = 0;
	trans_msg[3].cvolt3 = 0;
	trans_msg[3].cvolt4 = 0;
	trans_msg[3].cvolt5 = 0;
	trans_msg[3].cvolt6 = 0;
	trans_msg[3].cvolt7 = 0;
	trans_msg[3].cvolt8 = 0;
	trans_msg[3].cvolt9 = 0;
	trans_msg[3].cvolt10 = 0;
	trans_msg[3].cvolt11 = 0;
	trans_msg[3].cvolt12 = 0;
	trans_msg[3].cvolt13 = 0;
	trans_msg[3].cvolt14 = 0;

	HANDLE h = LINUX_CAN_Open("/dev/pcan32", O_RDWR);	//You may need to change the device node '/dev/pcan32'

	status_Init = CAN_Init(h, CAN_BAUD_500K, CAN_INIT_TYPE_ST);

	while(1)
	{
		status_Read = CAN_Read(h,raw_msg);

		bmsIdx = translate(trans_msg, raw_msg);

		publish(&trans_msg[bmsIdx]);

	}
	CAN_Close(h);
	return 0;
}
