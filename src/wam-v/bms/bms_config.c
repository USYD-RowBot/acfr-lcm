//This program configures all BMS with SAME settings.
//It is NECESSARY to configure the BMS EVERYTIME when it's turned on, this will assure the normal operation of BMS.
//To change the configuration, change the variables in the program.
//To run this program, type $./config

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
#define		OFFSET4			30

#include <stdio.h>
#include <stdlib.h>
#include <libpcan.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>

int main(void)
{
	DWORD status;
	TPCANMsg rev_config1, rev_config2;
	HANDLE h = LINUX_CAN_Open("/dev/pcan32", O_RDWR);	//You may need to change the device node '/dev/pcan32'
	int status_Init = CAN_Init(h, CAN_BAUD_500K, CAN_INIT_TYPE_ST);

	//CANID: 504	receive config part 1
	uint8_t cell_num = 14;

	uint8_t pack_cap = 105;		//unit: x10Ah

	uint8_t soc_warn = 20;

	uint8_t full_volt = 54;		//unit: V

	uint8_t current_warn = 15;	//unit: x10A

	uint8_t current_trip = 0;	//unit: x10A

	uint8_t temp_lim = 60;

	uint8_t min_cellvolt = 60;	//unit: x0.05V

	rev_config1.ID = RECEIVE_CONFIG1 + OFFSET1;
	rev_config1.MSGTYPE = MSGTYPE_STANDARD;
	rev_config1.LEN = 8;
	rev_config1.DATA[0] = cell_num;
	rev_config1.DATA[1] = pack_cap;
	rev_config1.DATA[2] = soc_warn;
	rev_config1.DATA[3] = full_volt;
	rev_config1.DATA[4] = current_warn;
	rev_config1.DATA[5] = current_trip;
	rev_config1.DATA[6] = temp_lim;
	rev_config1.DATA[7] = min_cellvolt;

	// 505, receive config part2	
	uint8_t max_cellvolt = 82;	//unit: x0.05V

	uint8_t hysteresis = 0;		//unit: x0.05V

	uint8_t shunt_threshold_volt = 82;	//unit: x0.05V

	uint8_t shunt_size = 1;		// 0 = no shunt ; 1 = 100A ; 2 = 200A ; 3 = 500A

	rev_config2.ID = RECEIVE_CONFIG2 + OFFSET1;
	rev_config2.MSGTYPE = MSGTYPE_STANDARD;
	rev_config2.LEN = 8;
	rev_config2.DATA[0] = max_cellvolt;
	rev_config2.DATA[1] = hysteresis;
	rev_config2.DATA[2] = shunt_threshold_volt;
	rev_config2.DATA[3] = shunt_size;	

	//---------------------------------------------------------------------------------------------
	//Configure BMS 1
	for(int i=0; i<20; i++)
	{
		status = CAN_Write(h, &rev_config1);
		printf("CAN Status 0: %d\n", status);
	}
	for(int j=0; j<20; j++)
	{
		status = CAN_Write(h, &rev_config2);
		printf("CAN Status 0: %d\n", status);
	}
	//Configure BMS 2
	rev_config1.ID = RECEIVE_CONFIG1 + OFFSET2;
	rev_config2.ID = RECEIVE_CONFIG2 + OFFSET2;
	for(int i=0; i<20; i++)
	{
		status = CAN_Write(h, &rev_config1);
		printf("CAN Status 1: %d\n", status);
	}
	for(int j=0; j<20; j++)
	{
		status = CAN_Write(h, &rev_config2);
		printf("CAN Status 1: %d\n", status);
	}
	//Configure BMS 3
	rev_config1.ID = RECEIVE_CONFIG1 + OFFSET3;
	rev_config2.ID = RECEIVE_CONFIG2 + OFFSET3;
	for(int i=0; i<20; i++)
	{
		status = CAN_Write(h, &rev_config1);
		printf("CAN Status 2:  %d\n", status);
	}
	for(int j=0; j<20; j++)
	{
		status = CAN_Write(h, &rev_config2);
		printf("CAN Status 2: %d\n", status);
	}
	//Configure BMS 4
	rev_config1.ID = RECEIVE_CONFIG1 + OFFSET4;
	rev_config2.ID = RECEIVE_CONFIG2 + OFFSET4;
	for(int i=0; i<20; i++)
	{
		status = CAN_Write(h, &rev_config1);
		printf("CAN Status 3: %d\n", status);
	}
	for(int j=0; j<20; j++)
	{
		status = CAN_Write(h, &rev_config2);
		printf("CAN Status 3: %d\n", status);
	}

	printf("All bms configuared with:\n");
	printf("Number of cells: %d\n", cell_num);
	printf("Pack capacity: %d\n", pack_cap * 10);
	printf("State of charge warning: %d%%\n", soc_warn);
	printf("Full voltage: %dV\n", full_volt);
	printf("Current warning: %d\n", current_warn * 10);
	printf("Current trip: %d\n", current_trip * 10);
	printf("Temperature limit: %d\n", temp_lim);
	printf("Minimum cell voltage: %f\n", (float)min_cellvolt * 0.05);
	printf("Maximum cell voltage: %f\n", (float)max_cellvolt * 0.05);
	printf("Hysteresis on cell voltage trips: %f\n", (float)hysteresis * 0.05);
	printf("Shunt balance threshold voltage: %f\n", (float)shunt_threshold_volt * 0.05);
	char* shunt_status;
	if(shunt_size == 0)
	{
		shunt_status = "No Shunt.";
	}
	else if(shunt_size == 1)
	{
		shunt_status = "100A";
	}
	else if(shunt_size == 2)
	{
		shunt_status = "200A";
	}
	else if(shunt_size == 3)
	{
		shunt_status = "500A";
	}
	printf("Shunt size: %s\n", shunt_status);

	return 0;
	CAN_Close(h);

}
