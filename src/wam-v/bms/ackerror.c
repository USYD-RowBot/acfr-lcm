//This program tells all bms error is acknowledged, use by typing $./ackerror

#define		BMS_STATUS		500
#define		CELL_VOLTAGES1		501
#define		CELL_VOLTAGES2		502
#define		CELL_VOLTAGES3		503
#define		RECEIVE_CONFIG1		504
#define		RECEIVE_CONFIG2		505
#define		ACK_ERROR		506
#define		RESET_SOC		507
#define		POWER_OFF		508
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

int main(void)
{
	DWORD status;
	TPCANMsg ackerror;	
	
	HANDLE h = LINUX_CAN_Open("/dev/pcan32", O_RDWR);	//You may need to change the device node '/dev/pcan32'
	int status_Init = CAN_Init(h, CAN_BAUD_500K, CAN_INIT_TYPE_ST);

	ackerror.ID =ACK_ERROR + OFFSET1;
	ackerror.MSGTYPE = MSGTYPE_STANDARD;
	ackerror.LEN = 0;

	for(int i=0; i<20; i++)
	{
		status = CAN_Write(h, &ackerror);
	}
	printf("BMS %d Error Acknowledged.\n", (OFFSET1/10)+1);

	ackerror.ID = ACK_ERROR + OFFSET2;
	for(int i=0; i<20; i++)
	{
		status = CAN_Write(h, &ackerror);
	}
	printf("BMS %d Error Acknowledged.\n", (OFFSET2/10)+1);

	ackerror.ID = ACK_ERROR + OFFSET3;
	for(int i=0; i<20; i++)
	{
		status = CAN_Write(h, &ackerror);
	}
	printf("BMS %d Error Acknowledged.\n", (OFFSET3/10)+1);

	ackerror.ID = ACK_ERROR + OFFSET4;
	for(int i=0; i<20; i++)
	{
		status = CAN_Write(h, &ackerror);
	}
	printf("BMS %d Error Acknowledged.\n", (OFFSET4/10)+1);

	return 0;

}

