Hello, this is a brief manaul for BMS software use.
Before using the program, please make sure the USB/CAN Adapter driver is installed in the computer.

You may need to change the device node for USB/CAN Adapter, device node is '/dev/pcan32' used on a PC, it may be different on different PCs. The locations of device nodes in the programsare
	in ackerror.c	:	Ln 32
	in allOFF.c	:	ln 31
	in bms_config.c	:	ln 33
	in bms.c	:	ln 291
	in resetSOC.c	:	ln 31

To compile all program, type
	$ make

To clear all compiled files, type
	$ make clean

To configure all BMS with SAME configuration, type
	$ ./config
It is NECESSARY to configure the BMS to get correct data. Please run the ./config program EVERYTIME when BMS is turned ON.
To change configuration, go to bms_config.c and change the variables, make sure you use correct unit.

To take input from BMS through USB/CAN adapter and publish to LCM channel 'bmsCAN', type
	$ ./bms

To receive data form LCM channel 'bmsCAN', type
	$ ./receiver

To reset State of Charge to 100% for all BMS, type
	$ ./resetSOC

To acknowledge errors for all BMS, type
	$ ./ackerror

To turn OFF all BMS, type
	$ ./allOFF
You CAN NOT turn the BMS back ON by software.

To contact author, talk to Kai Zhang on #Slack or call 0424783573



