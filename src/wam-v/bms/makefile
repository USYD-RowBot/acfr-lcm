bms.o receiver.o config.o allOFF.o resetSOC.o ackerror: bms.c bmsCAN_data.c receiver.c bms_config.c allOFF.c resetSOC.c ackerror.c
	gcc -o bms bms.c bmsCAN_data.c -llcm -lpcan
	gcc -o receiver receiver.c bmsCAN_data.c -llcm
	gcc -o config bms_config.c -lpcan
	gcc -o allOFF allOFF.c -lpcan
	gcc -o resetSOC resetSOC.c -lpcan
	gcc -o ackerror ackerror.c -lpcan
clean :
	rm bms receiver config allOFF resetSOC ackerror
