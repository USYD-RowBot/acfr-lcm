#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>

#include <lcm/lcm.h>

#include <perls-common/lcm_util.h>
#include "perls-common/timestamp.h"

#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_command_t.h"

#define MAXRPM  120	

struct termios orig_termios;

void reset_terminal_mode(){
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode(){
    struct termios new_termios;
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

int ready(){
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getchar(){
    int r;
    unsigned char c;
    if((r = read(0, &c, sizeof(c))) < 0){
        return r;
    } else {
        return c;
    }
}

int main(int argc, char *argv[]){
	lcm_t *lcm;
	lcm = lcm_create (NULL);
    	if(!lcm){
        	printf("lcm_create() failed!");
        	return -1;
    	}
	set_conio_terminal_mode();
	float vp = 0;
	float vs = 0; 
	char c = '0';
	while(c != 'q'){
		if(ready()){
			c = getchar();
			if(c == 'w'){
				vp = MAXRPM;
				vs = MAXRPM;
			}
			else if(c == 's'){
				vp = -1*MAXRPM;
				vs = -1*MAXRPM;
			}
			else if(c == 'a'){
				vs = MAXRPM;
				vp = -1*MAXRPM;
			}
			else if(c == 'd'){
				vs = -1*MAXRPM;
				vp = MAXRPM;
			}
			else{
				vs = 0;
				vp = 0;
			}
		
            // send motor command
            acfrlcm_auv_sirius_motor_command_t motor_command;

            motor_command.utime = timestamp_now();
            motor_command.vertical = 0;
            motor_command.port = vp;
            motor_command.starboard = vs;
            acfrlcm_auv_sirius_motor_command_t_publish(lcm, "SIRIUS_MOTOR", &motor_command);
		}
	}
}

