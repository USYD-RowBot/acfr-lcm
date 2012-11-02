/*	Ocean server power system monitoring LCM module
	Christian Lees
	28/5/11
    
    13/1/12
    updated to use sockets as user on Sirius
    moved away from using the gsd code
*/


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <signal.h>
#include <libgen.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <bot_param/param_client.h>

#include "perls-common/serial.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes/senlcm_os_power_system_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"


#define MAX_MESSAGE_SIZE 128


#define MAX_CONTS 8
#define MAX_BUF_LEN 256

#define BATT_VOLTAGE 14.4

enum {io_socket, io_serial};

typedef struct 
{
    lcm_t *lcm;
    int num_devs;
	senlcm_os_power_system_t ps;
	int initialised[MAX_CONTS];
	int batt_present[MAX_CONTS][8];
} state_t;

/*
int
battery_validate_checksum (const char *buf)
{
    int cs;
    if (1 != sscanf (buf, "$%*[^\%]s\%%2X", &cs)) // invalid parse
        return -1;

    int checksum = nmea_compute_checksum (buf);
    if (cs == checksum)
        return 0;
    else
        return -1;
}
*/


/* correctly handles empty NMEA field */
int
battery_arg (const char *str, int n, char out[])
{
    const char *c1, *c2, *c;
    
    c = str;
    // find a delim
    while ((*c != ',') && (*c != '%') && (*c != '\0'))
        c++;    // $CCCCC,
    c++;        // skip the comma

    // for preceding args
    for (int i=0; i<(n-1); i++) {
        while ((*c != ',') && (*c != '%') && (*c != '\0'))
            c++; // skip preceding args
        c++;     // skip comma
    }
    char pa = *(c-1);
    if ( pa == '%' || pa == '\0') {
        return 0;    
    }
    
    c1 = c; // points to first char of arg we want, or next comma
    while ((*c != ',') && (*c != '%') && (*c != '\0'))
        c++;     // skip to end
    c2 = c - 1;  // points to last char before comma

    if (c1 <= c2) {
        out[0] = '\0';
        strncat (out, c1, (c2-c1+1));
        return 1;
    }
    else {
        out = NULL;
        return 0;
    }       
}



void init_controller(int fd) {
	// put the controller in a known state
	write(fd, " ", 1);
	write(fd, "x", 1);
}

unsigned short ahtoi(char *in) {

    unsigned short result = 0;
    int len = strlen(in);
    int i;
    
    for(i=0; i<len; i++) {
        result = result << 4;
        if((in[i] >= 48) && (in[i] <=57))
            result += (in[i]-48);
        if((toupper(in[i]) >= 65) && ((toupper(in[i]) <=70)))
            result += (toupper(in[i])-65)+10;
    }
    
    return result;
}

int parse_os_controller(char *buf, state_t *state, int cont_num) {
	
	// parse the line and fill in the relevant data
//    int checksum = battery_validate_checksum(buf);
//    printf("Checksum =  %02X, %d\n", checksum, checksum);
//    if(checksum == -1)
//		return 0;
		
	
	int i = 1;
	char field[4], value[32];
	unsigned short valueI, fieldI, tmp;
	// the data is valid, lets see what it is
	if(!strncmp(buf, "$S", 2)) {
		// system data
		while(battery_arg(buf, i++, field)) {
			battery_arg(buf, i++, value);
			fieldI = ahtoi(field);
			
			if(fieldI == 1)
				state->ps.minutes_tef = (int)ahtoi(value);
			if(fieldI == 3) 
				strcpy(state->ps.controller[cont_num].sys_message, value);
			if(fieldI == 4) 
				state->ps.controller[cont_num].avg_charge_p = (int)ahtoi(value);
		}
	}
	
	if(!strncmp(buf, "$C", 2)) {
		// controller message
        memset(state->ps.controller[cont_num].battery_state, 0, 9);
		while(battery_arg(buf, i++, field)) {
			battery_arg(buf, i++, value);
			fieldI = ahtoi(field);
			switch(fieldI) {
				case 1:
				    if(!state->initialised[cont_num]) {
					    // which batteries are present
					    valueI = ahtoi(value);
					    valueI &= 0x00FF;
                        tmp = valueI;
					    int j = 0;
					    for(int k=0; k<8; k++) {
						    if(valueI & 0x001)
                            {
							    state->batt_present[cont_num][k] = j++;
                            }
						    else
                            {
							    state->batt_present[cont_num][k] = -1;
                            }
						    valueI = valueI >> 1;
					    }
                        state->ps.controller[cont_num].num_batteries = j;
                        
					    state->ps.controller[cont_num].battery = (senlcm_smartbattery_t *)malloc(sizeof(senlcm_smartbattery_t) * state->ps.controller[cont_num].num_batteries);
//    					for(int k=0; k<state->ps.controller[cont_num].num_batteries; k++)
//    					    state->ps.controller[cont_num].battery[i].status = (char *)malloc(sizeof();
					    state->initialised[cont_num] = 1;
					}
                    
                    for(int j=0; j<8; j++)
                    {
                        if(tmp & 0x01)
                            state ->ps.controller[cont_num].battery_state[j] = '.';
                        else
                            state ->ps.controller[cont_num].battery_state[j] = '^';
                        tmp = tmp >> 1;
                    }    
                    
					break;
				case 2:
					// which batteries are charging
                    valueI = ahtoi(value);
					valueI &= 0x00FF;
                    for(int j=0; j<8; j++)
                    {
                        if(valueI & 0x01)
                            state->ps.controller[cont_num].battery_state[j] = 'C';
                        valueI = valueI >> 1;
                    }    
					break;
                case 3:
					// which batteries are charging
                    valueI = ahtoi(value);
					valueI &= 0x00FF;
                    for(int j=0; j<8; j++)
                    {
                        if(valueI & 0x01)
                            state->ps.controller[cont_num].battery_state[j] = 'D';
                        valueI = valueI >> 1;
                    }    
					break;
                case 6:
					// which batteries are charging
                    valueI = ahtoi(value);
					valueI &= 0x00FF;
                    for(int j=0; j<8; j++)
                    {
                        if(valueI & 0x01)
                            state->ps.controller[cont_num].battery_state[j] = 'F';
                        valueI = valueI >> 1;
                    }    
					break;
			}
		}
	}
	
	if(!strncmp(buf, "$B", 2) && state->initialised[cont_num]) {
		// individual battery information, first get the battery number
		int batt_num;
		battery_arg(buf, 0, field);
		sscanf(buf, "$B%d,%*s", &batt_num);
		batt_num = batt_num % 10;	// strip of the controller number, one controller only
		senlcm_smartbattery_t *batt = &state->ps.controller[cont_num].battery[state->batt_present[cont_num][batt_num-1]];
		while(battery_arg(buf, i++, field)) {
			battery_arg(buf, i++, value);
			fieldI = ahtoi(field);
			valueI = ahtoi(value);
			switch(fieldI) {
				case 0x08:	// temperature (deg c)
					batt->temperature = ((double)valueI * 0.1) - 273.0;
					break;
				case 0x09:	// voltage (V)
					batt->voltage = (double)valueI / 1000;
					break;
				case 0x0A:	// current (A)
					batt->current = ((double)(*(short *)&valueI)) / 1000;
					break;
				case 0x0B:	// average current (A)
					batt->avg_current = ((double)(*(short *)&valueI)) / 1000;
					break;
				case 0x0F:	// remaining capacity (Ah)
					batt->remaining_capacity = ((double)(*(short *)&valueI)) / 1000;
					break;
				case 0x10:	// full capacity (Ah)
					batt->full_capacity = ((double)(*(short *)&valueI)) / 1000;
					break;
				case 0x0D:	// charge state (%)
					batt->charge_state = (int)valueI;
					break;
				case 0x12:	// average time to empty
					batt->avg_tte = (int)valueI;
					break;
				case 0x13:	// average time to full
					batt->avg_ttf = (int)valueI;
					break;
				case 0x17:	// cycles
					batt->cycles = (int)valueI;
					break;
				case 0x1C:	// serial number
					batt->serial_num = (int)valueI;
					break;
				case 0x16:	// status
					// todo
					break;
			}
		}				
	    
	}
	return 1;
}					

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    int initialised = 1;
    
    for(int j=0; j<state->num_devs; j++) 
    {
        if(state->initialised[j] == 0)
            initialised = 0;
    }
    if(initialised)
    {
        
        state->ps.num_controllers = state->num_devs;
        state->ps.utime = timestamp_now();
        state->ps.current = 0;
        state->ps.power = 0;
        state->ps.capacity = 0;
        state->ps.capacity_full = 0;
        state->ps.minutes_tef = 0;
        state->ps.avg_charge_p = 0;

        int minutes_tte;
        int minutes_ttf;
        int tte_good;
        int ttf_good;
        
        for(int j=0; j<state->num_devs; j++)
        {
            // Capacities in watts
            state->ps.controller[j].capacity = 0;
            state->ps.controller[j].capacity_full = 0;  
            state->ps.controller[j].current = 0;
            state->ps.controller[j].power = 0; 
            state->ps.controller[j].minutes_tef = 0;         
            minutes_tte = 0;
            minutes_ttf = 0;
            tte_good = 1;
            ttf_good = 1;
            for(int i=0; i<state->ps.controller[j].num_batteries; i++)
            {
                state->ps.controller[j].capacity += state->ps.controller[j].battery[i].remaining_capacity * BATT_VOLTAGE;
                state->ps.controller[j].capacity_full += state->ps.controller[j].battery[i].full_capacity * BATT_VOLTAGE;
                state->ps.controller[j].current += state->ps.controller[j].battery[i].current;
                

                if(state->ps.controller[j].battery[i].avg_tte == 65535)
                    tte_good = 0;
                minutes_tte += state->ps.controller[j].battery[i].avg_tte;

                if(state->ps.controller[j].battery[i].avg_ttf == 65535)
                    ttf_good = 0;
                minutes_ttf += state->ps.controller[j].battery[i].avg_ttf;
                state->ps.controller[j].power += state->ps.controller[j].battery[i].current * state->ps.controller[j].battery[i].voltage;            
                
            }

            if(ttf_good)
                state->ps.controller[j].minutes_tef = minutes_ttf / state->ps.controller[j].num_batteries;
            else if(tte_good)
                state->ps.controller[j].minutes_tef = minutes_tte / state->ps.controller[j].num_batteries;
            else
                state->ps.controller[j].minutes_tef = -1;
                
            state->ps.current += state->ps.controller[j].current;
            state->ps.power += state->ps.controller[j].power;
            state->ps.capacity += state->ps.controller[j].capacity;
            state->ps.capacity_full += state->ps.controller[j].capacity_full;
            state->ps.minutes_tef += state->ps.controller[j].minutes_tef;
            state->ps.avg_charge_p += state->ps.controller[j].avg_charge_p;
            
        } 
        state->ps.avg_charge_p = state->ps.avg_charge_p / state->num_devs;
        state->ps.minutes_tef = state->ps.minutes_tef / state->num_devs;

        senlcm_os_power_system_t_publish (state->lcm, "BATTERY", &state->ps);
    }

}

int readline(int fd, char *buf, int max_len)
{
    int i=0;
    do
    {
        read(fd, &buf[i++], 1);
    } while(buf[i-1] != '\n');
}
				

int program_exit;
void 
signal_handler(int sigNum) 
{
    // do a safe exit
    program_exit = 1;
}

int main (int argc, char *argv[]) {
		
	state_t state;

    
    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);   
    
    
    // read the config file
    BotParam *param;
	char rootkey[64];
	char key[64];
	
    state.lcm = lcm_create(NULL);
    param = bot_param_new_from_server (state.lcm, 1);
    
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
    
    // read the config file
    char *io_str;
    int io;
	sprintf(key, "%s.io", rootkey);
	io_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(io_str, "serial"))
        io = io_serial;
    else if(!strcmp(io_str, "socket"))
        io = io_socket;
    
    char **serial_devs;
    char **inet_ports;
    char *ip;
    int baud;
    char *parity;
    
    sprintf(key, "%s.num_devs", rootkey);
    state.num_devs = bot_param_get_int_or_fail(param, key);
    
    if(io == io_serial)
    {
        sprintf(key, "%s.serial_devs", rootkey);
        serial_devs = bot_param_get_str_array_alloc(param, key);

    	sprintf(key, "%s.baud", rootkey);
	    baud = bot_param_get_int_or_fail(param, key);

	    sprintf(key, "%s.parity", rootkey);
	    parity = bot_param_get_str_or_fail(param, key);
    }
    
    if(io == io_socket)
    {
        sprintf(key, "%s.ip", rootkey);
        ip = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.ports", rootkey);
        inet_ports = bot_param_get_str_array_alloc(param, key);
    }
    
    
    
    int lcm_fd = lcm_get_fileno(state.lcm);    
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    
    // open the ports
    struct addrinfo hints, *batt_addr;
    int *batt_fd = (int *)malloc(sizeof(int) * state.num_devs);
    for(int i=0; i<state.num_devs; i++)
    {
        if(io == io_serial)
        {
            batt_fd[i] = serial_open(serial_devs[i], serial_translate_speed(baud), serial_translate_parity(parity), 1);
            if(batt_fd[i] < 0)
            {
                printf("Error opening port %s\n", serial_devs[i]);
                return 0;
            }
        }        
        else if(io == io_socket)
        {
            memset(&hints, 0, sizeof hints);
	        hints.ai_family = AF_UNSPEC;
	        hints.ai_socktype = SOCK_STREAM;
	        getaddrinfo(ip, inet_ports[i], &hints, &batt_addr);
        	batt_fd[i] = socket(batt_addr->ai_family, batt_addr->ai_socktype, batt_addr->ai_protocol);
	        if(connect(batt_fd[i], batt_addr->ai_addr, batt_addr->ai_addrlen) < 0) 
            {
		        printf("Could not connect to %s on port %s\n", ip, inet_ports[i]);
        		return 1;
	        }
        
        }
        
        state.initialised[i] = 0;
    }
    
    
    // now we are open we can put all the controllers in the right mode
    for(int i=0; i<state.num_devs; i++)
        init_controller(batt_fd[i]);        
    
    // allocate the battery controller memory
    state.ps.controller = (senlcm_os_power_cont_t *)malloc(state.num_devs * sizeof(senlcm_os_power_cont_t));
    for(int i=0; i<state.num_devs; i++)
    {
        state.ps.controller[i].sys_message = (char *)malloc(64);
        state.ps.controller[i].battery_state = (char *)malloc(9);
    }
    

    fd_set rfds;
    char buf[MAX_BUF_LEN];
    int64_t timestamp;
       
    // loop to collect data, parse and send it on its way
    while(!program_exit) 
    {
        FD_ZERO(&rfds);
        FD_SET(lcm_fd, &rfds);
        for(int i=0; i<state.num_devs; i++)
            FD_SET(batt_fd[i], &rfds);
	
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
	    
	    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm_handle(state.lcm);
            else
            {
                for(int i=0; i<state.num_devs; i++)
                    if(FD_ISSET(batt_fd[i], &rfds))
                    {                    
                        readline(batt_fd[i], buf, MAX_BUF_LEN);
                        timestamp = timestamp_now();
                        if(parse_os_controller(buf, &state, i)) 
                        {   
                            state.ps.controller[i].utime = timestamp;        
                        }
                    }
            }
	    }
    }
    
    // lets exit
    for(int i=0; i<state.num_devs; i++)
    {    
        close(batt_fd[i]);
        free(state.ps.controller[i].battery);
    }
    free(state.ps.controller);
    
}

