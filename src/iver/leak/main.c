#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   
#include <unistd.h>
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_leak_t.h"

#define LEAK_DEVICE "/dev/leak"
#define LEAK_PIN 0x10

typedef struct
{
    lcm_t *lcm;
    int leak_fd;
    int leak;
} state_t;

int main_exit;
void signal_handler(int sigNum) {
    // do a safe exit
    main_exit = 1;
}

void
leak_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    senlcm_leak_t leak;
    unsigned char pins;
    if(read(state->leak_fd, &pins, 1) == 1)
    {
        leak.utime = timestamp_now();
        if(!(pins & LEAK_PIN))
            state->leak = 1;
        
	leak.leak = state->leak;
        senlcm_leak_t_publish(state->lcm, "LEAK", &leak);
    }
}    
        
            
    
    

int main(int argc, char **argv)
{
    // install the signal handler
	main_exit = 0;
	signal(SIGINT, signal_handler);
    
    state_t state;
    state.leak = 0;
    // open the leak device file
    state.leak_fd = open(LEAK_DEVICE, O_RDONLY);
    if(state.leak_fd == -1)
    {
        printf("Could not open leak device %s\n", LEAK_DEVICE);
        return 0;
    }
    
    state.lcm = lcm_create(NULL);
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &leak_handler, &state);
    
    
    while (!main_exit) 
    {
        struct timeval tv;
	    tv.tv_sec = 1;
	    tv.tv_usec = 0;

        lcmu_handle_timeout(state.lcm, &tv);
    }
    
    close(state.leak_fd);
}
