
#include <stdio.h>
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

typedef struct
{
    char return_string[256];
    int got_string;
} state_t;

static void callback(const lcm_recv_buf_t *rbuf, const char *channel, const senlcm_raw_ascii_t *buf, void *user)
{
    state_t *state = (state_t *)user;
    strcpy(state->return_string, buf->msg);
    state->got_string = 1;
}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        printf("Usage: send-string [LCM channel] [timeout in us]\n");
        return 0;
    }
    state_t state;
    state.got_string = 0;
    long long start_time;
    
    int timeout = atoi(argv[2]);
    lcm_t *lcm = lcm_create(NULL);
    senlcm_raw_ascii_t_subscribe(lcm, argv[1], &callback, &state);
    
    while(!(state.got_string) && (timeout > 0))
    {
        struct timeval tv;
	    tv.tv_sec = timeout / 1000000;
	    tv.tv_usec = timeout % 1000000;
        start_time = timestamp_now();
        lcmu_handle_timeout(lcm, &tv);
        timeout -= (timestamp_now() - start_time);
    }
    
    if(state.got_string)
        printf("%s", state.return_string);
        
    lcm_destroy(lcm);
    
    return 1;
}
    
    
