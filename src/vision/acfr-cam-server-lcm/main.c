/*
 *  ACFR cam server
 *  Interface between the old tcpclient interface and LCM
 *
 *  Christian Lees
 *  ACFR
 *  4/11/11
 *
 */

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<errno.h>
#include<sys/types.h>
#include <signal.h>

#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_trigger_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"

// command messages
#define SET_FREQ				1
#define SET_DELAY				2
#define	SET_WIDTH				3
#define SET_STATE		        4
#define SET_ALL					5

typedef struct
{
    lcm_t *lcm;
    char dir[256];
    pthread_mutex_t *lock;
    int started;
} state_t;

int program_exit;

void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGINT)
        program_exit = 1;

}

void mp_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_raw_ascii_t *msg, void *u)
{
    float f_value;
    char command[256];
    char *ptr;

    state_t *state = (state_t *)u;

    acfrlcm_auv_camera_trigger_t ct;
    memset(&ct, 0, sizeof(acfrlcm_auv_camera_trigger_t));

    if(strstr(msg->msg, "VIS") != NULL)
    {
        // it is a message for us
        if(strstr(msg->msg, "start") != NULL)
        {
            if(!state->started)
            {
                // create the directory
                sprintf(command, "mkdir -p %s\n", state->dir);
                printf("Creating directory %s\n", state->dir);
                system(command);
                system("killall acfr-cam-logger");
                sprintf(command, "acfr-cam-logger -e PROSILICA_..16 -o %s &\n", state->dir);
                printf("Starting logger with command: %s", command);
                printf("Command: %s", command);
                system(command);

                ct.command = SET_STATE;
                ct.enabled = 1;
                ct.utime = timestamp_now();
                acfrlcm_auv_camera_trigger_t_publish(state->lcm, "CAMERA_TRIGGER", &ct);
                state->started = 1;
            }
        }

        if((ptr = strstr(msg->msg, "savedir='")) != NULL)
        {
            // we have a save dir command
            int parse_state = 1;
            int i=0;
            ptr += strlen("savedir='");
            while( (*ptr != '\'') && parse_state )	//0x60 -> hex for '
            {
                if( (*ptr == 0) || (i >= sizeof(state->dir)) )	//Null char or buffer len overrun
                {
                    parse_state = 0;
                    i=0;
                }
                else
                    state->dir[i++] = *ptr++;	//copy until the delimiter

            }
            // check to see that the dir has a slash on the end
            if(state->dir[strlen(state->dir)] != '/')
                state->dir[i] = '/';
            state->dir[++i] = 0;
            printf("Save dir set to %s\n", state->dir);
        }

        if(strstr(msg->msg, "stop") != NULL)
        {
            ct.command = SET_STATE;
            ct.enabled = 0;
            ct.utime = timestamp_now();
            acfrlcm_auv_camera_trigger_t_publish(state->lcm, "CAMERA_TRIGGER", &ct);
            system("killall acfr-cam-logger");
            state->started = 0;
        }

        if((ptr = strstr(msg->msg, "f=")) != NULL)
        {
            sscanf(ptr, "%f", &f_value);
            ct.command = SET_FREQ;
            ct.freq = f_value;
            ct.utime = timestamp_now();
            acfrlcm_auv_camera_trigger_t_publish(state->lcm, "CAMERA_TRIGGER", &ct);
        }
    }
}



int main()
{
    state_t state;
    state.started = 0;

    // set a default save dir
    strcpy(state.dir, "/media/data/itest/");

    program_exit = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);

    // start LCM
    state.lcm = lcm_create(NULL);

    // subscribe to the required lcm messages
    senlcm_raw_ascii_t_subscribe(state.lcm, "MP_PASSOUT", &mp_callback, &state);

    struct timeval tv;

    while(!program_exit)
    {
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        lcmu_handle_timeout(state.lcm, &tv);
    }

    printf("exiting\n");
    // close up everything

    return 0;
}

