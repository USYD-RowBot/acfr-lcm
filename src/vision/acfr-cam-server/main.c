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
#include<sys/socket.h>
#include<sys/select.h>
#include<netinet/in.h>
#include <signal.h>
#include <netdb.h>
#include<fcntl.h>
#include<pthread.h>

#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_trigger_t.h"
#include "perls-lcmtypes/acfrlcm_auv_vis_rawlog_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#define PORT 20000

// command messages
#define SET_FREQ				1
#define SET_DELAY				2
#define	SET_WIDTH				3
#define SET_STATE		        4
#define SET_ALL					5

typedef struct 
{
    lcm_t *lcm;
    int connected;
    int sock_fd;
    char dir[256];
    pthread_mutex_t *lock;
} state_t;

int program_exit;
int connected;

void signal_handler(int sig_num) 
{
    // do a safe exit
    if(sig_num == SIGINT)
        program_exit = 1;
    if(sig_num == SIGPIPE)
    {
        connected = 0;
    }
}

// Process LCM messages with callbacks
static void *lcm_thread (void *context) {
    lcm_t *lcm = (lcm_t *) context;

    while (!program_exit) {
        struct timeval tv;
	    tv.tv_sec = 1;
	    tv.tv_usec = 0;

        lcmu_handle_timeout(lcm, &tv);
    }
    return 0;
}

int encode_message(char *in, char *out)
{
    sprintf(out, "%02i-%s", strlen(in), in);
    return 0;
}

int send_log_message(state_t *state, char *msg)
{
    char msg_out[256];
    encode_message(msg, msg_out);
    pthread_mutex_lock(state->lock);
    send(state->sock_fd, msg_out, strlen(msg_out), 0);
    pthread_mutex_unlock(state->lock);
    
    return 0;
}

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    
    // send out a beat msg once a second
    send_log_message(state, "\0");
}


void vis_rawlog_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_vis_rawlog_t *rl, void *u)
{
    state_t *state = (state_t *)u;
    
    // we are going to send back the entry that ends up in the main log file
    // the format is VIS: [timestamp] [image_name] exp: [exposure time]
    
    char out_str[128];
    sprintf(out_str, "[%f] %s exp: %d\0", rl->utime/1e6, rl->image_name, rl->exp_time);
    send_log_message(state, out_str);
    
}



int parse_message(char *data, state_t *state)
{
    int num_msg, msg_len;
    char msg[256];
    float f_value;
    char command[256];
    
    acfrlcm_auv_camera_trigger_t ct;
    memset(&ct, 0, sizeof(acfrlcm_auv_camera_trigger_t));  

    
    num_msg = sscanf(data, "%u-%s", &msg_len, msg);

    if(!strncmp(msg, "start", 5))
    {
        // create the directory
        sprintf(command, "mkdir -p %s\n", state->dir);
        system(command);
        system("killall acfr-cam-logger");
        sprintf(command, "/home/auv/perls/bin/acfr-cam-logger -e PROSILICA_..16 -o %s &\n", state->dir);
        printf("Command: %s", command);
        system(command);
    
        ct.command = SET_STATE;
        ct.enabled = 1;
        ct.utime = timestamp_now();
        acfrlcm_auv_camera_trigger_t_publish(state->lcm, "CAMERA_TRIGGER", &ct);
        send_log_message(state, "Capture Start... [SUCCESS]");
        return 1;
    }
        
        
    else if(!strncmp(msg, "stop", 4))
    {
        ct.command = SET_STATE;
        ct.enabled = 0;
        ct.utime = timestamp_now();
        acfrlcm_auv_camera_trigger_t_publish(state->lcm, "CAMERA_TRIGGER", &ct);
        send_log_message(state, "Capture Stop... [SUCCESS]");
        system("killall acfr-cam-logger");
        return 1;
    }    

    else if(!strncmp(msg, "f=", 2))
    {
        sscanf(msg, "f=%f", &f_value);
        ct.command = SET_FREQ;
        ct.freq = f_value;
        ct.utime = timestamp_now();
        acfrlcm_auv_camera_trigger_t_publish(state->lcm, "CAMERA_TRIGGER", &ct);
        send_log_message(state, "External Trigger SetFreq [SUCCESS]");
        return 1;

    }

    else if(!strncmp(msg, "savedir=", 8))
	{
        int parse_state = 1;
        int i=0;
        memset(state->dir, 0, sizeof(state->dir));
		char *ptr = strstr(msg, "savedir='");
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
        // check to see that the dir has a slah on the end
        if(state->dir[strlen(state->dir)] != '/')
            state->dir[i] = '/';
            
        printf("Save dir set to %s\n", state->dir);
        return 1;

    } 
    else       
        return 0;
}
        


int main()
{
    int sock_fd, new_fd;  // listen on sock_fd, new connection on new_fd
    struct sockaddr_storage their_addr; // connector's address information
    socklen_t addr_size;

    state_t state;
    state.connected = 0;
    state.lock = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init(state.lock, NULL);
    
    // set a default save dir
    strcpy(state.dir, "/media/data/itest/");

    program_exit = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);
    
    
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);
    memset(&(server_addr.sin_zero), 0, 8);
    

    if(bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        perror("Bind");
        return 1;
    }

    if(listen(sock_fd, 10) != 0)
    {
        perror("listen");
    }
    
    
    // start LCM
    state.lcm = lcm_create(NULL);

    // create an LCM thread to listen to stuff
    pthread_t tid;
    pthread_create(&tid, NULL, lcm_thread, state.lcm);
    pthread_detach(tid);	
    
    // subscribe to the required lcm messages
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    acfrlcm_auv_vis_rawlog_t_subscribe(state.lcm, "ACFR_AUV_VIS_RAWLOG", &vis_rawlog_handler, &state);
    
    // now accepting tcp connections
    struct timeval tv;
    fd_set sock_fds, read_fds;
    char buffer[256];
    int bytes_read;
    
    
    while(!program_exit)
    {   
        FD_ZERO(&sock_fds);
        FD_SET(sock_fd, &sock_fds);
        tv.tv_sec = 2;
        tv.tv_usec = 0;    
        connected = 0;
        if(select(sock_fd+1, &sock_fds, NULL, NULL, NULL) > 0)
        {
            // we have a connection
            printf("cam server: Got a connection\n");
            addr_size = sizeof their_addr;
            new_fd = accept(sock_fd, (struct sockaddr *)&their_addr, &addr_size);
            state.sock_fd = new_fd;
            connected = 1;
            // we are only going to accept one connection so there is no need to thread
            while(!program_exit && connected)
            {
                FD_ZERO(&read_fds);
                FD_SET(new_fd, &read_fds);

                tv.tv_sec = 2;
                if(select(new_fd+1, &read_fds, NULL, NULL, NULL) > 0)
                {
                    // we have data to read
                    memset(buffer, 0, sizeof(buffer));
                    bytes_read = read(new_fd, buffer, sizeof(buffer));
                    parse_message(buffer, &state);
                    FD_SET(new_fd, &read_fds);
                }
            }
            printf("cam server: Connection closed\n");
        }
    }
    printf("exiting\n");
    // close up everything
    close(new_fd);
    close(sock_fd);
    pthread_join(tid, NULL);
    pthread_mutex_destroy(state.lock);
    free(state.lock);
   
    return 0;
}
    
    
