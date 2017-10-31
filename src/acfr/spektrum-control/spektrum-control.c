#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <math.h>

#include "acfr-common/timestamp.h"
#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_spektrum_control_command_t.h"

//#define DEBUG

#define LOOPTIMEOUT 20 // No of times to try read UDP message bits before continuing without read

int
create_udp_listen(char *port)
{
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int ret;
    char host[64];

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    if((ret = getaddrinfo(NULL, port, &hints, &servinfo)) != 0)
    {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(ret));
        return -1;
    }

    for(p = servinfo; p != NULL; p = p->ai_next)
    {
	getnameinfo(p->ai_addr, sizeof(struct sockaddr_in), host, sizeof(host), NULL, 0,  NI_NUMERICHOST);
        printf("Host name: %s\n", host);
    }
	// loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next)
    {
	getnameinfo(p->ai_addr, sizeof(struct sockaddr_in), host, sizeof(host), NULL, 0,  NI_NUMERICHOST);
        printf("Host name: %s\n", host);

        if((sockfd = socket(p->ai_family, p->ai_socktype,
                            p->ai_protocol)) == -1)
        {
            perror("listener: socket");
            continue;
        }
        if(bind(sockfd, p->ai_addr, p->ai_addrlen) == -1)
        {
            close(sockfd);
            perror("listener: bind");
            continue;
        }
        break;
    }

    if(p == NULL)
    {
        fprintf(stderr, "listener: failed to bind socket\n");
        return -2;
	}
    printf("UDP listen socket open on port %s\n", port);

    return sockfd;
}

// Parse the 16 bytes that come from the RC controller
int
parse_rc(char *buf, lcm_t *lcm, int channels)
{

    unsigned short channel_value;
    char channel_id;
    short channel_values[channels];

    //float rcmult = RC_MAX_PROP_RPM/(RC_HALF_INPUT_RANGE - RC_DEADZONE);
    //if(buf[0] != 0 && buf[1] != 162)
    //    return 0;


    for(int i=1; i<8; i++)
    {
        channel_id = (buf[i*2] & 0xF8) >> 3;
        channel_value = ((buf[i*2] & 0x07) << 8) | (buf[(i*2)+1] & 0xFF);

        channel_values[(int)channel_id] = channel_value;

    }

    acfrlcm_auv_spektrum_control_command_t sc;
    sc.utime = timestamp_now();
    sc.channels = channels;
    for(int i=0; i<6; i++)
	    sc.values[i] = channel_values[i];
    acfrlcm_auv_spektrum_control_command_t_publish(lcm, "SPEKTRUM_CONTROL", &sc);

    return 1;
}

int main_exit;
void signal_handler(int sigNum)
{
    // do a safe exit
    main_exit = 1;
}

int
main(int argc, char **argv)
{
    // install the signal handler
    main_exit = 0;
    signal(SIGINT, signal_handler);

    lcm_t *lcm = lcm_create(NULL);
    BotParam *param = bot_param_new_from_server (lcm, 1);

    if(param == NULL)
        return 0;

    // Read the config params
    char rootkey[64];
    char key[64];
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    sprintf(key, "%s.control_port", rootkey);
    char *control_port = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.channels", rootkey);
    int channels = bot_param_get_int_or_fail(param, key);

    // Create the UDP listener
    int control_sockfd = create_udp_listen(control_port);
    fd_set rfds;
    struct timeval tv;
    int data_len;
    char buf[16];
    int count = 0;

    while(!main_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(control_sockfd, &rfds);
        tv.tv_sec = 1.0;
        tv.tv_usec = 0;

        int loopcount = 0;
        int ret = select (control_sockfd + 1, &rfds, NULL, NULL, &tv);
        if(ret != -1)
        {
            data_len = 0;
            while((data_len < 16)&&(loopcount < LOOPTIMEOUT))
            {
                data_len += recvfrom(control_sockfd, &buf[data_len], 16 - data_len, 0, NULL, NULL);
                loopcount++;
		count++;
            }
//	    printf("Data len: %d, %d\n", data_len, count);
            if(data_len >= 16)
            {
                parse_rc(buf, lcm, channels);
            }
        }
    }
    close(control_sockfd);

    return 1;
}




