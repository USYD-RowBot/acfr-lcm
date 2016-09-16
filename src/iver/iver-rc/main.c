#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <math.h>

#include "perls-common/timestamp.h"
#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_iver_motor_command_t.h"

//#define DEBUG

// Original Iver config

enum
{
    RC_THROTTLE = 0,
    RC_AILERON,
    RC_ELEVATOR,
    RC_RUDDER,
    RC_GEAR,
    RC_AUX1,
    RC_AUX2
};

/*
// Config using the borrowed controller from Mitch
enum
{
    RC_THROTTLE = 0,
    RC_RUDDER,
    RC_ELEVATOR,
    RC_AILERON,
    RC_GEAR,
    RC_AUX1,
    RC_AUX2
};
*/
#define RC_OFFSET 505
//#define RC_THROTTLE_OFFSET 415    // Iver
#define RC_THROTTLE_OFFSET 493      // Testing 16092016 JJM
#define RC_TO_RAD (0.071875*M_PI/180)*2
//#define RC_TO_RPM 3.8             // Iver
#define RC_TO_RPM 8              // Mitch
#define RC_MAX_PROP_RPM 1500
//#define RC_HALF_INPUT_RANGE 350 // Testing 160902016 JJM
#define RC_DEADZONE 40 // Testing 160902016 JJM
#define RCMULT 4.8 //RC_MAX_PROP_RPM/(RC_HALF_INPUT_RANGE-RC_DEADZONE)

int
create_udp_listen(char *port)
{
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int ret;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    if((ret = getaddrinfo(NULL, port, &hints, &servinfo)) != 0)
    {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(ret));
        return -1;
    }
    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next)
    {
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
parse_rc(char *buf, lcm_t *lcm)
{

    unsigned short channel_value;
    char channel_id;
    int channel_values[7];
    int rcval = 0;

    //float rcmult = RC_MAX_PROP_RPM/(RC_HALF_INPUT_RANGE - RC_DEADZONE);

    if(buf[0] != 3 && buf[1] != 1)
        return 0;


    for(int i=1; i<8; i++)
    {
        channel_id = (buf[i*2] & 0xFC) >> 2;
        channel_value = ((buf[i*2] & 0x03) << 8) | (buf[(i*2)+1] & 0xFF);

        channel_values[(int)channel_id] = channel_value;

    }

#ifdef DEBUG
    for(int i=1; i<8; i++)
        printf("Channel %d: value %d\n", i, channel_values[i] & 0xFFFF);
#endif


    // Controller Spektrum DX5e 3 postion switch used for mode
    // marked: 0/1/2 output: 852/596/170 (approx) tested 16092016
    // postion 2: automated (path planner) control, so do nothing
    // postion 1: send zeroed lcm control messages stop prop, centre fins
    // postion 0: RC control, send control messages based on RC input
    // create the LCM message to send
    if(channel_values[RC_GEAR] > 724) // POS 0: RC 724 = cutoff between pos 0, pos1
    {
#ifdef DEBUG       
        printf("POS 0: %d\n", channel_values[RC_GEAR]);
#endif
        acfrlcm_auv_iver_motor_command_t rc;
        rc.utime = timestamp_now();
        rc.top = -(channel_values[RC_AILERON] - RC_OFFSET) * RC_TO_RAD;
        rc.bottom = rc.top;
        rc.port = -(channel_values[RC_ELEVATOR] - RC_OFFSET) * RC_TO_RAD;
        rc.starboard = rc.port;
        
        rcval = channel_values[RC_THROTTLE] - RC_THROTTLE_OFFSET;

        // if in deadzone at centre
        if (abs(rcval) < RC_DEADZONE)
        {
            rc.main = 0;
        }
        else
        {
            rc.main = (rcval - RC_DEADZONE) * RCMULT;
            if (rc.main > RC_MAX_PROP_RPM)
                rc.main = RC_MAX_PROP_RPM;
            else if (rc.main < -RC_MAX_PROP_RPM)
                rc.main = -RC_MAX_PROP_RPM;
        }        

//       rc.main = (channel_values[RC_THROTTLE] - RC_THROTTLE_OFFSET) * RC_TO_RPM;
//       if(abs(rc.main) < 40) #DEADZONE 
//            rc.main = 0;

        rc.source = ACFRLCM_AUV_IVER_MOTOR_COMMAND_T_REMOTE;
        acfrlcm_auv_iver_motor_command_t_publish(lcm, "IVER_MOTOR", &rc);
    }
    else
    {
       // Use position 1 on rc gear switch to send zero movement command
       if(channel_values[RC_GEAR] > 383) // POS 1: ZEROED 383 = cutoff between pos 1, pos 2
        {
#ifdef DEBUG       
            printf("POS 1: %d\n", channel_values[RC_GEAR]);
#endif
            acfrlcm_auv_iver_motor_command_t rc;
            rc.utime = timestamp_now();
            rc.top = 0;
            rc.bottom = 0;
            rc.port = 0;
            rc.starboard = 0;
            rc.main = 0;
            rc.source = ACFRLCM_AUV_IVER_MOTOR_COMMAND_T_REMOTE;
            acfrlcm_auv_iver_motor_command_t_publish(lcm, "IVER_MOTOR", &rc);
        }
    }    
    
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

    // Create the UDP listener
    int control_sockfd = create_udp_listen(control_port);
    fd_set rfds;
    struct timeval tv;
    int data_len;
    char buf[16];

    while(!main_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(control_sockfd, &rfds);
        tv.tv_sec = 1.0;
        tv.tv_usec = 0;

        int ret = select (control_sockfd + 1, &rfds, NULL, NULL, &tv);
        if(ret != -1)
        {
            data_len = 0;
            while(data_len < 16)
                data_len += recvfrom(control_sockfd, &buf[data_len], 16 - data_len, 0, NULL, NULL);
            parse_rc(buf, lcm);
        }
    }
    close(control_sockfd);

    return 1;
}




