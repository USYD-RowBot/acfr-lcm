/*
    Remote control interface

    This module will listen for UDP messages from the RC system and
    override the control system.

    Both the thruster and the fin controls now come from this module.
    Two TCP server ports will act as the interface to UVC, in the future
    thet will be LCM interfaces.  A UDP rx interface will receive messages
    from the deck box.

    Christian Lees
    ACFR
    15/10/12
*/

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
#include <time.h>

#include <bot_param/param_client.h>
#include "perls-common/lcm_util.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-common/serial.h"
#include "perls-common/timestamp.h"

#define ANIMATICS_PRINT_STRING "PRINT(V,\" \",UIA,\" \",UJA,\" \",TEMP,#13,#10)\n"


#define RC_REMOTE 1
#define RC_FINS 2
#define RC_MOTOR 3

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


int main_exit;

int max(int a, int b)
{
    return a > b ? a : b;
}

typedef struct
{
    int sockfd;
    char *portstr;
    int connected;
    pthread_mutex_t ready;
} socket_info_t;

typedef struct
{
    char motor_string[256];
    char fins_data[12];
    pthread_mutex_t motor_d_lock;
    pthread_mutex_t motor_port_lock;
    pthread_mutex_t fins_d_lock;
    pthread_mutex_t fins_port_lock;
    int motor_fd;
    int fins_fd;
    int *remote;
} motor_state_t;

int accept_timeout(int sockfd, struct sockaddr_storage *their_addr, socklen_t *addrsize, int timeout)
{

    fd_set rfds;
    FD_ZERO (&rfds);
    FD_SET (sockfd, &rfds);

    struct timeval tv;
    tv.tv_sec = timeout;
    tv.tv_usec = 0;

    int retfd = -1;

    int ret = select (sockfd + 1, &rfds, NULL, NULL, &tv);

    if(ret == -1)
        perror("acceptTimeout select()");
    else if(ret != 0)
        retfd = accept(sockfd, (struct sockaddr *)their_addr, addrsize);

    return retfd;
}

int check_socket(int sockfd)
{
    fd_set rfds;
    FD_ZERO (&rfds);
    FD_SET (sockfd, &rfds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    select (sockfd + 1, &rfds, NULL, NULL, &tv);
    if(FD_ISSET(sockfd, &rfds))
        return 0;
    else
        return 1;
}

static void *
socket_handler (void *u)
{
    int sockfd, tmpfd;
    struct addrinfo hints, *res;
    socklen_t addrsize;
    struct sockaddr_storage their_addr;

    socket_info_t *socket_info = (socket_info_t *)u;

    printf("Opening Socket!\n");

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;  // use IPv4 or IPv6, whichever
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;     // fill in my IP for me

    getaddrinfo(NULL, socket_info->portstr, &hints, &res);

    // make a socket:
    sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    //setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, NULL, NULL);
    if (sockfd == -1)
    {
        printf("Error opening socket(%i): %s\n", errno, strerror(errno));
        exit(1);
    }

    // bind it to the port we passed in to getaddrinfo():
    if (-1 == bind(sockfd, res->ai_addr, res->ai_addrlen))
    {
        printf("Error binding socket(%i): %s\n", errno, strerror(errno));
        exit(1);
    }

    // now listen.. accepting one connection at a time
    if (-1 == listen(sockfd, 1))
    {
        printf("Error listening on socket(%i): %s\n", errno, strerror(errno));
        exit(1);
    }

    printf("Socket opened, listening on port %s\n", socket_info->portstr);

    // now accept connections!
    socket_info->sockfd = -1;
    socket_info->connected = 0;
    addrsize = sizeof(their_addr);

    pthread_mutex_unlock(&socket_info->ready);

    while (1)
    {
        if(!socket_info->connected)
        {
            // we can listen for a new connection
            if((tmpfd = accept_timeout(sockfd, (struct sockaddr_storage *)&their_addr, &addrsize, 1)) > 0)
            {
                socket_info->sockfd = tmpfd;
                socket_info->connected = 1;
            }
        }
        if(!check_socket(socket_info->sockfd))
            socket_info->connected = 0;

        usleep(500000);
        sched_yield();

        if(main_exit)
            break;
    }
    printf("Exiting socket thread\n");
    close(socket_info->sockfd);
    close(sockfd);
    return NULL;
}


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
    freeaddrinfo(servinfo);

    printf("UDP listen socket open on port %s\n", port);

    return sockfd;
}

// Parse the 16 bytes that come from the RC controller
int
parse_rc(char *buf, int *channel_values)
{

    unsigned short channel_value;
    char channel_id;

    if(buf[0] != 3 && buf[1] != 1)
        return 0;


    for(int i=1; i<8; i++)
    {
        channel_id = (buf[i*2] & 0xFC) >> 2;
        channel_value = ((buf[i*2] & 0x03) << 8) | (buf[(i*2)+1] & 0xFF);

        channel_values[channel_id] = channel_value;
    }
    printf("*");
    for(int i=0; i<7; i++)
        fprintf(stderr, "%u ", channel_values[i]);
    printf("\n\n");


    return 1;
}

void
motor_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    motor_state_t *state = (motor_state_t *)u;

    if(*state->remote)
    {
        pthread_mutex_lock(&state->motor_d_lock);
        pthread_mutex_lock(&state->motor_port_lock);
        write(state->motor_fd, state->motor_string, strlen(state->motor_string));
        printf("RC: %s\n", state->motor_string);
        pthread_mutex_unlock(&state->motor_d_lock);
        pthread_mutex_unlock(&state->motor_port_lock);

        pthread_mutex_lock(&state->fins_d_lock);
        pthread_mutex_lock(&state->fins_port_lock);
        for(int i=0; i<12; i++)
            printf("%02X ", state->fins_data[i] & 0xFF);
        printf("\n");
        write(state->fins_fd, state->fins_data, 12);
        pthread_mutex_unlock(&state->fins_d_lock);
        pthread_mutex_unlock(&state->fins_port_lock);

    }
    //write(state->sockfd, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));

    return NULL;
}





// Process LCM messages with callbacks
void *lcm_handler(void *u)
{
    lcm_t *lcm = (lcm_t *)u;

    while (!main_exit)
    {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        lcmu_handle_timeout(lcm, &tv);
    }
    printf("LCM thread exiting\n");
    return 0;
}

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
    BotParam *param = NULL;

//    if(argc < 1)
    param = bot_param_new_from_server (lcm, 1);
//    else
//        param = bot_param_new_from_named_server (lcm, argv[1], 1);


    if(param == NULL)
        return 0;

    // Read the config params
    char rootkey[64];
    char key[64];
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    sprintf(key, "%s.control_port", rootkey);
    char *control_port = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.motor_tcp_port", rootkey);
    char *motor_tcp_port = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.fins_tcp_port", rootkey);
    char *fins_tcp_port = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.motor_device", rootkey);
    char *motor_device = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.motor_baud", rootkey);
    int motor_baud = bot_param_get_int_or_fail(param, key);

    sprintf(key, "%s.motor_parity", rootkey);
    char *motor_parity = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.fins_device", rootkey);
    char *fins_device = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.fins_baud", rootkey);
    int fins_baud = bot_param_get_int_or_fail(param, key);

    sprintf(key, "%s.fins_parity", rootkey);
    char *fins_parity = bot_param_get_str_or_fail(param, key);


    // open the serial ports
    int motorfd = serial_open(motor_device, serial_translate_speed(motor_baud), serial_translate_parity(motor_parity), 1);
    if(motorfd < 0)
    {
        printf("Error opening motor port %s\n", motor_device);
        return 0;
    }

    int finsfd = serial_open(fins_device, serial_translate_speed(fins_baud), serial_translate_parity(fins_parity), 1);
    if(finsfd < 0)
    {
        printf("Error opening fins port %s\n", fins_device);
        return 0;
    }

    // Create the UDP listener
    int control_sockfd = create_udp_listen(control_port);

    // TCP isn't quite as simple, we will use a thread per port
    socket_info_t motor_sock;
    pthread_mutex_init(&motor_sock.ready, NULL);
    pthread_mutex_lock(&motor_sock.ready);
    pthread_t motor_sock_thread;
    motor_sock.portstr = motor_tcp_port;
    pthread_create(&motor_sock_thread, NULL, socket_handler, &motor_sock);
    pthread_detach(motor_sock_thread);

    socket_info_t fins_sock;
    pthread_mutex_init(&fins_sock.ready, NULL);
    pthread_mutex_lock(&fins_sock.ready);
    pthread_t fins_sock_thread;
    fins_sock.portstr = fins_tcp_port;
    pthread_create(&fins_sock_thread, NULL, socket_handler, &fins_sock);
    pthread_detach(fins_sock_thread);

    int remote = 0;
    motor_state_t motor_send_state;
    motor_send_state.remote = &remote;
    motor_send_state.motor_fd = motorfd;
    motor_send_state.fins_fd = finsfd;
    pthread_mutex_init(&motor_send_state.motor_d_lock, NULL);
    pthread_mutex_init(&motor_send_state.motor_port_lock, NULL);
    pthread_mutex_init(&motor_send_state.fins_d_lock, NULL);
    pthread_mutex_init(&motor_send_state.fins_port_lock, NULL);

    pthread_t lcm_thread;
    pthread_create(&lcm_thread, NULL, lcm_handler, lcm);
    pthread_detach(lcm_thread);

    perllcm_heartbeat_t_subscribe(lcm, "HEARTBEAT_10HZ", &motor_handler, &motor_send_state);

    // we need to wait for the threads to wake up
    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += 2;
    if(pthread_mutex_timedlock(&motor_sock.ready, &timeout) != 0)
    {
        printf("Failed to wake up motor sock thread in time\n");
        main_exit = 1;
    }

    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += 2;
    int ret;
    if((ret = pthread_mutex_timedlock(&fins_sock.ready, &timeout)) != 0)
    {
        perror("Fins");
        printf("Failed to wake up fins sock thread in time, %d\n", ret);
        main_exit = 1;
    }



    fd_set rfds;
    struct timeval tv;

    int data_len;
    char buf[256];
    uint64_t remote_time;


    while(!main_exit)
    {
        // this is where the magic happens
        int maxfd = max(motor_sock.sockfd, fins_sock.sockfd);
        maxfd = max(maxfd, control_sockfd);
        FD_ZERO(&rfds);
        FD_SET(motor_sock.sockfd, &rfds);
        FD_SET(fins_sock.sockfd, &rfds);
        FD_SET(control_sockfd, &rfds);
        int control_values[7];

        tv.tv_sec = 1.0;
        tv.tv_usec = 0;

        int ret = select (maxfd + 1, &rfds, NULL, NULL, &tv);
        if(ret != -1)
        {
            if(FD_ISSET(motor_sock.sockfd, &rfds))
            {
                if(!remote)
                {
                    memset(buf, 0, sizeof(buf));
                    data_len = recv(motor_sock.sockfd, buf, sizeof(buf), 0);
                    pthread_mutex_lock(&motor_send_state.motor_port_lock);
                    write(motorfd, buf, data_len);
                    pthread_mutex_unlock(&motor_send_state.motor_port_lock);
                    printf("Iver send: %s\n", buf);
                }
            }
            if(FD_ISSET(fins_sock.sockfd, &rfds))
            {
                if(!remote)
                {
                    memset(buf, 0, sizeof(buf));
                    data_len = recv(fins_sock.sockfd, buf, sizeof(buf), 0);
                    printf("Iver: ");
                    for(int i=0; i<data_len; i++)
                        printf("%02X ", buf[i]&0xFF);
                    printf("\n");
                    pthread_mutex_lock(&motor_send_state.fins_port_lock);
                    write(finsfd, buf, data_len);
                    pthread_mutex_unlock(&motor_send_state.fins_port_lock);
                }
            }

            if(FD_ISSET(control_sockfd, &rfds))
            {
                data_len = 0;
                while(data_len < 16)
                    data_len += recvfrom(control_sockfd, &buf[data_len], 16 - data_len, 0, NULL, NULL);
                if(parse_rc(buf, control_values))
                {
                    if(control_values[RC_GEAR] > 500)
                    {
                        remote = 1;
                        remote_time = timestamp_now();
                    }
                    else
                        remote = 0;

                    if(remote == 1)
                    {
                        // main thruster
                        pthread_mutex_lock(&motor_send_state.motor_d_lock);
                        memset(motor_send_state.motor_string, 0, sizeof(motor_send_state.motor_string));
                        int throttle = control_values[RC_THROTTLE] - 415;
                        if(abs(throttle) < 20)
                            throttle = 0;
                        sprintf(motor_send_state.motor_string, "MV a=0 A=1000 V=%d G\n", (throttle) * 32212 / 30);
                        pthread_mutex_unlock(&motor_send_state.motor_d_lock);

                        // fins
                        pthread_mutex_lock(&motor_send_state.fins_d_lock);
                        int rudder_c = control_values[RC_AILERON];
                        int plane_c = control_values[RC_ELEVATOR];
                        if(rudder_c > 820)
                            rudder_c = 820;
                        if(rudder_c < 180)
                            rudder_c = 180;
                        if(plane_c > 820)
                            plane_c = 820;
                        if(plane_c < 180)
                            plane_c = 180;


                        int rudder = (int)(((float)rudder_c - 180) / 2.8);
                        int plane = (int)(((float)plane_c - 180) / 2.8);
                        memset(motor_send_state.fins_data, 0xFF, 12);
                        motor_send_state.fins_data[1] = 0x01;
                        motor_send_state.fins_data[2] = plane;
                        motor_send_state.fins_data[4] = 0x02;
                        motor_send_state.fins_data[5] = plane;
                        motor_send_state.fins_data[7] = 0x03;
                        motor_send_state.fins_data[8] = rudder;
                        motor_send_state.fins_data[10] = 0x04;
                        motor_send_state.fins_data[11] = rudder;


                        pthread_mutex_unlock(&motor_send_state.fins_d_lock);


                    }
                }
            }
        }

        // remote timeout
        if((timestamp_now() - remote_time) > 2e6)
            remote = 0;
    }
    printf("Main exit\n");
    pthread_join(motor_sock_thread, NULL);
    printf("Motor thread joined\n");
    pthread_join(fins_sock_thread, NULL);
    printf("Fins thread joined\n");
    pthread_join(lcm_thread, NULL);
    printf("LCM thread joined\n");
    close(control_sockfd);

    return 0;
}





