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

#include <bot_param/param_client.h>
#include "perls-common/serial.h"
#include "perls-common/timestamp.h"


#define RC_REMOTE 1
#define RC_FINS 2
#define RC_MOTOR 3


int main_exit;

int max(int a, int b) {
  return a > b ? a : b;
}

typedef struct {
    int sockfd;
    char *portstr;
	int connected;
} socket_info_t;

int accept_timeout(int sockfd, struct sockaddr_storage *their_addr, socklen_t *addrsize, int timeout) {

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

int check_socket(int sockfd) {
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

    // now accept connections!
	socket_info->sockfd = -1;
	socket_info->connected = 0;
	addrsize = sizeof(their_addr);
    while (1)
    {
        if(!socket_info->connected) {
            // we can listen for a new connection
            if((tmpfd = accept_timeout(sockfd, (struct sockaddr_storage *)&their_addr, &addrsize, 1)) > 0) {
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
    
    if((ret = getaddrinfo(NULL, port, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(ret));
        return -1;
    }
    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("listener: socket");
            continue;
        }
        if(bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("listener: bind");
            continue;
        }
        break;
    }
    
    if(p == NULL) {
        fprintf(stderr, "listener: failed to bind socket\n");
        return -2;
    }
    freeaddrinfo(servinfo);
    
    return sockfd;
}

int
parse_rc(char *buf, int *contol, int *value)
{
    return 1;
}

void signal_handler(int sigNum) {
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
	pthread_t motor_sock_thread;
	motor_sock.portstr = motor_tcp_port;
    pthread_create(&motor_sock_thread, NULL, socket_handler, &motor_sock);	
    pthread_detach(motor_sock_thread);	
	
	socket_info_t fins_sock;
	pthread_t fins_sock_thread;
	fins_sock.portstr = fins_tcp_port;
    pthread_create(&fins_sock_thread, NULL, socket_handler, &fins_sock);	
	pthread_detach(fins_sock_thread);	

    fd_set rfds;		
    struct timeval tv;
    int maxfd = max(motor_sock.sockfd, fins_sock.sockfd);
    maxfd = max(maxfd, control_sockfd);
    int data_len;
    char buf[256];
    int remote = 0;
    uint64_t remote_time;
    
	while(!main_exit)
	{
	    // this is where the magic happens
	    
        FD_ZERO(&rfds);
        FD_SET(motor_sock.sockfd, &rfds);
        FD_SET(fins_sock.sockfd, &rfds);
        FD_SET(control_sockfd, &rfds);
        int control, value;
        
        tv.tv_sec = 1.0;
        tv.tv_usec = 0;

        int ret = select (maxfd + 1, &rfds, NULL, NULL, &tv);
        if(ret != -1)
        {
            if(FD_ISSET(motor_sock.sockfd, &rfds))
            {
                if(!remote)
                {
                    data_len = recv(motor_sock.sockfd, buf, sizeof(buf), 0);
                    write(motorfd, buf, data_len);
                }
            }
            else if(FD_ISSET(fins_sock.sockfd, &rfds))
            {
                if(!remote)
                {
                    data_len = recv(fins_sock.sockfd, buf, sizeof(buf), 0);
                    write(finsfd, buf, data_len);
                }
            }
            else if(FD_ISSET(control_sockfd, &rfds))
            {
                data_len = recvfrom(control_sockfd, buf, sizeof(buf), 0, NULL, NULL);
                if(parse_rc(buf, &control, &value))
                    switch(control)
                    {
                        case RC_REMOTE:
                            remote_time = timestamp_now();
                            remote = 1;
                            break;
                        case RC_FINS:
                            break;
                        case RC_MOTOR:
                            break;
                    }
            }         
                
        }
        
        // remote timeout
        if((timestamp_now() - remote_time) > 1e6)
            remote = 0;
	}
	
	pthread_join(motor_sock_thread, NULL);
	pthread_join(fins_sock_thread, NULL);
	printf("threads joined\n");
	close(control_sockfd);
	
	return 0;
}
	
	
    
    
    
