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


int main_exit;


typedef struct {
    int sockfd;
    char *portstr;
	int connected;
} socket_info_t;

int accept_timeout(int sockfd, struct sockaddr_storage *their_addr, socklen_t *addrsize, int timeout) {

    fd_set rfds;
    FD_ZERO (&rfds);
    FD_SET (sockFd, &rfds);
    
    struct timeval tv;
    tv.tv_sec = timeout;
    tv.tv_usec = 0;

    int retfd = -1; 
   
    int ret = select (sockFd + 1, &rfds, NULL, NULL, &tv);
    
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

    getaddrinfo(NULL, socket_info->portStr, &hints, &res);

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
	socket_info->sockFd = -1;
	socket_info->connected = 0;
	addrsize = sizeof(their_addr);
    while (1)
    {
        if(!socket_info->connected) {
            // we can listen for a new connection
            if((tmpfd = acceptTimeout(sockfd, (struct sockaddr_storage *)&their_addr, &addrsize, 1)) > 0) {
        		socket_info->sockfd = tmpfd;   
        		socket_info->connected = 1;
        	}
        }
        if(!checkSocket(socket_info->sockFd))
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
    struct sockaddr_storage their_addr;
    char buf[MAXBUFLEN];
    socklen_t addr_len;
    int ret;
    char s[INET6_ADDRSTRLEN];
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
            close(control_sockfd);
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
    
    return sock_fd;
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
    
    if(argc < 1)
        param = bot_param_new_from_server (lcm, 1);
    else
        param = bot_param_new_from_named_server (lcm, argv[1], 1);
        
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
    
    
	
	// Create the UDP listener
	int control_sockfd = create_udp_listen(control_port);
	
	// TCP isn't quite as simple, we will use a thread per port
	socket_info_t motor_sock;
	pthread_t motor_sock_thread;
	motor_sock.port_str = motor_tcp_port;
    pthread_create(&motor_sock_thread, NULL, socket_handler, &motor_sock);	
    pthread_detach(motor_sock_thread);	
	
	socket_info_t fins_tcp_sock;
	pthread_t fins_sock_thread;
	fins_sock.port_str = fins_tcp_port;
    pthread_create(&fins_sock_thread, NULL, socket_handler, &fins_sock);	
	pthread_detach(fins_sock_thread);	
	
	
	while(!main_exit)
	    sleep(1);
	    
	pthread_join(motor_sock_thread);
	pthread_join(fins_sock_thread);
	close(control_sockfd);
	
	return 0;
}
	
	
    
    
    
