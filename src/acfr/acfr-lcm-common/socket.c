/*
 * Socket routines used by LCM code
 *
 * Christian Lees
 * 23/4/15
 */

#include "socket.h"

// Create a UDP listen socket
int create_udp_listen(char *port)
{
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int ret;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

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

// Create a UDP send socket
int create_udp_send(udp_info_t *info, char *hostname, int port)
{

    struct addrinfo hints;
    int ret;

    memset(&hints, 0, sizeof hints);
    hints.ai_family=AF_UNSPEC;
    hints.ai_socktype=SOCK_DGRAM;
    hints.ai_protocol=0;
    hints.ai_flags=AI_ADDRCONFIG;
    char port_str[16];
    sprintf(port_str, "%d", port);
    if((ret = getaddrinfo(hostname, port_str, &hints, &info->info)) != 0)
    {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(ret));
        return -1;
    }

    info->fd = socket(info->info->ai_family, info->info->ai_socktype, info->info->ai_protocol);
    if(info->fd == -1)
    {
        perror("listener: socket");
        return 0;
    }

    return 1;
}

int send_udp(udp_info_t *info, const char *d, int len)
{
    if(sendto(info->fd,d,len,0,info->info->ai_addr,info->info->ai_addrlen)==-1)
    {
        perror("UDP send error");
        return 0;
    }
    return 1;
}


void get_localhost_sockaddr(struct sockaddr_in *servaddr, int port)
{
    bzero(servaddr,sizeof(struct sockaddr_in));
    servaddr->sin_family = AF_INET;
    servaddr->sin_addr.s_addr=inet_addr("127.0.0.1");
    servaddr->sin_port=htons(port);

}


int create_udp_multicast_listen(char *group, int port)
{
    int sockfd;
    struct sockaddr_in addr;
    struct ip_mreq mreq;
    unsigned int yes;

    // Create a UDP socket
    if((sockfd=socket(AF_INET,SOCK_DGRAM,0)) < 0)
    {
        perror("socket");
        return -1;
    }

    // Set address reuse
    if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
    {
        perror("Reusing ADDR failed");
        return -1;
    }

    // Set recieve errors
    if(setsockopt(sockfd, SOL_SOCKET, IP_RECVERR, &yes, sizeof(yes)) < 0)
    {
        perror("IP_RECVERR failed");
        return -1;
    }

    // Set up the address
    memset(&addr,0,sizeof(addr));
    addr.sin_family=AF_INET;
    addr.sin_addr.s_addr=htonl(INADDR_ANY);
    addr.sin_port=htons(port);

    // Bind
    if(bind(sockfd,(struct sockaddr *) &addr,sizeof(addr)) < 0)
    {
        perror("bind");
        return -1;
    }

    // Set multicast
    mreq.imr_multiaddr.s_addr=inet_addr(group);
    mreq.imr_interface.s_addr=htonl(INADDR_ANY);
    //mreq.imr_interface.s_addr=inet_addr("192.168.2.10");
    if (setsockopt(sockfd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) < 0)
    {
        perror("setsockopt");
        return -1;
    }

    printf("UDP multicast listen socket open on group %s using port %d\n", group, port);

    return sockfd;
}


