#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#ifndef _ACFR_SOCKET_H
#define _ACFR_SOCKET_H

typedef struct 
{
	int fd;
	struct addrinfo	*info;
} udp_info_t;

int create_udp_listen(char *port);
int create_udp_send(udp_info_t *info, char *hostname, int port);
int send_udp(udp_info_t *info, const char *d, int len);
void get_localhost_sockaddr(struct sockaddr_in *servaddr, int port);


#endif // _ACFR_SOCKET_H
