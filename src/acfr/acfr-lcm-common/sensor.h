#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>
#include <string.h>
#include <stdlib.h>
#include <bot_param/param_client.h>
#include "error.h"
#include "serial.h"


#ifndef SENSOR_H
#define SENSOR_H

enum {io_tcp, io_udp, io_serial};

typedef struct
{
    int fd;
    int io_type;
    int canonical;
	char t1, t2;
    char *ip;
    char *inet_port;
    char *serial_dev;
    char *parity;
    int baud;
    int port_open;
} acfr_sensor_t;


int acfr_sensor_open(acfr_sensor_t *s);
int acfr_sensor_load_config(lcm_t *lcm, acfr_sensor_t *s, char *root_key);
int acfr_sensor_noncanonical(acfr_sensor_t *s, int min, int time);
int acfr_sensor_canonical(acfr_sensor_t *s, char t1, char t2);
int acfr_sensor_write(acfr_sensor_t *s, char *d, int size);
int acfr_sensor_read(acfr_sensor_t *s, char *d, int len);
int acfr_sensor_destroy(acfr_sensor_t *s);
acfr_sensor_t *acfr_sensor_create(lcm_t *lcm, char *rootkey);



#endif // SENSOR_H
