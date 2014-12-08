#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>
#include <string.h>
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
    char *ip;
    char *inet_port;
    char *serial_dev;
    char *parity;
    int baud;
    int port_open;
} acfr_sensor_t;


int acfr_sensor_open(acfr_sensor_t *s);
int acfr_sensor_load_config(lcm_t *lcm, acfr_sensor_t *s, char *root_key);



#endif // SENSOR_H
