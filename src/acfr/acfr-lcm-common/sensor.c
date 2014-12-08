#include "sensor.h"


// load the config from the param server
int acfr_sensor_load_config(lcm_t *lcm, acfr_sensor_t *s, char *rootkey)
{
    // Read the LCM config file
    BotParam *param;
	char key[64];
	
    param = bot_param_new_from_server (lcm, 1);
    
    // read the config file

	sprintf(key, "%s.io", rootkey);
	char *io_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(io_str, "serial"))
        s->io_type = io_serial;
    else if(!strcmp(io_str, "tcp"))
        s->io_type = io_tcp;
    else if(!strcmp(io_str, "udp"))
        s->io_type = io_udp;
    else
    {
        fprintf(stderr, "Unknown io type %s\n", io_str);
        return 0;
    }
    
    if(s->io_type == io_serial)
    {
        sprintf(key, "%s.serial_dev", rootkey);
        s->serial_dev = bot_param_get_str_or_fail(param, key);

    	sprintf(key, "%s.baud", rootkey);
	    s->baud = bot_param_get_int_or_fail(param, key);

	    sprintf(key, "%s.parity", rootkey);
	    s->parity = bot_param_get_str_or_fail(param, key);
    }
    
    if(s->io_type == io_tcp || s->io_type == io_udp)
    {
        sprintf(key, "%s.ip", rootkey);
        s->ip = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.port", rootkey);
        s->inet_port = bot_param_get_str_or_fail(param, key);
    }
    
    s->port_open = 0;
    
    return 1;
}

// open the device
int acfr_sensor_open(acfr_sensor_t *s)
{
    // Open either the serial port or the socket
    struct addrinfo hints, *spec_addr;
    
    if(s->port_open)
    {
        fprintf(stderr, "Port already open\n");
        return 0;
    }
    
    if(s->io_type == io_serial)
    {
        s->fd = serial_open(s->serial_dev, serial_translate_speed(s->baud), serial_translate_parity(s->parity), 1);
        if(s->fd < 0)
        {
            printf("Error opening port %s\n", s->serial_dev);
            return 0;
        }
        s->port_open = 1;
    }        
    else if(s->io_type == io_tcp)
    {
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo(s->ip, s->inet_port, &hints, &spec_addr);
    	s->fd = socket(spec_addr->ai_family, spec_addr->ai_socktype, spec_addr->ai_protocol);
        if(connect(s->fd, spec_addr->ai_addr, spec_addr->ai_addrlen) < 0) 
        {
	        printf("Could not connect to %s on port %s\n", s->ip, s->inet_port);
    		return 0;
        }
        s->port_open = 1;
    
    }
    //TODO: UDP   
    return 1;
}

// read from the port, the behavior is different depending on if the connection is canonical or not
int acfr_sensor_read(acfr_sensor_t *s, char *d, int len)
{
	if(!s->port_open)
		acfr_sensor_open(s);
	
	if(s->io_type == io_serial && s->canonical)
		return read(s->fd, d, len);
	else if((s->io_type == io_tcp || s->io_type == io_udp) && s->canonical)
	{
		int bytes = 0;
		int term = 0;
		while(bytes < len)
		{
			bytes += read(s->fd, &d[bytes], 1);
			if(d[bytes-1] == s->t1)
			{
				term = 1;
				break;
			}
		}
		if(term)
			return bytes;
		else
			return -1;
	}
	else
	{
		int bytes = 0;
		while(bytes < len)
			bytes += read(s->fd, &d[bytes], len - bytes);
		return bytes;
	}
		
		
}

// write to the port, check from broken pipes, if it is broken then we need to reopen it
int acfr_sensor_write(acfr_sensor_t *s, char *d, int size)
{
	if(!s->port_open)
		acfr_sensor_open(s);
	
	return write(s->fd, d, size);
}

// Set the canonical processing termination characters, t2 is only used if it is a serial port
int acfr_sensor_canonical(acfr_sensor_t *s, char t1, char t2)
{
	if(s->io_type == io_serial)
		serial_set_canonical (s->fd, t1, t2);
	s->canonical = 1;
	s->t1 = t1;
	s->t2 = t2;
	
	return 1;
}

// Set noncanonical processing
int acfr_sensor_noncanonical(acfr_sensor_t *s, int min, int time)
{
	if(s->io_type == io_serial)
		serial_set_noncanonical (s->fd, min, time) ;
	s->canonical = 0;
	return 1;
}