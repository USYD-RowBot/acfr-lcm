#include "sensor.h"


acfr_sensor_t *acfr_sensor_create(lcm_t *lcm, char *rootkey)
{
    acfr_sensor_t *s = (acfr_sensor_t *)malloc(sizeof(acfr_sensor_t));
    if(acfr_sensor_load_config(lcm, s, rootkey))
    {
        if(acfr_sensor_open(s))
            return s;
        else
            return NULL;
    }
    else
        return NULL;
}

int acfr_sensor_destroy(acfr_sensor_t *s)
{
    if(s->port_open)
        close(s->fd);
    free(s);
    
    return 1;
}


// load the config from the param server
int acfr_sensor_load_config(lcm_t *lcm, acfr_sensor_t *s, char *rootkey)
{
    // Read the LCM config file
	char key[64];
	
    s->param = bot_param_new_from_server (lcm, 1);
    
    // read the config file

	sprintf(key, "%s.io", rootkey);
	char *io_str = bot_param_get_str_or_fail(s->param, key);
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
        s->serial_dev = bot_param_get_str_or_fail(s->param, key);

    	sprintf(key, "%s.baud", rootkey);
	    s->baud = bot_param_get_int_or_fail(s->param, key);

	    sprintf(key, "%s.parity", rootkey);
	    s->parity = bot_param_get_str_or_fail(s->param, key);
    }
    
    if(s->io_type == io_tcp || s->io_type == io_udp)
    {
        sprintf(key, "%s.ip", rootkey);
        s->ip = bot_param_get_str_or_fail(s->param, key);

        sprintf(key, "%s.port", rootkey);
        s->inet_port = bot_param_get_str_or_fail(s->param, key);
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
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo(s->ip, s->inet_port, &hints, &spec_addr);
    	s->fd = socket(spec_addr->ai_family, spec_addr->ai_socktype, spec_addr->ai_protocol);
        if(connect(s->fd, spec_addr->ai_addr, spec_addr->ai_addrlen) < 0) 
        {
	        printf("Could not connect to %s on port %s\n", s->ip, s->inet_port);
    		return 0;
        }

		// set the socket keepalive flag
		int optval = 1;
		setsockopt(s->fd, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval));

		// set a 1 second timeout on socket reads		
		struct timeval tv;
		tv.tv_sec = 1;  // 1 Secs Timeout 
		tv.tv_usec = 0;  // Not init'ing this can cause strange errors
		setsockopt(s->fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

		// set a 1 second timeout on socket writes		
		tv.tv_sec = 1;  // 1 Secs Timeout 
		tv.tv_usec = 0;  // Not init'ing this can cause strange errors
		setsockopt(s->fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(struct timeval));
		
		//set output buffer size so that send blocks
		int buffersize = 2;
		setsockopt(s->fd, SOL_SOCKET, SO_SNDBUF, &buffersize, 4);

        s->port_open = 1;
    
    }
    else if(s->io_type == io_udp)
    {
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_flags = AI_PASSIVE;
        getaddrinfo(0, s->inet_port, &hints, &spec_addr);
    	s->fd = socket(spec_addr->ai_family, spec_addr->ai_socktype, spec_addr->ai_protocol);
        if(bind(s->fd, spec_addr->ai_addr, spec_addr->ai_addrlen) < 0) 
        {
	        printf("Could not bind to udp on port %s\n", s->inet_port);
    		return 0;
        }

		// set a 1 second timeout on socket reads		
		struct timeval tv;
		tv.tv_sec = 1;  // 1 Secs Timeout 
		tv.tv_usec = 0;  // Not init'ing this can cause strange errors
		setsockopt(s->fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

		// set a 1 second timeout on socket writes		
		tv.tv_sec = 1;  // 1 Secs Timeout 
		tv.tv_usec = 0;  // Not init'ing this can cause strange errors
		setsockopt(s->fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(struct timeval));
		
		//set output buffer size so that send blocks
		int buffersize = 2;
		setsockopt(s->fd, SOL_SOCKET, SO_SNDBUF, &buffersize, 4);

        s->port_open = 1;
    
    }
    return 1;
}

int acfr_sensor_read(acfr_sensor_t *s, char *d, int len)
{
	return acfr_sensor_read_timeout(s, d, len, -1);
}

// read from the port, the behavior is different depending on if the connection is canonical or not
int acfr_sensor_read_timeout(acfr_sensor_t *s, char *d, int len, int timeout)
{
	if(!s->port_open)
		acfr_sensor_open(s);
	
	if(s->io_type == io_serial && s->canonical)
		return read(s->fd, d, len);
	else if((s->io_type == io_tcp || s->io_type == io_udp) && s->canonical)
	{
		// We have a global read timeout set on the socket of 1 second
		// as we loop until we have all the bytes the timeout is easy to
		// implement
		int bytes = 0;
		int term = 0;
		int64_t start_time = timestamp_now();
		while(bytes < len)
		{
			bytes += read(s->fd, &d[bytes], 1);
			if(d[bytes-1] == s->t1)
			{
				term = 1;
				break;
			}
			if(timeout > 0)
				if(((timestamp_now() - start_time) / 1e6) > timeout)
					break;
		}
		if(term)
			return bytes;
		else
			return -1;
	}
	else if((s->io_type == io_tcp || s->io_type == io_udp) && !s->canonical && timeout > 0)
	{
		
		// get the current timeout value
		struct timeval old_tv, new_tv;
		socklen_t s_size = sizeof(struct timeval);
		getsockopt(s->fd, SOL_SOCKET, SO_RCVTIMEO, &old_tv, &s_size);

		new_tv.tv_sec = timeout;
		new_tv.tv_usec = 0;
		setsockopt(s->fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&new_tv, sizeof(struct timeval));

		int bytes = read(s->fd, d, len);
		
		// put the timeout back how we found it
		setsockopt(s->fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&old_tv, sizeof(struct timeval));

		return bytes;
	}
	else
	{
		// Read with the standard timeout
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

	if(s->io_type == io_tcp)
        return send(s->fd, d, size, 0);
    else
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
		serial_set_noncanonical (s->fd, min, time);
	s->canonical = 0;
	return 1;
}
