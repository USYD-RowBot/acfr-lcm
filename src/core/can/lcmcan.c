/*
 *      CAN interface for LCM
 *
 *      Christian Lees
 *      ACFR
 *      12/4/13
 */


#include "lcmcan.h"

// Create a CAN device, we assume you have already started LCM
// Needs to be passed the root key so it can read the config file
// Returns a pointer to the CAN device structure
canlcm_driver *canlcm_create(lcm_t *lcm, char *rootkey)
{
    // Allocate the structure we are going to send back
    canlcm_driver *cld = (canlcm_driver *)malloc(sizeof(canlcm_driver);

                         // Read the config file to get the device name and the target id
                         BotParam *params;
                         params = bot_param_new_from_server (lcm, 1);

                         sprintf(key, "%s.can.device", rootkey);
                         cld->device = bot_param_get_str_or_fail(params, key);

                         sprintf(key, "%s.can.id", rootkey);
                         cld->id = bot_param_get_int_or_fail(params, key);


                         // create a can socket
                         if((cld->socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
{
    perror("Error while opening socket");
        return NULL;
    }

    // bind it to the interface
    struct ifreq ifr;
    struct sockaddr_can addr;
    strcpy(ifr.ifr_name, cld->device);
    ioctl(cld->socket, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
                       addr.can_ifindex = ifr.ifr_ifindex;

                       if(bind(cld->socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
{
    perror("Error in socket bind");
        return NULL;
    }

    // Set the CAN filter based on the target id
    struct can_filter filter;
    filter.can_id = cld->id;
                    filter.can_mask = CAN_SFF_MASK;
                    setsockopt(cld->socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

                    return cld;
}

// Close the socket and free the memory
int canlcm_destroy(canlcm_driver *cld
{
    close(cld->socket);
    free(cld);

    return 1;
}

// Send some data to the can device
// Returns the number of bytes sent
int canlcm_write(canlcm_driver *cld, unsigned char *d, int len)
{
    struct can_frame frame;
    frame.can_id = cld->id;
    int bytes;

    if(len <= 8)
    {
        frame.can_dlc = len;
        memset(frame.data, 0, 8);
        memcpy(frame.data, d, len);
        return write(cld->socket, frame, sizeof(frame));
    }
    return -1;
}

// Get some data, blocking
// Returns the number of bytes read
int canlcm_read(canlcm_driver *cld, unsigned char *d, int len, int64_t *timestamp)
{
    struct can_frame frame;
    int bytes;

    bytes = read(cld->socket, frame, len);
    *timestamp = timestamp_now();
    if(bytes == 13)
    {
        memcpy(d, frame.data, frame.can_dlc);
        return frame.can_dlc;
    }
    else
        return -1;
}

// Get some data, timeout
// Returns the number of bytes read, return 0 on timeout
int canlcm_read_timeout(canlcm_driver *cld, unsigned char *d, int len, int timeout_s, int64_t *timestamp)
{
    struct can_frame frame;
    int bytes;
    fd_set rfds;

    struct timeval tv;
    tv.tv_sec = timeout_s;
    tv.tv_usec = 0;

    FD_ZERO(&rfds);
    FD_SET(cld->socket, &rfds);

    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
    if(ret == -1)
        perror("Select failure: ");
    else if(ret != 0)
    {
        if(FD_ISSET(cld->socket, &rfds))
        {
            bytes = read(cld->socket, frame, len);
            *timestamp = timestamp_now();
            if(bytes == 13)
            {
                memcpy(d, frame.data, frame.can_dlc);
                return frame.can_dlc;
            }
            else
                return -1;
        }
        else    // timeout
            return 0;
    }

