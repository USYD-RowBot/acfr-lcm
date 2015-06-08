#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

#include "acfr-common/timestamp.h"
#include "perls-lcmtypes/senlcm_seabotix_vlbv_t.h"
#include "acfr-common/socket.h"

#define MULTICAST_ADDRESS "230.0.0.0"
#define MULTICAST_PORT 65289
#define MSG_LEN 136

#define d2f(x) *(float *)x
#define d2current(x) ((float)x / 10.0)
#define d2speed(x) (short)((short)(x - 0x80) * -1 * 4500 / 102)

int parse_vlbv(char *d, lcm_t *lcm, int64_t timestamp)
{
    senlcm_seabotix_vlbv_t rov;
    memset(&rov, 0, sizeof(senlcm_seabotix_vlbv_t));
    
    rov.utime = timestamp;
    rov.heading = d2f(&d[8]);
    rov.depth = d2f(&d[12]);
    rov.pitch = d2f(&d[16]);
    rov.roll = d2f(&d[20]);
    rov.turns = d2f(&d[24]);
    rov.temperature_internal = d2f(&d[28]);
    rov.temperature_external = d2f(&d[32]);
    
    rov.PF_faults = d[36];
    rov.PF_current = d2current(d[37]);
    rov.PF_speed = d2speed(d[38]);
    rov.PF_temperature = d[39];
    
    rov.PA_faults = d[40];
    rov.PA_current = d2current(d[41]);
    rov.PA_speed = d2speed(d[42]);
    rov.PA_temperature = d[43];

    rov.PV_faults = d[44];
    rov.PV_current = d2current(d[45]);
    rov.PV_speed = d2speed(d[46]);
    rov.PV_temperature = d[47];

    rov.SF_faults = d[48];
    rov.SF_current = d2current(d[49]);
    rov.SF_speed = d2speed(d[50]);
    rov.SF_temperature = d[51];

    rov.SA_faults = d[52];
    rov.SA_current = d2current(d[53]);
    rov.SA_speed = d2speed(d[54]);
    rov.SA_temperature = d[55];

    rov.SV_faults = d[56];
    rov.SV_current = d2current(d[57]);
    rov.SV_speed = d2speed(d[58]);
    rov.SV_temperature = d[59];

    senlcm_seabotix_vlbv_t_publish(lcm, "SEABOTIX_STATS", &rov);
    
    return 1;
}
    
    
    

int program_exit;
void
signal_handler(int sig_num)
{
    program_exit = 1;
}


int main (int argc, char *argv[])
{
    // Install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);


    // open the multicast socket to receive data from the ROV
    int sockfd = create_udp_multicast_listen(MULTICAST_ADDRESS, MULTICAST_PORT);
    if(sockfd < 0)
    {
        printf("Could not create the multicast listen port\n");
        return 0;
    }
    
    // start lcm
    //lcm_t *lcm = lcm_create(NULL);
    
    
    fd_set rfds;
    struct timeval tv;
    uint64_t timestamp;
    char buf[256];
    
    buf[0] = 0;
    recvfrom(sockfd, &buf, 1, MSG_ERRQUEUE, NULL, NULL);
    printf("Got message from error queue: %d\n", buf[0]);
    
    // Loop and listen for data on the UDP socket
    while(!program_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(sockfd, &rfds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
	    
        int ret = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            timestamp = timestamp_now();
            printf("Got data\n");
            // read the data from the socket, it'll be 136 bytes but first lets check for the preamble
            do
            {
                recvfrom(sockfd, &buf, 2, 0, NULL, NULL);
                printf("Got data, %X, %X\n", buf[0] && 0xFF, buf[1] & 0xFF);
            } while(buf[0] != 0x69 && buf[1] != 0x54);
            
            // now read the rest of the data
            int data_len = 2;
            while(data_len < MSG_LEN)
                data_len += recvfrom(sockfd, &buf[data_len], MSG_LEN - data_len, 0, NULL, NULL);
                
            //parse_vlbv(buf, lcm, timestamp);
        }
        else
            printf("Timeout\n");
    }
    
    close(sockfd);
    
    return 1;
}
            
