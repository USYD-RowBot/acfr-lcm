/*
 *  Seabotix AHRS data extractor
 *  Using libpcap to get around the stupid Seabotix implementaion
 *
 *  Christian Lees
 *  ACFR
 *  6/6/15
 */

#include <pcap.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <netinet/if_ether.h>
#include "acfr-common/timestamp.h"
#include "perls-lcmtypes/senlcm_seabotix_vlbv_t.h"

// libpcap network filter
#define NET_FILTER "host 230.0.0.0"
#define AHRS_PORT 65289

#define SIZE_ETHERNET 14

// Data conversion
#define d2f(x) *(float *)x
#define d2current(x) ((float)x / 10.0)
#define d2speed(x) (short)((short)(x - 0x80) * -1 * 4500 / 102)

typedef struct 
{
    lcm_t *lcm;
    pcap_t* descr;
} state_t;


int program_exit;
void
signal_handler(int sig_num)
{
    program_exit = 1;
}


int parse_vlbv(const unsigned char *d, lcm_t *lcm, int64_t timestamp)
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
   

void packet_callback(u_char *u,const struct pcap_pkthdr* pkthdr,const u_char* packet)
{
    state_t *state = (state_t *)u;
    
    if(program_exit)
        pcap_breakloop(state->descr);
    
    // check to see what port the data is on
    const struct ip *ip;
    ip = (struct ip*)(packet + sizeof(struct ether_header));
    
    // We are only interested if the packet protocol is UDP
    if(ip->ip_p != IPPROTO_UDP)
        return;
        
    const struct udphdr *udp;
    udp = (struct udphdr *)(packet + sizeof(struct ether_header) + sizeof(struct ip));
    
    // check the destination port
    if(udp->dest != AHRS_PORT)
        return;
    
    // get a pointer to the payload data
    const unsigned char *payload = (unsigned char *)(packet + sizeof(struct ether_header) + sizeof(struct ip) + sizeof(struct udphdr));
            
    // check to see if the packet is correct
    if(payload[0] == 0x69 && payload[1] == 0x54)
        parse_vlbv(payload, state->lcm, timestamp_now());       
       
}

int main(int argc,char **argv)
{
    char errbuf[PCAP_ERRBUF_SIZE];
    
    struct bpf_program fp;      /* hold compiled program     */
    bpf_u_int32 maskp;          /* subnet mask               */
    bpf_u_int32 netp;           /* ip                        */
    
    state_t state;

    if(argc != 2)
    {
        printf("Usage: %s <device>\n", basename(argv[0]));
        return 0;
    }
    
    char *dev = argv[1];    
    
    // Install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    /* grab a device to peak into... */
/*    dev = pcap_lookupdev(errbuf);
    if(dev == NULL)
    {
        printf("%s\n",errbuf);
        exit(1);
    }
*/
    pcap_lookupnet(dev,&netp,&maskp,errbuf);

    /* open device for reading. NOTE: defaulting to promiscuous mode*/
    state.descr = pcap_open_live(dev,BUFSIZ,1,-1,errbuf);
    if(state.descr == NULL)
    {
        printf("pcap_open_live(): %s\n",errbuf);
        exit(1);
    }


    /* Lets try and compile the program.. non-optimized */
    if(pcap_compile(state.descr, &fp, NET_FILTER, 0, netp) == -1)
    {
        fprintf(stderr,"Error calling pcap_compile\n");
        exit(1);
    }

    /* set the compiled program as the filter */
    if(pcap_setfilter(state.descr,&fp) == -1)
    {
        fprintf(stderr,"Error setting filter\n");
        exit(1);
    }
 
 
    // start LCM
    state.lcm = lcm_create(NULL);
 
    /* ... and loop */
    pcap_loop(state.descr, -1, packet_callback, (unsigned char *)&state);

    fprintf(stdout,"\nfinished\n");
    return 0;
}
