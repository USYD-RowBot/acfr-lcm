/*	Imagenex DeltaT driver for LCM
	Records data to a file, does not transmit it over the network.
	Listens to network for config changes.

	Christian Lees
	ACFR
	7/12/10
*/

#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <pthread.h>
#include <math.h>
#include <libgen.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include <bot_param/param_client.h>

#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"
#include "perls-lcmtypes/senlcm_deltat_config_t.h"
#include "perls-lcmtypes/senlcm_deltat_t.h"
#include "perls-lcmtypes/senlcm_deltat_ping_t.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"

#define DELTA_T_PORT "4040"

// Related to the switch data command sent to the multibeam to request data
#define DELTAT_SW_LENGTH 27

// Related to the sonar return data read from the multibeam after a switch data command
#define DELTAT_RETURN_BUFFER_LENGTH   1033
#define DELTAT_RETURN_DATA_SIZE 1000
#define DELTAT_RETURN_DATA_START_INDEX 32

// Related to the 837 output file format
#define DELTAT_837_FILE_HEADER_LENGTH 100
#define DELTAT_837_DATA_HEADER_LENGTH 12
#define DELTAT_837_DATA_HEADER_START_INDEX 100
#define DELTAT_837_RETURN_DATA_START_INDEX 112

#define DELTAT_837_IUX_TERM_CHAR_INDEX  8112
#define DELTAT_837_IVX_TERM_CHAR_INDEX 16112

#define NBYTES_8000 8192
#define NBYTES_16000 16384

#define NBYTES_TO_READ_8000 8013
#define NBYTES_TO_READ_16000 16013

#define DELTAT_TERM_CHAR 0xFC

#ifndef BOT_CONF_DIR
#define DEFAULT_BOT_CONF_PATH "../config/sirius.cfg"
#else
#define DEFAULT_BOT_CONF_PATH BOT_CONF_DIR "/sirius.cfg"
#endif

// DeltaT configuration variables, these can be changed by the mission controller
typedef struct
{
    int range;
    int startGain;
    int pulseLength;
    int frequency;
    int dataPoints;
    int delay;
    int autoRange;
    char *IP;
    int navAlt;
    pthread_mutex_t lock;
} deltaTConfig_t;

typedef struct
{
    double altitude;
    pthread_mutex_t lock;
} rdi_t;

typedef struct
{
    int sockFd;
    char *portStr;
    int connected;
} socketInfo_t;

int deltaTExit;

int acceptTimeout(int sockFd, struct sockaddr_storage *their_addr, socklen_t *addrsize, int timeout)
{

    fd_set rfds;
    FD_ZERO (&rfds);
    FD_SET (sockFd, &rfds);

    struct timeval tv;
    tv.tv_sec = timeout;
    tv.tv_usec = 0;

    int retFd = -1;

    int ret = select (sockFd + 1, &rfds, NULL, NULL, &tv);

    if(ret == -1)
        perror("acceptTimeout select()");
    else if(ret != 0)
        retFd = accept(sockFd, (struct sockaddr *)their_addr, addrsize);

    return retFd;
}

int checkSocket(int sockFd)
{
    fd_set rfds;
    FD_ZERO (&rfds);
    FD_SET (sockFd, &rfds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    select (sockFd + 1, &rfds, NULL, NULL, &tv);
    if(FD_ISSET(sockFd, &rfds))
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

    socketInfo_t *socketInfo = (socketInfo_t *)u;

    printf("Opening Socket!\n");


    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;  // use IPv4 or IPv6, whichever
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;     // fill in my IP for me

    getaddrinfo(NULL, socketInfo->portStr, &hints, &res);

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
    socketInfo->sockFd = -1;
    socketInfo->connected = 0;
    addrsize = sizeof(their_addr);
    while (1)
    {
        if(!socketInfo->connected)
        {
            // we can listen for a new connection
            if((tmpfd = acceptTimeout(sockfd, (struct sockaddr_storage *)&their_addr, &addrsize, 1)) > 0)
            {
                socketInfo->sockFd = tmpfd;
                socketInfo->connected = 1;
            }
        }
        if(!checkSocket(socketInfo->sockFd))
            socketInfo->connected = 0;

        usleep(500000);
        sched_yield();

        if(deltaTExit)
            break;
    }
    printf("Exiting socket thread\n");
    close(socketInfo->sockFd);
    close(sockfd);
    return NULL;
}

void composeSwitchPacket(deltaTConfig_t *cfg, char *switchPacket, int sendPacketNumber, rdi_t *rdi)
{

    // check to see if we are auto ranging
    if(cfg->autoRange)
    {
        // use the altitude from the RDI
        pthread_mutex_lock(&rdi->lock);
        if(rdi->altitude > 0)
        {
            double altitude = rdi->altitude;
            pthread_mutex_unlock(&rdi->lock);

            pthread_mutex_lock(&cfg->lock);
            if(altitude < 5.0)
                cfg->range = 5;
            else if(altitude < 60.0)
                cfg->range = 10*(int)ceil(altitude/10.0);
            else if(altitude < 100.0)
                cfg->range = 20*(int)ceil(altitude/20.0);
            else if(altitude >= 100.0)
                cfg->range = 100;
            else
                cfg->range = 100;

            // pulse length
            cfg->pulseLength = cfg->range * 6;

            // set start gain based on Mike Jakuba's observation that the start gain needs to be higher at short ranges
            // to avoid missing data around the nadir
            if(cfg->range == 5)
                cfg->startGain = 9;
            else if(cfg->range == 10)
                cfg->startGain = 6;
            else
                cfg->startGain = 3;

            pthread_mutex_unlock(&cfg->lock);
        }
        else
        {
            fprintf(stderr, "Altitude from DVL not available\n");
            pthread_mutex_unlock(&rdi->lock);
        }
    }

    // now compose the switch packet
    pthread_mutex_lock(&cfg->lock);

    switchPacket[0]  = 0xFE;
    switchPacket[1]  = 0x44;
    switchPacket[2]  = 0x10;				//Head ID
    switchPacket[3]  = cfg->range;			//Range
    switchPacket[4]  = 0;   				//Range Offset
    switchPacket[5]  = 0;
    switchPacket[6]  = 0;
    switchPacket[7]  = 0;
    switchPacket[8]  = cfg->startGain;		//Gain
    switchPacket[9]  = 1;					//LOGF
    switchPacket[10] = 10;					//Absorption
    switchPacket[11] = 0;
    switchPacket[12] = 0;
    switchPacket[13] = sendPacketNumber;	//0-15
    switchPacket[14] = cfg->pulseLength/10;		//Pulse Length
    switchPacket[15] = 0;
    switchPacket[16] = 0;
    switchPacket[17] = 0;
    switchPacket[18] = 0;
    switchPacket[19] = cfg->dataPoints/1000;//8 = 8000 data points (IUX mode) 16 = 16000 (IVX mode)
    switchPacket[20] = 8;					//8 Data Bits
    switchPacket[21] = 0x80; // to get orientation
    switchPacket[22] = 0;
    switchPacket[23] = 0;
    switchPacket[24] = cfg->delay/2;        //Switch Delay

    if(cfg->frequency == 245)
        switchPacket[25] = 83;				//Frequency, 83 = 245kHz
    if(cfg->frequency == 250)
        switchPacket[25] = 84;				//Frequency, 84 = 250kHz
    if(cfg->frequency == 255)
        switchPacket[25] = 85;				//Frequency, 85 = 255kHz
    if(cfg->frequency == 260)
        switchPacket[25] = 86;				//Frequency, 86 = 260kHz
    if(cfg->frequency == 675)
        switchPacket[25] = 169;				//Frequency, 169 (0xA9) = 675kHz

    switchPacket[26] = 0xFD;

    pthread_mutex_unlock(&cfg->lock);
}

void writeRPH(char *pingHeader, double roll, double pitch, double heading)
{
    // convert to degrees from radians
    // assume range of +- pi/2 for roll and pitch
    // and heading of 0-2pi
    roll *= 360.0/(2.0*M_PI);
    pitch *= 360.0/(2.0*M_PI);
    heading *= 360.0/(2.0*M_PI);
    // convert to imagenex desired format, making sure high bit is set
    // to indicate presence of valid rph
    uint16_t r = (uint16_t)(roll * 10.0 + 900.0) | 0x8000;
    uint16_t p = (uint16_t)(pitch * 10.0 + 900.0) | 0x8000;
    uint16_t h = (uint16_t)(heading * 10.0) | 0x8000;

    // pack into the values
    pingHeader[82] = (0xFF00 & p) >> 8;
    pingHeader[83] = (0xFF & p);

    pingHeader[84] = (0xFF00 & r) >> 8;
    pingHeader[85] = (0xFF & r);

    pingHeader[86] = (0xFF00 & h) >> 8;
    pingHeader[87] = (0xFF & h);
}

void writeHeader(char *pingHeader, deltaTConfig_t *cfg, int64_t timeStamp)
{

    // work out the time stamp
    struct tm *tm;
    time_t currentTime;
    int64_t uSeconds;

    currentTime = (time_t)(timeStamp/1000000);
    uSeconds = (timeStamp - (int64_t)currentTime * 1000000);
    tm = gmtime(&currentTime);
    char *month = NULL;

    pingHeader[0] = '8';
    pingHeader[1] = '3';
    pingHeader[2] = '7';

    if(cfg->dataPoints == 8000)
    {
        pingHeader[3] = 10;
        pingHeader[4] = ((NBYTES_8000&0xFF00)>>8);
        pingHeader[5] = (NBYTES_8000&0x00FF);
        pingHeader[6] = ((NBYTES_TO_READ_8000&0xFF00)>>8);
        pingHeader[7] = (NBYTES_TO_READ_8000&0x00FF);
    }
    else
    {
        pingHeader[3] = 11;
        pingHeader[4] = ((NBYTES_16000&0xFF00)>>8);
        pingHeader[5] = (NBYTES_16000&0x00FF);
        pingHeader[6] = ((NBYTES_TO_READ_16000&0xFF00)>>8);
        pingHeader[7] = (NBYTES_TO_READ_16000&0x00FF);
    }

    switch(tm->tm_mon)
    {
    case (0)  :
        month = "JAN";
        break;
    case (1)  :
        month = "FEB";
        break;
    case (2)  :
        month = "MAR";
        break;
    case (3)  :
        month = "APR";
        break;
    case (4)  :
        month = "MAY";
        break;
    case (5)  :
        month = "JUN";
        break;
    case (6)  :
        month = "JUL";
        break;
    case (7)  :
        month = "AUG";
        break;
    case (8)  :
        month = "SEP";
        break;
    case (9)  :
        month = "OCT";
        break;
    case (10) :
        month = "NOV";
        break;
    case (11) :
        month = "DEC";
        break;
    }

    snprintf(&pingHeader[8], 12, "%02d-%s-%04d", tm->tm_mday,month,1900+tm->tm_year);
    snprintf(&pingHeader[20], 9, "%02d:%02d:%02d", tm->tm_hour,tm->tm_min,tm->tm_sec);
    snprintf(&pingHeader[29], 4, ".%02d", (int)uSeconds/10000);


    pingHeader[37] = 0x80;
    pingHeader[38] = cfg->startGain;
    pingHeader[42] = 10;  //reserved
    pingHeader[44] = cfg->pulseLength/10;

    int temp = (int)(1500.0*10.0);
    pingHeader[46] = (0x80 | ((temp&0x7F00)>>8));
    pingHeader[47] = (temp&0x00FF);

    pingHeader[80] = ((cfg->frequency&0xFF00)>>8);
    pingHeader[81] = (cfg->frequency&0x00FF);

    // these are set in another function called writeRPH
    // pitch
    // roll
    // yaw

    pingHeader[88] = ((100&0xFF00)>>8);  //rep_rate hard coded to 100 millisec
    pingHeader[89] = (100&0x00FF);
    pingHeader[90] = 90;                       //display gain
}


// update the deltaT config structure from messages send from the mission controller
static void deltaTConfigCallback(const lcm_recv_buf_t *rbuf, const char *channel,
                                 const senlcm_deltat_config_t *cfg, void *user)
{

    deltaTConfig_t *deltaTConfig = (deltaTConfig_t *)user;

    // lock the data
    pthread_mutex_lock(&deltaTConfig->lock);
    deltaTConfig->range = cfg->range;
    deltaTConfig->startGain = cfg->startGain;
    deltaTConfig->pulseLength = cfg->pulseLength;
    deltaTConfig->frequency = cfg->frequency;
    deltaTConfig->dataPoints = cfg->dataPoints;
    deltaTConfig->delay = cfg->delay;
    deltaTConfig->autoRange = cfg->autoRange;
    pthread_mutex_unlock(&deltaTConfig->lock);
}

static void
acfr_nav_callback (const lcm_recv_buf_t *rbuf, const char *channel, const acfrlcm_auv_acfr_nav_t *nav, void *user)
{
    rdi_t *rdi = (rdi_t *)user;
    pthread_mutex_lock(&rdi->lock);
    rdi->altitude = nav->altitude;
    pthread_mutex_unlock(&rdi->lock);
}


static void rdiPD5Callback(const lcm_recv_buf_t *rbuf, const char *channel,
                           const senlcm_rdi_pd5_t *msg, void *user)
{

    rdi_t *rdi = (rdi_t *)user;
    pthread_mutex_lock(&rdi->lock);
    rdi->altitude = msg->pd4.altitude;
    pthread_mutex_unlock(&rdi->lock);
}

static void rdiPD4Callback(const lcm_recv_buf_t *rbuf, const char *channel,
                           const senlcm_rdi_pd4_t *msg, void *user)
{

    rdi_t *rdi = (rdi_t *)user;
    pthread_mutex_lock(&rdi->lock);
    rdi->altitude = msg->altitude;
    pthread_mutex_unlock(&rdi->lock);
}


// copied from Beej's guide to network programming
int recvtimeout(int s, char *buf, int len, int timeout)
{
    fd_set fds;
    int n;
    struct timeval tv;
    // set up the file descriptor set
    FD_ZERO(&fds);
    FD_SET(s, &fds);
    // set up the struct timeval for the timeout
    tv.tv_sec = timeout;
    tv.tv_usec = 0;
    // wait until timeout or data received

    n = select(s+1, &fds, NULL, NULL, &tv);
    if (n == 0) return -2; // timeout!
    if (n == -1) return -1; // error
    // data must be here, so do a normal recv()
    return recv(s, buf, len, 0);
}


void mp_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_raw_ascii_t *msg, void *u)
{

    int i_value;
    char *ptr;
    int ret;

    deltaTConfig_t *config = (deltaTConfig_t *)u;

    if(strstr(msg->msg, "DELTA_T") != NULL)
    {
        // the message is for us
        pthread_mutex_lock(&config->lock);
        if((ptr = strstr(msg->msg, "autorange=")) != NULL)
        {
            ptr += strlen("autorange=");
            ret = sscanf(ptr, "%d", &i_value);
            if(ret == 1)
                config->autoRange = i_value;
            printf("Autorange changed to %d\n", config->autoRange);
        }

        if((ptr = strstr(msg->msg, "range=")) != NULL)
        {
            ptr += strlen("range=");
            ret = sscanf(ptr, "%d", &i_value);
            if(ret == 1)
                config->range = i_value;
            printf("Range changed to %d\n", config->range);
        }

        if((ptr = strstr(msg->msg, "start_gain=")) != NULL)
        {
            ptr += strlen("start_gain=");
            ret = sscanf(ptr, "%d", &i_value);
            if(ret == 1)
                config->startGain = i_value;
            printf("Start gain changed to %d\n", config->startGain);
        }

        if((ptr = strstr(msg->msg, "pulse_length=")) != NULL)
        {
            ptr += strlen("pulse_length=");
            ret = sscanf(ptr, "%d", &i_value);
            if(ret == 1)
                config->pulseLength = i_value;
            printf("Pulse length changed to %d\n", config->pulseLength);
        }

        pthread_mutex_unlock(&config->lock);
    }
}



void signalHandler(int sigNum)
{
    // do a safe exit
    deltaTExit = 1;
}

// Process LCM messages with callbacks
static void *
lcmThread (void *context)
{
    lcm_t *lcm = (lcm_t *) context;

    while (!deltaTExit)
    {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        lcmu_handle_timeout(lcm, &tv);
    }
    return 0;
}

void print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void parse_args (int argc, char **argv, char **channel_name, char **channel_name2)
{
    int opt;

    const char *default_name = "DEFAULT";
    *channel_name = malloc(strlen(default_name)+1);
    strcpy(*channel_name, default_name);

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
            case 'h':
                print_help (0, argv);
                break;
            case 'n':
                free(*channel_name);
                *channel_name = malloc(200);
                snprintf(*channel_name, 200, "%s.DELTAT", (char *)optarg);
		*channel_name2 = malloc(200);
		snprintf(*channel_name2, 200, "%s.DELTAT_PING", (char *)optarg);
                break;
        }
    }
}

int main(int argc, char *argv[])
{

    // install the signal handler
    deltaTExit = 0;
    signal(SIGINT, signalHandler);

    double roll, pitch, heading;

    char *channel_name;
    char *channel_name2;
    parse_args(argc, argv, &channel_name, &channel_name2);
    // now we have a connection to the sonar we can start LCM so we can listen for messages
    // about the config settings
    printf("Initialising LCM: ");
    lcm_t *lcm = lcm_create (NULL);
    printf("OK\n");

    // read the config file
    BotParam *cfg;
    char rootkey[64];
    char key[64];

    cfg = bot_param_new_from_server (lcm, 1);

    //char *path = getenv ("BOT_CONF_PATH");
    //if (!path)
    //    path = DEFAULT_BOT_CONF_PATH;
    //cfg = bot_param_new_from_file(path);
    //if(cfg == NULL)
   // {
    //    printf("cound not open config file\n");
    //    return 0;
   // }

    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    deltaTConfig_t deltaTConfig;
    pthread_mutex_init(&deltaTConfig.lock, NULL);

    sprintf(key, "%s.range", rootkey);
    deltaTConfig.range = bot_param_get_int_or_fail(cfg, key);

    sprintf(key, "%s.startGain", rootkey);
    deltaTConfig.startGain = bot_param_get_int_or_fail(cfg, key);

    sprintf(key, "%s.pulseLength", rootkey);
    deltaTConfig.pulseLength = bot_param_get_int_or_fail(cfg, key);

    sprintf(key, "%s.frequency", rootkey);
    deltaTConfig.frequency = bot_param_get_int_or_fail(cfg, key);

    sprintf(key, "%s.dataPoints", rootkey);
    deltaTConfig.dataPoints = bot_param_get_int_or_fail(cfg, key);

    sprintf(key, "%s.delay", rootkey);
    deltaTConfig.delay = bot_param_get_int_or_fail(cfg, key);

    sprintf(key, "%s.autoRange", rootkey);
    deltaTConfig.autoRange = bot_param_get_int_or_fail(cfg, key);

    sprintf(key, "%s.IP", rootkey);
    deltaTConfig.IP = bot_param_get_str_or_fail(cfg, key);

    sprintf(key, "%s.useNavAlt", rootkey);
    deltaTConfig.navAlt = bot_param_get_boolean_or_fail(cfg, key);

    // if we got here then we have all the config variables
    // open the port to the sonar head
    struct addrinfo hints, *deltaTAddr;
    int deltaTFd;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    getaddrinfo(deltaTConfig.IP, DELTA_T_PORT, &hints, &deltaTAddr);
    deltaTFd = socket(deltaTAddr->ai_family, deltaTAddr->ai_socktype, deltaTAddr->ai_protocol);
    int flags = fcntl(deltaTFd, F_GETFL, 0);
    fcntl(deltaTFd, F_SETFL, flags | O_NONBLOCK);

    //if(connect(deltaTFd, deltaTAddr->ai_addr, deltaTAddr->ai_addrlen) < 0) {
    //	perror("DeltaT connect");
    //	return 1;
    //}

    // all modes need the socket opened
    socketInfo_t socketInfo;
    pthread_t socketthread;
    socketInfo.portStr = "4040";
    pthread_create(&socketthread, NULL, socket_handler, &socketInfo);

    // listen for config changes
    printf("Subscribing to deltaTConfiguration: ");
    senlcm_deltat_config_t_subscribe (lcm, "DELTA_T_CONFIG", &deltaTConfigCallback, &deltaTConfig);
    printf("OK\n");

    // listen to the RDI
    rdi_t rdi;
    pthread_mutex_init(&rdi.lock, NULL);

    // also listen for changes on the newer MP_PASSOUT channel
    senlcm_raw_ascii_t_subscribe(lcm, "MP_PASSOUT", &mp_callback, &deltaTConfig);

    // need a way to know which mode we are using...
    // else a lot of messages are pumped out when types are not correct...
    // but otherwise harmless (this has been checked in code)
    printf("Subscribing to DVL messages for altitude: ");
    if(deltaTConfig.navAlt)
        acfrlcm_auv_acfr_nav_t_subscribe (lcm, "ACFR_NAV", &acfr_nav_callback, &rdi);
    else
    {
        senlcm_rdi_pd5_t_subscribe(lcm, "RDI", &rdiPD5Callback, &rdi);
        senlcm_rdi_pd4_t_subscribe(lcm, "RDI", &rdiPD4Callback, &rdi);
    }

    // create an LCM thread to listen so the command pass through will work
    printf("Creating LCM Thread: ");
    pthread_t tid;
    pthread_create(&tid, NULL, lcmThread, lcm);
    pthread_detach(tid);
    printf("OK\n");

    int packetNumber = 0;
    char switchPacket[27];
    int bytesRead;
    int64_t timeStamp;
    char *returnData = (char *)malloc(DELTAT_RETURN_BUFFER_LENGTH);
    // as not every byte is set, make sure it is initialised at zero everywhere
    char *pingBuffer = (char *)calloc(NBYTES_16000, 1);
    /*if (deltaTConfig.dataPoints == 8000)
    {
        pingBuffer = (char *)malloc(NBYTES_8000);
    }
    else
    {
        pingBuffer = (char *)malloc(NBYTES_16000);
    }*/

    senlcm_deltat_t deltaTStats;
    memset( &deltaTStats, 0, sizeof( senlcm_deltat_t ) );


    int ret;
    int timeout;
    struct timeval tv;
    fd_set rfds;
    int connected = 0;

    // main loop
    printf("Data Capture: 0\n");
    while(!deltaTExit)
    {
        if(!connected)
        {
            tv.tv_sec = 2;
            tv.tv_usec = 0;
            FD_ZERO(&rfds);
            FD_SET(deltaTFd, &rfds);
            int ret = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
            if(ret != -1)
            {
                if(connect(deltaTFd, deltaTAddr->ai_addr, deltaTAddr->ai_addrlen) < 0)
                {
                    perror("DeltaT connect");
                    continue;
                }
                else
                {
                    printf("DeltaT: got a connection\n");
                    connected = 1;
                }
            }
            else
            {
                printf("Select timeout\n");
                continue;
            }
        }


        // compose the switch packet
        composeSwitchPacket(&deltaTConfig, switchPacket, packetNumber, &rdi);

        // send the packet
        send(deltaTFd, switchPacket, sizeof(switchPacket), 0);

        // get some data back, we want to use a recv with a timeout so we can exit the program if
        // the sonar head is not present or has stopped sending data
        bytesRead = 0;
        timeout = 0;
        while(bytesRead < DELTAT_RETURN_BUFFER_LENGTH)
        {
            ret = recvtimeout(deltaTFd, &returnData[bytesRead], DELTAT_RETURN_BUFFER_LENGTH - bytesRead, 5); // 5s timeout
            if(ret > 0)
                bytesRead += ret;
            if(ret == -2)
            {
                timeout = 1;
                connected = 0;  // we can assume that if we haven't seen any data for a while the connection has closed
                close(deltaTFd);
                deltaTFd = socket(deltaTAddr->ai_family, deltaTAddr->ai_socktype, deltaTAddr->ai_protocol);
                int flags = fcntl(deltaTFd, F_GETFL, 0);
                fcntl(deltaTFd, F_SETFL, flags | O_NONBLOCK);
                break;
            }
        }

        if(!timeout)
        {
            // a quick check to see if its the correct packet
            if((int)returnData[5] != packetNumber)
            {
                fprintf(stderr, "Delta T: wrong packet number returned\n");
                continue;
            }

            // if its the first packet
            if(packetNumber == 0)
            {
                printf("\rData Capture: %d", deltaTStats.count);
                fflush(NULL);
                timeStamp = timestamp_now();
                writeHeader(pingBuffer, &deltaTConfig, timeStamp);

                // copy the ping header
                memcpy(&pingBuffer[DELTAT_837_FILE_HEADER_LENGTH], returnData, DELTAT_837_DATA_HEADER_LENGTH);

                // append the sensed RPY data (used with Aggelos multibeam, should be empty/zero otherwise
                // and thus harmless, must be done here as it is onhly in packet 0
                if(deltaTConfig.dataPoints == 8000)
                    memcpy(&pingBuffer[DELTAT_837_IUX_TERM_CHAR_INDEX+13], &returnData[13], 9);
                else
                    memcpy(&pingBuffer[DELTAT_837_IVX_TERM_CHAR_INDEX+13], &returnData[13], 9);

                // if we are extracting the sonars RPH data then it will be in packet 0
                if(returnData[13] != 0)
                {

                    int binary_1 = (unsigned char)(returnData[16]);
                    int binary_2 = (unsigned char)(returnData[17]);
                    roll    = ( (binary_2&0x80)== 0 ? (((binary_2) << 8) | binary_1)*2.0*M_PI/65536 :
                                ( (((binary_2) << 8) | binary_1) - 65536)*(2.0*M_PI/65536) );

                    //printf("Roll: %g\n", roll);

                    binary_1 = (unsigned char)(returnData[14]);
                    binary_2 = (unsigned char)(returnData[15]);
                    pitch   = ( (binary_2&0x80)== 0 ? (((binary_2) << 8) | binary_1)*2.0*M_PI/65536 :
                                ( (((binary_2) << 8) | binary_1) - 65536)*(2.0*M_PI/65536) );

                    binary_1 = (unsigned char)(returnData[18]);
                    binary_2 = (unsigned char)(returnData[19]);
                    heading = ( (binary_2&0x80)== 0 ? (((binary_2) << 8) | binary_1)*2.0*M_PI/65536 :
                                ( (((binary_2) << 8) | binary_1) - 65536)*(2.0*M_PI/65536) );

                    /*if(returnData[13] == 2) {
                    	// signs are reversed
                    	roll = -roll;
                    	pitch = -pitch;
                    	heading = -heading;
                    }*/

                    // put heading in 0 - 359
                    // check this carefully!
                    // heading += M_PI;
                    if (heading < 0.0)
                        heading += 2.0 * M_PI;

                    writeRPH(pingBuffer, roll, pitch, heading);

                }
            }

            // copy the ping data in
            memcpy((void *)&pingBuffer[DELTAT_837_RETURN_DATA_START_INDEX + DELTAT_RETURN_DATA_SIZE * packetNumber],
                   (void *)&returnData[DELTAT_RETURN_DATA_START_INDEX], DELTAT_RETURN_DATA_SIZE);

            packetNumber++;

            if( packetNumber > (deltaTConfig.dataPoints/1000)-1)
            {

                deltaTStats.utime = timeStamp;
                deltaTStats.count++;
                senlcm_deltat_ping_t ping;
                ping.utime = deltaTStats.utime;
                ping.count = deltaTStats.count;

                if (deltaTConfig.dataPoints == 8000)
                {
                    pingBuffer[DELTAT_837_IUX_TERM_CHAR_INDEX] = DELTAT_TERM_CHAR;  //close the complete ping

                    //write to the output file
                    ping.size = NBYTES_8000;
                    ping.ping = pingBuffer;

                    // PUBLISH LCM STRUCT WITH THIS DATA
                }
                else
                {
                    pingBuffer[DELTAT_837_IVX_TERM_CHAR_INDEX] = DELTAT_TERM_CHAR;  //close the complete ping

                    //close the complete ping
                    //write to the output file
                    ping.size = NBYTES_16000;
                    ping.ping = pingBuffer;

                    // PUBLISH LCM STRUCT WITH THIS DATA
                }
                //reset packet number for the next ping
                packetNumber = 0;

                // report that we have written a packet
                senlcm_deltat_t_publish(lcm, channel_name, &deltaTStats);
                // report the packet itself
                senlcm_deltat_ping_t_publish(lcm, channel_name2, &ping);
            }

            // send it out to the listening deltat windows computer if it is connected
            if(socketInfo.connected)
                send(socketInfo.sockFd, returnData, DELTAT_RETURN_BUFFER_LENGTH, 0);


        }
    }


    // wait for the LCM thread to exit

    pthread_join(tid, NULL);
    pthread_join(socketthread, NULL);
    close(deltaTFd);
    free(returnData);
    free(pingBuffer);
}
