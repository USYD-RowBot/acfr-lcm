/*	LCM interface to the ACFR strobe/camera trigger board
    using a TCP interface

	Config read from the master.cfg file

	Christian Lees
	ACFR
	19/4/11
*/

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <libgen.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>

#include <lcm/lcm.h>
#include <bot_param/param_client.h>
#include "perls-common/lcm_util.h"
#include "perls-common/serial.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_trigger_t.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_trigger_out_t.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_temp_t.h"


//packet structure
// STX	| type | direction | data | ETX

//Message start & end bytes
#define STX						0x02
#define ETX						0x03

//Message direction
#define SET						0x01
#define GET						0x04
#define ACK						0x0F

//message types
#define MSG_TYPE_BASE			0x0F	//must be higher than the control codes above
#define AUTHENTICATION			MSG_TYPE_BASE + 1
#define IP_SETTINGS				MSG_TYPE_BASE + 2
#define STROBE_SETTINGS			MSG_TYPE_BASE + 3
#define STROBE_OVERVIEW			MSG_TYPE_BASE + 4
#define STROBE_STATUS_CAP		MSG_TYPE_BASE + 5
#define STROBE_STATUS_LED		MSG_TYPE_BASE + 6
#define STROBE_STATUS_RUNTIME	MSG_TYPE_BASE + 7

#define TRIGGER_FREQ			MSG_TYPE_BASE + 8
#define TRIGGER_PULSEWIDTH		MSG_TYPE_BASE + 9
#define TRIGGER_STROBE_DLY		MSG_TYPE_BASE + 10
#define TRIGGER_TICKLE			MSG_TYPE_BASE + 11
#define TRIGGER_TEMP            MSG_TYPE_BASE + 13

#define TRIGGER_STATUS			MSG_TYPE_BASE + 19
#define TRIGGER_ENABLE			MSG_TYPE_BASE + 20
#define TRIGGER_DISABLE			MSG_TYPE_BASE + 21

// command messages
#define SET_FREQ				1
#define SET_DELAY				2
#define	SET_WIDTH				3
#define SET_STATE				4
#define SET_ALL					5

typedef struct
{
    int triggerFd;
    pthread_mutex_t *mutex;
    lcm_t *lcm;
    int enabled;
    int keepAliveCount;
} state_t;

typedef struct
{
    char msg[32];
    int size;
} triggerMsg_t;


unsigned short ui_sprintf(char * loc, const char *format, ...)
{
    unsigned short num = 0;
    va_list args;
    va_start (args, format);
    num = vsprintf (loc, format, args);
    va_end (args);

    return num;
}


unsigned short setTriggerFrequency(char * pktPtr, double freq)
{
    unsigned short i = 0;

    *(pktPtr+i++) = STX;
    *(pktPtr+i++) = TRIGGER_FREQ;
    *(pktPtr+i++) = SET;

    i += ui_sprintf((pktPtr+i), "%0.2f", freq);

    *(pktPtr+i++) = ETX;

    return i;		//return bytes in frame
}

unsigned short setPulseWidthUs(char * pktPtr, unsigned int pulseWidthUs)
{
    unsigned short i = 0;

    *(pktPtr+i++) = STX;
    *(pktPtr+i++) = TRIGGER_PULSEWIDTH;
    *(pktPtr+i++) = SET;

    i += ui_sprintf((pktPtr+i), "%i", pulseWidthUs);

    *(pktPtr+i++) = ETX;

    return i;		//return bytes in frame
}

unsigned short setStrobeDelayUs(char * pktPtr, unsigned int strobeDelayUs)
{
    unsigned short i = 0;

    *(pktPtr+i++) = STX;
    *(pktPtr+i++) = TRIGGER_STROBE_DLY;
    *(pktPtr+i++) = SET;

    i += ui_sprintf((pktPtr+i), "%i", strobeDelayUs);

    *(pktPtr+i++) = ETX;

    return i;		//return bytes in frame
}

unsigned short setTriggerSettings(char * pktPtr, double freq, unsigned int pulseWidthUs, unsigned int strobeDelayUs)
{
    unsigned short i = 0;

    *(pktPtr+i++) = STX;
    *(pktPtr+i++) = STROBE_SETTINGS;
    *(pktPtr+i++) = SET;

    i += ui_sprintf((pktPtr+i), "%0.2f,", freq);
    i += ui_sprintf((pktPtr+i), "%i,", strobeDelayUs);
    i += ui_sprintf((pktPtr+i), "%i", pulseWidthUs);

    *(pktPtr+i++) = ETX;

    return i;		//return bytes in frame
}

unsigned short enableTrigger(char * pktPtr)
{
    unsigned short i = 0;

    *(pktPtr+i++) = STX;
    *(pktPtr+i++) = TRIGGER_ENABLE;
    *(pktPtr+i++) = SET;
    *(pktPtr+i++) = ETX;

    return i;		//return bytes in frame
}

unsigned short disableTrigger(char * pktPtr)
{
    unsigned short i = 0;

    *(pktPtr+i++) = STX;
    *(pktPtr+i++) = TRIGGER_DISABLE;
    *(pktPtr+i++) = SET;
    *(pktPtr+i++) = ETX;

    return i;		//return bytes in frame
}

//send this every 15s when enabled
unsigned short tickleTrigger(char * pktPtr)
{
    unsigned short i = 0;

    *(pktPtr+i++) = STX;
    *(pktPtr+i++) = TRIGGER_TICKLE;
    *(pktPtr+i++) = SET;
    *(pktPtr+i++) = ETX;

    return i;		//return bytes in frame
}

void triggerWrite(state_t *state, triggerMsg_t *triggerMsg)
{
    pthread_mutex_lock(state->mutex);
    if(triggerMsg->msg[1] == TRIGGER_ENABLE)
        printf("Sending start message\n");
    else if(triggerMsg->msg[1] == TRIGGER_DISABLE)
        printf("Sending stop message\n");
    else if(triggerMsg->msg[1] == TRIGGER_TICKLE)
        printf("Sending tickle message\n");
    else
        printf("Sending unknown message\n");


    write(state->triggerFd, triggerMsg->msg, triggerMsg->size);
    pthread_mutex_unlock(state->mutex);
}

// used to send the trickle command to the strobe every 15 seconds
void heartBeatHandler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    if(state->enabled)
    {
//		printf("KA = %d\n", state->keepAliveCount);
        if(state->keepAliveCount == 14)
        {
            triggerMsg_t *triggerMsg = (triggerMsg_t *)malloc(sizeof(triggerMsg_t));
            triggerMsg->size = tickleTrigger(triggerMsg->msg);
            triggerWrite(state, triggerMsg);
            free(triggerMsg);
            state->keepAliveCount = 0;
        }
        else
            state->keepAliveCount++;
    }

    // publish a copy of the camera trigger message on CAMERA_TRIGGER.OUT
    acfrlcm_auv_camera_trigger_out_t ct;
    ct.enabled = state->enabled;
    ct.utime = timestamp_now();
    acfrlcm_auv_camera_trigger_out_t_publish(state->lcm, "CAMERA_TRIGGER.OUT", &ct);

}

void cameraTriggerHandler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_camera_trigger_t *ct, void *u)
{
    state_t *state = (state_t *)u;
    triggerMsg_t *triggerMsg;

    triggerMsg = (triggerMsg_t *)malloc(sizeof(triggerMsg_t));
    switch(ct->command)
    {
    case SET_FREQ:
        triggerMsg->size = setTriggerFrequency(triggerMsg->msg, ct->freq);
        triggerWrite(state, triggerMsg);
        break;
    case SET_DELAY:
        triggerMsg->size = setStrobeDelayUs(triggerMsg->msg, ct->strobeDelayUs);
        triggerWrite(state, triggerMsg);
        break;
    case SET_WIDTH:
        triggerMsg->size = setPulseWidthUs(triggerMsg->msg, ct->pulseWidthUs);
        triggerWrite(state, triggerMsg);
        break;
    case SET_STATE:
        if(ct->enabled)
        {
            if(!state->enabled)
            {
                // This is done this way so we don't send multiple start messages
                // as the trigger board fails if this happens
                state->enabled = 1;
                triggerMsg->size = enableTrigger(triggerMsg->msg);
                triggerWrite(state, triggerMsg);
            }

        }
        else
        {
            if(state->enabled)
            {
                state->enabled = 0;
                triggerMsg->size = disableTrigger(triggerMsg->msg);
                triggerWrite(state, triggerMsg);
            }
        }
        break;
    case SET_ALL:
        triggerMsg->size = setTriggerSettings(triggerMsg->msg, ct->freq, ct->pulseWidthUs, ct->strobeDelayUs);
        triggerWrite(state, triggerMsg);
        break;
    }



    free(triggerMsg);

}



int progExit;
void signalHandler(int sigNum)
{
    // do a safe exit
    if(sigNum == SIGINT)
        progExit = 1;
    if(sigNum == SIGPIPE)
    {
        printf("Broken pipe\n");
        fflush(NULL);
    }
}

static void *listenThread (void *u)
{
    state_t *state = (state_t *)u;
    fd_set read_fds;
    int len;
    int i,j;
    char buffer[32], temp_str[32];
    float temp;

    while(!progExit)
    {
        FD_ZERO(&read_fds);
        FD_SET(state->triggerFd, &read_fds);

        if(select(state->triggerFd+1, &read_fds, NULL, NULL, NULL) > 0)
        {
            len = recv(state->triggerFd, buffer, sizeof(buffer), 0);
            if((buffer[0] == STX) && (buffer[1] == TRIGGER_TEMP))
            {
                while(buffer[1 + i++] != ETX && len--)
                    temp_str[j++] = buffer[1+i];
                temp = atof(temp_str);
            }
            acfrlcm_auv_camera_temp_t ct;
            ct.utime = timestamp_now();
            ct.temperature = temp;
            acfrlcm_auv_camera_temp_t_publish(state->lcm, "CAMERA_TEMP", &ct);
        }
    }
}

int main(int argc, char **argv)
{
    state_t state;
    state.enabled = 0;
    state.keepAliveCount = 0;

    // install the signal handler
    progExit = 0;
    signal(SIGINT, signalHandler);
    signal(SIGPIPE, signalHandler);

    // start lcm
    state.lcm = lcm_create(NULL);


    BotParam *cfg;
    char rootkey[64];
    cfg = bot_param_new_from_server (state.lcm, 1);
    sprintf (rootkey, "acfr.%s", basename (argv[0]));
    char key[128];

    char *triggerPort;
    sprintf(key, "%s.port", rootkey);
    if(bot_param_get_str(cfg, key, &triggerPort) == -1)
    {
        printf("No port variable found\n");
        return 1;
    }

    char *triggerAddress;
    sprintf(key, "%s.IP", rootkey);
    if(bot_param_get_str(cfg, key, &triggerAddress) == -1)
    {
        printf("No IP variable found\n");
        return 1;
    }

    int pulseWidthUs;
    sprintf(key, "%s.pulseWidthUs", rootkey);
    if(bot_param_get_int(cfg, key, &pulseWidthUs) == -1)
    {
        printf("No pulseWidthUs variable found\n");
        return 1;
    }

    int strobeDelayUs;
    sprintf(key, "%s.strobeDelayUs", rootkey);
    if(bot_param_get_int(cfg, key, &strobeDelayUs) == -1)
    {
        printf("No strobeDelayUs variable found\n");
        return 1;
    }

    double freq;
    sprintf(key, "%s.frequency", rootkey);
    if(bot_param_get_double(cfg, key, &freq) == -1)
    {
        printf("No frequency variable found\n");
        return 1;
    }

    // open the port to the sonar head
    struct addrinfo hints, *triggerAddr;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(triggerAddress, triggerPort, &hints, &triggerAddr);

    state.triggerFd = socket(triggerAddr->ai_family, triggerAddr->ai_socktype, triggerAddr->ai_protocol);
    fcntl(state.triggerFd, F_SETFL, O_NONBLOCK);
    connect(state.triggerFd, triggerAddr->ai_addr, triggerAddr->ai_addrlen);

    fd_set fdset;
    struct timeval tv;

    FD_ZERO(&fdset);
    FD_SET(state.triggerFd, &fdset);
    tv.tv_sec = 5;             /* 10 second timeout */
    tv.tv_usec = 0;
    if (select(state.triggerFd + 1, NULL, &fdset, NULL, &tv) < 1)
    {
        fprintf(stderr, "Trigger connect timeout\n");
        return 1;
    }

    state.mutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init(state.mutex, NULL);

    // send the initial settings
    triggerMsg_t *triggerMsg = g_malloc(sizeof(triggerMsg_t));
    triggerMsg->size = setTriggerSettings(triggerMsg->msg, freq, pulseWidthUs, strobeDelayUs);
    triggerWrite(&state, triggerMsg);
    free(triggerMsg);

    // subscribe to the relevant LCM messages
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartBeatHandler, &state);
    acfrlcm_auv_camera_trigger_t_subscribe(state.lcm, "CAMERA_TRIGGER", &cameraTriggerHandler, &state);

    // create socket read thread
    pthread_t tid;
    pthread_create(&tid, NULL, listenThread, &state);
    pthread_detach(tid);

    // process
    while (!progExit)
    {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        lcmu_handle_timeout(state.lcm, (struct timeval *)&tv);
    }

    // exit cleanly
    pthread_mutex_destroy(state.mutex);
    lcm_destroy(state.lcm);
    close(state.triggerFd);
    pthread_join(tid, NULL);

    return 0;
}

