/*
 *  Evologics MODEM/USBL parsers and message routines
 *
 *  Christan Lees
 *  ACFR
 *  10/2/14
 */

#include "evologics.h"

#define MAX_OUT_SIZE 4096

int readline(int fd, char *buf, int max_len)
{
    int i=0;
    do
    {
        if(recv(fd, &buf[i++], 1, 0) == -1)
            break;
    } while((buf[i-1] != '\r'));
    return i;
}

int chop_string(char *data, char **tokens)
{
    char *token;
    int i = 0;
    
    token = strtok(data, " ,:");
    while(token != NULL) 
    {
        tokens[i++] = token;            
        token = strtok(NULL, " ,:");            
    }
    return i;        
}

int wait_response(state_t *state, char *data, int size)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(state->fd, &rfds);
    
    struct timeval tv;
	tv.tv_sec = 5;
	tv.tv_usec = 0;
	    
    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
    if(ret == -1) 
    {
        perror("Select failure: ");
        return 0;
    }
    else if(ret != 0)
    {
        char buf[32];
        // get the data and check for an OK
        if(state->io == io_socket)                    
            readline(state->fd, data, size);
        else
            read(state->fd, data, size);      
        if(strstr(buf, "OK") != NULL)
            return 1;
        else
            return 0;
    }
    return 0;
}
        
// Parse a USBLONG message and publish the LCM message
int parse_usbllong(char *data, state_t *state, int64_t timestamp)
{
    char *tokens[19];
    int ret;
    ret = chop_string(data, tokens);
    
    if(ret != 19)
        return 0;
        
    senlcm_evologics_usbl_t ud;
    ud.utime = timestamp;
    ud.remote_id = atoi(tokens[5]);
    ud.x = atof(tokens[6]);
    ud.y = atof(tokens[7]);
    ud.z = atof(tokens[8]);
    ud.e = atof(tokens[9]);
    ud.n = atof(tokens[10]);
    ud.u = atof(tokens[11]);
    ud.r = atof(tokens[12]);
    ud.p = atof(tokens[13]);
    ud.h = atof(tokens[14]);
    ud.prop_time = atof(tokens[15]);
    ud.rssi = atoi(tokens[16]);
    ud.integrity = atoi(tokens[17]);
    ud.accuracy = atof(tokens[18]);
    
    senlcm_evologics_usbl_t_publish(state->lcm, "EVOLOGICS_USBL", &ud);
    
    return 1;
}

// Parse an instant message, not really going to do anything with them at this point
int parse_im(char *data, state_t *state)
{
    char *tokens[12];
    int ret;
    ret = chop_string(data, tokens);
    if(ret > 1)
        printf("Got IM: %s\n", data);
//    if(ret != 12)
        return 0;    
}    
    

// Parse a message from an Evologics modem or usbl
int parse_evologics_message(char *data, int size, state_t *state, int64_t timestamp)
{
    // Since we are in data mode we need to see if it starts with a +++AT
    // which indicates if it is control message or data
    
    if(strstr((const char *)data, "+++AT") != NULL)
    {
        // parse the message type
        if(strstr((const char *)data, "USBLLONG") != NULL)
            parse_usbllong(data, state, timestamp);
        else if(strstr((const char *)data, "RECVIM") != NULL)
            parse_im(data, state);
        else
            printf("Unknown modem control message: %s\n", data);
    }
    else
    {
        // must be normal data, decode, parse and publish
        printf("Got a data message: %s\n", data);
        struct basE91 b91;
        int out_size;
        basE91_init(&b91);
        char *out = malloc(MAX_OUT_SIZE);
        memset(out, 0, MAX_OUT_SIZE);
        out_size = basE91_decode(&b91, data, size, out);
        out_size += basE91_decode_end(&b91, &out[out_size]);
        
        if(out_size < 1)
        {
            fprintf(stderr, "Failed to decode message\n");
            return 0;
        }
        
        // reconstruct the message based on its type and publish it
        if(out[0] == LCM_AUV_STATUS)
        {
            acfrlcm_auv_status_t s;
            char channel[16];
            memset(channel, 0, 16);
            strncpy(channel, &out[2], out[1]);
            memcpy(&s, &out[out[1] + 2], sizeof(acfrlcm_auv_status_t));
            acfrlcm_auv_status_t_publish(state->lcm, channel, &s);
        }
        
        free(out);
    }
    return 1;
}


int send_evologics_data(char *data, int size, int target, state_t *state)
{
    // first if the target has changed we need to send a modem command to change the destination target
    if(target != state->current_target)
    {
        char buf[16];
        sprintf(buf, "+++ATZ4\n\r");
        if(write(state->fd, buf, strlen(buf)) == -1)
            fprintf(stderr, "Faild to write to modem\n");
        wait_response(state, NULL, 1024);
        
        sprintf(buf, "+++AT!AR%d\n\r", target);
        if(write(state->fd, buf, strlen(buf)) == -1)
            fprintf(stderr, "Faild to write to modem\n");
        
        if(!wait_response(state, NULL, 1024))
        {    
            fprintf(stderr, "Could not change target address\n");
//            return 0;
        }
        state->current_target = target;
    }
    
    
    struct basE91 b91;
    basE91_init(&b91);
    char *out = malloc(MAX_OUT_SIZE);
    memset(out, 0, MAX_OUT_SIZE);
    int out_size;
    out_size = basE91_encode(&b91, data, size, out);
    out_size += basE91_encode_end(&b91, &out[out_size]);
    strcat(out, "\n\r");
    printf("Sending : %s", out);
    if(write(state->fd, out, out_size) == -1)
    {
        fprintf(stderr, "Could not write data to modem\n");
        return 0;
    }
    
    return 1; 
}    
 
// send a ping, used for tracking when we don't need to send any data    
int send_ping(int target, state_t *state)
{    
    char msg[32];
    memset(msg, 0, 32);
    sprintf(msg, "+++AT*SENDIM,1,%d,ack,1\n\r", target);
    if(write(state->fd, msg, strlen(msg)) == -1)
    {
        fprintf(stderr, "Failed to send ping to target %d\n", target);
        return 0;
    }
    return 1;
}
    


