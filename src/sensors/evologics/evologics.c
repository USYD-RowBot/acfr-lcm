/*
 *  Evologics MODEM/USBL parsers and message routines
 *
 *  Christan Lees
 *  ACFR
 *  10/2/14
 */

#include "evologics.h"

#ifdef EVO_ETHER
#define TERM "\n"
#else
#define TERM "\r"
#endif

#define MAX_OUT_SIZE 4096
#define SELECT_TIMEOUT 5

int readline(int fd, char *buf, int max_len)
{
    int i=0;
    do
    {
        if(recv(fd, &buf[i++], 1, 0) == -1)
            break;
    } while((buf[i-1] != 0x0A));
    return i;
}

/*
int readline(int fd, char *buf, int max_len)
{
    int i=0;
    int read = 0;
    do
    {
        printf("i=%d\n", i);
        int r = recv(fd, &buf[i++], max_len, 0);
        if(r == -1)
            break;
        read += r;
        
    } while((buf[read] != '\n') || ((buf[read] != '\r') && (buf[read - 1] != '\n')));
    return i;
}
*/


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



int wait_response(el_state_t *state, char *data, int size)
{
    int ret;
    fd_set rfds;
	int timeout = 0;
	
	while(timeout < SELECT_TIMEOUT)
	{    
	    FD_ZERO(&rfds);
        FD_SET(state->fd, &rfds);
        
        struct timeval tv;
	    tv.tv_sec = 1;
	    tv.tv_usec = 0;
	    
        ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret != 0)
        {
            // get the data and check for an OK
            if(state->interface == IO_ENET)
                ret = readline(state->fd, data, size);
            else
                ret = read(state->fd, data, size);
            return ret;
        }
        timeout++;
    }
    printf("Timed out\n");
    return ret;
}




        
// Parse a USBLONG message and publish the LCM message
parse_message_t parse_usbllong(char *data,  el_state_t *state, int64_t timestamp, int *target)
{
    //printf("parsing usbl\n");
    char *tokens[19];
    int ret;
    ret = chop_string(data, tokens);
    
    if(ret != 19)
        return  PARSE_ERROR;
        
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
    if(state->usbl != NULL)
        memcpy(state->usbl, &ud, sizeof(senlcm_evologics_usbl_t));
    
    int i;
    for(i=0; i<state->num_targets; i++)
        if(state->targets[i] == ud.remote_id)
            break;
    *target = i;
    
    return PARSE_USBL;
}
    
// Parse a USBLONG message and publish the LCM message
parse_message_t parse_usblangles(char *data,  el_state_t *state, int64_t timestamp, int *target)
{
    //printf("parsing usbl\n");
    char *tokens[19];
    int ret;
    ret = chop_string(data, tokens);
    
    if(ret != 19)
        return  PARSE_ERROR;
        
    // for now we just need to get the target id
    int target_id = atoi(tokens[5]);
    int i;
    for(i=0; i<state->num_targets; i++)
        if(state->targets[i] == target_id)
            break;
    *target = i;
    
    return PARSE_NONE;
}    
        
        

// Parse an instant message, not really going to do anything with them at this point
parse_message_t parse_im(char *data,  el_state_t *state)
{
//    char *tokens[12];
//    int ret;
//    ret = chop_string(data, tokens);
//    if(ret > 1)
//        printf("Got IM: %s\n", data);
//    if(ret != 12)
        return PARSE_IM;    
}

int parse_im_response(char *data, el_state_t *state)
{
    // return the channel number from an IM response message
    // either a DELIVEREDIM or FAILEDIM
    char *token = strtok(data, ",");
    token = strtok(NULL, ",");
    int target = atoi(token);
    int i;
    for(i=0; i<state->num_targets; i++)
        if(state->targets[i] == target)
            break;
    //printf("reseting channel index %d\n", i);       
    return i;
}
    
    
    

// Parse a message from an Evologics modem or usbl
parse_message_t parse_evologics_message(char *data, int size, el_state_t *state, int64_t timestamp)
{
    // Since we are in data mode we need to see if it starts with a +++AT
    // which indicates if it is control message or data
    parse_message_t ret;
    if(strstr((const char *)data, "+++AT") != NULL)
    {
        printf("Got a command message\n");
        if(strstr((const char *)data, "USBLLONG") != NULL)
        {
            int target;
            ret = parse_usbllong(data, state, timestamp, &target);
            //state->ping_semaphore[target] = 0;
            //state->channel_ready = 1;
        }
        else if(strstr((const char *)data, "USBLANGLES") != NULL)
        {
            int target;
            ret = parse_usblangles(data, state, timestamp, &target);
            //state->ping_semaphore[target] = 0;
            //state->channel_ready = 1;
        }    
        else if(strstr((const char *)data, "RECVIM") != NULL)
            ret = parse_im(data, state);
        else if((strstr((const char *)data, "FAILEDIM") != NULL) ||
            (strstr((const char *)data, "DELIVEREDIM") != NULL) ||
            (strstr((const char *)data, "CANCELEDIM") != NULL))
        {
            printf("reseting channel state to ready\n");
            //state->ping_semaphore[parse_im_response(data, state)] = 0;
            //state->channel_ready = 1;
            state->modem_state = MODEM_AVAILABLE;
            ret = PARSE_NONE;
        }    
        else if(strstr((const char *)data, "INITIATION LISTEN") != NULL)
        {
            //state->ping_semaphore[parse_im_response(data, state)] = 0;
            //state->channel_ready = 1;
            //state->sending_data = 0;
            state->modem_state = MODEM_AVAILABLE;
            ret = PARSE_NONE;
        }

        
        else
        {
            printf("Unknown modem control message: %s\n", data);
            ret = PARSE_ERROR;
        }
    }
    else
    {
        //printf("Got data: %s\n", data);
        // check for the preamble
        if(strstr((const char *)data, "EVDATA") != NULL)
        {
            //printf("Preamble correct, data length %d\n", size);
            // must be normal data, decode, parse and publish
            struct basE91 b91;
            int out_size;
            basE91_init(&b91);
            char *out = malloc(MAX_OUT_SIZE);
            memset(out, 0, MAX_OUT_SIZE);
            printf("string length = %d\n", (int)strlen(data));
            out_size = basE91_decode(&b91, &data[6], size-6, out);
            if(out_size != 0)
                out_size += basE91_decode_end(&b91, &out[out_size]);
            else
                printf("decode failure\n");
            
            //printf("decoded length = %d\n", out_size);
            //for(int i=0; i<out_size; i++)
            //    printf("%02X ", out[i] & 0xFF);
            //printf("\n");    

            
            if(out_size < 1)
            {
                fprintf(stderr, "Failed to decode message\n");
                ret =  PARSE_ERROR;
            }
            else
            {
                // check the CRC
                unsigned long crc = crc32(0, (const void *)out, out_size - 4);
                unsigned long d_crc = *(unsigned long *)&out[out_size - 4];
                if(crc != d_crc)
                {
                    fprintf(stderr, "CRC error\n");
                    ret = PARSE_ERROR;
                }
                else
                {
                    // reconstruct the message based on its type and publish it
                    if(out[0] == LCM_AUV_STATUS)
                    {
                        lcm_publish(state->lcm, "AUV_HEALTH", &out[1], out_size - 5);
                        ret = PARSE_DATA;
                    }
                    else if(out[0] == LCM_USBL_FIX)
                    {
                        lcm_publish(state->lcm, "USBL_FIX_AUV", &out[1], out_size - 5);
                        ret = PARSE_DATA;
                    }    
                    else if(out[0] == LCM_TASK_COMMAND)
                    {
                        lcm_publish(state->lcm, "TASK_PLANNER_COMMAND", &out[1], out_size - 5);
                        ret = PARSE_DATA;
                    }
                    else
                        ret = PARSE_NONE;
                }
            }    
            free(out);
        }
    }
    return ret;
}


int send_evologics_data(char *data, int size, int target, el_state_t *state)
{
    // if the channel is still busy just return
    int count = 0;
    while(state->modem_state != MODEM_AVAILABLE && count < 5)
    {
        usleep(500e6);
        count++;
    }
    if(state->modem_state != MODEM_AVAILABLE)
        return -2;        

    char msg[16];
    char ret_msg[256];
    // first if the target has changed we need to send a modem command to change the destination target
    if(target != state->current_target)
    {
        
        sprintf(msg, "+++ATZ4%s", TERM);
        send_evologics_command(msg, NULL, 256, state);
        
        int retry = 0;
        do
        {
            sprintf(msg, "+++AT!AR%d%s", state->targets[target], TERM);
            send_evologics_command(msg, ret_msg, 256, state);
            retry++;
        }
        while((strstr((const char *)ret_msg, "OK") == NULL) && retry < 5);

        // check to see if the target actually changed
        sprintf(msg, "+++AT?AR%s", TERM);
        send_evologics_command(msg, ret_msg, 256, state);
        char *tokens[4];
        int ret;
        ret = chop_string(ret_msg, tokens);
        if(atoi(tokens[1]) == target)
        {
            fprintf(stderr, "Changing target to %d\n", target);
            state->current_target = target;
        }
        else
        {
            fprintf(stderr, "Failed to change target\n");
            return 0;
        }
    }
    
    // calculate the CRC
    unsigned long crc = crc32(0, (const void *)data, size);
    // stick it on the end
    char *d = malloc(size + 8);
    strcpy(d, "LD");
    memcpy(&d[2], data, size);
    memcpy(&d[size + 2], &crc, 4);  
    strcpy(&d[size + 6], "LE");
    
    
    
    /*
    struct basE91 b91;
    basE91_init(&b91);
    char *out = malloc(MAX_OUT_SIZE);
    char *msg = malloc(MAX_OUT_SIZE);
    memset(msg, 0, MAX_OUT_SIZE);
    memset(out, 0, MAX_OUT_SIZE);
    int out_size;
    out_size = basE91_encode(&b91, d, (size + 4), msg);
    out_size += basE91_encode_end(&b91, &msg[out_size]);
    strcat(msg, "LE");
    sprintf(out, "LD");
    strcat(out, msg);
    */
    printf("Sending : %s\n", d);
    if(write(state->fd, d, size+8) == -1)
    {
        fprintf(stderr, "Could not write data to modem\n");
        return 0;
    }
    
    state->modem_state = MODEM_SENDING_DATA;
    
    free(d);

//    free(out);
//   free(msg);

    // Check to see that the data went through
 //   do 
 //   {
 //       send_evologics_command("+++AT?ZE\n", buf, 64, state);
        
 //   } 
    
    return 1; 
}    
 
int send_evologics_command(char *data, char *ret_str, int size,  el_state_t *state)
{
    int count = 0;
    while(state->modem_state != MODEM_AVAILABLE && count < 5)
    {
        usleep(500e6);
        count++;
    }
    if(state->modem_state != MODEM_AVAILABLE)
        return 0;
        
    int free_buf = 0;
    if(ret_str == NULL)
    {
        ret_str = malloc(256);
        free_buf = 1;
        memset(ret_str, 0, 256);
    }
        
    printf("sending: %s", data);
    int bytes = write(state->fd, data, strlen(data));
    if( bytes == -1)
    {
        fprintf(stderr, "Failed to send data to modem\n");
        return 0;
    }
    
    state->modem_state = MODEM_SENDING_COMMAND;
/*    
    state->channel_ready = 0;
    // we are always expecting a return
    int ret = 0;
    while(ret < 1)
        ret = wait_response(state, ret_str, size);    

    printf("Modem response: %s", ret_str);
    
    if(free_buf)
        free(ret_str);

    return ret;
*/
}


 
// send a ping, used for tracking when we don't need to send any data    
int send_ping(int target,  el_state_t *state)
{   
    char msg[128];
    char ret_msg[256];
    
    if(state->modem_state != MODEM_AVAILABLE)
        return 0;
    
    if(state->ping_semaphore[target] !=0 )
    {   
        printf("Target %d busy\n", target);
        return 0;
    }
    
    memset(msg, 0, 32);
    memset(ret_msg, 0, 256);
    sprintf(msg, "+++AT*SENDIM,1,%d,ack,%d%s", state->targets[target], state->targets[target], TERM);
    printf("Sending message %s", msg);
    
    int bytes = write(state->fd, msg, strlen(msg));
    if( bytes == -1)
    {
        fprintf(stderr, "Failed to send data to modem\n");
        return 0;
    }
    
    state->modem_state = MODEM_SENDING_IM;
    
    return 1;
}
/*


    // check to see that the ping went through ok
    if(strstr(ret_msg, "OK") != NULL)
    {
        //fprintf(stderr, "ping success to target %d(%d)\n", state->targets[target], target);
        state->ping_semaphore[target] = 1;
    }
    else if(strstr(ret_msg, "CANCEL") != NULL)
    {
        state->ping_semaphore[target] = 0;
    }    
    else if(strstr(ret_msg, "DELIVEREDIM") != NULL)
    {
        state->ping_semaphore[target] = 0;
    }    
    else    
    {
        fprintf(stderr, "ping fail to target %d(%d)\n", state->targets[target], target);
        state->ping_semaphore[target] = 0;
    }
        
    return 1;
}
*/    


