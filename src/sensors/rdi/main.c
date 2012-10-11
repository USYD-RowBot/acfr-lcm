#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <lcm/lcm.h>

#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd0_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include "perls-lcmtypes/acfrlcm_auv_relay_t.h"

#include "perls-common/units.h"
#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/generic_sensor_driver.h"

#include "rdi.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RDI_DEV_TICKS_PER_SECOND (100) //  1/tofp_hundredth
#define RDI_DEV_TICKS_WRAPAROUND (24*3600*100)

#define EXPECT_RESPONSE 1
#define NO_RESPONSE 0

enum {MODE_PD0, MODE_PD5, MODE_PD4};

typedef struct 
{
    int pd5_count_max;
    int pd0_count_max;
    pthread_mutex_t count_lock;
    int mode;
    generic_sensor_driver_t *gsd;
    int programming;
} state_t;

rdi_pd_mode_t rdi_pd_mode = RDI_PD5_MODE;
int rdi_pd_len = RDI_PD5_LEN;

static int64_t
rdi_timestamp_sync (timestamp_sync_state_t *tss, int64_t tofp_hours, int64_t tofp_minute,
                    int64_t tofp_second, int64_t tofp_hundredth, int64_t host_utime)
{
    int64_t dev_ticks = (tofp_hours*3600 + tofp_minute*60 + tofp_second)*100 + tofp_hundredth;
    
    return timestamp_sync (tss, dev_ticks, host_utime);
}

int rdi_send_command(generic_sensor_driver_t *gsd, char *cmd, int er)
{
    gsd_write(gsd, cmd, strlen(cmd));
    char buf;
    int count = 0;
    
    if(er)
    {
        // wait to get a prompt
        do
        {
            gsd_read(gsd, &buf, 1, NULL);
            if(++count == 50)
                return 0;                   
        } while(buf != '>');
    }
    return 1;
}

static void
program_dvl(generic_sensor_driver_t *gsd, int mode) //const char *config)
{
    if (gsd->io == GSD_IO_PLAYBACK)
        return;
    
    char buf[256];
    bool alive=0;
    
    tcsendbreak(gsd->fd, 0);
    usleep(2000000);
    
    gsd_canonical(gsd, '\r', '\n');
    
    int alive_tries = 0;
    while (!alive) {
        tcsendbreak(gsd->fd, 0);
        //gsd_write (gsd, "===\n", strlen ("===\n")); // software break
        alive = gsd_read_timeout (gsd, buf, 256, NULL, 10000);
        printf("*\n");
        if(++alive_tries > 20)
        {
            printf("Couldn't wake the DVL up\n");
            return;
        }
    }

    
    // wait to get a prompt
    do
    {
        gsd_read(gsd, buf, 1, NULL);
        printf("%c", buf[0]);
                           
    } while(buf[0] != '>');
    
    if(mode == MODE_PD5)
    {
        printf("Programming PD5 mode\n");
        rdi_send_command(gsd, "BP1\r", EXPECT_RESPONSE);
        rdi_send_command(gsd, "WP0\r", EXPECT_RESPONSE);
        rdi_send_command(gsd, "PD5\r", EXPECT_RESPONSE);   
        rdi_send_command(gsd, "CF11110\r", EXPECT_RESPONSE);
        rdi_send_command(gsd, "CS\r", NO_RESPONSE);
    }
    else if(mode == MODE_PD4)
    {
        printf("Programming PD4 mode\n");
        rdi_send_command(gsd, "BP1\r", EXPECT_RESPONSE);
        rdi_send_command(gsd, "WP0\r", EXPECT_RESPONSE);
        rdi_send_command(gsd, "PD4\r", EXPECT_RESPONSE);   
        rdi_send_command(gsd, "CF11110\r", EXPECT_RESPONSE);
        rdi_send_command(gsd, "CS\r", NO_RESPONSE);
    }
    else
    {
        printf("Programming PD0 mode\n");
        rdi_send_command(gsd, "WD 100 000 000\r", EXPECT_RESPONSE);
        rdi_send_command(gsd, "CF01110\r", EXPECT_RESPONSE); 
    }
    fflush(NULL);   
}


static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "RDI Workhorse & Explorer DVL sensor driver.");
    return 0;
}


            
      

int get_rdi_and_send(generic_sensor_driver_t *gsd, timestamp_sync_state_t *tss)
{
    // get an RDI response
    char buf[1024];
    int len;
    int64_t timestamp;
    
    // get the start
    do 
    {
        gsd_read(gsd, buf, 1, &timestamp);
    } while (buf[0] != 0x7F && buf[0] != 0x7D);
    
    len = 0; 
    while(len < 3)
    	len += gsd_read(gsd, &buf[1+len], 3 - len, NULL);
    
    // find the length of the data to recv
    unsigned short data_len = *(unsigned short *)&buf[2];
    // get the rest
    len = 4;
    while(len < (data_len + 2))
        len += gsd_read(gsd, &buf[len], data_len + 2 - len, NULL);

     
    if(buf[0] == RDI_PD0_HEADER)
    {   rdi_pd0_t pd0;
        // we have a PD0 message
        if(rdi_parse_pd0(buf, len, &pd0))
        {
            senlcm_rdi_pd0_t lcm_pd0 = rdi_pd0_to_lcm_pd0(&pd0);
            lcm_pd0.utime = rdi_timestamp_sync (tss, pd0.variable.rtc_hour, pd0.variable.rtc_min, pd0.variable.rtc_sec, 
                                                        pd0.variable.rtc_hund, timestamp);
            senlcm_rdi_pd0_t_publish (gsd->lcm, "RDI_PD0", &lcm_pd0); 
            //gsd_update_stats (gsd, true);   
            free_rdi_pd0(&pd0);
        }
        else
            gsd_update_stats (gsd, false);

    }
    else if(buf[0] == RDI_PD45_HEADER)
    {
        // try to parse it
        switch (rdi_pd_mode) {   
        case RDI_PD4_MODE: {
            rdi_pd4_t pd4;
            if (0 == rdi_parse_pd4 (buf, len, &pd4)) {
                senlcm_rdi_pd4_t lcm_pd4 = rdi_pd4_to_lcm_pd4 (&pd4);
                lcm_pd4.utime = rdi_timestamp_sync (tss, pd4.tofp_hour, pd4.tofp_minute, pd4.tofp_second, 
                                                    pd4.tofp_hundredth, timestamp);
                senlcm_rdi_pd4_t_publish (gsd->lcm, gsd->channel, &lcm_pd4);
                gsd_update_stats (gsd, true);
            }
            else
            {
                gsd_update_stats (gsd, false);
                printf("parse error, PD4\n");
            }
            break;
        }
        case RDI_PD5_MODE: {
            rdi_pd5_t pd5;
            if (0 == rdi_parse_pd5 (buf, len, &pd5)) {
                senlcm_rdi_pd5_t lcm_pd5 = rdi_pd5_to_lcm_pd5 (&pd5);
                lcm_pd5.utime = lcm_pd5.pd4.utime = timestamp; //rdi_timestamp_sync (tss, pd5.tofp_hour, pd5.tofp_minute, 
                                                                 //       pd5.tofp_second, pd5.tofp_hundredth, timestamp);
                senlcm_rdi_pd5_t_publish (gsd->lcm, gsd->channel, &lcm_pd5);
                gsd_update_stats (gsd, true);
            }
            else
            {
                gsd_update_stats (gsd, false);
                printf("parse error, PD5\n");
            }

            break;
        }
        default:
            ERROR ("unsupported PD mode by driver");
            gsd_update_stats (gsd, -1);
        } // switch
    }
    
    return 1;
}

void mp_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_raw_ascii_t *msg, void *u)
{
    state_t *state = (state_t *)u;
    int i_value;
    char *ptr;
    int ret;
    
    if(strstr(msg->msg, "RDI") != NULL)
    {
        // the message is for us
        pthread_mutex_lock(&state->count_lock);
        if((ptr = strstr(msg->msg, "pd0count=")) != NULL)
        {
            ptr += strlen("pd0count=");
            ret = sscanf(ptr, "%d", &i_value);
            if(ret == 1)
                state->pd0_count_max = i_value;
        }
        
        if((ptr = strstr(msg->msg, "pd5count=")) != NULL)
        {
            ptr += strlen("pd5count=");
            ret = sscanf(ptr, "%d", &i_value);
            if(ret == 1)
                state->pd5_count_max = i_value;
        }
        pthread_mutex_unlock(&state->count_lock);
    }
}


void relay_callback(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_relay_t *msg, void *u)
{
    state_t *state = (state_t *)u;
    
    if(!strcmp(msg->channel, "doppler"))
        if(msg->state == 1)
        {
            if(!state->programming)
            {
                state->programming = 1;
                printf("Reprogramming the DVL\n");
                sleep(2);
                program_dvl(state->gsd, state->mode);
                state->programming = 0;
            }    
        }
    
}
// Process LCM messages with callbacks
static void *
lcm_thread (void *context)
{
    generic_sensor_driver_t *gsd = (generic_sensor_driver_t *) context;
    while (!gsd->done) {
       struct timeval tv;
	    tv.tv_sec = 1;
	    tv.tv_usec = 0;

        lcmu_handle_timeout(gsd->lcm, &tv);
    }
    
    return 0;
}


int
main (int argc, char *argv[])
{
    timestamp_sync_state_t *tss = timestamp_sync_init (RDI_DEV_TICKS_PER_SECOND, RDI_DEV_TICKS_WRAPAROUND, 1.01);

    state_t state;
    state.gsd = gsd_create (argc, argv, NULL, &myopts);
    gsd_launch (state.gsd);
    pthread_mutex_init(&state.count_lock, NULL);
    int pd5_count = 0, pd0_count = 0;

    
    // listen for changes
    state.pd0_count_max = 0;
    state.pd5_count_max = 0;
    
    
    char key[256];
    sprintf(key, "%s.mode", state.gsd->rootkey);    
    char *key_str = bot_param_get_str_or_fail(state.gsd->params, key);
    if(!strcmp(key_str, "PD0"))
        state.mode = MODE_PD0;
    else if(!strcmp(key_str, "PD5")) 
    {
        rdi_pd_mode = RDI_PD5_MODE;
        state.mode = MODE_PD5;
	rdi_pd_len = RDI_PD5_LEN;
    }
    else if(!strcmp(key_str, "PD4"))
    {
        rdi_pd_mode = RDI_PD4_MODE;
        state.mode = MODE_PD4;
	rdi_pd_len = RDI_PD4_LEN;
    }
    else
    {
        printf("Unknown mode %s in config file\n", key_str);
        return 0;
    }
    sprintf(key, "%s.pd5_count_max", state.gsd->rootkey);
    state.pd5_count_max = bot_param_get_int_or_fail(state.gsd->params, key);
    sprintf(key, "%s.pd0_count_max", state.gsd->rootkey);
    state.pd0_count_max = bot_param_get_int_or_fail(state.gsd->params, key);
    

    // initialize dvl
    gsd_flush (state.gsd);
    gsd_reset_stats (state.gsd);
    program_dvl(state.gsd, state.mode);

    gsd_noncanonical(state.gsd, 1024, 1);

    gsd_flush (state.gsd);
    gsd_reset_stats (state.gsd);
    
    // create an LCM thread to listen so the command pass through will work
    pthread_t tid;
    pthread_create(&tid, NULL, lcm_thread, state.gsd);
    pthread_detach(tid);	

    senlcm_raw_ascii_t_subscribe(state.gsd->lcm, "MP_PASSOUT", &mp_callback, &state);
    acfrlcm_auv_relay_t_subscribe(state.gsd->lcm, "RELAY", &relay_callback, &state);


    while (!state.gsd->done) {
        if(state.mode == MODE_PD0)
        {
            // Are we doing PD5 now?
            pthread_mutex_lock(&state.count_lock);
	        if(pd5_count++ < state.pd5_count_max)
	        {
                //Is this the first bottom ping in this sequence?
	            if(pd5_count == 1)
	            {
	                //Turn off water pings and on bottom pings
	                rdi_send_command(state.gsd, "BP1\r", EXPECT_RESPONSE);
	                rdi_send_command(state.gsd, "WP0\r", EXPECT_RESPONSE);
	                rdi_send_command(state.gsd, "PD5\r", EXPECT_RESPONSE);
	            }

	            rdi_send_command(state.gsd,"CS\r", NO_RESPONSE); //trigger the ping
	            get_rdi_and_send(state.gsd, tss);
            }
            else if(pd0_count++ < state.pd0_count_max)
            {

	            if(pd0_count == 1)
	            {
	                //Turn on water pings and off bottom pings
	                rdi_send_command(state.gsd,"BP0\r", EXPECT_RESPONSE);
	                rdi_send_command(state.gsd,"WP1\r", EXPECT_RESPONSE);
	                rdi_send_command(state.gsd,"PD0\r", EXPECT_RESPONSE);
	            }
         
	            rdi_send_command(state.gsd,"CS\r", NO_RESPONSE); //trigger the ping
	            get_rdi_and_send(state.gsd, tss);
            }
	        else
	        {
          	  pd0_count=0;
	          pd5_count=0;
	        }
            pthread_mutex_unlock(&state.count_lock);   
        }
        else
            // we are in free running mode, just PD5 or PD4
            get_rdi_and_send(state.gsd, tss);
    } // while

    timestamp_sync_free (tss);
    pthread_join(tid, NULL);
}
