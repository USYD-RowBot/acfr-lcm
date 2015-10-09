/*
    Listens to the motor command message and sends it onto the motors.
    A failsafe is implemented using the 1Hz heart beat

    Christian Lees
    ACFR
    13/9/11
*/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <sys/select.h>

#include "perls-common/serial.h"
#include "perls-common/timestamp.h"
#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_status_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/acfrlcm_auv_relay_t.h"

#ifndef BOT_CONF_DIR
#define DEFAULT_BOT_CONF_PATH "../config/master.cfg"
#else
#define DEFAULT_BOT_CONF_PATH BOT_CONF_DIR "/master.cfg"
#endif

#define MAX_MESSAGE_SIZE 128
#define MOTOR_TIMEOUT 10000000  // this is in microseconds

#define ANIMATICS_PRINT_STRING "PRINT(V,\" \",UIA,\" \",UJA,\" \",TEMP,#13,#10)\n"

typedef struct
{
    // serial port stuff
    int vert_fd;
    int port_fd;
    int starb_fd;
    pthread_mutex_t port_lock;

    // lcm stuff
    lcm_t *lcm;

    int motors_on_vert;
    int motors_on_mains;

    // failsafe
    int64_t last_motor_command;
    pthread_mutex_t time_lock;

} state_t;

typedef struct
{
    state_t *state;
    int fd;
} motor_thread_state_t;


void
send_motor_command(state_t *state, double vertical, double port, double starboard)
{
    // send a command to the motors
    char animatics_str[MAX_MESSAGE_SIZE];
    int thr_ref, len;

    // from animatics user manual p46
    // the 32212 is for the motor, 32.5 is for the gear box, 60 is from RPM to Hz
    // compose animatics command string for velocity mode

    thr_ref = (int)(vertical * 32212.0 * 32.5 / 60.0);
    len = sprintf(animatics_str, "MV A=800 V=%i G t=0\n",thr_ref);
    write(state->vert_fd, animatics_str, len);

    thr_ref = (int)(port * 32212.0 * 32.5 / 60.0);
    len = sprintf(animatics_str, "MV A=800 V=%i G t=0\n",thr_ref);
    write(state->port_fd, animatics_str, len);

    thr_ref = (int)(starboard * 32212.0 * 32.5 / 60.0);
    len = sprintf(animatics_str, "MV A=800 V=%i G t=0\n",thr_ref);
    write(state->starb_fd, animatics_str, len);

    usleep(5000);
    write(state->vert_fd, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));
    write(state->port_fd, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));
    write(state->starb_fd, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));

}

void
motor_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_sirius_motor_command_t *mc, void *u)
{
    state_t *state = (state_t *)u;

    pthread_mutex_lock(&state->time_lock);
    state->last_motor_command = mc->utime;
    pthread_mutex_unlock(&state->time_lock);

    pthread_mutex_lock(&state->port_lock);
    send_motor_command(state, mc->vertical, mc->port, mc->starboard);
    pthread_mutex_unlock(&state->port_lock);
}

void
heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    // this is used to make sure some process up stream doesn't die and leave us in a bad
    // state.  If a timeout occurs then we will shut the motors down
    state_t *state = (state_t *)u;

    pthread_mutex_lock(&state->time_lock);
    int64_t time_diff = hb->utime - state->last_motor_command;
    if(time_diff > MOTOR_TIMEOUT)
    {
        pthread_mutex_lock(&state->port_lock);
        send_motor_command(state, 0, 0, 0);
        pthread_mutex_unlock(&state->port_lock);
    }
    pthread_mutex_unlock(&state->time_lock);
}



void relay_callback(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_relay_t *msg, void *u)
{
    state_t *state = (state_t *)u;
    int send_break = 0;

    if(!strcmp(msg->channel, "mains") || !strcmp(msg->channel, "lat_vert"))
    {
        if(msg->state == 1)
        {
            if(!state->motors_on_mains)
            {
                tcsendbreak(state->port_fd, 2);
                tcsendbreak(state->starb_fd, 2);
                state->motors_on_mains = 1;
                send_break = 1;
            }

            if(!state->motors_on_vert)
            {
                tcsendbreak(state->vert_fd, 2);
                state->motors_on_vert = 1;
                send_break = 1;
            }

            if(send_break)
            {
                printf("Reseting ports\n");
                usleep(5000000);
                tcflush(state->vert_fd, TCIOFLUSH);
                tcflush(state->port_fd, TCIOFLUSH);
                tcflush(state->starb_fd, TCIOFLUSH);
            }


        }
        else if(msg->state == 0)
        {
            if(!strcmp(msg->channel, "mains"))
                state->motors_on_mains = 0;
            if(!strcmp(msg->channel, "lat_vert"))
                state->motors_on_vert = 0;
        }
    }
}


int program_exit;
void
signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}



// listen to the responses from the motors and post the LCM messages
void *motor_listen_thread(void *u)
{
    motor_thread_state_t *ms = (motor_thread_state_t *)u;
    fd_set rfds;
    struct timeval tv;
    int ret;
    char buf[MAX_MESSAGE_SIZE];
    double omega_raw, current_raw, voltage_raw, temp_raw;

    while(!program_exit)
    {

        FD_ZERO(&rfds);
        FD_SET(ms->fd, &rfds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            ret = read(ms->fd, buf, MAX_MESSAGE_SIZE);
            if(ret > 0)
            {
                ret = sscanf(buf,"%lf %lf %lf %lf", &omega_raw, &current_raw, &voltage_raw, &temp_raw);
                if(ret == 4)
                {
                    // form the LCM message and publish it
                    acfrlcm_auv_sirius_motor_status_t status;
                    status.utime = timestamp_now();

                    status.rpm = omega_raw / 32212.0 / 32.5 * 60.0;
                    status.voltage = voltage_raw / 10.0;
                    status.current = current_raw / 100.0;

                    if(ms->fd == ms->state->vert_fd)
                    {
                        status.thruster_num = 0;
                        acfrlcm_auv_sirius_motor_status_t_publish(ms->state->lcm, "THRUSTER_STATUS_VERT", &status);
                    }
                    else if(ms->fd == ms->state->port_fd)
                    {
                        status.thruster_num = 1;
                        acfrlcm_auv_sirius_motor_status_t_publish(ms->state->lcm, "THRUSTER_STATUS_PORT", &status);
                    }
                    else if(ms->fd == ms->state->starb_fd)
                    {
                        status.thruster_num = 2;
                        acfrlcm_auv_sirius_motor_status_t_publish(ms->state->lcm, "THRUSTER_STATUS_STRB", &status);
                    }

                }
            }
        }
    }
    return NULL;
}

int
main(int argc, char **argv)
{
    state_t state;


    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    // read the config file
    BotParam *cfg;
    char rootkey[64];
    char key[64];

    char *path = getenv ("BOT_CONF_PATH");
    if (!path)
        path = DEFAULT_BOT_CONF_PATH;
    cfg = bot_param_new_from_file(path);
    if(cfg == NULL)
    {
        printf("cound not open config file\n");
        return 0;
    }

    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    // vertical thruster info
    char *vert_device;
    sprintf(key, "%s.vert_device", rootkey);
    vert_device = bot_param_get_str_or_fail(cfg, key);

    int vert_baud;
    sprintf(key, "%s.vert_baud", rootkey);
    vert_baud = bot_param_get_int_or_fail(cfg, key);

    char *vert_parity;
    sprintf(key, "%s.vert_parity", rootkey);
    vert_parity = bot_param_get_str_or_fail(cfg, key);

    // port thruster info
    char *port_device;
    sprintf(key, "%s.port_device", rootkey);
    port_device = bot_param_get_str_or_fail(cfg, key);

    int port_baud;
    sprintf(key, "%s.port_baud", rootkey);
    port_baud = bot_param_get_int_or_fail(cfg, key);

    char *port_parity;
    sprintf(key, "%s.port_parity", rootkey);
    port_parity = bot_param_get_str_or_fail(cfg, key);

    // starbord thruster info
    char *starb_device;
    sprintf(key, "%s.starb_device", rootkey);
    starb_device = bot_param_get_str_or_fail(cfg, key);

    int starb_baud;
    sprintf(key, "%s.starb_baud", rootkey);
    starb_baud = bot_param_get_int_or_fail(cfg, key);

    char *starb_parity;
    sprintf(key, "%s.starb_parity", rootkey);
    starb_parity = bot_param_get_str_or_fail(cfg, key);


    // now lets open some ports
    state.vert_fd = serial_open(vert_device, serial_translate_speed(vert_baud), serial_translate_parity(vert_parity), 1);
    if(state.vert_fd < 1)
    {
        fprintf(stderr, "Could not open vert serial port %s\n", vert_device);
        return 1;
    }

    state.port_fd = serial_open(port_device, serial_translate_speed(port_baud), serial_translate_parity(port_parity), 1);
    if(state.port_fd < 1)
    {
        fprintf(stderr, "Could not open port serial port %s\n", port_device);
        return 1;
    }


    state.starb_fd = serial_open(starb_device, serial_translate_speed(starb_baud), serial_translate_parity(starb_parity), 1);
    if(state.starb_fd < 1)
    {
        fprintf(stderr, "Could not open starb serial port %s\n", starb_device);
        return 1;
    }

    state.motors_on_mains = 0;
    state.motors_on_vert = 0;


    // lets start LCM
    state.lcm = lcm_create(NULL);
    state.last_motor_command = timestamp_now();
    acfrlcm_auv_sirius_motor_command_t_subscribe(state.lcm, "SIRIUS_MOTOR", &motor_handler, &state);
//	perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    acfrlcm_auv_relay_t_subscribe(state.lcm, "RELAY", &relay_callback, &state);

    // start the listen threads
    motor_thread_state_t ms_v;
    ms_v.state = &state;
    ms_v.fd = state.vert_fd;
    pthread_t tid_v;
    pthread_create(&tid_v, NULL, motor_listen_thread, &ms_v);
    pthread_detach(tid_v);

    motor_thread_state_t ms_p;
    ms_p.state = &state;
    ms_p.fd = state.port_fd;
    pthread_t tid_p;
    pthread_create(&tid_p, NULL, motor_listen_thread, &ms_p);
    pthread_detach(tid_p);

    motor_thread_state_t ms_s;
    ms_s.state = &state;
    ms_s.fd = state.starb_fd;
    pthread_t tid_s;
    pthread_create(&tid_s, NULL, motor_listen_thread, &ms_s);
    pthread_detach(tid_s);

    int lcm_fd = lcm_get_fileno(state.lcm);
    fd_set rfds;

    tcflush(state.vert_fd, TCIOFLUSH);
    tcflush(state.port_fd, TCIOFLUSH);
    tcflush(state.starb_fd, TCIOFLUSH);

    // listen to LCM and all three serial ports
    while(!program_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(lcm_fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
            lcm_handle(state.lcm);


    }

    pthread_join(tid_v, NULL);
    pthread_join(tid_p, NULL);
    pthread_join(tid_s, NULL);

    close(state.vert_fd);
    close(state.port_fd);
    close(state.starb_fd);

    return 0;
}
