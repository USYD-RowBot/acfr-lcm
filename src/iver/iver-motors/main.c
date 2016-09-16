/*
    Listens to the motor command message and sends it onto the main thruster
    and the servo controller.  Easliy expanded to support cannards.

    Christian Lees
    ACFR
    2/11/12
*/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#include "perls-common/lcm_util.h"
#include "perls-common/serial.h"
#include "perls-common/timestamp.h"
#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_iver_motor_command_t.h"

#define SERVO_RANGE 0
#define DTOR M_PI/180.
#define TOP_SCALE -1.2

// gears are servo 19T, intermediate 40T, fin shaft 36T
// gear ratio is 0.5277
// Based on testing at OTI it appears that the 0-255 servo commands scale
// to +/- 45 on the fins.  We therefore have removed the servo gear ratio
//#define FIN_TO_SERVO_GEAR_RATIO 0.5277
//#define FIN_TO_SERVO (1 / FIN_TO_SERVO_GEAR_RATIO)
// servo range is 90 degrees for input 0 - 255
#define SERVO_SCALE (256.0/(90.0*DTOR))
// we never allow is to get close to 0 or 255
// the servo does not change when commaned with 255
#define MAX_SERVO_ANGLE (80.0 * DTOR) / 2

#define REMOTE_TIMEOUT 2000000

#define COMMAND_PERIOD 100000

// only global for signalling exit
int program_exit;

void signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}


typedef enum {
    MISSION,
    REMOTE,
    DEAD
} control_source_t;

typedef struct
{
    // serial port stuff
    int servo_fd;
    int motor_fd;

    // lcm stuff param = bot_param_new_from_server (lcm, 1);
    lcm_t *lcm;

    int64_t remote_time;
    control_source_t source_state;

    int64_t last_data_time;

} state_t;

void
send_control(double top, double bottom, double port, double starboard, double main, state_t *state)
{

    // scale the motor commands from actual fin angles to value to send to the motors
    char rudder_top, rudder_bottom;
    char plane_port, plane_starboard;

    rudder_top = (char)((top + 45*DTOR) * SERVO_SCALE);
    rudder_bottom = (char)((bottom + 45*DTOR) * SERVO_SCALE);
    plane_port = (char)((port + 45*DTOR) * SERVO_SCALE);
    plane_starboard = (char)((starboard + 45*DTOR) * SERVO_SCALE);


    char servo_command[12];
    memset(servo_command, 0xFF, 12);
    char motor_string[64];

    servo_command[1] = 0x04;
    servo_command[2] = plane_starboard;

    servo_command[4] = 0x01 + SERVO_RANGE;
    servo_command[5] = -rudder_top; //FIXME: inverted sign for WHOI tail

    servo_command[7] = 0x02 + SERVO_RANGE;
    servo_command[8] = rudder_bottom;

    servo_command[10] = 0x03 + SERVO_RANGE;
    servo_command[11] = plane_port;

    sprintf(motor_string, "MV a=0 A=1000 V=%d G\n", (int)(-main * 32212.0 / 60.0));

    // we don't want to send the data at too high a rate
    if((timestamp_now() - state->last_data_time) > COMMAND_PERIOD)
    {
        static int throttle = 0;

        if (throttle++ > 10)
        {
            printf("Motor cmd: %f %s\n", main, motor_string);
            printf("Servo cmd: t:%f %x b:%f %x s:%f %x p:%f %x\n",
                   top/DTOR, rudder_top,
                   bottom/DTOR, rudder_bottom,
                   starboard/DTOR, plane_starboard,
                   port/DTOR, plane_port);
            throttle = 0;
        }
        state->last_data_time = timestamp_now();
        int res;
        res = write(state->servo_fd, servo_command, 12);
        if (res < 0)
        {
            fprintf(stderr, "Failed to write to servo control socket. %i - %s", errno, strerror(errno));
            program_exit = 1;
        }

        res = write(state->motor_fd, motor_string, strlen(motor_string));
        if (res < 0)
        {
            fprintf(stderr, "Failed to write to motor control socket. %i - %s", errno, strerror(errno));
            program_exit = 1;
        }

    }

}

void
motor_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_iver_motor_command_t *mc_, void *u)
{
    state_t *state = (state_t *)u;

    acfrlcm_auv_iver_motor_command_t mc = *mc_;

    // we got a remote command, set the time and mode
    if(mc.source == ACFRLCM_AUV_IVER_MOTOR_COMMAND_T_REMOTE)
    {
        state->remote_time = mc.utime;
        state->source_state = REMOTE;
    }

    // we got a remote command telling us to go to auto mode
    if(mc.source == ACFRLCM_AUV_IVER_MOTOR_COMMAND_T_RCRELEASE)
    {
        state->source_state = MISSION;
    }

    // if we got an auto command but we are not in mission mode skip
    if(mc.source == ACFRLCM_AUV_IVER_MOTOR_COMMAND_T_AUTO &&
        state->source_state != MISSION)
        return;

    // limit check
    mc.top = mc.top * TOP_SCALE;
    if(mc.top >= MAX_SERVO_ANGLE)
        mc.top = MAX_SERVO_ANGLE;
    else if(mc.top <= -MAX_SERVO_ANGLE)
        mc.top = -MAX_SERVO_ANGLE;


    if(mc.bottom >= MAX_SERVO_ANGLE)
        mc.bottom = MAX_SERVO_ANGLE;
    else if(mc.bottom <= -MAX_SERVO_ANGLE)
        mc.bottom = -MAX_SERVO_ANGLE;

    if(mc.port >= MAX_SERVO_ANGLE)
        mc.port = MAX_SERVO_ANGLE;
    else if(mc.port <= -MAX_SERVO_ANGLE)
        mc.port = -MAX_SERVO_ANGLE;

    if(mc.starboard >= MAX_SERVO_ANGLE)
        mc.starboard = MAX_SERVO_ANGLE;
    else if(mc.starboard <= -MAX_SERVO_ANGLE)
        mc.starboard = -MAX_SERVO_ANGLE;

    send_control(mc.top, mc.bottom, mc.port, mc.starboard, mc.main, state);
}

int
main(int argc, char **argv)
{
    state_t state;
    state.source_state = REMOTE;

    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    // lets start LCM
    state.lcm = lcm_create(NULL);


    // read the config file
    char rootkey[64];
    char key[128];
    BotParam *param = bot_param_new_from_server (state.lcm, 1);
    if(param == NULL)
        return 0;
    sprintf(rootkey, "acfr.%s", basename(argv[0]));


    // servo port info
    sprintf(key, "%s.servo_device", rootkey);
    char *servo_device = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.servo_baud", rootkey);
    int servo_baud = bot_param_get_int_or_fail(param, key);

    sprintf(key, "%s.servo_parity", rootkey);
    char *servo_parity = bot_param_get_str_or_fail(param, key);

    // motor port info
    sprintf(key, "%s.motor_device", rootkey);
    char *motor_device = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.motor_baud", rootkey);
    int motor_baud = bot_param_get_int_or_fail(param, key);

    sprintf(key, "%s.motor_parity", rootkey);
    char *motor_parity = bot_param_get_str_or_fail(param, key);

    // now lets open some ports
    state.servo_fd = serial_open(servo_device, serial_translate_speed(servo_baud), serial_translate_parity(servo_parity), 1);
    if(state.servo_fd < 1)
    {
        fprintf(stderr, "Could not open servo serial port %s\n", servo_device);
        return 1;
    }
    serial_set_ctsrts(state.servo_fd, 0);

    state.motor_fd = serial_open(motor_device, serial_translate_speed(motor_baud), serial_translate_parity(motor_parity), 1);
    if(state.motor_fd < 1)
    {
        fprintf(stderr, "Could not open motor serial port %s\n", servo_device);
        return 1;
    }
    serial_set_ctsrts(state.motor_fd, 0);

    state.last_data_time = timestamp_now();

    acfrlcm_auv_iver_motor_command_t_subscribe(state.lcm, "IVER_MOTOR", &motor_handler, &state);

    // process
    while(!program_exit)
    {
        // if we are in REMOTE mode
        // check the remote time out and zero control (motors off) if required
        if(state.source_state == REMOTE &&
            (timestamp_now() - state.remote_time) > REMOTE_TIMEOUT)
        {
            state.source_state = DEAD;
            // ensure we aren't limited by the command period
            // from sending this
            state.last_data_time -= COMMAND_PERIOD;

            // set fins to null, motor to off
            send_control(0.0, 0.0, 0.0, 0.0, 0.0, &state);
        }

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        lcmu_handle_timeout(state.lcm, &tv);
    }

    close(state.servo_fd);
    close(state.motor_fd);

    return 0;
}
