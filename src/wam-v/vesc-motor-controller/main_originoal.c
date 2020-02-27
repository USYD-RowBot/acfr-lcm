/*
 * 
 * 
 * 
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>

#include "buffer.h"
#include "datatypes.h"
#include "bldc_interface.h"
#include "crc.h"
#include "packet.h"
#include "bldc_interface_uart.h"
#include "comm_uart.h"

static mc_values values;
static mc_configuration mcconf;
static app_configuration appconf;

int fd;

////////////////////////////////////////////////////

////////////////////////////////////////////////////

////////////////////////////////////////////////////

////////////////////////////////////////////////////

////////////////////////////////////////////////////

/* Serial Configurations */

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5; /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

////////////////////////////////////////////////////

////////////////////////////////////////////////////

////////////////////////////////////////////////////

// standard send function for enabling other files
static void send_func(unsigned char *d, unsigned int len)
{
    printf("sending: ");
    for (int i = 0; i < len; i++)
        printf("0x%02X ", d[i]);
    printf("\n");

    write(fd, (void *)d, len);
}

// global exit for the threads
int main_exit;

// handles the control-c command to exit/close a thread
void sig_handler(int signo)
{
    if (signo == SIGINT)
        main_exit = 1;
}

// timer thread which must run at least once every millisecond
void *timer_thread()
{
    printf("timer thread starting\n");
    while (!main_exit)
    {
        packet_timerfunc();
        usleep(1000);
    }
}

// read thread which collect all incoming serial and processes int 
void *read_thread()
{
    fd_set rfds;
    struct timeval tv;
    int retval;
    unsigned char d;
    printf("read thread starting\n");

    // will run unless the exit command (control-c) is triggered
    while (!main_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        // timeouts set to occur every second
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        // allows program to monitor different files, such as the file descript fd used for the serial communication
        retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);

        if (retval > 0)
        {

            read(fd, &d, 1);
            //printf("0x%02X\n", d & 0xFF);
            bldc_interface_uart_process_byte(d);
        }
        else
        {
            printf("select timeout, %d\n", retval);
        }
    }
}

void mcconf_callback(mc_configuration *mcconf)
{
    printf("\r\n");
    printf("cc_min_current: %.4f A\r\n", mcconf->cc_min_current);
    printf("End of call\n");
}

/**
 * Print the data values received  
 *
 * @param val
 * The saved values.
 *
 */
void bldc_val_received(mc_values *val)
{
    printf("\r\n");
    printf("Input voltage: %.2f V\r\n", val->v_in);
    printf("Temp:          %.2f degC\r\n", val->temp_mos);
    printf("Current motor: %.2f A\r\n", val->current_motor);
    printf("Current in:    %.2f A\r\n", val->current_in);
    printf("RPM:           %.1f RPM\r\n", val->rpm);
    printf("Duty cycle:    %.1f %%\r\n", val->duty_now * 100.0);
    printf("Ah Drawn:      %.4f Ah\r\n", val->amp_hours);
    printf("Ah Regen:      %.4f Ah\r\n", val->amp_hours_charged);
    printf("Wh Drawn:      %.4f Wh\r\n", val->watt_hours);
    printf("Wh Regen:      %.4f Wh\r\n", val->watt_hours_charged);
    printf("Tacho:         %i counts\r\n", val->tachometer);
    printf("Tacho ABS:     %i counts\r\n", val->tachometer_abs);
    printf("Fault Code:    %s\r\n", bldc_interface_fault_to_string(val->fault_code));
}

int main()
{
    main_exit = 0;
    signal(SIGINT, sig_handler);        // SIGINT is control-c

    pthread_t timerT;
    pthread_t readT;

    // initialise serial communications
    char *portname = "/dev/ttyUSB0";
    int wlen;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    // baudrate 115200, 8 bits, no parity, 1 stop bit 
    set_interface_attribs(fd, B115200);

    pthread_create(&timerT, NULL, timer_thread, NULL);
    pthread_create(&readT, NULL, read_thread, NULL);


    // initiate the bldc specific interfaces
    bldc_interface_uart_init(send_func);

    // bldc_interface_set_rx_mcconf_func(mcconf_callback);
    bldc_interface_set_rx_value_func(bldc_val_received);
    // bldc_interface_set_rpm(3000);

    // simple noncanonical input 
    do
    {

        printf("Acquiring values: \n");
        bldc_interface_get_values();
        // bldc_interface_get_mcconf();


        // set the motor rpm
        bldc_interface_set_rpm(3000);
        usleep(1e6);

    } while (!main_exit);

    bldc_interface_set_rpm(0);
    pthread_join(timerT, NULL);
    pthread_join(readT, NULL);

}
