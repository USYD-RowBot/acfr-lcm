#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>

#include "perls-common/timestamp.h"
#include "perls-lcmtypes/senlcm_tcm_t.h"
#include "acfr-common/sensor.h"

#include "tcm.h"

#define update_rate 0.1

#define DTOR 3.141592/180

int parse_tcm(char *buf, senlcm_tcm_t *tcm)
{
    // decode the tcm frame
    char frame_id = buf[2];
    char *current_pos;


    if(frame_id == kDataResp)
    {
        int count = (int)buf[3];
        current_pos = &buf[4];
        int index = 0;
        while(1)
        {
            int data_id = (int)*current_pos++;

            switch(data_id)
            {
            case kHeading:
                tcm->heading = (*(float *)current_pos) * DTOR;
                current_pos += 4;
                index++;
                break;
            case kPAngle:
                tcm->pitch = (*(float *)current_pos) * DTOR;
                current_pos += 4;
                index++;
                break;
            case kRAngle:
                tcm->roll = (*(float *)current_pos) * DTOR;
                current_pos += 4;
                index++;
                break;
            case kTemperature:
                tcm->temperature = *(float *)current_pos;
                current_pos += 4;
                index++;
                break;

//                    case kXAligned	:
//                        tcm->mag_x = (*(float *)current_pos);
//                        current_pos += 4;
//                        index++;
//                        break;
//                    case kYAligned:
//                        tcm->mag_y = (*(float *)current_pos);
//                        current_pos += 4;
//                        index++;
//                        break;
//                    case kZAligned:
//                        tcm->mag_z = (*(float *)current_pos);
//                        current_pos += 4;
//                        index++;
//                        break;

            }
            if(index == count)
                break;

        }
    }

    return 1;

}

int tcm_form_message(char *in, int data_len, char *out)
{
    unsigned short len = data_len + 4;
    out[0] = ((char *)&len)[1];
    out[1] = ((char *)&len)[0];

    memcpy(&out[2], in, data_len);
    unsigned short crc = tcm_crc(out, data_len + 2);
    out[data_len+3] = crc & 0xff;
    out[data_len+2] = crc >> 8;

    return len;
}



int program_tcm(acfr_sensor_t *s)
{
    char data[128];
    char out[128];
    int len;

    // set the return types
    memset(data, 0, sizeof(data));
    data[0] = kSetDataComponents;
    data[1] = 4;
    data[2] = kHeading;
    data[3] = kPAngle;
    data[4] = kRAngle;
    data[5] = kTemperature;
//    data[6] = kXAligned;
//    data[7] = kYAligned;
//    data[8] = kZAligned;
    len = tcm_form_message(data, 6, out);
//    len = tcm_form_message(data, 9, out);
    acfr_sensor_write(s, out, len);


    // set the acquisition mode
    memset(data, 0, sizeof(data));
    data[0] = kSetAcqParams;
    data[1] = 0;     // data push mode
    data[2] = 0;     // flush filter is false
    float interval = update_rate;
    memcpy(&data[7], &interval, sizeof(float));
    len = tcm_form_message(data, 11, out);
    acfr_sensor_write(s, out, len);

    // set the filter mode
    double c4[4] = {4.6708657655334e-2, 4.5329134234467e-1, 4.5329134234467e-1, 4.6708657655334e-2};
    double c8[8] = {01.9875512449729e-2, 06.4500864832660e-2, 01.6637325898141e-1, 02.4925036373620e-1,
                    02.4925036373620e-1, 01.6637325898141e-1, 06.4500864832660e-2, 01.9875512449729e-2
                   };
    char filter_length = 8;
    memset(data, 0, sizeof(data));
    data[0] = 12; //kSetFIRFilters;
    data[1] = 3;
    data[2] = 1;
    data[3] = filter_length;
    if( filter_length == 4 )
    {
        memcpy(&data[4], c4, sizeof(double)*filter_length);
    }
    else
    {
        memcpy(&data[4], c8, sizeof(double)*filter_length);
    }
    len = tcm_form_message(data, sizeof(double)*filter_length + 4, out);
    acfr_sensor_write(s, out, len);


    // start
    memset(data, 0, sizeof(data));
    data[0] = kStartIntervalMode;
    len = tcm_form_message(data, 1, out);
    acfr_sensor_write(s, out, len);

    return 1;
}

int program_exit;
int broken_pipe;
void
signal_handler(int sig_num)
{
    printf("Got a signal\n");
    // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else if (sig_num == SIGTERM)
        broken_pipe = 1;
    else if(sig_num == SIGINT)
        program_exit = 1;
}

int
main (int argc, char *argv[])
{
    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;

    struct sigaction sa;
    sa.sa_flags = 0;
    sigemptyset (&sa.sa_mask);
    sigaddset(&sa.sa_mask, SIGPIPE);
    sigaddset(&sa.sa_mask, SIGHUP);
    sigaddset(&sa.sa_mask, SIGTERM);
    sigaddset(&sa.sa_mask, SIGINT);
    sa.sa_handler = SIG_IGN;
    if (sigaction(SIGPIPE, &sa, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
    sa.sa_handler = signal_handler;
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    // signal(SIGINT, signal_handler);
    //signal(SIGPIPE, signal_handler);

    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    acfr_sensor_noncanonical(sensor, 1, 0);

    int len;
    char buf[256];

    int64_t timestamp;
    senlcm_tcm_t tcm;

    program_tcm(sensor);
    int programmed = 1;

    fd_set rfds;

    while(!program_exit)
    {
        //printf("Programed: %d, open: %d, pipe: %d\n", programmed, sensor->port_open, broken_pipe);

        // check for broken pipes, if it is broken make sure it is closed and then reopen it
        if(broken_pipe)
        {
            sensor->port_open = 0;
            programmed = 0;
            fprintf(stderr, "Pipe broken\n");
            continue;
        }

        if(!programmed)
        {
            fprintf(stderr, "reprogramming the device\n");
            program_tcm(sensor);
            programmed = 1;
            continue;
        }

        memset(buf, 0, sizeof(buf));

        FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            timestamp = timestamp_now();
            len = acfr_sensor_read(sensor, buf, 1);
            if(len == 1)
            {
                if (buf[0] == 0)
                {
                    // read another byte
                    len += acfr_sensor_read(sensor, &buf[1], 1);
                    if(len != 2)
                        continue;

                    unsigned short data_len = (buf[0] << 8) + buf[1];
                    //printf ("Looking for %d bytes\n", data_len);
                    if(data_len > 5 && data_len < 56)
                    {
                        // read the rest of the data
                        while(len < data_len)
                            len += acfr_sensor_read(sensor, &buf[len], data_len - len);

                        // check the checksum
                        unsigned short crc = *(unsigned short *)&buf[len - 2];
                        if(tcm_crc(&buf[0], data_len-2) == (((crc >> 8) | (crc & 0xff) << 8)))
                        {
                            memset(&tcm, 0, sizeof(senlcm_tcm_t));
                            tcm.utime = timestamp;
                            // its good data, lets parse it

                            if(parse_tcm(&buf[0], &tcm))
                                senlcm_tcm_t_publish(lcm, "TCM", &tcm);
                        }
                        else
                            printf("Bad CRC\n");
                    }
                }
                else
                    printf("Bad data_len\n");
            }
            else
            {
                fprintf(stderr, "Error in read\n");
            }
        }
        else
        {
            // timeout, check the connection
            fprintf(stderr, "Timeout: Checking connection\n");
            acfr_sensor_write(sensor, "\n", 1);
            //if (acfr_sensor_write(sensor, "\n", 1) < 0)
            //fprintf(stderr, "Broken pipe!\n");
            //else
            //fprintf(stderr, "Pipe appears fine.\n");
        }
    }

    acfr_sensor_destroy(sensor);
    lcm_destroy(lcm);

    return 0;

}
