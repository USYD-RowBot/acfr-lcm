#include <signal.h>
#include <stdio.h>
#include <bot_param/param_client.h>


#include "perls-lcmtypes/senlcm_posmv_t.h"
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
#include "acfr-common/units.h"
#include "acfr-common/nmea.h"


// State information to pass between callbacks
typedef struct
{
    lcm_t *lcm;

    int64_t pashr_utime;
    int64_t xxgst_utime;
    int64_t xxggk_utime;

    // posmv lcm message
    senlcm_posmv_t posmv;

    // local time variables
    int year;
    int month;
    int day;

    char *channel_name;
} state_t;


// Takes a GPS time string in the format hhmmss.sss and returns us since epoch
int64_t utc_time(state_t *state, char *time_str)
{
    char hours[3] = {"\0"};
    char mins[3] = {"\0"};
    char secs[7] = {"\0"};

    strncpy(hours, &time_str[0], 2);
    strncpy(mins, &time_str[2], 2);
    strncpy(secs, &time_str[4], 6);


    struct tm gps_time;
    gps_time.tm_year = state->year + 100;
    gps_time.tm_mon = state->month - 1;
    gps_time.tm_mday = state->day;
    gps_time.tm_hour = atoi(hours);
    gps_time.tm_min = atoi(mins);
    gps_time.tm_sec = (int)floor(atof(secs));

    time_t gps_utc_time = mktime(&gps_time);
    return (int64_t)(gps_utc_time * 1e6) + (int64_t)((fmod(atof(secs),1.0)) * 1e6);
}

// Parse a PASHR attitude message
// $PASHR,hhmmss.sss,xxx.xx,T,RRR.RR,PPP.PP,HHH.HH,a.aaa,b.bbb,c.ccc,d,e*hh<CRLF>
int parse_pashr(state_t *state, char *d)
{
    double val_d;
    int val_i;
    char time_val[16] = {"/0"};
    int ret = 0;

    ret += nmea_argf(d, 4, &val_d);
    state->posmv.roll = val_d * DTOR;

    ret += nmea_argf(d, 5, &val_d);
    state->posmv.pitch = val_d * DTOR;

    ret += nmea_argf(d, 2, &val_d);
    state->posmv.heading = val_d * DTOR;

    ret += nmea_argf(d, 6, &val_d);
    state->posmv.roll_accuracy = val_d * DTOR;

    ret += nmea_argf(d, 7, &val_d);
    state->posmv.pitch_accuracy = val_d * DTOR;

    ret += nmea_argf(d, 8, &val_d);
    state->posmv.heading_accuracy = val_d * DTOR;

    ret += nmea_argi(d, 10, &val_i);
    state->posmv.imu_status = val_i;

    ret += nmea_argc(d, 1, time_val);
    state->pashr_utime = utc_time(state, time_val);

    if(ret == 8)
        return 1;
    else
        return 0;
}

// Parse a xxGST message to get the standard deviations
// $xxGST,hhmmss.sss,,smjr.smjr,smnr.smnr,ooo.o,l.l,y.y,a.a*hh<CRLF>
int parse_xxgst(state_t *state, char *d)
{
    double val_d;
    char time_val[16] = {"/0"};
    int ret = 0;

    ret += nmea_argf(d, 6, &val_d);
    state->posmv.latitude_sd = val_d;

    ret += nmea_argf(d, 7, &val_d);
    state->posmv.longitude_sd = val_d;

    ret += nmea_argc(d, 1, time_val);
    state->xxgst_utime = utc_time(state, time_val);

    if(ret == 3)
        return 1;
    else
        return 0;
}

// Parse a xxVTG message to get velocity
// $xxGST,hhmmss.sss,,smjr.smjr,smnr.smnr,ooo.o,l.l,y.y,a.a*hh<CRLF>
int parse_xxvtg(state_t *state, char *d)
{
    double val_d;
    char time_val[16] = {"/0"};
    int ret = 0;

    ret += nmea_argf(d, 1, &val_d);
    state->posmv.velocity_heading = val_d * DTOR;

    ret += nmea_argf(d, 7, &val_d);
    state->posmv.velocity = val_d * 1000.0 / 3600.0;

    if(ret == 2)
        return 1;
    else
        return 0;
}
// Parse a xxGGK message to get the UTC date and position
// $xxGGK,hhmmss.ss,mmddyy,llll.llllllll,a,yyyyy.yyyyyyyy,b,t,nn,v.v,x.xxx,M*hh<CRLF>
int parse_xxggk(state_t *state, char *d)
{
    double val_d;
    int val_i;
    char val_c;
    char time_val[16] = {"/0"};
    int ret = 0;
    ret += nmea_argf(d, 3, &val_d);
    ret += nmea_argc(d, 4, &val_c);
    int lat_deg_int = (double)((int)val_d / 100);
    double lat_deg = (val_d - 100 * lat_deg_int) / 60 + (double)lat_deg_int;
    state->posmv.latitude = lat_deg * DTOR * (val_c == 'N' ? 1 : -1);

    ret += nmea_argf(d, 5, &val_d);
    ret += nmea_argc(d, 6, &val_c);
    int lon_deg_int = (double)((int)val_d / 100);
    double lon_deg = (val_d - 100 * lon_deg_int) / 60 + (double)lon_deg_int;
    state->posmv.longitude = lon_deg * DTOR * (val_c == 'E' ? 1 : -1);

    ret += nmea_argi(d, 7, &val_i);
    state->posmv.gnss_quality = val_i;

    ret += nmea_argi(d, 8, &val_i);
    state->posmv.num_sats = val_i;

    char day[3] = {"\0"};
    char month[3] = {"\0"};
    char year[3] = {"\0"};
    char date_str[7] = {"\0"};

    ret += nmea_argc(d, 2, date_str);
    strncpy(month, &date_str[0], 2);
    strncpy(day, &date_str[2], 2);
    strncpy(year, &date_str[4], 2);

    state->year = atoi(year);
    state->month = atoi(month);
    state->day = atoi(day);
    ret += nmea_argc(d, 1, time_val);
    state->xxggk_utime = utc_time(state, time_val);

    if(ret == 8)
    {
        state->posmv.utime = state->xxggk_utime;
        state->posmv.gps_time = state->xxggk_utime;
        //if((abs(state->pashr_utime - state->xxggk_utime) < 100e3) && (abs(state->xxgst_utime - state->xxggk_utime) < 100e3))
            senlcm_posmv_t_publish(state->lcm, state->channel_name, &state->posmv);
        //else
        //    printf("Timestamp mismatch\n");

        return 1;
    }
    else
        return 0;

}

int program_gps(int fd, char *cmd)
{
    write(fd, cmd, strlen(cmd));
    return 1;
}


void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, char **channel_name)
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
            snprintf(*channel_name, 200, "%s.POSMV", (char *)optarg);
            break;
         }
    }
}


int program_exit;
void signal_handler(int sig_num)
{
    // do a safe exit
    program_exit = 1;
}

int main(int argc, char *argv[])
{
    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    state_t state;
    memset(&state, 0, sizeof(state_t));
    parse_args(argc, argv, &state.channel_name);

    state.lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(state.lcm, rootkey);
    if(sensor == NULL)
        return 0;

    acfr_sensor_canonical(sensor, '\r', '\n');

    char posmv_str[512];
    int bytes;

    while(!program_exit)
    {
	memset(posmv_str, 0, sizeof(posmv_str));
        bytes = acfr_sensor_read_timeout(sensor, posmv_str, sizeof(posmv_str), 1);
        if(bytes > 0)
        {
            if(strstr(posmv_str, "PASHR") != NULL)
                parse_pashr(&state, posmv_str);
            else if(strstr(posmv_str, "GST") != NULL)
                parse_xxgst(&state, posmv_str);
            else if(strstr(posmv_str, "GGK") != NULL)
                parse_xxggk(&state, posmv_str);
            else if(strstr(posmv_str, "VTG") != NULL)
                parse_xxvtg(&state, posmv_str);
        }
    }

    acfr_sensor_destroy(sensor);

    return 1;
}




