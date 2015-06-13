/*	DeltaT logger for LCM
    used in conjunction with deltaT program

	Lachlan Toohey
	ACFR
	4/7/2011
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

#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"

#include "perls-lcmtypes/senlcm_deltat_ping_t.h"

void getFileName(const char *logDir, char *fileName, int fileCounter)
{
    struct tm  *tm;
    time_t      currentTime;

    // call time() and gmtime for time
    currentTime = time(NULL);
    tm = gmtime(&currentTime);

    sprintf(fileName,"%s/%04d%02d%02d_%02d%02d-%03d.837", logDir, (int)tm->tm_year+1900,(int) tm->tm_mon+1,
            (int) tm->tm_mday, (int) tm->tm_hour, (int) tm->tm_min, fileCounter);
}

int deltaTExit;

void signalHandler(int sigNum)
{
    // do a safe exit
    deltaTExit = 1;
    printf("Received Signal, exiting\n");
}

struct
{
    const char *logDir;
    int fileCounter;
    int fileSize;
    char fileName[512];
    FILE *outFile;
} loggingFile;

static void deltaTPingCallback(const lcm_recv_buf_t *rbuf, const char *channel,
                               const senlcm_deltat_ping_t *msg, void *user)
{
    // Check if current file is too full.
    struct stat statStruct;
    fstat(fileno(loggingFile.outFile),&statStruct);
    if((int)(statStruct.st_size/1000000.0) > loggingFile.fileSize)
    {
        fclose(loggingFile.outFile);
        getFileName(loggingFile.logDir, loggingFile.fileName, loggingFile.fileCounter);
        loggingFile.outFile = fopen(loggingFile.fileName, "wb");
        printf("\ndeltaT: Logging to file %s\n",loggingFile.fileName);
        loggingFile.fileCounter++;
    }

    fwrite(msg->ping, 1, msg->size, loggingFile.outFile);
    fflush(loggingFile.outFile);
}

int main(int argc, char *argv[])
{

    // install the signal handler
    deltaTExit = 0;
    signal(SIGINT, signalHandler);

    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt,
                            "deltaT logger creates the .837 files from DeltaT Pings sent out on LCM.\n"
                            "\te.g.  deltaT-logger -o \"/media/data/rXXX\" -s 500");
    getopt_add_string (gopt, 'o',  "outdir",          "./",             "Output directory path");
    getopt_add_int    (gopt, 's',  "file-size", "100",           "Maximum size of an individual file (MB)");
    getopt_add_string (gopt, 'l',  "lcm-url",          "\0",            "Log messages on the specified LCM URL");
    getopt_add_help   (gopt, NULL);

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len !=0)
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    const char *outdir = getopt_get_string (gopt, "outdir");
    if (outdir[strlen (outdir)-1] != '/' &&
            outdir[strlen (outdir)-1] != '.')
    {
        fprintf(stderr, "[%s] does not specify a fully qualified directory path", outdir);
        exit (EXIT_FAILURE);
    }

    loggingFile.logDir = outdir;

    loggingFile.fileSize = (int64_t)getopt_get_int (gopt, "file-size");

    // now we have a connection to the sonar we can start LCM so we can listen for messages
    const char *lcmurl = NULL;
    lcm_t *lcm;
    if (getopt_has_flag (gopt, "lcm-url"))
        lcmurl = getopt_get_string (gopt, "lcm-url");
    lcm = lcm_create (lcmurl);
    if (!lcm)
    {
        fprintf(stderr, "lcm_create() failed");
        exit (EXIT_FAILURE);
    }

    // listen for config changes
    senlcm_deltat_ping_t_subscribe (lcm, "DELTA_T_PING", &deltaTPingCallback, NULL);

    // open the initial file
    loggingFile.fileCounter = 1;
    getFileName(loggingFile.logDir, loggingFile.fileName, loggingFile.fileCounter);
    loggingFile.outFile = fopen(loggingFile.fileName, "wb");
    if(loggingFile.outFile == NULL)
        perror("File open");
    fflush(loggingFile.outFile);
    loggingFile.fileCounter++;
    printf("\ndeltaT: Logging to file %s\n",loggingFile.fileName);

    while (!deltaTExit)
    {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        lcmu_handle_timeout(lcm, &tv);
    }

    // wait for the LCM thread to exit
    fclose(loggingFile.outFile);
}
