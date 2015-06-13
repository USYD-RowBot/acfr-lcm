/* 	OAS sensor driver
	Based on SIO_oas.c from the vehicle code

	Christian Lees
	ACFR
	6/12/10
*/


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <math.h>

#include <bot_param/param_client.h>
#include "perls-common/units.h"
#include "perls-common/timestamp.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_oas_t.h"
#include "oas.h"

#include "test_ranges.h"


typedef struct
{
    int range;
    int startGain;
    double absorption;
    int pulse;
    int dataPoints;
    double minRange;
    double altitudeOffset;
    double declinationAngRad;
    generic_sensor_driver_t *gsd;
    unsigned char oasCmd[IMA_TOTAL_SIZE];
    double *rangeFilter;
    int filterWidth;
    int count;
} imagenex852_t;

FILE *fp;

static int double_comparison( const void *p1, const void *p2 )
{
    double d1 = *(double *)p1;
    double d2 = *(double *)p2;
    return (int)((d1 - d2) / fabs(d1 - d2));
}

int oasCommandCompose(imagenex852_t *cfg, unsigned char *serCmd)
{

    unsigned char newAbsv  = (unsigned char)(cfg->absorption * 100);
    unsigned char newDataPts;

    if(cfg->pulse == IMA_TERMBYTE)	//Add protection for 0xFD...
        cfg->pulse = 254;

    // data points...
    // One point is the standard profiling mode.
    if((cfg->dataPoints == IMA_SHORT_N_POINTS) || (cfg->dataPoints == IMA_LONG_N_POINTS))
        newDataPts = cfg->dataPoints/10;
    else if (cfg->dataPoints == 1 )
        newDataPts = 1;		//No trace data, profiling Mode....
    else
    {
        printf("OAS: Wrong num data points (%d) in config. Valid values 1, 250 or 500\n", cfg->dataPoints);
        return 0;  //failure
    }

    serCmd[IMA_HEAD1]   = IMA_HEADER1;			//always 0xFE;
    serCmd[IMA_HEAD2]   = IMA_HEADER2;			//Always 0x44
    serCmd[IMA_H_ID]    = IMA_HEADID;			// Head ID, fixed @ 0x11
    serCmd[IMA_RNG]     = cfg->range;        //available ranges: 5, 10, 20, 30, 40, 50m
    serCmd[IMA_RNG_O]    = 0;
    serCmd[IMA_HOLD]     = 0;			// 1 holds
    serCmd[IMA_MA_SL]    = 0;			// Byte 6, always 0, reserved.
    serCmd[IMA_R_HSTAT]  = 0;
    serCmd[IMA_S_GAIN]   = cfg->startGain;
    serCmd[IMA_LOGF]     = 0;			// 9; Reserved, always 0.
    serCmd[IMA_ABSV]     = newAbsv;     // 10
    serCmd[IMA_TRAIN]    = 0;		    // 11; reserved, always 0
    serCmd[IMA_SECTW]    = 0;			// 12; reserved, always 0
    serCmd[IMA_STEP]     = 0;			// 13: reserved, always 0
    serCmd[IMA_PULSE]    = cfg->pulse; // 14
    serCmd[IMA_LPF]      = (unsigned char)(cfg->minRange * 10);			// 15: minimum profile range
    serCmd[IMA_ST_SCAN]  = 0;			// 16: reserved, always 0
    serCmd[IMA_MOVE_REL] = 0;			// 17: reserved, always 0
    serCmd[IMA_NSWEEPS]  = 0;			// 18: reserved, always 0
    serCmd[IMA_NDPOINTS] = newDataPts;	// 19: if not profile mode: 50 for 500 points, 25 for 250
    serCmd[IMA_NDBITS]   = 0;			// 20: Reserved, always 0
    serCmd[IMA_UPBAUD]   = 0;			// 21: reserved, always 0

    if(cfg->dataPoints == 1)
        serCmd[IMA_PROF]     = 1;          // 22  profile mode
    else
        serCmd[IMA_PROF]     = 0;          // 22  get all 250/500 points back; trace mode

    serCmd[IMA_CALIB]    = 0;			// 23: reserved, always 0
    serCmd[IMA_DELAY]    = 0;			// 24  Switch delay 0 - 252 in 2ms increments
    serCmd[IMA_FREQ]     = 0;     // 25
    serCmd[IMA_TERM]     = 0xFD;			// 26, termination byte

    return 1; //This is success...
}



static int myopts(generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Imagenex 852 OAS driver.");
    return 0;
}

void
heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    imagenex852_t *oasConfig = (imagenex852_t *)u;
    int ret;
    unsigned char *serialData = malloc(513);

    gsd_write(oasConfig->gsd, (char *)oasConfig->oasCmd, IMA_TOTAL_SIZE);

    // read the return message

    // read one byte and make sure its the correct character
    gsd_noncanonical (oasConfig->gsd, 1, 0);

    ret = gsd_read_timeout(oasConfig->gsd, (char *)serialData, 1, NULL, 1000000);
    if((ret == 1) && (serialData[0] == 'I'))
    {
        // the first character is correct, get the rest of the header
        gsd_noncanonical (oasConfig->gsd, 1, 0);
        ret = 0;
        do
        {
            ret += gsd_read_timeout(oasConfig->gsd, (char *)&serialData[1+ret], 11-ret, NULL, 10000000);
        }
        while (ret < 11);

        if((serialData[1] == 'M') || (serialData[1] == 'P'))   //&& (serialData[2] == 'G')) {
        {
            // now get the rest of the packet
            char dataSizeH = (serialData[11] & 0x7E) >> 1;
            char dataSizeL = ((serialData[11] & 0x01) << 7) | (serialData[10] & 0x7F);
            unsigned short dataSize = (dataSizeH << 8) + (dataSizeL & 0xFF);
            gsd_noncanonical (oasConfig->gsd, 1, 0);
            ret = 0;
            do
            {
                ret += gsd_read_timeout(oasConfig->gsd, (char *)&serialData[12+ret], dataSize + 1 - ret, NULL, 1000000);
            }
            while(ret < dataSize+1);

            printf("%s\n", serialData);
            // now we can parse the data
//				char headPosH   = ((serialData[6] & 0x3E) >> 1);
//				char headPosL   = ((serialData[6] & 0x01) << 7) | (serialData[5] & 0x7F) ;
//				int iHeadPos    = (headPosH << 8 ) | headPosL ;

//				int range       = serialData[7];

            char profRngH   = ((serialData[9] & 0x7E) >> 1);
            char profRngL   = ((serialData[9] & 0x01) << 7) | (serialData[8] & 0x7F) ;
            unsigned int iProfRng    = (profRngH << 8 ) | profRngL ;

            double profRange     = ((double) iProfRng) * IMA_PROFRNG_FACTOR ;
            //if(oasConfig->count == rSize)
            //    exit(-1);

            //double profRange = r[oasConfig->count];
            // if the reported range is less then the minimum range send out a -1
            // FIXME: this will effect the output rate
            if(profRange < oasConfig->minRange)
            {
                printf("minRange, profRange = %f, iRange = %d\n", profRange, iProfRng);
                return;
            }

            // linear buffer here
            for(int i=1; i<oasConfig->filterWidth; i++)
                oasConfig->rangeFilter[oasConfig->filterWidth - i] = oasConfig->rangeFilter[oasConfig->filterWidth - i - 1];
            oasConfig->rangeFilter[0] = profRange;

            // count up
            oasConfig->count++;

            senlcm_oas_t oasData;
            // post the LCM message
            //if( oasConfig->count % oasConfig->filterWidth == 0 ) {

            // Get median value
            double medianRange = 0;
            double rangeFilter[oasConfig->filterWidth];
            memcpy(rangeFilter, oasConfig->rangeFilter, sizeof(double) * oasConfig->filterWidth);
            qsort ( rangeFilter, oasConfig->filterWidth, sizeof(double), &double_comparison);
            if( oasConfig->filterWidth % 2 == 0 )
            {
                medianRange = rangeFilter[ oasConfig->filterWidth/2+1 ];
            }
            else
            {
                medianRange = rangeFilter[ oasConfig->filterWidth/2 ];
            }

            double fProfRange = medianRange;
            double fPseudoAlt = fProfRange*sin(oasConfig->declinationAngRad) - oasConfig->altitudeOffset;
            double fPseudoFwdDistance = fProfRange*cos(oasConfig->declinationAngRad);

            // }


            oasData.utime = timestamp_now();
            oasData.id = oasConfig->count / oasConfig->filterWidth;

            oasData.profRange = profRange;
            oasData.pseudoAlt = oasData.profRange*sin(oasConfig->declinationAngRad) - oasConfig->altitudeOffset;
            oasData.pseudoFwdDistance = oasData.profRange*cos(oasConfig->declinationAngRad);


            if(fProfRange > oasConfig->minRange && fProfRange < oasConfig->range)
            {
                oasData.fProfRange = fProfRange;
                oasData.fPseudoAlt = fPseudoAlt;
                oasData.fPseudoFwdDistance = fPseudoFwdDistance;
            }
            else
            {
                oasData.fProfRange = oasConfig->range;
                oasData.fPseudoAlt = oasConfig->range;
                oasData.fPseudoFwdDistance = oasConfig->range;
            }

            // copy the trace data into the buffer
            oasData.dataPoints = dataSize;
            oasData.trace = malloc(dataSize);
            memset(oasData.trace, 0, dataSize);
            memcpy(oasData.trace, &serialData[12], dataSize);

            fprintf(fp,"%d, %f, %f, %f, %f\n", oasConfig->count, oasData.profRange, oasData.fProfRange, oasData.pseudoAlt, oasData.fPseudoAlt);

            senlcm_oas_t_publish(oasConfig->gsd->lcm, oasConfig->gsd->channel, &oasData);
            free(oasData.trace);
            gsd_update_stats(oasConfig->gsd, 1);

        }
    }
    free(serialData);
}


int main (int argc, char *argv[])
{


    // lets read the config file
    fp = fopen("/tmp/oas.out", "w");
    imagenex852_t oasConfig;
    memset(&oasConfig, 0, sizeof(imagenex852_t));
    oasConfig.gsd = gsd_create (argc, argv, NULL, myopts);

    oasConfig.range = bot_param_get_int_or_fail(oasConfig.gsd->params, "sensors.oas.range");

    oasConfig.startGain = bot_param_get_int_or_fail(oasConfig.gsd->params, "sensors.oas.startGain");

    oasConfig.absorption = bot_param_get_double_or_fail(oasConfig.gsd->params, "sensors.oas.absorption");

    oasConfig.pulse = bot_param_get_int_or_fail(oasConfig.gsd->params, "sensors.oas.pulse");

    oasConfig.dataPoints = bot_param_get_int_or_fail(oasConfig.gsd->params, "sensors.oas.dataPoints");

    oasConfig.minRange = bot_param_get_double_or_fail(oasConfig.gsd->params, "sensors.oas.minRange");

    oasConfig.altitudeOffset = bot_param_get_double_or_fail(oasConfig.gsd->params, "sensors.oas.altitudeOffset");

    oasConfig.declinationAngRad = bot_param_get_double_or_fail(oasConfig.gsd->params, "sensors.oas.declinationAngRad");

    oasConfig.filterWidth = bot_param_get_int_or_fail(oasConfig.gsd->params, "sensors.oas.filterWidth");

    if(oasConfig.filterWidth <= 0)
    {
        printf("filter width is invalid\n");
        return 1;
    }

    oasConfig.rangeFilter = (double *)malloc(oasConfig.filterWidth * sizeof(double));

    if(oasConfig.rangeFilter == NULL)
        return 1;
    for(int i=0; i<oasConfig.filterWidth; i++)
        oasConfig.rangeFilter[i] = (double)oasConfig.range;

    // create the switch command for the sonar head
    memset(&oasConfig.oasCmd, 0, IMA_TOTAL_SIZE);
    if(!oasCommandCompose(&oasConfig, oasConfig.oasCmd))
        return 1;

    // if we get to here then we can start the port
    gsd_launch (oasConfig.gsd);
    gsd_flush (oasConfig.gsd);
    gsd_reset_stats (oasConfig.gsd);

    perllcm_heartbeat_t_subscribe(oasConfig.gsd->lcm, "HEARTBEAT_10HZ", &heartbeat_handler, &oasConfig);

    // the main loop
    while(!oasConfig.gsd->done)
    {
        lcm_handle(oasConfig.gsd->lcm);
        // send a switch command to the OAS

    }
    free(oasConfig.rangeFilter);
    fclose(fp);
    return 0;
}



