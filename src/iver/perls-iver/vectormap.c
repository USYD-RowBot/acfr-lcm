/************************************************************************
 *    parse_mission()   -- parses waypoints from vetor map mission file *
 *    parse_cmd() -- parses the cmd string in each waypoint             *
 *                                                                      *
 *      see Iver2_MissionPlanningManual_Rev_1.2.pdf for info regarding  *
 *    decoding a mission file                                           *
 *                                                                      *
 *    this function catalogs data from the waypoint section             *
 *    of the mission file and ignores everything else                   *
 ************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// external linking req'd
#include <glib.h>

#include "perls-common/error.h"
#include "perls-common/units.h"

#include "vectormap.h"

#define MAXLINE 1024


//--------------------------------------------------------------------------
// parse cmd
//    splits the command string (after the lat lng, dist, and heading) on
//    whitespace and then fills the element array. order of cmd string not
//    important
//--------------------------------------------------------------------------
static void
_iver_vectormap_parse_cmd (const char *cmd, iver_vectormap_waypoint_t *element, char *cmd_fixed)
{

    //printf ("CMD I: %s\n", cmd);

    const char *cmd_ptr = cmd;
    char *cmd_fixed_ptr = cmd_fixed;
    char chunk[MAXLINE];
    char chunk_fixed[MAXLINE];
    int len = strlen (cmd);
    while ((cmd_ptr-cmd < len ) && sscanf (cmd_ptr, "%s", chunk))
    {

        int is_chunk_fixed = 0;

        switch (chunk[0])
        {
        case 'D': // D# = Depth from surface
            element->dfs.use = 1;
            element->hfb.use = 0;
            element->undulate.use = 0;
            if (1==sscanf (chunk, "D%lf", &element->dfs.depth))
                element->dfs.depth *= UNITS_FEET_TO_METER;
            else
                ERROR ("unable to parse D [depth from surface]");
            break;
        case 'H': // H%lf Height from bottom
            element->dfs.use = 0;
            element->hfb.use = 1;
            element->undulate.use = 0;
            if (1==sscanf (chunk, "H%lf", &element->hfb.height))
                element->hfb.height *= UNITS_FEET_TO_METER;
            else
                ERROR ("unable to parse H [height from bottom]");
            break;
        case 'U': // Undulate
            element->dfs.use = 0;
            element->hfb.use = 0;
            element->undulate.use = 1;
            if (3==sscanf (chunk, "U%lf,%lf,%lf",
                           &element->undulate.depth1,
                           &element->undulate.depth2,
                           &element->undulate.dive_angle))
            {
                element->undulate.dive_angle *= UNITS_DEGREE_TO_RADIAN;
                element->undulate.depth1 *= UNITS_FEET_TO_METER;
                element->undulate.depth2 *= UNITS_FEET_TO_METER;
            }
            else
                ERROR ("unable to parse U [undulate]");
            break;
        case 'P': // P# =  Time to Park
            if (1!=sscanf (chunk, "P%lf", &element->park_time))
                ERROR ("unable to parse P [parking time]");
            break;
        case 'S':
            if (chunk[1] == 'S')   // SS sidescan sonar
            {
                element->sonar.use = 1;
                if (5!=sscanf (chunk, "SS%2d%3d%c%c%1d",
                               &element->sonar.gain,
                               &element->sonar.range,
                               &element->sonar.chan,
                               &element->sonar.freq,
                               &element->sonar.type))
                    ERROR ("unable to parse SS [sidescan sonar]");
            }
            else   // S speed
            {
                if (1==sscanf (chunk, "S%lf", &element->speed))
                    element->speed *= UNITS_KNOT_TO_METER_PER_SEC;
                else
                    ERROR ("unable to parse S [speed]");
            }
            break;
        case 'V': //as in VC (vehicle camera)
            if (7!=sscanf (chunk, "VC1,%d,%d,%d,%d,VC2,%d,%d,%d",
                           &element->cam1.mode, //0=off, 1=video, 2=still
                           &element->cam1.quality, //0=not used, 1=high, 2=med, 3=low
                           &element->cam1.rate, //in steps of 500 ms
                           &element->camera_overlay, // 0=none, 1=depth, 2=position, 3=both
                           &element->cam2.mode,
                           &element->cam2.quality,
                           &element->cam2.rate))
                ERROR ("unable to parse VC [vehicle camera]");
            if (NULL != cmd_fixed)
            {
                sprintf (chunk_fixed, "VC1,%d,%d,%d,%d,VC2,%d,%d,%d,%d",
                         0, //0=off, 1=video, 2=still
                         0, //0=not used, 1=high, 2=med, 3=low
                         element->cam1.rate, //in steps of 500 ms
                         0, // 0=none, 1=depth, 2=position, 3=both
                         0,
                         0,
                         element->cam2.rate,
                         0);
                is_chunk_fixed = 1;
            }

            break;
        default:
            printf ("\n!----- UNKNOWN COMMAND PARAM %s -----!\n", chunk);
        }

        // if chunk fixed write it to the output
        if (NULL != cmd_fixed )
        {

            if (is_chunk_fixed)
            {
                sprintf (cmd_fixed_ptr, "%s ", chunk_fixed);
                cmd_fixed_ptr += strlen(chunk_fixed) + 1;
            }
            else
            {
                sprintf (cmd_fixed_ptr, "%s ", chunk);
                cmd_fixed_ptr += strlen(chunk) + 1;
            }
        }

        // move the cmd pointer to the start of the next chunk
        cmd_ptr += strlen (chunk) + 1;
    }

    // null terminate fixed cmd
    cmd_fixed_ptr[-1] = '\0'; //-1 because removing last trailing space;

    //printf ("CMD O: %s\n", cmd_fixed);
}

GList *
iver_vectormap_parse_n_fix_mission (const char *filename, const char *filename_out)
{

    //list of all the waypoints in the file
    GList *list = NULL;

    FILE *f = fopen (filename, "r");
    if (f == NULL)
    {
        ERROR ("cannot open %s\n", filename);
        return NULL;
    }
    else
        printf ("file \"%s\" opened successfully\n", filename);

    FILE *f_out = NULL;
    if (NULL != filename_out)
    {
        f_out = fopen (filename_out, "w");
        if (f_out == NULL)
        {
            ERROR ("cannot open fix output file %s\n", filename_out);
            return NULL;
        }
        else
            printf ("fix output file \"%s\" opened successfully\n", filename_out);
    }

    char line[MAXLINE];
    char line_left_overs[MAXLINE];
    int in_waypoint_block = 0; //flag if we are in the waypoint plock
    int finished = 0; //flag for while loop
    while (NULL != fgets (line, MAXLINE, f))    //scroll through each line
    {
        if (0==strncmp (line, "1;", 2) && !finished)
            in_waypoint_block = 1;
        else if (0==strncmp (line, "END", 3) && !finished)
        {
            in_waypoint_block = 0;
            finished = 1; //might be other START -  END blocks only look at first
        }

        if (in_waypoint_block)
        {
            iver_vectormap_waypoint_t *element = g_malloc0 (sizeof (*element));
            //to make sure the bool flags are initiated to zero
            element->dfs.use = 0;
            element->dfs.use = 0;
            element->dfs.use = 0;
            element->sonar.use = 0;

            // scan the waypoint position information and then get
            // command string for further processing
            //if (6==sscanf (line, "%d; %lf; %lf; %lf; %lf; %[^;\n]",
            if (7==sscanf (line, "%d; %lf; %lf; %lf; %lf; %[^;]; %[^\n]]",
                           &element->num,
                           &element->latitude,
                           &element->longitude,
                           &element->distance,
                           &element->heading,
                           element->raw_cmd,
                           line_left_overs))
            {

                char cmd_out[MAXLINE] = {0};
                _iver_vectormap_parse_cmd (element->raw_cmd, element, cmd_out);
                char line_out[MAXLINE] = {0};
                sprintf (line_out, "%d; %lf; %lf; %0.3lf; %0.3lf; %s; %s\n",
                         element->num,
                         element->latitude,
                         element->longitude,
                         element->distance,
                         element->heading,
                         cmd_out,
                         line_left_overs);

                list = g_list_prepend (list, (gpointer)element);

                if (NULL != filename_out && NULL != f_out)
                {
                    fputs (line_out, f_out);
                }
                //printf ("LI: %s", line);
                //printf ("LO: %s", line_out);

                // convert angles to radians to keep everything SI
                element->heading   *= UNITS_DEGREE_TO_RADIAN;
                element->latitude  *= UNITS_DEGREE_TO_RADIAN;
                element->longitude *= UNITS_DEGREE_TO_RADIAN;
                element->distance  *= UNITS_FEET_TO_METER;

            }
            else
            {
                ERROR ("unable to scan waypoint position information");
                printf ("RAW_CMD: %s \n", element->raw_cmd);
            }

            //print_waypoint (element);
        }
        else
        {

            //write line to output file?
            if (NULL != filename_out && NULL != f_out)
            {
                fputs (line, f_out);
            }

        }

    }

    list = g_list_reverse (list);

    fclose (f);
    if (NULL != filename_out && NULL != f_out)
    {
        fclose (f_out);
    }
    return list;
}

//--------------------------------------------------------------------------
// Parses the mission file specified by file_name
//--------------------------------------------------------------------------
GList *
iver_vectormap_parse_mission (const char *filename)
{
    return iver_vectormap_parse_n_fix_mission (filename, NULL);
}



GList *
iver_vectormap_parse_buoy (const char *filename)
{

    //list of all the buoys in the file
    GList *list = NULL;

    FILE *f = fopen (filename, "r");
    if (f == NULL)
    {
        ERROR ("cannot open %s\n", filename);
        return NULL;
    }
    else
        printf ("file \"%s\" opened successfully\n", filename);


    char line[MAXLINE];
    int in_buoy_block = 0; //flag if we are in the waypoint plock
    int finished = 0; //flag for while loop
    int count = 1;
    while (NULL != fgets (line, MAXLINE, f))    //scroll through each line
    {

        if (0==strncmp (line, "END BUOY DATA", 13) && !finished)
        {
            in_buoy_block = 0;
            finished = 1; //might be other START -  END blocks only look at first
        }

        if (in_buoy_block)
        {

            iver_vectormap_buoy_t *element = g_malloc0 (sizeof (*element));

            if (2 == sscanf (line, "%lf; %lf;", &element->latitude, &element->longitude))
            {

                list = g_list_prepend (list, (gpointer)element);

                // convert angles to radians to keep everything SI
                element->latitude  *= UNITS_DEGREE_TO_RADIAN;
                element->longitude *= UNITS_DEGREE_TO_RADIAN;

                element->num = count;
                count++;

            }
            else
            {
                ERROR ("unable to scan buoy position information: %s", line);
            }

        }

        if (0==strncmp (line, "START BUOY DATA", 15) && !finished)
        {
            in_buoy_block = 1;
        }

    }

    list = g_list_reverse (list);

    return list;
}


//--------------------------------------------------------------------------
// free the list built by iver_vectormap_parse_mission()
//--------------------------------------------------------------------------
static void
my_free (gpointer data, gpointer user_data)
{
    g_free (data);
}

void
iver_vectormap_free_waypoints (GList *waypoints)
{
    g_list_foreach (waypoints, &my_free, NULL);
    g_list_free (waypoints);
}

void
iver_vectormap_free_buoys (GList *buoys)
{
    g_list_foreach (buoys, &my_free, NULL);
    g_list_free (buoys);
}

//--------------------------------------------------------------------------
// Print a waypoint (for debugging)
//--------------------------------------------------------------------------
void
iver_vectormap_print_waypoint (iver_vectormap_waypoint_t *element_ptr, char *output)
{
    iver_vectormap_waypoint_t element = *element_ptr;

    char tmp[1024] = {'\0'};

    sprintf (tmp, "%sWaypoint %d: \n", tmp, element.num);
    sprintf (tmp, "%slat: %0.6lf deg\nlng: %0.6lf deg\n", tmp,
             element.latitude * UNITS_RADIAN_TO_DEGREE,
             element.longitude * UNITS_RADIAN_TO_DEGREE);
    sprintf (tmp, "%sdist: %0.1lf m \nheading: %0.1lf deg\ntime %0.1lf sec\n", tmp,
             element.distance, element.heading * UNITS_RADIAN_TO_DEGREE, element.time);
    //sprintf (tmp, "%scmd: %s \n", tmp, element.raw_cmd);
    sprintf (tmp, "%spark-time: %0.1lf sec\nspeed: %0.1lf m/s\n", tmp, element.park_time, element.speed);
    if (element.dfs.use)
        sprintf (tmp, "%sdfs: %0.1lf m\n", tmp, element.dfs.depth);
    if (element.hfb.use)
        sprintf (tmp, "%shfb: %0.1lf m\n", tmp, element.hfb.height);
    if (element.undulate.use)
        sprintf (tmp, "%sundulate: d1: %0.1lf d2 %0.1lf\n dive-angle: %0.1lf\n", tmp,
                 element.undulate.depth1, element.undulate.depth2,
                 element.undulate.dive_angle * UNITS_RADIAN_TO_DEGREE);
    if (element.sonar.use)
        sprintf (tmp, "%sSonar: gain: %d range: %d chan: %c freq: %c type: %d\n", tmp,
                 element.sonar.gain, element.sonar.range,
                 element.sonar.chan, element.sonar.freq, element.sonar.type);
    if (element.cam1.mode)
        sprintf (tmp, "%scam1: mode: %d quality: %d rate: %d \n", tmp,
                 element.cam1.mode, element.cam1.quality, element.cam1.rate);
    if (element.cam2.mode)
        sprintf (tmp, "%scam2: mode: %d quality: %d rate: %d \n", tmp,
                 element.cam2.mode, element.cam2.quality, element.cam2.rate);
    sprintf (tmp, "%s\n", tmp);

    if (NULL == output)
        printf ("%s", tmp);
    else
        sprintf (output, "%s", tmp);
}

void
iver_vectormap_print_buoy (iver_vectormap_buoy_t *element_ptr, char *output)
{
    iver_vectormap_buoy_t element = *element_ptr;

    char tmp[1024] = {'\0'};

    sprintf (tmp, "%sBuoy %d: \n", tmp, element.num);
    sprintf (tmp, "%slat: %0.6lf deg\nlng: %0.6lf deg\n", tmp,
             element.latitude * UNITS_RADIAN_TO_DEGREE,
             element.longitude * UNITS_RADIAN_TO_DEGREE);

    if (NULL == output)
        printf ("%s", tmp);
    else
        sprintf (output, "%s", tmp);
}
