#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

// external linking req'd
#include <glib.h>
#include <math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_position_t.h"
#include "perls-lcmtypes/perllcm_segway_navigator_t.h"
#include "perls-lcmtypes/perllcm_velodyne_laser_return_collection_t.h"
#include "perls-lcmtypes/senlcm_velodyne_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/units.h"

#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "perls-sensors/velodyne.h"

#define MAX_NUMBER_OF_OBSERVATIONS 5000000 //approximately 1.5 min of data
#define VELODYNE_NUM_LASERS 32

typedef struct observations_ observations_t;
struct observations_
{
    uint8_t laser_id; //0 to 63
    uint8_t laser_reflectivity; //0 to 255
    // map cell size: (10cm X 10cm X 10cm)
    char map_cell_id[16];
};

typedef struct map_cell_info_ map_cell_info_t;
struct map_cell_info_
{
    uint16_t index;
    uint32_t sum_reflectivity;
    //average reflectivity of the cell = sum_reflectivity / index
};

typedef GHashTable  Map_t;

/**
 * Dummy function to perform operation on each element of hash table
 */
void
get_average_reflectivity (gpointer key, gpointer value, gpointer user_data)
{
    map_cell_info_t *temp_info = value;
    char *temp_key = key;
    printf ("Cell id %s, Sum Reflectivity %d, Index %d \n",
            temp_key, temp_info->sum_reflectivity, temp_info->index);
}

/**
 * This functions converts the pose of vehicle to the velodyne_state_t data type
 * It calculates the sensor pose and velocities from the vehicle pose and its
 * body frame velocities.
 */
void
get_collector_state (perllcm_segway_navigator_t *pose, perllcm_position_t *state, double x_vs[])
{
    state->utime = pose->utime;

    // find sensor pose in local/world frame
    double x_lr[6];
    x_lr[0] = (-1 == pose->index.x) ? 0 : pose->mu[pose->index.x];
    x_lr[1] = (-1 == pose->index.y) ? 0 : pose->mu[pose->index.y];
    x_lr[2] = (-1 == pose->index.z) ? 0 : pose->mu[pose->index.z];
    x_lr[3] = (-1 == pose->index.r) ? 0 : pose->mu[pose->index.r];
    x_lr[4] = (-1 == pose->index.p) ? 0 : pose->mu[pose->index.p];
    x_lr[5] = (-1 == pose->index.h) ? 0 : pose->mu[pose->index.h];

    ssc_head2tail (state->xyzrph, NULL, x_lr, x_vs);

    // move velocities and rates into sensor frame
    double O_sv[9];
    double rph_vs[3] = {x_vs[3], x_vs[4], x_vs[5]};
    so3_rotxyz (O_sv, rph_vs);
    double t_vs[3] = {x_vs[0], x_vs[1], x_vs[2]};

    //uvw_sensor = O_sv * [uvw - skewsym(t_vs)*abc];
    double uvw[3];
    uvw[0] = (-1 == pose->index.u) ? 0 : pose->mu[pose->index.u];
    uvw[1] = (-1 == pose->index.v) ? 0 : pose->mu[pose->index.v];
    uvw[2] = (-1 == pose->index.w) ? 0 : pose->mu[pose->index.w];

    double abc[3];
    abc[0] = (-1 == pose->index.a) ? 0 : pose->mu[pose->index.a];
    abc[1] = (-1 == pose->index.b) ? 0 : pose->mu[pose->index.b];
    abc[2] = (-1 == pose->index.c) ? 0 : pose->mu[pose->index.c];

    double skewsym[9] = { 0,       -t_vs[2],  t_vs[1],
                          t_vs[2],  0,       -t_vs[0],
                          -t_vs[1],  t_vs[0],  0
                        };
    GSLU_VECTOR_VIEW (uvw_sensor,3, {0});
    gsl_vector_view abc_v = gsl_vector_view_array (abc, 3);
    gsl_vector_view uvw_v = gsl_vector_view_array (uvw, 3);
    gsl_matrix_view O_sv_v = gsl_matrix_view_array (O_sv, 3, 3);
    gsl_matrix_view skewsym_v = gsl_matrix_view_array (skewsym, 3, 3);
    //skewsym(t_vs)*abc;
    gslu_blas_mv (&uvw_sensor.vector, &skewsym_v.matrix, &abc_v.vector);
    //[uvw - skewsym(t_vs)*abc]
    gsl_vector_sub (&uvw_v.vector, &uvw_sensor.vector);
    //uvw_sensor = O_sv * [uvw - skewsym(t_vs)*abc];
    gslu_blas_mv (&uvw_sensor.vector, &O_sv_v.matrix, &uvw_v.vector);

    // sensor frame rates
    GSLU_VECTOR_VIEW (abc_sensor, 3, {0});
    gsl_matrix_view O_vs_v = gsl_matrix_view_array (O_sv, 3, 3);
    gsl_matrix_transpose (&O_vs_v.matrix);
    gslu_blas_mv (&abc_sensor.vector, &O_vs_v.matrix, &abc_v.vector);

    //rotate velodyne body velocities into local frame
    double R_sl[9];
    so3_rotxyz (R_sl, &(state->xyzrph[3])); //state.rph = rph_ls
    gsl_matrix_view R_sl_v = gsl_matrix_view_array (R_sl, 3, 3);
    GSLU_VECTOR_VIEW (xyz_dot_sensor,3, {0});
    gslu_blas_mv (&xyz_dot_sensor.vector, &R_sl_v.matrix, &uvw_sensor.vector);
    memcpy (&(state->xyzrph_dot[0]), xyz_dot_sensor.vector.data, 3*sizeof (double));

    // set euler rates
    so3_body2euler (abc_sensor.vector.data, &(state->xyzrph[3]), &(state->xyzrph_dot[3]), NULL);
}

/**
 * Find the coordinates of points in this collection wrt the first laser pose
 */
void
get_collection_in_local_frame (perllcm_position_t state, double first_laser_pose[],
                               perllcm_velodyne_laser_return_collection_t *new_returns)
{
    // data packet rate 1.8 Khz and pose rate max is about 0.1 Khz
    // so for each data packet we only need to find the first laser to current pose once
    double x_flp_cp[6] = {0}; //first laser pose to current pose
    double current_pose[6] = {state.xyzrph[0], state.xyzrph[1], state.xyzrph[2],
                              state.xyzrph[3], state.xyzrph[4], state.xyzrph[5]
                             };
    ssc_tail2tail (x_flp_cp, NULL, first_laser_pose, current_pose);

    // compensate the entire data packet to at once based on timestamp of first laser in packet
    // Copied this code from velodyne.c motion_compensate() function
    // This simply project all the points in the current collector to the
    // reference frame of first laser pose.

    double dt = (new_returns->utime - state.utime)/1e6;
    double x_flp_clp[6] = {0}; // pose at this laser firing (current pose + velocities*dt)
    x_flp_clp[0] = x_flp_cp[0] + dt*state.xyzrph_dot[0];
    x_flp_clp[1] = x_flp_cp[1] + dt*state.xyzrph_dot[1];
    x_flp_clp[2] = x_flp_cp[2] + dt*state.xyzrph_dot[2];
    x_flp_clp[3] = x_flp_cp[3] + dt*state.xyzrph_dot[3];
    x_flp_clp[4] = x_flp_cp[4] + dt*state.xyzrph_dot[4];
    x_flp_clp[5] = x_flp_cp[5] + dt*state.xyzrph_dot[5];

    double R_clp_flp[9];
    double rph_flp_clp[3] = {x_flp_clp[3], x_flp_clp[4], x_flp_clp[5]};
    so3_rotxyz (R_clp_flp, rph_flp_clp);
    gsl_matrix_view R_clp_flp_v = gsl_matrix_view_array (R_clp_flp, 3, 3);

    // stack all laser returns into matrix
    gsl_matrix *xyz_stacked = gsl_matrix_calloc (3, new_returns->num_lr);
    gsl_matrix *xyz_stacked_r = gsl_matrix_calloc (3, new_returns->num_lr);
    for (int i=0 ; i<(new_returns->num_lr) ; i++)
    {
        perllcm_velodyne_laser_return_t *lr = &(new_returns->laser_returns[i]);
        gsl_matrix_set (xyz_stacked, 0, i, lr->xyz[0]);
        gsl_matrix_set (xyz_stacked, 1, i, lr->xyz[1]);
        gsl_matrix_set (xyz_stacked, 2, i, lr->xyz[2]);
    }
    // rotate laser returns
    gslu_blas_mm (xyz_stacked_r, &R_clp_flp_v.matrix, xyz_stacked);

    // translate and pull data out of matrix
    for (int i=0 ; i<(new_returns->num_lr) ; i++)
    {
        perllcm_velodyne_laser_return_t *lr = &(new_returns->laser_returns[i]);
        lr->xyz[0] = gsl_matrix_get (xyz_stacked_r, 0, i) + x_flp_clp[0];
        lr->xyz[1] = gsl_matrix_get (xyz_stacked_r, 1, i) + x_flp_clp[1];
        lr->xyz[2] = gsl_matrix_get (xyz_stacked_r, 2, i) + x_flp_clp[2];
    }

    gsl_matrix_free (xyz_stacked);
    gsl_matrix_free (xyz_stacked_r);
}

int
main (int argc, char *argv[])
{
    if (argc < 2)
    {
        printf ("Usage: %s <lcm log file> <calib_file>\n", argv[0]);
        return 0;
    }


    //parse command line args
    const char *lcmlog_file = argv[1];
    const char *calib_file = argv[2];

    velodyne_calib_t *calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_32E, calib_file);
    printf ("Calib created\n");

    //create event log
    lcm_eventlog_t *evLogRead = lcm_eventlog_create (lcmlog_file, "r");
    printf ("Event log created\n");

    //Hash table for holding the intensity values of each laser falling
    //into a map cell
    Map_t *map_cell_table = g_hash_table_new (g_str_hash, g_str_equal);

    //Array for holding all the observations (Use bot_ptr_circular)
    GPtrArray *arr_observations = g_ptr_array_new (); //(MAX_NUMBER_OF_OBSERVATIONS); //approximately 1.5 min of data

    lcm_eventlog_event_t *currEvent = NULL;
    senlcm_velodyne_t v;
    perllcm_segway_navigator_t p;

    fasttrig_init ();

    // load from config file
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);

    //get the pose of sensor with respect to vehicle
    double x_vs[6];
    if (6 == bot_param_get_double_array (param, "sensors.velodyne.x_vs", x_vs, 6) )
    {
        x_vs[3] =  x_vs[3] * UNITS_DEGREE_TO_RADIAN;
        x_vs[4] =  x_vs[4] * UNITS_DEGREE_TO_RADIAN;
        x_vs[5] =  x_vs[5] * UNITS_DEGREE_TO_RADIAN;
    }

    //Get the collector state
    perllcm_position_t state;

    //get the first pose of the vehicle
    while (1)
    {
        currEvent = lcm_eventlog_read_next_event (evLogRead);
        if ((0 == strcmp (currEvent->channel, "SEG_NAVIGATOR")))
        {
            //get the current pose of vehicle
            memset (&p, 0, sizeof (p));
            perllcm_segway_navigator_t_decode (currEvent->data, 0, currEvent->datalen, &p);
            get_collector_state (&p, &state, x_vs);
            break;
        }
    }

    //Set first laser pose
    double first_laser_pose[6];
    first_laser_pose[0] = state.xyzrph[0];
    first_laser_pose[1] = state.xyzrph[1];
    first_laser_pose[2] = state.xyzrph[2];
    first_laser_pose[3] = state.xyzrph[3];
    first_laser_pose[4] = state.xyzrph[4];
    first_laser_pose[5] = state.xyzrph[5];

    //For debugging
    int hist_refc[256];
    for (int i = 0; i < 256; i++)
        hist_refc[i] = 0;

    //Now loop over all the velodyne data and subsequent poses
    int count = 0, num_map_cells = 0;
    perllcm_velodyne_laser_return_collection_t *lrc = NULL;
    while (1)
    {
        currEvent = lcm_eventlog_read_next_event (evLogRead);
        if (!currEvent)
        {
            printf ("Finished reading the log file\n");
            break;
        }
        else
        {
            //check the channel name
            if ((0==strcmp (currEvent->channel, "SEG_VELODYNE")))
            {
                //if its velodyne
                //get the velodyne data
                memset (&v, 0, sizeof (v));

                senlcm_velodyne_t_decode (currEvent->data, 0, currEvent->datalen, &v);

                //reconstruct the coordinates
                //Decode the collection of laser returns
                if (v.datalen != VELODYNE_DATA_PACKET_LEN)
                {
                    //printf ("Error: v->datalen = %d\n", v.datalen);
                    //printf ("Error: currEvent->datalen = %d\n", currEvent->datalen);
                    continue;
                }
                else
                    lrc = velodyne_decode_data_packet (calib, v.data, v.datalen, v.utime);

                //Get state for this collection
                perllcm_position_t curr_state;
                get_collector_state (&p, &curr_state, x_vs);

                // Find the coordinates of points in this collection wrt the first laser pose
                get_collection_in_local_frame (curr_state, first_laser_pose, lrc);

                //For each sample in this collection
                for (unsigned int s = 0; s < lrc->num_lr; s++)
                {
                    perllcm_velodyne_laser_return_t *vsample = &(lrc->laser_returns[s]);

                    //coordinates in local/common reference frame
                    double local_xyz[3] = { vsample->xyz[0], vsample->xyz[1], vsample->xyz[2] };

                    //Convert intensity to [0, 255] integer range
                    //double temp_intensity = vsample->intensity*255.0;
                    //uint8_t intensity = (uint8_t)(temp_intensity + 0.5);
                    uint8_t intensity = vsample->intensity;
                    if (intensity < 0 || intensity > 255)
                        printf ("Error: Intensity = %d\n", vsample->intensity);
                    hist_refc[intensity]++;

                    //check in which voxel this 3D point (local_xyz) is located
                    //voxel size : 10cm X 10cm X 10cm
                    int x = round (local_xyz[0]*100/10);
                    int y = round (local_xyz[1]*100/10);
                    int z = round (local_xyz[2]*100/10);

                    //generate key (Always allocate memory using malloc)
                    //memory on stack is deleted once the control gets
                    //out of this loop
                    char *key = malloc (32);
                    memset (key, 0, 32);

                    sprintf (key, "%d@%d@%d", x, y, z);
                    //printf("key = %s\n", key); fflush(stdout);

                    //Check if there is any info in this key/cell
                    map_cell_info_t *curr_map_cell_info;
                    gboolean isPresent = g_hash_table_lookup_extended (map_cell_table, key, NULL, (void **)&curr_map_cell_info);
                    if(isPresent)  //this is an old cell
                    {
                        //update it with the new laser info
                        curr_map_cell_info->index++;
                        curr_map_cell_info->sum_reflectivity += intensity;
                    }
                    else  //this is a new cell
                    {
                        //printf("New Cell id = %s\n", key);
                        map_cell_info_t *new_map_cell_info = (map_cell_info_t *) calloc (1,sizeof (map_cell_info_t));
                        if (new_map_cell_info == NULL)
                        {
                            printf ("Error in malloc(): new_map_cell_info, Num Observations = %d\n", count);
                            exit (EXIT_FAILURE);
                        }
                        new_map_cell_info->index = 1;
                        new_map_cell_info->sum_reflectivity = intensity;
                        g_hash_table_insert (map_cell_table, key, new_map_cell_info);
                        num_map_cells++;
                    }
                    //Add this observation to the vector of observations
                    observations_t *curr_observation = malloc (sizeof (*curr_observation));
                    if (!curr_observation)
                    {
                        printf ("Error in malloc(): curr_observation, Num Observations = %d\n", count);
                        exit (EXIT_FAILURE);
                    }
                    curr_observation->laser_id = (uint8_t)vsample->physical;
                    curr_observation->laser_reflectivity = (uint8_t)intensity;
                    sprintf (curr_observation->map_cell_id, "%s", key);
                    //save the current sample with the bin id
                    g_ptr_array_add (arr_observations, curr_observation);
                    count++;
                }

                if (count > MAX_NUMBER_OF_OBSERVATIONS-200)  //observation array is full
                {
                    printf("Observation buffer full\n");
                    break;
                }
            }
            else if ((0==strcmp (currEvent->channel, "SEG_NAVIGATOR")))
            {
                //get the current pose of vehicle
                memset (&p, 0, sizeof (p));
                perllcm_segway_navigator_t_decode (currEvent->data, 0, currEvent->datalen, &p);
            }
        }

        //free the data
        if (currEvent != NULL)
        {
            lcm_eventlog_free_event (currEvent);
            currEvent = NULL;
        }
        //Free the laser collection
        if (lrc != NULL)
        {
            velodyne_free_laser_return_collection (lrc);
            lrc = NULL;
        }
    }

    //Debugging Print the histogram
    for (int i = 0; i < 256; i++)
        printf ("refc = %d, num = %d\n", i, hist_refc[i]);

    return 0;
    //Calculate the calibrated intensity for each laser beam

    //Initialize a [VELODYNE_NUM_LASERSx256] matrix to hold the calibrated intensities (0 to 255)
    //for each laser beam (0 to VELODYNE_NUM_LASERS - 1)
    int calibrated_intensity[VELODYNE_NUM_LASERS][256];
    int temp_sum[VELODYNE_NUM_LASERS][256]; //matrix to hold the running sum
    int temp_num[VELODYNE_NUM_LASERS][256]; //matrix to hold the number of laser contributing to the
    //running sum "temp_sum"

    //initialize all the matrices to zero
    for (int i = 0; i < VELODYNE_NUM_LASERS; i++)
    {
        for (int j = 0; j < 256; j++)
        {
            calibrated_intensity[i][j] = 0;
            temp_sum[i][j] = 0;
            temp_num[i][j] = 0;
        }
    }

    char *intensityCalibFile = (char *) malloc (64);
    if (intensityCalibFile == NULL)
    {
        printf ("Error in malloc(): v, Num Observations = %d\n", count);
        exit (EXIT_FAILURE);
    }

    //mapping function file
    sprintf (intensityCalibFile, "intensity_calibration.conf");

    FILE *fptr = fopen (intensityCalibFile, "w");
    printf ("File opened\n");

    //Loop over all the observations
    uint8_t J, A;
    int avg = 0;

    printf ("Observations = %d\n", arr_observations->len);
    for (int i = 0; i < arr_observations->len; i++)
    {
        observations_t *curr_observation = (observations_t *) g_ptr_array_index (arr_observations, i);
        J = curr_observation->laser_id;
        A = curr_observation->laser_reflectivity;

        //get the average intensity of map cell on which this laser strikes
        map_cell_info_t *curr_map_cell_info;
        gboolean isPresent = g_hash_table_lookup_extended (map_cell_table, curr_observation->map_cell_id, NULL, (void **)&curr_map_cell_info);

        if (isPresent)
            avg = curr_map_cell_info->sum_reflectivity / curr_map_cell_info->index;

        //printf ("Map cell = %s, Laser id = %d, Observation = %d, Average Intensity of Map cell = %d\n", curr_observation->map_cell_id, J, A, avg);
        //sleep(1);
        //Calibrated output when beam "J" observes intensity "A" is the conditional expectation of all
        //other beams' intensity readings for map cells where beam J observed intensity A
        temp_sum[J][A] += avg;
        temp_num[J][A]++;
    }

    printf ("Sum calculated\n");
    //Calculate the calibrated intensity
    for (int i = 0; i < VELODYNE_NUM_LASERS; i++)
    {
        for (int j = 0; j < 256; j++)
        {
            printf ("temp_sum[%d][%d] = %d \t", i, j, temp_sum[i][j]);
            printf ("temp_num[%d][%d] = %d \n", i, j, temp_num[i][j]);
            if (temp_num[i][j] > 0)
            {
                if (calibrated_intensity[i][j] > 0)
                    calibrated_intensity[i][j] = (calibrated_intensity[i][j] + round(temp_sum[i][j] / temp_num[i][j]))/2;
                else
                    calibrated_intensity[i][j] = round(temp_sum[i][j] / temp_num[i][j]);
            }
            //save it to a file
            fprintf (fptr, "%d ", calibrated_intensity[i][j]);
        }
        fprintf (fptr, "\n");
    }

    fflush (fptr);
    fclose (fptr);

    printf ("Observations = %d, Number of map cells = %d\n", count, num_map_cells);
    exit (EXIT_SUCCESS);
}
