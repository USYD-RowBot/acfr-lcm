#ifndef __PERLS_IVER_VECTORMAP_H__
#define __PERLS_IVER_VECTORMAP_H__

#include <glib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup PerlsIverVectormap Vectormap Mission File Parser
 * @brief Code to parse Ocean-Server's Vectormap mission file.
 * @ingroup PerlsIver
 * @include: perls-iver/vectormap.h
 * @author Nicholas Carlevaris-Bianco
 * 
 * @{
 */

/**
 * @brief Depth from surface setpoint
 * @see iver_vectormap_waypoint_t
 */
typedef struct _iver_vectormap_dfs_t iver_vectormap_dfs_t;
struct _iver_vectormap_dfs_t {
    int use;        //!< boolean if used
    double depth;   //!< depth from surface
};

/**
 * @brief height from bottom setpoint
 * @see iver_vectormap_waypoint_t
 */
typedef struct _iver_vectormap_hfb_t iver_vectormap_hfb_t;
struct _iver_vectormap_hfb_t {
    int use;        //!< boolean if used
    double height;  //!< height from bottom
};

/**
 * @brief undulate setpoint
 * @see iver_vectormap_waypoint_t
 */
typedef struct _iver_vectormap_undulate_t iver_vectormap_undulate_t;
struct _iver_vectormap_undulate_t {
    int use;            //!< boolean if used
    double depth1;      //!< depth setpoint 1
    double depth2;      //!< depth setpoint 2
    double dive_angle;  //!< dive angle 
};

/**
 * @brief camera setpoint
 * @see iver_vectormap_waypoint_t
 */
typedef struct _iver_vectormap_camera_t iver_vectormap_camera_t;
struct _iver_vectormap_camera_t {
    int mode;       //!< 0 = off, 1 = video, 2 = stills
    int quality;    //!< 0 = not used, 1 = high, 2 = med, 3 = low
    int rate;       //!< rate in 500 ms increments 500 to 20000 (could treat as exposure in us)
};

/**
 * @brief sidescan sonar setpoint
 * @see iver_vectormap_waypoint_t
 */
typedef struct _iver_vectormap_sonar_t iver_vectormap_sonar_t;
struct _iver_vectormap_sonar_t 
{
    int use;    //!< boolean if used
    int type;   //!< 2=oceanserver, 1=Imagenex
    int gain;   //!< sonar gain
    int range;  //!< sonar range    
    char chan;  //!< sonar channel
    char freq;  //!< sonar frequency
};

/**
 * @brief waypoint structure 
 * @details waypoint structure loaded from vectormap file
 *
 * @see iver_vectormap_parse_mission()
 * @see iver_vectormap_print_waypoint()
 */
typedef struct _iver_vectormap_waypoint_t iver_vectormap_waypoint_t;
struct _iver_vectormap_waypoint_t {
    int num;                            //!< waypoint number in mission
    double latitude, longitude;         //!< latitude and longitude in rads 
    double heading;                     //!< heading to this waypoint in rads
    double distance;                    //!< distance to this waypoint 
    double time;                        //!< mission time
    double park_time;                   //!< time to park
    double speed;                       //!< speed for this waypont
    char raw_cmd[256];                  //!< the raw command string
    iver_vectormap_dfs_t dfs;               //!< depth from surface set point
    iver_vectormap_hfb_t hfb;               //!< height from bottom set point
    iver_vectormap_undulate_t undulate;     //!< undulate setpoint
    iver_vectormap_camera_t cam1;           //!< camera 1 setpoint
    iver_vectormap_camera_t cam2;           //!< camera 2 setpoint
    int camera_overlay;                 //!< 0=none, 1=depth, 2=position, 3=both
    iver_vectormap_sonar_t sonar;           //!< sonor setpoint
};


/**
 * @brief buoy structure 
 * @details buoy structure loaded from vectormap file
 * 
 */
typedef struct _iver_vectormap_buoy_t iver_vectormap_buoy_t;
struct _iver_vectormap_buoy_t {
    int num;
    double latitude;
    double longitude;
};

/**
 * @brief parse a vectormap mission file for waypoints
 * @details parses a data from the waypoint section of the mission file and
 * ignores everything else.
 *
 * @return a GList of iver_vectormap_waypoint_t structures
 *
 * @see iver_vectormap_waypoint_t
 */
GList *
iver_vectormap_parse_mission (const char *filename);


/**
 * @brief parse and fix a vectormap mission file for waypoints
 * @details parses a data from the waypoint section of the mission file and
 * ignores everything else. Fixes mission by turning cameras off
 *
 * @return a GList of iver_vectormap_waypoint_t structures
 * @param filename the input filename
 * @param filename_out the output filename of the corrected file
 *
 * @see iver_vectormap_waypoint_t
 */
GList *
iver_vectormap_parse_n_fix_mission (const char *filename, const char *filename_out);

/**
 * @brief prints the contents of a iver_vectormap_waypoint_t for debugging
 *
 * @see iver_vectormap_waypoint_t
 */
void 
iver_vectormap_print_waypoint (iver_vectormap_waypoint_t *element_ptr, char *output);

/**
 * @brief frees a GList of waypoints created by iver_vectormap_parse_mission()
 *
 * @see iver_vectormap_waypoint_t
 * @see iver_vectormap_parse_mission()
 */
void
iver_vectormap_free_waypoints (GList *waypoints);


/**
 * @brief parse a vectormap mission file for buoys
 * @details parses a data from the buoy section of the mission file and
 * ignores everything else.
 *
 * @return a GList of iver_vectormap_buoy_t structures
 *
 * @see iver_vectormap_buoy_t
 */
GList *
iver_vectormap_parse_buoy (const char *filename);

/**
 * @brief frees a GList of buoys created by iver_vectormap_parse_buoy()
 *
 * @see iver_vectormap_buoy_t
 * @see iver_vectormap_parse_buoy()
 */
void
iver_vectormap_free_buoys (GList *buoys);



void
iver_vectormap_print_buoy (iver_vectormap_buoy_t *element_ptr, char *output);


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif //__PERLS_IVER_VECTORMAP_H__
