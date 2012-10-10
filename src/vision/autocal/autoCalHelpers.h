/**
 * @author Paul Ozog - paulozog@umich.edu
 *
 * @file autoCalHelpers.h
 *
 * @brief functions to make the autocal's main() easier to read
 */

#ifndef __AUTOCAL_HELPERS_H__
#define __AUTOCAL_HELPERS_H__

#include <opencv/cv.h>

#include "perls-common/getopt.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_FILE_LENGTH 512

/* #ifndef __cplusplus */
/* typedef int bool; */
/* #endif */

/**
 * @brief Wrapper to getopt_* methods
 *
 * @param argc
 * @param argv
 * @param opts : Read args in argv and place in this getopt_t object
 *
 * @return EXIT_SUCCESS for success, EXIT_FAILURE for error
 */
int
parseArgs (getopt_t *opts, int argc, char **argv);

/**
 * @brief Print "Done." to screen
 *
 * @param None
 *
 * @return None
 */
void
sayDone ();

/**
 * @brief Print an INFO string to stdout
 *
 * @param message
 *
 * @return none
 */
void
printInfo (const char* message);


/**
 * @brief Computes the reprojection error for calibration data
 *
 * @param objectPoints The array of object points, 3xN or Nx3
 * 1-channel or 1xN or Nx1 3-channel , where N is the number of points
 * in the view
 *
 * @param rotVects The rotation vectors
 *
 * @param transVects The translation vectors
 *
 * @param cameraMatrix Computed from cvCalibrateCamera2
 *
 * @param distCoeffs Computed from cvCalibrateCamera2
 *
 * @param imagePoints The locations of all the successfully detected
 * feature points
 *
 * @param pointCounts Number of points in each view
 *
 * @param perViewErrors Will be assigned the per-image reprojection
 * error in non-NULL
 *
 * @return L1 reprojection error: in pixels.
 */
double
computeReprojectionError (const CvMat* object_points,
                          const CvMat* rot_vects, const CvMat* trans_vects,
                          const CvMat* camera_matrix, const CvMat* dist_coeffs,
                          const CvMat* image_points, const CvMat* point_counts,
                          CvMat* per_view_errors);

/**
 * @brief Prints a opencv matrix to stdout
 * @param A : some matrix
 * @return None
 */
void
printMat (CvMat *A);

#ifdef __cplusplus
}
#endif

#endif //__AUTOCAL_HELPERS_H__
