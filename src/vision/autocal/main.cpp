/**
 * @author Paul Ozog - paulozog@umich.edu
 *
 * @file main.c
 *
 * @brief Contains main() for autocal, an OpenCV-based camera
 * calibration program
 *
 * @detail When specifying the chessboard dimensions, orient the
 * target like a portrait (not a landmark).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "autoCalHelpers.h"

int
main (int argc, char *argv[])
{
    getopt_t *options = getopt_create ();

    if (0 != parseArgs (options, argc, argv)) {
        getopt_do_usage (options, "images");
        return (EXIT_FAILURE);
    }
    else if (getopt_get_bool (options, "help")) {
        getopt_do_usage (options, "images");
        return (EXIT_SUCCESS);
    }

    //Command line options
    int numDesiredFrames = getopt_get_int (options, "num-frames");
    int	boardWidth = getopt_get_int (options, "board-width");
    int	boardHeight = getopt_get_int (options, "board-height");
    int	frameDelay = getopt_get_int (options, "frame-delay");
    int	drawFrames = !(getopt_get_bool (options, "quiet"));
    float displayScale = getopt_get_double (options, "scale");
    float squareSize = getopt_get_double (options, "square-size");
    int numFiles = options->extraargs->len;
    char **fileNames = (char**)(options->extraargs->pdata);
    const char *transOutfile = getopt_get_string (options, "translations");
    const char *rotOutfile = getopt_get_string (options, "rotations");
    const char *intrOutfile = getopt_get_string (options, "intrinsics");
    const char *distOutfile = getopt_get_string (options, "dist-coeffs");
    const char *residOutfile = getopt_get_string (options, "residuals");
    const char *indexOutfilename = getopt_get_string (options, "index-outfile");

    if (numFiles < numDesiredFrames &&
        numDesiredFrames != 0) {
        printf ("You did not provide enough image files\n");
        getopt_do_usage (options, "images");
        return (EXIT_FAILURE);
    }

    int numSquares = boardWidth * boardHeight;
    CvSize boardSize = cvSize (boardWidth, boardHeight);
    CvMat *imagePoints = cvCreateMat (numFiles*numSquares, 2, CV_32FC1);
    CvMat *objectPoints = cvCreateMat (numFiles*numSquares, 3, CV_32FC1);
    CvMat *pointCounts = cvCreateMat (numFiles, 1, CV_32SC1);
    CvMat *intrinsicMatrix  = cvCreateMat (3, 3, CV_32FC1);
    CvMat *distortionCoeffs = cvCreateMat (5, 1, CV_32FC1);
    CvPoint2D32f *corners = (CvPoint2D32f *)malloc (numSquares * sizeof(CvPoint2D32f));
    int successes = 0;
    int step = 0;
    int imageIndex = 0;
    IplImage *imageGray = NULL;
    IplImage *imageRgb = NULL;
    IplImage *imageRgbSmall = NULL;
    CvSize imageSize;
    int cornerCount, found, i, j;
    //int keyPress;
    float reprojectionError;
    FILE *indexOutfile;

    if (drawFrames)
        cvNamedWindow ("autocal", CV_WINDOW_AUTOSIZE);

    indexOutfile = fopen (indexOutfilename, "w");

    if (!indexOutfile) {
        printf ("Could not open %s for writing\n", indexOutfilename);
        return (EXIT_FAILURE);
    }

    printInfo ("Locating corners in images...\n");

    //We assume all images are the same size.
    imageSize = cvGetSize (cvLoadImage (fileNames[0], CV_LOAD_IMAGE_GRAYSCALE));

    // Capture Corner views loop until we've got numDesiredFrames
    // succesful captures (all corners on the board are found)
    while ((successes < numDesiredFrames && imageIndex < numFiles) ||
           (numDesiredFrames == 0 && imageIndex < numFiles)) {

        //load the current image
        imageGray = cvLoadImage (fileNames[imageIndex],
                                 CV_LOAD_IMAGE_GRAYSCALE);
        imageRgb  = cvCreateImage (cvGetSize (imageGray), IPL_DEPTH_8U, 3);
        cvCvtColor (imageGray, imageRgb, CV_GRAY2RGB);

        // Find chessboard corners:
        found = cvFindChessboardCorners (imageGray, boardSize, corners,
                                         &cornerCount,
                                         CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        // Get subpixel accuracy on those corners
        cvFindCornerSubPix (imageGray, corners, cornerCount, cvSize (7, 7),
                            cvSize (-1, -1),
                            cvTermCriteria (CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, .1));

        // Draw it
        if (drawFrames) {
            cvDrawChessboardCorners (imageRgb, boardSize, corners, cornerCount, found);
            imageRgbSmall = cvCreateImage (cvSize ((int)(imageRgb->width*displayScale),
                                                   (int)(imageRgb->height*displayScale)),
                                           imageRgb->depth, imageRgb->nChannels);
            cvResize (imageRgb, imageRgbSmall, CV_INTER_LINEAR);
            cvShowImage ("autocal", imageRgbSmall);
        }

        // If we got a good board, add it to our data
        if (found) {
            step = successes*numSquares;

            //corners has points placed "row by row, left to right in every row"
            for (i=step, j=0; j < numSquares; ++i, ++j) {
                CV_MAT_ELEM (*imagePoints, float, i, 0) = corners[j].x;
                CV_MAT_ELEM (*imagePoints, float, i, 1) = corners[j].y;
                CV_MAT_ELEM (*objectPoints, float, i, 0) = j/boardWidth * squareSize;
                CV_MAT_ELEM (*objectPoints, float, i, 1) = j%boardWidth * squareSize;
                CV_MAT_ELEM (*objectPoints, float, i, 2) = 0.0f;
            }

            CV_MAT_ELEM (*pointCounts, int, successes, 0) = numSquares;

            fprintf (indexOutfile, "%d,%s\n", imageIndex, fileNames[imageIndex]);
            successes++;
        }

        // Handle pause/unpause and ESC
        if (drawFrames)
            cvWaitKey (frameDelay);

        imageIndex += 1;

        fflush (stdout);

        // Free the images
        cvReleaseImage (&imageGray);
        cvReleaseImage (&imageRgb);
        cvReleaseImage (&imageRgbSmall);

        printf ("Successful Frames: [%d] - Total Frames: [%d]\r", 
                successes, imageIndex);

    } // End collection while loop

    if (successes < numDesiredFrames) {
        printf ("\nERROR: Could not find enough chessboards in the given images.\n");
        printf ("Try adding more images or decreasing the number of desired frames\n");
        return (EXIT_FAILURE);
    }

    sayDone ();

    // Allocate point matrices according to how many chessboards found
    CvMat *objectPoints2  = cvCreateMat (successes*numSquares, 3, CV_32FC1);
    CvMat *imagePoints2   = cvCreateMat (successes*numSquares, 2, CV_32FC1);
    CvMat *pointCounts2   = cvCreateMat (successes, 1, CV_32SC1);
    CvMat *rotVector      = cvCreateMat (successes, 3, CV_32F);
    CvMat *transVector    = cvCreateMat (successes, 3, CV_32FC1);
    CvMat *rotVectorT     = cvCreateMat (3, successes, CV_32F); //transpose
    CvMat *transVectorT   = cvCreateMat (3, successes, CV_32FC1); //transpose
    CvMat *calibResiduals = cvCreateMat (1, successes, CV_32F);

    // Transfer the detected points into the correct size matrices
    for(i = 0; i < successes*numSquares; ++i){
        CV_MAT_ELEM (*imagePoints2, float, i, 0) = CV_MAT_ELEM (*imagePoints, float, i, 0);
        CV_MAT_ELEM (*imagePoints2, float, i, 1) = CV_MAT_ELEM (*imagePoints, float, i, 1);
        CV_MAT_ELEM (*objectPoints2, float, i, 0) = CV_MAT_ELEM (*objectPoints, float, i, 0);
        CV_MAT_ELEM (*objectPoints2, float, i, 1) = CV_MAT_ELEM (*objectPoints, float, i, 1);
        CV_MAT_ELEM (*objectPoints2, float, i, 2) = CV_MAT_ELEM (*objectPoints, float, i, 2);
    }

    for (i=0; i < successes; ++i)
        CV_MAT_ELEM (*pointCounts2, int, i, 0) = CV_MAT_ELEM (*pointCounts, int, i, 0);

    cvReleaseMat (&objectPoints);
    cvReleaseMat (&imagePoints);
    cvReleaseMat (&pointCounts);

    // At this point we have all the chessboard corners we need
    // Initiliazie the intrinsic matrix such that the two focal lengths
    // have a ratio of 1.0

    CV_MAT_ELEM (*intrinsicMatrix, float, 0, 0) = 1.0;
    CV_MAT_ELEM (*intrinsicMatrix, float, 1, 1) = 1.0;

    CvAttrList emptyAttrList;

    printInfo ("Calibrating cameras...");

    cvCalibrateCamera2 (objectPoints2, imagePoints2, pointCounts2,
                        imageSize, intrinsicMatrix, distortionCoeffs,
                        rotVector, transVector, 0);


    reprojectionError = computeReprojectionError (objectPoints2, rotVector, transVector,
                                                  intrinsicMatrix, distortionCoeffs, 
                                                  imagePoints2, pointCounts2, calibResiduals);

    sayDone ();

    printf ("Reprojection error (standard deviation): %f pixels\n", reprojectionError);

    // I want to save the extrinsics as Nx3 matrix, N is number of views
    cvTranspose (transVector, transVectorT);
    cvTranspose (rotVector, rotVectorT);

    // save the intrinsics and distortions to xml
    cvSave (intrOutfile,   intrinsicMatrix,  NULL, NULL,  emptyAttrList);
    cvSave (distOutfile,   distortionCoeffs, NULL, NULL,  emptyAttrList);
    cvSave (transOutfile,  transVectorT,     NULL, NULL,  emptyAttrList);
    cvSave (rotOutfile,    rotVectorT,       NULL, NULL,  emptyAttrList);
    cvSave (residOutfile,  calibResiduals,   NULL, NULL,  emptyAttrList);

    cvReleaseMat (&intrinsicMatrix);
    cvReleaseMat (&distortionCoeffs);
    cvReleaseMat (&transVectorT);
    cvReleaseMat (&transVector);
    cvReleaseMat (&rotVectorT);
    cvReleaseMat (&rotVector);

    fclose (indexOutfile);

    return (EXIT_SUCCESS);

}
