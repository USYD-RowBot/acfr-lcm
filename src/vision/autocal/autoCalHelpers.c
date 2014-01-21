#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <glob.h>
#include <unistd.h>
#include <libgen.h>
#include <math.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-common/getopt.h"

#include "autoCalHelpers.h"

void
printMat (CvMat *A)
{
    int i, j;
    for (i = 0; i < A->rows; i++) {
        printf ("\n"); 
        switch (CV_MAT_DEPTH (A->type)) {
        case CV_32F:
        case CV_64F:
            for (j = 0; j < A->cols; j++)
                printf ("%8.3f ", (float)cvGetReal2D (A, i, j));
            break;
        case CV_8U:
        case CV_16U:
            for (j = 0; j < A->cols; j++)
                printf ("%6d",(int)cvGetReal2D (A, i, j));
            break;
        default:
            break;
        }
    }
    printf ("\n");
}

double
computeReprojectionError (const CvMat *objectPoints,
                          const CvMat *rotVects, const CvMat *transVects,
                          const CvMat *cameraMatrix, const CvMat *distCoeffs,
                          const CvMat *imagePoints, const CvMat *pointCounts,
                          CvMat* perViewErrors)
{
    CvMat* imagePoints2 = cvCreateMat (imagePoints->rows,
                                       imagePoints->cols, imagePoints->type);
    int i, imageCount = rotVects->rows, pointsSoFar = 0;
    double sumSquaredDiff = 0, err, stdev;
    
    for(i = 0; i < imageCount; i++) {

        CvMat objectPointsI, imagePointsI, imagePoints2I;
        int pointCount = pointCounts->data.i[i];
        CvMat rotVect, transVect;

        cvGetRows (objectPoints, &objectPointsI,
                   pointsSoFar, pointsSoFar + pointCount, 1);
        cvGetRows (imagePoints, &imagePointsI,
                   pointsSoFar, pointsSoFar + pointCount, 1);
        cvGetRows (imagePoints2, &imagePoints2I,
                   pointsSoFar, pointsSoFar + pointCount, 1);
        pointsSoFar += pointCount;

        cvGetRow (rotVects, &rotVect, i);
        cvGetRow (transVects, &transVect, i);

        cvProjectPoints2 (&objectPointsI, &rotVect, &transVect,
                          cameraMatrix, distCoeffs, &imagePoints2I,
                          NULL, NULL, NULL, NULL, NULL, 0);

        err = cvNorm (&imagePointsI, &imagePoints2I, CV_L2, NULL);

        if (perViewErrors)
            CV_MAT_ELEM (*perViewErrors, float, 0, i) = err;

        sumSquaredDiff += err * err;
    }

    cvReleaseMat (&imagePoints2);
    stdev = sqrt (sumSquaredDiff / (pointsSoFar * 2));
    
    return stdev;
}

int
parseArgs (getopt_t *opts, int argc, char** argv)
{
    getopt_add_description (opts, "autocal - OpenCV camera calibrator");
    getopt_add_int         (opts, 'N', "num-frames", "50", "Number of desired key frames (use 0 if not known)");
    getopt_add_int         (opts, 'W', "board-width", "12", "Board width, in squares");
    getopt_add_int         (opts, 'H', "board-height", "7", "Board height, in squares");
    getopt_add_int         (opts, 'd', "frame-delay", "25", "Frame delay, in milliseconds.  0 for keypress");
    getopt_add_bool        (opts, 'Q', "quiet", 0, "Don't draw frames if true");
    getopt_add_double      (opts, 's', "scale", "0.7", "Frame scale");
    getopt_add_double      (opts, 'S', "square-size", "0.03", "Square size, in meters");
    getopt_add_string      (opts, '\0', "translations", "ExtrTrans.xml", 
                            "Extrinsic Translation output file");
    getopt_add_string      (opts, '\0', "rotations", "ExtrRotation.xml", 
                            "Extrinsic Rotation output file");
    getopt_add_string      (opts, '\0', "intrinsics", "Intrinsics.xml", 
                            "Intrinsics output file");
    getopt_add_string      (opts, '\0', "residuals", "Residuals.xml", 
                            "Reprojection error residuals output file");
    getopt_add_string      (opts, '\0', "dist-coeffs", "Distortion.xml", 
                            "Distortion coefficients output file");
    getopt_add_string      (opts, '\0', "index-outfile", "indeces.txt", 
                            "Indeces to image name output file");
    getopt_add_help        (opts, NULL);

    return !getopt_parse (opts, argc, argv, 1);
}

void
sayDone()
{  
    printf ("\nDone.\n");
    fflush (stdout);
}

void
printInfo (const char* message)
{  
    printf ("INFO: %s", message);
    fflush (stdout); 
}
