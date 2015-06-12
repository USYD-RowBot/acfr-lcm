#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h> /* needed for PRId64 macros */
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "perls-common/error.h"
#include "perls-math/gsl_util.h"

#include "highgui_util.h"

/* This data will be passed to correspondence callbacks.  Should be hidden from user */
typedef struct vis_hgu_manual_corresp_data_t
{
    const IplImage *img1;
    const IplImage *img2;
    char *img1Name;
    char *img2Name;
    int numCorrespSoFar;
    gsl_matrix *pointsSoFar1;
    gsl_matrix *pointsSoFar2;
    const IplImage *currentImg;
} vis_hgu_manual_corresp_data_t;

/* The common mouse callback for two-view feature correspondence */
static void
_vis_hgu_manual_corres_mouse_callback (int event,
                                       int x,
                                       int y,
                                       int flags,
                                       void* user)
{
    vis_hgu_manual_corresp_data_t *data = (vis_hgu_manual_corresp_data_t *) user;
    int width = data->currentImg->width;
    int height = data->currentImg->height;
    IplImage *imgClone = cvCloneImage (data->currentImg);
    gsl_matrix *currentPoints = (data->currentImg == data->img1) ? data->pointsSoFar1 : data->pointsSoFar2;

    /* draw all the current points */
    for (int i=0; i<currentPoints->size2; i++)
    {
        cvCircle (imgClone, cvPoint (gsl_matrix_get (currentPoints, 0, i), gsl_matrix_get (currentPoints, 1, i)),
                  3, cvScalar (0, 255, 0, 0), 1, CV_AA, 0);
    }

    switch (event)
    {
    case CV_EVENT_MOUSEMOVE:
        /* Draw crosshair */
        cvLine (imgClone, cvPoint (0, y), cvPoint (width, y), cvScalar (0, 255, 0, 0), 1, 8, 0);
        cvLine (imgClone, cvPoint (x, 0), cvPoint (x, height), cvScalar (0, 255, 0, 0), 1, 8, 0);
        break;
    case CV_EVENT_LBUTTONDOWN:
        cvCircle (imgClone, cvPoint (x, y),
                  3, cvScalar (0, 255, 0, 0), 1, CV_AA, 0);
        gsl_matrix_set (currentPoints, 0, data->numCorrespSoFar, x);
        gsl_matrix_set (currentPoints, 1, data->numCorrespSoFar, y);

        /* if we clicked point in image 2, then we've just completed a correspondence */
        if (data->currentImg == data->img2)
            data->numCorrespSoFar++;

        break;
    }

    if (data->currentImg == data->img1)
        cvShowImage (data->img1Name, imgClone);
    else
        cvShowImage (data->img2Name, imgClone);

    /* set the next image, if appropriate */
    if (event == CV_EVENT_LBUTTONDOWN)
        data->currentImg = data->currentImg == data->img1 ? data->img2 : data->img1;

    cvReleaseImage (&imgClone);
}

/* callback for image 1 */
static void
_vis_hgu_manual_corres_mouse_callback1 (int event,
                                        int x,
                                        int y,
                                        int flags,
                                        void* user)
{
    vis_hgu_manual_corresp_data_t *data = (vis_hgu_manual_corresp_data_t *) user;
    if (data->currentImg != data->img1)
        return;
    _vis_hgu_manual_corres_mouse_callback (event, x, y, flags, user);
}

/* callback for image 2 */
static void
_vis_hgu_manual_corres_mouse_callback2 (int event,
                                        int x,
                                        int y,
                                        int flags,
                                        void* user)
{
    vis_hgu_manual_corresp_data_t *data = (vis_hgu_manual_corresp_data_t *) user;
    if (data->currentImg != data->img2)
        return;
    _vis_hgu_manual_corres_mouse_callback (event, x, y, flags, user);
}


void
vis_hgu_manual_corresp (const IplImage *i1,
                        const IplImage *i2,
                        gsl_matrix *uv1,
                        gsl_matrix *uv2,
                        int n)
{
    assert (uv1->size1 == 2 && uv2->size1 == 2 && uv1->size2 == uv2->size2 && uv1->size2 == n);

    char *i1Name = strdup("Image 1");
    char *i2Name = strdup("Image 2");

    vis_hgu_manual_corresp_data_t *data = calloc (1, sizeof (*data));

    /* Convert the image to color here (NOT in the callback - it's much faster this way) */
    IplImage *i1Clone = cvCreateImage (cvGetSize (i1), IPL_DEPTH_8U, 3);
    IplImage *i2Clone = cvCreateImage (cvGetSize (i2), IPL_DEPTH_8U, 3);
    cvCvtColor (i1, i1Clone, CV_GRAY2RGB);
    cvCvtColor (i2, i2Clone, CV_GRAY2RGB);
    data->img1 = i1Clone;
    data->img2 = i2Clone;

    data->img1Name = i1Name;
    data->img2Name = i2Name;
    data->currentImg = data->img1;
    data->pointsSoFar1 = uv1;
    data->pointsSoFar2 = uv2;

    cvNamedWindow (i1Name, CV_WINDOW_AUTOSIZE);
    cvNamedWindow (i2Name, CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback (i1Name, _vis_hgu_manual_corres_mouse_callback1, data);
    cvSetMouseCallback (i2Name, _vis_hgu_manual_corres_mouse_callback2, data);

    cvShowImage (i1Name, data->img1);
    cvShowImage (i2Name, data->img2);

    /* break once n correspondences are clicked or until user closes window */
    while (data->numCorrespSoFar < n && cvGetWindowHandle (i1Name) && cvGetWindowHandle (i2Name))
        cvWaitKey (10);

    /* clean up */
    cvReleaseImage (&i1Clone);
    cvReleaseImage (&i2Clone);
    free (data->img1Name);
    free (data->img2Name);
    free (data);
}
