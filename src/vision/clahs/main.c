#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-vision/clahs.h"

#define DESCRIPTION                                                     \
    "This function implements Contrast Limited Adaptive Histogram Specification (CLAHS).\n" \
    "\n"                                                                \
    "Notes\n"                                                           \
    "-----\n"                                                           \
    "- 'ntiles' specifies the number of rectangular contextual regions (tiles)\n" \
    "  into which the image is divided. The contrast transform function is\n" \
    "  calculated for each of these regions individually. The optimal number of\n" \
    "  tiles depends on the type of the input image, and it is best determined\n" \
    "  through experimentation.\n"                                      \
    "\n"                                                                \
    "- The 'cliplimit' is a contrast factor that prevents over-saturation of the\n" \
    "  image specifically in homogeneous areas.  These areas are characterized\n" \
    "  by a high peak in the histogram of the particular image tile due to many\n" \
    "  pixels falling inside the same gray level range. Without the clip limit,\n" \
    "  the adaptive histogram equalization technique could produce results that,\n" \
    "  in some cases, are worse than the original image.\n"             \
    "\n"                                                                \
    "- CLAHS can use Uniform, Rayleigh, or Exponential distribution as\n" \
    "  the basis for creating the contrast transform function. The distribution\n" \
    "  that should be used depends on the type of the input image.\n"   \
    "  For example, underwater imagery appears to look more natural when the\n" \
    "  Rayleigh distribution is used.\n"                                \
    "\n"                                                                \
    "Example batch process command from a bash shell:\n"                \
    "for i in *.tif; do ./clahs $i CLAHS-$i; done"


int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    getopt_t *gopt = getopt_create ();

    getopt_add_bool   (gopt, 'h',  "help",      0,          "Show this");
    getopt_add_string (gopt, '\0', "tiles",    "[8,10]",    "Number of tiles, [R,C] specifies tile rows and columns");
    getopt_add_double (gopt, '\0', "cliplimit", "0.0075",   "Normalized ClipLimit, [0,1] higher values yield more contrast");
    getopt_add_int    (gopt, '\0', "bins",     "1024",      "Number of bins used in histogram");
    getopt_add_string (gopt, '\0', "range",     "[0,0]",    "Controls range of min/max output intensity; e.g. [0,255] for depth=8");
    getopt_add_string (gopt, '\0', "dist",      "rayleigh", "Desired histogram shape, {'uniform','exponential','rayleigh'}");
    getopt_add_double (gopt, '\0', "alpha",     "0.4",      "Distribution parameter for exponential and rayliegh");
    getopt_add_bool   (gopt, 'd',  "display",   0,          "Display results in window");
    getopt_add_description (gopt, DESCRIPTION);

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help") || gopt->extraargs->len != 2)
    {
        getopt_do_usage (gopt, "input-file output-file");
        return -1;
    }

    // parse command line args
    vis_clahs_opts_t opts =
    {
        .cliplimit = getopt_get_double (gopt, "cliplimit"),
        .bins = getopt_get_int (gopt, "bins"),
        .alpha = getopt_get_double (gopt, "alpha"),
    };

    if (2 != sscanf (getopt_get_string (gopt, "tiles"), "[%zd,%zd]", &opts.tiles[0], &opts.tiles[1]))
    {
        printf ("can't parse arg: --ntiles\n");
        return -1;
    }
    if (2 != sscanf (getopt_get_string (gopt, "range"), "[%zd,%zd]", &opts.range[0], &opts.range[1]))
    {
        printf ("can't parse arg: --range\n");
        return -1;
    }


    if (0 == strcasecmp (getopt_get_string (gopt, "dist"), "rayleigh"))
        opts.dist = VIS_CLAHS_DIST_RAYLEIGH;
    else if (0 == strcasecmp (getopt_get_string (gopt, "dist"), "exponential"))
        opts.dist = VIS_CLAHS_DIST_EXPONENTIAL;
    else if (0 == strcasecmp (getopt_get_string (gopt, "dist"), "uniform"))
        opts.dist = VIS_CLAHS_DIST_UNIFORM;
    else
    {
        printf ("can't parse arg: --dist\n");
        return -1;
    }

    char *ifile = strdup (gopt->extraargs->pdata[0]);
    char *ofile = strdup (gopt->extraargs->pdata[1]);

    // process image
    IplImage *I = cvLoadImage (ifile, CV_LOAD_IMAGE_UNCHANGED);
    if (I->nChannels > 1)
    {
        printf ("clahs only works on grayscale images\n");
        cvReleaseImage (&I);
        return -1;
    }

    IplImage *J = NULL;
    if (getopt_get_bool (gopt, "display"))
        J = cvCloneImage (I);
    else
        J = I;

    int ret = vis_clahs (J->imageData, J->width, J->height, J->depth, &opts);
    if (ret < 0 )
    {
        printf ("clahs error, ret=%d\n", ret);
    }

    // map CLAHS output to 8-bit since opencv save only supports 8-bit images
    IplImage *K = NULL;
    if (I->depth > 8)
    {
        CvSize size = {J->width, J->height};
        K = cvCreateImage (size, IPL_DEPTH_8U, 1);
        cvConvertImage (J, K, 0);
    }
    else
        K = J;
    cvSaveImage (ofile, K, 0);

    // display?
    if (getopt_get_bool (gopt, "display"))
    {
        cvNamedWindow ("Original", CV_WINDOW_AUTOSIZE);
        cvShowImage ("Original", I);

        cvNamedWindow ("CLAHS", CV_WINDOW_AUTOSIZE);
        cvShowImage ("CLAHS", K);

        cvWaitKey (0);
        cvDestroyWindow ("Original");
        cvDestroyWindow ("CLAHS-K");
    }

    // clean up
    if ( K != J)
        cvReleaseImage (&K);
    if ( J != I)
        cvReleaseImage (&J);
    cvReleaseImage (&I);
}
