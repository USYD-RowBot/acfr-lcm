//System includes
#include <stdio.h>

//PeRLs includes
#include "perls-lcmtypes/perllcm_van_feature_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_cvsurf_t.h"

#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"

#include "perls-vision/feature.h"
#include "perls-vision/plot.h"
#include "perls-vision/featuregpu.h"

//OpenCV includes
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define DESCRIPTION \
    "Test example file for running SurfGPU feature extraction on an image file."

int
main (int argc, char *argv[])
{

    float scale = 0.5;
    // options
    char default_port_str[16];
    snprintf (default_port_str, sizeof (default_port_str), "%d", VIS_SURFGPU_TCP_PORT);
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, DESCRIPTION);
    getopt_add_bool   (gopt, 'v',   "visualize",      0,                "show resulting features");
    getopt_add_string (gopt, 'h',   "host",           "127.0.0.1",      "Server IP address");
    getopt_add_int    (gopt, 'p',   "port",           default_port_str, "Server port");
    getopt_add_help   (gopt, NULL);

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help") || gopt->extraargs->len !=1)
    {
        getopt_do_usage (gopt, "imagefile");
        exit (EXIT_FAILURE);
    }

    // load image
    const char *img_fullname = gopt->extraargs->pdata[0];
    IplImage* imggray = cvLoadImage (img_fullname, CV_LOAD_IMAGE_GRAYSCALE);
    if (!imggray)
    {
        printf ("ERROR: loading image\n");
        getopt_do_usage (gopt, NULL);
        return 0;
    }

    const char *host = strdup (getopt_get_string (gopt, "host"));
    int port = getopt_get_int (gopt, "port");

    int64_t t0 = timestamp_now ();
    perllcm_van_feature_t *f = NULL;
    f = vis_feature_surfgpu (imggray, NULL, host, port, VIS_SURFGPU_MAX_FEATURES);
    int64_t dt = timestamp_now () - t0;
    if (!f)
        exit (EXIT_FAILURE);
    printf ("%"PRId64" ms\n", dt);

    // write to a key file
    const char *key_fullname = "out_gpu.key";
    vis_feature_fprintf (f, key_fullname);

    if (getopt_get_bool (gopt, "visualize"))
    {
        vis_plot_feature (imggray, f, "gpu", scale, 0);
        //cvWaitKey(0);
    }

    // opencv case
    t0 = timestamp_now ();
    perllcm_van_feature_t *f_ocv = NULL;
    CvSURFParams params = cvSURFParams (500, 1);
    params.extended = 1; // or 1 (64 vec or 128 vec)
    params.hessianThreshold = 1000; // 300~500
    params.nOctaves = 3;
    params.nOctaveLayers = 4;
    f_ocv = vis_feature_cvsurf (imggray, NULL, params, VIS_SURFGPU_MAX_FEATURES);
    dt = timestamp_now () - t0;
    if (!f_ocv)
        exit (EXIT_FAILURE);
    printf ("%"PRId64" ms\n", dt);

    // write to a key file
    const char *key_fullname_ocv = "out_ocv.key";
    vis_feature_fprintf (f_ocv, key_fullname_ocv);

    if (getopt_get_bool (gopt, "visualize"))
    {
        vis_plot_feature (imggray, f_ocv, "ocv", scale, 0);
        //cvWaitKey(0);
    }
    cvWaitKey(0);

    exit (EXIT_SUCCESS);

}
