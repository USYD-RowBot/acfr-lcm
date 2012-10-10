#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

// external linking req'd
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-lcmtypes/perllcm_van_feature_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_siftgpu_t.h"

#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"

#include "perls-vision/feature.h"
#include "perls-vision/plot.h"
#include "perls-vision/featuregpu.h"

#define DESCRIPTION \
    "Command line util for running SiftGPU feature extraction on an image file."

int
main (int argc, char *argv[])
{
    // options
    char default_port_str[16];
    snprintf (default_port_str, sizeof (default_port_str), "%d", VIS_SIFTGPU_TCP_PORT);
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, DESCRIPTION);
    getopt_add_string (gopt, 'k',   "keyfile",        "./output.key",   "generated key file");
    getopt_add_string (gopt, 'p',   "params",           "\"-loweo -s 1 -fo 0 -v 1 -p 1360x1024 -tc1 1000\"", "SiftGPU params");
    getopt_add_bool   (gopt, 'v',   "visualize",      0,                "show resulting features");
    getopt_add_string (gopt, 'h',   "host",           "127.0.0.1",      "Server IP address");
    getopt_add_int    (gopt, 'p',   "port",           default_port_str, "Server port");
    getopt_add_help   (gopt, NULL);

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help") || gopt->extraargs->len !=1) {
        getopt_do_usage (gopt, "imagefile");
        exit (EXIT_FAILURE);
    }

    // load image
    const char *img_fullname = gopt->extraargs->pdata[0];
    IplImage* imggray = cvLoadImage (img_fullname, CV_LOAD_IMAGE_GRAYSCALE);
    if (!imggray) {
        printf ("ERROR: loading image\n");
        getopt_do_usage (gopt, NULL);
        return 0;
    }

    const char *host = strdup (getopt_get_string (gopt, "host"));
    int port = getopt_get_int (gopt, "port");

    // run siftgpu
    int64_t t0 = timestamp_now ();
    perllcm_van_feature_t *f = NULL;
    f = vis_feature_siftgpu (imggray, NULL, host, port, 1000);
    int64_t dt = timestamp_now () - t0;
    if (!f)
        exit (EXIT_FAILURE);
    printf ("%"PRId64" ms\n", dt);

    // write to a key file
    const char *key_fullname = getopt_get_string (gopt, "keyfile");
    vis_feature_fprintf (f, key_fullname);

    if (getopt_get_bool (gopt, "visualize")) {
        vis_plot_feature (imggray, f, img_fullname, 1.0, 0);
        cvWaitKey(0);
    }

    // clean up
    perllcm_van_feature_t_destroy (f);
    cvReleaseImage (&imggray);
    exit (EXIT_SUCCESS);
}