#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h> // needed for PRId64 macros

#include <lcm/lcm.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/bot_core_image_sync_t.h"

#include "perls-vision/botimage.h"
#include "perls-common/getopt.h"
#include "perls-common/error.h"

int
main (int argc, char *argv[])
{

    // options
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "bot_core_image_t publisher.\n It works with matlab/botimg_publisher/botimg_publisher.m\n See botimg_publisher.m for more detail ");
    getopt_add_bool    (gopt, 'h',  "help", 0,  "Show this");
    getopt_add_long    (gopt, 't',  "time", "1234567890",  "utime to publish");
    getopt_add_string  (gopt, 'f',  "filename", "image.tif", "image filename");
    getopt_add_string  (gopt, 'c',  "channel",  "PROSILICA_M", "LCM Channel");
    getopt_add_bool    (gopt, '\0', "no-sync", 0,  "Publish bot_core_image_t without bot_core_image_sync_t");


    if (!getopt_parse (gopt, argc, argv, 1) || argc == 1)
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }


    int64_t utime = getopt_get_long (gopt, "time");
    const char *filename = getopt_get_string (gopt, "filename");
    const char *channel_name = getopt_get_string (gopt, "channel");

    // lcm
    lcm_t *lcm = lcm_create (NULL);
    if (!lcm)
    {
        printf ("lcm_create() failed!\n");
        return -1;
    }

    // load and publish the image
    IplImage* img = cvLoadImage (filename, -1);

    printf ("%"PRId64", %s, %s\n", utime, filename, channel_name);
    if (img)
    {
        // publish img
        bot_core_image_t *botimg = vis_iplimage_to_botimage_copy (img);
        if (!botimg)
        {
            ERROR ("null bot\n");
            cvReleaseImage (&img);
            exit (-1);
        }

        if (!getopt_get_bool (gopt, "no-sync"))
        {
            char *channel_sync = g_strconcat (channel_name, ".SYNC", NULL);
            bot_core_image_sync_t botsync =
            {
                .utime = utime,
            };
            bot_core_image_sync_t_publish (lcm, channel_sync, &botsync);
            g_free (channel_sync);
        }

        botimg->utime = utime;
        bot_core_image_t_publish (lcm, channel_name, botimg);
        bot_core_image_t_destroy (botimg);

        //cvNamedWindow ("TMP", CV_WINDOW_AUTOSIZE);
        //cvShowImage ("TMP", img);
        //cvWaitKey(0);
        cvReleaseImage (&img);
    }
    else
    {
        ERROR ("Error reading image: %s", filename);
        cvReleaseImage (&img);
    }
}
