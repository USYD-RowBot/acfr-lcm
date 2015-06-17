#include <stdio.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-vision/botimage.h"
#include "perls-lcmtypes/bot_core_image_t.h"

static void
my_handler (const lcm_recv_buf_t *rbuf, const char *channel,
            const bot_core_image_t *img_bot, void *user)
{
    printf ("Got Event\n");
    IplImage img_cv = vis_botimage_to_iplimage_view (img_bot);
    cvNamedWindow ("Image", CV_WINDOW_AUTOSIZE);
    cvShowImage ("Image", &img_cv);
    cvWaitKey (10);
}

int
main (int argc, char *argv[])
{
    lcm_t *lcm;

    lcm = lcm_create (NULL);
    if (!lcm)
        return -1;

    const char channel[] = "PROSILICA_C";
    //const char channel[] = "SENSOR.CAM1.DATA";
    bot_core_image_t_subscription_t *sub =
        bot_core_image_t_subscribe (lcm, channel, &my_handler, NULL);

    while (1)
        lcm_handle (lcm);

    bot_core_image_t_unsubscribe (lcm, sub);
    lcm_destroy (lcm);

    return 0;
}
