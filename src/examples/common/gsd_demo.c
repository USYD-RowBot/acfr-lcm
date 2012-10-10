#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/generic_sensor_driver.h"

int
main (int argc, char *argv[])
{
    /* initialize gsd struct, which consisting of reading the master.cfg
       file for our key and then setting up the i/o type based upon that */
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, "foo.bar", NULL);

    /* configure for either line-based ascii reading, 
       or if binary data, character-based output */
    //gsd->ascii = 1;
    //gsd_canonical (gsd, eol, eol2);
    gsd_noncanonical (gsd, 88, 1);

    /* open device and launch device i/o thread */
    gsd_launch (gsd);

    /* do any sensor initialize stuff here */

    /* create lcm object */
    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }

    while (1) {
        char buf[1024];
        int64_t timestamp;
        int len;
        len = gsd_read (gsd, buf, 88, &timestamp);
        //int len = gsd_read_timeout (gsd, buf, 1024, &timestamp, 1000);

        /* parse / poke */
        //lcmtype_mystruct_t mystruct;
        //myparse_fcn (buf, len, &mystruct)
        //gsd_write (gsd, buf, len);

        /* publish our parsed data */
        //lcmtype_mystruct_t_publish (lcm, gsd->channel, &mystruct);
    }

    gsd_destroy (gsd);

    return 0;
}
