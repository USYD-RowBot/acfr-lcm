/* ================================================================
** getoptdemo.c
**
** Simple demo of the getopt.c commandline parser
**
** 29 DEC 2008  ryan eustice  Created and written.
** ================================================================
*/

#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>  /* exit() definition */

#include "perls-common/getopt.h"

int main (int argc, char *argv[])
{

    getopt_t *gopt = getopt_create ();

    getopt_add_bool (gopt,   'h',  "help",            0,        "Show this");
    getopt_add_string (gopt, 'c',  "channel",         "LASER",  "LC channel name");
    getopt_add_spacer (gopt, "");
    getopt_add_int (gopt,    '\0', "hz",              "15",     "Target update rate (in Hertz)");
    getopt_add_string (gopt, 'd',  "device",          "",       "Device to connect to");
    getopt_add_int (gopt,    'b',  "baud",            "500000", "Baud rate");
    getopt_add_int (gopt,    'r',  "resolution",      "25",     "Angular resolution (hundredths of a degree)");
    getopt_add_double (gopt, 'f',  "fov",             "180",    "Field of view (Degrees)");
    getopt_add_bool (gopt,   'i',  "interlaced",      1,        "Interlaced (required for most high-res modes)"); 
    getopt_add_bool (gopt,   '\0', "intensities",     0,        "Request intensities");
    getopt_add_bool (gopt,   '\0', "exit-on-failure", 1,        "Exit -1 if sick connection fails"); 
    getopt_add_bool (gopt,   '\0', "dump",            0,        "Dump all packets to stdout");
    getopt_add_spacer (gopt, "");

    if (!getopt_parse (gopt, argc, argv, 1) 
        || getopt_get_bool (gopt,"help")
        || gopt->extraargs->len!=0) {

        getopt_do_usage (gopt, NULL);
        return 0;
    }


    printf ("Options are:\n");
    const char *channel = getopt_get_string (gopt, "channel");
    printf ("channel\t\t= %s\n", channel);
    printf ("hz\t\t= %d\n", getopt_get_int (gopt, "hz"));
    printf ("device\t\t= %s\n", getopt_get_string (gopt, "device"));
    printf ("baud\t\t= %d\n", getopt_get_int (gopt, "baud"));
    printf ("resolution\t= %d\n", getopt_get_int (gopt, "resolution"));
    printf ("fov\t\t= %f\n", getopt_get_double (gopt, "fov"));
    printf ("interlaced\t= %d\n", getopt_get_bool (gopt, "interlaced"));
    printf ("intensities\t= %d\n", getopt_get_bool (gopt, "intensities"));
    printf ("exit-on-failure\t= %d\n", getopt_get_bool (gopt, "exit-on-failure"));
    printf ("dump\t\t= %d\n", getopt_get_bool (gopt, "dump"));

    getopt_destroy (gopt);

    return 0;
}
