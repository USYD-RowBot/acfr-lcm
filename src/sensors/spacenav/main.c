#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <X11/Xlib.h>
#include <spnav.h>

#include "perls-common/getopt.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"

#include "perls-lcmtypes/senlcm_spacenav_t.h"

void
sig_interrupt (int s)
{
    spnav_close ();
    exit (EXIT_SUCCESS);
}

int
main (int argc, char *argv[])
{
    // read command line args
    getopt_t *gopt = getopt_create ();

    getopt_add_description (gopt, "Run SpaceNavigator Controller");
    getopt_add_bool (gopt, 'D', "daemon", 0,    "Run as system daemon");
    getopt_add_bool (gopt, 'h', "help",   0,    "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len > 0)
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    if (getopt_get_bool (gopt, "daemon"))
        daemon_fork ();

    Display *dpy;
    Window win;
    unsigned long bpix;

    spnav_event sev;
    signal (SIGINT, sig_interrupt);

    // connect to xserver
    if ( !(dpy = XOpenDisplay (0) ) )
    {
        ERROR ("failed to connect to the X server\n");
        exit (EXIT_FAILURE);
    }
    bpix = BlackPixel (dpy, DefaultScreen (dpy));
    win = XCreateSimpleWindow (dpy, DefaultRootWindow (dpy), 0, 0, 1, 1, 0, bpix, bpix);

    // connect to spacenavd
    if (spnav_x11_open (dpy, win) == -1)
    {
        ERROR ("failed to connect to the space navigator daemon\n");
        exit (EXIT_FAILURE);
    }

    // create lcm
    lcm_t *lcm = lcm_create (NULL);
    if (!lcm)
    {
        ERROR ("lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }

    while (spnav_wait_event (&sev))
    {
        if (sev.type == SPNAV_EVENT_MOTION)
        {
            printf ("t(%d, %d, %d) ", sev.motion.x, sev.motion.y, sev.motion.z);
            printf ("r(%d, %d, %d)\n", sev.motion.rx, sev.motion.ry, sev.motion.rz);

            senlcm_spacenav_t sevent =
            {
                .x     = sev.motion.x,
                .y     = sev.motion.y,
                .z     = sev.motion.z,
                .rot_x = sev.motion.rx,
                .rot_y = sev.motion.ry,
                .rot_z = sev.motion.rz,
            };
            senlcm_spacenav_t_publish (lcm, "SPACENAV", &sevent);
        }
        else
        {
            // will eventually reset spaceball on button press -- will not
            // report event
            printf ("button %s event b(%d)\n", sev.button.press ? "press" : "release", sev.button.bnum);
        }
    }

    // clean up
    spnav_close ();
    getopt_destroy (gopt);
    exit (EXIT_SUCCESS);
}
