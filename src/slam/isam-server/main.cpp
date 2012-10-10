#include <iostream>

// external linking req'd
#include <glib.h>

#include "perls-math/fasttrig.h"

#include "isam_server.h"        // shm

// ONLY FOR SIGINT HANDLER, DONT USE IN FUNCTIONS
isamServer *server = NULL;

static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("my_signal_handler()\n");
    if (server->done) {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        server->done = 1;
}

//------------------------------------------------------------------------
int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    if (!g_thread_supported ()) 
        g_thread_init (NULL);

    // install custom signal handler
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    fasttrig_init ();

    // initialize
    server = new isamServer (argc, argv);
    server->run ();

    // clean up
    std::cout << "Goodbye" << std::endl;

    return 0;
}


