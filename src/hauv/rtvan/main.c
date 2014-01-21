#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

// external linking req'd
#include <glib.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/error.h"

#include "perls-math/fasttrig.h"

#include "feature_thread.h"
#include "hauv_thread.h"
#include "saliency_thread.h"
#include "link_thread.h"
#include "plot_thread.h"
#include "secam_thread.h"
#include "isam_interf_thread.h"
#include "shared_memory.h"
#include "twoview_thread.h"

#define printf(format, ...)                             \
    printf ("%-12s " format, "[main]", ## __VA_ARGS__)

// shared memory pointer
shm_t *shm=NULL;

static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("my_signal_handler()\n");
    if (shm->done) {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        shm->done = 1;
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    if (!g_thread_supported ()) 
        g_thread_init (NULL);

    // install custom signal handler
    struct sigaction act = {
        .sa_sigaction = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    // initialize
    shm = shared_memory_init (argc, argv);

    fasttrig_init ();

    // launch threads
    shm->tid_hauv_thread = g_thread_create (&hauv_thread, NULL, 1, NULL);
    shm->tid_feature_thread = g_thread_create (&feature_thread, NULL, 1, NULL);

    shm->tid_saliency_thread = g_thread_create (&saliency_thread, NULL, 1, NULL);

    bool use_seserver = 1;
    bool use_isamserver = 0;

    botu_param_get_boolean_to_bool (shm->param, "rtvan.link_thread.use_seserver", &use_seserver);
    botu_param_get_boolean_to_bool (shm->param, "rtvan.link_thread.use_isamserver", &use_isamserver);

    if (use_seserver && use_isamserver) {
        ERROR ("use_seserver and use_isamserver can not both be ON\n");
        exit (EXIT_FAILURE);
    }

    if (use_seserver) {
        printf ("using seserver link proposal. (see master.cfg)\n");
        shm->tid_isam_interf_thread = g_thread_create (&secam_thread, NULL, 1, NULL);
    }
    else if (use_isamserver) {
        printf ("using isam-server link proposal. (see master.cfg)\n");
        shm->tid_isam_interf_thread = g_thread_create (&isam_interf_thread, NULL, 1, NULL);
    }
    else {
        printf ("using rtvan link proposal. (see master.cfg)\n");
        shm->tid_link_thread = g_thread_create (&link_thread, NULL, 1, NULL);
        shm->active = true;
    }
    shm->tid_plot_thread = g_thread_create (&plot_thread, NULL, 1, NULL);
    shm->tid_twoview_thread = g_thread_create (&twoview_thread, NULL, 1, NULL);

    while (!shm->done) {
        struct timeval timeout = {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        lcmu_handle_timeout (shm->lcm, &timeout);
    }

    // wait for children threads to die
    if (shm->tid_feature_thread)     g_thread_join (shm->tid_feature_thread);
    if (shm->tid_hauv_thread)        g_thread_join (shm->tid_hauv_thread);
    if (shm->tid_link_thread)        g_thread_join (shm->tid_link_thread);
    if (shm->tid_isam_interf_thread) g_thread_join (shm->tid_isam_interf_thread);
    if (shm->tid_plot_thread)        g_thread_join (shm->tid_plot_thread);
    if (shm->tid_twoview_thread)     g_thread_join (shm->tid_twoview_thread);
    if (shm->tid_saliency_thread)    g_thread_join (shm->tid_saliency_thread);

    // clean up
    shared_memory_free (shm);

    printf ("Goodbye\n");
    return 0;
}
