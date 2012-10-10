#include <iostream>

#include <glib.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"

#include "isam_server.h"        // shm
#include "fileio_thread.h"

gpointer
fileio_thread (gpointer user)
{   
    std::cout << "[fileio]\tSpawning" << std::endl;
    
    fileio_tdata_t *tdata = (fileio_tdata_t *)user;

    // identify that we are using queue in this thread
    g_async_queue_ref (tdata->gq);

    BotParam *param;
    param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not get config parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }
    
    // graph parser
    gParser *gp = new gParser (tdata->lcm, tdata->input_fname);
    if (!gp)
        std::cout << "[fileio]\tError: init_gp()" << std::endl;
    
    gp->set_gq (tdata->gq);

    // parse line by line and publish nodes and factors
    gp->fread_graph ();
    
    std::cout << "[fileio]\tDone parsing: " << g_async_queue_length (tdata->gq) << " nodes and vlinks." << std::endl;

    *(tdata->active) = true;

    std::cout << "[fileio]\tExiting" << std::endl;
    
    g_async_queue_unref (tdata->gq);
    
    g_thread_exit (0);
    return NULL;
}
