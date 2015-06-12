#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <assert.h>
#include <stdint.h>
#include <inttypes.h>

// external linking req'd
#include <math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"

#include "perls-common/bot_util.h"
#include "perls-common/getopt.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "perls-lcmtypes/perllcm_segway_navigator_t.h"
#include "perls-lcmtypes/perllcm_position_t.h"

// define state structure
typedef struct _state_t state_t;
struct _state_t
{
    int done;
    int is_daemon;
    lcm_t *lcm;

    char *channel;
    char *segway_navigator_channel;
};

//Init the state structure
state_t state = {0};

//----------------------------------------------------------------------------------
// Called when program shuts down
//----------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\n Caught Ctrl+C quitting ... \n");
    if (state.done)
    {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        state.done = 1;
}

void
segway_navigator_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const perllcm_segway_navigator_t *msg, void *user)
{

    if (!msg->mu_len)
        return;

    // create an position output
    perllcm_position_t *pose = calloc (1, sizeof (*pose));

    pose->utime = msg->utime;

    pose->xyzrph[0] = (-1 == msg->index.x) ? 0 : msg->mu[msg->index.x];
    pose->xyzrph[1] = (-1 == msg->index.y) ? 0 : msg->mu[msg->index.y];
    pose->xyzrph[2] = (-1 == msg->index.z) ? 0 : msg->mu[msg->index.z];
    pose->xyzrph[3] = (-1 == msg->index.r) ? 0 : msg->mu[msg->index.r];
    pose->xyzrph[4] = (-1 == msg->index.p) ? 0 : msg->mu[msg->index.p];
    pose->xyzrph[5] = (-1 == msg->index.h) ? 0 : msg->mu[msg->index.h];

    // fill velocities and rates
    if (-1 != msg->index.x_dot && -1 != msg->index.r_dot)
    {
        pose->xyzrph_dot[0] = (-1 == msg->index.x_dot) ? 0 : msg->mu[msg->index.x_dot];
        pose->xyzrph_dot[1] = (-1 == msg->index.y_dot) ? 0 : msg->mu[msg->index.y_dot];
        pose->xyzrph_dot[2] = (-1 == msg->index.z_dot) ? 0 : msg->mu[msg->index.z_dot];
        pose->xyzrph_dot[3] = (-1 == msg->index.r_dot) ? 0 : msg->mu[msg->index.r_dot];
        pose->xyzrph_dot[4] = (-1 == msg->index.p_dot) ? 0 : msg->mu[msg->index.p_dot];
        pose->xyzrph_dot[5] = (-1 == msg->index.h_dot) ? 0 : msg->mu[msg->index.h_dot];
    }
    else if (-1 != msg->index.u || -1 != msg->index.a)
    {

        //rotate body velocities into world frame
        double R_vw[9];
        so3_rotxyz (R_vw, &(pose->xyzrph[3]));
        gsl_matrix_view R_vw_v = gsl_matrix_view_array (R_vw, 3, 3);
        GSLU_VECTOR_VIEW (uvw_v, 3, {(-1 == msg->index.u) ? 0 : msg->mu[msg->index.u],
                                     (-1 == msg->index.v) ? 0 : msg->mu[msg->index.v],
                                     (-1 == msg->index.w) ? 0 : msg->mu[msg->index.w]
                                    });
        gsl_vector_view xyz_dot = gsl_vector_view_array (&(pose->xyzrph_dot[0]), 3);
        gslu_blas_mv (&xyz_dot.vector, &R_vw_v.matrix, &uvw_v.vector);

        // set euler rates
        double abc[3] = {(-1 == msg->index.a) ? 0 : msg->mu[msg->index.a],
                         (-1 == msg->index.b) ? 0 : msg->mu[msg->index.b],
                         (-1 == msg->index.c) ? 0 : msg->mu[msg->index.c]
                        };
        so3_body2euler (abc, &(pose->xyzrph[3]), &(pose->xyzrph_dot[3]), NULL);

    }

    // fill covariance
    int inds[6] = {msg->index.x, msg->index.y, msg->index.z,
                   msg->index.r, msg->index.p, msg->index.h
                  };

    gsl_matrix_view Sigma = gsl_matrix_view_array (pose->xyzrph_cov, 6, 6);
    for (int i=0;  i<6; i++)
    {
        for (int j=0;  j<6; j++)
        {
            if (inds[i] == -1 || inds[j] == -1)
                gsl_matrix_set (&Sigma.matrix, i, j, 0);
            else
            {
                int ii = inds[i] * msg->state_len + inds[j];
                gsl_matrix_set (&Sigma.matrix, i, j, msg->Sigma[ii]);
            }
        }
    }

    perllcm_position_t_publish (state.lcm, state.channel, pose);

    free (pose);
}


int main(int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    fasttrig_init ();

    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (! param)
    {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit(EXIT_FAILURE);
    }

    // read the config file
    state.channel = bot_param_get_str_or_fail (param, "seg-position.channel");
    state.segway_navigator_channel = bot_param_get_str_or_fail (param, "navigator.channel");

    // options
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "Segway Position Process");
    getopt_add_bool    (gopt, 'h', "help",    0,  "Show this");
    getopt_add_bool    (gopt, 'D', "daemon",  0,  "Run as daemon?");
    // TODO add option to parse and create new logfile

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=0)
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    // daemon mode?
    state.is_daemon = getopt_get_bool (gopt, "daemon");
    if (state.is_daemon)
        daemon_fork ();

    state.lcm = lcm_create (NULL);

    // subscribe to lcm channels
    perllcm_segway_navigator_t_subscribe (state.lcm, state.segway_navigator_channel, &segway_navigator_cb, NULL);

    while (!state.done)
        lcm_handle (state.lcm);

}
