#include <stdlib.h>
#include <cstdlib>
#include <unistd.h>
#include <termios.h>

#include <lcm/lcm-cpp.hpp>
#include "perls-lcmtypes++/perllcm/carlab_discrete_devices_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_error_report_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_error_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_signals_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_state_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_wrench_effort_t.hpp"

#include "perls-common/timeutil.h"
#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"

#include "jausocu.h"

struct config_t
{
    // put node configurations in here..
    char set_discrete_devices_channel[256];
    char set_signals_channel[256];
    char set_wrench_effort_channel[256];

    char report_discrete_devices_channel[256];
    char report_errors_channel[256];
    char report_signals_channel[256];
    char report_state_channel[256];
    char report_wrench_effort_channel[256];
};

struct state_t
{
    int done;
    int is_daemon;
    lcm::LCM lcm;

    JAUSOCU *ocu;

    config_t config;
};

// Init state structure
state_t state = {0};

//----------------------------------------------------------------------
// Loads the required info from the .cfg file into the state
//----------------------------------------------------------------------
void
lcm_jaus_ocu_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }

}

//----------------------------------------------------------------------------------
// Called when program shuts down 
//----------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\nmy_signal_handler()\n");
    if (state.done) {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    } 
    else
        state.done = 1;
}

class LCMHandler
{
public:
    LCMHandler (state_t *st) {this->state = st;}
    ~LCMHandler (void) {}

    //----------------------------------------------------------------------------------
    // LCM Input Messages (LCM -> JAUS)
    //----------------------------------------------------------------------------------
    void
    set_discrete_devices_cb (const lcm::ReceiveBuffer *rbuf, const std::string &chan,
            const perllcm::carlab_discrete_devices_t *msg)
    {
        state->ocu->set_discrete_devices (msg);
    }

    void
    set_signals_cb (const lcm::ReceiveBuffer *rbuf, const std::string &chan,
            const perllcm::carlab_signals_t *msg)
    {
        state->ocu->set_signals (msg);
    }

    void
    set_wrench_effort_cb (const lcm::ReceiveBuffer *rbuf, const std::string &chan,
            const perllcm::carlab_wrench_effort_t *msg)
    {
        state->ocu->set_wrench_effort (msg);
    }

    //----------------------------------------------------------------------------------
    // LCM Output Messages (JAUS -> LCM)
    //----------------------------------------------------------------------------------
    void
    report_discrete_devices_cb (const perllcm::carlab_discrete_devices_t *msg)
    {
        //printf ("REPORTING: discrete devices\n");
        state->lcm.publish (state->config.report_discrete_devices_channel, msg);
    }

    void
    report_errors_cb (const perllcm::carlab_error_report_t *msg)
    {
        //printf ("REPORTING: errors\n");
        state->lcm.publish (state->config.report_errors_channel, msg);
    }

    void
    report_signals_cb (const perllcm::carlab_signals_t *msg)
    {
        //printf ("REPORTING: signals\n");
        state->lcm.publish (state->config.report_signals_channel, msg);
    }

    void
    report_state_cb (const perllcm::carlab_state_t *msg)
    {
        //printf ("REPORTING: state\n");
        state->lcm.publish (state->config.report_state_channel, msg);
    }

    void
    report_wrench_effort_cb (const perllcm::carlab_wrench_effort_t *msg)
    {
        //printf ("REPORTING: wrench efforts\n");
        state->lcm.publish (state->config.report_wrench_effort_channel, msg);
    }

private:
    state_t *state;
};


//----------------------------------------------------------------------------------
int
main(int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // install custom signal handler
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    // load in config file option
    lcm_jaus_ocu_load_cfg (&state.config);
    snprintf (state.config.set_discrete_devices_channel, 256, "CARLAB_SET_DISCRETE_DEVICES");
    snprintf (state.config.set_signals_channel, 256, "CARLAB_SET_SIGNALS");
    snprintf (state.config.set_wrench_effort_channel, 256, "CARLAB_SET_WRENCH_EFFORT");
    snprintf (state.config.report_discrete_devices_channel, 256, "CARLAB_REPORT_DISCRETE_DEVICES");
    snprintf (state.config.report_errors_channel, 256, "CARLAB_REPORT_ERRORS");
    snprintf (state.config.report_signals_channel, 256, "CARLAB_REPORT_SIGNALS");
    snprintf (state.config.report_state_channel, 256, "CARLAB_REPORT_STATE");
    snprintf (state.config.report_wrench_effort_channel, 256, "CARLAB_REPORT_WRENCH_EFFORT");
    
    // read in the command line options
    getopt_t *gopt = getopt_create ();
    
    getopt_add_description (gopt, "LCM-JAUS OCU");
    getopt_add_bool (gopt, 'D', "daemon", 0, "Run as system daemon");
    getopt_add_bool (gopt, 'h', "help",   0, "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);;
    }
    
    //start as daemon if asked
    if (getopt_get_bool (gopt, "daemon")) {
        daemon_fork ();
        state.is_daemon = 1;
    }

    LCMHandler handle (&state);

    // subscribe to incoming channels
    lcm::Subscription *lcm_dd = state.lcm.subscribe (state.config.set_discrete_devices_channel,
            &LCMHandler::set_discrete_devices_cb, &handle);
    lcm::Subscription *lcm_sigs = state.lcm.subscribe (state.config.set_signals_channel,
            &LCMHandler::set_signals_cb, &handle);
    lcm::Subscription *lcm_we = state.lcm.subscribe (state.config.set_wrench_effort_channel,
            &LCMHandler::set_wrench_effort_cb, &handle);

    // initialize JAUS-OCU
    state.ocu = new JAUSOCU ();

    // register outgoing channels with JAUS-OCU
    state.ocu->register_cb (&LCMHandler::report_discrete_devices_cb, &handle);
    state.ocu->register_cb (&LCMHandler::report_errors_cb, &handle);
    state.ocu->register_cb (&LCMHandler::report_signals_cb, &handle);
    state.ocu->register_cb (&LCMHandler::report_state_cb, &handle);
    state.ocu->register_cb (&LCMHandler::report_wrench_effort_cb, &handle);


	while(!state.done) {
        state.lcm.handle ();
	}

    // destroy/close JAUS-OCU
    delete state.ocu;

    // unsubscribe lcm messages
    state.lcm.unsubscribe (lcm_dd);
    state.lcm.unsubscribe (lcm_sigs);
    state.lcm.unsubscribe (lcm_we);

    printf ("\nDone.\n");
    exit (EXIT_SUCCESS);
}
