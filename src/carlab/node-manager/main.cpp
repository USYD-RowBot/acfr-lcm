#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <boost/lexical_cast.hpp>

#include <openJaus/openJaus.h>

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"

#define CLEAR_COMMAND "clear"

struct config_t
{
    int subsystem_id;
    int node_id;
    char subsystem_name [256];
    char node_name [256];

    // component comms
    int compcomms_jausopc_udp;
    int compcomms_openjaus_udp;

    // node comms
    int nodecomms_enabled;
    int nodecomms_judp;
    char nodecomms_judp_addr[256];
    int nodecomms_opc;
    char nodecomms_opc_addr[256];

    // subsystem comms
    int subcomms_enabled;
    int subcomms_judp;
    char subcomms_judp_addr[256];
    int subcomms_opc;
    char subcomms_opc_addr[256];
};

struct state_t
{
    int done;
    int is_daemon;

    NodeManager *nm;
    config_t config;
};

// Init state structure
state_t state = {0};

//----------------------------------------------------------------------
// Help menu display
//----------------------------------------------------------------------
void
print_help_menu()
{
	printf ("\n\nOpenJAUS Node Manager Help\n");
	printf ("   t - Print System Tree\n");
	printf ("   T - Print Detailed System Tree\n");
	printf ("   c - Clear console window\n");
	printf ("   ? - This Help Menu\n");
	printf (" ESC - Exit Node Manager\n");
}

//----------------------------------------------------------------------
// Parse user input
//----------------------------------------------------------------------
void
parse_user_input(char input)
{
	switch (input)
	{
		case 'T':
			printf ("\n\n%s", state.nm->systemTreeToDetailedString().c_str());
			break;

		case 't':
			printf ("\n\n%s", state.nm->systemTreeToString().c_str());
			break;

		case 'c':
		case 'C':
			system (CLEAR_COMMAND);
			break;

		case '?':
			print_help_menu ();

		default:
			break;
	}

}

//----------------------------------------------------------------------
// Loads the required info from the .cfg file into the state
//----------------------------------------------------------------------
void
node_manager_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }

    config->subsystem_id = 1;
    config->node_id = 1;
    strcpy (config->subsystem_name, "Subsystem");
    strcpy (config->node_name, "Node");
    config->compcomms_jausopc_udp = false;
    config->compcomms_openjaus_udp = false;
    config->nodecomms_enabled = false;
    config->nodecomms_judp = false;
    strcpy (config->nodecomms_judp_addr, "");
    config->nodecomms_opc = false;
    strcpy (config->nodecomms_opc_addr, "");
    config->subcomms_enabled = false;
    config->subcomms_judp = false;
    strcpy (config->subcomms_judp_addr, "");
    config->subcomms_opc = false;
    strcpy (config->subcomms_opc_addr, "");

    bot_param_get_int (param, "jaus-node-manager.subsystem_id", &config->subsystem_id);
    bot_param_get_int (param, "jaus-node-manager.node_id", &config->node_id);

    char *buf;

    if (bot_param_get_str (param, "jaus-node-manager.subsystem_name", &buf) == 0) {
        snprintf (config->subsystem_name, 256, "%s", buf);
        free (buf);
    }

    if (bot_param_get_str (param, "jaus-node-manager.node_name", &buf) == 0) {
        snprintf (config->node_name, 256, "%s", buf);
        free (buf);
    }

    bot_param_get_boolean (param, "jaus-node-manager.component-comms.JAUS_OPC_UDP_Interface",
                &config->compcomms_jausopc_udp);
    bot_param_get_boolean (param, "jaus-node-manager.component-comms.OpenJAUS_UDP_Interface",
                &config->compcomms_openjaus_udp);

    if (bot_param_get_boolean (param, "jaus-node-manager.node-comms.JUDP_Interface", 
            &config->nodecomms_judp) == 0 && config->nodecomms_judp) {
        config->nodecomms_enabled = true;

        if (bot_param_get_str (param, "jaus-node-manager.node-comms.JUDP_IP_Address", &buf) == 0) {
            snprintf (config->nodecomms_judp_addr, 256, "%s", buf);
            free (buf);
        }
    }

    if (bot_param_get_boolean (param, "jaus-node-manager.node-comms.JAUS_OPC_UDP_Interface", 
            &config->nodecomms_opc) == 0 && config->nodecomms_opc) {
        config->nodecomms_enabled = true;

        if (bot_param_get_str (param, "jaus-node-manager.node-comms.JAUS_OPC_UDP_IP_Address", &buf) == 0) {
            snprintf (config->nodecomms_opc_addr, 256, "%s", buf);
            free (buf);
        }
    }

    if (bot_param_get_boolean (param, "jaus-node-manager.subsystem-comms.JUDP_Interface", 
            &config->subcomms_judp) == 0 && config->subcomms_judp) {
        config->subcomms_enabled = true;

        if (bot_param_get_str (param, "jaus-node-manager.subsystem-comms.JUDP_IP_Address", &buf) == 0) {
            snprintf (config->subcomms_judp_addr, 256, "%s", buf);
            free (buf);
        }
    }

    if (bot_param_get_boolean (param, "jaus-node-manager.subsystem-comms.JAUS_OPC_UDP_Interface", 
            &config->subcomms_opc) == 0 && config->subcomms_opc) {
        config->subcomms_enabled = true;

        if (bot_param_get_str (param, "jaus-node-manager.subsystem-comms.JAUS_OPC_UDP_IP_Address", &buf) == 0) {
            snprintf (config->subcomms_opc_addr, 256, "%s", buf);
            free (buf);
        }
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

//----------------------------------------------------------------------
// Handles JAUS messages and displays status updates
//----------------------------------------------------------------------
class MyEventHandler : public EventHandler
{
public:
	~MyEventHandler (void) {}

	void
    handleEvent (NodeManagerEvent *e)
	{
		SystemTreeEvent *treeEvent;
		ErrorEvent *errorEvent;
		JausMessageEvent *messageEvent;
		DebugEvent *debugEvent;
		ConfigurationEvent *configEvent;

		switch(e->getType())
		{
			case NodeManagerEvent::SystemTreeEvent:
				treeEvent = (SystemTreeEvent *)e;
				printf("%s\n", treeEvent->toString().c_str());
				delete e;
				break;

			case NodeManagerEvent::ErrorEvent:
				errorEvent = (ErrorEvent *)e;
				printf("%s\n", errorEvent->toString().c_str());
				delete e;
				break;

			case NodeManagerEvent::JausMessageEvent:
				messageEvent = (JausMessageEvent *)e;
				// If you turn this on, the system gets spam-y this is very useful for debug purposes
				if(messageEvent->getJausMessage()->commandCode != JAUS_REPORT_HEARTBEAT_PULSE)
				{
					//printf("%s\n", messageEvent->toString().c_str());
				}
				else
				{
					//printf("%s\n", messageEvent->toString().c_str());
				}
				delete e;
				break;

			case NodeManagerEvent::DebugEvent:
				debugEvent = (DebugEvent *)e;
				//printf("%s\n", debugEvent->toString().c_str());
				delete e;
				break;

			case NodeManagerEvent::ConfigurationEvent:
				configEvent = (ConfigurationEvent *)e;
				printf("%s\n", configEvent->toString().c_str());
				delete e;
				break;

			default:
				delete e;
				break;
		}
	}
};

//----------------------------------------------------------------------
// Takes config struct from file/command line and sets node config
//----------------------------------------------------------------------
NodeConfig *
set_node_cfg (config_t *config)
{
	NodeConfig *configData = new NodeConfig();

    configData->set_data_string ("JAUS", "SubsystemId", boost::lexical_cast<std::string>(config->subsystem_id).c_str());
    configData->set_data_string ("JAUS", "NodeId", boost::lexical_cast<std::string>(config->node_id).c_str());
    configData->set_data_string ("JAUS", "Subsystem_Identification", config->subsystem_name);
    configData->set_data_string ("JAUS", "Node_Identification", config->node_name);

    // component comms
    if (config->compcomms_jausopc_udp)
        configData->set_data_string ("Component_Communications", "JAUS_OPC_UDP_Interface", "true");
    else
        configData->set_data_string ("Component_Communications", "JAUS_OPC_UDP_Interface", "false");

    if (config->compcomms_openjaus_udp)
        configData->set_data_string ("Component_Communications", "OpenJAUS_UDP_Interface", "true");
    else
        configData->set_data_string ("Component_Communications", "OpenJAUS_UDP_Interface", "false");

    // node comms
    if (config->nodecomms_enabled) {
        configData->set_data_string ("Node_Communications", "Enabled", "true");

        if (config->nodecomms_judp) {
            configData->set_data_string ("Node_Communications", "JUDP_Interface", "true");
            configData->set_data_string ("Node_Communications", "JUDP_IP_Address", config->nodecomms_judp_addr);
        }
        else
            configData->set_data_string ("Node_Communications", "JUDP_Interface", "false");

        if (config->nodecomms_opc) {
            configData->set_data_string ("Node_Communications", "JAUS_OPC_UDP_Interface", "true");
            configData->set_data_string ("Node_Communications", "JAUS_OPC_UDP_IP_Address", config->nodecomms_opc_addr);
        }
        else
            configData->set_data_string ("Node_Communications", "JAUS_OPC_UDP_Interface", "false");
    }
    else
        configData->set_data_string ("Node_Communications", "Enabled", "false");

    // subsystem comms
    if (config->subcomms_enabled) {
        configData->set_data_string ("Subsystem_Communications", "Enabled", "true");

        if (config->subcomms_judp) {
            configData->set_data_string ("Subsystem_Communications", "JUDP_Interface", "true");
            configData->set_data_string ("Subsystem_Communications", "JUDP_IP_Address", config->subcomms_judp_addr);
        }
        else
            configData->set_data_string ("Subsystem_Communications", "JUDP_Interface", "false");

        if (config->subcomms_opc) {
            configData->set_data_string ("Subsystem_Communications", "JAUS_OPC_UDP_Interface", "true");
            configData->set_data_string ("Subsystem_Communications", "JAUS_OPC_UDP_IP_Address", config->subcomms_opc_addr);
        }
        else
            configData->set_data_string ("Subsystem_Communications", "JAUS_OPC_UDP_Interface", "false");
    }
    else
        configData->set_data_string ("Subsystem_Communications", "Enabled", "false");

    return configData;
}

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
    node_manager_load_cfg (&state.config);
    
    // read in the command line options
    getopt_t *gopt = getopt_create ();
    
    getopt_add_description (gopt, "JAUS 3.3 Node Manager");
    getopt_add_int (gopt, 's', "subid", boost::lexical_cast<std::string>(state.config.subsystem_id).c_str(), "Subsystem ID");
    getopt_add_int (gopt, 'n', "nodeid", boost::lexical_cast<std::string>(state.config.node_id).c_str(), "Node ID");
    getopt_add_string (gopt, 'S', "subname", state.config.subsystem_name, "Subsystem Name");
    getopt_add_string (gopt, 'N', "nodename", state.config.node_name, "Node Name");
    getopt_add_string (gopt, 'i', "nodeip", state.config.nodecomms_judp_addr, "JUDP Node IP Address");
    getopt_add_string (gopt, 'I', "subip", state.config.subcomms_judp_addr, "JUDP Subsystem IP Address");
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

    if (getopt_has_flag (gopt, "subid"))
        state.config.subsystem_id = getopt_get_int (gopt, "subid");

    if (getopt_has_flag (gopt, "nodeid"))
        state.config.subsystem_id = getopt_get_int (gopt, "nodeid");

    if (getopt_has_flag (gopt, "subname"))
        snprintf (state.config.subsystem_name, 256, "%s", getopt_get_string (gopt, "subname"));

    if (getopt_has_flag (gopt, "nodename"))
        snprintf (state.config.node_name, 256, "%s", getopt_get_string (gopt, "nodename"));

    if (getopt_has_flag (gopt, "nodeip")) {
        state.config.nodecomms_enabled = true;
        state.config.nodecomms_judp = true;
        snprintf (state.config.nodecomms_judp_addr, 256, "%s", getopt_get_string (gopt, "nodeip"));
    }

    if (getopt_has_flag (gopt, "subip")) {
        state.config.subcomms_enabled = true;
        state.config.subcomms_judp = true;
        snprintf (state.config.subcomms_judp_addr, 256, "%s", getopt_get_string (gopt, "subip"));
    }

    // setup UI
	struct termios newTermio;
	struct termios storedTermio;
	char choice[8] = {0};
	int count = 0;

	tcgetattr (0,&storedTermio);
	memcpy (&newTermio, &storedTermio, sizeof (struct termios) );

	// Disable canonical mode, and set buffer size to 0 byte(s)
	newTermio.c_lflag &= (~ICANON);
	newTermio.c_lflag &= (~ECHO);
	newTermio.c_cc[VTIME] = 10;
	newTermio.c_cc[VMIN] = 0;
	tcsetattr (0,TCSANOW,&newTermio);


    NodeConfig *configData = set_node_cfg (&state.config);
	MyEventHandler *handler = new MyEventHandler();

	try {
        // initiate node manager
		state.nm = new NodeManager(configData, handler);
		print_help_menu ();
	} catch(char *exceptionString) {
		printf("%s", exceptionString);
		printf ("Terminating Program...\n");
        state.done = 1;
	} catch(...) {
		printf ("Node Manager Construction Failed. Terminating Program...\n");
        state.done = 1;
	}

	while(!state.done) {
		memset (choice, 0, 8);
		count = read(0, &choice, 8);
		if(count == 1 && choice[0] == 27) { // ESC
			state.done = 1;
		} else if(count == 1) {
			parse_user_input (choice[0]);
		}
	}

    printf("Shutting Down Node Manager...\n");

	delete state.nm;
	delete handler;
	delete configData;


    printf ("\nDone.\n");
	tcsetattr(0, TCSANOW, &storedTermio);
    exit (EXIT_SUCCESS);
}
