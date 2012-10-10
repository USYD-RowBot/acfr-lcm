#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <map>

#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>

#include "perls-common/bot_util.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"

#include "perls-lcmtypes++/perllcm/auv_abort_t.hpp"
#include "perls-lcmtypes++/perllcm/auv_jump_t.hpp"


int main (int argc, char **argv)
{
    lcm::LCM lcm;
    if (!lcm.good ()) {
        std::cerr << "lcm not good!" << std::endl;
        exit (EXIT_FAILURE);
    }

    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);

    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "abort/jump iver script.");
    getopt_add_help (gopt, 0);
    getopt_add_bool (gopt, 'a', "abort", 0, "abort iver");
    getopt_add_int (gopt, 'j', "jump", "-1", "jump iver to waypoint number");

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=1) {
        getopt_do_usage (gopt, "vehicle");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    std::string vehicle_name = static_cast<char*>(g_ptr_array_index (gopt->extraargs, 0));
    boost::algorithm::to_lower (vehicle_name);

    bool abort = getopt_get_bool (gopt, "abort");
    int jump_waypoint = getopt_get_int (gopt, "jump");
    bool jump = (jump_waypoint > 0);
    if ((abort && jump) || (!abort && !jump)) {
        std::cerr << "must specify jump OR abort" << std::endl;
        exit (EXIT_FAILURE);
    }

    std::map<std::string, int> net;
    int n_nodes = bot_param_get_num_subkeys (param, "acomms_network");
    for (int i=1;i<=n_nodes;++i) {
        std::string key = "acomms_network.vehicle" + boost::lexical_cast<std::string>(i);

        int id;
        char *char_name;
        if (bot_param_get_int (param, (key+".id").c_str (), &id) ||
            bot_param_get_str (param, (key+".name").c_str (), &char_name) ) {
            std::cerr << "error parsing acomms_network!!!" << std::endl;
            exit (EXIT_FAILURE);
        }
        std::string name = char_name;
        free (char_name);
        boost::algorithm::to_lower (name);
        net[name] = id;
    }

    int node_id;
    std::map<std::string,int>::iterator it = net.find (vehicle_name);
    if (it == net.end ()) {
        std::cout << "Vehicle " << vehicle_name << " not found in network config!" << std::endl;
        for (std::map<std::string,int>::iterator i = net.begin ();i!=net.end ();++i)
            std::cout << i->first << std::endl;
    }
    else
        node_id = it->second;

    if (abort) {
        perllcm::auv_abort_t msg = {0};
        msg.utime = timestamp_now ();
        msg.dest = node_id;
        msg.abort = perllcm::auv_abort_t::ABORT_HARD;
        lcm.publish ("TOPSIDE_SEND_ABORT", &msg);
        std::cout << "sent abort command for " << vehicle_name << "!" << std::endl;
    }
    else if (jump) {
        perllcm::auv_jump_t msg = {0};
        msg.utime = timestamp_now ();
        msg.dest = node_id;
        msg.next_wypnt = jump_waypoint;
        lcm.publish ("TOPSIDE_SEND_JUMP", &msg);
        std::cout << "sent jump to waypoint " << jump_waypoint 
            << " command for " << vehicle_name << "!" << std::endl;
    }
        
    exit (EXIT_SUCCESS);
}

