#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "perls-math/fasttrig.h"
#include "perls-common/error.h"
#include "perls-common/lcm_util.h"

#include "HauvClient.h"

// HAUV Channel names
#define RNV_CHANNEL   "HAUV_BS_RNV_2"
#define PIT_CHANNEL   "HAUV_BS_PIT"
#define CNV_CHANNEL   "HAUV_BS_CNV"
#define DVL_CHANNEL   "HAUV_BS_DVL_2"
#define HB_2_CHANNEL  "PROSILICA_M.SYNC"
#define HB_5_CHANNEL  "HEARTBEAT_5HZ"

int
main (int argc, char *argv[])
{
    fasttrig_init();

    // lcm
    lcm::LCM lcm;
    if (!lcm.good())
        ERROR ("ERROR: NULL lcm init.");

    HauvClient *client = new HauvClient(lcm);

    // telemetry lcm channels
    lcm.subscribe(RNV_CHANNEL, &HauvClient::hauv_bs_rnv_2_t_callback, client);
    lcm.subscribe(PIT_CHANNEL, &HauvClient::hauv_bs_pit_t_callback, client);
    lcm.subscribe(CNV_CHANNEL, &HauvClient::hauv_bs_cnv_t_callback, client);
    lcm.subscribe(DVL_CHANNEL, &HauvClient::hauv_bs_dvl_2_t_callback, client);

#if 0
    //For testing purposes (add new nodes at 2Hz, no matter what)
    lcm.subscribe(HB_2_CHANNEL, &HauvClient::perllcm_heartbeat_t_2_callback, client);
#endif

    //For check for queued nodes.  The queued nodes will be send to isam-server
    lcm.subscribe(HB_5_CHANNEL, &HauvClient::perllcm_heartbeat_t_5_callback, client);

    // add node
    //For adding nodes from other processes (rtvan, sonar, etc.)
    lcm.subscribe(client->m_rtvan_add_node_channel, &HauvClient::perllcm_rtvan_add_node_callback, client);
    lcm.subscribe(client->m_sonar_add_node_channel, &HauvClient::perllcm_sonar_add_node_callback, client);
    lcm.subscribe(client->m_self_add_node_channel, &HauvClient::perllcm_self_add_node_callback, client);

    // waypoint navigation
    lcm.subscribe(client->m_wp_save_channel, &HauvClient::hauv_wp_save_t_callback, client);
    lcm.subscribe(client->m_rq_goto_channel, &HauvClient::hauv_wp_goto_request_t_callback, client);

    // state (for waypoints we can get away with 1Hz)
    lcm.subscribe(client->m_graph_vis_channel, &HauvClient::perllcm_isam_graph_vis_t_callback, client);

    // start button in viewer
    lcm.subscribe(client->m_isam_cmd_channel, &HauvClient::perllcm_isam_cmd_t_callback, client);

    client->run();

    return 0;   
}
