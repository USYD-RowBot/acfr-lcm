#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include "perls-math/fasttrig.h"
#include "perls-common/error.h"
#include "perls-common/lcm_util.h"

#include "SonarClient.h"

// HAUV Channel names
#define RNV_CHANNEL     "HAUV_BS_RNV_2"
#define PIT_CHANNEL     "HAUV_BS_PIT"
#define CNV_CHANNEL     "HAUV_BS_CNV"
#define DVL_CHANNEL     "HAUV_BS_DVL_2"
#define DIDSON_CHANNEL  "HAUV_DIDSON_FRAME"
//#define PLINK_CHANNEL   "HAUV_BS_RNV_2"
//#define GOTO_CHANNEL    "HAUV_BS_RNV_2"
//#define CMD_CHANNEL     "HAUV_BS_RNV_2"

//
// Mouse handler
//
void my_mouse_callback(int event, int x, int y, int flags, void* param)
{
    SonarClient* sonar = reinterpret_cast<SonarClient*>(param);

    switch( event) {
    case CV_EVENT_MOUSEMOVE:
        break;
    case CV_EVENT_LBUTTONDOWN:
    {
        std::cout << "mouse down" << std::endl;
        sonar->onMouseDown(event, x, y, flags);

        break;
    }
    case CV_EVENT_LBUTTONUP:
        break;
    }
}

int
main (int argc, char *argv[])
{
    // lcm
    lcm::LCM lcm;
    if (!lcm.good())
        ERROR ("ERROR: NULL lcm init.");

    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    fasttrig_init();

    SonarClient *client = new SonarClient (lcm);

    cvNamedWindow( "SonarImage", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("SonarImage",10,10);
    cvNamedWindow( "DisplayImage", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("DisplayImage",10,400);
    cvNamedWindow( "MatchImage", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("MatchImage",10,800);
    cvNamedWindow( "RegistrationImage", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("RegistrationImage",350,10);
    cvNamedWindow( "TargetsImage", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("TargetsImage",550,10);

    cvInitFont(&(client->g_font), CV_FONT_HERSHEY_PLAIN, 1.0, 1.0);
    cvSetMouseCallback("DisplayImage", my_mouse_callback, &client);
    cvSetMouseCallback("SonarImage", my_mouse_callback, &client);

    // subscribe lcm channels
    lcm.subscribe(RNV_CHANNEL, &SonarClient::hauv_bs_rnv_2_t_callback, client);
    lcm.subscribe(PIT_CHANNEL, &SonarClient::hauv_bs_pit_t_callback, client);
    lcm.subscribe(CNV_CHANNEL, &SonarClient::hauv_bs_cnv_t_callback, client);
    lcm.subscribe(DVL_CHANNEL, &SonarClient::hauv_bs_dvl_2_t_callback, client);
    lcm.subscribe(DIDSON_CHANNEL, &SonarClient::hauv_didson_t_callback, client);

    lcm.subscribe (client->m_add_node_ack_channel, &SonarClient::add_node_ack_t_callback, client);
    lcm.subscribe (client->m_isam_rt_st_channel, &SonarClient::return_state_t_callback, client);
    lcm.subscribe (client->m_isam_cmd_channel, &SonarClient::isam_cmd_t_callback, client);
    lcm.subscribe (client->m_wp_goto_channel, &SonarClient::wp_goto_t_callback, client);

    boost::thread registrationThread(&SonarClient::processRegistrations, client);

    client->run();

    registrationThread.join();

    cvDestroyAllWindows();

    return 0;   
}
