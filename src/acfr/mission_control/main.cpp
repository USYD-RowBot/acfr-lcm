
#include <signal.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "main_control.hpp"
#include "perls-lcmtypes++/senlcm/raw_ascii_t.hpp"

using namespace acfrlcm;
using namespace std;

int main_exit;
/*
void on_mp_command(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const senlcm::raw_ascii_t *mp, lcm::LCM* lcm)
{

    vector<string> tokens;
    boost::split(tokens, mp->msg, boost::is_any_of(","));
    
    if(tokens[0] == "RUN")
    {
        acfrlcm::auv_mission_command_t mc;
        mc.utime = mp->utime;
        mc.message = auv_mission_command_t::RUN;
        lcm->publish("MISSION_COMMAND", &mc);
    }
    
    if(tokens[0] == "STOP")
    {
        acfrlcm::auv_mission_command_t mc;
        mc.utime = mp->utime;
        mc.message = auv_mission_command_t::STOP;
        lcm->publish("MISSION_COMMAND", &mc);
    }
    
    if(tokens[0] == "ABORT")
    {
        acfrlcm::auv_mission_command_t mc;
        mc.utime = mp->utime;
        mc.message = auv_mission_command_t::ABORT;
        lcm->publish("MISSION_COMMAND", &mc);
    }
    
    if(tokens[0] == "GOTO")
    {
        if(tokens.size() >= 7)
        {
            acfrlcm::auv_mission_command_t mc;
            mc.utime = mp->utime;
            mc.message = auv_mission_command_t::GOTO;
            mc.x = atof(tokens[1].c_str());
            mc.y = atof(tokens[2].c_str());
            mc.z = atof(tokens[3].c_str());
            mc.xy_vel = atof(tokens[4].c_str());
            mc.z_vel = atof(tokens[5].c_str());
            mc.timeout = atof(tokens[6].c_str());
            lcm->publish("MISSION_COMMAND", &mc);
        }
    }
    
}
*/

void signal_handler(int sig)
{
    main_exit = 1;
}


int main(int argc, char **argv)
{
    main_exit = 0;
    signal(SIGINT, signal_handler);
    
    lcm::LCM lcm;
    
    main_control mc(&lcm);
   
    
    
    int fd = lcm.getFileno();
    fd_set rfds;
    while(!main_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select (fd + 1, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
            lcm.handle();
    }
}
    
