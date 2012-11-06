
#include <signal.h>
#include "main_control.hpp"
//#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"

using namespace acfrlcm;

int main_exit;



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
    
