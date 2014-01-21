#include "global_planner.hpp"
#include <signal.h>

int mainExit;

void signalHandler(int sigNum) {
    // do a safe exit
    mainExit = 1;
}


int main(int argv char **argv) {
    // install the signal handler
    mainExit = 0;
    signal(SIGINT, signalHandler);

    // create a global planner object and let it do its thing
    GlobalPlanner gp;
    gp.run();
    
    return 
