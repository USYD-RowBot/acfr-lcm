// Process a LCM log file pulling out the ACFR_AUV_VIS_RAWLOG messages
// and writing an ACFR RAW log with the VIS entries

#include "../acfr-nav-new/handlers.hpp"
#include <lcm/lcm-cpp.hpp>



using namespace auv_data_tools;
using namespace std;
using namespace lcm;
using namespace acfrlcm;

               
int main(int argc, char **argv) {
    if(argc < 3)
    {
        cerr << "Usage: lcm2raw: [lcm log file] [output raw file]" << endl;
        return 1;
    }

    state_c *state = new state_c;
    
    
    // open the files
    state->raw_out.open(argv[2], ofstream::out);
    if(!state->raw_out.is_open()) {
        cerr << "Could not open input file: " << argv[2] << endl;
        return 1;
    }
    
    state->mode = RAW;
    
    string lcm_file ="file://"+string(argv[1])+"?speed=0";
    state->lcm = new lcm::LCM(lcm_file);
    
    // subscribe to the channels
    state->lcm->subscribeFunction("GPSD_CLIENT", on_gps, state);
    state->lcm->subscribeFunction("TCM", on_tcm_compass, state);
    state->lcm->subscribeFunction("OS_COMPASS", on_os_compass, state);
    state->lcm->subscribeFunction("YSI", on_ysi, state);
    state->lcm->subscribeFunction("PAROSCI", on_parosci, state);
    state->lcm->subscribeFunction("SEABIRD", on_seabird_depth, state);
    state->lcm->subscribeFunction("RDI", on_rdi, state);
    state->lcm->subscribeFunction("IMU", on_imu, state);
    state->lcm->subscribeFunction("ACFR_AUV_VIS_RAWLOG", on_vis, state);
     
    cout << "Done subscribing" << endl;
    
    // run the log
    int64_t events = 0;
    while (0 == state->lcm->handle()) {
        if ((++events % 200000)==0)
            printf (".");
    };
    
    return 0;
}    
