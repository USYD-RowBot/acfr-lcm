// LCM to RAW log converter

#include "acfr_nav.hpp"
#include "handlers.hpp"

        

void printUsage() {
    // output the program usage
    cout << "lcm2raw [options] <LCM log file>\n";
    cout << "Options:   -m <Motor speed file>\n";
    cout << "           -n <Motor speed number>\n";
    cout << "           -d <Slam depth file>\n";
    
}
/*
int parseArguments(int argc, char **argv, char *logFileName) {
    if(argc <= 1) {
        cout << "Incorrect number of aurguments\n";
        printUsage();
        return 0;
    }


    
    int c;
    while((c = getopt(argc, argv, "m:n:d:")) != -1)
        switch(c) {
            case 'm':
                //motorSpeedsFileName = optarg;
                break;
            case 'n':
                //motorSpeedNumber = atoi(optarg);
                break;
           	case 'd':
            	//slamDepthFileName = optarg;
            	break;        
        }
    
    logFileName = argv[optind];
    return 1;
}
*/

int main(int argc, char **argv) {

    char *logFileName;
//    if(parseArguments(argc, argv, logFileName)) {
       if(argc != 2)
        {
            cout << "Usage: lcm2raw [lcm log file] [raw file]" << endl;
            return 0;
        }
    
        char url[256];
        sprintf(url, "file://%s?speed=0", argv[1]);
        cout << url << endl;
        
        state_c state;
        state.lcm = new lcm::LCM(url);
        state.mode = RAW;
        state.raw_out.open(argv[2]);
        
        // subscribe to all the channels
        state.lcm->subscribeFunction("GPSD_CLIENT", on_gps, &state);
        state.lcm->subscribeFunction("TCM", on_tcm_compass, &state);
        state.lcm->subscribeFunction("OS_COMPASS", on_os_compass, &state);
        state.lcm->subscribeFunction("YSI", on_ysi, &state);
        state.lcm->subscribeFunction("PAROSCI", on_parosci, &state);
        state.lcm->subscribeFunction("SEABIRD", on_seabird_depth, &state);
        state.lcm->subscribeFunction("RDI", on_rdi, &state);
        state.lcm->subscribeFunction("IMU", on_imu, &state);
        state.lcm->subscribeFunction("LQ_MODEM", on_lq_modem, &state);
        state.lcm->subscribeFunction("ACFR_AUV_VIS_RAWLOG", on_vis, &state);
        
        
        while(!state.lcm->handle());
        delete state.lcm;
//    }

    return 1;
}