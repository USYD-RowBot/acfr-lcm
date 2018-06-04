// LCM to RAW log converter

#include "acfr_nav.hpp"
#include "handlers.hpp"

        

void printUsage() {
    // output the program usage
    cout << "lcm2raw [options] [LCM log file] [raw file]\n";
    cout << "Options      -b Broken Iver altitude, only on the OSI message" << endl;
}

int parseArguments(int argc, char **argv, state_c *state, string &lcm_filename, string &raw_filename, string &vehicle_name_dot, string &usbl_name) {
    if(argc <= 2) {
        cout << "Incorrect number of aurguments\n";
        printUsage();
        return 0;
    }


    bool usbl_name_set = false;
    
    int c;
    while((c = getopt(argc, argv, "bn:u:i:o:")) != -1)
        switch(c) {
            case 'b':
                state->broken_iver_alt = true;
                break;     
            case 'n':
                vehicle_name_dot = optarg;
                break;     
            case 'u':
                usbl_name = optarg;
		usbl_name_set = true;
                break;     
            case 'i':
                lcm_filename = optarg;
                break;     
            case 'o':
                raw_filename = optarg;
                break;     
        }

    if (!usbl_name_set)
    {
        usbl_name = vehicle_name_dot;
        if (usbl_name.back() == '.')
        {
            usbl_name.pop_back();
        }
    }


    return 1;
}


int main(int argc, char **argv) {

    string lcm_filename;
    string raw_filename;
    string vehicle_name_dot = ".*";
    string usbl_name = ".*";

    state_c state;
    state.broken_iver_alt = false;

    parseArguments(argc, argv, &state, lcm_filename, raw_filename, vehicle_name_dot, usbl_name);
    string  url = string("file://") + lcm_filename + string("?speed=0");

    //sprintf(url, "file://%s?speed=0", lcm_filename);
    //cout << url << endl;


    cout << "Broken Iver alitmeter : " << state.broken_iver_alt << endl;
    cout << "Vehicle name: " << vehicle_name_dot << endl;
    cout << "Usbl name: " << usbl_name << endl;
 
        state.lcm = new lcm::LCM(url);
        state.mode = RAW;
        //state.raw_out.open(argv[2]);
        // open the files
        state.raw_out.open(raw_filename.c_str(), ofstream::out);
        if(!state.raw_out.is_open()) {
            cerr << "Could not open output file: " << raw_filename << endl;
            return 1;
        }
        
        // subscribe to all the channels
        state.lcm->subscribeFunction(vehicle_name_dot+"GPSD_CLIENT", on_gps, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"TCM", on_tcm_compass, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"OS_COMPASS", on_os_compass, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"YSI", on_ysi, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"PAROSCI", on_parosci, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"SEABIRD", on_seabird_depth, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"RDI", on_rdi, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"IMU", on_imu, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"LQ_MODEM", on_lq_modem, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"ACFR_AUV_VIS_RAWLOG", on_vis, &state);
        state.lcm->subscribeFunction(".*USBL_FIX."+usbl_name, on_evologics, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"UVC_DVL", on_uvc_dvl, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"UVC_RPH", on_uvc_rph, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"UVC_OSI", on_uvc_osi, &state);
        state.lcm->subscribeFunction(vehicle_name_dot+"SEABOTIX_SENSORS", on_seabotix_sensors, &state);
        //state.lcm->subscribeFunction(vehicle_name+".MICRON_SOUNDER", on_micron_sounder, &state);
        cout << "Done subscribing" << endl;

        
        
        while(!state.lcm->handle());
        delete state.lcm;
//    }

    return 0;
}
