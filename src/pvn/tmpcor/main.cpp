#include <sys/stat.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "tmpcor.h"

using namespace std;

perllcm::pvn_conditions_t
PVNTempCor::read_conditions (std::string file) {
    
    perllcm::pvn_conditions_t conds = {0};
    
    ifstream cf;
    cf.open (file.c_str()); 
    
    if (!cf.is_open()) {   
        ERROR ("Could not open conditions file: %s", file.c_str());
        // set all to unknown values
        conds.sky = perllcm::pvn_conditions_t::SKY_UNKNOWN;
        conds.tod = perllcm::pvn_conditions_t::TOD_UNKNOWN;
        conds.snow = -1;
        conds.rain = -1;
        conds.foliage = -1;
        return conds;
    }
    
    std::string line;
    double percip_1hr = 0.0, temp_c = 0.0;
    double sunset_time = 0.0, sunrise_time = 0.0, cur_time = 0.0;
    
    while (cf.good()) {
        getline (cf,line);
        
        if (0 == line.length())
            continue;
        
        std::string key = line.substr(0, line.find_first_of(": "));
        int chunk_start = line.find_first_of(":");
        std::string chunk = line.substr(chunk_start+1, line.length()-chunk_start-1);
        int key_start = chunk.find_first_not_of (" ");
        std::string value = chunk.substr(key_start, line.length()-key_start);
        
        if (0 == key.compare ("weather")) {
            
            if (0 == value.compare ("Clear")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_CLEAR;
                conds.rain = 0;
            } else if (0 == value.compare ("Scattered Clouds")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_CLEAR;
                conds.rain = 0;
            } else if (0 == value.compare ("Partly Cloudy")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_PARTLY_CLOUDY;
                conds.rain = 0;
            } else if (0 == value.compare ("Haze")) { 
                conds.sky = perllcm::pvn_conditions_t::SKY_FOG;
                conds.rain = 0;
            } else if (0 == value.compare ("Fog")) { 
                conds.sky = perllcm::pvn_conditions_t::SKY_FOG;
                conds.rain = 0;    
            } else if (0 == value.compare ("Mostly Cloudy")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_MOSTLY_CLOUDY;
                conds.rain = 0;
            } else if (0 == value.compare ("Overcast")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 0;
            } else if (0 == value.compare ("Rain")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 1;
            } else if (0 == value.compare ("Light Rain")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 1;
            } else if (0 == value.compare ("Light Thunderstorms and Rain")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 1;
            } else if (0 == value.compare ("Light Freezing Rain")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 1;
            } else if (0 == value.compare ("Thunderstorms and Rain")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 1;
            } else if (0 == value.compare ("Thunderstorm")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 1;
            } else if (0 == value.compare ("Heavy Thunderstorms and Rain")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 1;
            } else if (0 == value.compare ("Light Drizzle")) {
                conds.sky = perllcm::pvn_conditions_t::SKY_OVERCAST;
                conds.rain = 1;
            } else {
                conds.sky = perllcm::pvn_conditions_t::SKY_UNKNOWN;
                ERROR ("Unrecognized weather type: %s", value.c_str());
            }

        } else if (0 == key.compare ("temp_c")) {
            temp_c = boost::lexical_cast<double> (value);
        } else if (0 == key.compare ("precip_1hr")) {
            percip_1hr = boost::lexical_cast<double> (value);
        } else if (0 == key.compare ("cur_hr")) {
            cur_time += boost::lexical_cast<double> (value);
        } else if (0 == key.compare ("cur_mn")) {
            cur_time += (boost::lexical_cast<double> (value))/60.0;
        } else if (0 == key.compare ("sunset_hr")) {
            sunset_time += boost::lexical_cast<double> (value);
        } else if (0 == key.compare ("sunset_mn")) {
            sunset_time += (boost::lexical_cast<double> (value))/60.0;
        } else if (0 == key.compare ("sunrise_hr")) {
            sunrise_time += boost::lexical_cast<double> (value);
        } else if (0 == key.compare ("sunrise_mn")) {
            sunrise_time += (boost::lexical_cast<double> (value))/60.0;
        }
        
    }

    double day_start = (sunrise_time + 0.5);
    double day_end = (sunset_time - 0.5);
    double day_seg_dur = (day_end - day_start)/3.0;
    
    // calculate the time of day
    if (fabs(cur_time - sunrise_time) <= 0.5) {
        conds.tod = perllcm::pvn_conditions_t::TOD_DAWN;
    } else if (fabs(cur_time - sunset_time) <= 0.5) {
        conds.tod = perllcm::pvn_conditions_t::TOD_DUSK;
    } else if (cur_time < (sunrise_time - 0.5) || cur_time > (sunset_time + 0.5)) {
        conds.tod = perllcm::pvn_conditions_t::TOD_NIGHT;
    } else if (cur_time > day_start && cur_time <= (day_start+day_seg_dur)) {    
        conds.tod = perllcm::pvn_conditions_t::TOD_MORNING;
    } else if (cur_time > (day_start+day_seg_dur) && cur_time <= (day_end-day_seg_dur)) {    
        conds.tod = perllcm::pvn_conditions_t::TOD_MIDDAY;
    } else if (cur_time > (day_end-day_seg_dur) && cur_time <= day_end) {    
        conds.tod = perllcm::pvn_conditions_t::TOD_AFTERNOON;
    } else {
        ERROR ("Failed to determine TOD: %lf %lf %lf", cur_time, sunrise_time, sunset_time);
    }
    
    // do we have percipitation
    if (conds.rain == 0) { // if we already set it
        if (percip_1hr >= 0.1) {
            if (temp_c >= 0.0) {
                conds.rain = 1;
                conds.snow = 0;
            } else {
                conds.rain = 0;
                conds.snow = 1;
            }
        } else {
            conds.rain = 0;
            conds.snow = 0;
        }
    }
    conds.foliage = -1; //unknown
    
    cf.close();
    return conds;
    
}

vector<int64_t> 
PVNTempCor::read_utimes (char *dir) {
    
    vector<int64_t> utimes;
    
    char futimes[PATH_MAX];
    sprintf (futimes, "%s/utimes.txt", dir);
    ifstream f;
    f.open (futimes);
    string line;
    if (f.is_open()) {
        
        while ( f.good() ) {
            getline (f, line);
    
            int64_t utime;
            sscanf (line.c_str(), "%ld", &utime);            
            utimes.push_back(utime);
        }
        f.close();
        
    } else {
        ERROR ("Failed to open %s!", futimes);
        
    }
    
    return utimes;
}

bool
dir_exists (char *dir) {
    
    struct stat sb;

    if (stat(dir, &sb) == 0 && S_ISDIR(sb.st_mode))
        return true;
    else
        return false;
}

int
PVNTempCor::count_num_cams (char *dir) {
    
    int i=0;
    char tmp[PATH_MAX];
    do {
        sprintf (tmp, "%s/%d", dir, i);
        i++;
    } while (dir_exists (tmp));
    
    return i-1;
}



/**
 * @brief PVNMapBuilder Constructor
 */
PVNTempCor::PVNTempCor (int argc, char *argv[])
{
    // open param cfg file
    param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        std::cout << "could not find " << BOTU_PARAM_DEFAULT_CFG << std::endl;
    }
    std::cout << "Opened Param File [" << BOTU_PARAM_DEFAULT_CFG << "]" << std::endl;
    
    // read command line args
    gopt = getopt_create ();
    
    getopt_add_description (gopt, "Temporal Correlation Development Program");
    getopt_add_string (gopt,  'p',  "preprocess",      ".",                "Path to webcam data to preprocess");
    getopt_add_string (gopt,  'w',  "webcam_process",  ".",                "Path to webcam data to process");
    getopt_add_int    (gopt,  'e',  "every_nth",       "1",                "Use every nth image");
    getopt_add_int    (gopt,  'r',  "random_step",     "0",                "Use random step size between 1 and r hours");
    getopt_add_spacer (gopt, "------------------------------------------------");
    getopt_add_bool   (gopt,  'h',  "help",    	        0,                 "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);
    }
    
    every_nth = getopt_get_int (gopt, "every_nth");
    random_step = getopt_get_int (gopt, "random_step");
    
    // check lcm connection
    if(!lcm.good()) {
        is_done = 1;
        ERROR ("lcm_create () failed!");
    }
}

PVNTempCor::~PVNTempCor ()
{
    bot_param_destroy (param);
}

// ----------------------------------------------------------------------------
// Main Loop
// ----------------------------------------------------------------------------
void
PVNTempCor::run ()
{
    
    if (!getopt_has_flag (gopt, "webcam_process") &&
        !getopt_has_flag (gopt, "preprocess")) {
        cout << "No job selected ... exiting." << endl;
        exit (EXIT_SUCCESS);
    }
    
    if (getopt_has_flag (gopt, "preprocess")) {
        char tmp[PATH_MAX];
        strcpy (tmp, getopt_get_string (gopt, "preprocess"));
        run_preprocess (tmp);
    }
    
    if (getopt_has_flag (gopt, "webcam_process")) {
        char tmp[PATH_MAX];
        strcpy (tmp, getopt_get_string (gopt, "webcam_process"));
        run_webcam_process (tmp);
    }
    
}
    

// ----------------------------------------------------------------------------
// Ctrl+C catching
// ----------------------------------------------------------------------------
PVNTempCor *g_tmpcor;

void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    std::cout << "Sigint caught. Quitting ..." << std::endl;
    if (g_tmpcor->is_done ) {
        std::cout << "Goodbye" << std::endl;
        exit (EXIT_FAILURE);
    } 
    else
        g_tmpcor->is_done = 1;
}


// ----------------------------------------------------------------------------
// Main 
// ----------------------------------------------------------------------------
int
main (int argc, char *argv[])
{    
    fasttrig_init ();
    
    PVNTempCor tmpcor (argc, argv);
    g_tmpcor = &tmpcor;
    
    // install custom signal handler
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    // kick off the manager
    tmpcor.run ();
    
    return 0;     
}
