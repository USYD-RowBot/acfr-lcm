// post process version of the ACFR nav routine for LCM
// Christian Lees
// 20/9/2010

#include "acfrNav.hpp"

// global variables
char *slamDepthFileName;
char *motorSpeedsFileName;
int motorSpeedNumber;
char *logFileName;

void printUsage() {
    // output the program usage
    cout << "lcm2raw [options] <LCM log file>\n";
    cout << "Options:   -m <Motor speed file>\n";
    cout << "           -n <Motor speed number>\n";
    cout << "           -d <Slam depth file>\n";
    
}

int parseArguments(int argc, char **argv) {
    if(argc <= 1) {
        cout << "Incorrect number of aurguments\n";
        printUsage();
        return 0;
    }

    logFileName = NULL;
    motorSpeedsFileName = NULL;
    slamDepthFileName = NULL;
    
    int c;
    while((c = getopt(argc, argv, "m:n:d:")) != -1)
        switch(c) {
            case 'm':
                motorSpeedsFileName = optarg;
                break;
            case 'n':
                motorSpeedNumber = atoi(optarg);
                break;
           	case 'd':
            	slamDepthFileName = optarg;
            	break;        
        }
    logFileName = argv[optind];
    return 1;
}

int main(int argc, char **argv) {

    bool useIverPropCount;
    
    if(parseArguments(argc, argv)) {
        acfrNav nav;
        char url[256];
        sprintf(url, "file://%s?speed=0", logFileName);
        if(motorSpeedsFileName == NULL)
            useIverPropCount = false;
        else
            useIverPropCount = true;
        
        lcm_t *lcm = lcm_create(url);
            
        nav.initialise(lcm, NULL, slamDepthFileName, motorSpeedsFileName, motorSpeedNumber,
        0,                  // update rate
        false,              // save poses
        true,               // use os compass
        false,              // lcm publish
        true,               // use IMU
        true,               // use Parosci
        true,               // use YSI
        true,               // use RDI
        useIverPropCount,   // use prop count
        true);              // process to raw

        while (!lcm_handle (lcm));
 
        lcm_destroy(lcm);
    }

    return 0;
}
