// post process version of the ACFR nav routine for LCM
// Christian Lees
// 20/9/2010

#include "acfrNav.hpp"

// global variables
char *slamConfigFileName;
char *magFileName;
char *motorSpeedsFileName;
char *depthTareFileName;
int motorSpeedNumber;
char *logFileName;
float playBackSpeed;
bool saveOnlineSolution;
bool useOSCompass;
int updateRate;
bool useIMU;
bool useParosci;
bool useYsi;
bool useRdi;
bool useIverPropCount;

void printUsage() {
    // output the program usage
    cout << "acfr-nav-post <SLAM config file> <Magnetic variation file>\n";
    cout << "   <Motor speeds file> <Motor speed number> <LCM log file>\n";
    cout << "   <Depth tare file>\n";
    cout << "   Options:    -p <playback speed>, default 32x\n";
    cout << "               -s, save online solution\n";
    cout << "               -h, use the Honeywell HMR3600 compass\n";
    cout << "               -u <update rate>, solution update rate, default 1Hz\n";
    cout << "				-i use IMU\n";
    cout << "				-d use Parosci\n";
    cout << "               -y use YSI for depth\n";
    cout << "               -r use RDI\n";
    cout << "               -c use Iver prop count\n";
}

int parseArguments(int argc, char **argv) {
    if(argc < 5) {
        cout << "Incorrect number of aurguments\n";
        printUsage();
        return 0;
    }

    slamConfigFileName = argv[1];
    magFileName = argv[2];
    motorSpeedsFileName = argv[3];
    motorSpeedNumber = atoi(argv[4]);
    logFileName = argv[5];
    depthTareFileName  = argv[6];

    playBackSpeed = 32;
    saveOnlineSolution = false;
    useOSCompass = true;
    updateRate = 1;
    useYsi = false;
    useParosci = false;
    useIverPropCount = false;

    int c;
    while((c = getopt(argc, argv, "p:su:hirdyc")) != -1)
        switch(c) {
            case 'p':
                playBackSpeed = atof(optarg);
                break;
            case 's':
                saveOnlineSolution = true;
                break;
            case 'u':
                updateRate = atoi(optarg);
                break;
            case 'h':
                useOSCompass = false;
                break;
            case 'i':
            	useIMU = true;
            	break;
           	case 'd':
            	useParosci = true;
            	break;
           	case 'y':
            	useYsi = true;
            	break;
           	case 'r':
            	useRdi = true;
            	break;
           	case 'c':
            	useIverPropCount = true;
            	break;
            	

        }
    return 1;
}

int main(int argc, char **argv) {
    if(parseArguments(argc, argv)) {

        acfrNav nav;
        char url[256];
        sprintf(url, "file://%s?speed=%f", logFileName, playBackSpeed);
        lcm_t *lcm = lcm_create(url);
        nav.initialise(lcm, slamConfigFileName, magFileName, depthTareFileName, motorSpeedsFileName, motorSpeedNumber,
        updateRate, saveOnlineSolution, useOSCompass, false, useIMU, useParosci, useYsi, useRdi, 
        useIverPropCount, false);

        while (!lcm_handle (lcm));
        nav.saveData();

        lcm_destroy(lcm);
    }

    return 1;
}
