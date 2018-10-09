#include <string>
#include <signal.h>
#include <libgen.h>
#include <iostream>
#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include "bot_param/param_client.h"
#include <math.h>
#include "acfr-common/timestamp.h"

#include "perls-lcmtypes++/senlcm/micron_ping_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/senlcm/oa_t.hpp"

using namespace std;

class OAProcessor 
{
    public:
        OAProcessor(std::string rk, std::string vn);
        ~OAProcessor();
        void process();

    private:
        lcm::LCM lcm;
        BotParam *param;
        string vehicle_name;
        string root_key;
        
        senlcm::micron_ping_t mr;
        acfrlcm::auv_acfr_nav_t nav;
        void acfr_nav_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
            const acfrlcm::auv_acfr_nav_t *nav); 
        void micron_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
            const senlcm::micron_ping_t *mr); 

        int micron_calc();

        // Micron variables
        double sumPseudoAlt;
        double sumPseudoFwdDistance;
        int numPseudoAlt;
        int numPseudoFwdDistance;
        int currentScanDirn;
        double lastAngle;
        double minimum_range;
        double micron_threshold;
        double pitch_offset;
        double x_offset;
        double z_offset;
};

