#include "oa-processor.hpp"

bool program_exit;

OAProcessor::OAProcessor(std::string rk, std::string vn) : root_key(rk), vehicle_name(vn)
{
    // Micron sonar setup
    currentScanDirn = 0;
    numPseudoAlt = 0;
    numPseudoFwdDistance = 0;
    sumPseudoAlt = 0.0;
    sumPseudoFwdDistance = 0.0;
    lastAngle = 0.0;

    // load config
    param = bot_param_new_from_server(this->lcm.getUnderlyingLCM(), 1);
    std::string key = root_key + ".minimum_range";
    minimum_range = bot_param_get_double_or_fail(param, key.c_str());

    key = root_key + ".micron_threshold";
    micron_threshold = bot_param_get_double_or_fail(param, key.c_str());

    key = root_key + ".pitch_offset";
    pitch_offset = bot_param_get_double_or_fail(param, key.c_str());
    
    key = root_key + ".x_offset";
    x_offset = bot_param_get_double_or_fail(param, key.c_str());
    
    key = root_key + ".z_offset";
    z_offset = bot_param_get_double_or_fail(param, key.c_str());

    // subscribe
    this->lcm.subscribe(vehicle_name+".MICRON", &OAProcessor::micron_callback, this);            
    this->lcm.subscribe(vehicle_name+".ACFR_NAV", &OAProcessor::acfr_nav_callback, this);            

}   

OAProcessor::~OAProcessor()
{
}

void OAProcessor::acfr_nav_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
    const acfrlcm::auv_acfr_nav_t *nav)
{
    this->nav = *nav;
}

void OAProcessor::micron_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
    const senlcm::micron_ping_t *mr)
{
    this->mr = *mr;
    micron_calc();
}

int OAProcessor::micron_calc()
{

    int traceStart = (int)(minimum_range / mr.range_resolution);
    double powerdB = 0;
    double current_max = 0;
    int range_index = mr.num_bins;
    double ProfRng = 0.0;
    double pitch;
    int ii;
    double PseudoAlt = 0.0;
    double PseudoFwdDistance = 0.0;


    if(mr.angle > mr.left_limit && mr.angle < mr.right_limit)
    {
        // These values are dependent on the range resolution. Currently set at 0.05m
        // blank out returns from less than the Cfg value recommend 1.0m
        //int traceStart = (MINIMUM_RANGE / (0.5 * micron.range_resolution)) -1;
        //traceStart = (micron.minimum_range / micron.range_resolution);
        for(ii = traceStart; ii < mr.num_bins; ii++)
        {
            powerdB = mr.bins[ii]*80/255;

            if(powerdB > current_max)
            {
                current_max = powerdB;
                range_index = ii;

            }
        }
        // make sure it is over a threshold
        if(current_max > micron_threshold)
        {
            ProfRng = (double)range_index * (double)mr.range_resolution;
                        
            // FIXME: Added a 'hack' to ignore long returns.  This is probably an issue with the indexing of the return.
            if(ProfRng < 0.9*mr.range)
            {
                // for now set the quadrant from 200deg to 225deg (180 is forward) to calculate pseudo altitude
                //             the quadrant from 180deg to 200deg to calculate fwd distance
                // This accounts for the fact that as the angles go to 180 (270) the sin (cos) of the angle goes to zero meaning
                // that the calculated altitude (fwd distance) also go to zero
                // The angular ranges were chosen to avoid returns from the nose
                // This should be more sohpisticated to account for vertical walls, overhangs and the like.  It should also be configurable
                // so that the sonar can be flipped.
                if(mr.angle > 200*M_PI/180)
                {
                    // calculate the current pseudo Altitude.  We will store
                    // and report the minimum pseudo altitude in each sector
                    if(fabs(nav.utime - mr.utime) > 1e6)
		               pitch = 0;
                    else
                        pitch = nav.pitch;
			
		    	    PseudoAlt = (sin(mr.angle - pitch_offset - pitch - M_PI) * ProfRng) - x_offset*sin(pitch) + z_offset*cos(pitch);
                    sumPseudoAlt += PseudoAlt;
                    numPseudoAlt++;
                }
                else
                {
                    // calculate the current pseudo forward distance.  We will
                    // store and report the minimum pseudo forward distance
                    // in each sector
                    PseudoFwdDistance = cos(mr.angle - M_PI) * ProfRng;
                    sumPseudoFwdDistance += PseudoFwdDistance;
                    numPseudoFwdDistance++;
                } // endif calc pseudo altitude else pseudo fwd dist
            } // endif range < 0.9
        } // endif power greater than threshold
    } // endif angle inside margins

    bool sendUpdate = false;
    // Check if we have changed direction and report the average
    // value in each sector to the controller
    if (currentScanDirn == 0 && mr.angle < lastAngle)
    {
        currentScanDirn = 1;
        sendUpdate = true;
    }
    else if (currentScanDirn == 1 && mr.angle > lastAngle)
    {
        currentScanDirn = 0;
        sendUpdate = true;
    }
    lastAngle = mr.angle;

    if (sendUpdate == true)
    {
        double sectorPseudoAlt;
        double sectorPseudoFwdDistance;

        if (numPseudoAlt > 0)
        {
            sectorPseudoAlt = sumPseudoAlt/numPseudoAlt;
        }
        else
        {
            sectorPseudoAlt = mr.range;
        }
        if (numPseudoFwdDistance > 0)
        {
            sectorPseudoFwdDistance = sumPseudoFwdDistance/numPseudoFwdDistance;
        }
        else
        {
            sectorPseudoFwdDistance = mr.range;
        }

        senlcm::oa_t oa;
        oa.utime = timestamp_now();
        oa.forward_distance = sectorPseudoFwdDistance;
        oa.altitude = sectorPseudoAlt;
        lcm.publish(vehicle_name+".OA", &oa);

        numPseudoAlt = 0;
        numPseudoFwdDistance = 0;
        sumPseudoAlt = 0.0;
        sumPseudoFwdDistance = 0.0;

    }

    return 1;
}

void OAProcessor::process()
{
    while(!program_exit)
        lcm.handleTimeout(1000);
}

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

std::string
parse_args (int argc, char **argv)
{
    int opt;

    std::string vehicle_name = "DEFAULT";

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
            case 'h':
                print_help (0, argv);
                break;
            case 'n':
                vehicle_name = std::string((char*)optarg);
                break;
        }
    }
    return vehicle_name;
}
// Exit handler

void signal_handler(int sigNum)
{
    program_exit = true;
}
           

int main(int argc, char **argv)
{
    program_exit = false;
    signal(SIGINT, signal_handler);

    std::string root_key = "acfr."+string(basename(argv[0]));
    std::string vn = parse_args(argc, argv);

    OAProcessor oa(root_key, vn);
    oa.process();

    return 1;
}

