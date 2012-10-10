#include <iostream>
#include <algorithm>
#include <string>
#include <boost/lexical_cast.hpp>

#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>

#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"

#include "perls-lcmtypes++/perllcm/auv_iver_state_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_range_t.hpp"

#include "lbl.h"

#define UTIME_WINDOW 300*1e6         // median filter window
#define MEDIAN_THRESHOLD 200.0/1500  // threshold on deviation from median in seconds of round trip time

class State
{
  public:
    State (getopt_t *gopt, int64_t ut_window, double med_thresh);
    ~ State () {delete[] _seed_xy; bot_param_destroy(_param);}

    void do_work ();
    bool done () {return _done;}

  private:
    BotParam *_param;

    perls::Lbl_nav _lbl;

    lcm::LCM _lcm;
    void range_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_range_t* msg);
    void iver_state_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const perllcm::auv_iver_state_t* msg);

    BotGPSLinearize *_llxy;
    double _seed_xy[2];
    double _depth;

    bool _done;
};

State::State (getopt_t *gopt, int64_t ut_window, double med_thresh)
    : _lbl (ut_window, med_thresh), _lcm (), _llxy (new BotGPSLinearize), _done (false)
{
    _param = botu_param_new_from_getopt_or_fail (gopt, _lcm.getUnderlyingLCM());
    std::string chan = bot_param_get_str_or_fail (_param, "sensors.modem.gsd.channel");
    chan += "_RANGE";
    _lcm.subscribe (chan, &State::range_callback, this);

    chan = bot_param_get_str_or_fail (_param, "vehicle.state_channel");
    _lcm.subscribe (chan, &State::iver_state_callback, this);

    double orglatlon[2] = {0};
    bot_param_get_double_array_or_fail (_param, "site.orglatlon", orglatlon, 2);
    bot_gps_linearize_init (_llxy, orglatlon);

    _seed_xy[0] = 0;
    _seed_xy[1] = 0;
    _depth = 0;

    int nbeacons = bot_param_get_num_subkeys (_param, "lbl.network");
    for (int i=0;i<nbeacons;++i) {
        std::string key = "lbl.network.beacon" + boost::lexical_cast<std::string>(i) + ".latlon";
        std::string keyd = "lbl.network.beacon" + boost::lexical_cast<std::string>(i) + ".depth";

        double latlon[2] = {0}, yx[2] = {0};
        bot_param_get_double_array_or_fail (_param, key.c_str (), latlon, 2);
        bot_gps_linearize_to_xy (_llxy, latlon, yx);
        std::vector<double> xy (yx,yx+2);
        std::reverse (xy.begin (), xy.end ());
        
        double depth = bot_param_get_double_or_fail (_param, keyd.c_str ());
        
        _lbl.add_beacon (xy, depth);
    }
}

void State::range_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::acomms_range_t* msg)
{
    if (msg->type != senlcm::acomms_range_t::NARROWBAND_LBL) return;
    _lbl.compute_fix (msg->utime, msg->owtt, _seed_xy, _depth);
}

void State::iver_state_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const perllcm::auv_iver_state_t* msg)
{
    _seed_xy[0] = msg->position.xyzrph[0];
    _seed_xy[1] = msg->position.xyzrph[1];
    _depth = msg->position.xyzrph[2];
}

void State::do_work ()
{
    if (_lcm.handle ()) _done = true;;
}

int main (int argc, char *argv[])
{
    /* getopt/param stuff */
    getopt_t *gopt = getopt_create ();
    getopt_add_help (gopt, NULL);
    botu_param_add_pserver_to_getopt (gopt);

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }
    
    State state (gopt, UTIME_WINDOW, MEDIAN_THRESHOLD);
    while (!state.done ()) {
        state.do_work ();
    }

    /* clean up */
    getopt_destroy (gopt);
    exit (EXIT_SUCCESS);
}
