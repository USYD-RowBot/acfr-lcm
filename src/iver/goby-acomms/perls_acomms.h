#ifndef __PERLS_ACOMMS_H__
#define __PERLS_ACOMMS_H__

#include <goby/acomms/amac.h>
#include <goby/acomms/bind.h>
#include <goby/acomms/connect.h>
#include <goby/acomms/dccl.h>
#include <goby/acomms/modem_driver.h>
#include <bot_param/param_client.h>

#include "perls-common/timestamp.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/capture_iver_state_t.h"
#include "perls-lcmtypes/capture_perls_mini_t.h"
#include "perls-lcmtypes/capture_perls_abort_t.h"
#include "perls-lcmtypes/capture_perls_jump_t.h"

#include "protodefs/perls_subsea_state.pb.h"
#include "protodefs/perls_modem_mini2.pb.h"

// short hand namespaces
namespace ga = goby::acomms;
namespace gap = goby::acomms::protobuf;
namespace pp = perls::protobuf;

class Node {
    private:
        lcm_t &_lcm;            

        std::ofstream _mac_log;
        std::ofstream _driver_log;

        ga::MACManager _mac;
        ga::MMDriver _driver;

        capture_iver_state_t _lcm_state;

        int8_t _my_id;
        std::string _chan_prefix;
        std::string _abort_chan;
        std::string _jump_chan;

        bool _is_running;            

        void handle_received (const gap::ModemDataTransmission &data_msg);
        void handle_data_request (const gap::ModemDataRequest &reguest_msg,
            gap::ModemDataTransmission *data_msg);
        void handle_ranging (const gap::ModemRangingReply &msg);
        void publish_incoming (const gap::ModemMsgBase &msg_data);
        void publish_outgoing (const gap::ModemMsgBase &msg_data);
    public:
        Node (lcm_t &lcm, gap::DriverConfig &driver_cfg, 
            gap::MACConfig &mac_cfg, const std::string &chan_prefix);
        ~Node();

        // setters
        void set_lcm_state (capture_iver_state_t new_state) {
            _lcm_state = new_state;
        }
        void set_my_id (int8_t modem_id) {_my_id = modem_id;}
        void set_abort_chan (std::string chan) {
            _abort_chan = chan;
        }
        void set_jump_chan (std::string chan) {
            _jump_chan = chan;
        }

        // getters
        capture_iver_state_t get_lcm_state () {return _lcm_state;}
        int8_t get_my_id () {return _my_id;}

        // running methods
        bool is_running () const { return _is_running;}
        void run ();
        void stop () { _is_running = false;}

        // publish lcm commands
        void publish_lcm_abort (uint32_t abort_num) {
            capture_perls_abort_t cmd = {0};
            cmd.utime = timestamp_now ();
            cmd.dest = _my_id;
            cmd.abort = 1;
            capture_perls_abort_t_publish (&_lcm, _abort_chan.c_str(), &cmd);
        }
        void publish_lcm_jump (uint32_t wypnt) {
            capture_perls_jump_t cmd = {0};
            cmd.utime = timestamp_now ();   
            cmd.dest = _my_id;
            cmd.next_wypnt = wypnt;
            capture_perls_jump_t_publish (&_lcm, _jump_chan.c_str(), &cmd);
        }
        void publish_lcm_mini (uint32_t src, uint32_t dest, uint32_t msg) {
            capture_perls_mini_t mini = {0};
            mini.utime = timestamp_now ();
            mini.src   = src;
            mini.dest  = dest;
            mini.user  = msg;

            std::string chan = _chan_prefix + "ACOMMS_MINI";
            capture_perls_mini_t_publish (&_lcm, chan.c_str (), &mini);
        }
        void publish_iver_state (pp::subsea_state state, int src_id) {
            // pack lcm message from dccl
            capture_iver_state_t msg = {0};
            msg.utime           = timestamp_now ();
            msg.latitude        = state.latitude ();
            msg.longitude       = state.longitude ();
            msg.heading         = state.heading ();
            msg.depth           = state.depth ();
            msg.altitude        = state.altitude ();
            msg.next_wypnt      = state.next_waypoint ();
            msg.dist_nwp        = state.dist_to_next_waypoint ();
            msg.batt_percent    = state.battery_percent ();
            msg.error           = state.error_state ();
            msg.aborted_mission = state.abort_state ();
            msg.modem_id        = src_id;

            std::string chan = _chan_prefix + "ACOMMS_STATE";
            capture_iver_state_t_publish (&_lcm, chan.c_str (), &msg);
        }

        pp::subsea_state state_lcm2dccl () {
            pp::subsea_state dccl_state;                

            dccl_state.set_latitude (_lcm_state.latitude);
            dccl_state.set_longitude (_lcm_state.longitude);
            dccl_state.set_heading (_lcm_state.heading);
            dccl_state.set_depth (_lcm_state.depth);
            dccl_state.set_altitude (_lcm_state.altitude);
            dccl_state.set_next_waypoint (_lcm_state.next_wypnt);
            dccl_state.set_dist_to_next_waypoint (goby::util::as<int32_t>(_lcm_state.dist_nwp));
            dccl_state.set_battery_percent (_lcm_state.batt_percent);
            dccl_state.set_error_state (_lcm_state.error);
            dccl_state.set_abort_state (_lcm_state.aborted_mission);

            return dccl_state;
        }   
};

#endif //__PERLS_ACOMMS_H__
