#include <iostream>

#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>

#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#include "perls-lcmtypes++/perllcm/auv_abort_t.hpp"
#include "perls-lcmtypes++/perllcm/auv_jump_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_osp_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_pose_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_request_t.hpp"

#include "perls_acomms.h"
#include "perls_resource_manager.h"

/* 
 *  handling aborts as follows (so we can always send an abort without dccl):
    --- have hard abort 0x1fff
        check sent messages so we know we aren't `accidentally' transmitting a hard abort
        check received messages before decoding to intercept hard aborts
    --- use mini dccl for other aborts/jumps/...
*/

struct State
{
    State (BotParam *param);

    lcm::LCM lcm;
    std::string abort_chan;
    std::string nav_request_chan;
    std::string pose_reply_chan;
    std::string osp_reply_chan;
    std::string osp_recovery_reply_chan;

    int node_id;
    perls::AcommsResManager manager;
    bool is_running;

    std::string tx_rule;

    bool send_abort;
    void send_abort_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
            const perllcm::auv_abort_t* msg);
    bool send_jump;
    void send_jump_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
            const perllcm::auv_jump_t* msg);
    gap::ModemTransmission mini_cmd;

    bool send_recovery;
    senlcm::acomms_request_t recovery_rq;
    void recovery_request_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_request_t* msg);
};

State::State (BotParam *param)
    : manager (perls::AcommsResManager (&lcm, param)), is_running (true),
    send_abort (false), send_jump (false), send_recovery (false)
{
    std::string chan = bot_param_get_str_or_fail (param, "sensors.modem.gsd.channel");
    abort_chan = chan + "_ABORT";
    nav_request_chan = chan + "_NAV_REQUEST";
    pose_reply_chan = chan + "_NAV_POSE_REPLY";
    osp_reply_chan = chan + "_NAV_OSP_REPLY";
    osp_recovery_reply_chan = chan + "_NAV_OSP_RECOVERY_REPLY";

    node_id = bot_param_get_int_or_fail (param, "sensors.modem.id"); 

    // only care about tx_rule if we are the server
    std::string name = bot_param_get_str_or_fail (param, "vehicle.name");
    std::string server = bot_param_get_str_or_fail (param, "owtt-nav.server");
    if (server == name) {
        tx_rule = bot_param_get_str_or_fail (param, "owtt-nav.tx_rule");
    }
    else {
        std::string rrq_chan = chan + "_RECOVERY_REQUEST";
        lcm.subscribe (rrq_chan.c_str (), &State::recovery_request_callback, this);
    }
    
    // subscribe to send abort lcm channel
    if (node_id == 1) {
        lcm.subscribe ("TOPSIDE_SEND_ABORT", &State::send_abort_callback, this);
        lcm.subscribe ("TOPSIDE_SEND_JUMP", &State::send_jump_callback, this);
    }
}

void State::send_abort_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
        const perllcm::auv_abort_t* msg)
{
    if (msg->dest == node_id) 
        return;

    gap::ModemTransmission mini;
    mini.set_type (gap::ModemTransmission::DRIVER_SPECIFIC);
    mini.SetExtension (mp::type, mp::MICROMODEM_MINI_DATA);
    mini.set_src (node_id);
    mini.set_dest (msg->dest);
    mini.set_rate (0);
    mini.set_ack_requested(false);
    manager.encode_abort (&mini, msg);

    send_abort = true;
    mini_cmd = mini;
}

void State::send_jump_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
        const perllcm::auv_jump_t* msg)
{
    if (msg->dest == node_id) 
        return;

    gap::ModemTransmission mini;
    mini.set_type (gap::ModemTransmission::DRIVER_SPECIFIC);
    mini.SetExtension (mp::type, mp::MICROMODEM_MINI_DATA);
    mini.set_src (node_id);
    mini.set_dest (msg->dest);
    mini.set_rate (0);
    mini.set_ack_requested(false);
    manager.encode_jump (&mini, msg);

    send_jump = true;
    mini_cmd = mini;
}

void State::recovery_request_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
        const senlcm::acomms_request_t* msg)
{
    if (msg->type == senlcm::acomms_request_t::OSP_RECOVERY) {
        send_recovery = true;
        recovery_rq = *msg;
    }
}

void data_rx_callback (const gap::ModemTransmission &msg, void *user)
{
    std::cout << "DATA RECEIVED " << msg.DebugString () << std::endl;

    State *state = static_cast<State*>(user);
    state->manager.decode_data (msg);
}

void mini_rx_callback (const gap::ModemTransmission &msg, void *user)
{
    std::cout << "MINI RECEIVED " << msg.DebugString () << std::endl;
    State *state = static_cast<State*>(user);

    // check for hard abort first
    std::string bytes = msg.frame(0);
    if ((static_cast<unsigned char>(bytes[0]) == 0x1f) 
            && (static_cast<unsigned char>(bytes[1]) == 0xff)
            && msg.dest () == state->node_id) {
        perllcm::auv_abort_t abort_msg;
        abort_msg.utime = msg.time ();
        abort_msg.dest = state->node_id;
        abort_msg.abort = perllcm::auv_abort_t::ABORT_HARD;
        state->lcm.publish (state->abort_chan, &abort_msg);
    }
    else {
        state->manager.decode_mini (msg);
    }
}

struct Request_helper
{
    Request_helper (int8_t request_type, int tol_no=0)
        : pose_msg (0), rc_msg (0), osp_msg (0), waiting (true)
    {
        request_msg.type = request_type;
        request_msg.last_tol_no = tol_no;
    }
    ~Request_helper () 
    {
        if (pose_msg) delete pose_msg;
        if (rc_msg) delete rc_msg;
        if (osp_msg) delete osp_msg;
    }

    void waiting_game ()
    {
        std::cout << "waiting for message" << std::endl;
        struct timeval tv;
        tv.tv_sec = 0; 
        tv.tv_usec = 5e5; // wait 0.5s for reply
        while (waiting && tv.tv_usec > 0) {
            lcmu_handle_timeout (lcm.getUnderlyingLCM (), &tv);
        }
        std::cout << "finished listening" << std::endl;
    }

    senlcm::acomms_pose_t* get_pose_reply (const std::string& request_chan, const std::string& reply_chan)
    {
        std::cout << "subscribing to channel" << std::endl;
        lcm::Subscription *sub = lcm.subscribe (reply_chan, &Request_helper::pose_reply, this);

        request_msg.utime = timestamp_now ();
        lcm.publish (request_chan, &request_msg);

        waiting_game ();
        lcm.unsubscribe (sub);
        if (pose_msg) std::cout << "returning reply" << std::endl;
        else std::cout << "returning null, did not receive reply" << std::endl;
        return pose_msg;
    }

    senlcm::acomms_osp_recovery_t* get_osp_recovery_reply (const std::string& request_chan, 
            const std::string& reply_chan)
    {
        std::cout << "subscribing to channel" << std::endl;
        lcm::Subscription *sub = lcm.subscribe (reply_chan, &Request_helper::rc_reply, this);

        request_msg.utime = timestamp_now ();
        lcm.publish (request_chan, &request_msg);

        waiting_game ();
        lcm.unsubscribe (sub);
        if (rc_msg) std::cout << "returning recovery msg" << std::endl;
        else std::cout << "returning null" << std::endl;
        return rc_msg;
    }

    senlcm::acomms_two_osp_t* get_osp_reply (const std::string& request_chan, const std::string& reply_chan)
    {
        std::cout << "subscribing to channel" << std::endl;
        lcm::Subscription *sub = lcm.subscribe (reply_chan, &Request_helper::osp_reply, this);

        request_msg.utime = timestamp_now ();
        lcm.publish (request_chan, &request_msg);

        waiting_game ();
        lcm.unsubscribe (sub);
        if (osp_msg) std::cout << "returning osp_msg" << std::endl;
        else std::cout << "returning null" << std::endl;
        return osp_msg;
    }

    senlcm::acomms_request_t request_msg;
    senlcm::acomms_pose_t *pose_msg;
    senlcm::acomms_osp_recovery_t *rc_msg;
    senlcm::acomms_two_osp_t *osp_msg;

    lcm::LCM lcm;
    bool waiting;

    void pose_reply (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
            const senlcm::acomms_pose_t* msg)
    {
        pose_msg = new senlcm::acomms_pose_t;
        *pose_msg = *msg;
        waiting = false;
    }

    void rc_reply (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
            const senlcm::acomms_osp_recovery_t* msg)
    {
        rc_msg = new senlcm::acomms_osp_recovery_t;
        *rc_msg = *msg;
        waiting = false;
    }

    void osp_reply (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
            const senlcm::acomms_two_osp_t* msg)
    {
        osp_msg = new senlcm::acomms_two_osp_t;
        *osp_msg = *msg;
        waiting = false;
    }
};

void data_request_callback (gap::ModemTransmission *msg, void *user)
{
    std::cout << "DATA REQUEST " << msg->DebugString () << std::endl;
    State *state = static_cast<State*>(user);

    if (state->tx_rule == "independent") { 
        std::cout << "REQUSTING POSE INFO" << std::endl;
        Request_helper rqh (senlcm::acomms_request_t::POSE);
        senlcm::acomms_pose_t *pose_reply = rqh.get_pose_reply (state->nav_request_chan, state->pose_reply_chan);
        if (pose_reply) {
            std::cout << "RECEIVED POSE INFO" << std::endl;
            state->manager.encode_pose_data (msg, pose_reply);
        }
    }
    else if (state->tx_rule == "origin-state" && (msg->rate () == 2 || msg->rate () == 1)) { 
        int recovery_tol_no = state->manager.recovery_request ();
        if (recovery_tol_no) {
            std::cout << "REQUESTING OSP INFO/RECOVERY FROM TOL NO " << recovery_tol_no << std::endl;
            Request_helper rqh (senlcm::acomms_request_t::OSP_RECOVERY, recovery_tol_no);
            senlcm::acomms_osp_recovery_t *osp_reply = rqh.get_osp_recovery_reply (state->nav_request_chan, 
                    state->osp_recovery_reply_chan);
            if (osp_reply) {
                std::cout << "RECEIVED RECOVERY INFO" << std::endl;
                state->manager.encode_osp_recovery_reply (msg, osp_reply);
            }
        }
        else {
            std::cout << "REQUESTING OSP INFO" << std::endl;
            Request_helper rqh (senlcm::acomms_request_t::OSP);
            senlcm::acomms_two_osp_t *osp_reply = rqh.get_osp_reply (state->nav_request_chan, state->osp_reply_chan);
            if (osp_reply) {
                std::cout << "RECEIVED POSE INFO" << std::endl;
                state->manager.encode_osp_data (msg, osp_reply);
            }
        }
    }
    else if (state->send_recovery) {
        state->manager.encode_osp_recovery_request (msg, &state->recovery_rq);
        state->send_recovery = false;
    }
    else {
        state->manager.encode_iver_data (msg);
    }
}

void mini_request_callback (gap::ModemTransmission *msg, void *user)
{
    std::cout << "MINI REQUEST " << msg->DebugString () << std::endl;

    // make sure we dont already have data (from an abort)
    State *state = static_cast<State*>(user);
    if (msg->frame_size ()) {
        std::cout << "we already have a mini packet!" << std::endl;
    }
    else {
        state->manager.encode_iver_mini (msg);
    }
    
    // make sure we aren't sending a hard abort by mistake
    if (!msg->frame_size() )
        return;
    std::string bytes = msg->frame(0);
    if ((static_cast<unsigned char>(bytes[0]) == 0x1f)
            && (static_cast<unsigned char>(bytes[1]) == 0xff)) {
        std::cout << "This message is accidentally a hard abort, setting to null" 
            << std::endl;
        msg->set_frame(0, "");
    }

    if ((bytes[0]&0x1f) != bytes[0])
        std::cout << "CLIPPING ERROR!" << std::endl;

    // test abort message
    state->manager.decode_mini (*msg);
}

int main (int argc, char *argv[])
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG); 
    if (!param) {
        std::cerr << "could not find " << BOTU_PARAM_DEFAULT_CFG << "!" 
            << std::endl;
        exit (EXIT_FAILURE);
    }

    State state (param);
    perls::AcommsNode node (&state.lcm, param, 
            &data_rx_callback, &mini_rx_callback, &state, 
            &data_request_callback, &mini_request_callback, &state);

    int timeout_us = 1e5;
    while (state.is_running) {
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = timeout_us;
        while (tv.tv_usec > 0) {
            lcmu_handle_timeout (state.lcm.getUnderlyingLCM (), &tv);
        }
        
        if (state.send_abort) {
            node.blast_mini (state.mini_cmd, 1);
            state.send_abort = false;
        }
        else if (state.send_jump) {
            node.blast_mini (state.mini_cmd, 1);
            state.send_jump = false;
        }
        else 
            node.do_work ();
    }

    exit (EXIT_SUCCESS);
}
