#include <iostream>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <sstream>

// external linking req'd
#include <bot_param/param_client.h>
#include <lcm/lcm.h>

#include <goby/acomms/dccl.h>
#include <goby/acomms/modem_driver.h>

#include "perls-lcmtypes/capture_iver_state_t.h"
#include "perls-lcmtypes/capture_modem_data_t.h"
#include "perls-lcmtypes/capture_modem_range_t.h"
#include "perls-lcmtypes/senlcm_raw_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "protodefs/perls_subsea_state.pb.h"
#include "protodefs/perls_modem_mini2.pb.h"

#include "goby_acomms.h"
#include "perls_acomms.h"

#define MAXRANGES 4

using ga::operator<<;

// dccl codec
ga::DCCLCodec *dccl_;

// MicroModemMiniPacketDCCLIDCodec for encoding/decoding minipackets
class MicroModemMiniPacketDCCLIDCodec : public
ga::DCCLTypedFixedFieldCodec<goby::uint32>
{
    private:
    // 16 bits, only 13 are useable, so 3 "blank bits" + 3 bits for us
    ga::Bitset encode(const goby::uint32& wire_value)
        { return ga::Bitset(MINI_ID_SIZE, wire_value -
                            MINI_ID_OFFSET); }
    ga::Bitset encode()
        { return encode(MINI_ID_OFFSET); }
    goby::uint32 decode(ga::Bitset *bits)
        { return bits->to_ulong() + MINI_ID_OFFSET; }
    unsigned size() { return MINI_ID_SIZE; }
    void validate() { }
    
    // Add this value when decoding to put us safely in our own namespace
    // from the normal default DCCL Codec
    enum { MINI_ID_OFFSET = 0 };
    enum { MINI_ID_SIZE = 6 };
};

// RAII tool for setting and resetting the DCCL ID codec
struct MiniPacketIDCodecEnabler {
    MiniPacketIDCodecEnabler() { dccl_->set_id_codec("mini_id_codec"); }
    ~MiniPacketIDCodecEnabler() { dccl_->reset_id_codec(); }
};

// Handle SIGINT (ctrl+C) or SIGTERM (kill) gracefully.
static Node *global_node = 0;
void 
stop (int)
{
    if (global_node && global_node->is_running())
        global_node->stop();
}

// set most recent iver state
static void 
node_state_callback (const lcm_recv_buf_t *rbuf, 
                     const char *channel, const capture_iver_state_t *msg, void *user)
{
    Node *node = static_cast<Node *>(user);

    // set new iver lcm state 
    node->set_lcm_state (*msg);
}

// Node constructor
Node::Node (lcm_t &lcm, gap::DriverConfig &driver_cfg,
            gap::MACConfig &mac_cfg, const std::string &chan_prefix) : _lcm(lcm), 
                                                                       _mac(&_mac_log), _driver(&_driver_log),
                                                                       _my_id(0), _is_running(0)
{
    // set iver state 0 and start listening for state messages
    capture_iver_state_t init_iver = {0};
    _lcm_state = init_iver;

    _chan_prefix = chan_prefix;

    std::string chan = _chan_prefix + "SELF_STATE";
    capture_iver_state_t_subscribe (&_lcm, chan.c_str (),
            &node_state_callback, this);

    // set up mac/driver logs
    char fname[256];
    timeutil_strftime (fname, sizeof fname, "driver_%Y%m%d_%H%M%S.log", 
        timestamp_now ());
    _driver_log.open (fname);

    timeutil_strftime (fname, sizeof fname, "mac_%Y%m%d_%H%M%S.log",
        timestamp_now ());
    _mac_log.open (fname);    

    // bind manager to mac
    ga::bind (_mac, _driver); 

    // connect to a few signals
    ga::connect (&_driver.signal_receive, this, &Node::handle_received);
    ga::connect (&_driver.signal_data_request, this, &Node::handle_data_request);
    ga::connect (&_driver.signal_range_reply, this, &Node::handle_ranging);
    ga::connect (&_driver.signal_all_incoming, this, &Node::publish_incoming);
    ga::connect (&_driver.signal_all_outgoing, this, &Node::publish_outgoing);

    // configure mac and driver
    _mac.startup (mac_cfg);
    _driver.startup (driver_cfg); 
}

Node::~Node ()
{
    _driver.shutdown ();
}

// Node run method
void 
Node::run ()
{
    _is_running = true;
    while (_is_running) {
        // listen for a bit for iver state
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        
        while (tv.tv_usec > 0) {
            lcmu_handle_timeout (&_lcm, &tv);
        }

        _mac.do_work ();
        _driver.do_work ();
    }
    lcm_destroy (&_lcm);
}

// process received mini packets
void
process_mini (const gap::ModemDataTransmission &msg, void *user)
{
    Node *node = static_cast<Node *>(user);
    
    MiniPacketIDCodecEnabler scoped_enable;

    const std::string& data = msg.data();
    if (dccl_->id_from_encoded (data) == dccl_->id<pp::MiniOWTT>()) {
        pp::MiniOWTT mini_owtt = dccl_->decode<pp::MiniOWTT>(data);
        std::cout << "received mini owtt message: " 
            << mini_owtt.DebugString() << std::endl;
    }
    else if (dccl_->id_from_encoded(data)==dccl_->id<pp::MiniAbort>()) {
        pp::MiniAbort mini_abort = dccl_->decode<pp::MiniAbort>(data);
        std::cout << "received mini abort message: "
            << mini_abort.DebugString() << std::endl;

        if (mini_abort.user() == 1023 && msg.base().dest() == node->get_my_id() )
            node->publish_lcm_abort (mini_abort.user());
        else if (msg.base().dest() == node->get_my_id() )
            node->publish_lcm_jump (mini_abort.user());
    }
    else if (dccl_->id_from_encoded(data)==dccl_->id<pp::MiniUser>()) {
        pp::MiniUser mini_user = dccl_->decode<pp::MiniUser>(data);
        std::cout << "received mini user message: "
            << mini_user.DebugString() << std::endl;

        node->publish_lcm_mini (msg.base().src(), msg.base().dest(), 
            mini_user.user());
    }
}

// process received data packets
void
process_data (const gap::ModemDataTransmission &msg, void *user)
{
    Node *node = static_cast<Node *>(user);
    
    const std::string& data = msg.data();
    if (dccl_->id_from_encoded (data) != dccl_->id<pp::subsea_state>())
        return;

    pp::subsea_state state = dccl_->decode<pp::subsea_state>(data);
    std::cout << "received state message: " 
        << state.DebugString () << std::endl;

    node->publish_iver_state (state, msg.base().src());
}


// process received modem data
void
Node::handle_received (const gap::ModemDataTransmission &msg)
{
    std::cout << "modem received a message " << msg.DebugString() 
        << std::endl;
    const std::string& msg_data = msg.data();
    
    capture_modem_data_t data = {0};
    data.src           = msg.base().src();
    data.dest          = msg.base().dest();
    data.rate          = msg.base().rate();
    data.packet_type   = msg.GetExtension(MicroModem::packet_type); 
    data.ack_requested = msg.ack_requested();
    data.data_length   = msg.data().length();
    data.data          = (uint8_t *) msg.data().c_str();
    data.frame         = msg.frame();
    data.dccl_id       = dccl_->id_from_encoded (msg_data);
    
    std::string chan = _chan_prefix + "ACOMMS_DATA";
    capture_modem_data_t_publish (&_lcm, chan.c_str (), &data);

    switch (msg.GetExtension(MicroModem::packet_type)) {
      case MicroModem::PACKET_MINI: 
        process_mini (msg, this);
        break;
      case MicroModem::PACKET_DATA:
        process_data (msg, this);
        break;
    }
}

// deal with state data request from goby
void
Node::handle_data_request (const gap::ModemDataRequest &request_msg,
                           gap::ModemDataTransmission *data_msg)
{
    std::string data;    
    data_msg->set_ack_requested (false);

    // data request? 
    if (request_msg.GetExtension (MicroModem::request_slot).type() == gap::SLOT_DATA) {

        std::cout << "modem requests a data message " << std::endl;

        pp::subsea_state msg = state_lcm2dccl();  
        data = dccl_->encode (msg); 

        std::cout << "modem should send " << std::endl
            << msg.DebugString () << std::endl;
    }
    // mini request?
    else if (request_msg.GetExtension (MicroModem::request_slot).type() == gap::SLOT_MINI) {
        MiniPacketIDCodecEnabler scoped_enable;
        
        pp::MiniUser msg;
        msg.set_user (_lcm_state.error ? _lcm_state.error :_lcm_state.next_wypnt);
        data = dccl_->encode (msg);

        std::cout << "modem should send " << std::endl 
            << msg.DebugString () << std::endl;
    }
    data_msg->set_data (data);

    // print dccl id
    std::cout << "dccl id = " << dccl_->id_from_encoded (data)
        << std::endl;
}

// handle ranging messages
void
Node::handle_ranging (const gap::ModemRangingReply &msg)
{
    capture_modem_range_t *recvd = (capture_modem_range_t*) calloc (1, sizeof (*recvd));
    recvd->utime = timestamp_now ();
    recvd->src = msg.base().src();
    recvd->dest = msg.base().dest();

    recvd->owtt = (double*) malloc (MAXRANGES * sizeof (*recvd->owtt));
    recvd->nowtt = msg.one_way_travel_time_size();
    for (int i = 0; i<recvd->nowtt; i++)
        recvd->owtt[i] = msg.one_way_travel_time(i);

    recvd->ranging_type = msg.type();
    recvd->sender_clk_mode = msg.sender_clk_mode();
    recvd->receiver_clk_mode = msg.receiver_clk_mode();

    std::string chan = _chan_prefix + "ACOMMS_RANGE";
    capture_modem_range_t_publish (&_lcm, chan.c_str (), recvd);
    capture_modem_range_t_destroy (recvd);
}

// publish incoming
void
Node::publish_incoming (const gap::ModemMsgBase &msg_data)
{
    std::cout << "{driver in}:  " << msg_data.time () << "\t" <<
        msg_data.raw () << std::endl;

    senlcm_raw_t raw;
    raw.utime  = timestamp_now (); 
    raw.length = msg_data.raw().size ();
    raw.data   = (uint8_t *) msg_data.raw().c_str ();

    std::string chan = _chan_prefix + "ACOMMS.RAW";
    senlcm_raw_t_publish (&_lcm, chan.c_str (), &raw);
}


// publish outgoing
void Node::publish_outgoing (const gap::ModemMsgBase &msg_data)
{
    std::cout << "{driver out}: " << msg_data.time () << "\t" <<
        msg_data.raw () << std::endl;

    senlcm_raw_t raw;
    raw.utime  = timestamp_now ();
    raw.length = msg_data.raw().size ();
    raw.data   = (uint8_t *) msg_data.raw().c_str ();

    std::string chan = _chan_prefix + "ACOMMS.OUT";
    senlcm_raw_t_publish (&_lcm, chan.c_str (), &raw);
}


int 
main (int argc, char *argv[])
{
    // open param cfg file
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        std::cout << "could not find " << BOTU_PARAM_DEFAULT_CFG << std::endl;
    }
    std::cout << "Opened Param File [" << BOTU_PARAM_DEFAULT_CFG << "]" << std::endl;

    // parse cfg parameters
    std::string chan_base = bot_param_get_str_or_fail (param,
            "vehicle.lcm_channel_prefix");

    char *acomms_channel = bot_param_get_str_or_fail (param, "sensors.modem.gsd.channel");
    std::string abort_channel = std::string(acomms_channel) + "_ABORT";
    std::string jump_channel = std::string(acomms_channel) + "_JUMP";

    // parse modem device cfg
    std::string serial_port = bot_param_get_str_or_fail (param, 
            "sensors.modem.gsd.device");
    uint8_t modem_id = (uint8_t) bot_param_get_int_or_fail (param,
            "sensors.modem.id");
    if((modem_id < 1) || (modem_id > 7)) {
        std::cerr << "Syntax Error: Modem ID must be between 1 and 7" << std::endl;
        return 1;
    }

    // instantiate lcm
    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) {
        std::cerr << "lcm_create () failed!" << std::endl;
        return 1;
    }

    // set modem driver cfg
    gap::DriverConfig driver_cfg;
    driver_cfg.set_modem_id (modem_id);
    driver_cfg.set_serial_port (serial_port);

    // read nvram config parameters
    parse_mm_nvram_cfg (param, &driver_cfg);

    // add narrowband lbl configuration params 
    driver_cfg.SetExtension(MicroModemConfig::narrowband_turnaround_ms, 20);
    driver_cfg.SetExtension(MicroModemConfig::narrowband_transmit_freq, 26000);
    driver_cfg.SetExtension(MicroModemConfig::narrowband_transmit_ping_ms, 5);
    driver_cfg.SetExtension(MicroModemConfig::narrowband_receive_ping_ms, 5);
    driver_cfg.AddExtension(MicroModemConfig::narrowband_receive_freq, 27000);
    driver_cfg.AddExtension(MicroModemConfig::narrowband_receive_freq, 29000);
    driver_cfg.AddExtension(MicroModemConfig::narrowband_receive_freq, 31000);
    driver_cfg.SetExtension(MicroModemConfig::narrowband_transmit_flag, true);

    // set mac_cfg
    gap::MACConfig mac_cfg;
    mac_cfg.set_type (gap::MAC_FIXED_DECENTRALIZED);
    mac_cfg.set_modem_id (modem_id);
    build_tdma (param, &mac_cfg);

    // create local capture manager
    Node node (*lcm, driver_cfg, mac_cfg, chan_base);
    node.set_my_id (modem_id);
    node.set_abort_chan (abort_channel);
    node.set_jump_chan (jump_channel);

    // Register signal handlers to handle "quit" signals.
    global_node = &node;
    signal (SIGINT, stop);
    signal (SIGTERM, stop);

    // set dccl rolling
    dccl_ = ga::DCCLCodec::get ();

    // set up mini packet DCCL ID encoding
    dccl_->add_id_codec<MicroModemMiniPacketDCCLIDCodec>("mini_id_codec");
    // validate minipackets
    {
        MiniPacketIDCodecEnabler scoped_enable;
        assert(dccl_->validate<pp::MiniUser>());
        assert(dccl_->validate<pp::MiniOWTT>());
        assert(dccl_->validate<pp::MiniAbort>());
        dccl_->info<pp::MiniUser>(&std::cout);
        dccl_->info<pp::MiniOWTT>(&std::cout);
        dccl_->info<pp::MiniAbort>(&std::cout);
    }

    // validate perls state
    assert(dccl_->validate<pp::subsea_state>());
    dccl_->info<pp::subsea_state>(&std::cout);

    // Run until a signal is received that resets running to false
    node.run();

    // Cleanup and quit.
    printf ("Quit requested. Exiting.\n");
    return 0;
}
