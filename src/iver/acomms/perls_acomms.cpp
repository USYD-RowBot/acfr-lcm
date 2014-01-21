#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>

#include <goby/acomms/bind.h>

#include "perls-common/timestamp.h"

#include "perls-lcmtypes++/senlcm/raw_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_data_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_range_t.hpp"

#include "perls_acomms.h"

// FOR DEBUGGING
//void test_mac (const gap::ModemTransmission &m)
//{
//    std::cout << std::endl << "time to transmit " 
//        << m.DebugString ()
//        << "transmission has " << m.frame_size () << " frames" 
//        << std::endl;
//}

perls::AcommsNode::AcommsNode (lcm::LCM *lcm, BotParam *param, 
        ACOMMS_RX_TYPE rx, ACOMMS_RX_TYPE minirx, void *userrx, 
        ACOMMS_TX_TYPE tx, ACOMMS_TX_TYPE minitx, void *usertx)
    : _lcm (lcm), 
    _acomms_rx_cb (rx), _acomms_minirx_cb (minirx), _acomms_rx_user (userrx),
    _acomms_tx_cb (tx), _acomms_minitx_cb (minitx), _acomms_tx_user (usertx)
{
    if (!lcm || !lcm->good())
        throw perls::AcommsException ("*lcm not correctly initialized!");

    _node_id = bot_param_get_int_or_fail (param, "sensors.modem.id"); 
    _lcm_prefix = bot_param_get_str_or_fail (param, 
            "vehicle.lcm_channel_prefix");
    std::string channel = bot_param_get_str_or_fail (param, 
            "sensors.modem.gsd.channel");
    _data_chan = channel+"_DATA";
    _range_chan = channel+"_RANGE";
    _raw_in_chan = channel+"_MM_DRIVER_IN";
    _raw_out_chan = channel+"_MM_DRIVER_OUT";

    configure_mac (param);
    configure_driver (param);

    //// FOR DEBUGGING
    //ga::connect (&_mac.signal_initiate_transmission, &test_mac);

    ga::connect (&_driver.signal_receive, this, &AcommsNode::handle_received);
    ga::connect (&_driver.signal_data_request, this, &AcommsNode::handle_data_request);
    ga::connect (&_driver.signal_raw_incoming, this, &AcommsNode::handle_raw_incoming);
    ga::connect (&_driver.signal_raw_outgoing, this, &AcommsNode::handle_raw_outgoing);
    ga::bind (_mac, _driver);

    std::cout << "STARTING UP " << std::endl;
    _mac.startup (_mac_cfg);
    _driver.startup (_driver_cfg); 
}

void perls::AcommsNode::configure_driver (BotParam *param)
{
    std::string dev_port = bot_param_get_str_or_fail (param, 
            "sensors.modem.gsd.device");
    _driver_cfg.set_serial_port (dev_port);
    _driver_cfg.set_modem_id (_node_id);

    int n_nvram = bot_param_get_array_len (param, "sensors.modem.nvram_cfg");
    if (n_nvram <= 0) return;
    
    char **nvram_params = bot_param_get_str_array_alloc (param, "sensors.modem.nvram_cfg");
    for (int i=0;i<n_nvram;++i) {
        char *nvram = nvram_params[i];
        _driver_cfg.AddExtension (micromodem::protobuf::Config::nvram_cfg, nvram);
        free (nvram);
    }
    free (nvram_params);
}

void perls::AcommsNode::configure_mac (BotParam *param)
{
    _mac_cfg.set_type (gap::MAC_FIXED_DECENTRALIZED);
    _mac_cfg.set_modem_id (_node_id);

    bool set_lbl_params = false;

    int n_slots = bot_param_get_num_subkeys (param, "tdma");
    if (n_slots <= 0) 
        throw perls::AcommsException ("must specify a non-empty tdma schedule!");

    for (int i=0;i<n_slots;++i) {
        std::string key = "tdma.slot" + boost::lexical_cast<std::string>(i);

        int src, dest, rate, secs;
        char *type_tmp;
        if (bot_param_get_int (param, (key+".source").c_str (), &src) ||
                bot_param_get_int (param, (key+".destination").c_str (), &dest) ||
                bot_param_get_int (param, (key+".rate").c_str (), &rate) ||
                bot_param_get_int (param, (key+".slot_seconds").c_str (), &secs) ||
                bot_param_get_str (param, (key+".slot_type").c_str (), &type_tmp) )
            throw perls::AcommsException ("error parsing tdma schedule!");
        std::string slot_type = type_tmp;
        free (type_tmp);

        gap::ModemTransmission *slot = _mac_cfg.add_slot ();
        slot->set_src (src);
        slot->set_dest (dest);
        slot->set_rate (rate);
        slot->set_slot_seconds (secs);
        if (slot_type == "DATA")
            slot->set_type (gap::ModemTransmission::DATA);
        else if (slot_type == "MICROMODEM_TWO_WAY_PING") {
            slot->set_type (gap::ModemTransmission::DRIVER_SPECIFIC);
            slot->SetExtension (mp::type, mp::MICROMODEM_TWO_WAY_PING);
        }
        else if (slot_type == "MICROMODEM_REMUS_LBL_RANGING") {
            slot->set_type (gap::ModemTransmission::DRIVER_SPECIFIC);
            slot->SetExtension (mp::type, mp::MICROMODEM_REMUS_LBL_RANGING);
        }
        else if (slot_type == "MICROMODEM_NARROWBAND_LBL_RANGING") {
            slot->set_type (gap::ModemTransmission::DRIVER_SPECIFIC);
            slot->SetExtension (mp::type, mp::MICROMODEM_NARROWBAND_LBL_RANGING);
            if (src == _node_id) 
                set_lbl_params = true;
        }
        else if (slot_type == "MICROMODEM_MINI_DATA") {
            slot->set_type (gap::ModemTransmission::DRIVER_SPECIFIC);
            slot->SetExtension (mp::type, mp::MICROMODEM_MINI_DATA);
        }
        else 
            throw perls::AcommsException ("unknown tdma slot type!");
    }
    if (set_lbl_params) {
        mp::NarrowBandLBLParams* nb_cfg = _driver_cfg.MutableExtension(mp::Config::narrowband_lbl);

        int turnaround_ms = bot_param_get_int_or_fail (param, "lbl.turnaround_ms");
        nb_cfg->set_turnaround_ms (turnaround_ms);

        int tx_freq = bot_param_get_int_or_fail (param, "lbl.tx_frequency");
        nb_cfg->set_transmit_freq (tx_freq);

        int tx_pulse_ms = bot_param_get_int_or_fail (param, "lbl.tx_pulse_ms");
        nb_cfg->set_transmit_ping_ms (tx_pulse_ms);

        int rx_pulse_ms = bot_param_get_int_or_fail (param, "lbl.rx_pulse_ms");
        nb_cfg->set_receive_ping_ms (rx_pulse_ms);

        int n_beacons = bot_param_get_num_subkeys (param, "lbl.network");
        for (int i=0;i<n_beacons;++i) {
            std::string lbl_key = "lbl.network.beacon" + boost::lexical_cast<std::string>(i) + ".rx_frequency";
            int rx_freq = bot_param_get_int_or_fail (param, lbl_key.c_str ());
            nb_cfg->add_receive_freq (rx_freq);
        }

        bool tx_flag = bot_param_get_boolean_or_fail (param, "lbl.tx_flag");
        nb_cfg->set_transmit_flag(tx_flag);

        int max_range = bot_param_get_int_or_fail (param, "lbl.max_range");
        nb_cfg->set_lbl_max_range(max_range); 
    }
}

static int8_t senlcm_clk_mode (mp::ClockMode mode)
{
    int8_t senlcm_mode = 0;
    switch (mode) {
        case mp::INVALID_CLOCK_MODE:
            break;
        case mp::NO_SYNC_TO_PPS_AND_CCCLK_BAD:
            senlcm_mode = senlcm::acomms_range_t::NO_SYNC_TO_PPS_AND_CCCLK_BAD;
            break;
        case mp::NO_SYNC_TO_PPS_AND_CCCLK_GOOD:
            senlcm_mode = senlcm::acomms_range_t::NO_SYNC_TO_PPS_AND_CCCLK_GOOD;
            break;
        case mp::SYNC_TO_PPS_AND_CCCLK_BAD:
            senlcm_mode = senlcm::acomms_range_t::SYNC_TO_PPS_AND_CCCLK_BAD;
            break;
        case mp::SYNC_TO_PPS_AND_CCCLK_GOOD:
            senlcm_mode = senlcm::acomms_range_t::SYNC_TO_PPS_AND_CCCLK_GOOD;
            break;
    }
    return senlcm_mode;
}

void perls::AcommsNode::handle_received (const gap::ModemTransmission& msg)
{
    std::cout << "RECEIVED " << msg.DebugString () << std::endl;
    senlcm::acomms_data_t data_msg; 
    senlcm::acomms_range_t range_msg;

    data_msg.utime = msg.time ();
    data_msg.src = msg.src ();
    data_msg.dest = msg.dest ();
    data_msg.rate = msg.rate ();

    switch (msg.time_source ()) {
        case gap::ModemTransmission::MODEM_TIME:
            data_msg.time_source = senlcm::acomms_data_t::MODEM_TIME;
            break;
        case gap::ModemTransmission::GOBY_TIME:
        default:
            data_msg.time_source = senlcm::acomms_data_t::GOBY_TIME;
            break;
    }

    switch (msg.type ()) {
        case gap::ModemTransmission::DATA: 
            data_msg.type = senlcm::acomms_data_t::DATA;
            _acomms_rx_cb (msg, _acomms_rx_user);
            break;
        case gap::ModemTransmission::DRIVER_SPECIFIC:
            {
                switch (msg.GetExtension (mp::type)) {
                    case mp::MICROMODEM_MINI_DATA:
                        data_msg.type = senlcm::acomms_data_t::MINI_DATA;
                        _acomms_minirx_cb (msg, _acomms_rx_user);
                        break;
                    case mp::MICROMODEM_TWO_WAY_PING:
                        range_msg.type = senlcm::acomms_range_t::TWO_WAY_PING;
                        break;
                    case mp::MICROMODEM_REMUS_LBL_RANGING:
                        range_msg.type = senlcm::acomms_range_t::REMUS_LBL;
                        break;
                    case mp::MICROMODEM_NARROWBAND_LBL_RANGING:
                        range_msg.type = senlcm::acomms_range_t::NARROWBAND_LBL;
                        break;
                    default:
                        break;
                }
            }
            break;
        default:
            std::cout << "RECEIVED MESSAGE TYPE UNKNOWN" << std::endl;
            return;
    }
    if (msg.frame_size ()) {
        data_msg.max_num_frames = msg.max_num_frames ();
        data_msg.max_frame_bytes = msg.max_frame_bytes ();

        int frame_size = msg.frame_size ();
        data_msg.frame_size = frame_size;
        data_msg.frame = std::vector<std::string>(frame_size);
        for (int i=0;i<frame_size;++i)
            data_msg.frame[i] = msg.frame(i);
        _lcm->publish (_data_chan, &data_msg);
    }

    if (msg.HasExtension (mp::ranging_reply)) {
        mp::RangingReply range = msg.GetExtension (mp::ranging_reply);

        range_msg.utime = data_msg.utime;
        range_msg.time_source = data_msg.time_source;

        int num_returns = range.one_way_travel_time_size ();
        range_msg.nowtt = num_returns;
        range_msg.owtt = std::vector<double>(num_returns);
        std::cout << "RANGES = ";
        for (int i=0;i<num_returns;++i) {
            range_msg.owtt[i] = range.one_way_travel_time(i);
            std::cout << "\t" << range_msg.owtt[i];
        }
        std::cout << std::endl;

        if (range.is_one_way_synchronous ()) {
            range_msg.src = msg.src ();
            range_msg.type = senlcm::acomms_range_t::ONE_WAY_SYNCHRONOUS;
        }
        else {
            range_msg.src = 0;
        }

        range_msg.sender_clk_mode = senlcm_clk_mode (range.sender_clk_mode ());
        range_msg.receiver_clk_mode = senlcm_clk_mode (range.receiver_clk_mode ());
        std::cout << "RANGE CHAN " << _range_chan << std::endl;
        _lcm->publish (_range_chan, &range_msg);
    }
}

void perls::AcommsNode::handle_data_request (gap::ModemTransmission *msg)
{
    std::cout << "DATA REQUESTED" << std::endl;

    // we don't use acks
    msg->set_ack_requested (false);

    switch (msg->type ()) {
        case gap::ModemTransmission::DATA: 
            _acomms_tx_cb (msg, _acomms_tx_user);
            break;
        case gap::ModemTransmission::DRIVER_SPECIFIC:
            switch (msg->GetExtension (mp::type)) {
                case mp::MICROMODEM_MINI_DATA:
                    _acomms_minitx_cb (msg, _acomms_tx_user);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

static void publish_raw (lcm::LCM *lcm, const gap::ModemRaw& msg, std::string channel)
{
    senlcm::raw_t raw;
    raw.utime = timestamp_now ();
    const std::string& raw_str = msg.raw ();
    raw.length = raw_str.length ();
    raw.data = std::vector<uint8_t>(raw_str.begin(), raw_str.end());
    lcm->publish (channel, &raw);
}

void perls::AcommsNode::handle_raw_incoming (const gap::ModemRaw& msg)
{
    publish_raw (_lcm, msg, _raw_in_chan);
    std::cout << "MM RX " << msg.raw () << std::endl;
}

void perls::AcommsNode::handle_raw_outgoing (const gap::ModemRaw& msg)
{
    publish_raw (_lcm, msg, _raw_out_chan);
    std::cout << "MM TX " << msg.raw () << std::endl;
}

void perls::AcommsNode::do_work ()
{
    _mac.do_work ();
    _driver.do_work ();
}

void perls::AcommsNode::blast_mini (const gap::ModemTransmission &mini, int n_transmits)
{
    std::cout << "HALTING MAC" << std::endl; 
    _mac.shutdown ();

    while (n_transmits-- > 0) {
        std::cout << "TRANSMITTING MINI PACKET " << mini.DebugString ()
            << std::endl;
        _driver.handle_initiate_transmission (mini);
        _driver.do_work ();
        sleep (5);
    }

    std::cout << "RESTARTING MAC" << std::endl; 
    _mac.restart ();
}
