#ifndef __GOBY_ACOMMS_H__
#define __GOBY_ACOMMS_H__

#include <iostream>
#include <glib.h>

#include <bot_param/param_client.h>

#include <goby/acomms/amac.h>
#include <goby/acomms/modem_driver.h>

// add slots for tdma
// returns 0 on success, -1 on failure
int
build_tdma (BotParam *param, goby::acomms::protobuf::MACConfig *mac_cfg)
{
    // determine number of slots in tdma
    int num_slots = bot_param_get_num_subkeys (param, "tdma");

    // populate slots
    char key[32], src_key[32], dest_key[32], rate_key[32], secs_key[32], type_key[32];
    for (int i = 1; i <= num_slots; ++i) {
        snprintf (key, sizeof key, "tdma.slot%d", i);
        std::cout << "{cfg}: adding " << key << std::endl;

        snprintf (src_key,  sizeof src_key,  "%s.source", key);
        snprintf (dest_key, sizeof dest_key, "%s.destination", key);
        snprintf (rate_key, sizeof rate_key, "%s.rate", key);
        snprintf (secs_key, sizeof secs_key, "%s.slot_seconds", key);
        snprintf (type_key, sizeof type_key, "%s.slot_type", key);

        int src, dest, rate, secs;
        char *type;

        if (bot_param_get_int (param, src_key,  &src)  ||
            bot_param_get_int (param, dest_key, &dest) ||
            bot_param_get_int (param, rate_key, &rate) ||
            bot_param_get_int (param, secs_key, &secs) ||  
            bot_param_get_str (param, type_key, &type) ) {
            std::cerr << "{cfg}: could not correctly parse " << key << std::endl;
            return 1;
        }

        std::cout << "\tsource = " << src  << std::endl;
        std::cout << "\tdest   = " << dest << std::endl;
        std::cout << "\trate   = " << rate << std::endl;
        std::cout << "\tsecs   = " << secs << std::endl;
        std::cout << "\ttype   = " << type << std::endl;

        goby::acomms::protobuf::Slot slot;
        slot.set_src  (src);
        slot.set_dest (dest);
        slot.set_rate (rate);
        slot.set_slot_seconds (secs);
        if (0==strcmp (type, "SLOT_DATA"))
            slot.set_type (goby::acomms::protobuf::SLOT_DATA);
        else if (0==strcmp (type, "SLOT_PING"))
            slot.set_type (goby::acomms::protobuf::SLOT_PING);
        else if (0==strcmp (type, "SLOT_REMUS_LBL"))
            slot.set_type (goby::acomms::protobuf::SLOT_REMUS_LBL);
        else if (0==strcmp (type, "SLOT_NARROWBAND_LBL"))
            slot.set_type (goby::acomms::protobuf::SLOT_NARROWBAND_LBL);
        else if (0==strcmp (type, "SLOT_MINI"))
            slot.set_type (goby::acomms::protobuf::SLOT_MINI);
        else {
            std::cerr << "{cfg} unrecognized tdma slot type" << std::endl;
            return 1;
        }

        // add slot to mac_cfg
        mac_cfg->add_slot()->CopyFrom (slot);
    } // for loop parsing slots

    return 0;
}

// bot param helper function
void
parse_mm_nvram_cfg (BotParam *param,goby::acomms::protobuf::DriverConfig *mm_cfg)
{
    // determine number of params to set
    int num_params = bot_param_get_array_len (param, "sensors.modem.cfg");
    if (num_params <= 0)
        return;

    // set params
    char **cfg_params = bot_param_get_str_array_alloc (param, "sensors.modem.cfg");
    for (int i = 0; i < num_params; ++i) {
        char *cfg_param = cfg_params[i];
        mm_cfg->AddExtension (MicroModemConfig::nvram_cfg, cfg_param);
        std::cout << "{cfg}: added param " << cfg_param << std::endl;
    }
}



#endif // __GOBY_ACOMMS_H__
