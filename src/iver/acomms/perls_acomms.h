#ifndef __PERLS_ACOMMS_H__
#define __PERLS_ACOMMS_H__

#include <exception>
#include <stdexcept>

#include <bot_param/param_client.h>

#include <lcm/lcm-cpp.hpp>

#include <goby/acomms/amac.h>
#include <goby/acomms/modem_driver.h>

namespace ga = goby::acomms;
namespace gap = goby::acomms::protobuf;
namespace mp = micromodem::protobuf;

namespace perls 
{
    typedef void (*ACOMMS_RX_TYPE) (const gap::ModemTransmission&, void*);
    typedef void (*ACOMMS_TX_TYPE) (gap::ModemTransmission*, void*);

    class AcommsNode 
    {
      public:
        AcommsNode (lcm::LCM *lcm, BotParam *param, 
                ACOMMS_RX_TYPE rx=0, ACOMMS_RX_TYPE minirx=0, void *userrx=0, 
                ACOMMS_TX_TYPE tx=0, ACOMMS_TX_TYPE minitx=0, void *usertx=0);

        void do_work ();

        int node_id () const {return _node_id;}

        void blast_mini (const gap::ModemTransmission &mini, int n_transmits);
      private:
        lcm::LCM *_lcm;

        int _node_id;
        std::string _lcm_prefix;
        std::string _data_chan;
        std::string _range_chan;
        std::string _raw_in_chan;
        std::string _raw_out_chan;

        gap::DriverConfig _driver_cfg;
        ga::MMDriver _driver;

        gap::MACConfig _mac_cfg;
        ga::MACManager _mac;

        void configure_mac (BotParam *param);
        void configure_driver (BotParam *param);

        void handle_received (const gap::ModemTransmission& msg);
        void handle_data_request (gap::ModemTransmission *msg);
        void handle_raw_incoming (const gap::ModemRaw& msg);
        void handle_raw_outgoing (const gap::ModemRaw& msg);

        ACOMMS_RX_TYPE _acomms_rx_cb;
        ACOMMS_RX_TYPE _acomms_minirx_cb;
        void *_acomms_rx_user;

        ACOMMS_TX_TYPE _acomms_tx_cb;
        ACOMMS_TX_TYPE _acomms_minitx_cb;
        void *_acomms_tx_user;
    };

    class AcommsException : public std::runtime_error
    {
      public:
        AcommsException (const std::string &s)
            : std::runtime_error (s) 
        { }
    };
}

#endif // __PERLS_ACOMMS_H__
