#ifndef __PERLS_RESOURCE_MANAGER_H__
#define __PERLS_RESOURCE_MANAGER_H__

#include <map>
#include <lcm/lcm-cpp.hpp>

//#include<goby/protobuf/message.h>

#include <bot_param/param_client.h>

#include "perls-lcmtypes++/perllcm/auv_abort_t.hpp"
#include "perls-lcmtypes++/perllcm/auv_iver_state_t.hpp"
#include "perls-lcmtypes++/perllcm/auv_jump_t.hpp"
#include "perls-lcmtypes++/perllcm/owtt_nav_status_t.hpp"
#include "perls-lcmtypes++/perllcm/position_t.hpp"

#include "perls-lcmtypes++/senlcm/acomms_osp_recovery_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_pose_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_request_t.hpp"
#include "perls-lcmtypes++/senlcm/acomms_two_osp_t.hpp"
#include "perls-lcmtypes++/senlcm/uvc_osi_t.hpp"
#include "perls-lcmtypes++/senlcm/uvc_opi_t.hpp"

#include <goby/acomms/dccl.h>

namespace ga = goby::acomms;
namespace gap = goby::acomms::protobuf;
using ga::operator<<;

namespace perls
{
    const static std::string MiniPacketCodec = "mini_id_codec";

    class MM_MiniPacketDCCLCodec : public ga::DCCLTypedFixedFieldCodec<goby::uint32>
    {
      private:
        // 16 bits, only 13 are useable, so 3 "blank bits" + 3 bits for us
        ga::Bitset encode(const goby::uint32& wire_value)
            { return ga::Bitset(MINI_ID_SIZE, wire_value); }
        ga::Bitset encode()
            { return encode(0); }
        goby::uint32 decode(ga::Bitset *bits)
            { return bits->to_ulong(); }
        unsigned size() { return MINI_ID_SIZE; }
        void validate() { }

        // allow for up to 7 mini packet messages 
        enum { MINI_ID_SIZE = 3 };
    };

    struct MM_MiniPacketIDEnabler
    {
        ga::DCCLCodec *codec;
        MM_MiniPacketIDEnabler () : codec (ga::DCCLCodec::get ()) {codec->set_id_codec (MiniPacketCodec);}
        ~MM_MiniPacketIDEnabler () {codec->reset_id_codec ();}
    };

    class AcommsResManager 
    {
      public:
        AcommsResManager (lcm::LCM *lcm, BotParam *param);

        void decode_mini (const gap::ModemTransmission &msg);
        void decode_data (const gap::ModemTransmission &msg);
        void encode_iver_mini (gap::ModemTransmission *msg);
        void encode_iver_data (gap::ModemTransmission *msg);

        void encode_abort (gap::ModemTransmission *msg, const perllcm::auv_abort_t *abort);
        void encode_jump (gap::ModemTransmission *msg, const perllcm::auv_jump_t *jump);

        void encode_pose_data (gap::ModemTransmission *msg, const senlcm::acomms_pose_t *pose);
        void encode_osp_data (gap::ModemTransmission *msg, const senlcm::acomms_two_osp_t *pose);
        void encode_osp_recovery_request (gap::ModemTransmission *msg, const senlcm::acomms_request_t *rq);
        void encode_osp_recovery_reply (gap::ModemTransmission *msg, const senlcm::acomms_osp_recovery_t *rq);

        int recovery_request () 
        {  
            std::cout << "ACOMMS RES MANAGER SHOULD RETURN " << _recovery_tol_no << std::endl;
            int tol_no = 0;
            if (_recovery_tol_no) {
                tol_no = _recovery_tol_no;
                _recovery_tol_no = 0;
            }
            return tol_no; 
        }

        int node_id () const { return _node_id;}
      private:
        lcm::LCM *_lcm;

        perllcm::auv_iver_state_t _iver_state;
        void iver_state_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
                const perllcm::auv_iver_state_t* msg);
        senlcm::uvc_osi_t _uvc_osi_state;
        void uvc_osi_state_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
                const senlcm::uvc_osi_t* msg);
        senlcm::uvc_opi_t _uvc_opi_state;
        void uvc_opi_state_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan, 
                const senlcm::uvc_opi_t* msg);

        // estimator information
        perllcm::position_t _owtt_nav_pose;
        void owtt_nav_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
                const perllcm::position_t* msg);
        perllcm::owtt_nav_status_t _owtt_nav_status;
        void owtt_nav_status_callback (const lcm::ReceiveBuffer *rbuf, const std::string& chan,
                const perllcm::owtt_nav_status_t* msg);

        std::string _abort_chan;
        std::string _jump_chan;
        std::string _nav_pose_chan;
        std::string _nav_osp_chan;
        std::string _nav_osp_recovery_chan;

        int _recovery_tol_no;

        int _node_id;
        std::map<int, std::string> _network_map;

        ga::DCCLCodec& _dccl;
    };
}

#endif // __PERLS_RESOURCE_MANAGER_H__
