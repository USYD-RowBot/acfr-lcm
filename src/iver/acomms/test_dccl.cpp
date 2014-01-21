#include <iostream>
#include <string>

#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>

#include <goby/common/logger.h>
#include <goby/util.h>

#include "perls-common/bot_util.h"

#include "perls_resource_manager.h"

//#include "perls-protobuf/iver_state.pb.h"
//#include "perls-protobuf/iver_mini.pb.h"
#include "perls-protobuf/owtt_nav.pb.h"

namespace pp = perls::protobuf;

int main (int argc, char *argv[])
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG); 
    if (!param) {
        std::cerr << "could not find " << BOTU_PARAM_DEFAULT_CFG << "!" 
            << std::endl;
        exit (EXIT_FAILURE);
    }

    //goby::glog.add_stream(goby::common::logger::VERBOSE, &std::cout);
    //goby::glog.add_stream(goby::common::logger::DEBUG1, &std::cout);
    //goby::glog.add_stream(goby::common::logger::DEBUG2, &std::cout);
    //goby::glog.add_stream(goby::common::logger::DEBUG3, &std::cout);

    lcm::LCM lcm;
    perls::AcommsResManager manager (&lcm, param);

    ga::DCCLCodec& dccl = *ga::DCCLCodec::get ();


    std::string str_bytes;
    {
        pp::osm_server data;
        data.set_depth    (4.1310599040221092);
        data.set_eta_x1   (-137.6034997292912);
        data.set_eta_y1   (128.69524752997833);
        data.set_eta_x0   (-267.52686737767613);
        data.set_eta_y0   (-189.29975617674654);
        data.set_lam_x1x1 (1.2903304353735723);
        data.set_lam_x1y1 (0.0029051880329826964);
        data.set_lam_x1x0 (-0.19240584191495946);
        data.set_lam_x1y0 (-0.00812592824931816);
        data.set_lam_y1y1 (1.246059867491025);
        data.set_lam_y1x0 (-0.0078927234934449437);
        data.set_lam_y1y0 (-0.18691105368869784);
        data.set_lam_x0x0 (1.9448769064937259);
        data.set_lam_x0y0 (0.062368022653579269);
        data.set_lam_y0y0 (1.9479244972873597);
        data.set_org_no (0);
        data.set_new_no (1);
        std::cout << "TX OSM " << data.DebugString () << std::endl;

        std::string bytes;
        dccl.encode (&bytes, data);
        str_bytes = goby::util::hex_encode (bytes);
    }
    std::cout << "encoded " << str_bytes << std::endl;

    std::string bytes = goby::util::hex_decode (str_bytes);
    unsigned dccl_id = dccl.id_from_encoded (bytes);
    std::cout << "received id: " << dccl_id << std::endl;
    std::cout << "expect id: " << dccl.id<pp::osm_server>() << std::endl;
    if (dccl_id == dccl.id<pp::osm_server>() ) {
        std::cout << "received osm_server packet!" << std::endl;

        pp::osm_server data;
        dccl.decode (bytes, &data);
        std::cout << "RX OSM " << data.DebugString () << std::endl;
    }

    str_bytes = "";
    {
        pp::osm_recovery_request data;
        data.set_last_no (23);

        std::cout << "TX OSM RECOVERY " << data.DebugString () << std::endl;

        std::string bytes;
        dccl.encode (&bytes, data);
        str_bytes = goby::util::hex_encode (bytes);
    }
    std::cout << "encoded " << str_bytes << std::endl;

    bytes = goby::util::hex_decode (str_bytes);
    dccl_id = dccl.id_from_encoded (bytes);
    std::cout << "received id: " << dccl_id << std::endl;
    std::cout << "expect id: " << dccl.id<pp::osm_recovery_request>() << std::endl;
    if (dccl_id == dccl.id<pp::osm_recovery_request>() ) {
        std::cout << "received osm_server packet!" << std::endl;

        pp::osm_recovery_request data;
        dccl.decode (bytes, &data);
        std::cout << "RX OSM " << data.DebugString () << std::endl;
    }

    exit (EXIT_SUCCESS);
}

