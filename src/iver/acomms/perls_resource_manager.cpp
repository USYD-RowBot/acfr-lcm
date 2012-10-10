#include <iostream>
#include <ctime>
#include <boost/lexical_cast.hpp>

#include "perls-common/lcm_util.h"

#include "perls-lcmtypes++/perllcm/auv_acomms_iver_state_t.hpp"
#include "perls-lcmtypes++/perllcm/auv_acomms_mini_t.hpp"

#include "perls-protobuf/iver_state.pb.h"
#include "perls-protobuf/iver_mini.pb.h"
#include "perls-protobuf/owtt_nav.pb.h"

#include "perls_resource_manager.h"

namespace pp = perls::protobuf;

void fill_network_map (BotParam *param, std::map<int,std::string>& net)
{
    int n_nodes = bot_param_get_num_subkeys (param, "acomms_network");
    for (int i=1;i<=n_nodes;++i) {
        std::string key = "acomms_network.vehicle" + boost::lexical_cast<std::string>(i);

        int id;
        char *name;
        if (bot_param_get_int (param, (key+".id").c_str (), &id) ||
            bot_param_get_str (param, (key+".name").c_str (), &name) ) {
            std::cerr << "error parsing acomms_network!!!" << std::endl;
            exit (EXIT_FAILURE);
        }
        net[id] = std::string (name);
        free (name);
    }
}

perls::AcommsResManager::AcommsResManager (lcm::LCM *lcm, BotParam *param) 
    : _lcm (lcm), _dccl (*ga::DCCLCodec::get ())
{
    if (!_lcm || !_lcm->good()) {
        std::cerr << "lcm is not good!" << std::endl;
        exit (EXIT_FAILURE);
    }
    char *ch_state_chan;
    if (!bot_param_get_str (param, "vehicle.state_channel", &ch_state_chan)) {
        std::string state_chan = ch_state_chan;
        lcm->subscribe (state_chan, &perls::AcommsResManager::iver_state_callback, this);
        //only for ivers
        state_chan = lcmu_channel_get_os_conduit (param, LCMU_CHANNEL_OS_CONDUIT_OSI);
        lcm->subscribe (state_chan, &perls::AcommsResManager::uvc_osi_state_callback, this);
        state_chan = lcmu_channel_get_os_conduit (param, LCMU_CHANNEL_OS_CONDUIT_OPI);
        lcm->subscribe (state_chan, &perls::AcommsResManager::uvc_opi_state_callback, this);
        free (ch_state_chan);
    }

    std::string chanpre = bot_param_get_str_or_fail (param, "vehicle.lcm_channel_prefix");
    lcm->subscribe (chanpre+"EST_TEST", &perls::AcommsResManager::owtt_nav_callback, this);
    lcm->subscribe (chanpre+"NAV_STATUS", &perls::AcommsResManager::owtt_nav_status_callback, this);

    // initialize state messages to zeros
    perllcm::auv_iver_state_t iver_tmp = {0};
    _iver_state = iver_tmp;
    senlcm::uvc_osi_t osi_tmp = {0};
    _uvc_osi_state = osi_tmp;
    senlcm::uvc_opi_t opi_tmp = {0};
    _uvc_opi_state = opi_tmp;

    perllcm::position_t pose_tmp = {0};
    _owtt_nav_pose = pose_tmp;
    perllcm::owtt_nav_status_t stat_tmp = {0};
    _owtt_nav_status = stat_tmp;

    // parse param file
    _node_id = bot_param_get_int_or_fail (param, "sensors.modem.id"); 

    std::string acomms_chan = bot_param_get_str_or_fail (param, "sensors.modem.gsd.channel");
    _abort_chan = acomms_chan + "_ABORT";
    _jump_chan = acomms_chan + "_JUMP";
    _nav_pose_chan = acomms_chan + "_NAV_POSE";
    _nav_osp_chan = acomms_chan + "_NAV_OSP";
    _nav_osp_recovery_chan = acomms_chan + "_NAV_OSP_RECOVERY";
    _recovery_tol_no = 0;

    fill_network_map (param, _network_map);

    // validate data messages
    try {
        _dccl.validate<pp::iver_state>();
        _dccl.validate<pp::pose_server>();
        _dccl.validate<pp::osm_server>();
        _dccl.validate<pp::osm_recovery_request>();
        _dccl.validate<pp::osm_recovery_reply>();

    } 
    catch (ga::DCCLException& e) {
        std::cerr << "DCCL validate exception! " << e.what () << std::endl;
        throw;
    }

    // validate mini messages
    _dccl.add_id_codec<MM_MiniPacketDCCLCodec>(perls::MiniPacketCodec);
    {
        MM_MiniPacketIDEnabler mini_enabled;
        try {
            _dccl.validate<pp::MiniWaypoint>();
            _dccl.validate<pp::MiniError>();
            _dccl.validate<pp::MiniOWTT>();
            _dccl.validate<pp::MiniAbort>();
            _dccl.validate<pp::MiniJump>();
        } 
        catch (ga::DCCLException& e) {
            std::cerr << "DCCL validate exception! " << e.what () << std::endl;
            throw;
        }
        catch (...) {
            std::cerr << "uh oh! caught unknown exception!" << std::endl;
            throw;
        }
    }
    std::cout << "DCCL: " << std::endl << _dccl << std::endl;

    //goby::glog.add_stream(goby::common::logger::VERBOSE, &std::cout);
    //goby::glog.add_stream(goby::common::logger::DEBUG1, &std::cout);
    //goby::glog.add_stream(goby::common::logger::DEBUG2, &std::cout);
    //goby::glog.add_stream(goby::common::logger::DEBUG3, &std::cout);
}

void dccl_mini (std::string &bytes)
{
    if (bytes.size () == 1) {
        bytes.resize (bytes.size () + 1, 0x00);
    }
    std::reverse (bytes.begin (), bytes.end ());
}

void perls::AcommsResManager::decode_mini (const gap::ModemTransmission &msg)
{
    MM_MiniPacketIDEnabler mini_enabled;

    // mini packets should contain a SINGLE frame
    if (!msg.frame_size ()) {
        std::cout << "mini packet does not have any data!" << std::endl;
        return;
    }
    std::string src_prefix;
    std::map<int,std::string>::iterator it = _network_map.find (msg.src ());
    if (it != _network_map.end ())
        src_prefix = it->second+"_";

    std::string bytes = msg.frame(0);
    dccl_mini (bytes);
   
    perllcm::auv_acomms_mini_t lcm_mini = {0}; 
    unsigned dccl_id = _dccl.id_from_encoded (bytes);
    if (dccl_id == _dccl.id<pp::MiniWaypoint>() ) {
        pp::MiniWaypoint mini;
        _dccl.decode (bytes, &mini);
        lcm_mini.mini_type = perllcm::auv_acomms_mini_t::MINI_WAYPOINT;
        lcm_mini.waypoint = mini.waypoint ();
        std::cout << "RX MINI WAYPOINT " << mini.DebugString () << std::endl;
    }
    else if (dccl_id == _dccl.id<pp::MiniError>() ) {
        pp::MiniError mini;
        _dccl.decode (bytes, &mini);
        lcm_mini.mini_type = perllcm::auv_acomms_mini_t::MINI_ERROR;
        lcm_mini.uvc_error = static_cast<int>(mini.uvc_error_state ());
        std::cout << "RX MINI ERROR " << mini.DebugString () << std::endl;
    }
    else if (dccl_id == _dccl.id<pp::MiniOWTT>() ) {
        pp::MiniOWTT mini;
        _dccl.decode (bytes, &mini);
        std::cout << "RX MINI OWTT " << mini.DebugString () << std::endl;
    }
    else if (dccl_id == _dccl.id<pp::MiniAbort>() ) {
        pp::MiniAbort mini;
        _dccl.decode (bytes, &mini);
        std::cout << "RX MINI ABORT " << mini.DebugString () << std::endl;
        if (msg.dest () != _node_id) return;
        switch (mini.abort_code ())
        {
            case pp::ABORT_FALSE:
                break;
            case pp::ABORT_TO_SURFACE:
            case pp::ABORT_TO_WAYPOINT:
            case pp::ABORT_TO_POS:
            case pp::ABORT_HARD:
                {
                    perllcm::auv_abort_t abort_msg;
                    abort_msg.utime = msg.time ();
                    abort_msg.dest = _node_id;
                    abort_msg.abort = perllcm::auv_abort_t::ABORT_HARD;
                    _lcm->publish (_abort_chan, &abort_msg);
                }
                break;
        }
    }
    else if (dccl_id == _dccl.id<pp::MiniJump>() ) {
        pp::MiniJump mini;
        _dccl.decode (bytes, &mini);
        std::cout << "RX MINI JUMP " << mini.DebugString () << std::endl;
        if (msg.dest () != _node_id) return;
        perllcm::auv_jump_t jump_msg;
        jump_msg.utime = msg.time ();
        jump_msg.dest = _node_id;
        jump_msg.next_wypnt = mini.waypoint ();
        _lcm->publish (_jump_chan, &jump_msg);
    }
    else {
        std::cout << "could not decode mini packet!!!" << std::endl;
    }

    if (lcm_mini.mini_type) {
        lcm_mini.utime = msg.time ();

        std::string src_prefix;
        std::map<int,std::string>::iterator it = _network_map.find (msg.src ());
        if (it != _network_map.end ()) src_prefix = it->second+"_";
        else src_prefix = "UNKNOWN_";

        _lcm->publish (src_prefix+"ACOMMS_MINI", &lcm_mini);
    }
}

void perls::AcommsResManager::decode_data (const gap::ModemTransmission &msg)
{
    // 06/12: assume that each frame stands alone --- can publish each frame
    // independent of all other frames
    // 08/12: can't do that --- osp takes up 2 frames, fits in 1 lcm message
    // --- needs to be reworked
    std::cout << "ACOMMS MANAGER RECEIVED " << msg.frame_size () << " FRAMES" << std::endl;
    if (!msg.frame_size ()) {
        std::cout << "data packet does not have any data!" << std::endl;
        return;
    }

    // src_prefix is a terrible idea, very confusing when logs are merged
    std::string src_prefix;
    std::map<int,std::string>::iterator it = _network_map.find (msg.src ());
    if (it != _network_map.end ()) src_prefix = it->second+"_";
    else src_prefix = "UKNOWN_";

    // 08/12 hack for now:
    senlcm::acomms_two_osp_t *two_osp = 0;
    for (int i=0;i<msg.frame_size();++i) {
        const std::string& bytes = msg.frame (i);

        unsigned dccl_id = _dccl.id_from_encoded (bytes);
        if (dccl_id == _dccl.id<pp::iver_state>() ) {
            pp::iver_state data;
            _dccl.decode (bytes, &data);
            std::cout << "RX DATA " << data.DebugString () << std::endl;

            perllcm::auv_acomms_iver_state_t iver_msg = {0};
            iver_msg.utime = msg.time ();
            iver_msg.position.utime = msg.time ();
            iver_msg.position.xyzrph[0] = data.x ();
            iver_msg.position.xyzrph[1] = data.y ();
            iver_msg.position.xyzrph[2] = data.z ();
            iver_msg.position.xyzrph[3] = data.r ();
            iver_msg.position.xyzrph[4] = data.p ();
            iver_msg.position.xyzrph[5] = data.h ();
            iver_msg.altitude = data.altitude ();
            iver_msg.abort_state = data.abort_state ();
            iver_msg.uvc_error = static_cast<int>(data.uvc_error_state ());
            iver_msg.uvc_battery_percent = data.uvc_battery_percent ();
            iver_msg.uvc_next_waypoint = data.uvc_next_waypoint ();
            iver_msg.uvc_dist_to_next_waypoint = data.uvc_dist_to_next_waypoint ();
            _lcm->publish (src_prefix+"ACOMMS_IVER_STATE", &iver_msg);

            perllcm::position_t est_pose = {0};
            est_pose.utime = msg.time ();
            est_pose.xyzrph[0] = data.x_est ();
            est_pose.xyzrph[1] = data.y_est ();
            _lcm->publish (src_prefix+"ACOMMS_IVER_EST_STATE", &est_pose);

            perllcm::owtt_nav_status_t stat = {0};
            stat.utime = msg.time ();
            stat.last_tol_no = data.last_no ();
            _lcm->publish (src_prefix+"ACOMMS_IVER_EST_STATUS", &stat);
        }
        else if (dccl_id == _dccl.id<pp::pose_server>() ) {
            pp::pose_server data;
            _dccl.decode (bytes, &data);
            std::cout << "RX DATA " << data.DebugString () << std::endl;

            senlcm::acomms_pose_t pose_msg = {0};
            pose_msg.utime = msg.time ();
            pose_msg.depth = data.depth ();
            pose_msg.xy[0] = data.mu_x ();
            pose_msg.xy[1] = data.mu_y ();
            pose_msg.xy_cov[0] = data.sig_xx ();
            pose_msg.xy_cov[1] = data.sig_xy ();
            pose_msg.xy_cov[2] = data.sig_xy ();
            pose_msg.xy_cov[3] = data.sig_yy ();
            _lcm->publish (_nav_pose_chan, &pose_msg);
        }
        else if (dccl_id == _dccl.id<pp::osm_server>() ){
            std::cout << "OSM SERVER DATA" << std::endl;
            pp::osm_server data;
            _dccl.decode (bytes, &data);
            senlcm::acomms_osp_t osp = {0};
            osp.utime = msg.time ();
            osp.depth = data.depth ();

            osp.eta[0] = data.eta_x1 ();
            osp.eta[1] = data.eta_y1 ();
            osp.eta[2] = data.eta_x0 ();
            osp.eta[3] = data.eta_y0 ();

            osp.Lambda[0] = data.lam_x1x1 ();
            osp.Lambda[1] = data.lam_x1y1 ();
            osp.Lambda[2] = data.lam_x1x0 ();
            osp.Lambda[3] = data.lam_x1y0 ();

            osp.Lambda[4] = data.lam_x1y1 (); // y1x1
            osp.Lambda[5] = data.lam_y1y1 ();
            osp.Lambda[6] = data.lam_y1x0 ();
            osp.Lambda[7] = data.lam_y1y0 ();

            osp.Lambda[8]  = data.lam_x1x0 (); // x0x1
            osp.Lambda[9]  = data.lam_y1x0 (); // x0y1
            osp.Lambda[10] = data.lam_x0x0 ();
            osp.Lambda[11] = data.lam_x0y0 ();

            osp.Lambda[12] = data.lam_x1y0 (); // y0x1
            osp.Lambda[13] = data.lam_y1y0 (); // y0y1
            osp.Lambda[14] = data.lam_x0y0 (); // y0x0
            osp.Lambda[15] = data.lam_y0y0 ();

            osp.org_tol_no = data.org_no ();
            osp.new_tol_no = data.new_no ();

            if (i==0) {
                two_osp = new senlcm::acomms_two_osp_t;
                two_osp->utime = msg.time ();
                two_osp->current = osp;
            }
            else if (i==1 && two_osp) {
                two_osp->last = osp;
            }
            else two_osp = 0;
        }
        else if (dccl_id == _dccl.id<pp::osm_recovery_request> () ) {
            std::cout << "RECOVERY REQUEST" << std::endl;
            pp::osm_recovery_request data;
            _dccl.decode (bytes, &data);
            _recovery_tol_no = data.last_no ();
            std::cout << "RECEIVED RECOVERY REQUEST FOR " << _recovery_tol_no
                << std::endl;
        }
        else if (dccl_id == _dccl.id<pp::osm_recovery_reply> () ) {
            pp::osm_recovery_reply data;
            _dccl.decode (bytes, &data);
            senlcm::acomms_osp_recovery_t rc;

            rc.eta[0] = data.eta_x1 ();
            rc.eta[1] = data.eta_y1 ();
            rc.eta[2] = data.eta_x0 ();
            rc.eta[3] = data.eta_y0 ();

            rc.Lambda[0] = data.lam_x1x1 ();
            rc.Lambda[1] = data.lam_x1y1 ();
            rc.Lambda[2] = data.lam_x1x0 ();
            rc.Lambda[3] = data.lam_x1y0 ();

            rc.Lambda[4] = data.lam_x1y1 (); // y1x1
            rc.Lambda[5] = data.lam_y1y1 ();
            rc.Lambda[6] = data.lam_y1x0 ();
            rc.Lambda[7] = data.lam_y1y0 ();

            rc.Lambda[8]  = data.lam_x1x0 (); // x0x1
            rc.Lambda[9]  = data.lam_y1x0 (); // x0y1
            rc.Lambda[10] = data.lam_x0x0 ();
            rc.Lambda[11] = data.lam_x0y0 ();

            rc.Lambda[12] = data.lam_x1y0 (); // y0x1
            rc.Lambda[13] = data.lam_y1y0 (); // y0y1
            rc.Lambda[14] = data.lam_x0y0 (); // y0x0
            rc.Lambda[15] = data.lam_y0y0 ();

            rc.org_tol_no = data.org_no ();
            rc.new_tol_no = data.new_no ();

            if (i==2 && two_osp) {
                rc.two_osp_msg = *two_osp;
                _lcm->publish (_nav_osp_recovery_chan, &rc);

                delete two_osp;
                two_osp = 0;
            }
        }
        else {
            std::cout << "did not recognize data packet!!!" << std::endl;
        }
    }
    if (two_osp) {
        _lcm->publish (_nav_osp_chan, two_osp);
        delete two_osp;
    }
}

void perls::AcommsResManager::encode_iver_mini (gap::ModemTransmission *msg)
{
    MM_MiniPacketIDEnabler mini_enabled;

    std::string bytes;
    if (_uvc_osi_state.error) { // send MiniError
        pp::MiniError mini;
        mini.set_uvc_error_state (static_cast<pp::UVCError>(_uvc_osi_state.error));
        _dccl.encode (&bytes, mini);
        std::cout << "TX MINI " << mini.DebugString () << std::endl;
    }
    else { // send MiniWaypoint
        pp::MiniWaypoint mini;
        mini.set_waypoint (_uvc_osi_state.nextwp);
        _dccl.encode (&bytes, mini);
        std::cout << "TX MINI " << mini.DebugString () << std::endl;
    }
    dccl_mini (bytes);

    msg->add_frame (bytes);
    std::cout << "TX MSG " << bytes << std::endl;
}

void perls::AcommsResManager::encode_iver_data (gap::ModemTransmission *msg)
{
    pp::iver_state data;
    data.set_x (_iver_state.position.xyzrph[0]);
    data.set_y (_iver_state.position.xyzrph[1]);
    data.set_z (_iver_state.position.xyzrph[2]);
    data.set_r (_iver_state.position.xyzrph[3]);
    data.set_p (_iver_state.position.xyzrph[4]);
    data.set_h (_iver_state.position.xyzrph[5]);
    data.set_altitude (_iver_state.altitude);
    data.set_uvc_next_waypoint (_uvc_osi_state.nextwp);
    data.set_uvc_dist_to_next_waypoint (_uvc_osi_state.dist_to_nextwp);
    data.set_uvc_battery_percent (_uvc_opi_state.percent);
    data.set_uvc_error_state (static_cast<pp::UVCError>(_uvc_osi_state.error));
    data.set_abort_state (_iver_state.abort_state);

    data.set_x_est (_owtt_nav_pose.xyzrph[0]);
    data.set_y_est (_owtt_nav_pose.xyzrph[1]);
    data.set_last_no (_owtt_nav_status.last_tol_no);

    std::string bytes;
    bytes.resize (msg->max_frame_bytes (), 0);
    _dccl.encode (&bytes, data);
    std::cout << "TX DATA " << data.DebugString () << std::endl;
    msg->add_frame (bytes);
}

void perls::AcommsResManager::encode_abort (gap::ModemTransmission *msg, 
        const perllcm::auv_abort_t *abort) 
{
    MM_MiniPacketIDEnabler mini_enabled;

    pp::MiniAbort mm_abort;
    mm_abort.set_abort_code (static_cast<pp::AbortCode>(abort->abort));

    std::string bytes;
    _dccl.encode (&bytes, mm_abort);
    dccl_mini (bytes);

    msg->add_frame (bytes);
    std::cout << "ENCODED ABORT " << mm_abort.DebugString () << std::endl;
}

void perls::AcommsResManager::encode_jump (gap::ModemTransmission *msg, 
        const perllcm::auv_jump_t *jump) 
{
    MM_MiniPacketIDEnabler mini_enabled;
    
    pp::MiniJump mm_jump;
    mm_jump.set_waypoint (jump->next_wypnt);

    std::string bytes;
    _dccl.encode (&bytes, mm_jump);
    dccl_mini (bytes);

    msg->add_frame (bytes);
    std::cout << "ENCODED JUMP " << mm_jump.DebugString () << std::endl;
}

void perls::AcommsResManager::encode_pose_data (gap::ModemTransmission *msg,
        const senlcm::acomms_pose_t *pose)
{
    pp::pose_server data;
    data.set_depth (pose->depth);
    data.set_mu_x (pose->xy[0]);
    data.set_mu_y (pose->xy[1]);
    data.set_sig_xx (pose->xy_cov[0]);
    data.set_sig_xy (pose->xy_cov[1]);
    data.set_sig_yy (pose->xy_cov[3]);

    std::string bytes;
    bytes.resize (msg->max_frame_bytes (), 0);
    _dccl.encode (&bytes, data);
    std::cout << "TX DATA " << data.DebugString () << std::endl;
    msg->add_frame (bytes);
}

void perls::AcommsResManager::encode_osp_data (gap::ModemTransmission *msg,
        const senlcm::acomms_two_osp_t *osp)
{
    for (int i=0;i<2;++i) {
        senlcm::acomms_osp_t pack;
        if (i==0) pack = osp->current;
        else pack = osp->last;

        pp::osm_server data;
        data.set_depth (pack.depth);

        data.set_org_no (pack.org_tol_no);
        data.set_new_no (pack.new_tol_no);

        data.set_eta_x1 (pack.eta[0]);
        data.set_eta_y1 (pack.eta[1]);
        data.set_eta_x0 (pack.eta[2]);
        data.set_eta_y0 (pack.eta[3]);

        data.set_lam_x1x1 (pack.Lambda[0]);
        data.set_lam_x1y1 (pack.Lambda[1]);
        data.set_lam_x1x0 (pack.Lambda[2]);
        data.set_lam_x1y0 (pack.Lambda[3]);

        data.set_lam_y1y1 (pack.Lambda[5]);
        data.set_lam_y1x0 (pack.Lambda[6]);
        data.set_lam_y1y0 (pack.Lambda[7]);

        data.set_lam_x0x0 (pack.Lambda[10]);
        data.set_lam_x0y0 (pack.Lambda[11]);

        data.set_lam_y0y0 (pack.Lambda[15]);
        std::string bytes;
        bytes.resize (msg->max_frame_bytes (), 0);
        _dccl.encode (&bytes, data);
        std::cout << "TX DATA " << data.DebugString () << std::endl;
        msg->add_frame (bytes);
    }
}

void perls::AcommsResManager::encode_osp_recovery_request (gap::ModemTransmission *msg, 
        const senlcm::acomms_request_t *rq)
{
    pp::osm_recovery_request data;
    data.set_last_no (rq->last_tol_no);

    std::string bytes;
    _dccl.encode (&bytes, data);
    std::cout << "TX DATA " << data.DebugString () << std::endl;
    msg->add_frame (bytes);
}

void perls::AcommsResManager::encode_osp_recovery_reply (gap::ModemTransmission *msg, 
        const senlcm::acomms_osp_recovery_t *rc)
{
    // pack first two frames
    encode_osp_data (msg, &rc->two_osp_msg);

    // pack last frame with recovery packet
    pp::osm_recovery_reply data;

    data.set_org_no (rc->org_tol_no);
    data.set_new_no (rc->new_tol_no);

    data.set_eta_x1 (rc->eta[0]);
    data.set_eta_y1 (rc->eta[1]);
    data.set_eta_x0 (rc->eta[2]);
    data.set_eta_y0 (rc->eta[3]);

    data.set_lam_x1x1 (rc->Lambda[0]);
    data.set_lam_x1y1 (rc->Lambda[1]);
    data.set_lam_x1x0 (rc->Lambda[2]);
    data.set_lam_x1y0 (rc->Lambda[3]);

    data.set_lam_y1y1 (rc->Lambda[5]);
    data.set_lam_y1x0 (rc->Lambda[6]);
    data.set_lam_y1y0 (rc->Lambda[7]);

    data.set_lam_x0x0 (rc->Lambda[10]);
    data.set_lam_x0y0 (rc->Lambda[11]);

    data.set_lam_y0y0 (rc->Lambda[15]);

    std::string bytes;
    bytes.resize (msg->max_frame_bytes (), 0);
    _dccl.encode (&bytes, data);
    std::cout << "TX DATA " << data.DebugString () << std::endl;
    msg->add_frame (bytes);
}

void perls::AcommsResManager::iver_state_callback (const lcm::ReceiveBuffer *rbuf, 
        const std::string& chan, const perllcm::auv_iver_state_t* msg)
{
    _iver_state = *msg;
}

void perls::AcommsResManager::uvc_osi_state_callback (const lcm::ReceiveBuffer *rbuf, 
        const std::string& chan, const senlcm::uvc_osi_t* msg)
{
    _uvc_osi_state = *msg;
}

void perls::AcommsResManager::uvc_opi_state_callback (const lcm::ReceiveBuffer *rbuf, 
        const std::string& chan, const senlcm::uvc_opi_t* msg)
{
    _uvc_opi_state = *msg;
}

void perls::AcommsResManager::owtt_nav_callback (const lcm::ReceiveBuffer *rbuf, 
        const std::string& chan, const perllcm::position_t* msg)
{
    _owtt_nav_pose = *msg;
}

void perls::AcommsResManager::owtt_nav_status_callback (const lcm::ReceiveBuffer *rbuf, 
        const std::string& chan, const perllcm::owtt_nav_status_t* msg)
{
    _owtt_nav_status = *msg;
}


