/*
    ACFR navigation LCM handler routines
    
    Christian Lees
    ACFR
    16/10/12
*/

#ifndef HANDLERS_HPP
#define HANDLERS_HPP

#include <lcm/lcm-cpp.hpp>
#include "perls-common/units.h"
#include "acfr_nav.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/parosci_t.hpp"
#include "perls-lcmtypes++/senlcm/lq_modem_t.hpp"
#include "perls-lcmtypes++/senlcm/seabird_t.hpp"
#include "perls-lcmtypes++/senlcm/seabird_depth_t.hpp"
#include "perls-lcmtypes++/senlcm/rdi_pd5_t.hpp"
#include "perls-lcmtypes++/senlcm/ysi_t.hpp"
#include "perls-lcmtypes++/senlcm/oas_t.hpp"
#include "perls-lcmtypes++/senlcm/tcm_t.hpp"

#define RTOD (UNITS_RADIAN_TO_DEGREE)
#define DTOR (UNITS_DEGREE_TO_RADIAN)

using namespace senlcm;

void on_gps(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const gpsd3_t *gps, state_c* state);
void on_parosci(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const parosci_t *parosci, state_c* state);
void on_lq_modem(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const lq_modem_t *lqm, state_c* state);
void on_seabird_ct(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const seabird_t, state_c* state);
void on_seabird_depth(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const seabird_depth_t *sd, state_c* state);
void on_rdi(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const rdi_pd5_t *rdi, state_c* state);
void on_ysi(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const ysi_t *ysi, state_c* state);
void on_oas(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const oas_t *oas, state_c* state);
void on_tcm_compass(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const tcm_t *tcm, state_c* state);


#endif