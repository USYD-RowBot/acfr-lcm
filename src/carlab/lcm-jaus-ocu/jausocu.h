#ifndef __PERLS_JAUSOCU_H__
#define __PERLS_JAUSOCU_H__

#include <stdlib.h>
#include <cstdlib>
#include <unistd.h>
#include <termios.h>

#include <gsl/gsl_math.h>

// need to link against openjaus
#include <jaus/jaus.h>
#include <openJaus/openJaus.h>
#include <jaus/message/jausMessageHeaders.h>
#include "experimentalMessageHeaders.h"

// linking against lcm for now, may remove once data structures are set
#include <lcm/lcm-cpp.hpp>
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_discrete_devices_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_error_report_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_error_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_signals_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_state_t.hpp"
#include "perls-lcmtypes++/perllcm/carlab_wrench_effort_t.hpp"

#include "perls-common/timestamp.h"
#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>


class BaseCallback {
public:
    virtual ~BaseCallback() {}
};

template<class MessageType>
class CallbackMessage : public BaseCallback {
public:
    boost::function<void (const MessageType *m)> callback;
};





class JAUSOCU {

public:
    JAUSOCU (void);
    ~JAUSOCU (void);
    
    int
    init (int compID, double state_freq, char *str_addr_NM, char *str_addr_PD, 
        char *str_addr_VSS, char *str_addr_MPD, char *str_addr_SigD, char *str_addr_EM);

    // set functions go LCM -> JAUS
    void
    set_discrete_devices (const perllcm::carlab_discrete_devices_t *msg);

    void
    set_signals (const perllcm::carlab_signals_t *msg);

    void
    set_wrench_effort (const perllcm::carlab_wrench_effort_t *msg);

    void
    send_heartbeat ();

    // register callbacks and send callback functions
    template<class CallbackHandlerClass, class MessageType>
    void register_cb (
            void (CallbackHandlerClass::*handlerMethod)(const MessageType *msg),
            CallbackHandlerClass *handler);
    template<class MessageType>
    void callback (MessageType *x);

    JausAddress get_addr_local_NM () { return this->addr_local_NM; }
    JausAddress get_addr_NM () { return this->addr_NM; }
    JausAddress get_addr_PD () { return this->addr_PD; }
    JausAddress get_addr_VSS () { return this->addr_VSS; }
    JausAddress get_addr_MPD () { return this->addr_MPD; }
    JausAddress get_addr_SigD () { return this->addr_SigD; }
    JausAddress get_addr_EM () { return this->addr_EM; }

    JausAddress get_owner_PD () { return this->owner_PD; }
    bool set_owner_PD (JausAddress addr) { 
        char s[30];
        jausAddressToString (addr, s);
        state->PD_control = std::string (s);
        return jausAddressCopy (this->owner_PD, addr); 
    }
    JausAddress get_owner_MPD () { return this->owner_MPD; }
    bool set_owner_MPD (JausAddress addr) { 
        char s[30];
        jausAddressToString (addr, s);
        state->MPD_control = std::string (s);
        return jausAddressCopy (this->owner_MPD, addr); 
    }

    perllcm::carlab_state_t *get_state () { return this->state; }
    void set_state (perllcm::carlab_state_t *st) { 
        st->utime = timestamp_now ();
        callback (st); 
        this->state = st; 
    }

    void add_service_connection (int sc) { this->service_connections.push_back (sc); }

private:
    OjCmpt myOCU; // needs freeing

    JausAddress addr_local; // needs jausAddressDestroy
    JausAddress addr_local_NM; // needs jausAddressDestroy
    JausAddress addr_NM;
    JausAddress addr_PD;
    JausAddress addr_VSS;
    JausAddress addr_MPD;
    JausAddress addr_SigD;
    JausAddress addr_EM;

    JausAddress owner_PD; // owner of Primitive Driver
    JausAddress owner_MPD; // owner of Motion Profile Driver

    SetWrenchEffortMessage set_wrench;
    SetDiscreteDevicesMessage set_disc_dev;
    SetSignalsMessage set_sigs;
    ReportHeartbeatPulseMessage heartbeat;

    perllcm::carlab_state_t *state;

    std::vector<BaseCallback *> callbacks;
    std::vector<int> service_connections;
};






// register callbacks and send callback functions
template<class CallbackHandlerClass, class MessageType>
void
JAUSOCU::register_cb ( void (CallbackHandlerClass::*handlerMethod)(const MessageType *msg),
        CallbackHandlerClass *handler)
{
    CallbackMessage<MessageType> *m = new CallbackMessage<MessageType>();
    m->callback = boost::bind (handlerMethod, handler, _1);
    this->callbacks.push_back (m);
}

template<class MessageType>
void
JAUSOCU::callback (MessageType *x)
{
    std::vector<BaseCallback*>::iterator iter = this->callbacks.begin();
    for (; iter != this->callbacks.end (); ++iter) {
        CallbackMessage<MessageType> *cb = dynamic_cast<CallbackMessage<MessageType>* > (*iter);
        if (cb == 0) continue;

        cb->callback (x);
        return;
    }

    printf ("ERROR: unregistered callback (JAUS -> Network)\n");
}







JausAddress
jausStringToAddress (char * str);

JausAddress
jausBytesToAddress (unsigned char subsystem, unsigned char node, 
        unsigned char component, unsigned char instance);

// JAUS Callbacks
void
jaus_component_status_cb (OjCmpt ocu, JausMessage msg);

void
jaus_component_control_cb (OjCmpt ocu, JausMessage msg);

void
jaus_wrench_effort_cb (OjCmpt ocu, JausMessage msg);

void
jaus_discrete_devices_cb (OjCmpt ocu, JausMessage msg);

void
jaus_operational_data_cb (OjCmpt ocu, JausMessage msg);

void
jaus_velocity_state_cb (OjCmpt ocu, JausMessage msg);

void
jaus_wheel_speeds_cb (OjCmpt ocu, JausMessage msg);

void
jaus_curvature_cb (OjCmpt ocu, JausMessage msg);

void
jaus_signals_cb (OjCmpt ocu, JausMessage msg);

void
jaus_error_count_cb (OjCmpt ocu, JausMessage msg);

void
jaus_error_cb (OjCmpt ocu, JausMessage msg);

// STATE MACHINE FUNCTIONS
void
ocu_initialize_state (OjCmpt ocu);

void
ocu_ready_state (OjCmpt ocu);

void
ocu_standby_state (OjCmpt ocu);

void
ocu_failure_state (OjCmpt ocu);
// END State Machine


#endif // __PERLS_JAUSOCU_H__

