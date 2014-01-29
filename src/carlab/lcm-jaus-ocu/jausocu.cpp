#include "jausocu.h"


//----------------------------------------------------------------------------------
// Constructor
//----------------------------------------------------------------------------------
// set initial values
// create JAUS component and start it/state machine
JAUSOCU::JAUSOCU (void)
{
    this->init (2, 1.0,
            const_cast<char*>("1.1.1.1"),
            const_cast<char*>("1.1.33.1"),
            const_cast<char*>("1.1.42.1"),
            const_cast<char*>("1.1.10.1"),
            const_cast<char*>("1.1.11.1"),
            const_cast<char*>("1.1.12.1"));
}


//----------------------------------------------------------------------------------
// Deconstructor
//----------------------------------------------------------------------------------
// shutdown JAUS component
// cleanup
JAUSOCU::~JAUSOCU (void)
{
    printf ("cleaning up ocu...\n");

    setWrenchEffortMessageDestroy (this->set_wrench);
    setDiscreteDevicesMessageDestroy (this->set_disc_dev);
    setSignalsMessageDestroy (this->set_sigs);

    for (std::vector<int>::iterator it = this->service_connections.begin();
            it != this->service_connections.end(); ++it)
        ojCmptTerminateSc (this->myOCU, *it);

    jausAddressDestroy (this->addr_local);
    jausAddressDestroy (this->addr_local_NM);
    jausAddressDestroy (this->addr_NM);
    jausAddressDestroy (this->addr_PD);
    jausAddressDestroy (this->addr_VSS);
    jausAddressDestroy (this->addr_MPD);
    jausAddressDestroy (this->addr_SigD);
    jausAddressDestroy (this->addr_EM);
    jausAddressDestroy (this->owner_PD);
    jausAddressDestroy (this->owner_MPD);

    delete this->state;

    // destroy component
    ojCmptDestroy (this->myOCU);
}


int
JAUSOCU::init (int compID, double state_freq, char *str_addr_NM, char *str_addr_PD, 
        char *str_addr_VSS, char *str_addr_MPD, char *str_addr_SigD, char *str_addr_EM)
{
    // create the JAUS component, compID should be non-standard component ID
    this->myOCU = ojCmptCreate (const_cast<char*>("OCU"), compID, state_freq);

    // check that component succeeded in checking in with node manager
    if (this->myOCU == NULL) {
        printf ("Error starting component\n");
        return -1;
    }

    // no services provided *from* OCU (this would be the place to register them)

    // register component callback for all states
    ojCmptSetStateCallback (this->myOCU, JAUS_INITIALIZE_STATE, ocu_initialize_state);
    ojCmptSetStateCallback (this->myOCU, JAUS_READY_STATE, ocu_ready_state);
    ojCmptSetStateCallback (this->myOCU, JAUS_STANDBY_STATE, ocu_standby_state);
    ojCmptSetStateCallback (this->myOCU, JAUS_FAILURE_STATE, ocu_failure_state);
    ojCmptSetState (this->myOCU, JAUS_INITIALIZE_STATE);

    // set used data and start component state machine
    ojCmptSetUserData (this->myOCU, (void*)this);

    this->addr_local = ojCmptGetAddress (this->myOCU);
    this->addr_local_NM = jausBytesToAddress (this->addr_local->subsystem, this->addr_local->node,
            1, 1);
    this->addr_NM = jausStringToAddress (str_addr_NM);
    this->addr_PD = jausStringToAddress (str_addr_PD);
    this->addr_VSS = jausStringToAddress (str_addr_VSS);
    this->addr_MPD = jausStringToAddress (str_addr_MPD);
    this->addr_SigD = jausStringToAddress (str_addr_SigD);
    this->addr_EM = jausStringToAddress (str_addr_EM);

    this->owner_PD = jausAddressCreate ();
    this->owner_MPD = jausAddressCreate ();

    // set up wrench effort command
    this->set_wrench = setWrenchEffortMessageCreate ();
    this->set_wrench->presenceVector = 0;
	jausUnsignedShortSetBit (&this->set_wrench->presenceVector, JAUS_WRENCH_PV_PROPULSIVE_LINEAR_X_BIT);
	jausUnsignedShortSetBit (&this->set_wrench->presenceVector, JAUS_WRENCH_PV_PROPULSIVE_ROTATIONAL_Z_BIT);
	jausUnsignedShortSetBit (&this->set_wrench->presenceVector, JAUS_WRENCH_PV_RESISTIVE_LINEAR_X_BIT);

    // set up discrete devices command
    this->set_disc_dev = setDiscreteDevicesMessageCreate ();
    this->set_disc_dev->presenceVector = 0;
	jausByteSetBit (&this->set_disc_dev->presenceVector, JAUS_DEVICES_PV_PROPULSION_BIT);
	jausByteSetBit (&this->set_disc_dev->presenceVector, JAUS_DEVICES_PV_GEAR_BIT);

    // set up signals command
    this->set_sigs = setSignalsMessageCreate ();
    this->set_sigs->presenceVector = 0;
	jausByteSetBit (&this->set_sigs->presenceVector, JAUS_SIGNALS_PV_TURN_SIGNALS_BIT);
	jausByteSetBit (&this->set_sigs->presenceVector, JAUS_SIGNALS_PV_HORN_BIT);
	jausByteSetBit (&this->set_sigs->presenceVector, JAUS_SIGNALS_PV_HEADLIGHTS_BIT);

    // set up heartbeat command
    this->heartbeat = reportHeartbeatPulseMessageCreate ();

    this->state = new perllcm::carlab_state_t ();
    this->state->PD_control = std::string ("0.0.0.0");
    this->state->MPD_control = std::string ("0.0.0.0");

    // ojCmptRun returns 0 on success
    return ojCmptRun (this->myOCU);
}


void
JAUSOCU::set_discrete_devices (const perllcm::carlab_discrete_devices_t *msg)
{
    // JAUS message 0x0406 -- Set Discrete Devices

    // check state and ensure control of primitive driver
    if (!jausAddressEqual (this->addr_local, this->owner_PD)) {
        printf ("Primitive Driver not owned by OCU, discrete devices ignored.\n");
        return;
    }

    // package message
    this->set_disc_dev->mainPropulsion = msg->engine_on ? JAUS_TRUE : JAUS_FALSE;
    this->set_disc_dev->mainFuelSupply = msg->engine_on ? JAUS_TRUE : JAUS_FALSE;
    this->set_disc_dev->automaticStart = msg->engine_on ? JAUS_TRUE : JAUS_FALSE;
    this->set_disc_dev->automaticStop = msg->engine_on ? JAUS_FALSE : JAUS_TRUE;
    this->set_disc_dev->gear = msg->gear;

    // send message
    //if (!ojCmptLookupAddress (this->myOCU, this->addr_PD)) {
        //printf ("Primitive Driver does not exist as specified, discrete devices ignored.\n");
        //return;
    //}

    jausAddressCopy (this->set_disc_dev->destination, this->addr_PD);
    JausMessage jm = setDiscreteDevicesMessageToJausMessage (this->set_disc_dev);
    ojCmptSendMessage (this->myOCU, jm);
    jausMessageDestroy (jm);
}

void
JAUSOCU::set_signals (const perllcm::carlab_signals_t *msg)
{
    // JAUS experimental message 0xE322 -- Set Signals

    // check state -- no control needed over Signal Component
    // package message
    this->set_sigs->turnSignalStatus = msg->turn_signal;
    this->set_sigs->hornStatus = (msg->horn_on == 1) ? JAUS_TRUE : JAUS_FALSE;
    this->set_sigs->headlightsStatus = msg->headlights;
    this->set_sigs->highbeamsStatus = (msg->highbeams_on == 1) ? JAUS_TRUE : JAUS_FALSE;
    this->set_sigs->foglightsStatus = (msg->foglights_on == 1) ? JAUS_TRUE : JAUS_FALSE;

    // send message
    if (!ojCmptLookupAddress (this->myOCU, this->addr_SigD)) {
        printf ("Signals Driver does not exist as specified, set signals ignored.\n");
        return;
    }

    jausAddressCopy (this->set_sigs->destination, this->addr_SigD);
    JausMessage jm = setSignalsMessageToJausMessage (this->set_sigs);
    ojCmptSendMessage (this->myOCU, jm);
    jausMessageDestroy (jm);
}

void
JAUSOCU::set_wrench_effort (const perllcm::carlab_wrench_effort_t *msg)
{
    // JAUS message 0x0405 -- Set Wrench Effort

    // check state and ensure control of primitive driver
    if (!jausAddressEqual (this->addr_local, this->owner_PD)) {
        printf ("Primitive Driver not owned by OCU, wrench efforts ignored.\n");
        return;
    }

    // package message
    this->set_wrench->propulsiveLinearEffortXPercent = GSL_MIN (GSL_MAX (msg->throttle, 0), 100);
    this->set_wrench->propulsiveRotationalEffortZPercent = GSL_MIN (GSL_MAX (msg->steer_angle, -100), 100);
    this->set_wrench->resistiveLinearEffortXPercent = GSL_MIN (GSL_MAX (msg->brakes, 0), 100);
    
    printf ("~~~%4.4f\n", this->set_wrench->propulsiveLinearEffortXPercent);
    printf ("~~~%4.4f\n", this->set_wrench->propulsiveRotationalEffortZPercent);
    printf ("~~~%4.4f\n", this->set_wrench->resistiveLinearEffortXPercent);

    // send message
    //if (~ojCmptLookupAddress (this->myOCU, this->addr_PD)) {
        //printf ("Primitive Driver does not exist as specified, wrench efforts ignored.\n");
        //return;
    //}

    jausAddressCopy (this->set_wrench->destination, this->addr_PD);
    JausMessage jm = setWrenchEffortMessageToJausMessage (this->set_wrench);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    ojCmptSendMessage (this->myOCU, jm);
    jausMessageDestroy (jm);
}

void
JAUSOCU::send_heartbeat ()
{
    printf ("sent: HEARTBEAT\n");
    jausAddressCopy (heartbeat->destination, this->addr_local_NM);
    JausMessage jm = reportHeartbeatPulseMessageToJausMessage (this->heartbeat);
    ojCmptSendMessage (this->myOCU, jm);
    jausMessageDestroy (jm);
}


//----------------------------------------------------------------------------------
// Used to convert 4 bytes to jaus address subsystem.node.component.instance
//----------------------------------------------------------------------------------
JausAddress
jausBytesToAddress (unsigned char subsystem, unsigned char node, 
        unsigned char component, unsigned char instance)
{
    JausAddress tmp = jausAddressCreate ();
    tmp->subsystem = subsystem;
    tmp->node = node;
    tmp->component = component;
    tmp->instance = instance;

    return tmp;
}

//----------------------------------------------------------------------------------
// Used to convert a 32-bit jaus address string to the JausAddress struct
//----------------------------------------------------------------------------------
JausAddress
jausStringToAddress (char * str)
{
    char *token, *next_token;
    char cpy[20]; snprintf (cpy, 20, "%s", str);
    JausAddress tmp = jausAddressCreate ();

    token = strtok_r (cpy, ".", &next_token);
    if ((atoi(token) & 0xFF) == atoi(token)) tmp->subsystem = atoi (token);
    token = strtok_r (next_token, ".", &next_token);
    if ((atoi(token) & 0xFF) == atoi(token)) tmp->node = atoi (token);
    token = strtok_r (next_token, ".", &next_token);
    if ((atoi(token) & 0xFF) == atoi(token)) tmp->component = atoi (token);
    token = strtok_r (next_token, ".", &next_token);
    if ((atoi(token) & 0xFF) == atoi(token)) tmp->instance = atoi (token);

    return tmp;
}

// JAUS Callbacks
void
jaus_component_status_cb (OjCmpt ocu, JausMessage msg)
{
    ReportComponentStatusMessage status_msg = reportComponentStatusMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (status_msg) {
        if (jausAddressEqual (status_msg->source, jausocu->get_addr_NM())) {
        }
        else if (jausAddressEqual (status_msg->source, jausocu->get_addr_PD())) {
        }
        else if (jausAddressEqual (status_msg->source, jausocu->get_addr_MPD())) {
        }

        reportComponentStatusMessageDestroy (status_msg);
    }
}

void
jaus_component_control_cb (OjCmpt ocu, JausMessage msg)
{
    ReportComponentControlMessage control_msg = reportComponentControlMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (control_msg) {
        if (jausAddressEqual (control_msg->source, jausocu->get_addr_PD())) {

            JausAddress tmp_addr = jausBytesToAddress (control_msg->subsystemId, 
                                                        control_msg->nodeId, 
                                                        control_msg->componentId, 
                                                        control_msg->instanceId);
            jausocu->set_owner_PD (tmp_addr);
            jausAddressDestroy (tmp_addr);

        }
        else if (jausAddressEqual (control_msg->source, jausocu->get_addr_MPD())) {

            JausAddress tmp_addr = jausBytesToAddress (control_msg->subsystemId, 
                                                        control_msg->nodeId, 
                                                        control_msg->componentId, 
                                                        control_msg->instanceId);
            jausocu->set_owner_MPD (tmp_addr);
            jausAddressDestroy (tmp_addr);

        }

        reportComponentControlMessageDestroy (control_msg);
    }
}

void
jaus_confirm_component_control_cb (OjCmpt ocu, JausMessage msg)
{
    ConfirmComponentControlMessage control_msg = confirmComponentControlMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (control_msg) {
        if (jausAddressEqual (control_msg->source, jausocu->get_addr_PD())) {

            jausocu->set_owner_PD (control_msg->destination);

        }
        else if (jausAddressEqual (control_msg->source, jausocu->get_addr_MPD())) {

            jausocu->set_owner_MPD (control_msg->destination);

        }

        confirmComponentControlMessageDestroy (control_msg);
    }
}

void
jaus_wrench_effort_cb (OjCmpt ocu, JausMessage msg)
{
    ReportWrenchEffortMessage we_msg = reportWrenchEffortMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (we_msg) {
        perllcm::carlab_wrench_effort_t m;

        m.utime = timestamp_now ();
        m.throttle = we_msg->propulsiveLinearEffortXPercent;
        m.steer_angle = we_msg->propulsiveRotationalEffortZPercent;
        m.brakes = we_msg->resistiveLinearEffortXPercent;

        jausocu->callback (&m);

        reportWrenchEffortMessageDestroy (we_msg);
    }
}

void
jaus_discrete_devices_cb (OjCmpt ocu, JausMessage msg)
{
    ReportDiscreteDevicesMessage dd_msg = reportDiscreteDevicesMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (dd_msg) {
        perllcm::carlab_discrete_devices_t m;

        m.utime = timestamp_now ();
        m.engine_on = dd_msg->mainPropulsion && dd_msg->mainFuelSupply;
        m.gear = dd_msg->gear;

        jausocu->callback (&m);

        reportDiscreteDevicesMessageDestroy (dd_msg);
    }
}

void
jaus_operational_data_cb (OjCmpt ocu, JausMessage msg)
{
    ReportPlatformOperationalDataMessage pod_msg = reportPlatformOperationalDataMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (pod_msg) {
        perllcm::carlab_state_t *s = jausocu->get_state();
        s->fuel_level = pod_msg->fuelLevelPercent;
        jausocu->set_state (s);

        reportPlatformOperationalDataMessageDestroy (pod_msg);
    }
}

void
jaus_velocity_state_cb (OjCmpt ocu, JausMessage msg)
{
    ReportVelocityStateMessage vel_msg = reportVelocityStateMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (vel_msg) {
        perllcm::carlab_state_t *s = jausocu->get_state();
        s->velocity = vel_msg->velocityXMps;
        jausocu->set_state (s);

        reportVelocityStateMessageDestroy (vel_msg);
    }
}

void
jaus_wheel_speeds_cb (OjCmpt ocu, JausMessage msg)
{
    ReportWheelSpeedsMessage ws_msg = reportWheelSpeedsMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (ws_msg) {
        perllcm::carlab_state_t *s = jausocu->get_state();
        s->RF_wheel_speed = ws_msg->rightFrontWheelSpeed;
        s->LF_wheel_speed = ws_msg->leftFrontWheelSpeed;
        s->RR_wheel_speed = ws_msg->rightRearWheelSpeed;
        s->LR_wheel_speed = ws_msg->leftRearWheelSpeed;
        jausocu->set_state (s);

        reportWheelSpeedsMessageDestroy (ws_msg);
    }
}

void
jaus_curvature_cb (OjCmpt ocu, JausMessage msg)
{
    ReportCurvatureMessage curv_msg = reportCurvatureMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (curv_msg) {
        perllcm::carlab_state_t *s = jausocu->get_state();
        s->curvature = curv_msg->arctanCurvature;
        jausocu->set_state (s);

        reportCurvatureMessageDestroy (curv_msg);
    }
}

void
jaus_signals_cb (OjCmpt ocu, JausMessage msg)
{
    ReportSignalsMessage sig_msg = reportSignalsMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (sig_msg) {
        perllcm::carlab_signals_t m;

        m.utime = timestamp_now ();
        m.turn_signal = sig_msg->turnSignalStatus;
        m.horn_on = sig_msg->hornStatus;
        m.headlights = sig_msg->headlightsStatus;
        m.highbeams_on = sig_msg->highbeamsStatus;
        m.foglights_on = sig_msg->foglightsStatus;

        jausocu->callback (&m);

        reportSignalsMessageDestroy (sig_msg);
    }
}

void
jaus_error_count_cb (OjCmpt ocu, JausMessage msg)
{
    ReportErrorCountMessage ec_msg = reportErrorCountMessageFromJausMessage (msg);
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (ec_msg) {
        perllcm::carlab_state_t *s = jausocu->get_state();
        s->error_count = ec_msg->numErrors;
        jausocu->set_state (s);
        reportErrorCountMessageDestroy (ec_msg);
    }
}

void
jaus_error_cb (OjCmpt ocu, JausMessage msg)
{
    printf ("ERROR CB\n");
    ReportErrorMessage err_msg = reportErrorMessageFromJausMessage (msg);
    //JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);

    if (err_msg) {
        reportErrorMessageDestroy (err_msg);
    }
}

void
jaus_heartbeat (OjCmpt ocu, JausMessage msg)
{
    ReportHeartbeatPulseMessage status_msg = reportHeartbeatPulseMessageFromJausMessage (msg);

    if (status_msg) {
        char str[100];
        jausAddressToString (status_msg->source, str);
        printf ("HEARTBEAT: %s\n", str);

        reportHeartbeatPulseMessageDestroy (status_msg);
    }
}

//----------------------------------------------------------------------------------
// Called when component is in initialize state at state machine frequency
//----------------------------------------------------------------------------------
void
ocu_initialize_state (OjCmpt ocu)
{
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);
    printf ("state: INITIALIZE\n");



    // wait until XGV is visible
    //if (!ojCmptLookupAddress (ocu, jausocu->get_addr_NM())) 
        //return;
    QueryHeartbeatPulseMessage hm = queryHeartbeatPulseMessageCreate ();
    jausAddressCopy (hm->destination, jausocu->get_addr_NM());
    JausMessage j = queryHeartbeatPulseMessageToJausMessage (hm);
    ojCmptSendMessage (ocu, j);
    jausMessageDestroy (j);
    queryHeartbeatPulseMessageDestroy (hm);
    ojCmptSetMessageCallback (ocu, JAUS_REPORT_HEARTBEAT_PULSE, jaus_heartbeat);

    // register Callbacks and Service Connections
    ojCmptSetMessageCallback (ocu, JAUS_CONFIRM_COMPONENT_CONTROL, jaus_confirm_component_control_cb);

    ojCmptSetMessageCallback (ocu, JAUS_REPORT_COMPONENT_STATUS, jaus_component_status_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_COMPONENT_STATUS, 0xFF, jausocu->get_addr_NM(), 1, 1, 1));
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_COMPONENT_STATUS, 0xFF, jausocu->get_addr_PD(), 1, 1, 1));
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_COMPONENT_STATUS, 0xFF, jausocu->get_addr_MPD(), 1, 1, 1));

    ojCmptSetMessageCallback (ocu, JAUS_REPORT_COMPONENT_CONTROL, jaus_component_control_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_COMPONENT_CONTROL, 0xFF, jausocu->get_addr_PD(), 5, 1, 1));
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_COMPONENT_CONTROL, 0xFF, jausocu->get_addr_MPD(), 5, 1, 1));

    ojCmptSetMessageCallback (ocu, JAUS_REPORT_WRENCH_EFFORT, jaus_wrench_effort_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_WRENCH_EFFORT, 0xFF, jausocu->get_addr_PD(), 20, 1, 1));

    ojCmptSetMessageCallback (ocu, JAUS_REPORT_DISCRETE_DEVICES, jaus_discrete_devices_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_DISCRETE_DEVICES, 0xFF, jausocu->get_addr_PD(), 10, 1, 1));

    ojCmptSetMessageCallback (ocu, JAUS_REPORT_PLATFORM_OPERATIONAL_DATA, jaus_operational_data_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_PLATFORM_OPERATIONAL_DATA, 0xFF, jausocu->get_addr_PD(), 5, 1, 1));

    // add wheel speeds
    ojCmptSetMessageCallback (ocu, JAUS_REPORT_WHEEL_SPEEDS, jaus_wheel_speeds_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_WHEEL_SPEEDS, 0xFF, jausocu->get_addr_PD(), 50, 1, 1));

    ojCmptSetMessageCallback (ocu, JAUS_REPORT_VELOCITY_STATE, jaus_velocity_state_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_VELOCITY_STATE, 0xFF, jausocu->get_addr_VSS(), 1, 1, 1));

    // add curvature
    ojCmptSetMessageCallback (ocu, JAUS_REPORT_CURVATURE, jaus_curvature_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_CURVATURE, 0xFF, jausocu->get_addr_MPD(), 50, 1, 1));

    // add signals
    ojCmptSetMessageCallback (ocu, JAUS_REPORT_SIGNALS, jaus_signals_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_SIGNALS, 0xFF, jausocu->get_addr_SigD(), 5, 1, 1));

    // add errors
    ojCmptSetMessageCallback (ocu, JAUS_REPORT_ERROR_COUNT, jaus_error_count_cb);
    jausocu->add_service_connection (ojCmptEstablishSc (ocu, JAUS_REPORT_ERROR_COUNT, 0xFF, jausocu->get_addr_EM(), 5, 1, 1));
    ojCmptSetMessageCallback (ocu, JAUS_REPORT_ERROR, jaus_error_cb);

    // move onto standby state
    ojCmptSetState (ocu, JAUS_STANDBY_STATE);

    jausocu->send_heartbeat ();
}

//----------------------------------------------------------------------------------
// Called when component is in ready state at state machine frequency
//----------------------------------------------------------------------------------
void
ocu_ready_state (OjCmpt ocu)
{
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);
    printf ("state: READY\n");

    // check messsage age, switch to standby if too old

    jausocu->send_heartbeat ();
}

//----------------------------------------------------------------------------------
// Called when component is in standby state at state machine frequency
//----------------------------------------------------------------------------------
void
ocu_standby_state (OjCmpt ocu)
{
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);
    printf ("state: STANDBY\n");


    RequestComponentControlMessage rcc = requestComponentControlMessageCreate ();
    jausAddressCopy (rcc->destination, jausocu->get_addr_PD());
    rcc->authorityCode = ojCmptGetAuthority (ocu);
    JausMessage jm = requestComponentControlMessageToJausMessage (rcc);
    ojCmptSendMessage (ocu, jm);
    jausMessageDestroy (jm);
    requestComponentControlMessageDestroy (rcc);

    ojCmptSetState (ocu, JAUS_READY_STATE);

    jausocu->send_heartbeat ();
}

//----------------------------------------------------------------------------------
// Called when component is in failure state at state machine frequency
//----------------------------------------------------------------------------------
void
ocu_failure_state (OjCmpt ocu)
{
    JAUSOCU *jausocu = (JAUSOCU*) ojCmptGetUserData (ocu);
    printf ("state: FAILURE\n");
    // reserved for future use with dedicated e-stop on xbox controller

    jausocu->send_heartbeat ();
}
