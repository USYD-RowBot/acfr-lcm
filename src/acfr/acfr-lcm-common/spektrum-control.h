// TODO ifdef for compile


//Joystick Constants
#define RC_DEADZONE 80 // Testing w DX5 160902016 JJM

// Switch Constants (3 position flick switches)
// Test values front: 342, center: 1024, rear: 1706 DX6 24012018 JM
#define REAR_POS_CUTOFF 1300
#define CENTER_POS_CUTOFF 700


// RC Channel names
enum
{
    RC_THROTTLE = 0,
    RC_AILERON,
    RC_ELEVATOR,
    RC_RUDDER,
    RC_GEAR,
    RC_AUX1,
};

// RC Control Source (controlled by RC_AUX1 3 position switch)
enum rc_control_source_t
{   
    RC_MODE_RC = 0,
 	RC_MODE_ZERO,
    RC_MODE_AUTO,
};


