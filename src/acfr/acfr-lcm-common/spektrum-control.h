// TODO ifdef for compile


//Joystick Constants
#define RC_DEADZONE 80 // Testing w DX5 160902016 JM
#define HALF_DEADZONE RC_DEADZONE/2
// Test values from DX6 27012018 JM
#define LEFT_POS 1707
#define RIGHT_POS 346
#define UP_POS 1707
#define DOWN_POS 346
#define CENTR_POS 1024
#define VERT_RANGE UP_POS-DOWN_POS
#define HORZ_RANGE LEFT_POS-RIGHT_POS
#define AVAIL_V_RANGE VERT_RANGE-RC_DEADZONE
#define AVAIL_H_RANGE HORZ_RANGE-RC_DEADZONE


// Switch Constants (3 position flick switches @ top cnrs)
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
    RC_AUX1, // use for rc source control 
    RC_GEAR, // use for controller drive modes
};

// RC Control Source (controlled by RC_AUX1 3 position switch)
enum rc_control_source_t
{   
    RC_MODE_RC = 0,
 	RC_MODE_ZERO,
    RC_MODE_AUTO,
};


