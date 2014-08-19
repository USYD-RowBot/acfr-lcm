
#ifndef MISSION_COMMAND_HPP
#define MISSION_COMMAND_HPP

// Device that takes the command
typedef enum {
    CAMERA,
    DVL
}   missionCommandDeviceT;

// The actual command, some of these require a number or flag
typedef enum {
    CAMERA_START,
    CAMERA_STOP,
    CAMERA_FREQ,
    CAMERA_WIDTH,
    DVL_PD0,
    DVL_PD5,
    DVL_RANGE
} missionCommandTypeT;


// Class that holds a mission command
class MissionCommand {
    public:
        missionCommandDeviceT device;
        missionCommandTypeT command;
        int valueInt;
        double valueDouble;
};


#endif
