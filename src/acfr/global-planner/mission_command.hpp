
#ifndef MISSION_COMMAND_HPP
#define MISSION_COMMAND_HPP

// Device that takes the command
typedef enum {
    CAMERA,
    DVL
}   missionCommandDeviceT;

// The actual command, some of these require a number or flag
typedef enum {
    ON_OFF,
    CAMERA_RATE,
    CAMERA_STROBE_DURATION,
    PD0,
    PD5
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
