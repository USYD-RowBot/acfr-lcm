// type declarations

#define UInt8 unsigned char
#define UInt16 unsigned short
#define UInt32 unsigned int
#define Float32 float

typedef struct
{
    UInt8 pollingMode, flushFilter;
    Float32 sensorAcqTime, intervalRespTime;
} __attribute__ ((packed)) AcqParams;

typedef struct
{
    Float32 MagCalScore;
    Float32 reserve1;
    Float32 AccelCalScore;
    Float32 DistErr;
    Float32 TiltErr;
    Float32 TiltRange;
} __attribute__ ((packed)) CalScore;

enum
{
    // Frame IDs (Commands)
    kGetModInfo = 1, // 1
    kModInfoResp,     // 2
    kSetDataComponents,     // 3
    kGetData, // 4
    kDataResp, // 5
    kSetConfig, // 6
    kGetConfig, // 7
    kConfigResp,      // 8
    kSave,       // 9
    kStartCal, // 10
    kStopCal, // 11
    kSetParam, // 12
    kGetParam, // 13
    kParamResp, // 14
    kPowerDown, // 15
    kSaveDone, // 16
    kUserCalSampCount,      // 17
    kUserCalScore,    // 18
    kSetConfigDone, // 19
    kSetParamDone,    // 20
    kStartIntervalMode,     // 21
    kStopIntervalMode,      // 22
    kPowerUp, // 23
    kSetAcqParams,    // 24
    kGetAcqParams,    // 25
    kAcqParamsDone, // 26
    kAcqParamsResp, // 27
    kPowerDoneDown, // 28
    kFactoryUserCal, // 29
    kFactoryUserCalDone,    // 30
    kTakeUserCalSample,     // 31
    kFactoryInclCal = 36,   // 36
    kFactoryInclCalDone,    // 37
    kSetMode = 46,    // 46
    kSetModeDone,     // 47
    kSyncRead = 49, // 49
    // Cal Option IDs
    kFullRangeCal = 10,     // 10 - type Float32
    k2DCal = 20,      // 20 - type Float32
    kHIOnlyCal = 30, // 30 - type Float32
    kLimitedTiltCal = 40, // 40 - type Float32
    kAccelCalOnly = 100,    // 100 - type Float32
    kAccelCalwithMag = 110,       // 110 - type Float32
    // Param IDs
    kFIRConfig = 3,
    // 3- AxisID(UInt8)+Count(UInt8)+Value(Float64)+...
    // Data Component IDs
    kHeading = 5,     // 5 - type Float32
    kTemperature = 7, // 7 - type Float32
    kDistortion = 8, // 8 - type boolean
    kPAligned = 21, // 21 - type Float32
    kRAligned, // 22 - type Float32
    kIZAligned, // 23 - type Float32
    kPAngle,    // 24 - type Float32
    kRAngle,    // 25 - type Float32
    kXAligned = 27, // 27 - type Float32
    kYAligned, // 28 - type Float32
    kZAligned, // 29 - type Float32
    // Configuration Parameter IDs
    kDeclination = 1, // 1 - type Float32
    kTrueNorth, // 2 - type boolean
    kMountingRef = 10,      // 10 - type UInt8
    kUserCalStableCheck,    // 11 - type boolean
    kUserCalNumPoints,      // 12 - type UInt32
    kUserCalAutoSampling, // 13 - type boolean
    kBaudRate, // 14 - UInt8
    kMilOutPut = 15, // 15 - type Boolean
    kDataCal,    // 16 - type Boolean
    kCoeffCopySet = 18,     // 18 - type UInt32
    kAccelCoeffCopySet,     // 19 - type UInt32

    // Mounting Reference IDs
    kMountedStandard = 1, //   1
    kMountedXUp,      // 2
    kMountedYUp,      // 3
    kMountedStdPlus90,      // 4
    kMountedStdPlus180,     // 5
    kMountedStdPlus270,     // 6
    kMountedZDown,     // 7
    kMountedXUpPlus90, // 8
    kMountedXUpPlus180,      // 9
    kMountedXUpPlus270,      // 10
    kMountedYUpPlus90, // 11
    kMountedYUpPlus180,      // 12
    kMountedYUpPlus270,      // 13
    kMountedZDownPlus90,     // 14
    kMountedZDownPlus180,    // 15
    kMountedZDownPlus270,    // 16
    // Result IDs
    kErrNone = 0,     // 0
    kErrSave, // 1
};

// function to calculate CRC-16
UInt16 tcm_crc(void * data, UInt32 len)
{
    UInt8 * dataPtr = (UInt8 *)data;
    UInt32 index = 0;
    // Update the CRC for transmitted and received data using
    // the CCITT 16bit algorithm (X^16 + X^12 + X^5 + 1).
    UInt16 crc = 0;
    while(len--)
    {
        crc = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= dataPtr[index++];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }
    return crc;
}

