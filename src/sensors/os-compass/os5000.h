#ifndef __OS5000_H__
#define __OS5000_H__

#include <stdint.h>

#define OS5000_BITMASK_H     0x0001
#define OS5000_BITMASK_P     0x0002
#define OS5000_BITMASK_R     0x0004
#define OS5000_BITMASK_T     0x0008
#define OS5000_BITMASK_D     0x0010
#define OS5000_BITMASK_M     0x0020
#define OS5000_BITMASK_M_XYZ 0x0040
#define OS5000_BITMASK_A     0x0080
#define OS5000_BITMASK_A_XYZ 0x0100

#ifdef __cplusplus
extern "C" {
#endif

typedef struct os5000 os5000_t;
struct os5000 {
    uint16_t bitmask;
    double H, P, R;    // heading, pitch, and roll angles in degrees
    double T;          // Temperature of compass board in Celsius
    double D;          // depth in feet of water or volts depending on compass configuration
    double M, Mxyz[3]; // Magnetic field strength in milligauss
    double A, Axyz[3]; // Acceleration in units of "G"
};

// Format Type 0x01: "$C" format
os5000_t
os5000_parse_0x01 (const char buf[], size_t len);

static inline double
os5000_volts_to_psi (double volts)
{
    // US131-000005-010BA SN:01281069
    // 10  Bar Absolute

    // 10 bar in psi = 145.0
    //double psi = 145.0/2.25 * (volts - 0.25); // this is incorrect see explanation below
    
    double psi = 145.0/2.0 * (volts - 0.25);
    return psi;


    // US131-000005-300PA SN:0708121043
    // 300 PSI Absolute

    // voltage range = 0.5 .. 4.5 V
    // 300 PSI = 4.5 V
    // 0 PSI = 0.5 V
    // compass-os AD halves the voltage 
    // 4.5V -> 2.25V, 0.5V -> 0.25V
    //
    // 2.25 - 0.25 = 2.0
    // psi = 300 * (Volts - 0.25)/2.0

    //double psi = 300/2.0 * (volts - 0.25);
    //return psi;


    // M5131-000005-250PG
    // SN: 021714D055
    // 250 PSI Gauge

    //double psi = 250/2.0 * (volts - 0.25);
    //return psi;

}


#ifdef __cplusplus
}
#endif

#endif //__OS5000_H__
