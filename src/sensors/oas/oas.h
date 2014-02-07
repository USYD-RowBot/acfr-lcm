/* ======================================================================
**
** WHOI SeaBED Crossbow sensor header file
**
** History:
**       Date       Who      Description
**      -------     ---      --------------------------------------------
**     20020215     OP       Imagenex header file
**     20040412     cnr      added the _PROFILE and _TRACE options for the return header
**     20080904     djlm     Improved on some of the definitions
**
** ======================================================================
*/

#ifndef OAS_H
#define OAS_H


// When running in trace as opposed to profile mode.
// get the echo intensities at 252 or 500 different times per pulse.
#define IMA_LONG_N_POINTS         500
#define IMA_SHORT_N_POINTS        252

#define IMA_CENTER_STEP           600
#define IMA_PROFRNG_FACTOR        0.01  // meters per count
#define IMA_LOG_FILE             "RAW"           // extension of logger raw data file

#define IMA_HEADER1               0xFE
#define IMA_HEADER2               0x44
#define IMA_HEADID                0x11
#define IMA_TERMBYTE              0xFD

#define IMA_SRD_HD1               'I'
#define IMA_SRD_HD2_PROFILE       'P'
#define IMA_SRD_HD2_TRACE1        'G'
#define IMA_SRD_HD2_TRACE2        'M'
#define IMA_SRD_TERM               0xFC
#define IMA_SRD_LENGTH            13 //including the termination byte

//SWITCH DATA COMMAND BYTE DESCRIPTIONS
enum IMA_Data{
    IMA_HEAD1=0,   
    IMA_HEAD2,              //   1
    IMA_H_ID,               //   2
    IMA_RNG,                //   3
    IMA_RNG_O,              //   4
    IMA_HOLD,               //   5
    IMA_MA_SL,              //   6 
    IMA_R_HSTAT,            //   7   
    IMA_S_GAIN,             //   8
    IMA_LOGF,               //   9
    IMA_ABSV,               //   10
    IMA_TRAIN,              //   11
    IMA_SECTW,              //   12
    IMA_STEP,               //   13 
    IMA_PULSE,              //   14
    IMA_LPF,                //   15
    IMA_ST_SCAN,            //   16
    IMA_MOVE_REL,           //   17
    IMA_NSWEEPS,            //   18
    IMA_NDPOINTS,           //   19
    IMA_NDBITS,             //   20
    IMA_UPBAUD,             //   21
    IMA_PROF,               //   22
    IMA_CALIB,              //   23
    IMA_DELAY,              //   24 
    IMA_FREQ,               //   25
    IMA_TERM,               //   26
    IMA_TOTAL_SIZE          //   27 "Total size" must remain the last entry 
}IMA_data;


#endif //OAS_H
