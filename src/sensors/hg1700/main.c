// LCM driver for the Honeywell HG1700 IMU
// using code written by Duncan Mercer
// Christian Lees
// ACFR
// 18/11/10

// Headers to deal with the Commsync Driver
#include <comsync/CSPCI_pset.h>
//#include <comsync/sdlc_settings.h>
#include <comsync/ioctls.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <libgen.h>

#include <bot_param/param_client.h>

#include "perls-common/timestamp.h"
#include "perls-lcmtypes/senlcm_IMU_t.h"

#define ANG_RATE_SCALE 5.722045898e-4
#define ACCEL_SCALE 0.011162109
#define DELTA_ANG_SCALE 1.164153218e-10
#define DELTA_VEL_SCALE 1.490116119e-8

#ifndef BOT_CONF_DIR
#define DEFAULT_BOT_CONF_PATH "../config/master.cfg"
#else
#define DEFAULT_BOT_CONF_PATH BOT_CONF_DIR "/master.cfg"
#endif

// globals are bad, mmmmk
int imuExit;

void initialisePortSettings(int portnum, BHN_PortSettings *pset)
{

    // Use port settings specific for the imu...
    pset->port=portnum;

    //	Serial mode
    //		The transmitter and receiver serial mode is usually the same,
    //      but the IUSC does not explicity enforce this.
    //		Although, there are combinations that won't operate properly.
    pset->smode.tx= SMODE_HDLC;
    pset->smode.rx= SMODE_HDLC;

    //	Reciver clocking selections
    //	See: CSPCI_pset.h for details
    pset->rxclk.ref_freq=0;	        // ref_freq not needed
    pset->rxclk.bps_error=0;		// bps_error not needed
    pset->rxclk.bps=0;		     	// bps not needed
    pset->rxclk.bps_frac=0;			// bps_frac not needed
    pset->rxclk.async_div=0;		// async_div not needed
    pset->rxclk.dpll_div=0;			// dpll_div not needed
    pset->rxclk.ctr_div=0;			// ctr_div not needed
    pset->rxclk.enc_dec=NRZ;	         // data decoding
    pset->rxclk.clk_tree.A=CLK_RXC;	     // clocked from RXC pin
    pset->rxclk.clk_tree.B=DPLL_BRG0;    // from BRG0
    pset->rxclk.clk_tree.C=BRG_CTR0;     // not needed
    pset->rxclk.clk_tree.D=CTR_DISABLE;  // not needed
    pset->rxclk.clk_pin=XC_INP;			 // RxC pin is an input

    //	Transmitter clocking selections
    //	See: CSPCI_pset.h for details
    pset->txclk.ref_freq=0;	    // ref_freq not needed
    pset->txclk.bps_error=0;	// bps_error not needed
    pset->txclk.bps=0;			// bps not needed
    pset->txclk.bps_frac=0;			// bps_frac not needed
    pset->txclk.async_div=0;			// async_div not needed
    pset->txclk.dpll_div=0;			// dpll_div not needed
    pset->txclk.ctr_div=0;			// ctr_div not needed
    pset->txclk.enc_dec=NRZ;	    	// data decoding
    pset->txclk.clk_tree.A=CLK_NONE;	    // No clocking needed
    pset->txclk.clk_tree.B=DPLL_BRG0;    // from BRG0
    pset->txclk.clk_tree.C=BRG_CTR0;     // not needed
    pset->txclk.clk_tree.D=CTR_DISABLE;  // not needed
    pset->txclk.clk_pin=XC_INP;			// RxC pin is an input

    //	TX Preamble and Idle
    //	See: Transmitter Idles and Preamble Patterns in CSPCI_pset.h
    pset->tx_pre_idle.pre_pat=0;  //	preamble pattern
    pset->tx_pre_idle.pre_len=0;  //	Not required 8,16,32,64 bits
    pset->tx_pre_idle.tx_idle=IDLE_DEF;	// Tx nothing by default

    // Electrical line interface mode plus termination
    // Terminate bus & RS422 with V.10 DTR. See SP508 data sheet for details
    pset->line_mode=	LM_530A | LM_TERM;

    //	Duplex mode. Use in full duplex mode
    pset->duplex_mode=DUPLEX_FULL;

    // Sync characters (or HDLC addresses)
    pset->sync_addr.tx0=0;
    pset->sync_addr.tx1=0;
    pset->sync_addr.tx_len=8;
    pset->sync_addr.rx0=0x0A; // 1st IMU address byte
    pset->sync_addr.rx1=0;    // 2nd address byte
    pset->sync_addr.rx_len=8; // 8 bit sync char
    pset->sync_addr.strip_sync=0;	// don't strip SYN chars

    //	Data bits (number of)
    pset->dbits.tx=8;	//	1 to 8
    pset->dbits.rx=8;	//	1 to 8

    // Async Mode Transmitter (fractional) Stop bits
    pset->tx_frac_stop=0;

    //	Parity selections
    pset->parity.tx=PAR_NONE;
    pset->parity.rx=PAR_NONE;

    //	CRC selections
    pset->crc.tx_type=CRC_NONE;           // TX CRC setup
    pset->crc.tx_start=CRC_START_0;
    pset->crc.tx_mode=CRC_TX_UNDERRUN;
    pset->crc.rx_type=CCITT;              // RX CRC We want to use the 16bit CCITT version
    pset->crc.rx_start=CRC_START_1;       // This CRC should start pre loaded with all ones.

    // ASYNC parameters (inherited from older BHN_PortSettings structure)
    // See CSPCI_pset.h for more details...
    pset->async_settings.sflow = 'n';
    pset->async_settings.xoff = 0x13;
    pset->async_settings.xon = 0x11;
    pset->async_settings.hflow = 0;


    // HDLC/SDLC Settings
    // Define these so we match a 1 byte address and keep a 1 byte control ID
    pset->hdlc.tx_flag_share_zero=0;                 // tx_flag_share_zero-> We do not share the zeros
    pset->hdlc.rx_addr_control=HDLC_RX_ADDR1_CONT1;  // 1 byte address and 1 byte control (MSGID) field
    pset->hdlc.tx_underrun=HDLC_UNDERRUN_ABORT_7;    // Use the standard underrun sequence

}

// initializes a port
int start_port(int portPort, BHN_PortSettings *pset)
{
    char port_str[20];
    int ret;
    int fd;

    // Make this port blocking and read only
    int mode = O_RDONLY & ~O_NONBLOCK;

    sprintf(port_str,"/dev/csync%d",pset->port+1);
    fd = open(port_str, mode);
    if( fd == -1)
    {
        printf(" %s open error...\n",port_str);
        abort();
    }

    // This call passes the settings to the kernel driver.
    ret = ioctl(fd, TIOSETPORTSET, pset);
    if (ret != 0)
    {
        printf("TIOSETPORTSET failed ret 0x%x. Error codes in CSPCI_pset.h\n", ret);
        abort();
    }

    return fd;
}


void start_new_frame(int portFd)
{

    int ret;

    // Before we start force a flush of the Rx buffer
    ret = ioctl(portFd, TCFLSH, TCIFLUSH);
    if(ret != 0)
    {
        printf("Error flushing port\n");
        abort();
    }

    // Now put into hunt mode to find the start of the next frame
    ret = ioctl(portFd, TIOHUNT, NULL);
    if(ret != 0)
    {
        printf("Error enabling hunt\n");
        abort();
    }

}



// Get the next inertial frame from the port. Discarding any
// flight control frames as we go. Returns -1 for a failure, or the total
// number of bytes recieved in the current frame. The frame returned has
// the address, checksum and message ID bytes stripped


int read_inertial_frame(int portFd, unsigned char* frame_buf)
{

#define ADDRESS_CTRL_SIZE  2
#define FLIGHT_CTRL_SIZE   16
#define INERTIAL_SIZE      24
#define COMBINED_DATA_SIZE (FLIGHT_CTRL_SIZE + INERTIAL_SIZE)
#define CHECKSUM_SIZE      2   // 16bit message checksum

#define TOTAL_COMB_SIZE    (COMBINED_DATA_SIZE + ADDRESS_CTRL_SIZE + CHECKSUM_SIZE )

    // Define the biggest possible frame size and then allow some extra...
#define MAX_FRAME_SIZE     (TOTAL_COMB_SIZE + 10)

#define ADDR_BYTE          0x00 // 1st byte in message is the address byte
#define ID_BYTE            0x01 // 2nd byte in message is the message ID
    // Message ID values...
#define CTRL_MSG           0x01 // Flight control only message
#define INERT_MSG          0x02 // An inertial + flight control message

    int current_Nbytes = 0;

    unsigned long events=0;

    while(1)
    {
        // First we want the first two bytes. These are the address and control bytes.
        int Nbytes = ADDRESS_CTRL_SIZE;
        int len = read(portFd, frame_buf, Nbytes);
        if(len < 0)
        {
            printf("read frame failed errno %d\n",errno);
            start_new_frame(portFd); // reset for next time...
            return -1;
        }

        // This should not happen as we are using blocking IO
        if(len == 0)
        {
            continue;
        }

        //Make sure that the data looks like a frame. This should be caught by the receiver
        if(!frame_buf[ADDR_BYTE] == 0x0A)
        {
            printf("Frame address incorrect\n");
            start_new_frame(portFd); // reset for next time...
            return -1;
        }


        // Work out which message type we have and how many extra bytes are required.
        int MsgID = frame_buf[ID_BYTE];
        switch(MsgID)
        {
        case CTRL_MSG:
            Nbytes = FLIGHT_CTRL_SIZE + CHECKSUM_SIZE;
            break;
        case INERT_MSG:
            Nbytes = INERTIAL_SIZE + FLIGHT_CTRL_SIZE + CHECKSUM_SIZE;
            break;
        default:
            printf("Frame ID incorrect [%c]\n",MsgID);
            start_new_frame(portFd); // reset for next time...
            return -1;
        }

        // Note once we have checked the address and message ID bytes we will overwrite them.
        // How many bytes have we received in the current chunk..
        current_Nbytes = 0;

        // Get the rest of the frame.
        while(current_Nbytes < Nbytes)
        {

            // We need to clear the events flags in the driver before continuing...
            int ret = ioctl(portFd, TIOGETEVENT, &events);
            if ( ret != 0)
            {
                printf("TIOGETEVENT failed ret %x. Error codes in CSPCI_pset.h\n", ret);
                return -1;
            }

            // Get the next block of data
            len = read(portFd, frame_buf + current_Nbytes, Nbytes - current_Nbytes);
            if(len < 0)
            {
                printf("read frame failed errno %d\n",errno);
                start_new_frame(portFd); // reset for next time...
                return -1;
            }
            current_Nbytes += len;
        }

        // If this frame is not an inertial message, discard it and keep looking...
        if(MsgID == CTRL_MSG)
            continue;

        // Ask the driver for "events" any CRC errors should show up here.
        // This happens because the hardware is doing an on the fly CRC test.
        int ret = ioctl(portFd, TIOGETEVENT, &events);
        if (ret != 0)
        {
            printf("TIOGETEVENT failed ret %x. Error codes in CSPCI_pset.h\n", ret);
            return -1;
        }
        if(events > 0 )
        {
            printf("Failed events check, 0x%lx. Most likely CRC problem\n",events);
            start_new_frame(portFd); // reset for next time...
            return -1;
        }

        // If we get here then we have a valid inertial frame. So continue...
        break;
    }

    // Don't tell the outside world that our checksum exists...
    return current_Nbytes - CHECKSUM_SIZE;
}

// decode the IMU frame and publish it on LCM
void decodeFrame(short *frame, lcm_t *lcm)
{

    senlcm_IMU_t imu;
    imu.angRate[0] = (double)frame[0] * ANG_RATE_SCALE;
    imu.angRate[1] = (double)frame[1] * ANG_RATE_SCALE;
    imu.angRate[2] = (double)frame[2] * ANG_RATE_SCALE;

    imu.accel[0] = (double)frame[3] * ACCEL_SCALE;
    imu.accel[1] = (double)frame[4] * ACCEL_SCALE;
    imu.accel[2] = (double)frame[5] * ACCEL_SCALE;

    imu.deltaAngle[0] = (double)frame[8] * DELTA_ANG_SCALE;
    imu.deltaAngle[1] = (double)frame[9] * DELTA_ANG_SCALE;
    imu.deltaAngle[2] = (double)frame[10] * DELTA_ANG_SCALE;

    imu.deltaVelocity[0] = (double)frame[11] * DELTA_VEL_SCALE;
    imu.deltaVelocity[1] = (double)frame[12] * DELTA_VEL_SCALE;
    imu.deltaVelocity[2] = (double)frame[13] * DELTA_VEL_SCALE;

    imu.utime = timestamp_now();

    senlcm_IMU_t_publish(lcm, "IMU", &imu);
}


// we want to exit cleanly
void signalHandler(int sigNum)
{
    // do a safe exit
    imuExit = 1;
}

int main(int argc, char *argv[])
{

    // Sync port operation struct vars
    BHN_PortSettings pset;

    int portFd;
    int portPort;
    unsigned char frame_buffer[MAX_FRAME_SIZE];

    // set up the signal handler
    imuExit = 0;
    signal(SIGINT, signalHandler);

    // create an LCM instance
    lcm_t *lcm = lcm_create (NULL);


    BotParam *cfg;
    char rootkey[64];
    char *path = getenv ("BOT_CONF_PATH");
    if (!path)
        path = DEFAULT_BOT_CONF_PATH;
    cfg = bot_param_new_from_file(path);
    if(cfg == NULL)
    {
        printf("cound not open config file\n");
        return 0;
    }

    sprintf (rootkey, "sensors.%s", basename (argv[0]));

    // get the slam config filenames from the master LCM config
    char key[128];
    int imuPort;
    sprintf(key, "%s.imuPort", rootkey);
    imuPort = bot_param_get_int_or_fail(cfg,key);

    // set up the port
    initialisePortSettings(imuPort, &pset);
    portPort = pset.port;
    portFd = start_port(portPort, &pset);

    // now we loop
    while(!imuExit)
    {
        start_new_frame(portFd);
        while(!imuExit)
        {
            int len=read_inertial_frame(portFd, frame_buffer);
            if(len <= 0)
            {
                printf("read frame failed\n");
                continue;
            } // Unspecified error. We need to try again.

            decodeFrame((short *)frame_buffer, lcm);
        }
    }

    // clean up
    close(portFd);
}

