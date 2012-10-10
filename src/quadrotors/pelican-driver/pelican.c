#include <string.h>
#include <stdint.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/senlcm_pelican_t.h"

#include "pelican.h"

uint16_t
pelic_crcUpdate (uint16_t crc, uint8_t data)
{  
    data ^= (crc & 0xff);
    data ^= data << 4;

    return ((((uint16_t) data << 8) | ((crc>>8)&0xff)) ^ (uint8_t) (data >> 4) ^((uint16_t) data << 3 ));
}

uint16_t
pelic_crc16 (void *data, uint16_t cnt)
{  
    uint16_t crc = 0xff;
    uint8_t *ptr = data;

    for (int i=0; i<cnt; i++) {
        crc = pelic_crcUpdate (crc, *ptr);
        ptr++;
    }

    return crc;
}

uint8_t
pelic_verifyChecksum (uint8_t *buf, int bufLen)
{
    uint8_t newBuf[bufLen];
    memcpy (newBuf, buf, bufLen);
    void *structPointer = newBuf + 6;
    uint16_t lengthOfStruct = newBuf[3];
    uint16_t checksum = pelic_crc16 (structPointer, lengthOfStruct);
    uint16_t sum = *(uint16_t*)(newBuf+3+2+1+lengthOfStruct);

    return (checksum == sum) ? 1 : 0;
}

void
pelic_parseImuRawData (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic)
{  
    puts ("WTF");
    return;
}

uint8_t
pelic_parsePacket (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic)
{  
    if (!pelic_verifyChecksum (buf, bufLen))
        return 0;

    uint8_t packetDescriptor = buf[5]; // see manual

    switch (packetDescriptor) {
    case PELIC_LLSTATUS_ID :
        pelic_parseLlStatus (buf, bufLen, pelic);
        break;
    case PELIC_IMURAWDATA_ID :
        pelic_parseImuRawData (buf, bufLen, pelic);
        break;
    case PELIC_IMUCALCDATA_ID :
        pelic_parseImuCalcData (buf, bufLen, pelic);
        break;
    case PELIC_RCDATA_ID :
        pelic_parseRcData (buf, bufLen, pelic);
        break;
    case PELIC_GPSDATA_ID :
        pelic_parseGpsData (buf, bufLen, pelic);
        break;
    default:
        //unknown packet type
        return 0;
        break;
    }

    return 1;
}

void
pelic_parseGpsData (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic)
{  
    pelic_GpsDataStruct rawPacket;
    uint16_t length = (uint16_t)buf[3];
    uint8_t *structPointer = buf + 6;

    memcpy (&rawPacket, structPointer, length);

    //Assign stuff to the lcm struct, convert to double as described in manual:
    pelic->latitude = (rawPacket.latitude / 100000000.0) * UNITS_DEGREE_TO_RADIAN;
    pelic->longitude = (rawPacket.longitude / 100000000.0) * UNITS_DEGREE_TO_RADIAN;
    pelic->heightGps = rawPacket.height / 1000.0;
    pelic->speedGps[0] = rawPacket.speed_x / 1000.0;
    pelic->speedGps[1] = rawPacket.speed_y / 1000.0;
    pelic->heading = (rawPacket.heading / 1000.0) * UNITS_DEGREE_TO_RADIAN;
    pelic->horizontal_accuracy = rawPacket.horizontal_accuracy / 1000.0;
    pelic->vertical_accuracy = rawPacket.vertical_accuracy / 1000.0;
    pelic->speed_accuracy = rawPacket.speed_accuracy / 1000.0;
    pelic->numSV = rawPacket.numSV;
    pelic->statusGps = rawPacket.status;
}

void
pelic_parseLlStatus (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic)
{  
    pelic_LlStatusStruct rawPacket;
    uint16_t length = (uint16_t)buf[3];
    uint8_t *structPointer = buf + 6;

    memcpy (&rawPacket, structPointer, length);

    //Assign stuff to the lcm struct, convert to double as described in manual:
    pelic->battery_voltage = rawPacket.battery_voltage_1/1000.0;
    pelic->status = rawPacket.status;
    pelic->cpu_load = rawPacket.cpu_load;
    pelic->compass_enabled = rawPacket.compass_enabled;
    pelic->chksum_error = rawPacket.chksum_error;
    pelic->flying = rawPacket.flying;
    pelic->motors_on = rawPacket.motors_on;
    pelic->flightMode = rawPacket.flightMode;
    pelic->up_time = rawPacket.up_time;
}

void
pelic_parseImuCalcData (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic)
{
    pelic_ImuCalcDataStruct rawPacket;
    uint16_t length = (uint16_t)buf[3];
    uint8_t *structPointer = buf + 6;

    memcpy (&rawPacket, structPointer, length);

    //Assign stuff to lcm struct, convert to double as needed
    pelic->angle[0] = (rawPacket.angle_nick / 1000.0) * UNITS_DEGREE_TO_RADIAN;
    pelic->angle[1] = (rawPacket.angle_roll / 1000.0) * UNITS_DEGREE_TO_RADIAN;
    pelic->angle[2] = (rawPacket.angle_yaw  / 1000.0) * UNITS_DEGREE_TO_RADIAN;

    pelic->angvel[0] = rawPacket.angvel_nick;
    pelic->angvel[1] = rawPacket.angvel_roll;
    pelic->angvel[2] = rawPacket.angvel_yaw;

    pelic->acc_calib[0] = rawPacket.acc_x_calib / 10000.0;
    pelic->acc_calib[1] = rawPacket.acc_y_calib / 10000.0;
    pelic->acc_calib[2] = rawPacket.acc_z_calib / 10000.0;
  
    pelic->acc[0] = rawPacket.acc_x / 10000.0;
    pelic->acc[1] = rawPacket.acc_y / 10000.0;
    pelic->acc[2] = rawPacket.acc_z / 10000.0;

    pelic->acc_angle[0] = (rawPacket.acc_angle_nick / 1000.0) * UNITS_DEGREE_TO_RADIAN;
    pelic->acc_angle[1] = (rawPacket.acc_angle_roll / 1000.0) * UNITS_DEGREE_TO_RADIAN;

    pelic->acc_absolute_value = rawPacket.acc_absolute_value / 10000.0;

    pelic->magField[0] = rawPacket.Hx;
    pelic->magField[1] = rawPacket.Hy;
    pelic->magField[2] = rawPacket.Hz;

    pelic->mag_heading = (rawPacket.mag_heading / 1000.0) * UNITS_DEGREE_TO_RADIAN;

    // Documentation is vague - speed might have to be divided by 10000
    pelic->speed[0] = rawPacket.speed_x / 1000.0;
    pelic->speed[1] = rawPacket.speed_y / 1000.0;
    pelic->speed[2] = rawPacket.speed_z / 1000.0;

    pelic->heightImu = rawPacket.height / 1000.0;
    pelic->dheight = rawPacket.dheight / 1000.0;
    pelic->dheight_reference = rawPacket.dheight_reference / 1000.0;
    pelic->height_reference = rawPacket.height_reference / 1000.0;
}

void
pelic_parseRcData (uint8_t *buf, int bufLen, senlcm_pelican_t *pelic)
{  
    pelic_RcDataStruct rawPacket;

    uint16_t length = (uint16_t)buf[3];
    uint8_t *structPointer = buf + 6;

    memcpy (&rawPacket, structPointer, length);

    //We only care about the normalized RC data, which ranges from 0 to
    //4095 for each channel 
    for (int i=0; i<7; i++)
        pelic->rcData[i] = rawPacket.channels_out[i];
}
