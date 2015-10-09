#include <string.h>
#include <stdint.h>

#include "perls-lcmtypes/senlcm_ms_gx3_45_t.h"

#include "ms_3dm_gx3_45.h"

uint16_t
ms_compute_checksum (const uint8_t *buf, int buflen)
{
    uint8_t byte1 = 0;
    uint8_t byte2 = 0;
    for (int i=0; i<buflen-2; i++)
    {
        byte1 += buf[i];
        byte2 += byte1;
    }

    return ((uint16_t) byte1 << 8) + (uint16_t) byte2;
}

int
ms_send_command (generic_sensor_driver_t *gsd, ms_mip_cmd_t cmd, int wait_ack)
{
    // build command
    int cmd_len = cmd.field_len + CMD_OVERHEAD_LEN + FIELD_OVERHEAD_LEN;
    uint8_t *tmp = calloc(cmd_len, sizeof (*tmp));
    tmp[0] = SYNC1;
    tmp[1] = SYNC2;
    tmp[2] = cmd.set_desc;
    tmp[3] = cmd.field_len + FIELD_OVERHEAD_LEN; // paylod len = field len if we are only sending one command at a time
    tmp[4] = cmd.field_len + FIELD_OVERHEAD_LEN;
    tmp[5] = cmd.cmd_desc;
    for (int i=0; i < cmd.field_len ; i++)
        tmp[6 + i] = cmd.field_data[i];

    uint16_t cs = ms_compute_checksum (tmp, cmd_len);
    tmp[cmd_len-2] = ((uint8_t) (cs >> 8) & 0x00FF);
    tmp[cmd_len-1] = ((uint8_t) cs & 0x00FF);

    // send command
    gsd_flush (gsd);
    gsd_write (gsd, (char *) tmp, cmd_len);
    free (tmp);

    if (wait_ack)
    {
        // read ACK
        uint8_t buf[ACK_LEN];
        int64_t timestamp;
        gsd_noncanonical (gsd, ACK_LEN, 0);
        int ret = gsd_read_timeout (gsd, (char *)buf, ACK_LEN, &timestamp, 1e6);

        // check correct length ACK
        // buf[6] should be cmd desc echo
        // buf[7] should be zero for no errors
        if (ACK_LEN == ret && buf[6] == cmd.cmd_desc && buf[7] == 0x00)
            return 1;
        else
            return 0;
    }
    else
        return 1;
}


void
ms_setup (generic_sensor_driver_t *gsd)
{
    ms_mip_cmd_t cmd = {0};

    //// reset the device
    //cmd.set_desc = BASE_CMD_SET,
    //cmd.cmd_desc = CMD_RESET,
    //cmd.field_len = 0,
    //cmd.field_data = NULL,
    //if (ms_send_command (gsd, cmd, 1)) {
    //    printf ("Reset device. \n");
    //} else {
    //    ERROR ("ERROR: Failed to reset device");
    //}
    //timeutil_usleep (2e6);

    uint8_t comm_field_data[2] = {0x01, 0x01};
    cmd.set_desc = SYS_CMD_SET;
    cmd.cmd_desc = CMD_COMM_MODE;
    cmd.field_len = 2;
    cmd.field_data = comm_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Setting comm mode. \n");
    else
        ERROR ("ERROR: Failed to set comm mode.");

    // put the device in idle mode
    cmd.set_desc = BASE_CMD_SET;
    cmd.cmd_desc = CMD_SET_TO_IDLE;
    cmd.field_len = CMD_SET_TO_IDLE_LEN;
    cmd.field_data = NULL;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Set to idle mode. \n");
    else
        ERROR ("ERROR: Failed to put into idle mode");


    // request what we want to know from the Kalhman filter
    uint8_t nav_field_data[2+3*9] = {0x01, 0x09,
                                     DATA_NAV_GPS_TIMESTAMP,  RATE_100HZ_b1, RATE_100HZ_b2,
                                     DATA_FILTER_STATUS,      RATE_100HZ_b1, RATE_100HZ_b2,
                                     DATA_EST_LLH_POS,        RATE_100HZ_b1, RATE_100HZ_b2,
                                     DATA_EST_NED_VEL,        RATE_100HZ_b1, RATE_100HZ_b2,
                                     DATA_EST_EULER_ANGLES,   RATE_100HZ_b1, RATE_100HZ_b2,
                                     DATA_EST_ANG_RATE,       RATE_100HZ_b1, RATE_100HZ_b2,
                                     DATA_EST_LLH_POS_STD,    RATE_100HZ_b1, RATE_100HZ_b2,
                                     DATA_EST_NED_VEL_STD,    RATE_100HZ_b1, RATE_100HZ_b2,
                                     DATA_EST_EULER_STD,      RATE_100HZ_b1, RATE_100HZ_b2,
                                    };
    cmd.set_desc = MS3DM_CMD_SET;
    cmd.cmd_desc = CMD_NAV_MSG_FMT;
    cmd.field_len = 2+3*9;
    cmd.field_data = nav_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Set NAV message format. \n");
    else
        ERROR ("ERROR: Failed to set NAV message format.");

    // save configuration
    uint8_t save_nav_field_data[2] = {0x03, 0x00};
    cmd.set_desc = MS3DM_CMD_SET;
    cmd.cmd_desc = CMD_NAV_MSG_FMT;
    cmd.field_len = 2;
    cmd.field_data = save_nav_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Saved NAV message format. \n");
    else
        ERROR ("ERROR: Failed to save NAV message format.");

    // set antenna offset CMD_ANTENNA_OFFSET
    uint8_t set_ant_field_data[13] = {0x01,
                                      0x00, 0x00, 0x00, 0x00,   // x meters in sensor body frame
                                      0x00, 0x00, 0x00, 0x00,   // y meters in sensor body frame
                                      0x00, 0x00, 0x00, 0x00
                                     };  // z meters in sensor body frame
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_ANTENNA_OFFSET;
    cmd.field_len = 13;
    cmd.field_data = set_ant_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Set antenna offset for NAV. \n");
    else
        ERROR ("ERROR: Failed to set antenna offset for NAV.");

    // save antenna offset CMD_ANTENNA_OFFSET
    uint8_t save_ant_field_data[13] = {0x01,
                                       0x00, 0x00, 0x00, 0x00,   // x meters in sensor body frame
                                       0x00, 0x00, 0x00, 0x00,   // y meters in sensor body frame
                                       0x00, 0x00, 0x00, 0x00
                                      };  // z meters in sensor body frame
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_ANTENNA_OFFSET;
    cmd.field_len = 13;
    cmd.field_data = save_ant_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Saved antenna offset for NAV. \n");
    else
        ERROR ("ERROR: Failed to save antenna offset for NAV.");

    // set vehicle dynamics CMD_SET_VEHICLE_DYNAMICS
    uint8_t set_dyn_field_data[2] = {0x01, 0x02};
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_SET_VEHICLE_DYNAMICS;
    cmd.field_len = 2;
    cmd.field_data = set_dyn_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Set vehicle dynamics for NAV. \n");
    else
        ERROR ("ERROR: Failed to set vehicle dynamics for NAV.");

    // save vehicle dynamics CMD_SET_VEHICLE_DYNAMICS
    uint8_t save_dyn_field_data[2] = {0x03, 0x02};
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_SET_VEHICLE_DYNAMICS;
    cmd.field_len = 2;
    cmd.field_data = save_dyn_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Saved vehicle dynamics for NAV. \n");
    else
        ERROR ("ERROR: Failed to save vehicle dynamics for NAV.");

    // set heading update source
    uint8_t head_src_field_data[2] = {0x01, 0x02}; //gps velocity heading
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_HEADING_UPDATE_CTRL;
    cmd.field_len = 2;
    cmd.field_data = head_src_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Set heading source for NAV. \n");
    else
        ERROR ("ERROR: Failed to set heading source for NAV.");

    // save heading update source
    uint8_t save_head_src_field_data[2] = {0x03, 0x02}; //gps velocity heading
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_HEADING_UPDATE_CTRL;
    cmd.field_len = 2;
    cmd.field_data = save_head_src_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Saved heading source for NAV. \n");
    else
        ERROR ("ERROR: Failed to save heading source for NAV.");

    // set to auto initialize
    uint8_t auto_init_nav_field_data[2] = {0x01, 0x01};
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_AUTO_INIT_CTRL;
    cmd.field_len = 2;
    cmd.field_data = auto_init_nav_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Turn on auto init NAV. \n");
    else
        ERROR ("ERROR: Failed to turn on auto init NAV.");

    // save auto init
    uint8_t save_auto_init_nav_field_data[2] = {0x03, 0x01};
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_AUTO_INIT_CTRL;
    cmd.field_len = 2;
    cmd.field_data = save_auto_init_nav_field_data;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Save auto init NAV. \n");
    else
        ERROR ("ERROR: Failed to save auto init NAV.");

    // reset the filter
    cmd.set_desc = NAV_CMD_SET;
    cmd.cmd_desc = CMD_RESET_FILTER;
    cmd.field_len = 0;
    cmd.field_data = NULL;
    if (ms_send_command (gsd, cmd, 1))
        printf ("Reset filter. \n");
    else
        ERROR ("ERROR: Failed to reset filter");

    // old setup options when I was trying to stream the data
    //// resume device
    //cmd.set_desc = BASE_CMD_SET;
    //cmd.cmd_desc = CMD_RESUME;
    //cmd.field_len = 0;
    //cmd.field_data = NULL;
    //if (ms_send_command (gsd, cmd, 1)) {
    //    printf ("Resumed device. \n");
    //} else {
    //    ERROR ("ERROR: Failed to resume device.");
    //}

    //uint8_t en_nav_field_data[3] = {0x01, 0x03, 0x01}; //apply new settings, to the nav, enabled
    //cmd.set_desc = MS3DM_CMD_SET;
    //cmd.cmd_desc = CMD_ED_CONT_DATA_STREAM;
    //cmd.field_len = 3;
    //cmd.field_data = en_nav_field_data;
    //ms_send_command (gsd, cmd, 0);
}

int
ms_verify_checksum (uint8_t *buf, int len)
{
    uint16_t checksum = ((uint16_t) buf[len-2] << 8);
    checksum += ((uint16_t) buf[len-1]) & 0x00FF;
    uint16_t calc_checksum =  ms_compute_checksum (buf, len);

    if (checksum == calc_checksum)
        return 1;
    else
        return 0;
}

static void
ms_raw_to_double(uint8_t *raw, double *d, int num_doubles)
{
    uint8_t *bpd = (uint8_t *)d;
    int size = sizeof (double);

    for (int i = 0; i < num_doubles; i++)
        for (int j = 0; j < size; j++)
            bpd[i*size+(size-j-1)] = raw[i*size+j];
}

static void
ms_raw_to_float(uint8_t *raw, float *d, int num_floats)
{
    uint8_t *bpd = (uint8_t *)d;
    int size = sizeof (float);

    for (int i = 0; i < num_floats; i++)
        for (int j = 0; j < size; j++)
            bpd[i*size+(size-j-1)] = raw[i*size+j];
}

uint16_t
ms_raw_to_uint16_t (uint8_t *raw)
{
    return (uint16_t)(raw[0] << 8) + (uint16_t)raw[1];
}

int
ms_parse_nav_packet (generic_sensor_driver_t *gsd, senlcm_ms_gx3_45_t* msg_out, uint8_t *buf, int len)
{
    //verify packet type
    if (buf[0] != SYNC1 || buf[1] != SYNC2 || buf[2] != DATA_NAV_SET)
        return 0;
    //verify checksum
    if (!ms_verify_checksum (buf, len))
        return 0;

    //int payload_len = buf[3];
    uint8_t *start_p = buf;
    uint8_t *p = &buf[4];

    while (p < (start_p+len-2))
    {
        int field_len = p[0];
        uint8_t field_desc = p[1];

        switch (field_desc)
        {
        case DATA_NAV_GPS_TIMESTAMP:
// not correct currently
            ms_raw_to_double(&p[2], &msg_out->gps_timeofweek_seconds, 1);
            msg_out->gps_week_number = ms_raw_to_uint16_t(&p[10]);
            msg_out->gps_timestamp_valid = ms_raw_to_uint16_t(&p[12]);
            break;
        case DATA_FILTER_STATUS:
            msg_out->filter_state = ms_raw_to_uint16_t(&p[2]);
            msg_out->dynamics_mode = ms_raw_to_uint16_t(&p[4]);
            msg_out->status_flags = ms_raw_to_uint16_t(&p[6]);
            break;
        case DATA_EST_LLH_POS:
            ms_raw_to_double(&p[2], msg_out->lat_lon_alt, 3);
            msg_out->lat_lon_alt[0] *= UNITS_DEGREE_TO_RADIAN;
            msg_out->lat_lon_alt[1] *= UNITS_DEGREE_TO_RADIAN;
            msg_out->lat_lon_alt[2] *= UNITS_DEGREE_TO_RADIAN;
            msg_out->lat_lon_alt_valid = ms_raw_to_uint16_t(&p[26]);
            break;
        case DATA_EST_NED_VEL:
            ms_raw_to_float(&p[2], msg_out->ned_vel, 3);
            msg_out->ned_vel_valid = ms_raw_to_uint16_t(&p[14]);
            break;
        case DATA_EST_EULER_ANGLES:
            ms_raw_to_float(&p[2], msg_out->euler, 3);
            msg_out->euler_valid = ms_raw_to_uint16_t(&p[14]);
            break;
        case DATA_EST_ANG_RATE:
            ms_raw_to_float(&p[2], msg_out->lat_lon_alt_std, 3);
            msg_out->lat_lon_alt_std_valid = ms_raw_to_uint16_t(&p[14]);
            break;
        case DATA_EST_LLH_POS_STD:
            ms_raw_to_float(&p[2], msg_out->ang_rate, 3);
            msg_out->ang_rate_valid = ms_raw_to_uint16_t(&p[14]);
            break;
        case DATA_EST_NED_VEL_STD:
            ms_raw_to_float(&p[2], msg_out->ned_vel_std, 3);
            msg_out->ned_vel_std_valid = ms_raw_to_uint16_t(&p[14]);
            break;
        case DATA_EST_EULER_STD:
            ms_raw_to_float(&p[2], msg_out->euler_std, 3);
            msg_out->euler_std_valid = ms_raw_to_uint16_t(&p[14]);
            break;
        default :
            ERROR ("Unknown nav data packet field type desc = %02X", field_desc);
            return 0;
        }

        p += field_len;
    }

    return 1;
}


