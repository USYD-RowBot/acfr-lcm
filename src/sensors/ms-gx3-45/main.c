#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/getopt.h"
#include "perls-common/error.h"
#include "perls-common/units.h"
#include "perls-common/generic_sensor_driver.h"

#include "perls-lcmtypes/senlcm_ms_gx3_45_t.h"

#include "ms_3dm_gx3_45.h"

#define INST_MAGFIELD SENLCM_MS_GX3_45_T_INST_MAGFIELD
#define INST_ACCEL    SENLCM_MS_GX3_45_T_INST_ACCEL
#define INST_ANGRATE  SENLCM_MS_GX3_45_T_INST_ANGRATE
#define STAB_MAGFIELD SENLCM_MS_GX3_45_T_STAB_MAGFIELD
#define STAB_ACCEL    SENLCM_MS_GX3_45_T_STAB_ACCEL
#define STAB_ANGRATE  SENLCM_MS_GX3_45_T_STAB_ANGRATE
#define STAB_EULER    SENLCM_MS_GX3_45_T_STAB_EULER
#define STAB_M        SENLCM_MS_GX3_45_T_STAB_M
#define STAB_Q        SENLCM_MS_GX3_45_T_STAB_Q
#define TEMPERATURE   SENLCM_MS_GX3_45_T_TEMPERATURE

static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Microstrain 3DM-GX3-45 MEMS-INS sensor driver.");
//    getopt_add_bool (gsd->gopt, '\0', "stabAttitude", 
//		     0, "Report stabilized attitude (default)");

    return 0;
}



void
clean_up (generic_sensor_driver_t *gsd, void *user)
{    
    // put the device in idle mode before shuting down (allows for clean restart)
    ms_mip_cmd_t cmd = {
        .set_desc = BASE_CMD_SET,
        .cmd_desc = CMD_SET_TO_IDLE,
        .field_len = CMD_SET_TO_IDLE_LEN,
        .field_data = NULL,
    };
    ms_send_command (gsd, cmd, 0);
}



int
main (int argc, char *argv[])
{
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_launch (gsd);
    
    //timestamp_sync_state_t *tss = timestamp_sync_init (DEV_TICKS_PER_SECOND, 65536, 101.0/100.0);
  
    // turn off stat printing
    gsd_print_stats (gsd, 0);
    // attach a destroy cllaback ()
    gsd_attach_destroy_callback (gsd, clean_up, NULL);
    
    // send commands to setup ms
    ms_setup (gsd);
    
    // allocate output lcm type
    senlcm_ms_gx3_45_t msg_out = {0};
    
    // turn on stat printing
    gsd_print_stats (gsd, 1);
    // setup polling command
    const int reply_len = 152;
    uint8_t poll_data[2] = {0x01, 0x00}; // suppress ack
    ms_mip_cmd_t poll_nav_cmd = {
        .set_desc = MS3DM_CMD_SET,
        .cmd_desc = CMD_POLL_NAV_DATA,
        .field_len = 2,
        .field_data = poll_data,
    };
    gsd_noncanonical (gsd, reply_len, 0);

    while (1) {
        
        // publish nav
        uint8_t buf[MICROSTRAIN_MAX_LEN];
        
        // send polling command
        ms_send_command (gsd, poll_nav_cmd, 0);
        
        // read reply
        int ret = gsd_read_timeout (gsd, (char *)buf, MICROSTRAIN_MAX_LEN, NULL, 1e6);
        
        //printf ("ret = %d \n", ret);
        //for (int i=0; i<ret; i++){
        //    printf ("%02X ", buf[i]);    
        //}
        //printf ("\n");    
 
        if (0 == ret)
            gsd_update_stats (gsd, 0);
        else {
            if (ms_parse_nav_packet (gsd, &msg_out, buf, ret)) {
                senlcm_ms_gx3_45_t_publish(gsd->lcm, gsd->channel, &msg_out);
                gsd_update_stats (gsd, 1);
            }
            else
                gsd_update_stats (gsd, -1);
        }
  
  /*
   
    uint8_t read_buf[MICROSTRAIN_MAX_LEN] = {0};
    uint8_t out_buf[MICROSTRAIN_MAX_LEN] = {0};
    int64_t timestamp;
    int ret;
    int payload_len = 0; int payload_cnt = 0;
    uint16_t checksum = 0;
        ret = gsd_read_timeout (gsd, (char *)read_buf, MICROSTRAIN_MAX_LEN, &timestamp, 1e6);
        // loop over current read (Kinda gross)
        for (int i = 0; i < ret ; i++) {
          
            if (SYNC1 != out_buf[0]) {
                if (SYNC1 == read_buf[i]){
                    out_buf[0] = SYNC1;
                }
            } else if (SYNC2 != out_buf[1]) {
                if (SYNC2 == read_buf[i]){
                    out_buf[1] = SYNC2;
                }
                else{
                    out_buf[0] = 0; // didnt find both syncs in a row restart
                }
            } else if (SYNC1 == out_buf[0] && SYNC2 == out_buf[1]) {
                if (out_buf[2] == 0) { // set desc
                    out_buf[2] = read_buf[i];
                } else if (payload_len == 0) {
                    payload_len = read_buf[i];
                    out_buf[3] = payload_len;
                } else if (payload_cnt < payload_len) {
                    out_buf[4+payload_cnt] = read_buf[i];
                    payload_cnt++;
                } else if (out_buf[payload_len+CMD_OVERHEAD_LEN-2] == 0) {
                    checksum = ((uint16_t) read_buf[i] << 8);
                    out_buf[payload_len+CMD_OVERHEAD_LEN-2] = read_buf[i];
                } else if (out_buf[payload_len+CMD_OVERHEAD_LEN-1] == 0) {
                    checksum += ((uint16_t) read_buf[i]) & 0x00FF;
                    out_buf[payload_len+CMD_OVERHEAD_LEN-1] = read_buf[i];
                    
                    if (checksum == ms_compute_checksum (out_buf, payload_len+CMD_OVERHEAD_LEN)) { // check checksum
                        gsd_update_stats (gsd, 1);
                    } else {
                        gsd_update_stats (gsd, -1);
                    }
                    
                    // reset
                    payload_len = payload_cnt = checksum = 0;
                    memset(out_buf, 0, MICROSTRAIN_MAX_LEN);
                    
                }
            } else {
                ERROR ("Should not be here");
            }
            
        }
  */    
    }

}
