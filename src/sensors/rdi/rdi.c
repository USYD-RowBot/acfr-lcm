#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "perls-common/units.h"

#include "perls-math/gsl_util.h"
#include "perls-math/ssc.h"

#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/senlcm_rdi_bathy_t.h"


#include "rdi.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)


int
rdi_verify_checksum (const char *buf, int len)
{
    /* Bytes 3,4 contain the number of bytes from the start
       of the current ensemble up to, but no including, the 2-byte checksum 
    */
    uint16_t nbytes = *((uint16_t *) (buf+2));
    if (nbytes != (len-2))
        return -1;

    /* The last two bytes contain a modulo 65536 checksum.  the ExplorerDVL
       computes the checksum by summing all the bytes in the output buffer
       excluding the checksum.
    */
    uint16_t checksum = *((uint16_t *) (buf+len-2));

    uint16_t mychecksum = 0;
    const char *mybuf=buf;
    for (int i=0; i < nbytes; i++) {
        mychecksum += *((uint8_t *) mybuf++);
    }
    
    if (mychecksum == checksum)
        return 0;
    else
        return -1;
}
int
rdi_parse_pd4 (const char *buf, int len, rdi_pd4_t *pd4)
{
    if (0!=rdi_verify_checksum (buf, len) || len!=RDI_PD4_LEN) {
	printf("Vaild on checksum\n");
        return -1;
    }

    memcpy (pd4, buf, sizeof (rdi_pd4_t));
    if (pd4->header_id!=RDI_PD45_HEADER || pd4->data_id!=RDI_PD4_DATA_ID)
        return -1;
    else
        return 0;
}

int
rdi_parse_pd5 (const char *buf, int len, rdi_pd5_t *pd5)
{
    if (0!=rdi_verify_checksum (buf, len) || len!=RDI_PD5_LEN) 
    {
        return -1;
    }

    memcpy (pd5, buf, sizeof (rdi_pd5_t));
    if (pd5->header_id!=RDI_PD45_HEADER || pd5->data_id!=RDI_PD5_DATA_ID)
        return -1;
    else
        return 0;
}

int
rdi_parse_pd0(const char *buf, int len, rdi_pd0_t *pd0)
{
    if (rdi_verify_checksum (buf, len) != 0)
        return -1;
    
    char *ptr = buf;
    
    
    // copy the header info
    memcpy(&pd0->header, ptr, sizeof(rdi_pd0_header_t));
    ptr += 6;
    
    // check the length
    if((pd0->header.nbytes + 2) != len)
        return 0;
    
    
    // copy offset information
    pd0->header.offsets = (unsigned short *)malloc(pd0->header.num_data_types * sizeof(unsigned short));
    memcpy(pd0->header.offsets, ptr, pd0->header.num_data_types * sizeof(unsigned short));
    ptr += pd0->header.num_data_types * sizeof(unsigned short);
    
    // copy the fixed leader
    memcpy(&pd0->fixed, ptr, sizeof(rdi_pd0_fixed_leader_t));
    ptr += sizeof(rdi_pd0_fixed_leader_t);
    if(pd0->fixed.leader_id != 0x0000)
        return 0;
        
    
    // copy the variable leader
    memcpy(&pd0->variable, ptr, sizeof(rdi_pd0_variable_leader_t));
    ptr += sizeof(rdi_pd0_variable_leader_t);
    if(pd0->variable.leader_id != 0x8000)
        return 0;
    
    
    // copy the velocity data
    memcpy(&pd0->velocity.id, ptr, sizeof(unsigned short));
    ptr += sizeof(unsigned short);
    pd0->velocity.vel = (int16_t *)malloc(pd0->fixed.num_cells * 4 * sizeof(int16_t));
    memcpy(pd0->velocity.vel, ptr, pd0->fixed.num_cells * sizeof(int16_t) * 4);
    if(pd0->velocity.id != 0x0100)
        return 0;
        
    
    return 1;
}

void free_rdi_pd0(rdi_pd0_t *pd0)
{
    free(pd0->velocity.vel);
    free(pd0->header.offsets);
}
    
    
senlcm_rdi_pd0_t
rdi_pd0_to_lcm_pd0 (const rdi_pd0_t *pd0)
{
    // we are keeping most of the data just in case its required in the future
    senlcm_rdi_pd0_t lcm_pd0 = {0};
    
    // fixed leader
    memcpy(&lcm_pd0.cpu_fw_ver, &pd0->fixed.cpu_fw_ver, 5);
    memcpy(&lcm_pd0.num_beams, &pd0->fixed.num_beams, 4);
    lcm_pd0.depth_cell_length = pd0->fixed.depth_cell_length * 0.01;
    lcm_pd0.blank_after_xmit = pd0->fixed.blank_after_xmit * 0.01;
    memcpy(&lcm_pd0.profiling_mode, &pd0->fixed.profiling_mode, 4);
    lcm_pd0.err_vel_max = pd0->fixed.err_vel_max * 0.001;
    memcpy(&lcm_pd0.tpp_mins, &pd0->fixed.tpp_mins, 4);
    lcm_pd0.heading_align = pd0->fixed.heading_align * 0.01;
    lcm_pd0.heading_bias = pd0->fixed.heading_bias * 0.01;
    memcpy(&lcm_pd0.sensor_source, &pd0->fixed.sensor_source, 2);
    lcm_pd0.bin1_dist = pd0->fixed.bin1_dist * 0.01;
    lcm_pd0.xmit_pulse_len = pd0->fixed.xmit_pulse_len * 0.01;
    memcpy(&lcm_pd0.wp_ref_avg, &pd0->fixed.wp_ref_avg, 3);
    lcm_pd0.xmit_lag_dist = pd0->fixed.xmit_lag_dist * 0.01;
    memcpy(&lcm_pd0.cpu_serial, &pd0->fixed.cpu_serial, 4);
    
    // variable leader
    memcpy(&lcm_pd0.ens_num, &pd0->variable.ens_num, 14);
    lcm_pd0.depth = pd0->variable.depth * 0.1;
    lcm_pd0.heading = pd0->variable.heading * 0.01;
    lcm_pd0.pitch = pd0->variable.pitch * 0.01;
    lcm_pd0.roll = pd0->variable.roll * 0.01;
    memcpy(&lcm_pd0.salinity, &pd0->variable.salinity, 2);
    lcm_pd0.temp = pd0->variable.temp * 0.01;
    memcpy(&lcm_pd0.mpt_min, &pd0->variable.mpt_min, 4);
    lcm_pd0.pitch_std = pd0->variable.pitch_std * 0.1;
    lcm_pd0.roll_std = pd0->variable.roll_std * 0.1;
    memcpy(&lcm_pd0.adc[0], &pd0->variable.adc[0], 12);
    lcm_pd0.pressure = pd0->variable.pressure * 10;
    lcm_pd0.press_var = pd0->variable.press_var * 10;
    
    lcm_pd0.num_velocities = 4 * pd0->fixed.num_cells;
    lcm_pd0.velocity = malloc(lcm_pd0.num_velocities * sizeof(double));
    for(int i=0; i<lcm_pd0.num_velocities; i++)
        lcm_pd0.velocity[i] = pd0->velocity.vel[i] * 0.001;
    
    return lcm_pd0;
}

    

senlcm_rdi_pd4_t
rdi_pd4_to_lcm_pd4 (const rdi_pd4_t *pd4)
{
    senlcm_rdi_pd4_t lcm_pd4 = {0};

    lcm_pd4.system_config = pd4->system_config;

    int nbeams = 0;
    double altsum = 0.0;
    for (int i=0; i<4; i++) {
        lcm_pd4.btv[i] = pd4->btv[i] * UNITS_MILLI_TO_ONE; // mm/s to m/s
        lcm_pd4.wtv[i] = pd4->wtv[i] * UNITS_MILLI_TO_ONE; // mm/s to m/s

        if (pd4->altitude[i] > 0) {
            altsum += pd4->altitude[i] * UNITS_CENTI_TO_ONE; // cm to m
            nbeams++;
            const double cos30 = 0.866025403784439; // cos(30*DTOR)
            lcm_pd4.range[i] = pd4->altitude[i] * UNITS_CENTI_TO_ONE / cos30; // cm to m
        }
        else
            lcm_pd4.range[i] = SENLCM_RDI_PD4_T_RANGE_SENTINAL;
    }
    if (nbeams > 0)
        lcm_pd4.altitude = altsum/nbeams; // avg altitude [m]
    else
        lcm_pd4.altitude = SENLCM_RDI_PD4_T_ALTITUDE_SENTINAL;

    lcm_pd4.btv_status      = pd4->btv_status;
    lcm_pd4.wtv_status      = pd4->wtv_status;
    lcm_pd4.wtv_layer_start = pd4->wtv_layer_start * UNITS_DECI_TO_ONE; // dm to m
    lcm_pd4.wtv_layer_end   = pd4->wtv_layer_end * UNITS_DECI_TO_ONE;   // dm to m

    lcm_pd4.tofp_hour      = pd4->tofp_hour;
    lcm_pd4.tofp_minute    = pd4->tofp_minute;
    lcm_pd4.tofp_second    = pd4->tofp_second;
    lcm_pd4.tofp_hundredth = pd4->tofp_hundredth;

    memcpy (&lcm_pd4.builtin_test, &pd4->builtin_test, sizeof (pd4->builtin_test));

    lcm_pd4.speed_of_sound   = pd4->speed_of_sound;       // m/s
    lcm_pd4.xducer_head_temp = pd4->xducer_head_temp * UNITS_CENTI_TO_ONE; // cC to C

    return lcm_pd4;
}

senlcm_rdi_pd5_t
rdi_pd5_to_lcm_pd5 (const rdi_pd5_t *pd5)
{
    senlcm_rdi_pd5_t lcm_pd5 = {0};

    lcm_pd5.pd4 = rdi_pd4_to_lcm_pd4 ( (const rdi_pd4_t *) pd5);

    lcm_pd5.salinity = pd5->salinity; // ppt
    lcm_pd5.depth    = pd5->depth * UNITS_DECI_TO_ONE; // dm to m
    
    lcm_pd5.pitch   = pd5->pitch * UNITS_CENTI_TO_ONE * DTOR;   // .01 deg to rad
    lcm_pd5.roll    = pd5->roll * UNITS_CENTI_TO_ONE * DTOR;    // .01 deg to rad
    lcm_pd5.heading = pd5->heading * UNITS_CENTI_TO_ONE * DTOR; // .01 deg to rad

    for (int i=0; i<4; i++) {
        lcm_pd5.dmg_btv[i] = pd5->dmg_btv[i] * UNITS_DECI_TO_ONE; // dm to m
        lcm_pd5.dmg_wtv[i] = pd5->dmg_wtv[i] * UNITS_DECI_TO_ONE; // dm to m
    }

    return lcm_pd5;
}


senlcm_rdi_bathy_t
rdi_bathy (double r1, double r2, double r3, double r4, const double x_ws[6])
{
    senlcm_rdi_bathy_t bathy = {0};

    static double c = 100.0, s = 100.0;
    if (c > 1.0) {
        s = sin (30.0*DTOR);
        c = cos (30.0*DTOR);
    }

    /* dvl sensor frame:
     * +X-axis beam 1 to 2
     * +Y-axis beam 4 to 3
     * +Z-axis in toward sensor
     */

    // b1_hat = [-s, 0, -c]
    bathy.xyz[0][0] = -r1*s; // x
    bathy.xyz[0][1] =  0.0;  // y
    bathy.xyz[0][2] = -r1*c; // z

    // b2_hat = [s, 0, -c]
    bathy.xyz[1][0] =  r2*s;
    bathy.xyz[1][1] =  0.0;
    bathy.xyz[1][2] = -r2*c;

    // b3_hat = [0, s, -c]
    bathy.xyz[2][0] =  0.0;
    bathy.xyz[2][1] =  r3*s;
    bathy.xyz[2][2] = -r3*c;
    
    // b4_hat = [0, -s, -c]
    bathy.xyz[3][0] =  0.0;
    bathy.xyz[3][1] = -r4*s;
    bathy.xyz[3][2] = -r4*c;

    // ranges
    bathy.range[0] = r1;
    bathy.range[1] = r2;
    bathy.range[2] = r3;
    bathy.range[3] = r4;

    // transform to desired reference frame
    if (x_ws) {
        GSLU_MATRIX_VIEW (H_ws, 4, 4);
        ssc_homo4x4 (H_ws.data, x_ws);

        for (size_t i=0; i<4; i++) {
            GSLU_VECTOR_VIEW (X_s_h, 4, {bathy.xyz[i][0], bathy.xyz[i][1], bathy.xyz[i][2], 1.0});
            GSLU_VECTOR_VIEW (X_w_h, 4);
            gslu_blas_mv (&X_w_h.vector, &H_ws.matrix, &X_s_h.vector);
        
            bathy.xyz[i][0] = X_w_h.data[0];
            bathy.xyz[i][1] = X_w_h.data[1];
            bathy.xyz[i][2] = X_w_h.data[2];
        }
    }

    return bathy;
}
