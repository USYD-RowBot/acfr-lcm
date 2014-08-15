#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <libgen.h>
#include <ctype.h>
#include <lcm/lcm.h>

#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/textread.h"
#include "perls-common/unix.h"

#include "bot_core.h"
#include "hauv.h"
#include "perllcm.h"
#include "senlcm.h"
#include "se.h"
#include "acfrlcm.h"
#include "lcmlog_export.h"

static void
add_subscriptions (lcm_t *lcm, lcmlog_export_t *lle)
{
    // senlcm channels
    senlcm_acomms_range_t_subscribe (lcm, "^.*ACOMMS_RANGE$", &senlcm_acomms_range_t_handler, lle);
    senlcm_dstar_ssp1_t_subscribe (lcm, "^.*DESERT_STAR$", &senlcm_dstar_ssp1_t_handler, lle);
    senlcm_gpsd3_t_subscribe (lcm, "^.*GPSD_CLIENT", &senlcm_gpsd3_t_handler, lle);
//    senlcm_gpsd3_t_subscribe (lcm, "^.*GPSD3$", &senlcm_gpsd3_t_handler, lle);
//    senlcm_gpsd_t_subscribe (lcm, "^.*GPSD$", &senlcm_gpsd_t_handler, lle);
    senlcm_kvh_dsp3000_t_subscribe (lcm, "^.*KVH$", &senlcm_kvh_dsp3000_t_handler, lle);
    senlcm_mocap_t_subscribe (lcm, "^.*MOCAP_POSE_ARDRONE$", &senlcm_mocap_t_handler, lle);
    senlcm_mocap_t_subscribe (lcm, "^.*MOCAP_POSE_TARGET$", &senlcm_mocap_t_handler, lle);
    senlcm_ms_gx1_t_subscribe (lcm, "^.*MICROSTRAIN$", &senlcm_ms_gx1_t_handler, lle);
    senlcm_ms_gx3_t_subscribe (lcm, "^.*MICROSTRAIN$", &senlcm_ms_gx3_t_handler, lle);
    senlcm_ms_gx3_25_t_subscribe (lcm, "^.*MICROSTRAIN$", &senlcm_ms_gx3_25_t_handler, lle);
    senlcm_ms_gx3_25_t_subscribe (lcm, "^.*MICROSTRAIN_25$", &senlcm_ms_gx3_25_t_handler, lle);
    senlcm_os_compass_t_subscribe (lcm, "^.*OS_COMPASS$", &senlcm_os_compass_t_handler, lle);
    senlcm_ppsboard_t_subscribe (lcm, "^.*PPSBOARD$", &senlcm_ppsboard_t_handler, lle);
    senlcm_prosilica_t_subscribe (lcm, "^.*PROSILICA.*ATTRIBUTES$", &senlcm_prosilica_t_handler, lle);
//    senlcm_rdi_pd4_t_subscribe (lcm, "^.*RDI$", &senlcm_rdi_pd4_t_handler, lle);
    senlcm_rdi_pd5_t_subscribe (lcm, "^.*RDI$", &senlcm_rdi_pd5_t_handler, lle);
    senlcm_tritech_es_t_subscribe (lcm, "^.*TRITECH_ES$", &senlcm_tritech_es_t_handler, lle);
    senlcm_uvc_osi_t_subscribe (lcm, "^.*OS_CONDUIT_OSI$", &senlcm_uvc_osi_t_handler, lle);
    // senlcm added by ACFR
    senlcm_ecopuck_t_subscribe (lcm, "ECOPUCK", &senlcm_ecopuck_t_handler, lle);
	senlcm_oas_t_subscribe (lcm, "OAS", &senlcm_oas_t_handler, lle);
	senlcm_parosci_t_subscribe (lcm, "PAROSCI", &senlcm_parosci_t_handler, lle);
	senlcm_lq_modem_t_subscribe (lcm, "LQ_MODEM", &senlcm_lq_modem_t_handler, lle);
	senlcm_ysi_t_subscribe (lcm, "YSI", &senlcm_ysi_t_handler, lle);
    senlcm_rdi_pd0_t_subscribe (lcm, "RDI_PD0", &senlcm_rdi_pd0_t_handler, lle);
    senlcm_os_power_system_t_subscribe (lcm, "BATTERY", &senlcm_os_power_system_t_handler, lle);
	senlcm_micron_ping_t_subscribe (lcm, "MICRON", &senlcm_micron_ping_t_handler, lle);
	senlcm_tcm_t_subscribe (lcm, "TCM", &senlcm_tcm_t_handler, lle);
	senlcm_kvh1750_t_subscribe (lcm, "KVH1750", &senlcm_kvh1750_t_handler, lle);
    senlcm_usb2000_spec_t_subscribe (lcm, "SPEC_DOWN", &senlcm_usb2000_spec_t_handler, lle);
    senlcm_sts_spec_t_subscribe (lcm, "SPEC_UP", &senlcm_sts_spec_t_handler, lle);

    // bot_core channels
    bot_core_image_sync_t_subscribe (lcm, "^.*PROSILICA.*SYNC$", &bot_core_image_sync_t_handler, lle);

    // perllcm channels
    perllcm_auv_navigator_t_subscribe (lcm, "^.*AUV_NAVIGATOR$", &perllcm_auv_navigator_t_handler, lle);
    perllcm_segway_navigator_t_subscribe (lcm, "^.*SEG_NAVIGATOR$", &perllcm_segway_navigator_t_handler, lle);
    perllcm_segway_navigator_t_subscribe (lcm, "^.*SEG_NAVIGATOR_DROP_DS_ACK$", &perllcm_segway_navigator_t_handler, lle);
    perllcm_est_navigator_debug_meas_t_subscribe (lcm, "^.*NAVIGATOR_DEBUG_MEAS$", &perllcm_est_navigator_debug_meas_t_handler, lle);
    perllcm_est_navigator_debug_pred_t_subscribe (lcm, "^.*NAVIGATOR_DEBUG_PRED$", &perllcm_est_navigator_debug_pred_t_handler, lle);
    perllcm_segway_state_t_subscribe (lcm, "^.*SEGWAY_STATE$", &perllcm_segway_state_t_handler, lle);
    perllcm_van_vlink_t_subscribe (lcm, "^VAN_VLINKS$", &perllcm_van_vlink_t_handler, lle);
    perllcm_ardrone_state_t_subscribe (lcm, "^.*ARDRONE_STATE$",&perllcm_ardrone_state_t_handler, lle);
    perllcm_ardrone_drive_t_subscribe (lcm, "^.*ARDRONE_DRIVE$",&perllcm_ardrone_drive_t_handler, lle);
    perllcm_position_t_subscribe (lcm, "^.*ARDRONE_NAV$",&perllcm_position_t_handler, lle);
    perllcm_position_t_subscribe (lcm, "^.*TARGET_NAV$",&perllcm_position_t_handler, lle);
    perllcm_van_plink_t_subscribe (lcm, "^VAN_PLINKS$", &perllcm_van_plink_t_handler, lle);

    // hauv channels
    hauv_bs_cnv_t_subscribe (lcm, "^HAUV_BS_CNV$", &hauv_bs_cnv_t_handler, lle);
    hauv_bs_dvl_t_subscribe (lcm, "^HAUV_BS_DVL$", &hauv_bs_dvl_t_handler, lle);
    hauv_bs_dvl_2_t_subscribe (lcm, "^HAUV_BS_DVL_2$", &hauv_bs_dvl_2_t_handler, lle);
    hauv_bs_imu_t_subscribe (lcm, "^HAUV_BS_IMU$", &hauv_bs_imu_t_handler, lle);
    hauv_bs_nvg_t_subscribe (lcm, "^HAUV_BS_NVG$", &hauv_bs_nvg_t_handler, lle);
    hauv_bs_nvr_t_subscribe (lcm, "^HAUV_BS_NVR$", &hauv_bs_nvr_t_handler, lle);
    hauv_bs_pit_t_subscribe (lcm, "^HAUV_BS_PIT$", &hauv_bs_pit_t_handler, lle);
    hauv_bs_raw_t_subscribe (lcm, "^HAUV_BS_RAW$", &hauv_bs_raw_t_handler, lle);
    hauv_bs_rbs_t_subscribe (lcm, "^HAUV_BS_RBS$", &hauv_bs_rbs_t_handler, lle);
    hauv_bs_rcm_t_subscribe (lcm, "^HAUV_BS_RCM$", &hauv_bs_rcm_t_handler, lle);
    hauv_bs_rdp_t_subscribe (lcm, "^HAUV_BS_RDP$", &hauv_bs_rdp_t_handler, lle);
    hauv_bs_rnv_t_subscribe (lcm, "^HAUV_BS_RNV$", &hauv_bs_rnv_t_handler, lle);
    hauv_bs_rnv_2_t_subscribe (lcm, "^HAUV_BS_RNV_2$", &hauv_bs_rnv_2_t_handler, lle);
    hauv_vehicle_state_t_subscribe (lcm, "^HAUV_VECHICLE_STATE$", &hauv_vehicle_state_t_handler, lle);

    // seserver
    se_publish_link_t_subscribe (lcm, "^SE_PUBLISH_LINK_CLIENT$", &se_publish_link_t_handler, lle);
    se_return_state_t_subscribe (lcm, "^SE_RETURN_STATE_SERVER$", &se_return_state_t_handler, lle);
    se_goto_t_subscribe         (lcm, "^SE_GOTO$",                &se_goto_t_handler, lle);

    // acfr channels
    acfrlcm_auv_acfr_nav_t_subscribe(lcm, "ACFR_NAV", &acfrlcm_auv_acfr_nav_t_handler, lle);
    acfrlcm_auv_control_t_subscribe(lcm, "AUV_CONTROL", &acfrlcm_auv_control_t_handler, lle);
    acfrlcm_auv_path_command_t_subscribe(lcm, "PATH_COMMAND", &acfrlcm_auv_path_command_t_handler, lle);
    acfrlcm_auv_path_response_t_subscribe(lcm, "PATH_RESPONSE", &acfrlcm_auv_path_response_t_handler, lle);
    acfrlcm_auv_iver_motor_command_t_subscribe(lcm, "IVER_MOTOR", &acfrlcm_auv_iver_motor_command_t_handler, lle);
    acfrlcm_auv_global_planner_t_subscribe(lcm, "TASK_PLANNER_COMMAND", &acfrlcm_auv_global_planner_t_handler, lle);
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // add options
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "Export a lcm log file to .csv format for a predefined subset of lcmtypes");
    getopt_add_help    (gopt, NULL);
    getopt_add_bool    (gopt, 'm', "matlab", 0,        "Autogenerate a matlab loader script");
    getopt_add_string  (gopt, 'd', "dir",    "./",     "Output directory for .csv's");
    getopt_add_string  (gopt, 'F', "fname",  "",       "Output filename for .csv's (default lcmlog filename)");
    getopt_add_string  (gopt, 's', "strip",  "",       "| separated list of channel name prefixes to strip");
    getopt_add_string  (gopt, 'r', "delim_replace",  "",       "string containing delimiters to replace with underscores");
    getopt_add_example (gopt, 
                        "Strip IVER28_, IVER31_ and TOPSIDE_ channel prefixes when exporting a LCM log file to Matlab\n"
                        "%s -m --strip \"IVER28_|IVER31_|TOPSIDE_\" lcmlog", argv[0]);
                        

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=1) {
        getopt_do_usage (gopt, "LCMLOG");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, "LCMLOG");
        exit (EXIT_SUCCESS);
    }

    // process options
    const char *csvlog_dir = getopt_get_string (gopt, "dir");
    unix_mkpath (csvlog_dir, 0775);

    char *lcmlog_fullname = g_ptr_array_index (gopt->extraargs, 0);
    
    const char *lcmlog_dir = dirname (strdup (lcmlog_fullname));
    const char *lcmlog_fname = basename (strdup (lcmlog_fullname));

    const char *csvlog_prefix = NULL;
    if (getopt_has_flag (gopt, "fname"))
        csvlog_prefix = getopt_get_string (gopt, "fname");
    else
        csvlog_prefix = lcmlog_fname;

    const char *channel_strip = NULL;
    if (getopt_has_flag (gopt, "strip"))
        channel_strip = getopt_get_string (gopt, "strip");

    const char *delim_string = NULL;
    if (getopt_has_flag (gopt, "delim_replace"))
        delim_string = getopt_get_string (gopt, "delim_replace");

    lcmlog_export_t *lle = lle_create (csvlog_dir, csvlog_prefix,
        channel_strip, delim_string);


    // setup LCM log file for playback
    char lcm_url[PATH_MAX];
    snprintf (lcm_url, sizeof lcm_url, "file://%s/%s?speed=0", lcmlog_dir, lcmlog_fname);
    lcm_t *lcm = lcm_create (lcm_url);
    if (!lcm) {
        ERROR ("lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }
    add_subscriptions (lcm, lle);

    // export LCM events to CSV
    int64_t events = 0;
    while (0==lcm_handle (lcm)) {
        if ((++events % 200000)==0)
            printf (".");
    };
    printf ("\n");

    // generate matlab load script?
    if (getopt_get_bool (gopt, "matlab")) {
        char mfile_fname[NAME_MAX];
        sprintf (mfile_fname, "%s", lcmlog_fname);
        for (int i=0; i<strlen (mfile_fname); i++) {
            if (!isalnum (mfile_fname[i]))
                mfile_fname[i] = '_';
        }
        textread_t **tra = lle_get_textread_array (lle);
        textread_gen_matlab (tra, lle_get_length (lle), lle_get_csvdir (lle), mfile_fname);
    }


    // clean up
    lle_destroy (lle);
    lcm_destroy (lcm);

    exit (EXIT_SUCCESS);
}
