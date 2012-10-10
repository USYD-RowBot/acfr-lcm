#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <sys/types.h>

// external linking req'd
#include <bot_core/bot_core.h>

#include "perls-common/bot_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/unix.h"

#include "dir_mgmt.h"

//----------------------------------------------------------------------
// Loads the required info from the .cfg file into the state
//----------------------------------------------------------------------
void
dir_mgmt_load_cfg (BotParam *param, dir_mgmt_t *dm)
{
    dm->base_dir = botu_param_get_str_or_default (param, 
                                                  "os-remotehelm.dir-mgmt.base_dir", 
                                                  "/home/auv/mission-data");

    dm->perls_dir = botu_param_get_str_or_default (param,
                                                   "os-remotehelm.dir-mgmt.perls_dir", 
                                                   "/home/auv/perls");

    dm->wafer_mis_dir = botu_param_get_str_or_default (param, 
                                                       "os-remotehelm.dir-mgmt.wafer_mis_dir",
                                                       "C:\\Missions");

    dm->mounted_wafer = botu_param_get_str_or_default (param,
                                                       "os-remotehelm.dir-mgmt.mounted_wafer",
                                                       "/home/auv/wafer-missions");

    dm->auv_name = botu_param_get_str_or_default (param,
                                                  "vehicle.name",
                                                  "noname");
}


//----------------------------------------------------------------------
// Find the next dive number based off of the local directory structure
// make a new dive directory, set dir_mgmt dm vars 
//----------------------------------------------------------------------
void 
dir_mgmt_next_dive (dir_mgmt_t * dm)
{
    char *tmp = g_strconcat (dm->base_dir, "/" ,dm->auv_name, NULL);
    DIR *pdir =  opendir (tmp);
    free(tmp);
    struct dirent *pent;

    if (!pdir) {
        printf ("opendir() failure: Unable to open base_dir: %s \r\n",
                dm->base_dir);
        exit (EXIT_FAILURE);
    }

    errno=0;
    int max = 0; //start at 1 if we dont find anything we start at dive001
    while ((pent=readdir (pdir))) {
        int tmp=0;
        //format yyyy-mm-dd-dive.ddd (just scan until the '.' character)
        if (1==sscanf (pent->d_name, "%*[^.].%d", &tmp))
            max = (tmp > max ? tmp : max);
    }
    if (errno) {
        printf ("readdir() failure: Unable to read base_dir: %s  \r\n",
                dm->base_dir);
        exit (EXIT_FAILURE);
    }
    closedir (pdir);

    dm->dive_num = max+1;

    char time_str[14];
    timeutil_strftime (time_str, sizeof time_str, "%Y-%m-%d", timestamp_now ());
    
    dm->dive_dir = malloc (PATH_MAX * sizeof (char));
    snprintf (dm->dive_dir, PATH_MAX * sizeof (char), "%s/%s/%s-dive.%03d",
              dm->base_dir, dm->auv_name, time_str, dm->dive_num );
    printf ("dive dir %s\n",dm->dive_dir);

    if (unix_mkpath (dm->dive_dir, 0755)) {
        printf ("unix_mkpath() failure: Failed to make dive dir: %s  \r\n",
                dm->dive_dir);
        exit (EXIT_FAILURE);
    }
}

//----------------------------------------------------------------------
// Copy the mission file into the dive directory
//----------------------------------------------------------------------
void 
dir_mgmt_cpy_mis (dir_mgmt_t *dm, const char *mission_file)
{
    char cmd[512];
    snprintf (cmd, sizeof cmd, "cp -p %s %s/.", 
              mission_file, dm->dive_dir);
    if (system (cmd))
        printf ("Error in cp cmd: %s\n\r", cmd);
}

//----------------------------------------------------------------------
// Copy lcm_defs and config to mission dir
//----------------------------------------------------------------------
void 
dir_mgmt_cpy_config (dir_mgmt_t *dm)
{
    char cmd[1024];
    snprintf (cmd, sizeof cmd, "rsync -ak --exclude=\".svn\" %s/lcmdefs %s/.",
              dm->perls_dir, dm->dive_dir);
    if (system (cmd))
        printf ("Error in cp cmd: %s\n\r", cmd);

    snprintf (cmd, sizeof cmd, "rsync -L %s/config/master.cfg %s/.",
              dm->perls_dir, dm->dive_dir);
    if (system (cmd))
        printf ("Error in cp cmd: %s\n\r", cmd);

    // TODO copy config includes
}


//----------------------------------------------------------------------
// Move UVC log to mission dir
//----------------------------------------------------------------------
void 
dir_mgmt_mv_uvc_log (dir_mgmt_t *dm, const char *mission_file)
{
    char cmd[512];
    char mission_name[128];
    char base_name[128];
    int len = 0;

    len = (strrchr (mission_file,'/') - mission_file );
    strncpy (base_name, mission_file, len);
    base_name[len] = '\0';
    len = (strrchr (mission_file,'.')-1 - strrchr (mission_file,'/'));
    strncpy (mission_name,strrchr (mission_file,'/')+1, len);
    mission_name[len] = '\0';
    //printf ("%s/%s.log\n\r", base_name, mission_name);

    //snprintf (cmd, sizeof cmd, "cp -p %s/%s.log %s/%s.log",
    //         base_name, mission_name, dm->dive_dir, mission_name);
    snprintf (cmd, sizeof cmd, "mv %s/%s.log %s/%s.log",
              base_name, mission_name, dm->dive_dir, mission_name);
    if (system (cmd))
        printf ("Error in mv cmd: %s\n\r", cmd);
}
