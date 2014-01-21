#ifndef __DIR_MGMT_H__
#define __DIR_MGMT_H__

#include <bot_core/bot_core.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dir_mgmt dir_mgmt_t;
struct dir_mgmt 
{   
    int dive_num;
    char *base_dir;      //base directory for dive data, in config file, changes per trip
    char *dive_dir;      //abs path to dive dir, use for all coppying in and out (on pc104)
    char *perls_dir;     //abs path to perls dir
    char *wafer_mis_dir; //abs path to wafer's mission dir
    char *mounted_wafer;
    char *auv_name;      //name of the auv
};


void
dir_mgmt_load_cfg (BotParam *param, dir_mgmt_t *dm);

void
dir_mgmt_next_dive (dir_mgmt_t *dm);

void
dir_mgmt_cpy_mis (dir_mgmt_t *dm, const char *mission_file);

void
dir_mgmt_cpy_config (dir_mgmt_t *dm);

void
dir_mgmt_mv_uvc_log (dir_mgmt_t *dm, const char *mission_file); 


#ifdef __cplusplus
}
#endif

#endif //__DIR_MGMT_H__
