#ifndef __RTVAN_UTIL_H__
#define __RTVAN_UTIL_H__

#include <glib.h>

#include "perls-common/cache.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_van_corrset_t.h"
#include "perls-lcmtypes/perllcm_van_rdi_bathy_collection_t.h"
#include "perls-lcmtypes/se_save_isam_t.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CALIB_UW_WATER   0
#define CALIB_UW_AIR     1
#define CALIB_PERI_WATER 2
#define CALIB_PERI_AIR   3

typedef struct _vanu_glist_t vanu_glist_t;
struct _vanu_glist_t
{
    GSList *list;
    GStaticMutex mutex;
};

// link data for post analysis
typedef struct link_info link_info_t;
struct link_info
{
    double utime_i;
    double utime_j;
    double S_L;
    double Ig;
    bool   success;
};

/*
 * Load isam graph file, append with camoffset + sensor xform, and save (overwrite)
 */
void
vanu_isam_save_rtvan_appended (const char *filepath, GHashTable *camoffset_list, 
                               GHashTable *x_vc, GHashTable *S_ii);

void
vanu_isam_save_rtvan_appended_via_lcm (FILE *fid, se_save_isam_t *isam_line, 
                                       GHashTable *camoffset_list, GHashTable *x_vc, GHashTable *S_ii);

GSList *
vanu_load_isam_graph (const char *filename, GSList *utimelist, GHashTable *camoffset_list,
                      GHashTable *x_vc_hash, GHashTable *S_ii_hash, size_t *n_nodes);

/*  write {z, R, uv_i, uv_j} into disk for photo mosaic
 *  data will be written to "van-processed/corrset/utime1-utime2"
 */
void
vanu_corrset_save2disk (int64_t utime1, int64_t utime2, const char *logdir,
                        gsl_vector *z, gsl_matrix *R, gsl_matrix *uv1, gsl_matrix *uv2);

perllcm_van_corrset_t*
vanu_prepare_corrset (int64_t utime1, int64_t utime2, gsl_vector *z, gsl_matrix *R, 
                      gsl_matrix *uv1, gsl_matrix *uv2);

void
vanu_dvlpts_save2disk (const char *filepath, GSList *dvlpts_glist, 
                       perllcm_van_calib_t *calibUw_water, perllcm_van_calib_t *calibUw_air,
                       perllcm_van_calib_t *calibPeri_water, perllcm_van_calib_t *calibPeri_air);

int
vanu_load_manual_plink (const char *filename, GHashTable *plinks);

int
vanu_load_manual_plink_wo_prior (const char *filename, GHashTable *plinks);

int
vanu_saliency_save2disk (const char *filename, GSList *utimelist, GHashTable *S_L_list, GHashTable *sumidf_list, 
                         GHashTable *vocablen_list, double sumidf_max);

int
vanu_link_info_save2disk (const char *logdir, GSList *linklist);

#ifdef __cplusplus
}
#endif

#endif //__RTVAN_UTIL_H__
