#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h> // needed for PRId64 macros

// external linking req'd
#include <glib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>

#include "perls-common/cache.h"
#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"
#include "perls-common/bot_util.h"

#include "perls-vision/feature.h"
#include "perls-vision/modelfit.h"
#include "perls-vision/pccs.h"
#include "perls-vision/sba.h"
#include "perls-vision/sba_haralick_cov.h"
#include "perls-vision/sba_ut_cov.h"
#include "perls-vision/twoview.h"
#include "perls-vision/plot.h"
#include "perls-vision/distortion.h"

#include "perls-math/gsl_util.h"
#include "perls-math/dm.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_van_options_t.h"
#include "perls-lcmtypes/perllcm_van_plink_t.h"
#include "perls-lcmtypes/perllcm_van_plot_debug_t.h"
#include "perls-lcmtypes/perllcm_van_verify_ack_t.h"
#include "perls-lcmtypes/perllcm_van_vlink_t.h"
#include "perls-lcmtypes/perllcm_isam_vlink_t.h"

#include "perls-lcmtypes/se_save_isam_t.h"
#include "perls-lcmtypes/se_publish_link_t.h"

#include "shared_memory.h"
#include "van_util.h"
#include "twoview_thread.h"

#define printf(format, ...)                                     \
    printf ("%-12s " format, "[twoview]", ## __VA_ARGS__)

#define POOL_THREADS_MAX     10
#define FC_CACHE_MAX         100
#define TV_SBA_ERROR         -1

#define RTOD (UNITS_RADIAN_TO_DEGREE)
#define DTOR (UNITS_DEGREE_TO_RADIAN)

#define USE_TRIANGULATION    1      // in case of 3D, do trianulate and reject points
#define USE_PROJ_NONLIN      0      // use nonlinear projection for *NEGLIGIBLE* increase in camera covariance
#define USER_VERIFY          0      // periscope mode testing

typedef struct verify_set verify_set_t;
struct verify_set
{
    int64_t utime1;
    int64_t utime2;
    se_publish_link_t *se_vlink;
    perllcm_van_plot_debug_t *pd;
};

typedef struct thread_data thread_data_t;
struct thread_data
{
    cache_t     *fc_cache; // feature collection cache

    GThreadPool *pool;     // thread pool

    perllcm_van_options_t vanopts;

    // twoview register configuration
    double pccs_sift_simAB_thres;
    double pccs_harris_simAB_thres;
    double pccs_chiSquare2dof;
    size_t tv_min_npts;
    double tv_tri_const_min_dist;
    double tv_tri_const_max_dist;
    double tv_mdist_nav_thres;
    float ocv_scale;
    GMutex *sba_mutex;

    bool use_se_vlink;
    double tv_pccs_fudge;

    // to verify
    GList   *verifyset_list;           // vlink+pd list
    bool    plot_thread_idle;

    // post-process
    bool    save_corrset;       // photo mosaic
    bool    save_link_info;     // bookkeeping
    GSList *linklist;
};

typedef struct pool_data pool_data_t;
struct pool_data
{
    perllcm_van_plink_t *plink;
    perllcm_van_feature_collection_t *fci;
    perllcm_van_feature_collection_t *fcj;
};

/* Placeholder for allocating calibration covariance matrix -- this should eventually be
 * passed into the thread pool because each camera calibration covariance would be
 * different...*/
static gsl_matrix *
sigma_calib_alloc ()
{
    double varFx, varFy, varCx, varCy, varK1, varK2, varP1, varP2, varK3;

    varFx = 40.23497930180816;
    varFy = 38.82274569377955;
    varCx = 9.259693532351305;
    varCy = 27.62531126594374;
    varK1 = 0.000032012333624;
    varK2 = 0.000877222378770;
    varP1 = 0.000000506618851;
    varP2 = 0.000000382538938;
    varK3 = 0.00002532289434731;

    gsl_matrix *sigma = gsl_matrix_calloc (9, 9);
    gsl_matrix_set (sigma, 0, 0, varFx);
    gsl_matrix_set (sigma, 1, 1, varFy);
    gsl_matrix_set (sigma, 2, 2, varCx);
    gsl_matrix_set (sigma, 3, 3, varCy);
    gsl_matrix_set (sigma, 4, 4, varK1);
    gsl_matrix_set (sigma, 5, 5, varK2);
    gsl_matrix_set (sigma, 6, 6, varP1);
    gsl_matrix_set (sigma, 7, 7, varP2);
    gsl_matrix_set (sigma, 8, 8, varK3);

    return sigma;
}

static void
pooldata_free (pool_data_t *pdata)
{
    perllcm_van_plink_t_destroy (pdata->plink);
    perllcm_van_feature_collection_t_destroy (pdata->fci);
    perllcm_van_feature_collection_t_destroy (pdata->fcj);
    free (pdata);
}

#if USER_VERIFY
// PERISCOPE MODE TESTING (verify set)
static void
printf_vslist (GList *list)
{
    size_t listlen = g_list_length (list);
    for (size_t i=0; i < listlen; i++)
    {
        GList *event = g_list_nth (list, i);
        verify_set_t *vs = event->data;
        printf ("vs.t1 = %"PRId64", t2 = %"PRId64"\n", vs->utime1, vs->utime2);
    }
}

static bool
request_verify (GList *verifyset_list)
{
    bool idle = false;

    GList *event = g_list_last (verifyset_list);

    verify_set_t *vs = event->data;
    //printf ("%"PRId64", %"PRId64"\n", utime1, utime2);

    if (vs->pd)
        perllcm_van_plot_debug_t_publish (shm->lcm, VAN_PLOT_DEBUG_CHANNEL, vs->pd);
    else
        idle = true;

    return idle;
}

static void
perllcm_van_verify_ack_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                   const perllcm_van_verify_ack_t *v_ack, void *user)
{
    thread_data_t *tdata = user;

    size_t listlen = g_list_length (tdata->verifyset_list);
    //printf ("length %zu\n", listlen);

    size_t idx = 0;
    tdata->plot_thread_idle = true;

    if (v_ack->stop_verifying || !shm->van_opts.manual_corr)
    {
        // publish ALL in the list
        for (size_t i=0; i < listlen; i++)
        {
            GList *event = g_list_nth (tdata->verifyset_list, idx);
            verify_set_t *vs = event->data;
            if (!vs) return;

            se_publish_link_t_publish (shm->lcm, SE_VLINK_CHANNEL, vs->se_vlink);

            se_publish_link_t_destroy (vs->se_vlink);
            perllcm_van_plot_debug_t_destroy (vs->pd);
            tdata->verifyset_list = g_list_delete_link (tdata->verifyset_list, event);
        }
    }
    else
    {
        for (size_t i=0; i < listlen; i++)
        {
            GList *event = g_list_nth (tdata->verifyset_list, idx);
            verify_set_t *vs = event->data;
            if (!vs) return;

            //printf ("%"PRId64", %"PRId64"\n", vs->utime1, vs->utime2);
            if (vs->utime1 == v_ack->utime1 && vs->utime2 == v_ack->utime2)
            {
                if (!v_ack->valid) vs->se_vlink->accept = 0;      // could not pass verification
                se_publish_link_t_publish (shm->lcm, SE_VLINK_CHANNEL, vs->se_vlink);

                se_publish_link_t_destroy (vs->se_vlink);
                perllcm_van_plot_debug_t_destroy (vs->pd);
                tdata->verifyset_list = g_list_delete_link (tdata->verifyset_list, event);
            }
            else
                idx++;
        }

        listlen = g_list_length (tdata->verifyset_list);
        if (listlen > 0) tdata->plot_thread_idle = request_verify (tdata->verifyset_list);
    }
}
#endif

static void
perllcm_van_options_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_van_options_t *msg, void *user)
{
    thread_data_t *tdata = user;
    tdata->vanopts = *msg;
    shm->van_opts = *msg;
}

static void
perllcm_van_feature_collection_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
        const perllcm_van_feature_collection_t *fc, void *user)
{
    thread_data_t *tdata = user;

    // insert feature collection into cache for quick lookup
    cache_push (tdata->fc_cache, fc->utime, perllcm_van_feature_collection_t_copy (fc));
}

static void
perllcm_van_plink_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                              const perllcm_van_plink_t *plink, void *user)
{
    thread_data_t *tdata = user;

    // fci
    perllcm_van_feature_collection_t *fci = cache_pop (tdata->fc_cache, plink->utime_i);
    if (!fci)
    {
        char filename[PATH_MAX];
        snprintf (filename, sizeof filename, "%s/%"PRId64".feat", shm->logdir, plink->utime_i);
        int32_t ret = LCMU_FREAD (filename, &fci, perllcm_van_feature_collection_t);
        if (ret < 0)
            ERROR ("couldn't read %s from disk!  ret=%d", filename, ret);
        else
            cache_push (tdata->fc_cache, plink->utime_i, fci);
    }

    // fcj
    perllcm_van_feature_collection_t *fcj = cache_pop (tdata->fc_cache, plink->utime_j);
    if (!fcj)
    {
        char filename[PATH_MAX];
        snprintf (filename, sizeof filename, "%s/%"PRId64".feat", shm->logdir, plink->utime_j);
        int32_t ret = LCMU_FREAD (filename, &fcj, perllcm_van_feature_collection_t);
        if (ret < 0)
            ERROR ("couldn't read %s from disk!  ret=%d", filename, ret);
        else
            cache_push (tdata->fc_cache, plink->utime_j, fcj);
    }

    // warn if number of concurrent pool threads is unable to keep up
    guint ntasks = g_thread_pool_unprocessed (tdata->pool);
    if (ntasks >= g_thread_pool_get_max_threads (tdata->pool))
    {
        printf ("Warning: number of unprocessed tasks (%u) exceeds queue (%u)\n",
                ntasks, g_thread_pool_get_max_threads (tdata->pool));
    }

    // fire off pool thread
    if (fci && fcj)
    {
        pool_data_t *pdata = malloc (sizeof (*pdata));
        pdata->plink = perllcm_van_plink_t_copy (plink);
        pdata->fci   = perllcm_van_feature_collection_t_copy (fci);
        pdata->fcj   = perllcm_van_feature_collection_t_copy (fcj);
        g_thread_pool_push (tdata->pool, pdata, NULL);
    }
}

static void
twoview_pool_thread (gpointer pooldata, gpointer user)
{
    pool_data_t *pdata = pooldata;
    thread_data_t *tdata __attribute__((unused)) = user;

    printf ("Processing (%"PRId64",%"PRId64") dt = %"PRId64"\n", pdata->plink->utime_i, pdata->plink->utime_j, pdata->plink->utime_j - pdata->plink->utime_i);

#if 0
    if (0)
    {
        // read image from stack/disk
        IplImage *imgi = vis_cvu_iplimg_pop (shm->imgcache, shm->logdir, pdata->fci->utime);
        IplImage *imgj = vis_cvu_iplimg_pop (shm->imgcache, shm->logdir, pdata->fcj->utime);

        // plot pair
        if (imgi && imgj)
        {
            CvScalar color = CV_RGB (255, 255, 0);
            vis_plot_add_utime (imgi, pdata->fci->utime, NULL, NULL, &color);
            vis_plot_add_utime (imgj, pdata->fcj->utime, NULL, NULL, &color);
            vis_plot_pair (imgi, imgj, "PLINK", 0.5, VIS_PLOT_STACK_HORZ);
            cvWaitKey(5);
        }
        else
            printf ("images are NULL\n");

        // clean up
        if (imgi) cvReleaseImage (&imgi);
        if (imgj) cvReleaseImage (&imgj);
    }

    if (0)
    {
        // publish fake registration
        gsl_vector_view X_c2c1 = gsl_vector_view_array (pdata->plink->x_ji.mu, 6);
        gsl_matrix_view P_c2c1 = gsl_matrix_view_array (pdata->plink->x_ji.Sigma, 6, 6);
        //sleep(2);
        GSLU_VECTOR_VIEW (nav_p21_5dof, 5);
        GSLU_MATRIX_VIEW (nav_cov21_5dof, 5, 5);
        dm_trans2dm_pose_cov (&X_c2c1.vector, &P_c2c1.matrix, &nav_p21_5dof.vector, &nav_cov21_5dof.matrix, NULL);

        if (tdata->use_se_vlink)
        {
            perllcm_isam_vlink_t *vlink = vis_twoview_isam_vlink (pdata->fci->utime, pdata->fcj->utime,
                                          pdata->plink,
                                          PERLLCM_VAN_VLINK_T_TYPE_5DOF_EPIPOLAR,
                                          PERLLCM_VAN_VLINK_T_MSG_NO_ERROR,
                                          pdata->plink->link_id,
                                          &nav_p21_5dof.vector, &nav_cov21_5dof.matrix);
            if (vlink)
            {
                perllcm_isam_vlink_t_publish (shm->lcm, "VLINKS", vlink);
                perllcm_isam_vlink_t_destroy(vlink);
            }
        }
    }

    pooldata_free (pdata);
    return;
#endif

    // init
    // -------------------------------------------------------------- //
    int64_t t0, dt;                                     // for timing stats
    int32_t errmsg = PERLLCM_VAN_VLINK_T_MSG_NO_ERROR;  // errmsg in vlink
    int32_t type = PERLLCM_VAN_VLINK_T_TYPE_ERROR;      // result type in vlink
    size_t n_pcorr_sum = 0;                             // number of corr
    int ret_sba = TV_SBA_ERROR;                         // BA return
    int verbose = 0;                                    // BA verbose level
    int model_gic = 0;                                  // selected model (0 for none)
    gsl_matrix_view Ki_view = gsl_matrix_view_array (pdata->fci->calib.K, 3, 3);
#if USE_PROJ_NONLIN
    gsl_vector_view distCoeffs_view = gsl_vector_view_array (pdata->fci->calib.kc, 5);
#endif
    //gsl_matrix_view Kj_view = gsl_matrix_view_array (pdata->fcj->calib.K, 3, 3);

    int pt3d_debug_mode = tdata->vanopts.vis_plot_3pts || tdata->vanopts.vis_plot_relpose_3pts;

    // to be populated if plot_debug is "on"
    gsl_matrix *uvic = NULL, *uvjc = NULL;              // uv collection
    gslu_index *selic = NULL, *seljc = NULL;            // inliers indeces collection
    gsl_matrix *uvi_f = NULL, *uvj_f = NULL;            // inliers from F
    gsl_matrix *uvi_h = NULL, *uvj_h = NULL;            // inliers from H
    gslu_index *sel_f = NULL, *sel_h = NULL;            // inliers indeces
    gsl_matrix *X1_plot = NULL;
    gsl_vector *rel_pose21 = gsl_vector_alloc (5);         // BA return pose
    gsl_matrix *rel_pose_cov21 = gsl_matrix_alloc (5,5);   // BA return cov

    perllcm_van_plot_debug_t *pd = vis_tv_init_plot_debug (tdata->vanopts, pdata->fci, pdata->fcj, &pdata->fci->calib);

    // PCCS
    // -------------------------------------------------------------- //
    size_t offset_i = 0, offset_j = 0;
    for (int n=0; n<pdata->fci->ntypes; n++)
    {
        perllcm_van_feature_t *fi = &pdata->fci->f[n];
        perllcm_van_feature_t *fj = &pdata->fcj->f[n];

        gsl_matrix *uvi = vis_feature_get_uv_alloc (fi);
        gsl_matrix *uvj = vis_feature_get_uv_alloc (fj);

        // return from pccs
        gslu_index *seli = NULL, *selj = NULL;

        // sim AB thres
        double simAB_thres = 1.0;

        switch (fi->attrtype)
        {
        case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SURFGPU:
        case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF:
            simAB_thres = tdata->pccs_sift_simAB_thres;
            break;
        case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS:
            simAB_thres = tdata->pccs_harris_simAB_thres;
            break;
        case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU:
            simAB_thres = tdata->pccs_sift_simAB_thres;
            break;
        default:
            ERROR ("unknown attrtype %d", fi->attrtype);
        }

        t0 = timestamp_now ();
        int n_pcorr = vis_pccs_corrset_feat_alloc (fi, fj, pdata->plink->x_ji, &Ki_view.matrix,
                      simAB_thres, tdata->pccs_chiSquare2dof,
                      &seli, &selj, pd);

        dt = timestamp_now () - t0;
        printf ("pccs (%d/%d)\t\tnpts=%d\tdt=%"PRId64"\n", n+1, pdata->fci->ntypes, n_pcorr, dt);

        if (n_pcorr > 0)
        {
            n_pcorr_sum += n_pcorr; // sum up all number of corr
            vis_feature_uv_isel_collection_add (uvi, seli, &uvic, &selic, offset_i);
            vis_feature_uv_isel_collection_add (uvj, selj, &uvjc, &seljc, offset_j);
        }

        // sel i and j will have offset depends on previous npts
        offset_i += fi->npts;
        offset_j += fj->npts;

        // if plot option on, store each npts to color code inliers
        if (pd) if (pd->n_feat_types) pd->npts_each_type[n] = n_pcorr_sum;

        // clean up
        gslu_index_free (seli);
        gslu_index_free (selj);
        gslu_matrix_free (uvi);
        gslu_matrix_free (uvj);

    }

    /* Allocate scale for feature covariance */
    perllcm_van_feature_t *fi = &pdata->fci->f[0];
    perllcm_van_feature_t *fj = &pdata->fcj->f[0];
    gsl_vector *featscalei = NULL;
    gsl_vector *featscalej = NULL;
    gsl_vector *featscalei_h = NULL;
    gsl_vector *featscalej_h = NULL;
    gsl_vector *featscalei_f = NULL;
    gsl_vector *featscalej_f = NULL;
    if (selic && seljc)
        vis_feature_2v_corrset_scale_alloc (fi, fj, selic, seljc, &featscalei, &featscalej);

    errmsg = vis_tv_minpt_check_error (n_pcorr_sum, tdata->tv_min_npts, VIS_TV_PROC_PCCS);
    if (errmsg) goto ON_ERROR;

    // GIC test
    // -------------------------------------------------------------- //
    GSLU_MATRIX_VIEW (F,3,3);
    GSLU_MATRIX_VIEW (H,3,3);

    t0 = timestamp_now ();
    double gic_f = vis_modelfit_inliers_gic_3D (uvic, uvjc, &sel_f, &uvi_f, &uvj_f, &F.matrix, VIS_MODELFIT_OCV_RANSAC);
    double gic_h = vis_modelfit_inliers_gic_2D (uvic, uvjc, &sel_h, &uvi_h, &uvj_h, &H.matrix, VIS_MODELFIT_OCV_RANSAC);
    dt = timestamp_now () - t0;

    if (sel_h)
    {
        featscalei_h = gslu_vector_sel_alloc (featscalei, sel_h);
        featscalej_h = gslu_vector_sel_alloc (featscalej, sel_h);
    }
    if (sel_f)
    {
        featscalei_f = gslu_vector_sel_alloc (featscalei, sel_f);
        featscalej_f = gslu_vector_sel_alloc (featscalej, sel_f);
    }

    // BA depends on model
    // -------------------------------------------------------------- //
    gsl_vector_const_view X_c2c1 = gsl_vector_const_view_array (pdata->plink->x_ji.mu, 6);
    gsl_matrix_const_view P_c2c1 = gsl_matrix_const_view_array (pdata->plink->x_ji.Sigma, 6, 6);
    GSLU_VECTOR_VIEW (x21_o, 6);

    double avg_camoffset = vis_feature_get_scenedepth (&pdata->fci->f[0]);
    double tri_min_dist = tdata->tv_tri_const_min_dist * avg_camoffset;
    double tri_max_dist = tdata->tv_tri_const_max_dist * avg_camoffset;

    GSLU_MATRIX_VIEW (K_or_H, 3, 3);    // will be used in debugplot

    if (gic_f < gic_h && sel_f)   // CHOOSE F
    {
        size_t n_in_h = 0;
        if (sel_h) n_in_h = sel_h->size;
        printf ("gic: F =%3.2g / n=%zu (%3.2g/%zu) \tdt=%"PRId64"\n", gic_f, sel_f->size, gic_h, n_in_h, dt);
        model_gic = PERLLCM_VAN_PLOT_DEBUG_T_GIC_F;
        gsl_matrix_memcpy (&K_or_H.matrix, &Ki_view.matrix);
        errmsg = vis_tv_minpt_check_error (sel_f->size, tdata->tv_min_npts, VIS_TV_PROC_FIN);
        if (errmsg) goto ON_ERROR;

#if USE_TRIANGULATION
        // init BA & check mahalanobis distance
        errmsg = vis_sba_init_relorient (&Ki_view.matrix, uvi_f, uvj_f, &X_c2c1.vector, &P_c2c1.matrix,
                                         tri_min_dist, tri_max_dist,
                                         tdata->tv_mdist_nav_thres, verbose,
                                         &x21_o.vector);
        if (errmsg) goto ON_ERROR;

        // run BA
        t0 = timestamp_now ();

#if USE_PROJ_NONLIN

        gsl_matrix *uvi_f_dist = gsl_matrix_calloc(uvi_f->size1, uvi_f->size2);
        gsl_matrix *uvj_f_dist = gsl_matrix_calloc(uvj_f->size1, uvj_f->size2);
        vis_distort_pts_radial (uvi_f, uvi_f_dist, &Ki_view.matrix, &distCoeffs_view.vector);
        vis_distort_pts_radial (uvj_f, uvj_f_dist, &Ki_view.matrix, &distCoeffs_view.vector);

        //Refine points based on triangulation constraint (note that triangulation assumes
        //*undistorted* points, so this is an approximation.  On the plus side, enforcing
        //the triangulation constraint only seldom removes 1 or more feature points)
        gsl_matrix *uvi_f_dist_triconst = NULL;
        gsl_matrix *uvj_f_dist_triconst = NULL;
        vis_sba_2v_rae_enforce_tri_const_alloc (&Ki_view.matrix, &x21_o.vector, uvi_f_dist, uvj_f_dist,
                                                tri_min_dist, tri_max_dist, &uvi_f_dist_triconst, &uvj_f_dist_triconst);

        //This vector can be passed into vis_sba_img_proj_Rae_jacob to compute the
        //Jacobian of the two-view reprojection function
        gsl_vector *params = gsl_vector_calloc (10 + 3*uvi_f_dist_triconst->size2);
        ret_sba = vis_sba_2v_rae_nonlin_from_model_with_tri (&Ki_view.matrix, &distCoeffs_view.vector,
                  &x21_o.vector, uvi_f_dist_triconst, uvj_f_dist_triconst,
                  tdata->tv_min_npts, pt3d_debug_mode,
                  tri_min_dist, tri_max_dist,
                  rel_pose21, rel_pose_cov21, params, verbose,
                  &X1_plot, tdata->sba_mutex, featscalei_f, featscalej_f);

        /* calibration uncertainty */
        gsl_matrix *sigmaCalib = sigma_calib_alloc ();

        /* Haralick covariance estimate code */
        gsl_matrix *S21_haralick = NULL;
        if (ret_sba > 0)
        {
            S21_haralick = vis_sba_haralick_cov_2v_Rae_alloc (sigmaCalib, params, uvi_f_dist_triconst, uvj_f_dist_triconst,
                           &Ki_view.matrix, &distCoeffs_view.vector,
                           featscalei_f, featscalej_f);
            /* gsl_matrix_memcpy (rel_pose_cov21, S21_haralick); */
            gsl_matrix_free (S21_haralick);
        }

        /* ut-based covariance estimate code */
        gsl_matrix *S21_ut = NULL;
        if (ret_sba > 0)
        {
            S21_ut = vis_sba_ut_cov_2v_Rae_alloc (sigmaCalib, params, uvi_f_dist_triconst, uvj_f_dist_triconst,
                                                  &Ki_view.matrix, &distCoeffs_view.vector, featscalei_f, featscalej_f, tdata->sba_mutex);
            gsl_matrix_add (rel_pose_cov21, S21_ut);
            gsl_matrix_free (S21_ut);
        }

        /* clean up */
        gsl_matrix_free (sigmaCalib);
        gsl_vector_free (params);
        gsl_matrix_free (uvi_f_dist);
        gsl_matrix_free (uvj_f_dist);
        gsl_matrix_free (uvi_f_dist_triconst);
        gsl_matrix_free (uvj_f_dist_triconst);
#else
        ret_sba = vis_sba_2v_rae_from_model_with_tri (&Ki_view.matrix, &x21_o.vector, uvi_f, uvj_f,
                  tdata->tv_min_npts, pt3d_debug_mode,
                  tri_min_dist, tri_max_dist,
                  rel_pose21, rel_pose_cov21, NULL, verbose,
                  &X1_plot, tdata->sba_mutex, featscalei_f, featscalej_f);
#endif
        dt = timestamp_now () - t0;
#else
        // run BA
        t0 = timestamp_now ();
        ret_sba = vis_sba_2v_rae_from_model_wo_tri (&Ki_view.matrix, &x21_o.vector, uvi_f, uvj_f,
                  tdata->tv_min_npts, pt3d_debug_mode,
                  tri_min_dist, tri_max_dist,
                  rel_pose21, rel_pose_cov21, verbose,
                  &X1_plot, tdata->sba_mutex, featscalei_f, featscalej_f);
        dt = timestamp_now () - t0;
#endif

        // done
        if (ret_sba == TV_SBA_ERROR)
            errmsg = PERLLCM_VAN_VLINK_T_MSG_SBA_E_ERROR;
        else
            type = PERLLCM_VAN_VLINK_T_TYPE_5DOF_EPIPOLAR;
    }
    else if (gic_f > gic_h && sel_h)   // CHOOSE H
    {
        size_t n_in_f = 0;
        if (sel_f) n_in_f = sel_f->size;
        printf ("gic: H =%3.2g / n=%zu (%g/%zu) \tdt=%"PRId64"\n", gic_h, sel_h->size, gic_f, n_in_f, dt);
        model_gic = PERLLCM_VAN_PLOT_DEBUG_T_GIC_H;
        gsl_matrix_memcpy (&K_or_H.matrix, &H.matrix);
        errmsg = vis_tv_minpt_check_error (sel_h->size, tdata->tv_min_npts, VIS_TV_PROC_HIN);
        if (errmsg) goto ON_ERROR;

        // init BA & check mahalanobis distance
        //errmsg = vis_sba_init_relorient (&Ki_view.matrix, uvi_h, uvj_h, &X_c2c1.vector, &P_c2c1.matrix,
        //                                 tri_min_dist, tri_max_dist,
        //                                 tdata->tv_mdist_nav_thres, verbose,
        //                                 &x21_o.vector);
        //if (errmsg) goto ON_ERROR;

        // run BA
        t0 = timestamp_now ();
#if USE_PROJ_NONLIN

        /* Distort the detected feature points */
        gsl_matrix *uvi_h_dist = gsl_matrix_calloc(uvi_h->size1, uvi_h->size2);
        gsl_matrix *uvj_h_dist = gsl_matrix_calloc(uvj_h->size1, uvj_h->size2);
        vis_distort_pts_radial (uvi_h, uvi_h_dist, &Ki_view.matrix, &distCoeffs_view.vector);
        vis_distort_pts_radial (uvj_h, uvj_h_dist, &Ki_view.matrix, &distCoeffs_view.vector);

        gsl_vector *params = gsl_vector_calloc (16 + 2*uvi_h_dist->size2);

        ret_sba = vis_sba_2v_h_nonlin_from_model (&H.matrix, &distCoeffs_view.vector, uvi_h_dist, uvj_h_dist,
                  &Ki_view.matrix, &pdata->fci->f[0], &X_c2c1.vector,
                  rel_pose21, rel_pose_cov21, params, verbose,
                  tdata->sba_mutex, featscalei_h, featscalej_h);

        /* calibration uncertainty */
        gsl_matrix *sigmaCalib = sigma_calib_alloc ();

        /* haralick-based estimate */
        gsl_matrix *haralick = NULL;
        if (ret_sba > 0)
        {
            vis_sba_haralick_cov_2v_H_alloc (sigmaCalib, params, uvi_h_dist, uvj_h_dist,
                                             &Ki_view.matrix, &distCoeffs_view.vector,
                                             featscalei_h, featscalej_h);
            /* gsl_matrix_memcpy (rel_pose_cov21, haralick); */
            gsl_matrix_free (haralick);
        }

        /* get ut-based estimate for covariance */
        gsl_matrix *covEstUt = NULL;
        if (ret_sba > 0)
        {
            vis_sba_ut_cov_2v_H_alloc (sigmaCalib, params, &H.matrix, uvi_h_dist, uvj_h_dist,
                                       &Ki_view.matrix, &distCoeffs_view.vector, &pdata->fci->f[0],
                                       featscalei_h, featscalej_h, tdata->sba_mutex);
            gsl_matrix_add (rel_pose_cov21, covEstUt);
            gsl_matrix_free (covEstUt);
        }

        /* clean up */
        gsl_matrix_free (sigmaCalib);
        gsl_matrix_free (uvi_h_dist);
        gsl_matrix_free (uvj_h_dist);
        gsl_vector_free (params);
#else
        ret_sba = vis_sba_2v_h_from_model (&H.matrix, uvi_h, uvj_h, &Ki_view.matrix, &pdata->fci->f[0], &X_c2c1.vector,
                                           rel_pose21, rel_pose_cov21, NULL, verbose, tdata->sba_mutex, featscalei_h, featscalej_h);
#endif
        dt = timestamp_now () - t0;

        // done
        if (ret_sba == TV_SBA_ERROR)
            errmsg = PERLLCM_VAN_VLINK_T_MSG_SBA_H_ERROR;
        else
            type = PERLLCM_VAN_VLINK_T_TYPE_5DOF_HOMOGRAPHY;

    }
    else  // ERROR in model fitting
    {
        errmsg = PERLLCM_VAN_VLINK_T_MSG_NO_MODEL_FIT;
    }


    // Check mahal dist to nav prior
    // -------------------------------------------------------------- //
    if (type != PERLLCM_VAN_VLINK_T_TYPE_ERROR)
    {
        printf ("motion estimation summary...\n");

        // HACK --- START
        /* double diag_cov[5]; */
        /* for (size_t i=0; i<5; i++) diag_cov[i] = gsl_matrix_get (rel_pose_cov21,i,i); */
        /* gsl_matrix_set_zero (rel_pose_cov21); */
        /* for (size_t i=0; i<5; i++) gsl_matrix_set (rel_pose_cov21, i, i, diag_cov[i]); */
        /* gsl_matrix_set (rel_pose_cov21, 0, 1, 0); */
        /* gsl_matrix_set (rel_pose_cov21, 1, 0, 0); */

        gsl_matrix_set (rel_pose_cov21, 0, 2, 0);
        gsl_matrix_set (rel_pose_cov21, 0, 3, 0);
        gsl_matrix_set (rel_pose_cov21, 0, 4, 0);
        gsl_matrix_set (rel_pose_cov21, 1, 2, 0);
        gsl_matrix_set (rel_pose_cov21, 1, 3, 0);
        gsl_matrix_set (rel_pose_cov21, 1, 4, 0);

        gsl_matrix_set (rel_pose_cov21, 2, 0, 0);
        gsl_matrix_set (rel_pose_cov21, 3, 0, 0);
        gsl_matrix_set (rel_pose_cov21, 4, 0, 0);
        gsl_matrix_set (rel_pose_cov21, 2, 1, 0);
        gsl_matrix_set (rel_pose_cov21, 3, 1, 0);
        gsl_matrix_set (rel_pose_cov21, 4, 1, 0);

        // HACK --- END

        vis_tv_print_motion (&X_c2c1.vector, &P_c2c1.matrix, &x21_o.vector, rel_pose21, rel_pose_cov21);

        if (!tdata->vanopts.manual_corr)
            errmsg = vis_tv_mdist_check_error (&X_c2c1.vector, &P_c2c1.matrix, rel_pose21, rel_pose_cov21,
                                               tdata->tv_mdist_nav_thres, NULL);

        if (errmsg) // case of error
            type = PERLLCM_VAN_VLINK_T_TYPE_ERROR;
        else
            vis_tv_print_sba (rel_pose21, rel_pose_cov21, dt);
    }

ON_ERROR:
    ;

    // publish
    //--------------------------------------------------------------------
    if (tdata->use_se_vlink)
    {
        perllcm_isam_vlink_t *vlink = vis_twoview_isam_vlink (pdata->fci->utime, pdata->fcj->utime,
                                      pdata->plink,
                                      type, errmsg, pdata->plink->link_id,
                                      rel_pose21, rel_pose_cov21);

        if (vlink)
        {
            /*if (tdata->vanopts.manual_corr && !errmsg)  { // don't publish before verifying
                size_t listlen = g_list_length (tdata->verifyset_list);
                vis_tv_prepare_plot_debug (pd, tdata->vanopts, type, errmsg,
                                           selic, seljc, sel_h, sel_f, model_gic, &K_or_H.matrix,
                                           rel_pose21, rel_pose_cov21, X1_plot, pdata->plink->x_ji, listlen);

                verify_set_t *vs = malloc (sizeof (*vs));
                vs->utime1 = pdata->fci->utime;
                vs->utime2 = pdata->fcj->utime;
                vs->se_vlink = se_vlink;
                vs->pd = pd;

                tdata->verifyset_list = g_list_prepend (tdata->verifyset_list, vs);
                //printf_glist (tdata->verifyset_list);

                if (tdata->plot_thread_idle)
                    tdata->plot_thread_idle = request_verify (tdata->verifyset_list);
            }
            else { // publish it now*/
            perllcm_isam_vlink_t_publish (shm->lcm, "VLINKS", vlink);
            perllcm_isam_vlink_t_destroy (vlink);
            //}
        }
    }
    else
    {
        perllcm_van_vlink_t *vlink = vis_twoview_vlink (pdata->fci->utime, pdata->fcj->utime, type, errmsg,
                                     selic, seljc, sel_h, sel_f,
                                     rel_pose21, rel_pose_cov21);
        if (vlink) perllcm_van_vlink_t_publish (shm->lcm, VAN_VLINK_CHANNEL, vlink);
        if (vlink) perllcm_van_vlink_t_destroy(vlink);
    }

    //if (!tdata->vanopts.manual_corr) {
    vis_tv_prepare_plot_debug (pd, tdata->vanopts, type, errmsg,
                               selic, seljc, sel_h, sel_f, model_gic, &K_or_H.matrix,
                               rel_pose21, rel_pose_cov21, X1_plot, pdata->plink->x_ji, 0);
    if (pd) perllcm_van_plot_debug_t_publish (shm->lcm, VAN_PLOT_DEBUG_CHANNEL, pd);
    if (pd) perllcm_van_plot_debug_t_destroy (pd);
    //}

    // write some data for photo mosaic
    //--------------------------------------------------------------------
    if (tdata->save_corrset && !errmsg)
    {
        if (type == PERLLCM_VAN_VLINK_T_TYPE_5DOF_EPIPOLAR)
            vanu_corrset_save2disk (pdata->fci->utime, pdata->fcj->utime, shm->logdir, rel_pose21, rel_pose_cov21, uvi_f, uvj_f);
        else if (type == PERLLCM_VAN_VLINK_T_TYPE_5DOF_HOMOGRAPHY)
            vanu_corrset_save2disk (pdata->fci->utime, pdata->fcj->utime, shm->logdir, rel_pose21, rel_pose_cov21, uvi_h, uvj_h);
        else
            printf ("Unknown type to write corrset\n");
    }

    // write plink information for JOE
    //--------------------------------------------------------------------
    if (tdata->save_link_info)
    {
        perllcm_van_vlink_t *vlink = vis_twoview_vlink (pdata->fci->utime, pdata->fcj->utime, type, errmsg,
                                     selic, seljc, sel_h, sel_f,
                                     rel_pose21, rel_pose_cov21);
        vlink->S_L = pdata->plink->S_L;
        vlink->Ig = pdata->plink->Ig;
        if (vlink) perllcm_van_vlink_t_publish (shm->lcm, VAN_LINK_INFO_CHANNEL, vlink);
        if (vlink) perllcm_van_vlink_t_destroy(vlink);
    }

    // clean up
    //--------------------------------------------------------------------
    gslu_vector_free (featscalei);
    gslu_vector_free (featscalej);
    gslu_vector_free (featscalei_h);
    gslu_vector_free (featscalej_h);
    gslu_vector_free (featscalei_f);
    gslu_vector_free (featscalej_f);
    gslu_matrix_free (uvic);
    gslu_matrix_free (uvjc);
    gslu_index_free (selic);
    gslu_index_free (seljc);
    gslu_index_free (sel_f);
    gslu_matrix_free (uvi_f);
    gslu_matrix_free (uvj_f);
    gslu_index_free (sel_h);
    gslu_matrix_free (uvi_h);
    gslu_matrix_free (uvj_h);
    gslu_vector_free (rel_pose21);
    gslu_matrix_free (rel_pose_cov21);
    gslu_matrix_free (X1_plot);

    pooldata_free (pdata);
    return;
}

// SAVE / POST PROCESSING
// -------------------------------------------------------------------------
static void
se_save_isam_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                         const se_save_isam_t *msg, void *user )
{
    thread_data_t *tdata = user;

    if (msg->type == SE_SAVE_ISAM_T_TYPE_DONE && tdata->save_link_info)   // SAVE via lcm
    {
        printf ("saving link info into ./link.data ... ");
        vanu_link_info_save2disk (".", tdata->linklist);
        printf ("done\n");
    }
}


static void
perllcm_van_vlink_info_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                   const perllcm_van_vlink_t *link, void *user)
{
    thread_data_t *tdata = user;

    perllcm_van_vlink_t *link_cpy = perllcm_van_vlink_t_copy (link);
    tdata->linklist = g_slist_append (tdata->linklist, link_cpy);
    //printf (".... and save done\n");
}

static void
fc_cache_value_destroy (void *value)
{
    perllcm_van_feature_collection_t_destroy (value);
}

gpointer
twoview_thread (gpointer user)
{
    printf ("Spawning\n");

    // thread_data state
    thread_data_t *tdata = calloc (1, sizeof (*tdata));
    tdata->fc_cache = cache_new (FC_CACHE_MAX, NULL, &fc_cache_value_destroy);

    // twoview thread configuration
    tdata->tv_pccs_fudge = 1.0;
    bot_param_get_double (shm->param, "rtvan.twoview_thread.tv_pccs_fudge", &tdata->tv_pccs_fudge);

    double alpha = 1-2*gsl_cdf_ugaussian_P (-6.0);
    double chiSquare2dof = gsl_cdf_chisq_Pinv (alpha,2)* tdata->tv_pccs_fudge * tdata->tv_pccs_fudge;
    tdata->pccs_chiSquare2dof = chiSquare2dof;

    double nav_thresh_sigma = 1.0;
    bot_param_get_double (shm->param, "rtvan.twoview_thread.nav_thresh_sigma", &nav_thresh_sigma);

    tdata->tv_mdist_nav_thres = gsl_cdf_chisq_Pinv (1-2*gsl_cdf_ugaussian_P (-nav_thresh_sigma),5);

    tdata->pccs_sift_simAB_thres = 1.0;
    bot_param_get_double (shm->param, "rtvan.twoview_thread.sift_simAB_thres", &tdata->pccs_sift_simAB_thres);

    tdata->pccs_harris_simAB_thres = 1.0;
    bot_param_get_double (shm->param, "rtvan.twoview_thread.harris_simAB_thres", &tdata->pccs_harris_simAB_thres);

    tdata->tv_min_npts = 10;
    bot_param_get_int (shm->param, "rtvan.twoview_thread.tv_min_npts", (int*)&tdata->tv_min_npts);

    tdata->tv_tri_const_min_dist = 0.0;      // default = no minus scene depth
    bot_param_get_double (shm->param, "rtvan.twoview_thread.tv_tri_const_min_dist", &tdata->tv_tri_const_min_dist);

    tdata->tv_tri_const_max_dist = 100.0;    // default = accept upto 100m
    bot_param_get_double (shm->param, "rtvan.twoview_thread.tv_tri_const_max_dist", &tdata->tv_tri_const_max_dist);

    double ocv_scale_double = 1.0;
    bot_param_get_double (shm->param, "rtvan.plot_thread.scale", &ocv_scale_double);
    tdata->ocv_scale = ocv_scale_double;

    tdata->sba_mutex = g_mutex_new();

    tdata->use_se_vlink = 0; // default = use van_vlink lcmtype
    botu_param_get_boolean_to_bool (shm->param, "rtvan.link_thread.use_se_vlink", &tdata->use_se_vlink);

    tdata->plot_thread_idle = true;

    tdata->save_corrset = 0; // default = no corrset published
    botu_param_get_boolean_to_bool (shm->param, "rtvan.post_process.save_corrset", &tdata->save_corrset);

    tdata->save_link_info = 0; // default = no plink published
    botu_param_get_boolean_to_bool (shm->param, "rtvan.post_process.save_link_info", &tdata->save_link_info);

    // helper thread pool
    tdata->pool = g_thread_pool_new (twoview_pool_thread, tdata, POOL_THREADS_MAX, 1, NULL);

    // lcm subscriptions
    perllcm_van_feature_collection_t_subscription_t *perllcm_van_feature_collection_t_sub =
        perllcm_van_feature_collection_t_subscribe (shm->lcm, VAN_FEATURE_COLLECTION_CHANNEL,
                &perllcm_van_feature_collection_t_callback, tdata);

    perllcm_van_plink_t_subscription_t *perllcm_van_plink_t_sub =
        perllcm_van_plink_t_subscribe (shm->lcm, VAN_PLINK_CHANNEL, &perllcm_van_plink_t_callback, tdata);

    perllcm_van_options_t_subscription_t *perllcm_van_options_t_sub =
        perllcm_van_options_t_subscribe (shm->lcm, VAN_OPTIONS_CHANNEL, &perllcm_van_options_t_callback, tdata);

    se_save_isam_t_subscription_t *se_save_isam_t_cmd_sub =
        se_save_isam_t_subscribe (shm->lcm, SE_SAVE_ISAM_CHANNEL, &se_save_isam_t_callback, tdata);

    perllcm_van_vlink_t_subscription_t *perllcm_van_vlink_info_sub =
        perllcm_van_vlink_t_subscribe (shm->lcm, VAN_LINK_INFO_CHANNEL, &perllcm_van_vlink_info_t_callback, tdata);

#if USER_VERIFY
    perllcm_van_verify_ack_t_subscription_t *perllcm_van_verify_ack_t_sub =
        perllcm_van_verify_ack_t_subscribe (shm->lcm, VAN_VERIFY_CHANNEL, &perllcm_van_verify_ack_t_callback, tdata);
#endif

    while (!shm->done)
    {
        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        lcmu_handle_timeout (shm->lcm, &timeout);
    }

    // wait for thread pool to expire
    g_thread_pool_free (tdata->pool, 1, 1);

    // clean up
    perllcm_van_feature_collection_t_unsubscribe (shm->lcm, perllcm_van_feature_collection_t_sub);
    perllcm_van_plink_t_unsubscribe (shm->lcm, perllcm_van_plink_t_sub);
    perllcm_van_options_t_unsubscribe (shm->lcm, perllcm_van_options_t_sub);
    se_save_isam_t_unsubscribe (shm->lcm, se_save_isam_t_cmd_sub);
    perllcm_van_vlink_t_unsubscribe (shm->lcm, perllcm_van_vlink_info_sub);
#if USER_VERIFY
    perllcm_van_verify_ack_t_unsubscribe (shm->lcm, perllcm_van_verify_ack_t_sub);
#endif

    cache_destroy (tdata->fc_cache);
    g_mutex_free (tdata->sba_mutex);
    free (tdata);

    printf ("Exiting\n");
    g_thread_exit (0);
    return NULL;
}
