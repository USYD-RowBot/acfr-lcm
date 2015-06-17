#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <inttypes.h> // needed for PRId64 macros

#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_eigen.h>

#include "perls-lcmtypes/se_propose_link_t.h"

#include "perls-common/units.h"
#include "perls-common/timestamp.h"
#include "perls-math/dm.h"
#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "epipolar.h"
#include "twoview.h"

#define VLINK_MSG_LEN 20

#define RTOD (UNITS_RADIAN_TO_DEGREE)

/* enable debug calls to gslu_matrix_printf ?*/
//#define TWOVIEW_VERBOSE

perllcm_van_vlink_t *
vis_twoview_vlink (int64_t utime_i, int64_t utime_j, int32_t type, int32_t errmsg,
                   gslu_index *seli, gslu_index *selj, gslu_index *sel_h, gslu_index *sel_f,
                   gsl_vector *z_gsl, gsl_matrix *R_gsl)
{
    int dof = 5;    // 5 dof pose estimate

    perllcm_van_vlink_t *vlink = calloc (1, sizeof (*vlink));
    vlink->utime_i = utime_i;
    vlink->utime_j = utime_j;
    vlink->type = type;
    vlink->msg = errmsg;

    if (type > 0)   // no error
    {
        // publish z and R
        gsl_vector_view z_view = gsl_vector_view_array (vlink->z, dof);
        gsl_matrix_view R_view = gsl_matrix_view_array (vlink->R, dof, dof);
        gsl_vector_memcpy (&z_view.vector, z_gsl);
        gsl_matrix_memcpy (&R_view.matrix, R_gsl);

        // publish inliers indeces
        int32_t n_in = 0;
        if (type == PERLLCM_VAN_VLINK_T_TYPE_5DOF_EPIPOLAR)
            n_in = (int32_t) sel_f->size;
        else //(type == PERLLCM_VAN_VLINK_T_TYPE_5DOF_HOMOGRAPHY)
            n_in = (int32_t) sel_h->size;

        vlink->n_inliers = n_in;
        vlink->isel_i = malloc (n_in*sizeof(int32_t));
        vlink->isel_j = malloc (n_in*sizeof(int32_t));

        for (size_t ii=0; ii<n_in; ii++)
        {
            if (type == PERLLCM_VAN_VLINK_T_TYPE_5DOF_EPIPOLAR)
            {
                vlink->isel_i[ii] = gslu_index_get (seli, gslu_index_get (sel_f, ii));
                vlink->isel_j[ii] = gslu_index_get (selj, gslu_index_get (sel_f, ii));
            }
            else
            {
                vlink->isel_i[ii] = gslu_index_get (seli, gslu_index_get (sel_h, ii));
                vlink->isel_j[ii] = gslu_index_get (selj, gslu_index_get (sel_h, ii));
            }
        }
    }

    return vlink;
}

se_publish_link_t *
vis_twoview_se_vlink (int64_t utime_i, int64_t utime_j,
                      int32_t type, int32_t errmsg, int64_t link_id,
                      gsl_vector *z_gsl, gsl_matrix *R_gsl)
{
    int dof = 5;    // 5 dof pose estimate

    //int32_t    publisher_id;
    //int32_t    sensor_id;
    //int32_t    link_id;

    se_publish_link_t *se_vlink = calloc (1, sizeof (*se_vlink));
    se_vlink->utime1 = utime_i;
    se_vlink->utime2 = utime_j;
    se_vlink->link_type = SE_PUBLISH_LINK_T_LINK_POSE3DB;
    se_vlink->sensor_id = SE_PROPOSE_LINK_T_SENSOR_ID_CAMERA;
    se_vlink->comment = calloc (1, VLINK_MSG_LEN * sizeof (char));
    snprintf(se_vlink->comment, VLINK_MSG_LEN, "Camera Link");
    se_vlink->link_id = link_id;

    if (type > 0)   // no error
    {
        se_vlink->accept = 1;      // boolean
        se_vlink->accept_code = SE_PUBLISH_LINK_T_LINK_ACCEPTED;

        se_vlink->n = dof;
        se_vlink->n2 = dof*dof;
        se_vlink->measurement = malloc (se_vlink->n * sizeof (double));
        se_vlink->sigma = malloc (se_vlink->n2 * sizeof (double));

        // publish z and R
        gsl_vector_view z_view = gsl_vector_view_array (se_vlink->measurement, dof);
        gsl_matrix_view R_view = gsl_matrix_view_array (se_vlink->sigma, dof, dof);
        gsl_vector_memcpy (&z_view.vector, z_gsl);
        gsl_matrix_memcpy (&R_view.matrix, R_gsl);
    }
    else   // error occured
    {
        se_vlink->accept = 0;      // boolean

        if (errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_PCCS ||
                errmsg == PERLLCM_VAN_VLINK_T_MSG_TRI_CONST)
            se_vlink->accept_code = SE_PUBLISH_LINK_T_LINK_MIN_CORR;
        else if (errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_INLIERS_E ||
                 errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_INLIERS_H)
            se_vlink->accept_code = SE_PUBLISH_LINK_T_LINK_MIN_CORR;
        else if (errmsg == PERLLCM_VAN_VLINK_T_MSG_SBA_E_ERROR ||
                 errmsg == PERLLCM_VAN_VLINK_T_MSG_SBA_H_ERROR)
            se_vlink->accept_code = SE_PUBLISH_LINK_T_LINK_MIN_INLIERS;
        else if (errmsg == PERLLCM_VAN_VLINK_T_MSG_NO_MODEL_FIT)
            se_vlink->accept_code = SE_PUBLISH_LINK_T_LINK_INVALID_MODEL;
        else if (errmsg == PERLLCM_VAN_VLINK_T_MSG_MDIST_NAV)
            se_vlink->accept_code = SE_PUBLISH_LINK_T_LINK_MDIST_NAV;
    }

    return se_vlink;
}

perllcm_isam_vlink_t *
vis_twoview_isam_vlink (int64_t utime_i, int64_t utime_j,
                        perllcm_van_plink_t *plink,
                        int32_t type, int32_t errmsg, int64_t link_id,
                        gsl_vector *z_gsl, gsl_matrix *R_gsl)
{
    int dof = 5;    //5 dof pose estimate

    perllcm_isam_vlink_t *vlink = calloc (1, sizeof (*vlink));
    vlink->utime = timestamp_now();
    vlink->id1 = utime_i;
    vlink->id2 = utime_j;
    vlink->link_type = PERLLCM_ISAM_VLINK_T_LINK_POSE3DB;
    vlink->sensor_id = PERLLCM_ISAM_VLINK_T_SENSOR_CAMERA;
    vlink->link_id = link_id;
    vlink->dynamic_xvs = plink->dynamic_xvs;
    memcpy (vlink->x_vs1, plink->x_vs1, sizeof(vlink->x_vs1));
    memcpy (vlink->x_vs2, plink->x_vs2, sizeof(vlink->x_vs2));

    if (type > 0)   // no error
    {
        vlink->accept = 1;      // boolean
        vlink->accept_code = PERLLCM_ISAM_VLINK_T_CODE_ACCEPTED;

        vlink->n = dof;
        vlink->n2 = dof*dof;
        vlink->z = malloc (vlink->n * sizeof (*vlink->z));
        vlink->R = malloc (vlink->n2 * sizeof (*vlink->R));

        // publish z and R
        gsl_vector_view z_view = gsl_vector_view_array (vlink->z, dof);
        gsl_matrix_view R_view = gsl_matrix_view_array (vlink->R, dof, dof);
        gsl_vector_memcpy (&z_view.vector, z_gsl);
        gsl_matrix_memcpy (&R_view.matrix, R_gsl);
    }
    else   // error occured
    {
        vlink->accept = 0;      // boolean

        if (errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_PCCS ||
                errmsg == PERLLCM_VAN_VLINK_T_MSG_TRI_CONST)
            vlink->accept_code = PERLLCM_ISAM_VLINK_T_CODE_MIN_CORR;
        else if (errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_INLIERS_E ||
                 errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_INLIERS_H)
            vlink->accept_code = PERLLCM_ISAM_VLINK_T_CODE_MIN_CORR;
        else if (errmsg == PERLLCM_VAN_VLINK_T_MSG_SBA_E_ERROR ||
                 errmsg == PERLLCM_VAN_VLINK_T_MSG_SBA_H_ERROR)
            vlink->accept_code = PERLLCM_ISAM_VLINK_T_CODE_MIN_INLIERS;
        else if (errmsg == PERLLCM_VAN_VLINK_T_MSG_NO_MODEL_FIT)
            vlink->accept_code = PERLLCM_ISAM_VLINK_T_CODE_INVALID_MODEL;
        else if (errmsg == PERLLCM_VAN_VLINK_T_MSG_MDIST_NAV)
            vlink->accept_code = PERLLCM_ISAM_VLINK_T_CODE_MDIST_NAV;
    }

    return vlink;
}

gsl_vector *
vis_tv_use_navprior (const gsl_matrix *K, const gsl_matrix *uv1, const gsl_matrix *uv2,
                     const gsl_vector *x21, const gsl_matrix *p21, const gsl_vector *x21_prior,
                     int nsamples, bool verbose)
{

    double mdist = GSL_POSINF;
    gsl_vector *min_x21 = gsl_vector_alloc (6);

    // rph and cov_rph
    gsl_matrix_const_view cov_rph = gsl_matrix_const_submatrix (p21,3,3,3,3);

    // eig decomp
    GSLU_MATRIX_VIEW (VsqrtD, 3,3);
    GSLU_VECTOR_VIEW (eval, 3);
    GSLU_MATRIX_VIEW (V, 3,3);
    GSLU_MATRIX_VIEW (sqrtD, 3,3);
    gsl_matrix_memcpy (&VsqrtD.matrix, &cov_rph.matrix); // using VsqrtD as workspace
    gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc (3);
    gsl_eigen_symmv (&VsqrtD.matrix, &eval.vector, &V.matrix, w);
    gsl_eigen_symmv_free (w);
    gsl_eigen_symmv_sort (&eval.vector, &V.matrix, GSL_EIGEN_SORT_ABS_ASC);
    for (size_t i=0; i<eval.vector.size; i++)
        gsl_vector_set (&eval.vector, i, sqrt (gsl_vector_get (&eval.vector, i)));
    gsl_vector_view d = gsl_matrix_diagonal (&sqrtD.matrix);
    gsl_vector_memcpy (&d.vector, &eval.vector);
    gslu_blas_mm (&VsqrtD.matrix, &V.matrix, &sqrtD.matrix);

    // randomly sample rph space
    // random number gen ~N(0,1)
    const gsl_rng_type * T;
    gsl_rng * r;
    gsl_rng_env_setup ();
    T = gsl_rng_default;
    r = gsl_rng_alloc (T);

    // work spaces
    GSLU_VECTOR_VIEW (sample_rph, 3);
    GSLU_MATRIX_VIEW (sample_R, 3,3);
    GSLU_VECTOR_VIEW (drph, 3);

    GSLU_VECTOR_VIEW (t, 3);
    GSLU_MATRIX_VIEW (R, 3,3);
    GSLU_VECTOR_VIEW (X, 6);

    // first sample == rph itself with no drph
    gsl_vector_const_view rph = gsl_vector_const_subvector (x21,3,3);
    gsl_vector_memcpy (&sample_rph.vector, &rph.vector);    // drph = zero
    so3_rotxyz (sample_R.matrix.data, sample_rph.vector.data);
    /* double costi = vis_epi_relorient_horn (K, uv1, uv2, &sample_R.matrix, */
    /*                                        &R.matrix, &t.vector, verbose); */
    ssc_pose_set_Rt_gsl (&X.vector, &R.matrix, &t.vector);

    GSLU_MATRIX_VIEW (invCov, 6, 6);
    gslu_matrix_inv (&invCov.matrix, p21);
    GSLU_INDEX_VIEW (c, 3, {3, 4, 5});
    double dist = gslu_vector_mahal_circ_dist (&X.vector, x21_prior, &invCov.matrix, &c.vector);
    if (dist < mdist)
    {
        mdist = dist;
        gsl_vector_memcpy (min_x21, &X.vector);
    }

    for (size_t i=0; i<nsamples; i++)
    {
        gsl_vector_set (&t.vector, 0, gsl_ran_gaussian (r, 1)); // using t for r.v. storage
        gsl_vector_set (&t.vector, 1, gsl_ran_gaussian (r, 1)); // using t for r.v. storage
        gsl_vector_set (&t.vector, 2, gsl_ran_gaussian (r, 1)); // using t for r.v. storage

        // samples = V*sqrt(D)*randn(3,Nsamps);
        gslu_blas_mv (&drph.vector, &VsqrtD.matrix, &t.vector);

        // rph_i = rph_mean + drph_i;
        gsl_vector_memcpy (&sample_rph.vector, &rph.vector);
        gsl_vector_add (&sample_rph.vector, &drph.vector);

        so3_rotxyz (sample_R.matrix.data, sample_rph.vector.data);
        /* costi = vis_epi_relorient_horn (K, uv1, uv2, &sample_R.matrix, */
        /*                                 &R.matrix, &t.vector, verbose); */
        ssc_pose_set_Rt_gsl (&X.vector, &R.matrix, &t.vector);

        dist = gslu_vector_mahal_circ_dist (&X.vector, x21_prior, &invCov.matrix, &c.vector);
        if (dist < mdist)
        {
            mdist = dist;
            gsl_vector_memcpy (min_x21, &X.vector);
        }
    }

    // clean up
    gsl_rng_free (r);

    return min_x21;

}

int32_t
vis_tv_mdist_check_error (const gsl_vector *nav_p21, const gsl_matrix *nav_cov21,
                          const gsl_vector *cam_p21, const gsl_matrix *cam_cov21,
                          const double thresh, double *min_mdist)
{
    int32_t errmsg = PERLLCM_VAN_VLINK_T_MSG_NO_ERROR;

    // check dimension
    assert (nav_p21->size == 6 && nav_cov21->size1 == 6 && nav_cov21->size2 == 6
            && cam_p21->size == 5 && cam_cov21->size1 == 5 && cam_cov21->size2 == 5);

    // nav prior to bearing only vector
    GSLU_VECTOR_VIEW (nav_p21_5dof, 5);
    GSLU_MATRIX_VIEW (nav_cov21_5dof, 5, 5);
    dm_trans2dm_pose_cov (nav_p21, nav_cov21, &nav_p21_5dof.vector, &nav_cov21_5dof.matrix, NULL);


    GSLU_MATRIX_VIEW (nav_cov_inv, 5, 5);
    gsl_matrix_add (&nav_cov21_5dof.matrix, cam_cov21);
    gslu_matrix_inv (&nav_cov_inv.matrix, &nav_cov21_5dof.matrix);
    GSLU_INDEX_VIEW (c, 5, {0, 1, 2, 3, 4});
    double mdist = gslu_vector_mahal_circ_dist (&nav_p21_5dof.vector, cam_p21, &nav_cov_inv.matrix, &c.vector);

    if ( mdist > thresh)
    {
        printf ("[twoview]    ERROR: MAHAL. dist. (%g) > thresh (%g)\n", mdist, thresh);
        errmsg = PERLLCM_VAN_VLINK_T_MSG_MDIST_NAV;
    }
    else
        printf ("[twoview]    PASS: MAHAL. dist. (%g)\n", mdist);

    return errmsg;
}

int32_t
vis_tv_minpt_check_error (size_t n, size_t minpt, size_t proc_name)
{
    int32_t errmsg = PERLLCM_VAN_VLINK_T_MSG_NO_ERROR;

    if (n < minpt)
    {
        if (proc_name == VIS_TV_PROC_PCCS)
        {
            printf ("[twoview]    ERROR: NPTS (pccs): npts (%d) < required (%d)\n", (int) n, (int) minpt);
            errmsg = PERLLCM_VAN_VLINK_T_MSG_MIN_PCCS;
        }
        else if (proc_name == VIS_TV_PROC_FIN)
        {
            printf ("[twoview]    ERROR: NPTS (f_inliers): npts (%d) < required (%d)\n", (int) n, (int) minpt);
            errmsg = PERLLCM_VAN_VLINK_T_MSG_MIN_INLIERS_E;
        }
        else if (proc_name == VIS_TV_PROC_HIN)
        {
            printf ("[twoview]    ERROR: NPTS (h_inliers): npts (%d) < required (%d)\n", (int) n, (int) minpt);
            errmsg = PERLLCM_VAN_VLINK_T_MSG_MIN_INLIERS_H;
        }
        else
            printf ("[twoview]    Unknown process name\n");
    }

    return errmsg;
}
// print results
//---------------------------------------------------------------------------------
void
vis_tv_print_sba (const gsl_vector *rel_pose21, const gsl_matrix *rel_pose_cov21, int64_t dt)
{
    printf ("[twoview]    registered! --------------------- \tdt=%"PRId64"\n", dt);

#ifdef TWOVIEW_VERBOSE
    gslu_vector_printfc (rel_pose21,"rel_pose21", NULL, CblasTrans);
    gslu_matrix_printf (rel_pose_cov21,"rel_pose_cov21"); //,"%10.4e",CblasNoTrans);
#endif

    printf ("\t     z = [%3.4f  %3.4f  %3.4f  %3.4f  %3.4f]\n",
            gsl_vector_get (rel_pose21, 0), gsl_vector_get (rel_pose21, 1), gsl_vector_get (rel_pose21, 2), gsl_vector_get (rel_pose21, 3), gsl_vector_get (rel_pose21, 4));
    printf ("\t     R = [%3.4f  %3.4f  %3.4f  %3.4f  %3.4f\n\t\t  %3.4f  %3.4f  %3.4f  %3.4f  %3.4f\n\t\t  %3.4f  %3.4f  %3.4f  %3.4f  %3.4f\n\t\t  %3.4f  %3.4f  %3.4f  %3.4f  %3.4f\n\t\t  %3.4f  %3.4f  %3.4f  %3.4f  %3.4f]\n\n",
            gsl_matrix_get (rel_pose_cov21, 0, 0), gsl_matrix_get (rel_pose_cov21, 0, 1), gsl_matrix_get (rel_pose_cov21, 0, 2), gsl_matrix_get (rel_pose_cov21, 0, 3), gsl_matrix_get (rel_pose_cov21, 0, 4),
            gsl_matrix_get (rel_pose_cov21, 1, 0), gsl_matrix_get (rel_pose_cov21, 1, 1), gsl_matrix_get (rel_pose_cov21, 1, 2), gsl_matrix_get (rel_pose_cov21, 1, 3), gsl_matrix_get (rel_pose_cov21, 1, 4),
            gsl_matrix_get (rel_pose_cov21, 2, 0), gsl_matrix_get (rel_pose_cov21, 2, 1), gsl_matrix_get (rel_pose_cov21, 2, 2), gsl_matrix_get (rel_pose_cov21, 2, 3), gsl_matrix_get (rel_pose_cov21, 2, 4),
            gsl_matrix_get (rel_pose_cov21, 3, 0), gsl_matrix_get (rel_pose_cov21, 3, 1), gsl_matrix_get (rel_pose_cov21, 3, 2), gsl_matrix_get (rel_pose_cov21, 3, 3), gsl_matrix_get (rel_pose_cov21, 3, 4),
            gsl_matrix_get (rel_pose_cov21, 4, 0), gsl_matrix_get (rel_pose_cov21, 4, 1), gsl_matrix_get (rel_pose_cov21, 4, 2), gsl_matrix_get (rel_pose_cov21, 4, 3), gsl_matrix_get (rel_pose_cov21, 4, 4));
}

void
vis_tv_print_motion (const gsl_vector *nav21, const gsl_matrix *navS21,
                     const gsl_vector *horn21,
                     const gsl_vector *cam21, const gsl_matrix *camS21)
{
    // convert nav into 5 dof measurement
    GSLU_VECTOR_VIEW (nav_p21_5dof, 5);
    GSLU_MATRIX_VIEW (nav_cov21_5dof, 5, 5);
    dm_trans2dm_pose_cov (nav21, navS21, &nav_p21_5dof.vector, &nav_cov21_5dof.matrix, NULL);

    double nav_a, nav_e, nav_r, nav_p, nav_h;
    nav_a = gsl_vector_get (&nav_p21_5dof.vector,0);
    nav_e = gsl_vector_get (&nav_p21_5dof.vector,1);
    nav_r = gsl_vector_get (&nav_p21_5dof.vector,2);
    nav_p = gsl_vector_get (&nav_p21_5dof.vector,3);
    nav_h = gsl_vector_get (&nav_p21_5dof.vector,4);

    double navS_a, navS_e, navS_r, navS_p, navS_h;
    navS_a = sqrt (gsl_matrix_get (&nav_cov21_5dof.matrix,0,0));
    navS_e = sqrt (gsl_matrix_get (&nav_cov21_5dof.matrix,1,1));
    navS_r = sqrt (gsl_matrix_get (&nav_cov21_5dof.matrix,2,2));
    navS_p = sqrt (gsl_matrix_get (&nav_cov21_5dof.matrix,3,3));
    navS_h = sqrt (gsl_matrix_get (&nav_cov21_5dof.matrix,4,4));

    printf ("\n\t     [deg]\t%7s\t%7s\t%7s\t%7s\t%7s\n","az","el","r","p","h");
    printf ("\t     --------------------------	  -----------------------\n");
    printf ("\t     nav\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\n", nav_a*RTOD, nav_e*RTOD, nav_r*RTOD, nav_p*RTOD, nav_h*RTOD);
    printf ("\t     +/-\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\n\n", navS_a*RTOD, navS_e*RTOD, navS_r*RTOD, navS_p*RTOD, navS_h*RTOD);

    if (horn21)
    {
        GSLU_VECTOR_VIEW (b21, 3);
        gsl_vector_const_view t21 = gsl_vector_const_subvector (horn21, 0, 3);
        gsl_vector_const_view rph21 = gsl_vector_const_subvector (horn21, 3, 3);
        dm_trans2dm_gsl (&t21.vector, &b21.vector, NULL);

        double horn_a, horn_e, horn_r, horn_p, horn_h;
        horn_a = gsl_vector_get (&b21.vector,0);
        horn_e = gsl_vector_get (&b21.vector,1);
        horn_r = gsl_vector_get (&rph21.vector,0);
        horn_p = gsl_vector_get (&rph21.vector,1);
        horn_h = gsl_vector_get (&rph21.vector,2);
        printf ("\t     horn\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\n\n", horn_a*RTOD, horn_e*RTOD, horn_r*RTOD, horn_p*RTOD, horn_h*RTOD);

    }

    if (cam21 && camS21)
    {
        double cam_a, cam_e, cam_r, cam_p, cam_h;
        cam_a = gsl_vector_get (cam21,0);
        cam_e = gsl_vector_get (cam21,1);
        cam_r = gsl_vector_get (cam21,2);
        cam_p = gsl_vector_get (cam21,3);
        cam_h = gsl_vector_get (cam21,4);

        double camS_a, camS_e, camS_r, camS_p, camS_h;
        camS_a = sqrt (gsl_matrix_get (camS21,0,0));
        camS_e = sqrt (gsl_matrix_get (camS21,1,1));
        camS_r = sqrt (gsl_matrix_get (camS21,2,2));
        camS_p = sqrt (gsl_matrix_get (camS21,3,3));
        camS_h = sqrt (gsl_matrix_get (camS21,4,4));

        printf ("\t     sba\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\n", cam_a*RTOD, cam_e*RTOD, cam_r*RTOD, cam_p*RTOD, cam_h*RTOD);
        printf ("\t     +/-\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\t%+7.2f\n\n", camS_a*RTOD, camS_e*RTOD, camS_r*RTOD, camS_p*RTOD, camS_h*RTOD);
    }
}


// prepare plot_debug_t
//---------------------------------------------------------------------------------
perllcm_van_plot_debug_t *
vis_tv_init_plot_debug (perllcm_van_options_t van_opt,
                        const perllcm_van_feature_collection_t *fci, const perllcm_van_feature_collection_t *fcj,
                        const perllcm_van_calib_t *calib)
{

    int64_t utime_i = fci->utime;
    int64_t utime_j = fcj->utime;

    perllcm_van_plot_debug_t *pd = NULL;


    if (!van_opt.vis_plot_put_corr &&
            !van_opt.vis_plot_in_and_out &&
            !van_opt.vis_plot_inliers &&
            !van_opt.vis_plot_relpose &&
            !van_opt.vis_plot_3pts &&
            !van_opt.vis_plot_relpose_3pts &&
            !van_opt.vis_plot_search_ellipses &&
            !van_opt.vis_plot_summary &&
            !van_opt.manual_corr)  // all off
    {

        return pd;
    }
    else
    {
        pd = calloc (1, sizeof (*pd));
        pd->utime_i = utime_i;
        pd->utime_j = utime_j;

        // image width and height are needed for sampling
        if (van_opt.vis_plot_search_ellipses)
        {
            pd->plt_ellipses = 1;
            pd->img_w = calib->width;
            pd->img_h = calib->height;
        }

        // inliers are needed
        if (van_opt.vis_plot_put_corr || van_opt.vis_plot_inliers || van_opt.vis_plot_in_and_out
                || van_opt.vis_plot_summary || van_opt.manual_corr)
        {
            pd->n_feat_types = fci->ntypes;
            pd->npts_each_type = malloc (pd->n_feat_types * sizeof (int32_t));
        }

        return pd;
    }
}

void
vis_tv_prepare_plot_debug (perllcm_van_plot_debug_t *pd, perllcm_van_options_t van_opt, int32_t type, int32_t errmsg,
                           gslu_index *selic, gslu_index *seljc, gslu_index *sel_h, gslu_index *sel_f, int32_t model_gic,
                           gsl_matrix *K_or_H,
                           gsl_vector *z_gsl, gsl_matrix *R_gsl, gsl_matrix *X1, perllcm_pose3d_t p21,
                           size_t nlink_remaining)
{
    // if option has been on/off from viewer pd could be null
    // skip this round, and plot in the next time
    if (!pd)
        return;

    int8_t need_inliers = van_opt.vis_plot_put_corr
                          || van_opt.vis_plot_in_and_out
                          || van_opt.vis_plot_inliers
                          || van_opt.vis_plot_summary
                          || van_opt.manual_corr;

    // pccs
    // --------------------------------------
    if (need_inliers)   // seli and selj are needed
    {
        if (selic && seljc && selic->size == seljc->size)
        {
            pd->n_in_pccs = selic->size;
            pd->isel_pccs_i = malloc (selic->size * sizeof (int32_t));
            pd->isel_pccs_j = malloc (seljc->size * sizeof (int32_t));
            for (size_t ii=0; ii<selic->size; ii++)
            {
                pd->isel_pccs_i[ii] = gslu_index_get (selic, ii);
                pd->isel_pccs_j[ii] = gslu_index_get (seljc, ii);
            }
        }
    }

    // inliers & outliers OR inliers only
    // --------------------------------------
    if (van_opt.vis_plot_in_and_out || van_opt.vis_plot_inliers || van_opt.vis_plot_summary || van_opt.manual_corr)
    {
        if (model_gic == PERLLCM_VAN_PLOT_DEBUG_T_GIC_F)
        {
            if (selic && seljc && selic->size == seljc->size && sel_f)
            {
                pd->n_in = sel_f->size;
                pd->isel = malloc (sel_f->size * sizeof (int32_t));

                for (size_t ii=0; ii<sel_f->size; ii++)
                    pd->isel[ii] = gslu_index_get (sel_f, ii);

                pd->model_gic = model_gic;
                if (type > 0)
                    pd->reg_result = PERLLCM_VAN_PLOT_DEBUG_T_REG_SUCC;
                else
                    pd->reg_result = PERLLCM_VAN_PLOT_DEBUG_T_REG_FAIL;

            }
        }
        else  //(model_gic == PERLLCM_VAN_PLOT_DEBUG_T_GIC_H) {
        {
            if (selic && seljc && selic->size == seljc->size && sel_h)
            {
                pd->n_in = sel_h->size;
                pd->isel = malloc (sel_h->size * sizeof (int32_t));

                for (size_t ii=0; ii<sel_h->size; ii++)
                    pd->isel[ii] = gslu_index_get (sel_h, ii);

                pd->model_gic = model_gic;
                if (type > 0)
                    pd->reg_result = PERLLCM_VAN_PLOT_DEBUG_T_REG_SUCC;
                else
                    pd->reg_result = PERLLCM_VAN_PLOT_DEBUG_T_REG_FAIL;

            }
        }
        pd->errmsg = errmsg;
    }

    // relative pose int64_t utime_i, int64_t utime_j
    // ---------------------------------------------------
    gsl_vector_view t21 = gsl_vector_view_array (p21.mu, 3);
    GSLU_VECTOR_VIEW (b21,3);
    dm_trans2dm_gsl (&t21.vector, &b21.vector, NULL);

    // summary motion
    if (van_opt.vis_plot_summary && type > 0)
    {
        if (model_gic == PERLLCM_VAN_PLOT_DEBUG_T_GIC_F)
        {
            double mag = gsl_vector_get (&b21.vector, 2);
            GSLU_VECTOR_VIEW (z_6dof, 6);
            gsl_vector_view z_6dof_xyz = gsl_vector_subvector (&z_6dof.vector, 0, 3);
            gsl_vector_view z_6dof_rph = gsl_vector_subvector (&z_6dof.vector, 3, 3);
            gsl_vector_view z_ae = gsl_vector_subvector (z_gsl, 0, 2);
            gsl_vector_view z_rph = gsl_vector_subvector (z_gsl, 2, 3);

            double b_data[3] = {z_ae.vector.data[0], z_ae.vector.data[1], mag};
            dm_dm2trans (b_data, z_6dof_xyz.vector.data, NULL);
            gsl_vector_memcpy (&z_6dof_rph.vector, &z_rph.vector);

            // uv2' F21 uv1 = 0
            gsl_matrix_view F21 = gsl_matrix_view_array (pd->model, 3, 3);
            vis_epi_F_from_KX (&F21.matrix, K_or_H, &z_6dof.vector);
        }
        else    // homograpy
        {
            gsl_matrix_view H = gsl_matrix_view_array (pd->model, 3, 3);
            gsl_matrix_memcpy (&H.matrix, K_or_H);
        }
    }

    if (van_opt.vis_plot_relpose && type > 0)
    {
        pd->dof = 5;
        gsl_vector_view x21_view = gsl_vector_view_array (pd->x21, pd->dof);
        gsl_vector_memcpy (&x21_view.vector, z_gsl);
        pd->x21[5] = gsl_vector_get (&b21.vector, 2);   // last elem == scale

        // navigation prior
        gsl_vector_view nav21_view = gsl_vector_view_array (pd->nav21, 6);
        gsl_vector_view p21_view = gsl_vector_view_array (p21.mu, 6);
        gsl_vector_memcpy (&nav21_view.vector, &p21_view.vector);
    }

    if (van_opt.vis_plot_3pts && type > 0 && X1)
    {
        pd->npts3d = X1->size2; // 3xN matrix
        pd->x = malloc (pd->npts3d * sizeof (float));
        pd->y = malloc (pd->npts3d * sizeof (float));
        pd->z = malloc (pd->npts3d * sizeof (float));
        for (size_t i=0; i<pd->npts3d; i++)
        {
            pd->x[i] = (float) gsl_matrix_get (X1, 0, i);
            pd->y[i] = (float) gsl_matrix_get (X1, 1, i);
            pd->z[i] = (float) gsl_matrix_get (X1, 2, i);
        }
    }

    if (van_opt.vis_plot_relpose_3pts && type > 0 && X1)
    {
        pd->dof = 5;
        gsl_vector_view x21_view = gsl_vector_view_array (pd->x21, pd->dof);
        gsl_vector_memcpy (&x21_view.vector, z_gsl);
        pd->x21[5] = gsl_vector_get (&b21.vector, 2);   // last elem == scale

        pd->npts3d = X1->size2; // 3xN matrix
        pd->x = malloc (pd->npts3d * sizeof (float));
        pd->y = malloc (pd->npts3d * sizeof (float));
        pd->z = malloc (pd->npts3d * sizeof (float));
        for (size_t i=0; i<pd->npts3d; i++)
        {
            pd->x[i] = (float) gsl_matrix_get (X1, 0, i);
            pd->y[i] = (float) gsl_matrix_get (X1, 1, i);
            pd->z[i] = (float) gsl_matrix_get (X1, 2, i);
        }

        // navigation prior
        gsl_vector_view nav21_view = gsl_vector_view_array (pd->nav21, 6);
        gsl_vector_view p21_view = gsl_vector_view_array (p21.mu, 6);
        gsl_vector_memcpy (&nav21_view.vector, &p21_view.vector);

    }

    // prepare plot debug for search bound inside pccs.c
    // see _vis_pccs_prepare_plot_debug ()

    // additional information for verification window
    if (van_opt.manual_corr)
        pd->nlink_remaining = nlink_remaining;
}

