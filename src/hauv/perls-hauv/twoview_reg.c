#include <stdio.h>
#include <stdlib.h>

#include "perls-lcmtypes/perllcm_van_vlink_t.h"

#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"

#include "perls-vision/calib.h"
#include "perls-vision/feature.h"
#include "perls-vision/modelfit.h"
#include "perls-vision/featuregpu.h"
#include "perls-vision/epipolar.h"
#include "perls-vision/homography.h"
#include "perls-vision/sba.h"

#include "twoview_reg.h"

static gsl_vector*
_pose_from_Rt_alloc (const gsl_matrix *R, const gsl_vector *t)
{
    gsl_vector *x21 = gsl_vector_calloc (6);
    gsl_vector_view x21_xyz = gsl_vector_subvector (x21, 0, 3);
    gsl_vector_view x21_rph = gsl_vector_subvector (x21, 3, 3);

    gsl_vector_memcpy (&x21_xyz.vector, t);
    so3_rot2rph_gsl (R, &x21_rph.vector);

    return x21;
}

/* returns number of inliers */
static int
_get_correspondence_alloc (const gsl_matrix *uv1, const gsl_matrix *uv2,
                           gsl_matrix **uv1Inliers, gsl_matrix **uv2Inliers, int method)
{
    gslu_index *inliers;
    int n_inliers;

    /* correspondence will depend on which method is passed */
    switch (method)
    {
    case PERLS_HAUV_CORRESP_H:
    {
        gsl_matrix *H = gsl_matrix_calloc (3, 3);
        n_inliers = vis_modelfit_H_ocv (uv1, uv2, H, &inliers, VIS_MODELFIT_RANSAC);
        gsl_matrix_free (H);
        break;
    }
    case PERLS_HAUV_CORRESP_E:
        printf ("Essential matrix not yet supported\n");
        break;
    case PERLS_HAUV_CORRESP_F:
    {
        gsl_matrix *F = gsl_matrix_calloc (3, 3);
        n_inliers = vis_modelfit_F_ocv (uv1, uv2, F, &inliers, VIS_MODELFIT_RANSAC);
        gsl_matrix_free (F);
        break;
    }
    default:
        printf ("Unknown correspondence method\n");
        return EXIT_FAILURE;
    }

    if (n_inliers)
    {
        *uv1Inliers = gslu_matrix_selcol_alloc (uv1, inliers);
        *uv2Inliers = gslu_matrix_selcol_alloc (uv2, inliers);
    }

    /* clean up */
    gslu_index_free (inliers);

    return n_inliers;
}

static int
_get_pose_guess_alloc (gsl_matrix *uv1, gsl_matrix *uv2, gsl_matrix ***RMats, gsl_vector ***tVecs, int method)
{
    int numGuesses = 0;
    gslu_index *inliers;

    switch (method)
    {
    case PERLS_HAUV_CORRESP_H:
    {
        gsl_matrix *H = gsl_matrix_calloc (3, 3);
        vis_modelfit_H_ocv (uv1, uv2, H, &inliers, VIS_MODELFIT_RANSAC);
        gsl_vector **NVecs;
        vis_homog_pose_decomp_alloc (H, RMats, tVecs, &NVecs);
        gsl_matrix_free (H);
        vis_homog_pose_decomp_free (NULL, NULL, NVecs);
        numGuesses = VIS_HOMOG_NUM_POSES_FROM_H;
        break;
    }
    case PERLS_HAUV_CORRESP_E:
    {
        printf ("Essential matrix not yet supported\n");
        break;
    }
    case PERLS_HAUV_CORRESP_F:
    {
        gsl_matrix *F = gsl_matrix_calloc (3, 3);
        vis_modelfit_F_ocv (uv1, uv2, F, &inliers, VIS_MODELFIT_RANSAC);
        gsl_matrix_free (F);
        numGuesses = VIS_EPI_NUM_POSES_FROM_E;
        break;
    }
    default:
        printf ("Unkown correspondence method\n");
        return EXIT_FAILURE;
    }

    gslu_index_free (inliers);

    /* Normalize translation in case decomposition doesn't do it for you */
    for (int i=0; i<numGuesses; i++)
        gslu_vector_normalize ((*tVecs)[i]);

    return numGuesses;
}

gsl_vector *
perls_hauv_tv_register_images (IplImage *img1, IplImage *img2,
                               perls_hauv_tv_camera_params_t *camera,
                               perls_hauv_tv_reg_opts_t *opts)
{
    char *host = "127.0.0.1";
    int port = VIS_SIFTGPU_TCP_PORT;

    /* extract features with siftgpu */
    perllcm_van_feature_t *f1 = NULL, *f2 = NULL;
    f1 = vis_feature_siftgpu (img1, NULL, host, port, 1000);
    f2 = vis_feature_siftgpu (img2, NULL, host, port, 1000);
    if (!f1 || !f2)
        exit (EXIT_FAILURE);

    /* corresponding uv locations as found by ubc_match */
    gsl_matrix *uv1, *uv2;

    /* Pass features into appearance-based matching */
    if (vis_feature_ubc_match_alloc (f1, f2, &uv1, &uv2, opts->sim_thresh, VIS_UBC_SIMSCORE_MIN) == 1)
        return NULL;

    gsl_matrix *uv1Inliers, *uv2Inliers;
    int numInliers = _get_correspondence_alloc (uv1, uv2, &uv1Inliers, &uv2Inliers, opts->corresp_method);
    if (numInliers == 0)
        return NULL;

    /* Get initial guess of rotation, translation */
    gsl_matrix **RMats;
    gsl_vector **tVecs;
    int numGuesses;
    numGuesses = _get_pose_guess_alloc (uv1Inliers, uv2Inliers, &RMats, &tVecs, opts->corresp_method);

    gsl_vector *x21 = NULL, *x21Est = NULL, *params = NULL;
    gsl_matrix *relPoseCov21 = NULL;
    vis_calib_view_gsl_t calib = vis_calib_view_gsl (&camera->calib);
    gsl_matrix *K = &calib.K.matrix;

    /* For each of the possible poses, pass to bundle adjustment framework */
    for (int i=0; i<numGuesses; i++)
    {

        if (x21)
            gsl_vector_free (x21);
        if (x21Est)
            gsl_vector_free (x21Est);
        if (params)
            gsl_vector_free (params);
        if (relPoseCov21)
            gsl_matrix_free (relPoseCov21);

        /* Convert R, t to ssc vector */
        x21 = _pose_from_Rt_alloc (RMats[i], tVecs[i]);
        x21Est = gsl_vector_calloc (5); /* 5-DOF pose */
        relPoseCov21 = gsl_matrix_calloc (5, 5);
        int numPoints = uv1Inliers->size2;
        params = gsl_vector_calloc (10 + 3*numPoints);

        printf ("Running sba on extracted pose %d...\n", i);

        /* run sba */
        int ret_sba = vis_sba_2v_rae_from_model_with_tri (K, x21, uv1Inliers, uv2Inliers,
                      10, 0, 0, 10.0,
                      x21Est, relPoseCov21, NULL,
                      1, NULL, NULL, NULL, NULL);

        /* Some guesses will inevitably fail */
        if (ret_sba == -1 || ret_sba == PERLLCM_VAN_VLINK_T_MSG_TRI_CONST)
            continue;

        break;
    }

    /* clean up */
    switch (opts->corresp_method)
    {
    case PERLS_HAUV_CORRESP_H:
        vis_homog_pose_decomp_free (RMats, tVecs, NULL);
        break;
    case PERLS_HAUV_CORRESP_F:
        vis_epi_horn_decomp_free (RMats, tVecs);
        break;
    }
    perllcm_van_feature_t_destroy (f1);
    perllcm_van_feature_t_destroy (f2);
    gsl_matrix_free (uv1);
    gsl_matrix_free (uv2);
    gsl_matrix_free (uv1Inliers);
    gsl_matrix_free (uv2Inliers);
    gsl_vector_free (x21);
    gsl_matrix_free (relPoseCov21);
    gsl_vector_free (params);

    return x21Est;
}
