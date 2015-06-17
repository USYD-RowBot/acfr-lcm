#include <stdio.h>
#include <stdlib.h>
#include <limits.h> // PATH_MAX

// external linking req'd
#include <gsl/gsl_math.h> // GSL_POSINF
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_linalg.h>

#include "perls-common/error.h"
#include "perls-math/gsl_util.h"
#include "perls-math/homogenous.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "perls-lcmtypes/perllcm_van_feature_user_depth_t.h"

#include "epipolar.h"
#include "feature.h"
#include "pccs.h"

#define EPS 1E-10

#define SAVE_ELLIPSES_DATA 0                    // save ellipses data for debugging
#define SAVE_ELLIPSES_PATH "../opt/examples/libvision/_test_ellipses_files"

#define USE_FIXED_COV_Z 1                       // fixed covariance of 0.01 (= matlab van)

/* pre calculate parameters needed in point transfer in case of using ukf
 * returns:
 *  simga points (X_vectors), weights, H inf per each sigma points
 *  for 6 dof pose + uv + scene depth prior (z)
 *  L = 6 + 2 + 1  = 9
 *  r = 2*L +1     = total number of sigma points
 */
int
vis_pccs_precalc_single_xfer_params_ukf (const gsl_vector *X_c2c1, const gsl_matrix *P_c2c1,
        const gsl_matrix *K, const gsl_matrix *invK,
        const double cov_z,
        gsl_matrix *X_vectors, gsl_vector *weights, gsl_matrix *H_pre_vectors)
{
    int L = 9;				// = length(mu_x);
    int r = 2*L+1;

    // x_vectors and weights
    double Cov_u1v1 = 2.0;
    float h = sqrt(3);

    // mu_x
    gsl_vector *mu_x = gsl_vector_calloc (L);
    for (int i = 0; i < 6; i++)
        gsl_vector_set (mu_x, i, gsl_vector_get (X_c2c1, i));

    // turning off default gsl error handler (=abort) to check pos def
    gsl_error_handler_t *default_handler = gsl_set_error_handler_off();

    // cholesky factorization
    gsl_matrix *R_xx = gsl_matrix_calloc (L, L);
    gslu_matrix_set_submatrix (R_xx, 0, 0, P_c2c1);
    gsl_matrix_set (R_xx, 6, 6, cov_z);
    gsl_matrix_set (R_xx, 7, 7, Cov_u1v1);
    gsl_matrix_set (R_xx, 8, 8, Cov_u1v1);
    gsl_matrix_scale (R_xx, h*h);

    if (gsl_linalg_cholesky_decomp (R_xx) == GSL_EDOM)
        return 0;
    else
    {
        // The upper triangular part of the input matrix contains L^T = R
        // set lower triangular part 0
        for (int i = 0; i < L; i++)
            for (int j = 0; j < L; j++)
                if (i > j)
                    gsl_matrix_set (R_xx, i, j, 0);
    }
    // restore back to default
    gsl_set_error_handler (default_handler);


    // sigma points, weights
    gsl_vector_set_all (weights, 1.0/(2.0*h*h) );
    gsl_vector_set (weights, 0, (1.0-L/(h*h)) );

    // xvectors
    gsl_matrix *mu_x_stack = gsl_matrix_alloc (L, r);
    for (int j=0; j<r; j++)
        gsl_matrix_set_col (mu_x_stack, j, mu_x);

    gsl_matrix_set_zero (X_vectors);
    gslu_matrix_set_submatrix (X_vectors, 0, 1, R_xx);
    gsl_matrix_scale (R_xx, -1.0);
    gslu_matrix_set_submatrix (X_vectors, 0, L+1, R_xx);
    gsl_matrix_add (X_vectors, mu_x_stack);

    // H_inf per each sigma points
    GSLU_MATRIX_VIEW (R, 3, 3);
    GSLU_MATRIX_VIEW (H, 3, 3);
    GSLU_VECTOR_VIEW (Hcol, 9);

    GSLU_MATRIX_VIEW (work, 3, 3);
    for (int i=0; i<r; i++)
    {
        gsl_vector_const_view x_col = gsl_matrix_const_column (X_vectors, i);
        gsl_vector_const_view rph = gsl_vector_const_subvector (&x_col.vector, 3, 3);
        so3_rotxyz_gsl (&R.matrix, &rph.vector);
        gslu_blas_mmm (&H.matrix, K, &R.matrix, invK, &work.matrix);             // K*R*inv(K)
        gslu_matrix_stack (&Hcol.vector, &H.matrix, CblasTrans);            // h vec = [h11 h12 h13 h21 h22 h23 h31 h32 h33]
        gsl_matrix_set_col (H_pre_vectors, i, &Hcol.vector);
    }

    // clean up
    gslu_vector_free (mu_x);
    gslu_matrix_free (R_xx);
    gslu_matrix_free (mu_x_stack);

    return 1;
}

void
vis_pccs_single_ptxfer (const gsl_vector *X,
                        const gsl_vector *uv,
                        const double z,
                        const gsl_matrix *K,
                        const gsl_vector *h_inf, // h_inf = [h11 h12 h13 h21 h22 h23 h31 h32 h33]
                        gsl_vector *uvp)
{

    /*  for single point xfer case, uv is 2x1 vector u and v elements
        Do two things for efficiency.
        (1) precalculate H = K R K^-1
        (2) calculate uvp analytically

        uvp = Hinf * uv + K t/z
        [uvp_h1]  = [H11 H12 H13] [u] + [K11 K12 K13] [t1/z]
        [uvp_h2]    [H21 H22 H23] [v] + [K21 K22 K23] [t2/z]
        [uvp_h3]    [H31 H32 H33] [1] + [K11 K12 K13] [t3/z]

        up = uvp_h1 / uvp_h3
        vp = uvp_h2 / uvp_h3
    */

    double H11 = gsl_vector_get (h_inf, 0);
    double H12 = gsl_vector_get (h_inf, 1);
    double H13 = gsl_vector_get (h_inf, 2);
    double H21 = gsl_vector_get (h_inf, 3);
    double H22 = gsl_vector_get (h_inf, 4);
    double H23 = gsl_vector_get (h_inf, 5);
    double H31 = gsl_vector_get (h_inf, 6);
    double H32 = gsl_vector_get (h_inf, 7);
    double H33 = gsl_vector_get (h_inf, 8);

    double K11 = gsl_matrix_get (K, 0, 0);
    double K12 = gsl_matrix_get (K, 0, 1);
    double K13 = gsl_matrix_get (K, 0, 2);
    double K21 = gsl_matrix_get (K, 1, 0);
    double K22 = gsl_matrix_get (K, 1, 1);
    double K23 = gsl_matrix_get (K, 1, 2);
    double K31 = gsl_matrix_get (K, 2, 0);
    double K32 = gsl_matrix_get (K, 2, 1);
    double K33 = gsl_matrix_get (K, 2, 2);

    double t1 = gsl_vector_get (X, 0);
    double t2 = gsl_vector_get (X, 1);
    double t3 = gsl_vector_get (X, 2);

    // add sigma point
    double u = gsl_vector_get (uv,0) + gsl_vector_get (X, 7);
    double v = gsl_vector_get (uv,1) + gsl_vector_get (X, 8);
    double z_spt = z + gsl_vector_get (X,6);

    double up = (H11*u+H12*v+H13 + K11*t1/z_spt+K12*t2/z_spt+K13*t3/z_spt) / (H31*u+H32*v+H33 + K31*t1/z_spt+K32*t2/z_spt+K33*t3/z_spt);
    double vp = (H21*u+H22*v+H23 + K21*t1/z_spt+K22*t2/z_spt+K23*t3/z_spt) / (H31*u+H32*v+H33 + K31*t1/z_spt+K32*t2/z_spt+K33*t3/z_spt);

    // return uvp
    gsl_vector_set (uvp, 0, up);
    gsl_vector_set (uvp, 1, vp);
}

/*  Given:
 *   calibration matrix, navigation prior, depth uncertainty
 *   transfer sigma points to get projected image points uv2p
 *   for efficiency, we use precalculated X_vectors, weights, H_pre_vectors
 *  Returns:
 *   projection of image point uv1 onto image 2 = uv2p ( 2x1 vector [u2p, v2p])
 *   Uncertainty related to this transformation = cov_uv2p (2x2 matrix)
 */
void
vis_pccs_relview_single_ptxfer_ukf (const gsl_matrix *K, const gsl_vector *X_c2c1, const gsl_matrix *P_c2c1, const gsl_vector *uv1, const double z1,
                                    const gsl_matrix *X_vectors, const gsl_vector *weights, gsl_matrix *H_pre_vectors,
                                    gsl_vector *uv2p, gsl_matrix *cov_uv2p)
{
    int L = 9;				// = length(mu_x);
    int r = 2*L+1;

    gsl_matrix *Y_matrices = gsl_matrix_calloc (2, r);
    GSLU_VECTOR_VIEW (y_col, 2);                     // conveyer vector for col of Y_matrices y_col = [u; v;]
    GSLU_VECTOR_VIEW (mu_y, 2, {0.0, 0.0});

    for (int i=0; i<r; i++)
    {
        //evaluate model at sigma point
        gsl_vector_const_view h_inf = gsl_matrix_const_column (H_pre_vectors, i);
        gsl_vector_const_view x_col = gsl_matrix_const_column (X_vectors, i);
        vis_pccs_single_ptxfer (&x_col.vector, uv1, z1, K, &h_inf.vector, &y_col.vector);
        gsl_matrix_set_col (Y_matrices, i, &y_col.vector);

        //update weighted mean
        gsl_vector_scale (&y_col.vector, gsl_vector_get (weights, i));
        gsl_vector_add (&mu_y.vector, &y_col.vector);
    }

    // compute transformed covariance and cross-covariance
    double sum11 = 0, sum12 = 0, sum22 = 0;
    double mu_y1 = gsl_vector_get (&mu_y.vector, 0);
    double mu_y2 = gsl_vector_get (&mu_y.vector, 1);

    for (int i=0; i<r; i++)
    {
        double y1 = gsl_matrix_get (Y_matrices, 0, i) - mu_y1;
        double y2 = gsl_matrix_get (Y_matrices, 1, i) - mu_y2;
        sum11 = sum11 + y1*y1 * gsl_vector_get (weights, i);
        sum12 = sum12 + y1*y2 * gsl_vector_get (weights, i);
        sum22 = sum22 + y2*y2 * gsl_vector_get (weights, i);
    }

    // return
    gsl_matrix_set (cov_uv2p, 0, 0, sum11);
    gsl_matrix_set (cov_uv2p, 0, 1, sum12);
    gsl_matrix_set (cov_uv2p, 1, 0, sum12);
    gsl_matrix_set (cov_uv2p, 1, 1, sum22);
    gsl_vector_memcpy (uv2p, &mu_y.vector);

    // clean up
    gslu_matrix_free (Y_matrices);
}

/* lcm to gsl converter
 * read from lcm and conver to gsl format
 */
void
_pccs_read_to_gsl_format (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,
                          const perllcm_pose3d_t p21,
                          gsl_matrix *uv1, gsl_matrix *uv2, gsl_vector *z1, gsl_vector *z2,
                          gsl_vector *cov_z1, gsl_vector *cov_z2,
                          gsl_vector *X_c2c1, gsl_matrix *P_c2c1)
{
    // uv
    // -------------------------------------------------------------- //
    vis_feature_get_uv (f1, uv1);
    vis_feature_get_uv (f2, uv2);

    // z-prior
    // -------------------------------------------------------------- //
    perllcm_van_feature_user_depth_t *sp1 = malloc (sizeof (*sp1));
    perllcm_van_feature_user_depth_t_decode (f1->user, 0, f1->usersize, sp1);
    perllcm_van_feature_user_depth_t *sp2 = malloc (sizeof (*sp2));
    perllcm_van_feature_user_depth_t_decode (f2->user, 0, f2->usersize, sp2);

    if (sp1->npts > 0)   // if scene prior found
    {
        gsl_vector_float_const_view z1f = gsl_vector_float_const_view_array (sp1->mu_Z, sp1->npts);
        GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &z1f.vector, gsl_vector, z1);

        if (USE_FIXED_COV_Z)
            gsl_vector_set_all (cov_z1, 0.01);    // matlab uses fixed 0.01 covariance
        else
        {
            gsl_vector_float_const_view cov_z1f = gsl_vector_float_const_view_array (sp1->Sigma_Z, sp1->npts);
            GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &cov_z1f.vector, gsl_vector, cov_z1);
        }
    }
    else
    {
        gsl_vector_set_all (z1, 1.0);               // any finite value
        gsl_vector_set_all (cov_z1, GSL_POSINF);    // with large covariance
    }

    if (sp2->npts > 0)   // if scene prior found
    {
        gsl_vector_float_const_view z2f = gsl_vector_float_const_view_array (sp2->mu_Z, sp2->npts);
        GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &z2f.vector, gsl_vector, z2);
        if (USE_FIXED_COV_Z)
            gsl_vector_set_all (cov_z2, 0.01);    // matlab uses fixed 0.01 covariance
        else
        {
            gsl_vector_float_const_view cov_z2f = gsl_vector_float_const_view_array (sp2->Sigma_Z, sp2->npts);
            GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &cov_z2f.vector, gsl_vector, cov_z2);
        }
    }
    else
    {
        gsl_vector_set_all (z2, 1.0);               // any finite value
        gsl_vector_set_all (cov_z2, GSL_POSINF);    // with large covariance
    }

    // clean up
    perllcm_van_feature_user_depth_t_destroy (sp1);
    perllcm_van_feature_user_depth_t_destroy (sp2);

    // pose & covariance
    // -------------------------------------------------------------- //
    gsl_vector_const_view mu = gsl_vector_const_view_array (p21.mu, 6);
    gsl_matrix_const_view Sigma = gsl_matrix_const_view_array (p21.Sigma, 6, 6);
    gsl_vector_memcpy (X_c2c1, &mu.vector);
    gsl_matrix_memcpy (P_c2c1, &Sigma.matrix);
}

// -------------------------------------------------------------- //
// modifying for unified pccs
// -------------------------------------------------------------- //
int
vis_pccs_corrset_feat (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,           /* feature_t of image 1 and 2 */
                       const perllcm_pose3d_t p21,                                          /* relative pose 21, containing mu and sigma */
                       const gsl_matrix *K,                                              /* calibration matrix K */
                       double simAB_thres,                                               /* will accept sel if simA > (simAB_thres)*(simAB_thres)*(simB) */
                       double chiSquare2dof,                                             /* search bound if zero, we will set this again */
                       gslu_index *sel1, gslu_index *sel2,                               /* return sel: no allocation, therefore size of n1 and n2 */
                       perllcm_van_plot_debug_t *pd)                                          /* return populated plot_debug_t if needed */
{

    double pccs_simAB_thres = 1.0;    // 1.0, do not use sim thread

    int n1 = f1->npts;
    int n2 = f2->npts;

    int simscore_minmax = VIS_PCCS_SIMSCORE_MIN;
    switch (f1->attrtype)
    {
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF:
        simscore_minmax = VIS_PCCS_SIMSCORE_MIN;
        pccs_simAB_thres = simAB_thres;
        break;
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU:
        simscore_minmax = VIS_PCCS_SIMSCORE_MIN;
        pccs_simAB_thres = simAB_thres;
        break;
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS:
        // this should set to VIS_PCCS_SIMSCORE_MAX when using zernike
        simscore_minmax = VIS_PCCS_SIMSCORE_MIN;
        pccs_simAB_thres = simAB_thres;
        break;
    default:
        ERROR ("unknown attrtype %d", f1->attrtype);
        abort ();
    }

    // read feature and pose into gsl format
    // -------------------------------------------------------------- //
    gsl_matrix *uv1 = gsl_matrix_alloc (2, n1);
    gsl_matrix *uv2 = gsl_matrix_alloc (2, n2);
    gsl_vector *z1 = gsl_vector_alloc (n1);
    gsl_vector *z2 = gsl_vector_alloc (n2);
    gsl_vector *cov_z1 = gsl_vector_alloc (n1);
    gsl_vector *cov_z2 = gsl_vector_alloc (n2);
    GSLU_VECTOR_VIEW (X_c2c1, 6);
    GSLU_MATRIX_VIEW (P_c2c1, 6, 6);
    _pccs_read_to_gsl_format (f1, f2, p21, uv1, uv2, z1, z2, cov_z1, cov_z2,
                              &X_c2c1.vector, &P_c2c1.matrix);

    // run pccs
    // -------------------------------------------------------------- //
    int ret = vis_pccs_corrset_core (uv1, uv2, z1, z2, cov_z1, cov_z2, K,
                                     &X_c2c1.vector, &P_c2c1.matrix, NULL, NULL, f1, f2,
                                     sel1, sel2, pccs_simAB_thres, chiSquare2dof, simscore_minmax,
                                     pd);


    // clean up
    // -------------------------------------------------------------- //
    gslu_matrix_free (uv1);
    gslu_matrix_free (uv2);
    gslu_vector_free (z1);
    gslu_vector_free (z2);
    gslu_vector_free (cov_z1);
    gslu_vector_free (cov_z2);

    return ret;
}

int
vis_pccs_corrset_feat_alloc (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,             /* feature_t of image 1 and 2 */
                             const perllcm_pose3d_t p21,                                            /* relative pose 12, containing mu and sigma */
                             const gsl_matrix *K,                                                /* calibration matrix K */
                             double simAB_thres,                                                 /* will accept sel if simA > (simAB_thres)*(simAB_thres)*(simB) */
                             double chiSquare2dof,                                               /* search bound if zero, we will set this again */
                             gslu_index **sel1, gslu_index **sel2,                               /* return sel: allocate during function call with number of pairs found */
                             perllcm_van_plot_debug_t *pd)                                            /* return populated plot_debug_t if needed */
{
    int n_corr =0, ret = 0;
    int n1_init = f1->npts;
    int n2_init = f2->npts;

    gslu_index *sel1_temp = gslu_index_alloc (n1_init);
    gslu_index *sel2_temp = gslu_index_alloc (n2_init);

    n_corr = vis_pccs_corrset_feat (f1, f2, p21, K, simAB_thres, chiSquare2dof,
                                    sel1_temp, sel2_temp, pd);

    if (n_corr > 0)   // ret = number of correspondence
    {
        ret = n_corr;

        // re-allocate memory for sel1, sel2, uv1 and uv2 with size of n_corr
        (*sel1) = gslu_index_alloc (n_corr);
        (*sel2) = gslu_index_alloc (n_corr);

        // store values to the variable to be returned
        gslu_index_view sel1_sub = gslu_index_subvector (sel1_temp, 0, n_corr);
        gslu_index_view sel2_sub = gslu_index_subvector (sel2_temp, 0, n_corr);
        gslu_index_memcpy ((*sel1), &sel1_sub.vector);
        gslu_index_memcpy ((*sel2), &sel2_sub.vector);
    }
    else
        ret = -1;

    // clean up temp variable
    gslu_index_free (sel1_temp);
    gslu_index_free (sel2_temp);

    return ret;
}

int
vis_pccs_corrset_gsl_alloc (const gsl_matrix *uv1, const gsl_matrix *uv2,               /* input matrices uv1 (2 x n1) and uv2 (2 x n2)*/
                            const gsl_vector *z1, const gsl_vector *z2,                 /* input depth prior in gsl, z1 (n1 x 1) and z2 (n2 x 1)  */
                            const gsl_vector *cov_z1, const gsl_vector *cov_z2,         /* input depth prior uncertainty in gsl */
                            const gsl_matrix *K,                                        /* calibration matrix K */
                            const gsl_vector *X_c2c1,                                   /* relative pose 21 */
                            const gsl_matrix *P_c2c1,                                   /* related covariance to the relative pose */
                            const gsl_matrix_float *key1, const gsl_matrix_float *key2, /* feature key1 and key2 from image 1 and 2 */
                            gslu_index **sel1, gslu_index **sel2,                       /* return sel indeces: allocate during function call with number of pairs found */
                            double simAB_thres,                                         /* will accept sel if simA > (simAB_thres)*(simAB_thres)*(simB) */
                            bool simscore_minmax)                                       /* depending on feature type, we choose max score (harris) or min score (sift) */
{
    int n_corr =0, ret = 0;
    int n1_init = uv1->size2;
    int n2_init = uv2->size2;

    gslu_index *sel1_temp = gslu_index_alloc (n1_init);
    gslu_index *sel2_temp = gslu_index_alloc (n2_init);

    n_corr = vis_pccs_corrset_core (uv1, uv2, z1, z2, cov_z1, cov_z2, K,
                                    X_c2c1, P_c2c1, key1, key2, NULL, NULL,
                                    sel1_temp, sel2_temp, simAB_thres, -1, simscore_minmax,NULL);

    if (n_corr > 0)   // ret = number of correspondence
    {
        ret = n_corr;

        // re-allocate memory for sel1, sel2, uv1 and uv2 with size of n_corr
        (*sel1) = gslu_index_alloc (n_corr);
        (*sel2) = gslu_index_alloc (n_corr);

        // store values to the variable to be returned
        gslu_index_view sel1_sub = gslu_index_subvector (sel1_temp, 0, n_corr);
        gslu_index_view sel2_sub = gslu_index_subvector (sel2_temp, 0, n_corr);
        gslu_index_memcpy ((*sel1), &sel1_sub.vector);
        gslu_index_memcpy ((*sel2), &sel2_sub.vector);
    }
    else
        ret = -1;

    // clean up temp variable
    gslu_index_free (sel1_temp);
    gslu_index_free (sel2_temp);

    return ret;
}

gslu_index *
_vis_pccs_sample_uv_alloc (const gsl_matrix *uv, int img_w, int img_h)
{
    int nsample = 8;

    GSLU_VECTOR_VIEW (xsamp, 8, {0.10,0.50,0.90,0.30,0.70,0.10,0.50,0.90});
    GSLU_VECTOR_VIEW (ysamp, 8, {0.10,0.20,0.10,0.50,0.50,0.90,0.90,0.90});
    gsl_vector_scale (&xsamp.vector, (double) img_w);
    gsl_vector_scale (&ysamp.vector, (double) img_h);
    GSLU_MATRIX_VIEW (uv_sample, 2,8);
    gsl_matrix_set_row (&uv_sample.matrix, 0, &xsamp.vector);
    gsl_matrix_set_row (&uv_sample.matrix, 1, &ysamp.vector);

    gslu_index *sel = gslu_index_alloc (nsample);

    int n = uv->size2;

    for (int i=0; i<nsample; ++i)
    {
        gsl_vector_view uv_sample_i = gsl_matrix_column (&uv_sample.matrix, i);

        double min_dist = GSL_POSINF;
        int min_idx = 0;
        for (int j=0; j<n; j++)
        {
            gsl_vector_const_view uv_j = gsl_matrix_const_column (uv, j);
            double dist = gslu_vector_dist (&uv_j.vector, &uv_sample_i.vector);

            if (dist < min_dist)
            {
                min_dist = dist;
                min_idx = j;
            }

        }
        gslu_index_set (sel, i, min_idx);
    }

    return sel;
}

void
_vis_pccs_sample_uv (gslu_index *sel, const gsl_matrix *uv, int img_w, int img_h)
{
    int nsample = sel->size;

    GSLU_VECTOR_VIEW (xsamp, 8, {0.10,0.50,0.90,0.30,0.70,0.10,0.50,0.90});
    GSLU_VECTOR_VIEW (ysamp, 8, {0.10,0.20,0.10,0.50,0.50,0.90,0.90,0.90});
    gsl_vector_scale (&xsamp.vector, (double) img_w);
    gsl_vector_scale (&ysamp.vector, (double) img_h);
    GSLU_MATRIX_VIEW (uv_sample, 2,8);
    gsl_matrix_set_row (&uv_sample.matrix, 0, &xsamp.vector);
    gsl_matrix_set_row (&uv_sample.matrix, 1, &ysamp.vector);

    int n = uv->size2;

    for (int i=0; i<nsample; ++i)
    {
        gsl_vector_view uv_sample_i = gsl_matrix_column (&uv_sample.matrix, i);

        double min_dist = GSL_POSINF;
        int min_idx = 0;
        for (int j=0; j<n; j++)
        {
            gsl_vector_const_view uv_j = gsl_matrix_const_column (uv, j);
            double dist = gslu_vector_dist (&uv_j.vector, &uv_sample_i.vector);

            if (dist < min_dist)
            {
                min_dist = dist;
                min_idx = j;
            }

        }
        gslu_index_set (sel, i, min_idx);
    }
}

void
gslu_matrix_fprintf (const gsl_matrix *A, const char *filename)
{
    FILE *fid = fopen (filename, "wb");
    gsl_matrix_fprintf (fid, A, "%g");
    fclose (fid);
}

void
_write_to_disk_mat (const gsl_matrix *A, const char *name)
{
    char fullpath[PATH_MAX];
    snprintf (fullpath, sizeof fullpath, "%s/%s.txt", SAVE_ELLIPSES_PATH, name);
    gslu_matrix_fprintf (A, fullpath);
}

void
gslu_vector_fprintf (const gsl_vector *A, const char *filename)
{
    FILE *fid = fopen (filename, "wb");
    gsl_vector_fprintf (fid, A, "%g");
    fclose (fid);
}

void
_write_to_disk_vec (const gsl_vector *A, const char *name)
{
    char fullpath[PATH_MAX];
    snprintf (fullpath, sizeof fullpath, "%s/%s.txt", SAVE_ELLIPSES_PATH, name);
    gslu_vector_fprintf (A, fullpath);
}

/* before finishing pccs function,
 * populates plot_debug_t when it is needed
 */
void
_vis_pccs_prepare_plot_debug (const gsl_matrix *K, perllcm_van_plot_debug_t *pd,
                              const gsl_vector *X_c2c1, const gsl_matrix *P_c2c1, gsl_vector *X_c1c2, gsl_matrix *P_c1c2,
                              const gsl_matrix *uv1, const gsl_matrix *uv2, const gsl_vector *z1, const gsl_vector *z2,
                              gsl_matrix *X_vectors21_pre, gsl_vector *weights21_pre, gsl_matrix *H_vectors21_pre,
                              gsl_matrix *X_vectors12_pre, gsl_vector *weights12_pre, gsl_matrix *H_vectors12_pre)
{
    GSLU_MATRIX_VIEW (Cov_u1pv1p, 2, 2, {0.0, 0.0, 0.0, 0.0});
    GSLU_MATRIX_VIEW (Cov_u2pv2p, 2, 2, {0.0, 0.0, 0.0, 0.0});
    GSLU_VECTOR_VIEW (sigma_col, 4);

    // uv2' F21 uv1 = 0
    GSLU_MATRIX_VIEW (F21, 3,3);
    vis_epi_F_from_KX (&F21.matrix, K, X_c2c1);

    int img_w = pd->img_w;
    int img_h = pd->img_h;
    GSLU_INDEX_VIEW (sample_uv1, 8);
    GSLU_INDEX_VIEW (sample_uv2, 8);
    _vis_pccs_sample_uv (&sample_uv1.vector, uv1, img_w, img_h);
    _vis_pccs_sample_uv (&sample_uv2.vector, uv2, img_w, img_h);

    GSLU_MATRIX_VIEW (uv1_sample, 2,8);
    GSLU_MATRIX_VIEW (uv2_sample, 2,8);
    GSLU_MATRIX_VIEW (uv2p_sample, 2,8);
    GSLU_MATRIX_VIEW (uv1p_sample, 2,8);
    GSLU_MATRIX_VIEW (cov2p_sample, 4,8);
    GSLU_MATRIX_VIEW (cov1p_sample, 4,8);

#if SAVE_ELLIPSES_DATA
    GSLU_VECTOR_VIEW (z1_sample,8);
#endif

    for (int i=0; i<8; ++i)
    {
        int idx_sample1 = gslu_index_get (&sample_uv1.vector, i);
        int idx_sample2 = gslu_index_get (&sample_uv2.vector, i);
        gsl_vector_const_view uv1_i = gsl_matrix_const_column (uv1, idx_sample1);
        gsl_vector_const_view uv2_j = gsl_matrix_const_column (uv2, idx_sample2);
        gsl_vector_view uv2p_i = gsl_matrix_column (&uv2p_sample.matrix, i);
        gsl_vector_view uv1p_j = gsl_matrix_column (&uv1p_sample.matrix, i);

        vis_pccs_relview_single_ptxfer_ukf (K, X_c2c1, P_c2c1, &uv1_i.vector, gsl_vector_get (z1,idx_sample1),
                                            X_vectors21_pre, weights21_pre, H_vectors21_pre, &uv2p_i.vector, &Cov_u2pv2p.matrix);

        vis_pccs_relview_single_ptxfer_ukf (K, X_c1c2, P_c1c2, &uv2_j.vector, gsl_vector_get (z2,idx_sample2),
                                            X_vectors12_pre, weights12_pre, H_vectors12_pre, &uv1p_j.vector, &Cov_u1pv1p.matrix);

        // populate sample uv and uvp
        gsl_matrix_set_col (&uv1_sample.matrix, i, &uv1_i.vector);
        gsl_matrix_set_col (&uv2_sample.matrix, i, &uv2_j.vector);
        gslu_matrix_stack (&sigma_col.vector, &Cov_u2pv2p.matrix, CblasTrans);
        gsl_matrix_set_col (&cov2p_sample.matrix, i, &sigma_col.vector);
        gslu_matrix_stack (&sigma_col.vector, &Cov_u1pv1p.matrix, CblasTrans);
        gsl_matrix_set_col (&cov1p_sample.matrix, i, &sigma_col.vector);

#if SAVE_ELLIPSES_DATA
        gsl_vector_set (&z1_sample.vector,i,gsl_vector_get (z1,idx_sample1));
#endif

    }

    // populate plot_debug_t
    gsl_matrix_view F21_view = gsl_matrix_view_array (pd->F21, 3,3);
    gsl_matrix_memcpy (&F21_view.matrix, &F21.matrix);
    gsl_matrix_float_view uv1_sample_view = gsl_matrix_float_view_array (pd->uv1_sample, 2, 8);
    gsl_matrix_float_view uv2_sample_view = gsl_matrix_float_view_array (pd->uv2_sample, 2, 8);
    gsl_matrix_float_view uv2p_sample_view = gsl_matrix_float_view_array (pd->uv2p_sample, 2, 8);
    gsl_matrix_float_view uv1p_sample_view = gsl_matrix_float_view_array (pd->uv1p_sample, 2, 8);
    gsl_matrix_float_view cov2p_sample_view = gsl_matrix_float_view_array (pd->cov2p_sample, 4, 8);
    gsl_matrix_float_view cov1p_sample_view = gsl_matrix_float_view_array (pd->cov1p_sample, 4, 8);
    GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix, &uv1_sample.matrix, gsl_matrix_float, &uv1_sample_view.matrix);
    GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix, &uv2_sample.matrix, gsl_matrix_float, &uv2_sample_view.matrix);
    GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix, &uv2p_sample.matrix, gsl_matrix_float, &uv2p_sample_view.matrix);
    GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix, &uv1p_sample.matrix, gsl_matrix_float, &uv1p_sample_view.matrix);
    GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix, &cov2p_sample.matrix, gsl_matrix_float, &cov2p_sample_view.matrix);
    GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix, &cov1p_sample.matrix, gsl_matrix_float, &cov1p_sample_view.matrix);

    // in case of debugging, write some information to disk
    // K, x21, p21, uv1, uv2, z1
#if SAVE_ELLIPSES_DATA
    _write_to_disk_mat (K, "K");
    _write_to_disk_vec (X_c2c1, "x21");
    _write_to_disk_mat (P_c2c1, "p21");
    _write_to_disk_mat (&uv1_sample.matrix, "uv1");
    _write_to_disk_mat (&uv2_sample.matrix, "uv2");
    _write_to_disk_vec (&z1_sample.vector, "z1");
    _write_to_disk_mat (&cov2p_sample.matrix, "cov2p");
    gslu_matrix_printf (&cov2p_sample.matrix, "cov2p");
#endif
}

int
vis_pccs_corrset_core (const gsl_matrix *uv1, const gsl_matrix *uv2,                 /* input matrices uv1 (2 x n1) and uv2 (2 x n2)*/
                       const gsl_vector *z1, const gsl_vector *z2,                   /* input depth prior in gsl, z1 (n1 x 1) and z2 (n2 x 1)  */
                       const gsl_vector *cov_z1, const gsl_vector *cov_z2,           /* input depth prior uncertainty in gsl */
                       const gsl_matrix *K,                                          /* calibration matrix K */
                       const gsl_vector *X_c2c1,                                     /* relative pose 21 */
                       const gsl_matrix *P_c2c1,                                     /* related covariance to the relative pose */
                       const gsl_matrix_float *key1, const gsl_matrix_float *key2,   /* feature key1 and key2 from image 1 and 2 */
                       const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,       /* feature sturcture containing keys */
                       gslu_index *sel1, gslu_index *sel2,                           /* return sel: no allocation, therefore size of n1 and n2 */
                       double simAB_thres,                                           /* will accept sel if simA > (simAB_thres)*(simAB_thres)*(simB) */
                       double chiSquare2dof,                                         /* search bound if zero, we will set this again */
                       bool simscore_minmax,                                         /* depending on feature type, we choose max score (harris) or min score (sift) */
                       perllcm_van_plot_debug_t *pd)
{
    bool isfeature_t = 0;
    if (f1 && f2)
        isfeature_t = 1;
    else if (key1 && key2)
        isfeature_t = 0;
    else
    {
        ERROR ("Either feature_t or gsl matrix key should be provided\n");
        abort ();
    }

    int ret = 0;

    int n1 = uv1->size2;
    int n2 = uv2->size2;

    int nomatch = n1 > n2 ? n1 + 1 : n2 + 1;

    // memory alloc and initialization
    // -------------------------------------------------------------- //
    GSLU_VECTOR_VIEW (uv2p_i, 2);
    GSLU_MATRIX_VIEW (Cov_u1pv1p, 2, 2, {0.0, 0.0, 0.0, 0.0});
    GSLU_MATRIX_VIEW (Cov_u2pv2p, 2, 2, {0.0, 0.0, 0.0, 0.0});
    GSLU_MATRIX_VIEW (invCov_u1pv1p, 2, 2, {0.0, 0.0, 0.0, 0.0});
    GSLU_MATRIX_VIEW (invCov_u2pv2p, 2, 2, {0.0, 0.0, 0.0, 0.0});


    // init forward index and backward index
    gslu_index *fwd12_sel = gslu_index_alloc (n1);
    gslu_index_set_all (fwd12_sel, nomatch);
    gslu_index *fwd21_sel = gslu_index_alloc (n2);
    gslu_index_set_all (fwd21_sel, nomatch);
    gsl_vector *fwd21_score = gsl_vector_alloc (n2);
    if (simscore_minmax == VIS_PCCS_SIMSCORE_MIN) // this should check the type of feature
        gsl_vector_set_all (fwd21_score, GSL_POSINF);
    else
        gsl_vector_set_all (fwd21_score, GSL_NEGINF);

    // init loop up table before mapping 2 to 1 (zero = never mapped before)
    gslu_index *lut = gslu_index_alloc (n2);
    gslu_index_set_zero (lut);
    gsl_matrix *uv1p = gsl_matrix_alloc (2, n2);
    gsl_matrix *cov1p = gsl_matrix_alloc (4, n2);

    // temporay variable for storage
    GSLU_VECTOR_VIEW (sigma_col, 4);

    GSLU_MATRIX_VIEW (invK, 3, 3);
    gslu_matrix_inv (&invK.matrix, K);

    // pose & covariance
    GSLU_MATRIX_VIEW (Jminus, 6, 6);
    GSLU_MATRIX_VIEW (work, 6, 6);
    GSLU_VECTOR_VIEW (X_c1c2, 6);
    GSLU_MATRIX_VIEW (P_c1c2, 6, 6);
    ssc_inverse_gsl (&X_c1c2.vector, &Jminus.matrix, X_c2c1);
    gslu_blas_mmmT (&P_c1c2.matrix, &Jminus.matrix, P_c2c1, &Jminus.matrix, &work.matrix); // P_c2c1 = Jminus * P_c1c2 * Jminus';

    // pre calculate
    GSLU_MATRIX_VIEW (H_vectors21_pre, 9, 19);  // L x r
    GSLU_MATRIX_VIEW (X_vectors21_pre, 9, 19);  // L x r
    GSLU_VECTOR_VIEW (weights21_pre, 19);       // r
    GSLU_MATRIX_VIEW (H_vectors12_pre, 9, 19);  // L x r
    GSLU_MATRIX_VIEW (X_vectors12_pre, 9, 19);  // L x r
    GSLU_VECTOR_VIEW (weights12_pre, 19);       // r

    int xfer21_ret = vis_pccs_precalc_single_xfer_params_ukf (X_c2c1, P_c2c1, K, &invK.matrix, gsl_vector_get (cov_z1, 0),
                     &X_vectors21_pre.matrix, &weights21_pre.vector, &H_vectors21_pre.matrix);

    int xfer12_ret = vis_pccs_precalc_single_xfer_params_ukf (&X_c1c2.vector, &P_c1c2.matrix, K, &invK.matrix, gsl_vector_get (cov_z2, 0),
                     &X_vectors12_pre.matrix, &weights12_pre.vector, &H_vectors12_pre.matrix);

    if (!xfer21_ret || !xfer12_ret) // error occured in cholesky decomp
        return 0;                   // no corr. has found

    // set threshold
    // -------------------------------------------------------------- //
    if (chiSquare2dof<0)   // if it is zero (not provided, set value for chiSquare2dof
    {
        double alpha = 1.0-2.0*gsl_cdf_ugaussian_P (-6.0);
        double fudge = 1.2; // size of the ellipsoid
        chiSquare2dof = gsl_cdf_chisq_Pinv (alpha, 2)* fudge * fudge;
    }
    double simscore_thres = simAB_thres*simAB_thres;

    // MAIN LOOP
    // -------------------------------------------------------------- //
    for (int i=0; i<n1; ++i)
    {
        double dist12 = GSL_POSINF, dist21 = GSL_POSINF, key_dist_min = GSL_POSINF, key_dist_2nd_min = GSL_POSINF;

        gsl_vector_const_view uv1_i = gsl_matrix_const_column (uv1, i);
        vis_pccs_relview_single_ptxfer_ukf (K, X_c2c1, P_c2c1, &uv1_i.vector, gsl_vector_get (z1,i),
                                            &X_vectors21_pre.matrix, &weights21_pre.vector, &H_vectors21_pre.matrix, &uv2p_i.vector, &Cov_u2pv2p.matrix);
        gslu_matrix_inv (&invCov_u2pv2p.matrix, &Cov_u2pv2p.matrix);
        //gslu_vector_printf (&uv2p_i.vector,"&uv2p_i.vector - UKF1");
        //gslu_matrix_printf (&Cov_u2pv2p.matrix,"&Cov_u2pv2p.matrix - UKF1");

        for (int j=0; j < n2; ++j)
        {
            gsl_vector_const_view uv2_j = gsl_matrix_const_column (uv2, j);
            dist12 = gslu_vector_mahal_dist (&uv2p_i.vector, &uv2_j.vector, &invCov_u2pv2p.matrix);

            if (dist12*dist12 < chiSquare2dof)
            {
                gsl_vector_view uv1p_j = gsl_matrix_column (uv1p, j);
                gsl_vector_view sigma_col = gsl_matrix_column (cov1p, j);

                if (gslu_index_get (lut, j))     // been mapped before
                {
                    gslu_vector_reshape (&Cov_u1pv1p.matrix, &sigma_col.vector, CblasTrans);
                    gslu_matrix_inv (&invCov_u1pv1p.matrix, &Cov_u1pv1p.matrix);
                }
                else
                {
                    vis_pccs_relview_single_ptxfer_ukf (K, &X_c1c2.vector, &P_c1c2.matrix, &uv2_j.vector, gsl_vector_get (z2,j),
                                                        &X_vectors12_pre.matrix, &weights12_pre.vector, &H_vectors12_pre.matrix, &uv1p_j.vector, &Cov_u1pv1p.matrix);
                    gslu_matrix_inv (&invCov_u1pv1p.matrix, &Cov_u1pv1p.matrix);
                    //gslu_vector_printf (&uv1p_j.vector,"&uv1p_j.vector - UKF");
                    //gslu_matrix_printf (&Cov_u1pv1p.matrix,"&Cov_u1pv1p.matrix - UKF");

                    gslu_index_set (lut, j, 1.0);       // look up table set to be 1, once mapped.
                    gslu_matrix_stack (&sigma_col.vector, &Cov_u1pv1p.matrix, CblasTrans);

                }
                dist21 = gslu_vector_mahal_dist (&uv1p_j.vector, &uv1_i.vector, &invCov_u1pv1p.matrix);

                if (dist21*dist21 < chiSquare2dof)
                {
                    double key_dist = GSL_POSINF;
                    if (isfeature_t)
                    {
                        key_dist = vis_feature_get_simscore (f1, f2, i, j);
                    }
                    else
                    {
                        gsl_vector_float_const_view key1_i = gsl_matrix_float_const_column (key1, i);
                        gsl_vector_float_const_view key2_j = gsl_matrix_float_const_column (key2, j);
                        key_dist = vis_feature_get_simscore_gsl_float (&key1_i.vector, &key2_j.vector);
                    }

                    // find the best score (min/max) & simA and simB test & update fwd12_sel
                    if (simscore_minmax == VIS_PCCS_SIMSCORE_MIN)   // this should check the type of feature
                    {
                        if (key_dist < key_dist_min)
                        {
                            key_dist_2nd_min = key_dist_min;
                            key_dist_min = key_dist;

                            // check simA simB
                            if (key_dist_min < simscore_thres*key_dist_2nd_min)
                                gslu_index_set (fwd12_sel, i, j);
                            else
                                gslu_index_set (fwd12_sel, i, nomatch);

                        }
                        if (key_dist_min < key_dist && key_dist < key_dist_2nd_min)
                        {
                            key_dist_2nd_min = key_dist;
                            if (key_dist_min > simscore_thres*key_dist_2nd_min) // check simA simB
                                gslu_index_set (fwd12_sel, i, nomatch);
                        }
                    }
                    else
                    {
                        // this is case when feature key is harris
                    }

                    // simultaneously.. find the best score (min/max) & update fwd21_sel
                    if (key_dist < gsl_vector_get (fwd21_score, j))
                    {
                        gslu_index_set (fwd21_sel, j, i);
                        gsl_vector_set (fwd21_score, j, key_dist);
                    }
                } // (dist21*dist21 < chiSquare2dof) = Smatrix part
            } // if (dist12*dist12 < chiSquare2dof)
        } // for j in fj
    } // for i in fi
    // -------------------------------------------------------------- //

    // Done. Now check fwd and bwd to populate sel1 sel2
    int idx_sel = 0;
    for (int j=0; j<n2; ++j)
    {
        if (gslu_index_get (fwd21_sel,j) != nomatch)
        {
            if ( gslu_index_get (fwd12_sel, gslu_index_get (fwd21_sel,j)) == j)
            {
                gslu_index_set (sel2, idx_sel, j);
                idx_sel++;
            }
        }
    }
    int n_corr = idx_sel;
    ret = n_corr;

    for (int i=0; i<n_corr; ++i)
    {
        int j = gslu_index_get (fwd21_sel, gslu_index_get (sel2, i));
        gslu_index_set (sel1, i, j);
    }

    // check if we need to prepare plot_debug_t
    // -------------------------------------------------------------- //
    if (pd && pd->plt_ellipses)
    {
        pd->chiSquare2dof = chiSquare2dof;
        _vis_pccs_prepare_plot_debug (K, pd,
                                      X_c2c1, P_c2c1, &X_c1c2.vector, &P_c1c2.matrix, uv1,uv2, z1, z2,
                                      &X_vectors21_pre.matrix, &weights21_pre.vector, &H_vectors21_pre.matrix,
                                      &X_vectors12_pre.matrix, &weights12_pre.vector, &H_vectors12_pre.matrix);
    }

    // clean up
    // -------------------------------------------------------------- //
    gslu_index_free (fwd12_sel);
    gslu_index_free (fwd21_sel);
    gslu_vector_free (fwd21_score);

    gslu_index_free (lut);
    gslu_matrix_free (uv1p);
    gslu_matrix_free (cov1p);

    return ret;
}

