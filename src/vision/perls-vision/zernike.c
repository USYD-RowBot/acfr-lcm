#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "perls-math/gsl_util.h"

#include <gsl/gsl_sf.h>
#include <gsl/gsl_complex_math.h>

#include "opencv_util.h"
#include "homography.h"
#include "zernike.h"

double
_mul_elem_in_range (int r1, int r2)
{
    if (r1 > r2)
        return 1.0;

    if (r1*r2 < 0)
        return 0.0;

    double mul = r1;
    size_t range = r2-r1;
    for (size_t i=1; i<range+1; i++)
        mul = mul * (r1+i);

    return mul;
}

void
vis_zernike_polynomial (const size_t n, const size_t m,
                        const gsl_vector *rsamp, const gsl_vector *tsamp,
                        vis_zernike_params_t params,
                        gsl_vector_complex *V_nm_col)
{
    size_t nsamp = rsamp->size;    // total number of sample pts

    gsl_vector_view R_nm = gsl_vector_complex_real (V_nm_col);
    gsl_vector_set_zero (&R_nm.vector);

    size_t itr = 0.5*(n-abs(m));

    /*       (n-|m|)/2
     * Rnm =                   (-1)^s (n-s)!
     *       sum      -------------------------------  r^{n-2s}
     *       s=0      s! ((n+|m|)/2-s)! ((n-|m|)/2-s)!
     */

    // NOTE: max value of n in gsl_fs_fact () is 170.
    for (size_t s=0; s<=itr; s++)
    {
        ulong D[3] = {s, (n+abs(m))/2-s, (n-abs(m))/2-s};
        gsl_vector_ulong_view dens = gsl_vector_ulong_view_array (D, 3);

        ulong dmax = gsl_vector_ulong_max (&dens.vector);
        double num = pow(-1.0, s)*_mul_elem_in_range (dmax+1, n-s);

        size_t dmax_idx = gsl_vector_ulong_max_index (&dens.vector);

        double den = 1.0;
        for (size_t i=0; i<3; i++)
            if (i != dmax_idx)
                den = den * gsl_sf_fact (gsl_vector_ulong_get (&dens.vector, i));

        double factor = num/den;

        // evaluate radial function: R_nm = R_nm + factor*rho.^(n-2*s);
        for (size_t i=0; i<nsamp; i++)
        {
            double val = gsl_vector_get (&R_nm.vector, i) + factor * pow (gsl_vector_get (rsamp, i),(n-2*s));
            gsl_vector_set (&R_nm.vector, i, val);
        }
    }

    // complex Zernike basis function evaluated at all (rho,theta)
    // V_nm = R_nm .* exp(j*m*theta);
    // V_nm = ( Zbasis.darea(:) * sqrt((N+1)/pi) ) .* Zbasis.V_nm;
    double sqrt_Nplus1_pi = sqrt ( (n+1)/M_PI );
    for (size_t i=0; i<nsamp; i++)
    {
        //printf("%g + %gi\n", GSL_REAL(z), GSL_IMAG(z));
        double darea = gsl_vector_get (params.darea, i);
        gsl_complex exp_jmtheta= gsl_complex_polar (1.0, m*gsl_vector_get (tsamp, i));
        gsl_complex val = gsl_complex_mul_real (exp_jmtheta, gsl_vector_get (&R_nm.vector, i)*darea*sqrt_Nplus1_pi);

        // we store conjugate of V_nm
        gsl_vector_complex_set (V_nm_col, i, val);
    }
}


vis_zernike_params_t
vis_zernike_init (size_t window_size, size_t r_nsamp, size_t t_nsamp, size_t order)
{

    // store setting into zernike_params
    vis_zernike_params_t params;

    params.window_size = window_size;
    params.r_nsamp = r_nsamp;
    params.t_nsamp = t_nsamp;
    params.moment_order = order;

    double dr = 1.0 / r_nsamp;
    double dt = 2.0 * M_PI / t_nsamp;

    int nsamp = r_nsamp*t_nsamp;    // total number of sample pts
    params.nsamp = nsamp;

    if (nsamp>0)
        params.sampler = gsl_matrix_alloc (2, nsamp);
    else
        params.sampler = NULL;

    // calculate repetition
    size_t repetition = order+1;
    for (size_t i=0; i<order+1; i++)
        repetition += floor(0.5*i);

    params.repetition = repetition;

    // store r samples and theta samples locally
    // for zernike polynomical calculation
    gsl_vector *rsamp = gsl_vector_alloc (nsamp);
    gsl_vector *tsamp = gsl_vector_alloc (nsamp);

    params.darea = gsl_vector_alloc (nsamp);
    params.m_idx = gslu_index_alloc (repetition);

    // computes x samples and y samples and gen sampler
    for (size_t i=0; i<r_nsamp; i++)
    {

        double r = 0.5*dr + dr*i;

        for (size_t j=0; j<t_nsamp; j++)
        {
            double theta = dt*j;
            double darea = r*dr*dt;
            gsl_vector_set (params.darea, i*t_nsamp+j, darea);
            gsl_vector_set (rsamp, i*t_nsamp+j, r);
            gsl_vector_set (tsamp, i*t_nsamp+j, theta);

            // expand pre-sampled polar grid with window size
            double xsamp = window_size*r*cos (theta);
            double ysamp = window_size*r*sin (theta);

            gsl_matrix_set (params.sampler, 0, i*t_nsamp+j, xsamp);
            gsl_matrix_set (params.sampler, 1, i*t_nsamp+j, -ysamp);
        }
    }

    // calculate zernike polynomial
    params.V_nm = gsl_matrix_complex_alloc (nsamp, repetition);
    size_t idx_col = 0;
    for (size_t n=0; n<order+1; n++)
    {
        for (size_t m = fmod (n,2.0); m<=n; m+=2)
        {
            //printf ("idx=%d,n=%d, m=%d\n",(int)idx_col,(int)n,(int)m);
            gsl_vector_complex_view V_nm_col = gsl_matrix_complex_column (params.V_nm, idx_col);
            vis_zernike_polynomial (n, m, rsamp, tsamp, params, &V_nm_col.vector);

            gslu_index_set (params.m_idx, idx_col, m);
            idx_col++;
        }

    }

    // clean up
    gslu_vector_free (rsamp);
    gslu_vector_free (tsamp);

    return params;
}

void
zernike_detrend_polar (gsl_vector *polarpatch, vis_zernike_params_t params)
{
    double polarmean = gslu_vector_dot (polarpatch, params.darea) / M_PI; // for unit circle total area = PI
    gsl_vector_add_constant (polarpatch, -polarmean);
    gsl_vector * workspace = gslu_vector_clone (polarpatch);
    gsl_vector_mul (workspace, params.darea);
    double polarE = gslu_vector_dot (polarpatch, workspace);
    gsl_vector_scale (polarpatch, 1/sqrt(polarE));

    gslu_vector_free (workspace);
}

void
vis_zernike_polarpatch (const CvArr* image, const double u, const double v, gsl_matrix *Hinf,
                        vis_zernike_params_t params, gsl_vector *workspace,
                        gsl_vector_complex *patch_col)
{
    // project via Hinf and warp to N-E coordinate
    double uw, vw, xo, yo;

    vis_homog_single_pt_project (Hinf, u, v, &uw, &vw);

    GSLU_MATRIX_VIEW (invHinf, 3, 3);
    gslu_matrix_inv (&invHinf.matrix, Hinf);

    size_t n_samp = params.nsamp;
    gsl_matrix *sampler = params.sampler;

    for (size_t i=0; i<n_samp; i++)
    {
        double xsamp = gsl_matrix_get (sampler, 0, i) + uw;
        double ysamp = gsl_matrix_get (sampler, 1, i) + vw;

        vis_homog_single_pt_project (&invHinf.matrix, xsamp, ysamp, &xo, &yo);

        // get pixel value at xy_o
        double val = vis_cvu_get_pixel_value_1c (image, xo, yo);

        gsl_vector_set (workspace, i, val);
    }

    zernike_detrend_polar (workspace, params);

    gsl_vector_view patch_col_real = gsl_vector_complex_real (patch_col);
    gsl_vector_memcpy (&patch_col_real.vector, workspace);

}

void
vis_zernike_destroy (vis_zernike_params_t params)
{
    gslu_matrix_free (params.sampler);
    gsl_vector_free (params.darea);
    gslu_index_free (params.m_idx);
    if (params.V_nm)
        gsl_matrix_complex_free (params.V_nm);
}



