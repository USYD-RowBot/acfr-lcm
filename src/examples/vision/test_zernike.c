/* This is temporary file to implement Harris feature point
 * will be deleted soon.
 * 2010 Jul 21, Ayoung Kim
 *
 */
#include <stdio.h>
#include <stdlib.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

// for timing
#include <sys/time.h>
#include <time.h>

#include <gsl/gsl_complex_math.h>

#include "perls-vision/zernike.h"
#include "perls-vision/feature.h"
#include "perls-vision/homography.h"
#include "perls-math/math.h"

int tic_sec, tic_usec;

void tic()
{
    struct timeval now;
    // TIC
    gettimeofday(&now, NULL);
    tic_sec = now.tv_sec;
    tic_usec = now.tv_usec;
}

void toc()
{
    // TOC
    float time;
    int toc_sec, toc_usec;
    struct timeval now;

    gettimeofday(&now, NULL);
    toc_sec = now.tv_sec;
    toc_usec = now.tv_usec;
    time = (toc_sec*1000000+toc_usec)-(tic_sec*1000000+tic_usec);
    //printf("time taken = %3.0f usec\n",time);
    printf("%3.4fs\n",time/1000000);

}

double
_matrix_compare (gsl_matrix *mat1, gsl_matrix *mat2)
{
    if (mat1->size1 != mat2->size1 || mat1->size2 != mat2->size2)
    {
        printf ("size should be the same!\n");
        return GSL_POSINF;
    }

    double err = 0.0;

    size_t ncol = mat1->size2;

    gsl_matrix *err_mat = gslu_matrix_clone (mat1);
    gsl_matrix_sub (err_mat, mat2);

    for (size_t i=0; i<ncol; i++)
    {
        gsl_vector_view col = gsl_matrix_column (err_mat, i);
        err = err + gslu_vector_norm (&col.vector);
    }

    gslu_matrix_free (err_mat);
    return err;
}

double
_matrix_complex_compare (gsl_matrix *mat1_real, gsl_matrix *mat1_imag, gsl_matrix_complex *mat2)
{
    if (mat1_real->size1 != mat2->size1 || mat1_real->size2 != mat2->size2)
    {
        printf ("size should be the same!\n");
        return GSL_POSINF;
    }

    double err = 0.0;

    gsl_vector *err_vec = gsl_vector_alloc (mat1_real->size1);

    size_t ncol = mat1_real->size2;
    for (size_t i=0; i<ncol; i++)
    {
        gsl_vector_view col_real1 = gsl_matrix_column (mat1_real, i);
        gsl_vector_view col_imag1 = gsl_matrix_column (mat1_imag, i);

        gsl_vector_complex_view col2 = gsl_matrix_complex_column (mat2, i);
        gsl_vector_view col_real2 = gsl_vector_complex_real (&col2.vector);
        gsl_vector_view col_imag2 = gsl_vector_complex_imag (&col2.vector);

        gsl_vector_memcpy (err_vec, &col_real1.vector);
        gsl_vector_sub (err_vec, &col_real2.vector);
        err = err + gslu_vector_norm (err_vec);

        gsl_vector_memcpy (err_vec, &col_imag1.vector);
        gsl_vector_sub (err_vec, &col_imag2.vector);
        err = err + gslu_vector_norm (err_vec);
    }

    gslu_vector_free (err_vec);
    return err;

}

int main(int argc, char** argv)
{
    double thresh = 1E-3;
    fasttrig_init ();

    // RUN matlab file "test_zernike.m" first
    // that will generate "_test_zernike_files" folder and resulting files

    // read param from matlab result
    gsl_vector *ans_info = gsl_vector_alloc (5);
    FILE *finfo = fopen ("_test_zernike_files/zernike_ans_info.txt", "rb");
    if (!finfo)
    {
        printf ("Did you run matlab test_zernike ? e.g. test_zernike (128, 32)\n");
        return -1;
    }

    gsl_vector_fscanf (finfo, ans_info);
    fclose(finfo);
    size_t t_nsamp = (size_t) gsl_vector_get (ans_info, 0);
    size_t r_nsamp = (size_t) gsl_vector_get (ans_info, 1);
    size_t moment_order = (size_t) gsl_vector_get (ans_info, 2);
    size_t w = (size_t) gsl_vector_get (ans_info, 3);
    size_t nfeat = (size_t) gsl_vector_get (ans_info, 4);

    printf("\n-----------------------\n");
    printf ("params: t_nsamp=%d, r_nsamp=%d, order=%d, w=%d, n=%d\n",(int) t_nsamp, (int) r_nsamp, (int) moment_order, (int) w, (int) nfeat);

    // Load needed data
    IplImage* img = cvLoadImage ("_test_corr_files/zernike_8bit_gray.png", CV_LOAD_IMAGE_UNCHANGED );

    gsl_vector *x12 = gsl_vector_alloc(6);
    gsl_matrix *K = gsl_matrix_alloc(3,3);
    gsl_matrix *uv = gsl_matrix_alloc (2, nfeat);
    FILE *fx12 = fopen ("_test_corr_files/x12.txt", "rb");
    gsl_vector_fscanf (fx12, x12);
    fclose(fx12);
    FILE *fk = fopen ("_test_corr_files/k.txt", "rb");
    gsl_matrix_fscanf (fk, K);
    fclose(fk);
    FILE *fuv = fopen ("_test_corr_files/harris_uv.txt", "rb");
    gsl_matrix_fscanf (fuv, uv);
    fclose(fuv);

    tic();
    vis_zernike_params_t zernike_params = vis_zernike_init (w, r_nsamp, t_nsamp, moment_order);
    toc();

    // check Init step: sampler & Vnm
    size_t nsamp = t_nsamp*r_nsamp;
    gsl_matrix *ans_sampler = gsl_matrix_alloc (2,nsamp);
    FILE *fsampler = fopen ("_test_zernike_files/sampler.txt", "rb");
    gsl_matrix_fscanf (fsampler, ans_sampler);
    fclose(fsampler);

    gsl_matrix *ans_vnm_real = gsl_matrix_alloc (nsamp,zernike_params.repetition);
    gsl_matrix *ans_vnm_imag = gsl_matrix_alloc (nsamp,zernike_params.repetition);

    FILE *fvnm_r = fopen ("_test_zernike_files/vnm_real.txt", "rb");
    gsl_matrix_fscanf (fvnm_r, ans_vnm_real);
    fclose(fvnm_r);
    FILE *fvnm_i = fopen ("_test_zernike_files/vnm_imag.txt", "rb");
    gsl_matrix_fscanf (fvnm_i, ans_vnm_imag);
    fclose(fvnm_i);

    double err1 = _matrix_compare (ans_sampler, zernike_params.sampler);
    double err2 = _matrix_complex_compare (ans_vnm_real, ans_vnm_imag, zernike_params.V_nm);

    double err = err1+err2;

    if (err < thresh)
        printf ("Test init (sampler + zernike polynomial): PASSED (err=%g)!\n", err);
    else
        printf ("Test init (sampler + zernike polynomial): FAILED (err=%g)!\n", err);

    GSLU_MATRIX_VIEW (Hinf, 3, 3);
    GSLU_MATRIX_VIEW (R, 3, 3);
    gsl_vector_const_view rph = gsl_vector_const_subvector (x12, 3, 3);
    so3_rotxyz_gsl (&R.matrix, &rph.vector);
    vis_homog_matrix_infinite (&Hinf.matrix, K, &R.matrix);
    //gslu_matrix_printf (&Hinf.matrix,"&Hinf.matrix");

    gsl_matrix_complex *patch_complexview = gsl_matrix_complex_alloc (nsamp, nfeat);
    gsl_vector *workspace = gsl_vector_alloc (nsamp);

    tic();
    for (size_t i=0; i < nfeat; i++)
    {
        // store keypoint pixel location
        double u = gsl_matrix_get (uv, 0, i);
        double v = gsl_matrix_get (uv, 1, i);
        gsl_vector_complex_view patch_col = gsl_matrix_complex_column (patch_complexview, i);
        vis_zernike_polarpatch (img, u, v, &Hinf.matrix, zernike_params, workspace, &patch_col.vector);
    }
    toc();

    tic();
    // encode patches with normalized Zernike moments
    gsl_matrix_complex *A_nm = gslu_blas_mHm_complex_alloc (zernike_params.V_nm, patch_complexview);
    toc();


    gsl_matrix *ans_anm_real = gsl_matrix_alloc (zernike_params.repetition, nfeat);
    gsl_matrix *ans_anm_imag = gsl_matrix_alloc (zernike_params.repetition, nfeat);

    FILE *fanm_r = fopen ("_test_zernike_files/anm_real.txt", "rb");
    gsl_matrix_fscanf (fanm_r, ans_anm_real);
    fclose(fanm_r);
    FILE *fanm_i = fopen ("_test_zernike_files/anm_imag.txt", "rb");
    gsl_matrix_fscanf (fanm_i, ans_anm_imag);
    fclose(fanm_i);

    err = _matrix_complex_compare (ans_anm_real, ans_anm_imag, A_nm);

    if (err < thresh)
        printf ("Test Anm : PASSED (err=%g)!\n", err);
    else
        printf ("Test Anm : FAILED (err=%g)!\n", err);

    // Clean up
    cvReleaseImage (&img);
    gslu_vector_free (x12);
    gslu_matrix_free (K);
    gslu_matrix_free (uv);
    gslu_vector_free (workspace);
    gsl_matrix_complex_free (A_nm);
    gsl_matrix_complex_free (patch_complexview);

    gslu_vector_free (ans_info);
    gslu_matrix_free (ans_sampler);
    gslu_matrix_free (ans_vnm_real);
    gslu_matrix_free (ans_vnm_imag);
    gslu_matrix_free (ans_anm_real);
    gslu_matrix_free (ans_anm_imag);

    vis_zernike_destroy (zernike_params);

}

