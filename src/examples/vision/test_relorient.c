#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"
#include "perls-math/dm.h"
#include "perls-vision/vision.h"

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

void
print_relorient_results (gsl_matrix *R, gsl_vector *t, double E)
{
    printf("C results------------------\n\n");
    gslu_matrix_printf (R,"R");
    gslu_vector_printf (t,"t");
    printf ("E = %g\n",E);

    printf("\nmatlab results------------------\n\n");
    gsl_matrix *ans_R = gsl_matrix_alloc (3,3);
    gsl_vector *ans_t = gsl_vector_alloc (3);
    gsl_vector *ans_E = gsl_vector_alloc (1);
    FILE *fR = fopen ("../share/examples/_test_corr_files/ans_R_horn.txt", "rb");  gsl_matrix_fscanf (fR, ans_R);  fclose(fR);
    FILE *ft = fopen ("../share/examples/_test_corr_files/ans_t_horn.txt", "rb");  gsl_vector_fscanf (ft, ans_t);  fclose(ft);
    FILE *fE = fopen ("../share/examples/_test_corr_files/ans_E_horn.txt", "rb");  gsl_vector_fscanf (fE, ans_E);  fclose(fE);

    gslu_matrix_printf (ans_R,"R");
    gslu_vector_printf (ans_t,"t");
    printf ("E = %g\n",gsl_vector_get (ans_E, 0));


    // error calculation
    double thresh = 1E-4;
    GSLU_VECTOR_VIEW (err_vec , 3);
    gsl_vector_memcpy (&err_vec.vector, t);
    gsl_vector_sub (&err_vec.vector, ans_t);
    double t_err = gslu_vector_norm (&err_vec.vector);

    GSLU_MATRIX_VIEW (err_mat , 3,3);
    gsl_matrix_memcpy (&err_mat.matrix, R);
    gsl_matrix_sub (&err_mat.matrix, ans_R);
    double R_err = 0;
    for (size_t i=0; i<3; i++) {
        gsl_vector_view col = gsl_matrix_column (&err_mat.matrix, i);
        R_err = R_err + gslu_vector_norm (&col.vector);
    }

    printf("\n-----------------------\n");
        if (t_err < thresh && R_err < thresh && fabs(E-gsl_vector_get (ans_E, 0)) < thresh)
            printf ("Test1: REL ORIENT: PASSED (t err =%g / R err=%g / E err =%g)!\n",t_err, R_err, fabs(E-gsl_vector_get (ans_E, 0)));
        else
            printf ("Test1: REL ORIENT: FAILED (t err =%g / R err=%g / E err =%g)!\n",t_err, R_err, fabs(E-gsl_vector_get (ans_E, 0)));

    // clean up
    gsl_matrix_free (ans_R);
    gsl_vector_free (ans_t);
    gsl_vector_free (ans_E);

}
int main(int argc, char *argv[])
{
    int n = 1000;
    bool verbose = 0;

    gsl_matrix *K = gsl_matrix_alloc(3,3);
    gsl_vector *x12 = gsl_vector_alloc(6);
    gsl_vector *x21 = gsl_vector_alloc(6);
    gsl_matrix *p12 = gsl_matrix_alloc(6,6);
    gsl_matrix *p21 = gsl_matrix_alloc(6,6);
    gsl_matrix *uv1 = gsl_matrix_alloc(2, n);
    gsl_matrix *uv2 = gsl_matrix_alloc(2, n);

    // to generate these files, do make examples
    FILE *fk = fopen ("../share/examples/_test_corr_files/k.txt", "rb");  gsl_matrix_fscanf (fk, K); fclose(fk);
    FILE *fuv1 = fopen ("../share/examples/_test_corr_files/uv1.txt", "rb");  gsl_matrix_fscanf (fuv1, uv1);  fclose(fuv1);
    FILE *fuv2 = fopen ("../share/examples/_test_corr_files/uv2.txt", "rb");  gsl_matrix_fscanf (fuv2, uv2);  fclose(fuv2);
    FILE *fx12 = fopen ("../share/examples/_test_corr_files/x12.txt", "rb");  gsl_vector_fscanf (fx12, x12);  fclose(fx12);
    FILE *fx21 = fopen ("../share/examples/_test_corr_files/x21.txt", "rb");  gsl_vector_fscanf (fx21, x21);  fclose(fx21);
    FILE *fp12 = fopen ("../share/examples/_test_corr_files/p12.txt", "rb");  gsl_matrix_fscanf (fp12, p12);  fclose(fp12);
    FILE *fp21 = fopen ("../share/examples/_test_corr_files/p21.txt", "rb");  gsl_matrix_fscanf (fp21, p21);  fclose(fp21);


    // test1 : vis_tv_relorient_horn
    // -------------------------------------------------- //
    GSLU_MATRIX_VIEW (Ro, 3,3);
    GSLU_MATRIX_VIEW (R, 3,3);
    GSLU_VECTOR_VIEW (t, 3);

    gsl_vector_const_view rph = gsl_vector_const_subvector (x21,3,3);
    so3_rotxyz (Ro.matrix.data, rph.vector.data);
    //tic();
    double E = vis_epi_relorient_horn (K, uv1, uv2, &Ro.matrix,
                                       &R.matrix, &t.vector, verbose);
    //toc();

    printf ("\ntest1: \n");
    print_relorient_results (&R.matrix, &t.vector, E);

    // test2 : vis_tv_use_navprior
    // -------------------------------------------------- //
    int nsamples = 1;
    //tic();
    GSLU_VECTOR_VIEW (x21_prior, 6, {0,0,0,0,0,0});
    gsl_vector *min_x21 = vis_tv_use_navprior (K, uv1, uv2, 
                                               x21, p21, &x21_prior.vector,
                                               nsamples, verbose);
    //toc();
    printf ("\n");
    gslu_vector_printfc (x21, "x21 given", NULL, CblasTrans);
    gslu_vector_printfc (min_x21, "x21 new", NULL, CblasTrans);
    printf("-----------------------\n");
    gslu_vector_free (min_x21);


    // test3 : vis_tv_mdist_check_error
    // -------------------------------------------------- //
    double mdist = GSL_POSINF;
    gsl_vector *x21_5dof = gsl_vector_alloc (5);
    gsl_matrix *p21_5dof = gsl_matrix_alloc (5,5);
    GSLU_MATRIX_VIEW (Jtemp, 5, 6);

    //tic();
    dm_trans2dm_pose_cov (x21, p21, x21_5dof, p21_5dof, &Jtemp.matrix);
    vis_tv_mdist_check_error (x12, p12, x21_5dof, p21_5dof, 0, &mdist);
    //toc();

    gsl_vector *ans_mdist = gsl_vector_alloc (1);
    FILE *fmd = fopen ("../share/examples/_test_corr_files/ans_mdist.txt", "rb");  gsl_vector_fscanf (fmd, ans_mdist);  fclose(fmd);

    double md_err = fabs (mdist - gsl_vector_get (ans_mdist, 0));
    printf("\n-----------------------\n");
    if ( md_err  < 1E-6)
        printf ("Test2: MAHAL DIST: PASSED (err =%g) \n",md_err);
    else
        printf ("Test2: MAHAL DIST: FAILED (err =%g) \n",md_err);
    printf("-----------------------\n");

    // clean up
    gslu_matrix_free (K);
    gslu_vector_free (x12);
    gslu_vector_free (x21);
    gslu_matrix_free (p12);
    gslu_matrix_free (p21);
    gslu_matrix_free (uv1);
    gslu_matrix_free (uv2);

    gslu_vector_free (ans_mdist);
    gslu_vector_free (x21_5dof);
    gslu_matrix_free (p21_5dof);

    return 0;
}
