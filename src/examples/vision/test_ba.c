#include "perls-math/math.h"
#include "perls-vision/sba.h"

#define DO_ROBUST 0

// HUBER ROBUST COST FUNCTION
double huber(double k, double e)
{
    double w = 1;
    if(fabs(e) >= k)
    {
        w = sqrt(k/fabs(e));
    }
    return w;
}

void f_robust(const double *const e, int n, double * const w)
{

    double k = 1.345; //recomonded tuning parameter from Zhang paper

    for(int i=0 ; i<n ; i++)
    {
        w[i] = huber(k,e[i]);
        //w[i] = 1;
    }


    //calculate the mean as a test and print it
    //double sum = 0.0;
    //for(int i=0 ; i<n ; i++){
    //  sum += fabs(e[i]);
    //}
    //printf("MEAN ABS ERROR = %f\n", sum/n);


    //produces the same result but uses sba style blocking to run faster ...
    /*
      const int blocksize=8, bpwr=3; // 8=2^3
      register int i;
      int j1, j2, j3, j4, j5, j6, j7;
      int blockn;

      blockn = (n>>bpwr)<<bpwr; // (n / blocksize) * blocksize;

      for(i=blockn-1; i>0; i-=blocksize){
                e[i] = huber(k,e[i]);
        j1=i-1; e[j1] = huber(k,e[j1]);
        j2=i-2; e[j2] = huber(k,e[j2]);
        j3=i-3; e[j3] = huber(k,e[j3]);
        j4=i-4; e[j4] = huber(k,e[j4]);
        j5=i-5; e[j5] = huber(k,e[j5]);
        j6=i-6; e[j6] = huber(k,e[j6]);
        j7=i-7; e[j7] = huber(k,e[j7]);
      }
      i=blockn;
      if(i<n){
        switch(n - i){
          case 7 : e[i] = huber(k,e[i]); ++i;
          case 6 : e[i] = huber(k,e[i]); ++i;
          case 5 : e[i] = huber(k,e[i]); ++i;
          case 4 : e[i] = huber(k,e[i]); ++i;
          case 3 : e[i] = huber(k,e[i]); ++i;
          case 2 : e[i] = huber(k,e[i]); ++i;
          case 1 : e[i] = huber(k,e[i]); ++i;
        }
      }
    */

}

int main(int argc, char *argv[])
{
    // run matlab code in /src/featext to generate txt files
    fasttrig_init();

    // optimized pose and covariance
    gsl_vector* p21 = gsl_vector_alloc (5);
    gsl_matrix* S21 = gsl_matrix_alloc (5,5);
    // using robust function
    gsl_vector* p21_robust = gsl_vector_alloc (5);
    gsl_matrix* S21_robust = gsl_matrix_alloc (5,5);

    int ret = 0;
    int ret_robust = 0;
    int verbose = 1;    // 1 will return small verbose
    // 2 will return result (damping and etc) from sba

    gsl_vector* X_c2c1 = gsl_vector_alloc (6);
    gsl_vector* x_5dof = gsl_vector_alloc (5);
    gsl_vector* K_and_opt = gsl_vector_alloc (11);
    gsl_matrix* K_gsl = gsl_matrix_alloc (3,3);

    FILE * fk = fopen ("../share/examples/_test_ba_files/ba_k.txt", "rb");
    if (!fk)
    {
        printf ("Did you run test_ba matlab file?\n");
        return 0;
    }

    gsl_vector_fscanf (fk, K_and_opt);
    fclose (fk);
    FILE * fx21 = fopen ("../share/examples/_test_ba_files/ba_x21.txt", "rb");
    gsl_vector_fscanf (fx21, X_c2c1);
    fclose (fx21);
    FILE * fx_5dof = fopen ("../share/examples/_test_ba_files/x_5dof.txt", "rb");
    gsl_vector_fscanf (fx_5dof, x_5dof);
    fclose (fx_5dof);
    //FILE * fx21 = fopen ("_test_ba_files/ba_p21.txt", "rb");  gsl_vector_fscanf (fx21, X_c2c1);  fclose (fx21);

    int bundle_E = (int) gsl_vector_get (K_and_opt, 9);
    memcpy (K_gsl->data, K_and_opt->data, 9*sizeof(double));
    int n = (int) gsl_vector_get (K_and_opt, 10);

    if (bundle_E)
    {

        printf("\nEssential matrix with %d number of points (verbose = %d) \n------------------------\n\n",n,verbose);
        // in matlab, run testsba(1,1) to generate files
        gsl_matrix* uv1_f = gsl_matrix_alloc(2, n);
        gsl_matrix* uv2_f = gsl_matrix_alloc(2, n);
        gsl_matrix *X1 = gsl_matrix_alloc (3, n);

        FILE * fuv1 = fopen ("../share/examples/_test_ba_files/ba_uv1.txt", "rb");
        gsl_matrix_fscanf (fuv1, uv1_f);
        fclose(fuv1);
        FILE * fuv2 = fopen ("../share/examples/_test_ba_files/ba_uv2.txt", "rb");
        gsl_matrix_fscanf (fuv2, uv2_f);
        fclose(fuv2);
        FILE * fX1 = fopen ("../share/examples/_test_ba_files/ba_X1.txt", "rb");
        gsl_matrix_fscanf (fX1, X1);
        fclose(fX1);

        // using essential matrix based twoview bundle adjustment
        ret = vis_sba_2v_rae(X_c2c1, K_gsl, uv1_f, uv2_f, X1, p21, S21, NULL, verbose, NULL, NULL);  // X_c2c1 = [xyzrph]

        // using essential matrix based twoview bundle adjustment WITH ROBUST COST
#if DO_ROBUST
        ret_robust = vis_sba_2v_rae(X_c2c1, K_gsl, uv1_f, uv2_f, X1, p21_robust, S21_robust, NULL, verbose, NULL, &f_robust);  // X_c2c1 = [xyzrph]
#endif

        // clean up
        gsl_matrix_free(uv1_f);
        gsl_matrix_free(uv2_f);
        gsl_matrix_free(X1);

    }
    else
    {

        printf("\nHomography with %d number of points (verbose = %d) \n------------------------\n\n",n,verbose);
        // in matlab, run testsba(1,0) to generate files
        gsl_matrix* uv1_h = gsl_matrix_alloc(2, n);
        gsl_matrix* uv2_h = gsl_matrix_alloc(2, n);
        gsl_matrix* uv1_proj = gsl_matrix_alloc(2, n);

        FILE * fuv1 = fopen ("../share/examples/_test_ba_files/ba_uv1.txt", "rb");
        gsl_matrix_fscanf (fuv1, uv1_h);
        fclose(fuv1);
        FILE * fuv2 = fopen ("../share/examples/_test_ba_files/ba_uv2.txt", "rb");
        gsl_matrix_fscanf (fuv2, uv2_h);
        fclose(fuv2);
        FILE * fuv1p = fopen ("../share/examples/_test_ba_files/ba_uv1p.txt", "rb");
        gsl_matrix_fscanf (fuv1p, uv1_proj);
        fclose(fuv1p);

        double d_o = 2;                      // dist_o = mean(Z1);
        gsl_vector* n_o = gsl_vector_alloc(3);  // n_o = [0 0 -1]';
        double n_o_data[3] = {0, 0, -1};
        n_o->data = n_o_data;
        // using homography based twoview bundle adjustment
        ret = vis_sba_2v_h(X_c2c1, K_gsl, n_o, d_o, uv1_h, uv2_h, uv1_proj, p21, S21, verbose, NULL);

        // clean up
        gsl_matrix_free(uv1_h);
        gsl_matrix_free(uv2_h);
        gsl_matrix_free(uv1_proj);
    }

    if (ret!=VIS_SBA_ERROR && ret_robust!=VIS_SBA_ERROR)
    {
        GSLU_MATRIX_VIEW (S_check, 5, 5);
        gsl_matrix_memcpy (&S_check.matrix, S21);

        if (gsl_linalg_cholesky_decomp (&S_check.matrix) == GSL_EDOM)
            printf ("Not pos def\n");
        else
            printf ("pos def\n");

        printf("sba results ------------------\n");
        //gsl_vector_scale(p12,RTOD);
        gslu_vector_printf(p21,"p21");
        gslu_matrix_printf(S21,"S21"); //,"%10.4e",CblasNoTrans);

        printf("\nmatlab results ------------------\n");
        gsl_vector* ans_x21 = gsl_vector_alloc (5);
        gsl_matrix* ans_S21 = gsl_matrix_alloc (5,5);
        FILE *fx21_ans = fopen ("../share/examples/_test_ba_files/ans_x21.txt", "rb");
        gsl_vector_fscanf (fx21_ans, ans_x21);
        fclose(fx21_ans);
        FILE *fS21_ans = fopen ("../share/examples/_test_ba_files/ans_S21.txt", "rb");
        gsl_matrix_fscanf (fS21_ans, ans_S21);
        fclose(fS21_ans);
        //gsl_vector_scale(p12,RTOD);
        gslu_vector_printf (ans_x21,"p21");
        gslu_matrix_printf (ans_S21,"S21"); //,"%10.4e",CblasNoTrans);

        // error calculation
        double thresh = 1E-4;
        GSLU_VECTOR_VIEW (err_vec , 5);
        gsl_vector_memcpy (&err_vec.vector, p21);
        gsl_vector_sub (&err_vec.vector, ans_x21);
        double pos_err = gslu_vector_norm (&err_vec.vector);

        // calculate the error between the sba and the truth
        GSLU_VECTOR_VIEW (t_err_vec , 5);
        gsl_vector_memcpy (&t_err_vec.vector, p21);
        gsl_vector_sub (&t_err_vec.vector, x_5dof);
        double t_pos_err = gslu_vector_norm (&t_err_vec.vector);

        GSLU_MATRIX_VIEW (err_mat , 5,5);
        gsl_matrix_memcpy (&err_mat.matrix, S21);
        gsl_matrix_sub (&err_mat.matrix, ans_S21);

        double cov_err = 0;
        for (size_t i=0; i<5; i++)
        {
            gsl_vector_view col = gsl_matrix_column (&err_mat.matrix, i);
            cov_err = cov_err + gslu_vector_norm (&col.vector);
        }

        printf("\n\n-----------------------\n");
        if (pos_err < thresh && cov_err < thresh)
        {
            printf ("Test BA: PASSED (pos err =%g / cov err=%g)!\n",pos_err, cov_err);
        }
        else
        {
            printf ("Test BA: FAILED (pos err =%g / cov err=%g)!\n",pos_err, cov_err);
        }
        printf ("Test BA error vs truth: pos err = %g\n", t_pos_err);
        gsl_vector_free (ans_x21);
        gsl_matrix_free (ans_S21);

#if DO_ROBUST
        gsl_matrix_memcpy (&S_check.matrix, S21_robust);
        if (gsl_linalg_cholesky_decomp (&S_check.matrix) == GSL_EDOM)
            printf ("Robust Not pos def\n");
        else
            printf ("Robust pos def\n");

        printf("\nsba robust results ------------------\n");
        //gsl_vector_scale(p12,RTOD);
        gslu_vector_printf(p21_robust,"p21_robust");
        gslu_matrix_printf(S21_robust,"S21_robust"); //,"%10.4e",CblasNoTrans);

        printf("\nmatlab robust results ------------------\n");
        gsl_vector* ans_x21_robust = gsl_vector_alloc (5);
        gsl_matrix* ans_S21_robust = gsl_matrix_alloc (5,5);
        FILE *fx21_ans_robust = fopen ("../share/examples/_test_ba_files/ans_x21_robust.txt", "rb");
        gsl_vector_fscanf (fx21_ans_robust, ans_x21_robust);
        fclose(fx21_ans_robust);
        FILE *fS21_ans_robust = fopen ("../share/examples/_test_ba_files/ans_S21_robust.txt", "rb");
        gsl_matrix_fscanf (fS21_ans_robust, ans_S21_robust);
        fclose(fS21_ans_robust);
        //gsl_vector_scale(p12,RTOD);
        gslu_vector_printf (ans_x21_robust,"p21_robust");
        gslu_matrix_printf (ans_S21_robust,"S21_robust"); //,"%10.4e",CblasNoTrans);

        // error calculation
        GSLU_VECTOR_VIEW (err_vec_robust , 5);
        gsl_vector_memcpy (&err_vec_robust.vector, p21_robust);
        gsl_vector_sub (&err_vec_robust.vector, ans_x21_robust);
        double pos_err_robust = gslu_vector_norm (&err_vec_robust.vector);

        // calculate the error between the sba and the truth
        GSLU_VECTOR_VIEW (t_err_vec_robust , 5);
        gsl_vector_memcpy (&t_err_vec_robust.vector, p21_robust);
        gsl_vector_sub (&t_err_vec_robust.vector, x_5dof);
        double t_pos_err_robust = gslu_vector_norm (&t_err_vec_robust.vector);
        gslu_vector_printf (p21_robust,"p21_robust ------------------------------");
        gslu_vector_printf (x_5dof,"x_5dof ------------------------------");

        GSLU_MATRIX_VIEW (err_mat_robust , 5,5);
        gsl_matrix_memcpy (&err_mat_robust.matrix, S21_robust);
        gsl_matrix_sub (&err_mat_robust.matrix, ans_S21_robust);

        double cov_err_robust = 0;
        for (size_t i=0; i<5; i++)
        {
            gsl_vector_view col = gsl_matrix_column (&err_mat_robust.matrix, i);
            cov_err_robust = cov_err_robust + gslu_vector_norm (&col.vector);
        }

        if (pos_err_robust < thresh && cov_err_robust < thresh)
        {
            printf ("Test Robust BA: PASSED (pos err =%g / cov err=%g)!\n",pos_err_robust, cov_err_robust);
        }
        else
        {
            printf ("Test Robust BA: FAILED (pos err =%g / cov err=%g)!\n",pos_err_robust, cov_err_robust);
        }
        printf ("Test Robust BA error vs truth: pos err = %g\n", t_pos_err_robust);
        gsl_vector_free (ans_x21_robust);
        gsl_matrix_free (ans_S21_robust);
#endif
    }

    // free memory
    gsl_vector_free (K_and_opt);
    gsl_vector_free (X_c2c1);
    gsl_matrix_free (K_gsl);
    gsl_vector_free (p21);
    gsl_matrix_free (S21);
    gsl_vector_free (p21_robust);
    gsl_matrix_free (S21_robust);

}
