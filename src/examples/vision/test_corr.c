#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "perls-math/math.h"
#include "perls-vision/vision.h"

#define TEST_PCCS 1
#define TEST_MODELFIT 0
#define TEST_BA 0
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
print_pccs_results (gslu_index *sel1, gslu_index *sel2, gsl_vector *info)
{

    printf("\npccs results (n=%d)------------------\n\n", (int) sel1->size);
    gslu_index_printfc (sel1,"sel1","%g", CblasTrans); 
    gslu_index_printfc (sel2,"sel2","%g", CblasTrans); 

    int n = (int) gsl_vector_get (info, 0);
    printf("\nmatlab results (n=%d)------------------\n\n", n);
    gsl_vector* ans_sel1 = gsl_vector_alloc (n);
    gsl_vector* ans_sel2 = gsl_vector_alloc (n);
    FILE *fsel1_ans = fopen ("../share/examples/_test_corr_files/ans_pccs_sel1.txt", "rb");  gsl_vector_fscanf (fsel1_ans, ans_sel1);  fclose(fsel1_ans);
    FILE *fsel2_ans = fopen ("../share/examples/_test_corr_files/ans_pccs_sel2.txt", "rb");  gsl_vector_fscanf (fsel2_ans, ans_sel2);  fclose(fsel2_ans);

    //gsl_vector_scale(p12,RTOD);
    gsl_vector_add_constant (ans_sel1, -1.0);
    gsl_vector_add_constant (ans_sel2, -1.0);
    gslu_vector_printfc (ans_sel1,"sel1","%g", CblasTrans); 
    gslu_vector_printfc (ans_sel2,"sel2","%g", CblasTrans); 

    int thresh = 5;     // number allowed to mismatch
    int mismatch_count = 0;
    for (size_t i =0; i<sel1->size; i++) {
        bool mismatch = 1;
        for (size_t j =0; j<n; j++) {
            if (gslu_index_get (sel1, i) == (int) gsl_vector_get (ans_sel1, j)
                && gslu_index_get (sel2, i) == (int) gsl_vector_get (ans_sel2, j) ) {
                mismatch = 0;
                break;
            }
        }
        if (mismatch) mismatch_count++;
    }

    printf("\n-----------------------\n");
    if (mismatch_count < thresh)
        printf ("Test PCCS: PASSED (mismatch =%d / %d)!\n", mismatch_count, (int) sel1->size);
    else
        printf ("Test PCCS: FAILED (mismatch =%d / %d)!\n", mismatch_count, (int) sel1->size);
    printf("-----------------------\n");

    gsl_vector_free (ans_sel1);
    gsl_vector_free (ans_sel2);
}

int main(int argc, char *argv[])
{

    int ret = 0;
    int n = 1000;

    gsl_vector *x12 = gsl_vector_alloc(6);
    gsl_vector *x21 = gsl_vector_alloc(6);
    gsl_matrix *p12 = gsl_matrix_alloc(6,6);
    gsl_matrix *p21 = gsl_matrix_alloc(6,6);

    gsl_matrix *K_gsl = gsl_matrix_alloc(3,3);
    gsl_matrix *uv1 = gsl_matrix_alloc(2, n);
    gsl_matrix *uv2 = gsl_matrix_alloc(2, n);
    gsl_matrix_float *key1 = gsl_matrix_float_alloc (128, n);
    gsl_matrix_float *key2 = gsl_matrix_float_alloc (128, n);
    gsl_vector *z = gsl_vector_alloc(n);
    gsl_vector *cov_z = gsl_vector_alloc(n);
    gsl_vector_set_all (cov_z, 0.01);

    // to generate these files, do make examples
    FILE *fk = fopen ("../share/examples/_test_corr_files/k.txt", "rb");  gsl_matrix_fscanf (fk, K_gsl); fclose(fk);
    FILE *fuv1 = fopen ("../share/examples/_test_corr_files/uv1.txt", "rb");  gsl_matrix_fscanf (fuv1, uv1);  fclose(fuv1);
    FILE *fuv2 = fopen ("../share/examples/_test_corr_files/uv2.txt", "rb");  gsl_matrix_fscanf (fuv2, uv2);  fclose(fuv2);
    FILE *fkey1 = fopen ("../share/examples/_test_corr_files/key1.txt", "rb");  gsl_matrix_float_fscanf (fkey1, key1);  fclose(fkey1);
    FILE *fkey2 = fopen ("../share/examples/_test_corr_files/key2.txt", "rb");  gsl_matrix_float_fscanf (fkey2, key2);  fclose(fkey2);
    FILE *fz = fopen ("../share/examples/_test_corr_files/z1.txt", "rb");  gsl_vector_fscanf (fz, z);  fclose(fz);
    FILE *fx12 = fopen ("../share/examples/_test_corr_files/x12.txt", "rb");  gsl_vector_fscanf (fx12, x12);  fclose(fx12);
    FILE *fx21 = fopen ("../share/examples/_test_corr_files/x21.txt", "rb");  gsl_vector_fscanf (fx21, x21);  fclose(fx21);
    FILE *fp12 = fopen ("../share/examples/_test_corr_files/p12.txt", "rb");  gsl_matrix_fscanf (fp12, p12);  fclose(fp12);
    FILE *fp21 = fopen ("../share/examples/_test_corr_files/p21.txt", "rb");  gsl_matrix_fscanf (fp21, p21);  fclose(fp21);

    // answer from matlab to be compared
    gsl_vector* ans_info = gsl_vector_alloc (2);
    FILE *finfo = fopen ("../share/examples/_test_corr_files/ans_info.txt", "rb");  gsl_vector_fscanf (finfo, ans_info);  fclose(finfo);

    IplImage *img1 = cvLoadImage ("../share/examples/_test_corr_files/color1.jpg", 1);
    IplImage *img2 = cvLoadImage ("../share/examples/_test_corr_files/color2.jpg", 1);

#if TEST_PCCS || TEST_MODELFIT
    // pccs
    // --------------------------------------------------------- //
    // value to be returned
    gslu_index *sel1 = NULL; 
    gslu_index *sel2 = NULL;
    double simAB_thres = gsl_vector_get (ans_info, 1);

    //tic();
    ret = vis_pccs_corrset_gsl_alloc (uv1, uv2, z, z, cov_z, cov_z, K_gsl, 
                                      x21, p21, key1, key2, &sel1, &sel2, 
                                      simAB_thres, VIS_PCCS_SIMSCORE_MIN); 
    //toc();
    //cvWaitKey(0);
    gsl_matrix *uv1_sel = gslu_matrix_selcol_alloc (uv1, sel1);
    gsl_matrix *uv2_sel = gslu_matrix_selcol_alloc (uv2, sel2);
    //vis_plot_correspondences (img1, img2, sel1, sel2, uv1, uv2, 0, 1, "PUTATIVE CORR");
    //cvWaitKey(0);
    print_pccs_results (sel1, sel2, ans_info);
#endif


    // --------------------------------------------------------- //
    /* NOTE: The rest of the code is not directly related to the pccs
     * but I left it for further debugging purpose (gic, gnuplot with pccs)
     * because there is no matlab file to compare for BA and opencv modelfit part
     */  
    printf ("\n\n * NOTE: No comparison will be conducted for BA\n");

    gsl_matrix *uv1_h = NULL;
    gsl_matrix *uv2_h = NULL;
    gsl_matrix *uv1_f = NULL;
    gsl_matrix *uv2_f = NULL;
#if TEST_MODELFIT
    double lambda1 = 2.0, lambda2 = 4.0;
    // normalized coord
    // --------------------------------------------------------- //
    GSLU_MATRIX_VIEW (invK, 3,3);
    gslu_matrix_inv (&invK.matrix, K_gsl);
    gsl_matrix *UV1 = homogenize_alloc (uv1_sel);
    gsl_matrix *UV2 = homogenize_alloc (uv2_sel);
    gsl_matrix *XY1 = gslu_blas_mm_alloc (&invK.matrix, UV1);
    gsl_matrix *XY2 = gslu_blas_mm_alloc (&invK.matrix, UV2);
    gsl_matrix *xy1 = dehomogenize_alloc (XY1);
    gsl_matrix *xy2 = dehomogenize_alloc (XY2);
    gslu_matrix_free (UV1);
    gslu_matrix_free (UV2);
    gslu_matrix_free (XY1);
    gslu_matrix_free (XY2);
 
    // H estim
    // --------------------------------------------------------- //
    GSLU_MATRIX_VIEW (H,3,3);
    gslu_index *sel_h = NULL;
    int n_in_h = vis_modelfit_H_ransac (xy1, xy2, &H.matrix, &sel_h, NULL);
    printf ("n = %d\n", n_in_h);
    uv1_h = gslu_matrix_selcol_alloc (uv1_sel, sel_h);
    uv2_h = gslu_matrix_selcol_alloc (uv2_sel, sel_h);
    double gic_h = vis_modelfit_gic (&H.matrix, VIS_MODELFIT_H, uv1_sel, uv2_sel, lambda1, lambda2);
    printf ("gic h = %g\n", gic_h);
    //vis_plot_correspondences (img1, img2, sel_h, sel_h, uv1_sel, uv2_sel, 0, 3, "Inliers H"); cvWaitKey(0);

    // E estim
    // --------------------------------------------------------- //
    GSLU_MATRIX_VIEW (E,3,3);

    gsl_matrix *A = gsl_matrix_alloc(10,10);
    FILE *fA = fopen ("../share/examples/_test_corr_files/V.txt", "rb");  gsl_matrix_fscanf (fA, A); fclose(fA);

    gslu_matrix_printf (A,"A");
    gsl_vector *xroots = gsl_vector_alloc (6);
    gsl_vector *yroots = gsl_vector_alloc (6);
    tic();        
    vis_modelfit_weight_6p (A, xroots, yroots);
    toc();
    gslu_matrix_free (A);

    gslu_vector_printf (xroots,"xroots");
    gslu_vector_printf (yroots,"yroots");

    gslu_index *sel_f = NULL;
    double n_in_f = vis_modelfit_E_ransac (xy1, xy2, &E.matrix, &sel_f, NULL);
    uv1_f = gslu_matrix_selcol_alloc (uv1_sel, sel_f);
    uv2_f = gslu_matrix_selcol_alloc (uv2_sel, sel_f);
    double gic_f = vis_modelfit_gic (&E.matrix, VIS_MODELFIT_E, uv1_sel, uv2_sel, lambda1, lambda2);
    printf ("gic f = %g & ", gic_f);
    //vis_plot_correspondences (img1, img2, sel_f, sel_f, uv1_sel, uv2_sel, 0, 3, "Inliers F"); cvWaitKey(0);

    // clean up
    gslu_index_free (sel1);
    gslu_index_free (sel2);
    gslu_matrix_free (uv1_sel);
    gslu_matrix_free (uv2_sel);
    gslu_matrix_free (xy1);
    gslu_matrix_free (xy2);
    gslu_index_free (sel_h);
    gslu_index_free (sel_f);
#endif

#if TEST_BA
    // BA options and returns
    // -------------------------------------------------- //
    int ret_ba = 0;
    int verbose = 0;
    gsl_vector *rel_pose21 = gsl_vector_alloc (5);
    gsl_matrix *rel_pose_cov21 = gsl_matrix_alloc (5,5);

    if (gic_f < gic_h) {// fundamental matrix
        // triangulate
        GSLU_MATRIX_VIEW (R, 3,3);
        gsl_vector_view t = gsl_vector_subvector (x21,0,3);
        gsl_vector_view rph = gsl_vector_subvector (x21,3,3);
        so3_rotxyz (R.matrix.data, rph.vector.data);
        vis_triangulate_t *tri = vis_triangulate_alloc (K_gsl, &R.matrix, &t.vector, uv1_f, uv2_f);

        // enforce triangulation constraint
        double min_dist = 1.0;
        double max_dist = 3.0;
        gslu_index* tri_const_idx = vis_triangulate_constraint_alloc (tri->X1, 2, min_dist, max_dist);
        size_t n_accepted = tri_const_idx->size;
        //printf(" -- rejecting %d points out of %d points\n", n_in_f-n_accepted, n_in_f);

        if (n_accepted == n_in_f) {
            ret_ba = vis_sba_2v_rae(x21, K_gsl, uv1_f, uv2_f, tri->X1, rel_pose21, rel_pose_cov21, verbose, NULL, NULL);  // X_c2c1 = [xyzrph]
        }
        else {
            gsl_matrix* uv1_f_triconst = gslu_matrix_selcol_alloc (uv1_f, tri_const_idx);
            gsl_matrix* uv2_f_triconst = gslu_matrix_selcol_alloc (uv2_f, tri_const_idx);
            gsl_matrix* X1_triconst = gslu_matrix_selcol_alloc (tri->X1, tri_const_idx);
            //vis_plot_correspondences (img1, img2, tri_const_idx, tri_const_idx, uv1_f, uv2_f, 0, 3, "Inliers F"); cvWaitKey(0);

            ret_ba = vis_sba_2v_rae(x21, K_gsl, uv1_f_triconst, uv2_f_triconst, X1_triconst, rel_pose21, rel_pose_cov21, verbose, NULL, NULL);  // X_c2c1 = [xyzrph]

            gslu_matrix_free (uv1_f_triconst);
            gslu_matrix_free (uv2_f_triconst);
            gslu_matrix_free (X1_triconst);
        }
        
        gslu_index_free (tri_const_idx);
        vis_triangulate_free (tri);
    }
    else { // homography
        GSLU_MATRIX_VIEW (invH, 3, 3); 
        gslu_matrix_inv (&invH.matrix, &H.matrix);

        gsl_matrix *uv2_h_h = homogenize_alloc(uv2_h);
        gsl_matrix *uv1p_h = gsl_matrix_alloc (3, n_in_h);    
        gslu_blas_mm (uv1p_h, &invH.matrix, uv2_h_h);
        gsl_matrix *uv1p = dehomogenize_alloc (uv1p_h);

        // run BA
        // -------------------------------------------------- //
        double d_o = 2;                      // dist_o = mean(Z1);
        GSLU_VECTOR_VIEW (n_o, 3, {0, 0, -1});
        ret_ba = vis_sba_2v_h (x21, K_gsl, &n_o.vector, d_o, uv1_h, uv2_h, uv1p, rel_pose21, rel_pose_cov21, verbose, NULL);

        // clean up
        gslu_matrix_free (uv2_h_h);
        gslu_matrix_free (uv1p_h);    
        gslu_matrix_free (uv1p);
    }

    // print out ba results
    // -----------------------------------------------------------
    printf("\nBA results:\n");
    //gsl_vector_scale(p12,RTOD);
    gslu_vector_printf(rel_pose21,"rel_pose21");
    gslu_matrix_printf(rel_pose_cov21,"rel_pose_cov21"); //,"%10.4e",CblasNoTrans);

    FILE *gp = popen("gnuplot -persist","w"); // 'gp' is the pipe descriptor
    if (gp==NULL)
        printf("Error opening pipe to GNU plot. Check if you have it! \n");

    double scale = 1;
    vis_plot_relpose (gp, rel_pose21, scale, NULL);

    gslu_vector_free (rel_pose21);
    gslu_matrix_free (rel_pose_cov21);
#endif

    // clean up
    gslu_vector_free (x12);
    gslu_vector_free (x21);
    gslu_matrix_free (p12);
    gslu_matrix_free (p21);
    gslu_matrix_free (K_gsl);
    gslu_matrix_free (uv1);
    gslu_matrix_free (uv2);
    gslu_matrix_float_free (key1);
    gslu_matrix_float_free (key2);
    gslu_vector_free (z);
    gslu_vector_free (cov_z);
    gslu_vector_free (ans_info);
    cvReleaseImage (&img1);
    cvReleaseImage (&img2);

    gslu_matrix_free (uv1_sel);
    gslu_matrix_free (uv2_sel);
    gslu_matrix_free (uv1_h);
    gslu_matrix_free (uv2_h);
    gslu_matrix_free (uv1_f);
    gslu_matrix_free (uv2_f);

    return 0;
    //return ret_ba;
}
