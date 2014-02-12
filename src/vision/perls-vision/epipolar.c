#include <stdio.h>
#include <stdlib.h>

#include <gsl/gsl_eigen.h>

#include "perls-math/gsl_util.h"
#include "perls-math/homogenous.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "epipolar.h"

#define VIS_EPI_MIN_ITERATIONS 10
#define VIS_EPI_MAX_ITERATIONS 50

int
vis_epi_F_from_KRT (gsl_matrix *F, const gsl_matrix *K, const gsl_matrix *R, const gsl_vector *t)
{
    // check input format if K 3x3 and X 6 dof vector
    if (K->size1 != 3 || K->size2 != 3 || R->size1 != 3 || R->size2 != 3 || t->size != 3) {
        printf ("Error in vis_epi_F_from_KRT: wrong input format.\n");
        return -1;
    }
    // Given X = [x y z r p h]
    GSLU_MATRIX_VIEW (skewsym_t, 3, 3);
    gslu_matrix_skewsym (&skewsym_t.matrix, t);

    GSLU_MATRIX_VIEW (invK, 3, 3);
    gslu_matrix_inv (&invK.matrix, K);

    //F = transpose(Kinv)*skewsym(t)*R*Kinv;
    GSLU_MATRIX_VIEW (tRK, 3, 3);
    GSLU_MATRIX_VIEW (work, 3, 3);
    gslu_blas_mmm (&tRK.matrix, &skewsym_t.matrix, R, &invK.matrix, &work.matrix);
    gslu_blas_mTm (F, &invK.matrix, &tRK.matrix);

    return 1;
}

int
vis_epi_F_from_KX (gsl_matrix *F, const gsl_matrix *K, const gsl_vector *X)
{
    // check input format if K 3x3 and X 6 dof vector
    if (K->size1 != 3 || K->size2 != 3 || X->size != 6) {
        printf ("Error in vis_epi_F_from_KX: wrong input format.\n");
        return -1;
    }

    // Given X = [x y z r p h]
    gsl_vector_const_view t = gsl_vector_const_subvector (X, 0, 3);
    gsl_vector_const_view rph = gsl_vector_const_subvector (X, 3, 3);

    GSLU_MATRIX_VIEW (R, 3, 3);
    GSLU_MATRIX_VIEW (skewsym_t, 3, 3);
    so3_rotxyz_gsl (&R.matrix, &rph.vector);
    gslu_matrix_skewsym (&skewsym_t.matrix, &t.vector);

    GSLU_MATRIX_VIEW (invK, 3, 3);
    gslu_matrix_inv (&invK.matrix, K);

    //F = transpose(Kinv)*skewsym(t)*R*Kinv;
    GSLU_MATRIX_VIEW (tRK, 3, 3);
    GSLU_MATRIX_VIEW (work, 3, 3);
    gslu_blas_mmm (&tRK.matrix, &skewsym_t.matrix, &R.matrix, &invK.matrix, &work.matrix);
    gslu_blas_mTm (F, &invK.matrix, &tRK.matrix);

    return 1;
}

int
vis_epi_rbt_arun (gsl_matrix *corres_point1, gsl_matrix* corres_point2, 
                  int num_point, int dim,  gsl_matrix* R, gsl_vector* t)
{
    //check the size of matrices
    if (corres_point1->size1 != dim || corres_point1->size2 != num_point) {
        printf ("Dimensions of matrix 1 should be [%d, %d]\n", dim, num_point);
        return -1;
    }

    if (corres_point2->size1 != dim || corres_point2->size2 != num_point) {
        printf ("Dimensions of matrix 1 should be [%d, %d]\n", dim, num_point);
        return -1;
    }

    if (R->size1 != dim || R->size2 != dim) {
        printf ("Dimensions of matrix R should be [%d, %d]\n", dim, dim);
        return -1;
    }

    if (t->size != dim) {
        printf ("Dimensions of vector t should be [%d, 1]\n", dim);
        return -1;
    }
  
    //Find the mean of points 
    // mean_pts1 = mean(corres_points1);
    // mean_pts2 = mean(corres_points2);
    double *mean_pts1 = malloc (sizeof (double) * dim);
    double *mean_pts2 = malloc (sizeof (double) * dim);
    for (int i = 0; i < num_point; i++) {
        for (int j = 0 ; j < dim; j++) {
            mean_pts1[j] = mean_pts1[j] + gsl_matrix_get(corres_point1, j, i);
            mean_pts2[j] = mean_pts2[j] + gsl_matrix_get(corres_point2, j, i);
        }
    }

    for (int i = 0; i < dim; i++) {
        mean_pts1[i] = mean_pts1[i]/num_point;
        mean_pts2[i] = mean_pts2[i]/num_point;
    }

    //Subtract mean from all points
    // Pts1_ms = corres_points1 - mean_pts1;
    // Pts2_ms = corres_points2 - mean_pts2;
    // mean shifted point set
    double **Pts1_ms = malloc (num_point*sizeof(double*));
    double **Pts2_ms = malloc (num_point*sizeof(double*));
    for (int i = 0; i < num_point; i++) {
        Pts1_ms[i] = malloc (dim*sizeof(double));
        Pts2_ms[i] = malloc (dim*sizeof(double));
        for (int j = 0; j < dim ; j++) {
            Pts1_ms[i][j] = gsl_matrix_get(corres_point1, j, i) - mean_pts1[j];
            Pts2_ms[i][j] = gsl_matrix_get(corres_point2, j, i) - mean_pts2[j];
        }
    }

    //Multiply the array of points to get [3x3] matrix
    // H = Pts1_ms * Pts2_ms' { [3xN]*[Nx3] }
    gsl_matrix *H = gsl_matrix_alloc (dim, dim);
    for (int i = 0; i < dim; i++) {
        for (int j = 0; j < dim; j++) {
            double temp_val = 0;
            for (int k = 0; k < num_point; k++)
                temp_val = temp_val + Pts1_ms[i][k]*Pts2_ms[j][k];

            gsl_matrix_set (H, i, j, temp_val);
        }
    }

    //Compute SVD of H
    // [U S V] = svd(H);
    gslu_linalg_SV *USV = gslu_linalg_SV_decomp_econ_alloc (H);
    gsl_matrix *U_transpose = gsl_matrix_alloc (num_point, dim);
    gsl_matrix_transpose_memcpy ( U_transpose, USV->U);

    // R = V*U';
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                    1.0, USV->V, U_transpose, 0.0, R);

    // t = mean_pts1 - R*mean_pts2;
    double R_times_mean_pts2[] = {0, 0, 0};
    gsl_matrix_view gsl_mean_pts2 = gsl_matrix_view_array (mean_pts2, dim, 1);
    gsl_matrix_view gsl_R_times_mean_pts2 = gsl_matrix_view_array (R_times_mean_pts2, dim, 1);
    //R_times_mean_pts2 = R*mean_pts2;
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                    1.0, R, &gsl_mean_pts2.matrix, 0.0, &gsl_R_times_mean_pts2.matrix);

    //t = mean_pts1 - R_times_mean_pts2;
    double temp;
    for (int i = 0; i < dim; i++) {
        temp = mean_pts1[i] - R_times_mean_pts2[i];
        gsl_vector_set (t, i, temp);
    }
    
    //free memory before returning
    //free Pts1_ms, Pts2_ms
    for (int i = 0; i < num_point; i++) {
        free (Pts1_ms[i]);
        free (Pts2_ms[i]);
    }
    free (Pts1_ms);
    free (Pts2_ms);

    //free H, R, USV, U_transpose 
    gsl_matrix_free (H);
    gsl_matrix_free (U_transpose);
    gslu_linalg_SV_free (USV);

    return 1;
}

/*  Given ray1 (3xN, r1_mat), ray2 and rotation matrix R
 *  update r1_prime_mat, r1_prime_r2 for later use
 *  return baseline direction b and residual error E
 */
double 
_lsq_baseline_direction (const gsl_matrix *r1_mat, const gsl_matrix *r2_mat, const gsl_matrix *R,
                         gsl_matrix *r1_prime_mat, gsl_matrix *r1_prime_r2,
                         gsl_vector *b)
{
    // workspaces
    GSLU_MATRIX_VIEW (C, 3,3);         // c*c'
    GSLU_MATRIX_VIEW (work_mat, 3,3);  // for all 3x3 mat operations 
    GSLU_VECTOR_VIEW (work_vec, 3);    // for all vec operations

    size_t N = r1_mat->size2;
    int alpha_valid_count = 0;
    int beta_valid_count = 0;

    // rotate the ray from camera 1 into orientation of camera 2 frame
    gslu_blas_mm (r1_prime_mat, R, r1_mat);
    for (size_t i=0; i<N; i++) {
        gsl_vector_const_view r2 = gsl_matrix_const_column (r2_mat, i); 
        gsl_vector_view r1_prime = gsl_matrix_column (r1_prime_mat, i); 
        
        // c = r1_prime x r2
        gsl_vector_view c = gsl_matrix_column (r1_prime_r2, i); 
        gslu_vector_cross (&c.vector, &r1_prime.vector, &r2.vector);
    }
    gslu_blas_mmT (&C.matrix, r1_prime_r2, r1_prime_r2);

    // minimize E = b'*C*b  with constraint b'*b = 1
    GSLU_VECTOR_VIEW (eval, 3);
    GSLU_MATRIX_VIEW (evec, 3,3);
    gsl_matrix_memcpy (&work_mat.matrix, &C.matrix); // because matrix changes when eig decomposition
    gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc (3);
    gsl_eigen_symmv (&work_mat.matrix, &eval.vector, &evec.matrix, w);
    gsl_eigen_symmv_sort (&eval.vector, &evec.matrix, GSL_EIGEN_SORT_ABS_ASC);
    gsl_matrix_get_col (b, &evec.matrix, 0);
    gslu_blas_mv (&work_vec.vector, &C.matrix, b);
    double E = gslu_vector_dot (b, &work_vec.vector);    // E is the error cost associated with b

    for (size_t i=0; i<N; i++) {
        gsl_vector_const_view r2 = gsl_matrix_const_column (r2_mat, i); 
        gsl_vector_view r1_prime = gsl_matrix_column (r1_prime_mat, i);
        gsl_vector_view c = gsl_matrix_column (r1_prime_r2, i); 
        double c_norm = gslu_vector_norm (&c.vector);

        // magnitudes along rays r1_prime and r2 where closest intersection occurs
        double alpha = gslu_vector_scalar_triple_prod (&c.vector, b, &r2.vector); // alpha = c' * (b x r2)
        alpha /= c_norm*c_norm;
        if (alpha > 0)
            alpha_valid_count ++;

        double beta = gslu_vector_scalar_triple_prod (&c.vector, b, &r1_prime.vector); // gamma = c' * (b x r1_prime)
        beta /= c_norm*c_norm;
        if (beta > 0)
            beta_valid_count ++;
    }
    
    if ( (alpha_valid_count < N/2) || (beta_valid_count < N/2) )
        gsl_vector_scale (b, -1.0);     // wrong sign! fix it.

    // clean up
    gsl_eigen_symmv_free (w);

    // return residual error
    return E;

}

/* Given unnormalized coordinates uv1, uv2 [2xN]
 *       initial guess Ro [3x3]
 *
 * returns relative orientation, R [3x3]
 *         baseline direction, t [3x1]
 *         coplanarity error cost associated with R,t, E
 * This algorithm is based upon:
 *   Horn, B.K.P.  Relative Orientation, MIT A.I. Memo #994 September 1987
 */
double 
vis_epi_relorient_horn (const gsl_matrix *K, const gsl_matrix *uv1, const gsl_matrix *uv2, const gsl_matrix *Ro,
                        gsl_matrix *R, gsl_vector *t,
                        bool verbose)
{
    // check the input arguments
    assert (uv1->size1 == uv2->size1 && uv1->size2 == uv2->size2
            && Ro->size1 == Ro->size2 && Ro->size1 == 3);

    // number of features
    size_t N = uv1->size2;

    // put init guess to R
    gsl_matrix_memcpy (R, Ro);

    // determine the Euclidian ray direction vectors for each image point
    GSLU_MATRIX_VIEW (invK, 3,3);
    gslu_matrix_inv (&invK.matrix, K);
    gsl_matrix *uv1_h = homogenize_alloc (uv1);
    gsl_matrix *uv2_h = homogenize_alloc (uv2);

    gsl_matrix *r1_mat = gslu_blas_mm_alloc (&invK.matrix, uv1_h);
    gsl_matrix *r2_mat = gslu_blas_mm_alloc (&invK.matrix, uv2_h);
    gsl_matrix *r1_prime_mat = gsl_matrix_alloc (r1_mat->size1, r1_mat->size2);
    gsl_matrix *r1_prime_r2 = gsl_matrix_alloc (r1_mat->size1, r1_mat->size2);

    // workspaces
    GSLU_VECTOR_VIEW (b, 3);            // baseline vector
    GSLU_MATRIX_VIEW (work_mat, 3,3);   // for all 3x3 mat operations 
    GSLU_VECTOR_VIEW (work_vec, 3);     // for all vec operations

    // normalize ray1 and ray2
    for (size_t i=0; i<N; i++) {
        // normalize the rays to unit magnitude
        gsl_vector_view r1 = gsl_matrix_column (r1_mat, i);
        double r1_norm = gslu_vector_norm (&r1.vector);
        gsl_vector_scale (&r1.vector, 1.0/r1_norm); // r1_hat

        gsl_vector_view r2 = gsl_matrix_column (r2_mat, i); 
        double r2_norm = gslu_vector_norm (&r2.vector);
        gsl_vector_scale (&r2.vector, 1.0/r2_norm); // r2_hat
    }

    double E = _lsq_baseline_direction (r1_mat, r2_mat, R, r1_prime_mat, r1_prime_r2,
                                        &b.vector);

    // t = -b
    gsl_vector_memcpy (t, &b.vector);
    gsl_vector_scale (t, -1.0);

    int itr = 0;
    GSLU_VECTOR_VIEW (delta_b, 3, {1, 1, 1});
    GSLU_VECTOR_VIEW (delta_w, 3, {1, 1, 1}); // delta_w = delta_b;
    double Eprev = 1e4; 
    E = Eprev-1;

    // workspaces for H and g: for g = Hx equation
    GSLU_MATRIX_VIEW (C, 3,3);          // c*c'
    GSLU_MATRIX_VIEW (B, 3,3);
    GSLU_MATRIX_VIEW (D, 3,3);
    GSLU_MATRIX_VIEW (F, 3,3); 
    GSLU_VECTOR_VIEW (d_bar, 3);
    GSLU_VECTOR_VIEW (c_bar, 3);

    gsl_matrix *r1prime_x_r2_x_b = gsl_matrix_alloc (r1_mat->size1, r1_mat->size2); // where d's are stored
    gsl_vector *t_vec = gsl_vector_alloc (r1_mat->size2);   // where t's are stored

    while (itr < VIS_EPI_MAX_ITERATIONS) {

        // init
        gsl_matrix_set_zero (&C.matrix);
        gsl_matrix_set_zero (&D.matrix);
        gsl_matrix_set_zero (&F.matrix);
        gsl_vector_set_zero (&d_bar.vector);
        gsl_vector_set_zero (&c_bar.vector);
       
        // B
        gslu_blas_vvT (&work_mat.matrix, &b.vector, &b.vector);
        gsl_matrix_scale (&work_mat.matrix, -1.0);
        gsl_matrix_set_identity (&B.matrix);
        gsl_matrix_add (&B.matrix, &work_mat.matrix); // B = eye(3) - b*b'

        gslu_blas_mm (r1_prime_mat, R, r1_mat);
        for (size_t i=0; i<N; i++) {
            gsl_vector_view r2 = gsl_matrix_column (r2_mat, i);
            gsl_vector_view r1_prime = gsl_matrix_column (r1_prime_mat, i); 
    
            // update r1_prime & r1prime_x_r2_x_b
            gsl_vector_view c = gsl_matrix_column (r1_prime_r2, i); 
            gslu_vector_cross (&c.vector, &r1_prime.vector, &r2.vector);
            gsl_vector_view d = gsl_matrix_column (r1prime_x_r2_x_b, i);  
            gslu_vector_triple_prod (&d.vector, &r1_prime.vector, &r2.vector, &b.vector);
        }

        // t
        gslu_blas_vTm (t_vec, r1_prime_r2, &b.vector);

        // c_bar & d_bar
        gslu_blas_mv (&c_bar.vector, r1_prime_r2, t_vec);
        gslu_blas_mv (&d_bar.vector, r1prime_x_r2_x_b, t_vec);

        // C
        gslu_blas_mmT (&C.matrix, r1_prime_r2, r1_prime_r2);

        // D
        gslu_blas_mmT (&D.matrix, r1prime_x_r2_x_b, r1prime_x_r2_x_b);

        // F
        gslu_blas_mmT (&F.matrix, r1_prime_r2, r1prime_x_r2_x_b);

        // solve Hx = g where x = [delta_b; delta_w];
        //  H = [B*C B*F; ...
        //        F'   D];
        GSLU_MATRIX_VIEW (H, 6,6);
        gsl_matrix_view H11 = gsl_matrix_submatrix (&H.matrix, 0,0, 3,3);
        gsl_matrix_view H12 = gsl_matrix_submatrix (&H.matrix, 0,3, 3,3);
        gsl_matrix_view H21 = gsl_matrix_submatrix (&H.matrix, 3,0, 3,3);
        gsl_matrix_view H22 = gsl_matrix_submatrix (&H.matrix, 3,3, 3,3);
        gslu_blas_mm (&H11.matrix, &B.matrix, &C.matrix);
        gslu_blas_mm (&H12.matrix, &B.matrix, &F.matrix);
        gsl_matrix_transpose_memcpy (&H21.matrix, &F.matrix);
        gsl_matrix_memcpy (&H22.matrix, &D.matrix);

        //  g = -[B*c_bar; d_bar];
        GSLU_VECTOR_VIEW (g, 6);
        gsl_vector_view g1 = gsl_vector_subvector (&g.vector, 0,3);
        gsl_vector_view g2 = gsl_vector_subvector (&g.vector, 3,3);
        gslu_blas_mv (&g1.vector, &B.matrix, &c_bar.vector); gsl_vector_scale (&g1.vector, -1.0);
        gsl_vector_memcpy (&g2.vector, &d_bar.vector); gsl_vector_scale (&g2.vector, -1.0);

        // B is singular (b is an eigenvector with zero eigenvalue), thus the first
        // three equations in the system above are not independent.  one of them
        // will have to be removed.  for best numerical accuracy we eliminate the
        // equation with the smallest coefficients and replace it with the linear
        // constraint b'*delta_b = 0
        double min_sum = GSL_POSINF; int mindex = 0;
        for (size_t i=0; i<3; i++) {
            gsl_vector_view H_row = gsl_matrix_row (&H.matrix, i);
            if (gslu_vector_abs_sum (&H_row.vector) < min_sum) {
                min_sum = gslu_vector_abs_sum (&H_row.vector);
                mindex = i;
            }
        }
        GSLU_VECTOR_VIEW (H_row_update, 6, {0,0,0,0,0,0});
        gslu_vector_set_subvector (&H_row_update.vector, 0, &b.vector);
        gsl_vector_view H_row = gsl_matrix_row (&H.matrix, mindex);
        gsl_vector_memcpy (&H_row.vector, &H_row_update.vector);
        gsl_vector_set (&g.vector, mindex, 0);

        //  solve the system of equations for the incremental update
        GSLU_MATRIX_VIEW (Hinv, 6, 6);
        GSLU_VECTOR_VIEW (x, 6);
        gslu_matrix_inv (&Hinv.matrix, &H.matrix);
        gslu_blas_mv (&x.vector, &Hinv.matrix, &g.vector);

        // update error
        Eprev = E; E = 0;
        gslu_vector_get_subvector (&delta_b.vector, &x.vector, 0, 3);
        gslu_vector_get_subvector (&delta_w.vector, &x.vector, 3, 3);

        gsl_vector *N_vec = gsl_vector_alloc (r1_mat->size2);   // work vector
        gslu_blas_vTm (N_vec, r1_prime_r2, &delta_b.vector); 
        gsl_vector_add (t_vec, N_vec);
        gslu_blas_vTm (N_vec, r1prime_x_r2_x_b, &delta_w.vector); 
        gsl_vector_add (t_vec, N_vec);
        E = gslu_vector_dot (t_vec, t_vec);
        gslu_vector_free (N_vec);

        // check termination criteria
        if ((itr > VIS_EPI_MIN_ITERATIONS) && (E > Eprev)) {
            break; // terminate iteration if error has increased
        }
        else { // update R
            gsl_vector_add (&b.vector, &delta_b.vector);
            gslu_vector_normalize (&b.vector);
            GSLU_VECTOR_VIEW (q, 4, {1,0,0,0}); // quaternion
            gsl_vector_memcpy (&work_vec.vector, &delta_w.vector);
            gsl_vector_scale (&work_vec.vector, 0.5);
            gslu_vector_set_subvector (&q.vector, 1, &work_vec.vector);
            gslu_vector_normalize (&q.vector);
            GSLU_MATRIX_VIEW (Q, 3,3);
            so3_quat2rot (q.vector.data, Q.matrix.data);
            gsl_matrix_memcpy (&work_mat.matrix, R);
            gslu_blas_mm (R, &Q.matrix, &work_mat.matrix);
            itr++;
        }

    } // while iteration loop*/

    if (verbose)
        printf("relorient in %d itr with residual error of %g\n", itr, E);

    // recompute the least-squares baseline based upon our final orientation
    E = _lsq_baseline_direction (r1_mat, r2_mat, R, r1_prime_mat, r1_prime_r2,
                                 &b.vector);

    // t = -b
    gsl_vector_memcpy (t, &b.vector);
    gsl_vector_scale (t, -1.0);

    // clean up
    gslu_matrix_free (uv1_h);
    gslu_matrix_free (uv2_h);
    gslu_matrix_free (r1_mat);
    gslu_matrix_free (r2_mat);
    gslu_matrix_free (r1_prime_mat);
    gslu_matrix_free (r1_prime_r2);
    gslu_matrix_free (r1prime_x_r2_x_b);
    gslu_vector_free (t_vec);

    return E;
}

/* This is what we're trying to do (in MATLAB):
   function [Rs ts] = hornDecomp(E)

   bbT = .5*trace(E*E')*eye(3) - E*E';

   norms = diag(sqrt(bbT'*bbT));

   [maxNorm, index] = max(norms);

   b1 = bbT(index,:) / sqrt(bbT(index,index));
   b2 = bbT(index,:) / -sqrt(bbT(index,index));

   B1 = skewsym(b1);
   B2 = skewsym(b2);

   cofE = [cross(E(:,2), E(:,3)) cross(E(:,3), E(:,1)) cross(E(:,1), E(:,2))]';

   R1 = (cofE' - B1*E) / dot(b1,b1);
   R2 = (cofE' - B2*E) / dot(b2,b2);

   Rs        = R1;
   Rs(:,:,2) = R2;

   ts        = b1';
   ts(:,2)   = b2';
*/
static int
_vis_epi_horn_decomp (const gsl_matrix *E, gsl_matrix *R_mats[2], gsl_vector *t_vecs[2])
{
    gsl_matrix *EET = gslu_blas_mmT_alloc (E, E);

    /* compute: bbT = .5*trace(E*E')*eye(3) - E*E' */
    gsl_matrix *bbT = gsl_matrix_calloc (3, 3);
    double c = .5 * gslu_matrix_trace (EET);
    gsl_matrix *cI3 = gsl_matrix_calloc (3, 3);
    gsl_matrix_set_identity (cI3);
    gsl_matrix_scale (cI3, c);
    gsl_matrix_memcpy (bbT, cI3);
    gsl_matrix_sub (bbT, EET);

    /* compute norms = diag(sqrt(bbT'*bbT)) */
    gsl_vector *norms = gsl_vector_calloc (3);
    gsl_matrix *bbTTbbT = gslu_blas_mTm_alloc (bbT, bbT);
    for (int i=0; i<3; i++)
        gsl_vector_set (norms, i, sqrt(gsl_matrix_get (bbTTbbT, i, i)));

    /* find the maximum norm and max index corresponding to that norm */
    double maxNorm = 0;
    int maxNormInd;
    for (int i=0; i<norms->size; i++) {
        if (gsl_vector_get (norms, i) > maxNorm) {
            maxNorm = gsl_vector_get (norms, i);
            maxNormInd = i;
        }
    }

    /* compute: b1 = bbT(index,:) / sqrt(bbT(index,index))
                b2 = bbT(index,:) / -sqrt(bbT(index,index)) */
    gsl_vector *b1 = gsl_vector_alloc (3);
    gsl_vector *b2 = gsl_vector_alloc (3);
    gsl_vector_view b1_view = gsl_matrix_column (bbT, maxNormInd);
    gsl_vector_view b2_view = gsl_matrix_column (bbT, maxNormInd);
    gsl_vector_memcpy (b1, &b1_view.vector);
    gsl_vector_memcpy (b2, &b2_view.vector);
    c = sqrt(gsl_matrix_get (bbT, maxNormInd, maxNormInd));
    gsl_vector_scale (b1, 1/c);
    gsl_vector_scale (b2, -1/c);

    gsl_matrix *B1 = gslu_matrix_skewsym_alloc (b1);
    gsl_matrix *B2 = gslu_matrix_skewsym_alloc (b2);

    /* compute: cofE = [cross(E(:,2), E(:,3)) cross(E(:,3), E(:,1)) cross(E(:,1), E(:,2))]' */
    gsl_vector_const_view E1 = gsl_matrix_const_column (E, 0);
    gsl_vector_const_view E2 = gsl_matrix_const_column (E, 1);
    gsl_vector_const_view E3 = gsl_matrix_const_column (E, 2);
    gsl_matrix *cofE = gsl_matrix_calloc (3, 3);
    gsl_vector_view cofERow1View = gsl_matrix_row (cofE, 0);
    gsl_vector_view cofERow2View = gsl_matrix_row (cofE, 1);
    gsl_vector_view cofERow3View = gsl_matrix_row (cofE, 2);
    gslu_vector_cross (&cofERow1View.vector, &E2.vector, &E3.vector);
    gslu_vector_cross (&cofERow2View.vector, &E3.vector, &E1.vector);
    gslu_vector_cross (&cofERow3View.vector, &E1.vector, &E2.vector);

    /* compute: R1 = (cofE' - B1*E) / dot(b1,b1)
                R2 = (cofE' - B2*E) / dot(b2,b2) */
    gsl_matrix *R1, *R2, *B1E, *B2E, *cofET;
    B1E = gslu_blas_mm_alloc (B1, E);
    B2E = gslu_blas_mm_alloc (B2, E);

    /* dot product */
    double c1 = 0, c2 = 0;
    for (int i=0; i<b1->size; i++) {
        c1 += gsl_vector_get (b1, i) * gsl_vector_get (b1, i);
        c2 += gsl_vector_get (b2, i) * gsl_vector_get (b2, i);
    }

    R1 = R_mats[0];
    R2 = R_mats[1];

    cofET = gslu_matrix_transpose_alloc (cofE);
    gsl_matrix_memcpy (R1, cofET);
    gsl_matrix_memcpy (R2, cofET);
    
    gsl_matrix_sub (R1, B1E);
    gsl_matrix_sub (R2, B2E);

    gsl_matrix_scale (R1, 1/c1);
    gsl_matrix_scale (R2, 1/c2);

    /* Finally, assign the translation vectors */
    gsl_vector *t1, *t2;
    t1 = t_vecs[0];
    t2 = t_vecs[1];
    gsl_vector_memcpy (t1, b1);
    gsl_vector_memcpy (t2, b2);

    /* Clean up */
    gsl_matrix_free (EET);
    gsl_matrix_free (bbT);
    gsl_matrix_free (cI3);
    gsl_matrix_free (bbTTbbT);
    gsl_matrix_free (B1);
    gsl_matrix_free (B2);
    gsl_matrix_free (B1E);
    gsl_matrix_free (B2E);
    gsl_matrix_free (cofE);
    gsl_matrix_free (cofET);
    gsl_vector_free (norms);
    gsl_vector_free (b1);
    gsl_vector_free (b2);
    
    return EXIT_SUCCESS;
}

int
vis_epi_horn_decomp (const gsl_matrix *E, gsl_matrix **R_mats, gsl_vector **t_vecs)
{
    assert (E->size1 == 3 && E->size2 == 3);

    gsl_matrix *negE = gsl_matrix_calloc (E->size1, E->size2);
    gsl_matrix_memcpy (negE, E);
    gsl_matrix_scale (negE, -1);

    /* call _vis_epi_horn_decomp() on E and -E*/
    if (_vis_epi_horn_decomp (E, R_mats, t_vecs) == EXIT_FAILURE)
        return EXIT_FAILURE;
    if (_vis_epi_horn_decomp (negE, R_mats + 2, t_vecs + 2) == EXIT_FAILURE)
        return EXIT_FAILURE;

    /* Finally, this algorithm gives the relative pose Xc1c2, but we typically use
     * Xc2c1 */
    gsl_vector *relPose = gsl_vector_calloc (6);
    gsl_vector *relPoseInv = gsl_vector_calloc (6);
    gsl_vector_view rph = gsl_vector_subvector (relPose, 3, 3);
    gsl_vector_view xyz = gsl_vector_subvector (relPose, 0, 3);
    for (int i=0; i<VIS_EPI_NUM_POSES_FROM_E; i++) {
        so3_rot2rph_gsl (R_mats[i], &rph.vector);
        gsl_vector_memcpy (&xyz.vector, t_vecs[i]);
        ssc_inverse_gsl (relPoseInv, NULL, relPose);
        gsl_vector_view rphInv = gsl_vector_subvector (relPoseInv, 3, 3);
        gsl_vector_view xyzInv = gsl_vector_subvector (relPoseInv, 0, 3);
        so3_rotxyz_gsl (R_mats[i], &rphInv.vector);
        gsl_vector_memcpy (t_vecs[i], &xyzInv.vector);
        gslu_vector_normalize (t_vecs[i]);
    }
    
    /* Clean up */
    gsl_matrix_free (negE);
    gsl_vector_free (relPose);
    gsl_vector_free (relPoseInv);

    return EXIT_SUCCESS;
}
