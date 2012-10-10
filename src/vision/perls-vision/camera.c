#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "perls-math/homogenous.h"
#include "perls-math/gsl_util.h"
#include "perls-math/ssc.h"
#include "perls-math/so3.h"

#include "camera.h"
#include "distortion.h"


void
vis_camera_matrix (gsl_matrix *P, const gsl_matrix *K, const gsl_matrix *R, const gsl_vector *t)
{
    assert (P->size1 == 3 && P->size2 == 4);

    GSLU_MATRIX_VIEW (RT, 3, 4);

    if (R)
        gslu_matrix_set_submatrix (&RT.matrix, 0, 0, R);
    else {
        GSLU_MATRIX_VIEW (I, 3, 3, {1, 0, 0,   0, 1, 0,   0, 0, 1});
        gslu_matrix_set_submatrix (&RT.matrix, 0, 0, &I.matrix);
    }

    if (t)
        gsl_matrix_set_col (&RT.matrix, 3, t);
    else {
        GSLU_VECTOR_VIEW (t0, 3, {0, 0, 0});
        gsl_matrix_set_col (&RT.matrix, 3, &t0.vector);
    }

    if (K)
        gslu_blas_mm (P, K, &RT.matrix);
    else {
        GSLU_MATRIX_VIEW (I, 3, 3, {1, 0, 0,   0, 1, 0,   0, 0, 1});
        gslu_blas_mm (P, &I.matrix, &RT.matrix);
    }
}

void
vis_camera_matrix_from_pose (gsl_matrix *P, const gsl_matrix *K, const double x_lc[6])
{
    assert (P->size1 == 3 && P->size2 == 4 &&
            K->size1 == 3 && K->size2 == 3);            

    GSLU_MATRIX_VIEW (R_lc, 3, 3);
    so3_rotxyz (R_lc.data, SSC_RPH(x_lc));

    GSLU_MATRIX_VIEW (R_cl, 3, 3);
    gsl_matrix_transpose_memcpy (&R_cl.matrix, &R_lc.matrix);

    GSLU_VECTOR_VIEW (t_cl_l, 3, {-x_lc[SSC_DOF_X], -x_lc[SSC_DOF_Y], -x_lc[SSC_DOF_Z]});

    GSLU_VECTOR_VIEW (t_cl_c, 3);
    gslu_blas_mv (&t_cl_c.vector, &R_cl.matrix, &t_cl_l.vector);

    vis_camera_matrix (P, K, &R_cl.matrix, &t_cl_c.vector);
}

void
vis_camera_project (const gsl_matrix *P, const gsl_matrix *X, gsl_matrix *uv)
{
    assert (P->size1 == 3 && P->size2 == 4 && 
            (X->size1 == 3 || X->size1 == 4) &&
            (uv->size1 == 2 || uv->size1 == 3) &&
            X->size2 == uv->size2);

    gsl_matrix *X_h = NULL;
    if (X->size1 == 3)
        X_h = homogenize_alloc (X);
    else
        X_h = (gsl_matrix *) X;

    gsl_matrix *uv_h = NULL;
    if (uv->size1 == 2)
        uv_h = homogenize_alloc (uv);
    else
        uv_h = uv;

    gslu_blas_mm (uv_h, P, X_h);

    if (uv->size1 == 2) {
        dehomogenize (uv_h, uv);
        gsl_matrix_free (uv_h);
    }
    
    if (X->size1 == 3)
        gsl_matrix_free (X_h);
}

void
vis_camera_project_nonlin (const gsl_matrix *P, const gsl_matrix *K, const gsl_vector *distCoeffs, const gsl_matrix *X, gsl_matrix *uv)
{
    int is_homogen = (uv->size1 == 3);

    gsl_matrix *uv_homogen = NULL;
    gsl_matrix *uv_nonhomogen = NULL;

    //If input points X are nonhomogeneous, allocate homogeneous
    if (!is_homogen) {
        uv_homogen = homogenize_alloc (uv);
        uv_nonhomogen = uv;
    }
    else {
        uv_homogen = (gsl_matrix *) uv;
        uv_nonhomogen = dehomogenize_alloc (uv);
    }

    //Project to the image plane (assuming zero lens distortion)
    vis_camera_project (P, X, uv_nonhomogen);

    //Now apply forward distortion to the undistorted points (which must be 2xN)
    gsl_matrix *uv_nonhomogen_dist = gsl_matrix_calloc(uv_nonhomogen->size1, uv_nonhomogen->size2);
    vis_distort_pts_radial (uv_nonhomogen, uv_nonhomogen_dist, K, distCoeffs);

    //assign to uv, cleaning up appropriately
    if (!is_homogen) {
        gsl_matrix_free (uv_homogen);
        gsl_matrix_memcpy (uv, uv_nonhomogen_dist);
    } else {
        gsl_matrix_free (uv_nonhomogen);
        homogenize (uv_nonhomogen_dist, uv);
    }

    //clean up
    gsl_matrix_free (uv_nonhomogen_dist);

}

/**
 * J -> Jacobian w.r.t  extended state .i.e [mu;X;Y;Z]. 2x9 matrix
 * mu -> mean state (6x1) vector
 * K -> intrinsic camera parameters
 * XYZ -> 3D point
 */
void
vis_camera_projection_jacobian(double J_mu[2*6], double J_XYZ[2*6], double mu[6], double K[3*3], double XYZ[3]) {

    double fx= K[0*3+0];
    double fy= K[1*3+1];
    double a = K[0*3+2];
    double b = K[1*3+2];
    
    double x = mu[0], y = mu[1], z = mu[2];
    double X = XYZ[0], Y = XYZ[1], Z = XYZ[2];
    
    // calculate trig once
    double cr = cos(mu[3]); //roll
    double sr = sin(mu[3]);
    double cp = cos(mu[4]); //pitch
    double sp = sin(mu[4]);
    double ch = cos(mu[5]); //heading
    double sh = sin(mu[5]);
    
    // generated symbolicly in MATLAB. Is this faster than a numerical jacobian? Maybe not.
    
    if (J_mu != NULL) {
        J_mu[0] = -(a*(sh*sr+ch*cr*sp)+fx*ch*cp)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)-((sh*sr+ch*cr*sp)*(a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh)))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[1] = (a*(ch*sr-cr*sh*sp)-fx*cp*sh)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)+((ch*sr-cr*sh*sp)*(a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh)))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[2] = (fx*sp-a*cp*cr)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)-(cp*cr*(a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh)))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[3] = (a*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)+X*a*(cr*sh-ch*sp*sr)-Y*a*(ch*cr+sh*sp*sr)-Z*a*cp*sr)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)+((a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh))*(X*(cr*sh-ch*sp*sr)-Y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+y*(ch*cr+sh*sp*sr)-Z*cp*sr+z*cp*sr))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[4] = -(Z*(fx*cp+a*cr*sp)+a*(x*ch*cp*cr-z*cr*sp+y*cp*cr*sh)+X*(fx*ch*sp-a*ch*cp*cr)+Y*(fx*sh*sp-a*cp*cr*sh)-fx*(z*cp+x*ch*sp+y*sh*sp))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)-((a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh))*(Z*cr*sp-z*cr*sp-X*ch*cp*cr-Y*cp*cr*sh+x*ch*cp*cr+y*cp*cr*sh))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[5] = ((X*(ch*sr-cr*sh*sp)+Y*(sh*sr+ch*cr*sp)-x*(ch*sr-cr*sh*sp)-y*(sh*sr+ch*cr*sp))*(a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh)))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2)-(fx*(y*ch*cp-x*cp*sh)+a*(x*(ch*sr-cr*sh*sp)+y*(sh*sr+ch*cr*sp))-Y*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)-X*(a*(ch*sr-cr*sh*sp)-fx*cp*sh))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr);
            
        J_mu[6] = -(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)-((sh*sr+ch*cr*sp)*(b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[7] = (b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)+((ch*sr-cr*sh*sp)*(b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[8] = -(b*cp*cr+fy*cp*sr)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)-(cp*cr*(b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[9] = (Z*(fy*cp*cr-b*cp*sr)-fy*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+b*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)+X*(b*(cr*sh-ch*sp*sr)+fy*(sh*sr+ch*cr*sp))-Y*(b*(ch*cr+sh*sp*sr)+fy*(ch*sr-cr*sh*sp)))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)+((b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr)))*(X*(cr*sh-ch*sp*sr)-Y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+y*(ch*cr+sh*sp*sr)-Z*cp*sr+z*cp*sr))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr,2);
        J_mu[10] = -(Z*(b*cr*sp+fy*sp*sr)+b*(x*ch*cp*cr-z*cr*sp+y*cp*cr*sh)+fy*(x*ch*cp*sr-z*sp*sr+y*cp*sh*sr)-X*(b*ch*cp*cr+fy*ch*cp*sr)-Y*(b*cp*cr*sh+fy*cp*sh*sr))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)-((b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr)))*(Z*cr*sp-z*cr*sp-X*ch*cp*cr-Y*cp*cr*sh+x*ch*cp*cr+y*cp*cr*sh))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_mu[11] = (X*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))+Y*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))-b*(x*(ch*sr-cr*sh*sp)+y*(sh*sr+ch*cr*sp))+fy*(x*(ch*cr+sh*sp*sr)+y*(cr*sh-ch*sp*sr)))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)+((X*(ch*sr-cr*sh*sp)+Y*(sh*sr+ch*cr*sp)-x*(ch*sr-cr*sh*sp)-y*(sh*sr+ch*cr*sp))*(b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
    }
    
    if (J_XYZ != NULL) {
        J_XYZ[0] = (a*(sh*sr+ch*cr*sp)+fx*ch*cp)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)+((sh*sr+ch*cr*sp)*(a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh)))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_XYZ[1] = -(a*(ch*sr-cr*sh*sp)-fx*cp*sh)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)-((ch*sr-cr*sh*sp)*(a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh)))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_XYZ[2] = (cp*cr*(a*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)+Z*(fx*sp-a*cp*cr)-X*(a*(sh*sr+ch*cr*sp)+fx*ch*cp)+Y*(a*(ch*sr-cr*sh*sp)-fx*cp*sh)+fx*(x*ch*cp-z*sp+y*cp*sh)))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2)-(fx*sp-a*cp*cr)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr);
        
        J_XYZ[3] = (b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)+((sh*sr+ch*cr*sp)*(b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr,2);
        J_XYZ[4] = -(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)-((ch*sr-cr*sh*sp)*(b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
        J_XYZ[5] = (b*cp*cr+fy*cp*sr)/(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr)+(cp*cr*(b*(x*(sh*sr+ch*cr*sp)-y*(ch*sr-cr*sh*sp)+z*cp*cr)-Z*(b*cp*cr+fy*cp*sr)+fy*(y*(ch*cr+sh*sp*sr)-x*(cr*sh-ch*sp*sr)+z*cp*sr)-X*(b*(sh*sr+ch*cr*sp)-fy*(cr*sh-ch*sp*sr))+Y*(b*(ch*sr-cr*sh*sp)-fy*(ch*cr+sh*sp*sr))))/pow(X*(sh*sr+ch*cr*sp)-Y*(ch*sr-cr*sh*sp)-x*(sh*sr+ch*cr*sp)+y*(ch*sr-cr*sh*sp)+Z*cp*cr-z*cp*cr, 2);
    }


}
