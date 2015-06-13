#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h> // needed for PRId64 macros

// external linking req'd
#include <math.h>
#include <sba/sba.h>
#include <gsl/gsl_linalg.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"

#include "perls-math/dm.h"
#include "perls-math/gsl_util.h"
#include "perls-math/homogenous.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "perls-lcmtypes/perllcm_van_vlink_t.h"
#include "perls-lcmtypes/perllcm_van_feature_user_depth_t.h"

#include "camera.h"
#include "feature.h"
#include "epipolar.h"
#include "homography.h"
#include "distortion.h"
#include "triangulate.h"
#include "sba.h"

#define CLOCKS_PER_MSEC (CLOCKS_PER_SEC/1000.0)

#define MAXITER         100
#define MAXITER2        150

/* enable debug calls to gslu_matrix_printf ?*/
//#define SBA_VERBOSE

// pointers to additional data, used for computed image projections and their jacobians
typedef struct userdata userdata_t;
struct userdata
{
    double *rot0params; /* initial rotation parameters, combined with a local rotation parameterization
                         * only used in RT case when it uses quaternion
                         */
    double *intrcalib;  /* the 5 intrinsic calibration parameters [fu, u0, v0, ar, skew]
                         * where ar is the aspect ratio fv/fu for RT
                         * or
                         * the 9 elements in K matrix (3x3) for RAE and H
                         */
    double *distCoeffs; /* the 5 distortion coefficients [k1 k2 p1 p2 k3]
                         * consisting of 3 radial distortion coefficients
                         * and 2 tangential coefficients
                         */
    int cnp, pnp, mnp;  /* cnp = number of unknowns in camera pose
                         *     = 6 (RT)   [q1 q2 q3 x y z]
                         *     = 5 (RAE)  [a e r p h]
                         *     = 8 (H)    [a e r p h n_azim n_elev d]
                         * pnp = dof of structure points
                         *     = 3 (RT or RAE) dimensional points
                         *     = 2 (H)         planar points
                         * mnp = dof image points, usually 2 (u,v)
                         *     = 2 (RT, RAE, H)
                         */

    double *invcovret;  /* covariance to be returned */
    int useCmath;       /* if true, use math.h instead of fasttrig for computing P2 matrix */
};

/******************************************************************
 *                 Twoview bundle adjustment RAE
 *
 *
 ******************************************************************/

// -------------------------------------------------------------- //
// [cost function]
// -------------------------------------------------------------- //
static void
img_proj_Rae (double *p,
              struct sba_crsm *idxij,
              int *rcidxs,
              int *rcsubs,
              double *hx,
              void *adata)
{
    const userdata_t *udata = adata;

    const int npts = idxij->nr;     // n number of points (corr.)
    const int ncam = idxij->nc;     // n number of cameras (2 for twoview)

    const int cnp = udata->cnp;     // = 5 (5 dof for pose)
    const int mnp __attribute__((unused)) = udata->mnp;     // = 2 (2 dof for image coord. uv)

    const double *pa = p;           // a part (camera pose) [a1 e1 r1 p1 h1 a2 e2 r2 p2 h2]
    const double *pb = p+ncam*cnp;  // b part (3d points)   [x1 y1 z1 x2 y2 z2 ...]

    // read 3D points, and transpose because SBA uses [N x 3] format
    // -------------------------------------------------------------- //
    gsl_matrix_const_view K_view = gsl_matrix_const_view_array (udata->intrcalib, 3, 3);
    gsl_matrix_const_view X1T_view = gsl_matrix_const_view_array (pb, npts, 3);
    gsl_vector_const_view distCoeffs_view = gsl_vector_const_view_array (udata->distCoeffs, 5);
    gsl_matrix *X1 = gslu_matrix_transpose_alloc (&X1T_view.matrix);

    // compose camera projection matrix for camera 1
    // -------------------------------------------------------------- //
    GSLU_MATRIX_VIEW (P1_view, 3, 4);
    vis_camera_matrix (&P1_view.matrix, &K_view.matrix, NULL, NULL);

    // compose camera projection matrix for camera 2
    // -------------------------------------------------------------- //
    const double b_ext[3] = {(pa+cnp)[0], (pa+cnp)[1], 1};
    GSLU_VECTOR_VIEW (t2_view, 3);

    const double rph[3] = {(pa+cnp)[2], (pa+cnp)[3], (pa+cnp)[4]};
    GSLU_MATRIX_VIEW (R2_view, 3, 3);

    if (udata->useCmath)
    {
        dm_dm2trans_cmath (b_ext, t2_view.data, NULL);
        so3_rotxyz_cmath (R2_view.data, rph);
    }
    else
    {
        dm_dm2trans (b_ext, t2_view.data, NULL);
        so3_rotxyz (R2_view.data, rph);
    }

    GSLU_MATRIX_VIEW (P2_view, 3, 4);
    vis_camera_matrix (&P2_view.matrix, &K_view.matrix, &R2_view.matrix, &t2_view.vector);

    // calculate projected points
    // -------------------------------------------------------------- //
    gsl_matrix *uv1p = gsl_matrix_alloc (2, npts);
    gsl_matrix *uv2p = gsl_matrix_alloc (2, npts);

    if (udata->distCoeffs)
    {
        vis_camera_project_nonlin (&P1_view.matrix, &K_view.matrix, &distCoeffs_view.vector, X1, uv1p);
        vis_camera_project_nonlin (&P2_view.matrix, &K_view.matrix, &distCoeffs_view.vector, X1, uv2p);
    }
    else
    {
        vis_camera_project (&P1_view.matrix, X1, uv1p);
        vis_camera_project (&P2_view.matrix, X1, uv2p);
    }

    // transpose to match SBA format
    // -------------------------------------------------------------- //
    gsl_matrix_view uv1pT_view = gsl_matrix_view_array_with_tda (hx, npts, 2, 4);
    gsl_matrix_view uv2pT_view = gsl_matrix_view_array_with_tda (hx+2, npts, 2, 4);
    gsl_matrix_transpose_memcpy (&uv1pT_view.matrix, uv1p);
    gsl_matrix_transpose_memcpy (&uv2pT_view.matrix, uv2p);

    // clean up
    gsl_matrix_free (X1);
    gsl_matrix_free (uv1p);
    gsl_matrix_free (uv2p);
}

void
vis_sba_img_proj_Rae (const gsl_vector *params,
                      const gsl_matrix *K,
                      const gsl_vector *distCoeffs,
                      int npts,
                      int ncam,
                      int cnp,
                      int pnp,
                      int mnp,
                      gsl_vector *predicted_measurements)
{
    struct sba_crsm idxij =
    {
        .nr = npts,
        .nc = ncam,
    };

    userdata_t udata =
    {
        .cnp = cnp,
        .pnp = pnp,
        .mnp = mnp,
        .intrcalib = K->data,
        .distCoeffs = (distCoeffs) ? distCoeffs->data : NULL,
        .rot0params = NULL,
        .useCmath = 1,
    };

    img_proj_Rae (params->data, &idxij, NULL, NULL, predicted_measurements->data, &udata);
}

// -------------------------------------------------------------- //
// [jacobian of 2-view reprojection function]
// -------------------------------------------------------------- //
void
vis_sba_img_proj_Rae_jacob (const gsl_vector *params,
                            const gsl_matrix *K,
                            const gsl_vector *distCoeffs,
                            int ncam,        // n number of cameras (2 for twoview)
                            gsl_matrix *J)
{
    const int pnp = 3;              // 3 for Euclidean 3D points (x,y,z)
    const int cnp = 5;
    const int mnp = 2;              // = 2 (2 dof for image coord. uv)
    const int npts = (params->size - ncam*cnp) / pnp;        // n number of points (corr.)

    int numJacobRows = J->size1;
    int numJacobCols = J->size2;

    int i;
    double deltaP;

    gsl_vector *pPerturb = gsl_vector_calloc (numJacobCols);
    gsl_vector *measurements = gsl_vector_calloc (numJacobRows);
    gsl_vector *measurementsPrime = gsl_vector_calloc (numJacobRows);

    gsl_vector_view JColView;

    gsl_vector_memcpy (pPerturb, params);

    //for each parameter [a1 e1 r1 p1 h1 a2 e2 r2 p2 h2 x1 y1 z1 ...]
    for (i=0; i<numJacobCols; i++)
    {

        /* Use Hartley/Zisserman recommendation for delta size */
        double pi = gsl_vector_get (params, i);
        deltaP = (abs(pi*1e-4) > 1e-6) ? abs(pi*1e-4) : 1e-6;

        gsl_vector_set (pPerturb, i, pi + deltaP);

        //evaluate at non-perturbed parameter vector
        vis_sba_img_proj_Rae(params, K, distCoeffs, npts, ncam, cnp, pnp, mnp, measurements);

        //evaluate at perturbed parameter vector
        vis_sba_img_proj_Rae(pPerturb, K, distCoeffs, npts, ncam, cnp, pnp, mnp, measurementsPrime);

        //Use finite difference formula to set the ith column of the jacobian
        JColView = gsl_matrix_column(J, i);

        gsl_vector_memcpy(&JColView.vector, measurementsPrime);
        gsl_vector_sub(&JColView.vector, measurements);
        gsl_vector_scale(&JColView.vector, 1/deltaP);

        gsl_vector_set (pPerturb, i, pi);

    }

    gsl_vector_free(pPerturb);
    gsl_vector_free(measurements);
    gsl_vector_free(measurementsPrime);
}

void
vis_sba_img_proj_H_jacob (const gsl_vector *params,
                          const gsl_matrix *K,
                          const gsl_vector *distCoeffs,
                          int ncam,        // n number of cameras (2 for twoview)
                          gsl_matrix *J)
{
    const int pnp = 2;              // 2 for (u,v) in camera 1
    const int cnp = 8;
    const int mnp = 2;              // = 2 (2 dof for image coord. uv)
    const int npts = (params->size - ncam*cnp) / pnp;        // n number of points (corr.)

    int numJacobRows = J->size1;
    int numJacobCols = J->size2;

    int i;
    double deltaP;

    gsl_vector *pPerturb = gsl_vector_calloc (numJacobCols);
    gsl_vector *measurements = gsl_vector_calloc (numJacobRows);
    gsl_vector *measurementsPrime = gsl_vector_calloc (numJacobRows);

    gsl_vector_view JColView;

    gsl_vector_memcpy (pPerturb, params);

    //for each parameter [a1 e1 r1 p1 h1 a2 e2 r2 p2 h2 x1 y1 z1 ...]
    for (i=0; i<numJacobCols; i++)
    {

        /* Use Hartley/Zisserman recommendation for delta size */
        double pi = gsl_vector_get (params, i);
        deltaP = (abs(pi*1e-4) > 1e-6) ? abs(pi*1e-4) : 1e-6;

        gsl_vector_set (pPerturb, i, pi + deltaP);

        //evaluate at non-perturbed parameter vector
        vis_sba_img_proj_H(params, K, distCoeffs, npts, ncam, cnp, pnp, mnp, measurements);

        //evaluate at perturbed parameter vector
        vis_sba_img_proj_H(pPerturb, K, distCoeffs, npts, ncam, cnp, pnp, mnp, measurementsPrime);

        //Use finite difference formula to set the ith column of the jacobian
        JColView = gsl_matrix_column(J, i);

        gsl_vector_memcpy(&JColView.vector, measurementsPrime);
        gsl_vector_sub(&JColView.vector, measurements);
        gsl_vector_scale(&JColView.vector, 1/deltaP);

        gsl_vector_set (pPerturb, i, pi);

    }

    gsl_vector_free(pPerturb);
    gsl_vector_free(measurements);
    gsl_vector_free(measurementsPrime);
}

// -------------------------------------------------------------- //
// [read parameters into sba input structures]
// -------------------------------------------------------------- //
int
readfromgsl_Rae (const int cnp,
                 const int pnp,
                 const int mnp,
                 int *nframes,
                 int *numpts3D,
                 int *numprojs,
                 double **motstruct,
                 double **imgpts,
                 double **covimgpts,
                 char **vmask,
                 double **invcovret,
                 const gsl_vector *X_c1c2,
                 const gsl_matrix *uv1,
                 const gsl_matrix *uv2,
                 const gsl_matrix *X1,
                 const int n_inliers)
{

    /* VAN pose representation = [x y z r p h]      (6 dof)
     * Rae pose                = [azim elev r p h]  (5 dof)
     *
     * reads from: cnp, pnp, mnp, X_c1c2, uv1, uv2, X1, n_inliers
     * construct : motstruct = p vector [a b] = [camera pose, 3D structure]
     *             initrot   = initial quaterion to be used in cost function
     *             imgpts    = true measurement [uv1 and uv2]
     *             covimgpts = covariance of measurement (NULL = 1 pixel noise)
     *             vmask     = in twoview case all 1 (mask to determine if point i is seen by camera j)
     *             invcovret = just allocate memory to store covariance matrix
     */


    double t[3], rph[3], b[3];
    for (int i=0; i<3; i++)
    {
        t[i] = gsl_vector_get (X_c1c2, i);        // t
        rph[i] = gsl_vector_get (X_c1c2, i+3);    // rph
    }
    dm_trans2dm (t, b, NULL);

    *nframes  = 2;           // 2 cameras = twoview BA
    *numprojs = 2*n_inliers;
    *numpts3D = n_inliers;

    int ncam = *nframes;
    int npts = *numpts3D;

    // ------------------- vmask ------------------
    *vmask = malloc (npts * ncam * sizeof(char));
    if (*vmask==NULL)
    {
        ERROR ("memory allocation for 'vmask' failed in readfromgsl()\n");
        return VIS_SBA_ERROR;
    }
    memset (*vmask, 1, npts * ncam * sizeof(char));

    // ------------------ motstruct ------------------
    *motstruct = malloc ( (ncam*cnp + npts*pnp)*sizeof(double) );
    if (*motstruct==NULL)
    {
        ERROR ("memory allocation for 'motstruct' failed in readfromgsl()\n");
        return VIS_SBA_ERROR;
    }

    // cam1
    for (int i=0; i<cnp; i++)
        (*motstruct)[i] = 0.0;

    // cam2
    (*motstruct)[cnp] = b[0];
    (*motstruct)[cnp+1] = b[1];
    (*motstruct)[cnp+2] = rph[0];
    (*motstruct)[cnp+3] = rph[1];
    (*motstruct)[cnp+4] = rph[2];

    // X1
    for (int i=0; i<npts; i++)
    {
        (*motstruct)[ncam*cnp + 3*i]   = gsl_matrix_get (X1, 0, i);
        (*motstruct)[ncam*cnp + 3*i+1] = gsl_matrix_get (X1, 1, i);
        (*motstruct)[ncam*cnp + 3*i+2] = gsl_matrix_get (X1, 2, i);
    }


    // ------------------ imgpts ------------------
    *imgpts = malloc (2*npts * mnp * sizeof(double));
    if (*imgpts==NULL)
    {
        ERROR ("memory allocation for 'initrot' failed in readfromgsl()\n");
        return VIS_SBA_ERROR;
    }
    for (int i=0; i<npts; i++)
    {
        (*imgpts)[4*i]   = gsl_matrix_get (uv1, 0, i);
        (*imgpts)[4*i+1] = gsl_matrix_get (uv1, 1, i);
        (*imgpts)[4*i+2] = gsl_matrix_get (uv2, 0, i);
        (*imgpts)[4*i+3] = gsl_matrix_get (uv2, 1, i);
    }

    // measurements covariance matrices
    //*covimgpts = NULL;
    *covimgpts = malloc (*numprojs * mnp * mnp * sizeof (double));
    for (int i=0; i<*numprojs; i++)
    {
        (*covimgpts)[4*i] = 5.0*5.0;
        (*covimgpts)[4*i+1] = 0.0;
        (*covimgpts)[4*i+2] = 0.0;
        (*covimgpts)[4*i+3] = 5.0*5.0;
    }

    // storage for camera parameter covariance
    *invcovret = malloc ((cnp*cnp)*sizeof(double));

    return 0;
}

// ------------------------------------------------------------------------- //
// [main function - twoview bundle adjustment RAE with nonlinear projection]
// ------------------------------------------------------------------------ //
int
vis_sba_2v_rae_nonlin (const gsl_vector *X_c2c1,     // initial estimate of relative pose, 6 dof [x y z r p h]
                       const gsl_matrix *K,          // calibration matrix [3 x 3]
                       const gsl_vector *distCoeffs, // distortion coefficient [5 x 1] (k1, k2, p1, p2, k3)
                       const gsl_matrix *uv1,        // uv image points in image 1 [2 x N]
                       const gsl_matrix *uv2,
                       const gsl_matrix *X1,         // intial guess of 3D structure points [3 x N]
                       gsl_vector *p21,              // output 1 = optimized 5 dof pose [azim, elev, r, p, h]
                       gsl_matrix *S21,              // output 2 = related covariance matrix [5 x 5]
                       gsl_vector *params,               // optional output 3 = optimized pose + structure
                       int verbose,                  // verbose: 1=ON, 0=OFF
                       GMutex *sba_mutex,
                       void (*f_robust)(const double *const e, int n, double * const w), // robust cost function
                       const gsl_vector *featscalei_f,
                       const gsl_vector *featscalej_f)
{
    const int n_inliers = uv1->size2;    // N

    // X_c2c1 = [xyzrph]
    const int cnp=5;          // 2 trans params + 3 rot params
    const int pnp=3;          // euclidean 3D points
    const int mnp=2;          // image points are 2D
    const int constraint_nth_cam =1;     // fix the first camera

    // read from gsl to sba input arguments
    // -------------------------------------------------------------- //
    int nframes, numpts3D, numprojs;
    char *vmask;
    double *motstruct, *imgpts, *covimgpts, *invcovret;
    int ret = readfromgsl_Rae (cnp, pnp, mnp,
                               &nframes, &numpts3D, &numprojs,
                               &motstruct, &imgpts, &covimgpts, &vmask, &invcovret,
                               X_c2c1, uv1, uv2, X1, n_inliers);

    /* If provided the scale, we can estimate the feature covariance */
    if (featscalei_f && featscalej_f)
    {
        free (covimgpts);
        covimgpts = vis_feature_2v_covimgpts_alloc (featscalei_f, featscalej_f);
    }

    if (ret == VIS_SBA_ERROR)
        goto ON_ERROR;

    // set up userdata (need in cost function)
    // -------------------------------------------------------------- //
    userdata_t userdata =
    {
        .cnp = cnp,
        .pnp = pnp,
        .mnp = mnp,
        .intrcalib = K->data,
        .distCoeffs = (distCoeffs ? distCoeffs->data : NULL),
        .rot0params = NULL,
    };

    // SBA option
    // -------------------------------------------------------------- //
    int analyticjac = 0;                        // analytic or approximate jacobian?
    double opts[SBA_OPTSSZ] =
    {
        [0]=SBA_INIT_MU,
        [1]=SBA_STOP_THRESH,
        [2]=SBA_STOP_THRESH,
        [3]=SBA_STOP_THRESH,
        [4]=0.0,
    };
    //opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
    //opts[4]=1E-05; // uncomment to force termination if the relative reduction in the RMS reprojection error drops below 1E-05

    // RUN SBA
    // -------------------------------------------------------------- //
    if (verbose)
        printf ("Starting BA - RAE with fixed intrinsic parameters\n");

    //get mutex lock
    if (sba_mutex) g_mutex_lock (sba_mutex);

    double info[SBA_INFOSZ];
    int64_t start_time = timestamp_now ();
    ret = sba_motstr_levmar_x (numpts3D, 0, 2, constraint_nth_cam, vmask, motstruct, cnp, pnp, imgpts, covimgpts, mnp,
                               img_proj_Rae, NULL, &userdata, MAXITER2, verbose, opts, info, invcovret, f_robust);
    int64_t end_time = timestamp_now ();

    //if sba is successful, copy the resulting motstruct to params vector
    if (ret > 0)
    {
        if (params)
        {
            gsl_vector_view paramView = gsl_vector_view_array (motstruct, 2*cnp + pnp*numpts3D);
            gsl_vector_memcpy(params, &paramView.vector);
        }
    }

    //unlock mutex
    if (sba_mutex) g_mutex_unlock (sba_mutex);

    if (verbose)   // print results
    {
        if (ret==SBA_ERROR)
            ERROR ("SBA_ERROR occured\n");
        else
        {
            const int nvars=nframes*cnp + numpts3D*pnp;
            fflush (stdout);
            fprintf (stdout, "SBA using %d 3D pts, %d frames and %d image projections, %d variables\n",
                     numpts3D, nframes, numprojs, nvars);
            fprintf (stdout, "\n %s Jacobian, %s covariances",
                     analyticjac? "analytic" : "approximate", covimgpts? "with" : "without");
            fputs ("\n\n", stdout);
            fprintf (stdout, "SBA returned %d in %g iter, reason %g, error %g [initial %g], %d/%d func/fjac evals, %d lin. systems\n",
                     ret, info[5], info[6], info[1]/numprojs, info[0]/numprojs, (int)info[7], (int)info[8], (int)info[9]);
            fprintf (stdout, "Elapsed time: %"PRId64" usecs\n", end_time - start_time);
            fflush (stdout);
        }
    }

    if (ret > 0)
    {
        gsl_matrix_view S21_tmp = gsl_matrix_view_array (invcovret, cnp, cnp);

        // turning off default gsl error handler (=abort) to check pos def
        gsl_error_handler_t *default_handler = gsl_set_error_handler_off ();

        // cholesky failes for non pos def matrix
        if (gsl_linalg_cholesky_decomp (&S21_tmp.matrix) == GSL_EDOM)
        {
            ret = SBA_ERROR;
        }
        else
        {
            gsl_linalg_cholesky_invert (&S21_tmp.matrix);
            gsl_matrix_memcpy (S21, &S21_tmp.matrix);

            // store
            gsl_vector_set (p21, 0, motstruct[cnp+0]);
            gsl_vector_set (p21, 1, motstruct[cnp+1]);
            gsl_vector_set (p21, 2, motstruct[cnp+2]);
            gsl_vector_set (p21, 3, motstruct[cnp+3]);
            gsl_vector_set (p21, 4, motstruct[cnp+4]);

            /*// cpr -- DEBUG
            // -------------------------------------------------------------- //
            FILE *fk = fopen ("k.txt", "wb");  gsl_matrix_fprintf (fk, K,"%g"); fclose(fk);
            FILE *fuv1 = fopen ("uv1.txt", "wb");  gsl_matrix_fprintf (fuv1, uv1,"%g");  fclose(fuv1);
            FILE *fuv2 = fopen ("uv2.txt", "wb");  gsl_matrix_fprintf (fuv2, uv2,"%g");  fclose(fuv2);
            FILE *fX = fopen ("X.txt", "wb");  gsl_matrix_fprintf (fX, X1,"%g");  fclose(fX);
            FILE *fx21 = fopen ("x21.txt", "wb");  gsl_vector_fprintf (fx21, X_c2c1,"%g");  fclose(fx21);
            FILE *fp21 = fopen ("p21.txt", "wb");  gsl_vector_fprintf (fp21, p21,"%g");  fclose(fp21);*/

        }

        // restore back to default
        gsl_set_error_handler (default_handler);
    }

ON_ERROR:
    // clean up
    if (covimgpts) free (covimgpts);
    if (invcovret) free (invcovret);
    if (motstruct) free (motstruct);
    if (imgpts)    free (imgpts);
    if (vmask)     free (vmask);

    return ret;
}

// -------------------------------------------------------------- //
// [main function - twoview bundle adjustment RAE]
// -------------------------------------------------------------- //
int
vis_sba_2v_rae (const gsl_vector *X_c2c1,     // initial estimate of relative pose, 6 dof [x y z r p h]
                const gsl_matrix *K,          // calibration matrix [3 x 3]
                const gsl_matrix *uv1,        // uv image points in image 1 [2 x N]
                const gsl_matrix *uv2,
                const gsl_matrix *X1,         // intial guess of 3D structure points [3 x N]
                gsl_vector *p21,              // output 1 = optimized 5 dof pose [azim, elev, r, p, h]
                gsl_matrix *S21,              // output 2 = related covariance matrix [5 x 5]
                gsl_vector *params,               // optional output 3 = optimized pose + structure
                int verbose,                  // verbose: 1=ON, 0=OFF
                GMutex *sba_mutex,
                void (*f_robust)(const double *const e, int n, double * const w),  // robust cost function
                const gsl_vector *featscalei_f,
                const gsl_vector *featscalej_f)
{
    //use nonlinear vis_sba_2v_rae_nonlin with distCoeffs = NULL
    gsl_vector *distCoeffs = NULL;
    return vis_sba_2v_rae_nonlin (X_c2c1, K, distCoeffs, uv1, uv2, X1, p21, S21, params, verbose, sba_mutex, f_robust, featscalei_f, featscalej_f);
}

/******************************************************************
 *               Twoview bundle adjustment Homography
 *
 *
 ******************************************************************/

// -------------------------------------------------------------- //
// [cost function]
// -------------------------------------------------------------- //
static void
img_proj_H (double *p,
            struct sba_crsm *idxij,
            int *rcidxs,
            int *rcsubs,
            double *hx,
            void *adata)
{
    const userdata_t *udata = adata;

    const int npts = idxij->nr;    // n number of points (corr.)
    const int ncam = idxij->nc;    // n number of camera (2 for twoview)

    const int cnp = udata->cnp;    // = 8 (8 dof for pose)
    const int mnp __attribute__((unused)) = udata->mnp;    // = 2 (2 dof for image coord. uv)

    const double *pa = p;          // a part (camera pose) [a1 e1 r1 p1 h1 na1 ne1 d1 a2 e2 r2 p2 h2 na2 ne2 d2]
    const double *pb = p+ncam*cnp; // b part (2d points)   [u1 v1 u2 v2 u3 v3 ...]


    // read params for camera 2
    // -------------------------------------------------------------- //
    const double b_ext[3] = {(pa+cnp)[0], (pa+cnp)[1], 1};
    const double rph[3] = {(pa+cnp)[2], (pa+cnp)[3], (pa+cnp)[4]};
    const double n_ext[3] = {(pa+cnp)[5], (pa+cnp)[6], 1};
    const double d = (pa+cnp)[7];

    GSLU_VECTOR_VIEW (t_view, 3);
    dm_dm2trans (b_ext, t_view.data, NULL);

    GSLU_VECTOR_VIEW (n_view, 3);
    dm_dm2trans (n_ext, n_view.data, NULL);

    GSLU_MATRIX_VIEW (R2_view, 3, 3);
    so3_rotxyz (R2_view.data, rph);

    // read 3D points, and transpose because SBA uses [N x 2] format
    // -------------------------------------------------------------- //
    gsl_matrix_const_view K_view = gsl_matrix_const_view_array (udata->intrcalib, 3, 3);
    gsl_matrix_const_view uv1_hatT_view = gsl_matrix_const_view_array (pb, npts, 2);
    gsl_vector_const_view distCoeffs_view = gsl_vector_const_view_array (udata->distCoeffs, 5);
    gsl_matrix* uv1_hat = gslu_matrix_transpose_alloc (&uv1_hatT_view.matrix);

    // project
    // -------------------------------------------------------------- //
    GSLU_MATRIX_VIEW (H_view, 3, 3);
    vis_homog_matrix_plane_induced (&H_view.matrix, &K_view.matrix, &R2_view.matrix,
                                    &t_view.vector, &n_view.vector, d);

    gsl_matrix *uv2_hat = gsl_matrix_alloc (2, npts);
    if (udata->distCoeffs)
    {
        vis_homog_project_nonlin (&H_view.matrix, &K_view.matrix, &distCoeffs_view.vector, uv1_hat, uv2_hat);
        /* Apply distortion to uv1_hat */
        vis_distort_pts_radial (uv1_hat, uv1_hat, &K_view.matrix, &distCoeffs_view.vector);
    }
    else
    {
        vis_homog_project (&H_view.matrix, uv1_hat, uv2_hat);
    }

    // transpose to match SBA format
    // -------------------------------------------------------------- //
    gsl_matrix_view uv1_hatT_hx_view = gsl_matrix_view_array_with_tda (hx, npts, 2, 4);
    gsl_matrix_view uv2_hatT_hx_view = gsl_matrix_view_array_with_tda (hx+2, npts, 2, 4);
    if (udata->distCoeffs)
    {
        gsl_matrix_transpose_memcpy (&uv1_hatT_hx_view.matrix, uv1_hat);
    }
    else
    {
        gsl_matrix_memcpy (&uv1_hatT_hx_view.matrix, &uv1_hatT_view.matrix);
    }
    gsl_matrix_transpose_memcpy (&uv2_hatT_hx_view.matrix, uv2_hat);

    // clean up
    gsl_matrix_free (uv1_hat);
    gsl_matrix_free (uv2_hat);
}

void
vis_sba_img_proj_H (const gsl_vector *params,
                    const gsl_matrix *K,
                    const gsl_vector *distCoeffs,
                    int npts,
                    int ncam,
                    int cnp,
                    int pnp,
                    int mnp,
                    gsl_vector *predicted_measurements)
{
    struct sba_crsm idxij =
    {
        .nr = npts,
        .nc = ncam,
    };

    userdata_t udata =
    {
        .cnp = cnp,
        .pnp = pnp,
        .mnp = mnp,
        .intrcalib = K->data,
        .distCoeffs = (distCoeffs) ? distCoeffs->data : NULL,
        .rot0params = NULL,
        .useCmath = 1,
    };

    img_proj_H (params->data, &idxij, NULL, NULL, predicted_measurements->data, &udata);
}

// -------------------------------------------------------------- //
// [read parameters into sba input structures]
// -------------------------------------------------------------- //
int
readfromgsl_H (const int cnp,
               const int pnp,
               const int mnp,
               int *nframes,
               int *numpts2D,
               int *numprojs,
               double **motstruct,
               double **imgpts,
               double **covimgpts,
               char **vmask,
               double **invcovret,
               const gsl_vector *X_c1c2,
               const gsl_matrix *uv1,
               const gsl_matrix *uv2,
               const gsl_vector *n_o,
               double d_o,
               const gsl_matrix *uv1p,
               const int n_inliers)
{

    /* VAN pose representation = [x y z r p h n1 n2 d]      (9 dof)
     * Rae pose                = [azim elev r p h n1 n2 d]  (8 dof)
     *
     * reads from: cnp, pnp, mnp, X_c1c2, uv1, uv2, X1, n_inliers
     * construct : motstruct = p vector [a b] = [camera pose, 3D structure]
     *             initrot   = initial quaterion to be used in cost function
     *             imgpts    = true measurement [uv1 and uv2]
     *             covimgpts = covariance of measurement (NULL = 1 pixel noise)
     *             vmask     = in twoview case all 1 (mask to determine if point i is seen by camera j
     *             invcovret = just allocate memory to store covariance matrix
     */

    double t[3], rph[3], n[3];
    for (int i=0; i <3; i++)
    {
        t[i] = gsl_vector_get (X_c1c2,i);     // t
        rph[i] = gsl_vector_get (X_c1c2,i+3); // rph
        n[i] = gsl_vector_get (n_o,i);        // n
    }

    double b[3], n_dm[3];
    dm_trans2dm (t, b, NULL);
    dm_trans2dm (n, n_dm, NULL);

    *nframes = 2;           // 2 cameras = twoview BA
    *numprojs = 2*n_inliers;
    *numpts2D = n_inliers;

    const int ncam = *nframes;
    const int npts = *numpts2D;

    // ------------------- vmask ------------------
    *vmask = malloc (npts * ncam * sizeof (char));
    if (*vmask==NULL)
    {
        fprintf (stderr, "memory allocation for 'vmask' failed in readfromgsl()\n");
        return VIS_SBA_ERROR;
    }
    memset (*vmask, 1, npts * ncam * sizeof (char));

    // ------------------ motstruct ------------------
    *motstruct = malloc ( (ncam*cnp + npts*pnp) * sizeof (double));
    if(*motstruct == NULL)
    {
        fprintf (stderr, "memory allocation for 'motstruct' failed in readfromgsl()\n");
        return VIS_SBA_ERROR;
    }

    // cam1
    for (int i=0; i<cnp; i++)
        (*motstruct)[i] = 0;

    // cam2
    (*motstruct)[5] = n_dm[0];
    (*motstruct)[6] = n_dm[1];
    (*motstruct)[7] = d_o;   // temp. FIXIT = DELETE
    (*motstruct)[cnp] = b[0];
    (*motstruct)[cnp+1]  = b[1];
    (*motstruct)[cnp+2] = rph[0];
    (*motstruct)[cnp+3] = rph[1];
    (*motstruct)[cnp+4] = rph[2];
    (*motstruct)[cnp+5] = n_dm[0];
    (*motstruct)[cnp+6] = n_dm[1];
    (*motstruct)[cnp+7] = d_o;

    // uv1_hat
    for (int i=0; i<npts; i++)
    {
        (*motstruct)[ncam*cnp + 2*i]   = gsl_matrix_get (uv1p, 0, i);
        (*motstruct)[ncam*cnp + 2*i+1] = gsl_matrix_get (uv1p, 1, i);
    }

    // ------------------ imgpts ------------------
    *imgpts = malloc (*numprojs * mnp * sizeof (double));
    if (*imgpts==NULL)
    {
        fprintf (stderr, "memory allocation for 'initrot' failed in readfromgsl()\n");
        return VIS_SBA_ERROR;
    }
    for (int i=0; i<npts; i++)
    {
        (*imgpts)[4*i]   = gsl_matrix_get (uv1, 0, i);
        (*imgpts)[4*i+1] = gsl_matrix_get (uv1, 1, i);
        (*imgpts)[4*i+2] = gsl_matrix_get (uv2, 0, i);
        (*imgpts)[4*i+3] = gsl_matrix_get (uv2, 1, i);
    }

    // measurements covariance matrices
    //*covimgpts = NULL;
    *covimgpts = malloc (*numprojs *mnp * mnp * sizeof (double));
    //memset (*covimgpts, 0, (*numprojs *mnp * mnp) * sizeof (double));
    for (int i=0; i<*numprojs; i++)
    {
        (*covimgpts)[4*i] = 5.0*5.0;
        (*covimgpts)[4*i+1] = 0.0;
        (*covimgpts)[4*i+2] = 0.0;
        (*covimgpts)[4*i+3] = 5.0*5.0;
    }

    // allocate memory for camera parameter covariance
    *invcovret= malloc ((cnp*cnp)*sizeof (double));

    return 0;
}

int
vis_sba_2v_h_nonlin (const gsl_vector *X_c2c1,       // initial estimate of relative pose, 6 dof [x y z r p h]
                     const gsl_matrix *K,            // calibration matrix [3 x 3]
                     const gsl_vector *distCoeffs,   // distortion coefficient [5 x 1] (k1, k2, p1, p2, k3)
                     const gsl_vector *n_o,          // initial guess for plane normal vector n_o = [0 0 -1]
                     const double d_o,               // initial guess for scene depth (d = mean(Z))
                     const gsl_matrix *uv1,          // uv image points in image 1 [2 x N]
                     const gsl_matrix *uv2,
                     const gsl_matrix *uv1p,         // intial guess of projected points [2 x N]
                     gsl_vector *p21,                // output 1 = optimized 5 dof pose [azim, elev, r, p, h]
                     gsl_matrix *S21,                // output 2 = related covariance matrix [5 x 5]
                     gsl_vector *params,             // optional output 3 = optimized pose + structure
                     int verbose,                    // verbose: 1=ON, 0=OFF
                     GMutex *sba_mutex,
                     const gsl_vector *featscalei_h,
                     const gsl_vector *featscalej_h)
{
    const int n_inliers = uv1->size2;    // N

    // X_c2c1 = [xyzrph]
    const int cnp=8;          // 2 trans params + 3 rot params + 2 normal + 1 dist = 8
    const int pnp=2;          // projected image point 2D
    const int mnp=2;          // image points are 2D
    const int constraint_nth_cam =1;     // fix the first camera

    // read from gsl to sba input arguments
    // -------------------------------------------------------------- //
    double *motstruct, *imgpts, *covimgpts, *invcovret;
    char *vmask;
    int nframes, numpts2D, numprojs;
    int ret = readfromgsl_H (cnp, pnp, mnp, &nframes, &numpts2D, &numprojs,
                             &motstruct, &imgpts, &covimgpts, &vmask, &invcovret,
                             X_c2c1, uv1, uv2, n_o, d_o, uv1p, n_inliers);

    /* If provided the scale, we can estimate the feature covariance */
    if (featscalei_h && featscalej_h)
    {
        free (covimgpts);
        covimgpts = vis_feature_2v_covimgpts_alloc (featscalei_h, featscalej_h);
    }

    if (ret == VIS_SBA_ERROR)
        goto ON_ERROR;

    // set up userdata (need in cost function)
    // -------------------------------------------------------------- //
    userdata_t userdata =
    {
        .cnp = cnp,
        .pnp = pnp,
        .mnp = mnp,
        .intrcalib = K->data,         // K matrix
        .distCoeffs = (distCoeffs ? distCoeffs->data : NULL),
        .rot0params = NULL,
    };

    // SBA option
    // -------------------------------------------------------------- //
    int analyticjac=0;  // analytic or approximate jacobian?
    double opts[SBA_OPTSSZ] =
    {
        [0] = SBA_INIT_MU,
        [1] = SBA_STOP_THRESH,
        [2] = SBA_STOP_THRESH,
        [3] = SBA_STOP_THRESH,
        [4] = 0.0,
    };
    //opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
    //opts[4]=1E-05; // uncomment to force termination if the relative reduction in the RMS reprojection error drops below 1E-05


    // RUN SBA
    // -------------------------------------------------------------- //
    if (verbose)
        printf ("Starting BA-H with fixed intrinsic parameters\n");

    //get mutex lock
    g_mutex_lock (sba_mutex);

    double info[SBA_INFOSZ];
    int64_t start_time = timestamp_now ();
    ret = sba_motstr_levmar_x (numpts2D, 0, 2, constraint_nth_cam, vmask, motstruct, cnp, pnp, imgpts, covimgpts, mnp,
                               img_proj_H, NULL, &userdata, MAXITER2, verbose, opts, info, invcovret, NULL);
    int64_t end_time = timestamp_now ();

    //if sba is successful, copy the resulting motstruct to params vector
    if (ret > 0)
    {
        if (params)
        {
            gsl_vector_view paramView = gsl_vector_view_array (motstruct, 2*cnp + pnp*numpts2D);
            gsl_vector_memcpy(params, &paramView.vector);
        }
    }

    //get mutex unlock
    if (sba_mutex) g_mutex_unlock (sba_mutex);

    if (verbose)   // print results
    {
        if (ret==SBA_ERROR)
        {
            ERROR ("SBA_ERROR occured");
        }
        else
        {
            const int nvars = nframes*cnp + numpts2D*pnp;

            fflush (stdout);
            fprintf (stdout, "SBA using %d 3D pts, %d frames and %d image projections, %d variables\n",
                     numpts2D, nframes, numprojs, nvars);
            fprintf (stdout, "\n %s Jacobian, %s covariances",
                     analyticjac? "analytic" : "approximate",
                     covimgpts? "with" : "without");
            fputs ("\n\n", stdout);
            fprintf (stdout, "SBA returned %d in %g iter, reason %g, error %g [initial %g], %d/%d func/fjac evals, %d lin. systems\n",
                     ret, info[5], info[6], info[1]/numprojs, info[0]/numprojs,
                     (int)info[7], (int)info[8], (int)info[9]);
            fprintf (stdout, "Elapsed time: %"PRId64" usecs\n", end_time - start_time);
            fflush (stdout);
        }
    }

    if (ret > 0)
    {
        gsl_matrix_view S21_tmp = gsl_matrix_view_array (invcovret, cnp, cnp);

        // turning off default gsl error handler (=abort) to check pos def
        gsl_error_handler_t *default_handler = gsl_set_error_handler_off ();

        // cholesky failes for non pos def matrix
        if (gsl_linalg_cholesky_decomp (&S21_tmp.matrix) == GSL_EDOM)
        {
            ret = SBA_ERROR;
        }
        else
        {
            gsl_linalg_cholesky_invert (&S21_tmp.matrix);
            gsl_matrix_view S21_tmp_sub = gsl_matrix_submatrix (&S21_tmp.matrix, 0, 0, 5, 5);
            gsl_matrix_memcpy (S21, &S21_tmp_sub.matrix);

            // store
            gsl_vector_set (p21, 0, motstruct[cnp+0]);
            gsl_vector_set (p21, 1, motstruct[cnp+1]);
            gsl_vector_set (p21, 2, motstruct[cnp+2]);
            gsl_vector_set (p21, 3, motstruct[cnp+3]);
            gsl_vector_set (p21, 4, motstruct[cnp+4]);
        }

        // restore back to default
        gsl_set_error_handler (default_handler);

    }

    // clean up
ON_ERROR:
    if (covimgpts) free (covimgpts);
    if (invcovret) free (invcovret);
    if (motstruct) free (motstruct);
    if (imgpts)    free (imgpts);
    if (vmask)     free (vmask);

    return ret;
}

// -------------------------------------------------------------- //
// [main function - twoview bundle adjustment H]
// -------------------------------------------------------------- //
int
vis_sba_2v_h (const gsl_vector *X_c2c1,       // initial estimate of relative pose, 6 dof [x y z r p h]
              const gsl_matrix *K,            // calibration matrix [3 x 3]
              const gsl_vector *n_o,          // initial guess for plane normal vector n_o = [0 0 -1]
              const double d_o,               // initial guess for scene depth (d = mean(Z))
              const gsl_matrix *uv1,          // uv image points in image 1 [2 x N]
              const gsl_matrix *uv2,
              const gsl_matrix *uv1p,         // intial guess of projected points [2 x N]
              gsl_vector *p21,                // output 1 = optimized 5 dof pose [azim, elev, r, p, h]
              gsl_matrix *S21,                // output 2 = related covariance matrix [5 x 5]
              gsl_vector *params,            // optional output 3 = optimized pose + structure
              int verbose,                    // verbose: 1=ON, 0=OFF
              GMutex *sba_mutex,
              const gsl_vector *featscalei_h,
              const gsl_vector *featscalej_h)
{
    gsl_vector *distCoeffs = NULL;
    return vis_sba_2v_h_nonlin (X_c2c1, K, distCoeffs, n_o, d_o, uv1, uv2, uv1p, p21, S21, params, verbose, sba_mutex, featscalei_h, featscalej_h);
}

void
vis_sba_2v_rae_enforce_tri_const_alloc (const gsl_matrix *K, const gsl_vector *X_c2c1,
                                        const gsl_matrix *uvi_f, const gsl_matrix *uvj_f,
                                        double min_dist, double max_dist,
                                        gsl_matrix **uvi_f_triconst, gsl_matrix **uvj_f_triconst)
{
    size_t n_in_f = uvi_f->size2;

    // triangulate
    GSLU_MATRIX_VIEW (R, 3,3);
    gsl_vector_const_view t = gsl_vector_const_subvector (X_c2c1, 0, 3);
    gsl_vector_const_view rph = gsl_vector_const_subvector (X_c2c1, 3, 3);
    so3_rotxyz (R.matrix.data, rph.vector.data);

    vis_triangulate_t *tri = vis_triangulate_alloc (K, &R.matrix, &t.vector, uvi_f, uvj_f);

    // enforce triangulation constraint
    gslu_index *tri_const_idx = vis_triangulate_constraint_alloc (tri->X1, 2, min_dist, max_dist);

    size_t n_accepted = tri_const_idx ? tri_const_idx->size : 0;
    if (tri_const_idx && n_accepted != n_in_f)
    {
        *uvi_f_triconst = gslu_matrix_selcol_alloc (uvi_f, tri_const_idx);
        *uvj_f_triconst = gslu_matrix_selcol_alloc (uvj_f, tri_const_idx);
    }
    else
    {
        *uvi_f_triconst = gsl_matrix_calloc (uvi_f->size1, uvi_f->size2);
        *uvj_f_triconst = gsl_matrix_calloc (uvj_f->size1, uvj_f->size2);
        gsl_matrix_memcpy (*uvi_f_triconst, uvi_f);
        gsl_matrix_memcpy (*uvj_f_triconst, uvj_f);
    }

    gslu_index_free (tri_const_idx);
    vis_triangulate_free (tri);
}

int
vis_sba_2v_rae_nonlin_from_model_with_tri (const gsl_matrix *K, const gsl_vector *distCoeffs, const gsl_vector *X_c2c1,
        const gsl_matrix *uvi_f, const gsl_matrix *uvj_f, size_t minpt,
        int pt3d_debug_mode, double min_dist, double max_dist,
        gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, gsl_vector *params, int verbose, gsl_matrix **X1_plot,
        GMutex *sba_mutex, const gsl_vector *featscalei_f, const gsl_vector *featscalej_f)
{
    int ret_sba = VIS_SBA_ERROR;

    size_t n_in_f = uvi_f->size2;

    // triangulate
    GSLU_MATRIX_VIEW (R, 3,3);
    gsl_vector_const_view t = gsl_vector_const_subvector (X_c2c1, 0, 3);
    gsl_vector_const_view rph = gsl_vector_const_subvector (X_c2c1, 3, 3);
    so3_rotxyz (R.matrix.data, rph.vector.data);

    vis_triangulate_t *tri = vis_triangulate_alloc (K, &R.matrix, &t.vector, uvi_f, uvj_f);

    // enforce triangulation constraint
    gslu_index *tri_const_idx = vis_triangulate_constraint_alloc (tri->X1, 2, min_dist, max_dist);

    /* Also compute 3D points in camera 2's frame, but ONLY to make sure the triangulated
     * points are in front of camera 2.  Use camera1 to actually compute the indeces which
     * fall within the acceptable min_dist and max_dist */
    gsl_matrix *X1_h = homogenize_alloc (tri->X1);
    gsl_matrix *X2 = NULL;
    gsl_matrix *Rt = gsl_matrix_calloc (3, 4);
    gsl_matrix_view R_view = gsl_matrix_submatrix (Rt, 0, 0, 3, 3);
    gsl_vector_view t_view = gsl_matrix_column (Rt, 3);
    gsl_matrix_memcpy (&R_view.matrix, &R.matrix);
    gsl_vector_memcpy (&t_view.vector, &t.vector);
    X2 = gslu_blas_mm_alloc (Rt, X1_h);
    gslu_index *tri_const_idx2 = vis_triangulate_constraint_alloc (X2, 2, min_dist, max_dist);
    /* clean up depth test for camera 2 */
    gsl_matrix_free (Rt);
    gsl_matrix_free (X1_h);
    gsl_matrix_free (X2);

    if (tri_const_idx && tri_const_idx2)
    {
        size_t n_accepted = tri_const_idx->size;
        size_t n_accepted2 = tri_const_idx2->size;
        //printf("tri-constraint: rejecting %d points out of %d points\n", n_in_f-n_accepted, n_in_f);
        if (n_accepted < minpt && n_accepted2 < minpt)
        {
            printf ("[twoview]    ERROR: NPTS (tri): npts (%zu) < required (%zu: %g~%g\n)", n_accepted, minpt, min_dist, max_dist);
            gslu_index_free (tri_const_idx);
            gslu_index_free (tri_const_idx2);
            vis_triangulate_free (tri);
            return -1;
        }

        if (n_accepted == n_in_f)
        {
            ret_sba = vis_sba_2v_rae_nonlin (X_c2c1, K, distCoeffs, uvi_f, uvj_f, tri->X1, rel_pose21, rel_pose_cov21, params, verbose, sba_mutex, NULL, featscalei_f, featscalej_f);  // X_c2c1 = [xyzrph]

            if (ret_sba > 0 && pt3d_debug_mode )
                (*X1_plot) = gslu_matrix_clone (tri->X1);
        }
        else
        {
            gsl_matrix *uvi_f_triconst = gslu_matrix_selcol_alloc (uvi_f, tri_const_idx);
            gsl_matrix *uvj_f_triconst = gslu_matrix_selcol_alloc (uvj_f, tri_const_idx);
            gsl_matrix *X1_triconst = gslu_matrix_selcol_alloc (tri->X1, tri_const_idx);

            ret_sba = vis_sba_2v_rae_nonlin (X_c2c1, K, distCoeffs, uvi_f_triconst, uvj_f_triconst, X1_triconst, rel_pose21, rel_pose_cov21, params, verbose, sba_mutex, NULL, featscalei_f, featscalej_f);  // X_c2c1 = [xyzrph]

            if (ret_sba > 0 && pt3d_debug_mode )
                (*X1_plot) = gslu_matrix_clone (X1_triconst);

            gslu_matrix_free (uvi_f_triconst);
            gslu_matrix_free (uvj_f_triconst);
            gslu_matrix_free (X1_triconst);
        }
    }
    else
    {
        printf ("[twoview]    ERROR: NPTS (tri): npts (%d) < required (%zu: %g~%g\n)", 0, minpt, min_dist, max_dist);
#ifdef SBA_VERBOSE
        gslu_vector_printf (X_c2c1, "X_c2c1");
        gslu_matrix_printf (tri->X1, "X1");
#endif
        ret_sba = PERLLCM_VAN_VLINK_T_MSG_TRI_CONST;
    }

    gslu_index_free (tri_const_idx);
    gslu_index_free (tri_const_idx2);
    vis_triangulate_free (tri);

    return ret_sba;
}

int
vis_sba_2v_rae_from_model_with_tri (const gsl_matrix *K, const gsl_vector *X_c2c1, const gsl_matrix *uvi_f, const gsl_matrix *uvj_f,
                                    size_t minpt, int pt3d_debug_mode, double min_dist, double max_dist,
                                    gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, gsl_vector *params, int verbose, gsl_matrix **X1_plot,
                                    GMutex *sba_mutex, const gsl_vector *featscalei_f, const gsl_vector *featscalej_f)
{
    gsl_vector *distCoeffs = NULL;
    return vis_sba_2v_rae_nonlin_from_model_with_tri (K, distCoeffs, X_c2c1, uvi_f, uvj_f,
            minpt, pt3d_debug_mode, min_dist, max_dist,
            rel_pose21, rel_pose_cov21, params, verbose, X1_plot,
            sba_mutex, featscalei_f, featscalej_f);
}

int
vis_sba_2v_rae_from_model_wo_tri (const gsl_matrix *K, const gsl_vector *X_c2c1, const gsl_matrix *uvi_f, const gsl_matrix *uvj_f,
                                  size_t minpt, int pt3d_debug_mode, double min_dist, double max_dist,
                                  gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, int verbose, gsl_matrix **X1_plot,
                                  GMutex *sba_mutex)
{
    int ret_sba = VIS_SBA_ERROR;
    GSLU_VECTOR_VIEW (X_C21, 6, {1.,0.,0.,0.,0.,0.});   // just arbitrary non-zero x_c21

    // triangulate
    GSLU_MATRIX_VIEW (R, 3,3);
    gsl_vector_const_view t = gsl_vector_const_subvector (&X_C21.vector, 0, 3);
    gsl_vector_const_view rph = gsl_vector_const_subvector (&X_C21.vector, 3, 3);
    so3_rotxyz (R.matrix.data, rph.vector.data);

    vis_triangulate_t *tri = vis_triangulate_alloc (K, &R.matrix, &t.vector, uvi_f, uvj_f);

    ret_sba = vis_sba_2v_rae (&X_C21.vector, K, uvi_f, uvj_f, tri->X1, rel_pose21, rel_pose_cov21, NULL, verbose, sba_mutex, NULL, NULL, NULL);  // X_c2c1 = [xyzrph]

    if (ret_sba > 0 && pt3d_debug_mode )
        (*X1_plot) = gslu_matrix_clone (tri->X1);

    vis_triangulate_free (tri);

    return ret_sba;
}

int
vis_sba_2v_h_nonlin_from_model (gsl_matrix *H, const gsl_vector *distCoeffs, const gsl_matrix *uvi_h, const gsl_matrix *uvj_h,
                                const gsl_matrix *K, const perllcm_van_feature_t *fi, const gsl_vector *X_c2c1,
                                gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, gsl_vector *params, int verbose,
                                GMutex *sba_mutex, const gsl_vector *featscalei_h, const gsl_vector *featscalej_h)
{
    size_t n_in_h = uvj_h->size2;

    GSLU_MATRIX_VIEW (invH, 3, 3);
    gslu_matrix_inv (&invH.matrix, H);
    gsl_matrix *uvj_h_h = homogenize_alloc(uvj_h);
    gsl_matrix *uvip_h = gsl_matrix_alloc (3, n_in_h);
    gslu_blas_mm (uvip_h, &invH.matrix, uvj_h_h);
    gsl_matrix *uvip = dehomogenize_alloc (uvip_h);

    // run BA
    // -------------------------------------------------- //
    double d_o = 1.0;                      // dist_o = mean(Z1);
    if (fi)
    {
        perllcm_van_feature_user_depth_t *sp1 = malloc (sizeof (*sp1));
        perllcm_van_feature_user_depth_t_decode (fi->user, 0, fi->usersize, sp1);
        double z_avg = 0.0;
        for (int i=0; i<sp1->npts; i++)
            z_avg = z_avg+sp1->mu_Z[i];
        if (sp1->npts > 0) d_o = z_avg/sp1->npts;
        perllcm_van_feature_user_depth_t_destroy (sp1);
    }

    GSLU_VECTOR_VIEW (n_o, 3, {0, 0, -1});
    int ret_sba = vis_sba_2v_h_nonlin (X_c2c1, K, distCoeffs, &n_o.vector, d_o, uvi_h, uvj_h, uvip, rel_pose21, rel_pose_cov21, params, verbose, sba_mutex, featscalei_h, featscalej_h);

    // clean up
    gslu_matrix_free (uvj_h_h);
    gslu_matrix_free (uvip_h);
    gslu_matrix_free (uvip);

    return ret_sba;
}


int
vis_sba_2v_h_from_model (gsl_matrix *H, const gsl_matrix *uvi_h, const gsl_matrix *uvj_h,
                         const gsl_matrix *K, const perllcm_van_feature_t *fi, const gsl_vector *X_c2c1,
                         gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, gsl_vector *params, int verbose,
                         GMutex *sba_mutex, const gsl_vector *featscalei_h, const gsl_vector *featscalej_h)
{
    gsl_vector *distCoeffs = NULL;
    return vis_sba_2v_h_nonlin_from_model (H, distCoeffs, uvi_h, uvj_h, K, fi, X_c2c1, rel_pose21, rel_pose_cov21, params, verbose, sba_mutex, featscalei_h, featscalej_h);
}

int32_t
vis_sba_init_relorient (const gsl_matrix *K, const gsl_matrix *uv1, const gsl_matrix *uv2,
                        const gsl_vector *x21_nav, const gsl_matrix *p21_nav,
                        double min_dist, double max_dist,
                        const double thresh, int verbose, gsl_vector *x21_o)
{
    int32_t errmsg = PERLLCM_VAN_VLINK_T_MSG_NO_ERROR;

    // triangulation constraint before running horn's algorithm
    size_t n = uv1->size2;

    // get navigation prior ready
    gsl_vector_const_view t_nav = gsl_vector_const_subvector (x21_nav,0,3);
    gsl_vector_const_view rph_nav = gsl_vector_const_subvector (x21_nav,3,3);
    GSLU_MATRIX_VIEW (R_nav, 3,3);
    so3_rotxyz (R_nav.matrix.data, rph_nav.vector.data);
    double scale = gslu_vector_norm (&t_nav.vector);
    GSLU_MATRIX_VIEW (nav_cov_inv, 6, 6);
    gslu_matrix_inv (&nav_cov_inv.matrix, p21_nav);


    // initial guess to be computed
    GSLU_MATRIX_VIEW (R_o, 3,3);
    GSLU_VECTOR_VIEW (t_o, 3);

    /* double E = GSL_POSINF; */
    size_t minpt = 10;

    // triangulate
    vis_triangulate_t *tri = vis_triangulate_alloc (K, &R_nav.matrix, &t_nav.vector, uv1, uv2);

    // enforce triangulation constraint
    gslu_index *tri_const_idx = vis_triangulate_constraint_alloc (tri->X1, 2, min_dist, max_dist);

    if (tri_const_idx)
    {
        size_t n_accepted = tri_const_idx->size;
        if (n_accepted < minpt)
        {
            printf ("[twoview]    ERROR: NPTS (tri-init): npts (%zu) < required (%zu: %g~%g\n)", n_accepted, minpt, min_dist, max_dist);
#ifdef SBA_VERBOSE
            gslu_vector_printf (x21_nav, "x21_nav");
            gslu_matrix_printf (tri->X1, "X1");
#endif
            errmsg = PERLLCM_VAN_VLINK_T_MSG_TRI_CONST;
            //gslu_index_printf (tri_const_idx,"idx");
        }
        else if (n_accepted == n)
        {
            /* E = vis_epi_relorient_horn (K, uv1, uv2, &R_nav.matrix, &R_o.matrix, &t_o.vector, verbose); */
            vis_epi_relorient_horn (K, uv1, uv2, &R_nav.matrix, &R_o.matrix, &t_o.vector, verbose);
            gsl_vector_scale (&t_o.vector, scale);
            ssc_pose_set_Rt_gsl (x21_o, &R_o.matrix, &t_o.vector);
        }
        else
        {
            gsl_matrix *uv1_triconst = gslu_matrix_selcol_alloc (uv1, tri_const_idx);
            gsl_matrix *uv2_triconst = gslu_matrix_selcol_alloc (uv2, tri_const_idx);

            /* E = vis_epi_relorient_horn (K, uv1_triconst, uv2_triconst, &R_nav.matrix, &R_o.matrix, &t_o.vector, verbose); */
            vis_epi_relorient_horn (K, uv1_triconst, uv2_triconst, &R_nav.matrix, &R_o.matrix, &t_o.vector, verbose);
            gsl_vector_scale (&t_o.vector, scale);
            ssc_pose_set_Rt_gsl (x21_o, &R_o.matrix, &t_o.vector);

            gslu_matrix_free (uv1_triconst);
            gslu_matrix_free (uv2_triconst);
        }
    }
    else
    {
        printf ("[twoview]    ERROR: NPTS (tri-init): npts (%d) < required (%zu: %g~%g\n)", 0, minpt, min_dist, max_dist);
        errmsg = PERLLCM_VAN_VLINK_T_MSG_TRI_CONST;
#ifdef SBA_VERBOSE
        gslu_vector_printf (x21_nav, "x21_nav");
        gslu_matrix_printf (p21_nav, "p21_nav");
        gslu_matrix_printf (tri->X1, "X1");
#endif
    }

    gslu_index_free (tri_const_idx);
    vis_triangulate_free (tri);

    return errmsg;
}
