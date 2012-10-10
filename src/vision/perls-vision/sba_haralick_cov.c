#include "sba_haralick_cov.h"

#include "perls-vision/sba.h"
#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-math/gsl_util_matrix.h"
#include "perls-math/gsl_util_vector.h"
#include "perls-math/gsl_util_blas.h"

#include <glib.h>
#include <gsl/gsl_statistics.h>

#define POOL_THREADS_MAX 10

typedef struct ThreadData {
    int row;
    int col;
    gsl_matrix *u1;
    gsl_matrix *u2;
    gsl_matrix *K;
    gsl_vector *params;
    gsl_vector *distCoeffs;
    gsl_vector *featscalei;
    gsl_vector *featscalej;
} ThreadData;

typedef struct MatrixData {
    gsl_matrix *dg_dx;
    GMutex *mutex;
} MatrixData;

static gsl_vector *
stackCalibAlloc (const gsl_matrix *K, const gsl_vector *distCoeffs)
{
    gsl_vector *stacked = gsl_vector_calloc (9);
    
    double fx, fy, cx, cy, k1, k2, p1, p2, k3;

    /* get the values from K */
    fx = gsl_matrix_get (K, 0, 0);
    fy = gsl_matrix_get (K, 1, 1);
    cx = gsl_matrix_get (K, 0, 2);
    cy = gsl_matrix_get (K, 1, 2);

    /* get the values from distCoeffs */
    k1 = gsl_vector_get (distCoeffs, 0);
    k2 = gsl_vector_get (distCoeffs, 1);
    p1 = gsl_vector_get (distCoeffs, 2);
    p2 = gsl_vector_get (distCoeffs, 3);
    k3 = gsl_vector_get (distCoeffs, 4);

    /* set the values */
    gsl_vector_set (stacked, 0, fx);
    gsl_vector_set (stacked, 1, fy);
    gsl_vector_set (stacked, 2, cx);
    gsl_vector_set (stacked, 3, cy);
    gsl_vector_set (stacked, 4, k1);
    gsl_vector_set (stacked, 5, k2);
    gsl_vector_set (stacked, 6, p1);
    gsl_vector_set (stacked, 7, p2);
    gsl_vector_set (stacked, 8, k3);

    return stacked;
}

static void
unstackCalibAlloc (const gsl_vector *stacked, gsl_matrix **K, gsl_vector **distCoeffs)
{
    double fx, fy, cx, cy, k1, k2, p1, p2, k3;
    *K = gsl_matrix_calloc (3, 3);
    *distCoeffs = gsl_vector_calloc (5);

    /* get the values */
    fx = gsl_vector_get (stacked, 0);
    fy = gsl_vector_get (stacked, 1);
    cx = gsl_vector_get (stacked, 2);
    cy = gsl_vector_get (stacked, 3);
    k1 = gsl_vector_get (stacked, 4);
    k2 = gsl_vector_get (stacked, 5);
    p1 = gsl_vector_get (stacked, 6);
    p2 = gsl_vector_get (stacked, 7);
    k3 = gsl_vector_get (stacked, 8);

    /* set the K matrix */
    gsl_matrix_set (*K, 0, 0, fx);
    gsl_matrix_set (*K, 1, 1, fy);
    gsl_matrix_set (*K, 0, 2, cx);
    gsl_matrix_set (*K, 1, 2, cy);
    gsl_matrix_set (*K, 2, 2, 1.0);
    
    /* set the distortion coefficients */
    gsl_vector_set (*distCoeffs, 0, k1);
    gsl_vector_set (*distCoeffs, 1, k2);
    gsl_vector_set (*distCoeffs, 2, p1);
    gsl_vector_set (*distCoeffs, 3, p2);
    gsl_vector_set (*distCoeffs, 4, k3);
    
}

static gsl_vector *
stackFeaturesAlloc (const gsl_matrix *u1Observed, const gsl_matrix *u2Observed)
{
    assert (u1Observed->size1 == u2Observed->size1 && u1Observed->size2 == u2Observed->size2);

    int numberOfFeatures = u1Observed->size2;
    int numberOfElements = 2*u1Observed->size1*numberOfFeatures;
    
    gsl_vector *stackedFeatures = gsl_vector_calloc (numberOfElements);

    int stackedVectorIdx = 0;
    for (int featIdx=0; featIdx<numberOfFeatures; featIdx++) {
        gsl_vector_const_view ithFeature1 = gsl_matrix_const_column (u1Observed, featIdx);
        gsl_vector_const_view ithFeature2 = gsl_matrix_const_column (u2Observed, featIdx);
        gsl_vector_view stackedSubVector = gsl_vector_subvector (stackedFeatures, stackedVectorIdx, 2);
        gsl_vector_memcpy (&stackedSubVector.vector, &ithFeature1.vector);
        stackedSubVector = gsl_vector_subvector (stackedFeatures, stackedVectorIdx+2, 2);
        gsl_vector_memcpy (&stackedSubVector.vector, &ithFeature2.vector);
        stackedVectorIdx += 4;
    }

    return stackedFeatures;
}

static gsl_matrix *
sigmaUInvAlloc (const gsl_vector *featscalei,
                const gsl_vector *featscalej)
{
    gsl_matrix *sigmaUInv = gsl_matrix_calloc (4*featscalei->size, 4*featscalei->size);

    for (int i=0; i<featscalei->size; i++) {
        double scalei = gsl_vector_get (featscalei, i);
        double scalej = gsl_vector_get (featscalej, i);
        gsl_matrix_set (sigmaUInv, i*4+0, i*4+0, 1/(scalei*scalei));
        gsl_matrix_set (sigmaUInv, i*4+1, i*4+1, 1/(scalei*scalei));
        gsl_matrix_set (sigmaUInv, i*4+2, i*4+2, 1/(scalej*scalej));
        gsl_matrix_set (sigmaUInv, i*4+3, i*4+3, 1/(scalej*scalej));
    }

    return sigmaUInv;
}

static double
costFunctionRae (const gsl_vector *params,
                 const gsl_vector *stackedCalib,
                 const gsl_matrix *u1Observed,
                 const gsl_matrix *u2Observed,
                 const gsl_vector *featscalei,
                 const gsl_vector *featscalej)
{
    assert (featscalei->size == featscalej->size);
    gsl_matrix *K = NULL;
    gsl_vector *distCoeffs = NULL;
    gsl_matrix *sigmaUInv = sigmaUInvAlloc (featscalei, featscalej);

    unstackCalibAlloc (stackedCalib, &K, &distCoeffs);

    int npts = u1Observed->size2;
    gsl_vector *predictedMeasurements = gsl_vector_calloc (4*npts);

    vis_sba_img_proj_Rae (params, K, distCoeffs, npts, 2, 5, 3, 2, predictedMeasurements);

    gsl_vector *measurements = stackFeaturesAlloc (u1Observed, u2Observed);

    gsl_vector *diff = gsl_vector_calloc (measurements->size);
    gsl_vector_memcpy (diff, measurements);
    gsl_vector_sub (diff, predictedMeasurements);

    double cost = 0;
    /* equivalent for gslu_blas_vTmv for diagonal m */
    for (int i=0; i<sigmaUInv->size1; i++)
        cost += gsl_vector_get (diff, i) * gsl_vector_get (diff, i) * gsl_matrix_get (sigmaUInv, i, i);

    gsl_matrix_free (sigmaUInv);
    gsl_vector_free (measurements);
    gsl_vector_free (predictedMeasurements);
    gsl_vector_free (diff);
    gsl_vector_free (distCoeffs);
    gsl_matrix_free (K);

    return cost;
}

static double
costFunctionH (const gsl_vector *params,
               const gsl_vector *stackedCalib,
               const gsl_matrix *u1Observed,
               const gsl_matrix *u2Observed,
               const gsl_vector *featscalei,
               const gsl_vector *featscalej)
{
    assert (featscalei->size == featscalej->size);
    gsl_matrix *K = NULL;
    gsl_vector *distCoeffs = NULL;
    gsl_matrix *sigmaUInv = sigmaUInvAlloc (featscalei, featscalej);

    unstackCalibAlloc (stackedCalib, &K, &distCoeffs);

    int npts = u1Observed->size2;
    gsl_vector *predictedMeasurements = gsl_vector_calloc (4*npts);

    vis_sba_img_proj_H (params, K, distCoeffs, npts, 2, 8, 3, 2, predictedMeasurements);

    gsl_vector *measurements = stackFeaturesAlloc (u1Observed, u2Observed);

    gsl_vector *diff = gsl_vector_calloc (measurements->size);
    gsl_vector_memcpy (diff, measurements);
    gsl_vector_sub (diff, predictedMeasurements);

    double cost = 0;
    /* equivalent for gslu_blas_vTmv for diagonal m */
    for (int i=0; i<sigmaUInv->size1; i++)
        cost += gsl_vector_get (diff, i) * gsl_vector_get (diff, i) * gsl_matrix_get (sigmaUInv, i, i);

    gsl_matrix_free (sigmaUInv);
    gsl_vector_free (measurements);
    gsl_vector_free (predictedMeasurements);
    gsl_vector_free (diff);
    gsl_vector_free (distCoeffs);
    gsl_matrix_free (K);

    return cost;
}

/* Useful for ignoring errors when inverting matrices */
static void
doNothingHandler (const char * reason,
                  const char * file,
                  int line,
                  int gsl_errno)
{
    /* pass */
}

static void
printJacob (gsl_matrix *J)
{
    char *s = malloc (128*sizeof(*s));
    sprintf(s, "/home/paul/jacob_%zu_%zu", J->size1, J->size2);
    FILE *fp = fopen(s, "w");
    gsl_matrix_fprintf(fp, J, "%.60f");
    fflush(fp);
    free (s);
}

static void
printRelPose (gsl_matrix *Sigma)
{
    gsl_matrix_view relPose = gsl_matrix_submatrix (Sigma, 0, 0, 5, 5);
    gslu_matrix_printf (&relPose.matrix, "relPose");
}

static void
vis_sba_haralick_cov_hz_Rae (const gsl_vector *params, const gsl_matrix *K,
                             const gsl_vector *distCoeffs, 
                             const gsl_vector *featscalei, const gsl_vector *featscalej, 
                             gsl_matrix *hzCovEst)
{
    int ret, ncam;
    gsl_matrix *JTmp, *J, *JTSJ;
    gsl_matrix_view JSubView;
    gsl_error_handler_t *oldHandler;

    ncam = 2;

    /* The JTmp Jacobian will have 5 left-most columns all equal to zero */
    JTmp = vis_sba_img_proj_Rae_jacob_alloc (params, K, distCoeffs, ncam);

    J = gsl_matrix_calloc (JTmp->size1, JTmp->size2 - 5);
    JSubView = gsl_matrix_submatrix (JTmp, 0, 5, J->size1, J->size2);
    gsl_matrix_memcpy (J, &JSubView.matrix);

    JTSJ = gsl_matrix_calloc (J->size2, J->size2);
    gsl_matrix *sigmaUInv = sigmaUInvAlloc (featscalei, featscalej);
    gslu_blas_mTmm (JTSJ, J, sigmaUInv, J, NULL);

    /* Set error handler */
    oldHandler = gsl_set_error_handler (doNothingHandler);
    
    ret = gslu_matrix_spdinv(hzCovEst, JTSJ);

    if (ret) {
        printf ("Matrix is not invertible\n");
        goto ON_ERROR;
    } 

  ON_ERROR:
    gsl_matrix_free (JTmp);
    gsl_matrix_free (J);
    gsl_matrix_free (JTSJ);
    gsl_matrix_free (sigmaUInv);

    gsl_set_error_handler (oldHandler);
}

static void
vis_sba_haralick_cov_hz_H (const gsl_vector *params, const gsl_matrix *K,
                           const gsl_vector *distCoeffs, 
                           const gsl_vector *featscalei, const gsl_vector *featscalej,
                           gsl_matrix *hzCovEst)
{
    int ret, ncam;
    gsl_matrix *JTmp, *J, *JTSJ;
    gsl_matrix_view JSubView;
    gsl_error_handler_t *oldHandler;

    ncam = 2;

    /* The JTmp Jacobian will have 5 left-most columns all equal to zero */
    JTmp = vis_sba_img_proj_H_jacob_alloc (params, K, distCoeffs, ncam);

    J = gsl_matrix_calloc (JTmp->size1, JTmp->size2 - 8);
    JSubView = gsl_matrix_submatrix (JTmp, 0, 8, J->size1, J->size2);
    gsl_matrix_memcpy (J, &JSubView.matrix);

    JTSJ = gsl_matrix_calloc (J->size2, J->size2);
    gsl_matrix *sigmaUInv = sigmaUInvAlloc (featscalei, featscalej);
    gslu_blas_mTmm (JTSJ, J, sigmaUInv, J, NULL);

    /* Set error handler */
    oldHandler = gsl_set_error_handler (doNothingHandler);
    
    ret = gslu_matrix_spdinv(hzCovEst, JTSJ);

    if (ret) {
        printf ("Matrix is not invertible\n");
        goto ON_ERROR;
    } 

  ON_ERROR:
    gsl_matrix_free (JTmp);
    gsl_matrix_free (J);
    gsl_matrix_free (JTSJ);
    gsl_matrix_free (sigmaUInv);

    gsl_set_error_handler (oldHandler);
}

/* Computes the hessian at (row, col) for essential matrix */
static void
threadFuncRae (gpointer data, gpointer user_data)
{
    ThreadData *threadData = data;
    MatrixData *matrixData = user_data;

    int row = threadData->row;
    int col = threadData->col;

    gsl_vector *stackedCalib = stackCalibAlloc (threadData->K, threadData->distCoeffs);
    gsl_vector *stackedCalibPerturb = gsl_vector_calloc (stackedCalib->size);
    gsl_vector_memcpy (stackedCalibPerturb, stackedCalib);

    gsl_vector *paramsPerturb = gsl_vector_calloc (threadData->params->size);
    gsl_vector_memcpy (paramsPerturb, threadData->params);

    double paramWrt = gsl_vector_get (paramsPerturb, col);
    double calibWrt = gsl_vector_get (stackedCalibPerturb, row);

    /* The differentiation delta must be sufficiently large */
    double deltaX, deltaTheta;
    deltaX = 1e-3;
    deltaTheta = 1e-5;

    gsl_vector_set (paramsPerturb, col, paramWrt + deltaTheta);
    gsl_vector_set (stackedCalibPerturb, row, calibWrt + deltaX);

    /* Compute the differentiation */
    double term1, term2, term3, term4, secondDeriv;
    term1 = costFunctionRae (paramsPerturb, stackedCalibPerturb,
                             threadData->u1, threadData->u2, 
                             threadData->featscalei, threadData->featscalej);
    term2 = costFunctionRae (paramsPerturb, stackedCalib,
                             threadData->u1, threadData->u2,
                             threadData->featscalei, threadData->featscalej);
    term3 = costFunctionRae (threadData->params, stackedCalibPerturb,
                             threadData->u1, threadData->u2,
                             threadData->featscalei, threadData->featscalej);
    term4 = costFunctionRae (threadData->params, stackedCalib,
                             threadData->u1, threadData->u2,
                             threadData->featscalei, threadData->featscalej);
    
    secondDeriv = (term1 - term2 - term3 + term4) / (deltaX * deltaTheta);

    /* Set the value at (row, col) */
    gsl_matrix *dg_dx = matrixData->dg_dx;
    g_mutex_lock (matrixData->mutex);
    gsl_matrix_set (dg_dx, row, col, secondDeriv);
    g_mutex_unlock (matrixData->mutex);

    /* Thread is done - clean up */
    gsl_matrix_free (threadData->u1);
    gsl_matrix_free (threadData->u2);
    gsl_matrix_free (threadData->K);
    gsl_vector_free (threadData->distCoeffs);
    gsl_vector_free (threadData->params);
    gsl_vector_free (threadData->featscalei);
    gsl_vector_free (threadData->featscalej);
    gsl_vector_free (paramsPerturb);
    gsl_vector_free (stackedCalib);
    gsl_vector_free (stackedCalibPerturb);
    free (threadData);

}

/* Computes the hessian at (row, col) for essential matrix */
static void
threadFuncH (gpointer data, gpointer user_data)
{
    ThreadData *threadData = data;
    MatrixData *matrixData = user_data;

    int row = threadData->row;
    int col = threadData->col;

    gsl_vector *stackedCalib = stackCalibAlloc (threadData->K, threadData->distCoeffs);
    gsl_vector *stackedCalibPerturb = gsl_vector_calloc (stackedCalib->size);
    gsl_vector_memcpy (stackedCalibPerturb, stackedCalib);

    gsl_vector *paramsPerturb = gsl_vector_calloc (threadData->params->size);
    gsl_vector_memcpy (paramsPerturb, threadData->params);

    double paramWrt = gsl_vector_get (paramsPerturb, col);
    double calibWrt = gsl_vector_get (stackedCalibPerturb, row);

    /* The differentiation delta must be sufficiently large */
    double deltaX, deltaTheta;
    deltaX = row > 3 ? 1e-3 : .1;
    /* deltaTheta = 1e-5; */
    /* deltaX = 1e-4; */
    /* deltaX = 1e-2; */
    deltaTheta = 1e-3;

    gsl_vector_set (paramsPerturb, col, paramWrt + deltaTheta);
    gsl_vector_set (stackedCalibPerturb, row, calibWrt + deltaX);

    /* Compute the differentiation */
    double term1, term2, term3, term4, secondDeriv;
    term1 = costFunctionH (paramsPerturb, stackedCalibPerturb,
                           threadData->u1, threadData->u2,
                           threadData->featscalei, threadData->featscalej);
    term2 = costFunctionH (paramsPerturb, stackedCalib,
                           threadData->u1, threadData->u2,
                           threadData->featscalei, threadData->featscalej);
    term3 = costFunctionH (threadData->params, stackedCalibPerturb,
                           threadData->u1, threadData->u2,
                           threadData->featscalei, threadData->featscalej);
    term4 = costFunctionH (threadData->params, stackedCalib,
                           threadData->u1, threadData->u2,
                           threadData->featscalei, threadData->featscalej);
    secondDeriv = (term1 - term2 - term3 + term4) / (deltaX * deltaTheta);

    /* Set the value at (row, col) */
    gsl_matrix *dg_dx = matrixData->dg_dx;
    g_mutex_lock (matrixData->mutex);
    gsl_matrix_set (dg_dx, row, col, secondDeriv);
    g_mutex_unlock (matrixData->mutex);

    /* Thread is done - clean up */
    gsl_matrix_free (threadData->u1);
    gsl_matrix_free (threadData->u2);
    gsl_matrix_free (threadData->K);
    gsl_vector_free (threadData->distCoeffs);
    gsl_vector_free (threadData->params);
    gsl_vector_free (threadData->featscalei);
    gsl_vector_free (threadData->featscalej);
    gsl_vector_free (paramsPerturb);
    gsl_vector_free (stackedCalib);
    gsl_vector_free (stackedCalibPerturb);
    free (threadData);

}

static gsl_matrix *
computeATermRaeAlloc (const gsl_vector *params, const gsl_matrix *u1, 
                      const gsl_matrix *u2, const gsl_matrix *K,
                      const gsl_vector *distCoeffs,
                      const gsl_vector *featscalei, const gsl_vector *featscalej)
{
    /* Create the dg_dx matrix */
    int dg_dx_rows = distCoeffs->size + 4; /* 9 total */
    int dg_dx_cols = 10 + 3*u1->size2;
    gsl_matrix *dg_dx = gsl_matrix_calloc (dg_dx_rows, dg_dx_cols);
    MatrixData *user_data = calloc(1, sizeof(*user_data));
    user_data->dg_dx = dg_dx;
    user_data->mutex = g_mutex_new ();

    GThreadPool *pool = g_thread_pool_new (threadFuncRae, user_data, POOL_THREADS_MAX, TRUE, NULL);

    /* Push every (row, col) element onto the thread pool */
    for (int row=0; row<dg_dx_rows; row++) {
        for (int col=0; col<dg_dx_cols; col++) {

            ThreadData *threadData = calloc (1, sizeof(*threadData));
            threadData->row = row;
            threadData->col = col;
            threadData->params = gsl_vector_calloc (params->size);
            threadData->distCoeffs = gsl_vector_calloc (distCoeffs->size);
            threadData->u1 = gsl_matrix_calloc (u1->size1, u1->size2);
            threadData->u2 = gsl_matrix_calloc (u2->size1, u2->size2);
            threadData->K = gsl_matrix_calloc (K->size1, K->size2);
            threadData->featscalei = gsl_vector_calloc (featscalei->size);
            threadData->featscalej = gsl_vector_calloc (featscalej->size);

            gsl_vector_memcpy (threadData->params, params);
            gsl_vector_memcpy (threadData->distCoeffs, distCoeffs);
            gsl_matrix_memcpy (threadData->u1, u1);
            gsl_matrix_memcpy (threadData->u2, u2);
            gsl_matrix_memcpy (threadData->K, K);
            gsl_vector_memcpy (threadData->featscalei, featscalei);
            gsl_vector_memcpy (threadData->featscalej, featscalej);

            g_thread_pool_push (pool, threadData, NULL);

        }
    }
    
    /* Wait for all the jobs to finish */
    g_thread_pool_free (pool, FALSE, TRUE);

    /* Take out the first camera's 5-DOF pose (its reference frame is the origin) */
    gsl_matrix *A = gsl_matrix_calloc (dg_dx->size1, dg_dx->size2-5);
    gsl_matrix_view dg_dx_A = gsl_matrix_submatrix (dg_dx, 0, 5, A->size1, A->size2);
    gsl_matrix_memcpy (A, &dg_dx_A.matrix);

    /* clean up */
    gsl_matrix_free (user_data->dg_dx);
    g_mutex_free (user_data->mutex);
    free (user_data);
    return A;
}

static gsl_matrix *
computeATermHAlloc (const gsl_vector *params, const gsl_matrix *u1, 
                    const gsl_matrix *u2, const gsl_matrix *K,
                    const gsl_vector *distCoeffs,
                    const gsl_vector *featscalei, const gsl_vector *featscalej)
{
    /* Create the dg_dx matrix */
    int dg_dx_rows = distCoeffs->size + 4; /* 9 total */
    int dg_dx_cols = 16 + 2*u1->size2;
    gsl_matrix *dg_dx = gsl_matrix_calloc (dg_dx_rows, dg_dx_cols);
    MatrixData *user_data = calloc(1, sizeof(*user_data));
    user_data->dg_dx = dg_dx;
    user_data->mutex = g_mutex_new ();

    GThreadPool *pool = g_thread_pool_new (threadFuncH, user_data, POOL_THREADS_MAX, TRUE, NULL);

    /* Push every (row, col) element onto the thread pool */
    for (int row=0; row<dg_dx_rows; row++) {
        for (int col=0; col<dg_dx_cols; col++) {

            ThreadData *threadData = calloc (1, sizeof(*threadData));
            threadData->row = row;
            threadData->col = col;
            threadData->params = gsl_vector_calloc (params->size);
            threadData->distCoeffs = gsl_vector_calloc (distCoeffs->size);
            threadData->u1 = gsl_matrix_calloc (u1->size1, u1->size2);
            threadData->u2 = gsl_matrix_calloc (u2->size1, u2->size2);
            threadData->K = gsl_matrix_calloc (K->size1, K->size2);
            threadData->featscalei = gsl_vector_calloc (featscalei->size);
            threadData->featscalej = gsl_vector_calloc (featscalej->size);

            gsl_vector_memcpy (threadData->params, params);
            gsl_vector_memcpy (threadData->distCoeffs, distCoeffs);
            gsl_matrix_memcpy (threadData->u1, u1);
            gsl_matrix_memcpy (threadData->u2, u2);
            gsl_matrix_memcpy (threadData->K, K);
            gsl_vector_memcpy (threadData->featscalei, featscalei);
            gsl_vector_memcpy (threadData->featscalej, featscalej);

            g_thread_pool_push (pool, threadData, NULL);

        }
    }
    
    /* Wait for all the jobs to finish */
    g_thread_pool_free (pool, FALSE, TRUE);

    /* Take out the first camera's 5-DOF pose (its reference frame is the origin) */
    gsl_matrix *A = gsl_matrix_calloc (dg_dx->size1, dg_dx->size2-8);
    gsl_matrix_view dg_dx_A = gsl_matrix_submatrix (dg_dx, 0, 8, A->size1, A->size2);
    gsl_matrix_memcpy (A, &dg_dx_A.matrix);

    /* clean up */
    gsl_matrix_free (user_data->dg_dx);
    g_mutex_free (user_data->mutex);
    free (user_data);
    return A;
}

static void
haralickEquation (const gsl_matrix *hz, const gsl_matrix *A, const gsl_matrix *SigmaCalib, gsl_matrix *haralickCovEst)
{
    gsl_matrix *tmp = gsl_matrix_calloc (hz->size1, hz->size2);
    gsl_matrix_memcpy (tmp, hz);

    gsl_matrix *B2 = gslu_blas_mmT_alloc (hz, A);

    gsl_matrix *BSigmaBT = gslu_blas_mmmT_alloc (B2, SigmaCalib, B2, NULL);
    gsl_matrix_scale (BSigmaBT, .25);

    gsl_matrix_add (tmp, BSigmaBT);
    gsl_matrix_memcpy (haralickCovEst, tmp);

    gsl_matrix_free (tmp);
    gsl_matrix_free (B2);
    gsl_matrix_free (BSigmaBT);
}

void
vis_sba_haralick_cov_2v_Rae (const gsl_matrix *sigmaCalib,
                             const gsl_vector *params, const gsl_matrix *u1,
                             const gsl_matrix *u2, const gsl_matrix *K,
                             const gsl_vector *distCoeffs, 
                             const gsl_vector *featscalei, const gsl_vector *featscalej,
                             gsl_matrix *rel_pose_cov)
{
    assert (u1->size2 == u2->size2);
    int npts = u1->size2;

    /* Compute the "A" term (see doxygen for description with latex) */
    gsl_matrix *A = computeATermRaeAlloc (params, u1, u2, K, distCoeffs, featscalei, featscalej);

    /* Compute hartley/zisserman covariance estimate : inv(JT inv(Sigma) J) */
    gsl_matrix *hzCovEst = gsl_matrix_calloc (5 + 3*npts, 5 + 3*npts);
    vis_sba_haralick_cov_hz_Rae (params, K, distCoeffs, featscalei, featscalej, hzCovEst);

    /* Do the matrix calculations */
    gsl_matrix *haralickCovEst = gsl_matrix_calloc (hzCovEst->size1, hzCovEst->size2);
    haralickEquation (hzCovEst, A, sigmaCalib, haralickCovEst);

    gsl_matrix_view rel_pose_cov_view = gsl_matrix_submatrix (haralickCovEst, 0, 0, 5, 5);
    gsl_matrix_memcpy (rel_pose_cov, &rel_pose_cov_view.matrix);

    gsl_matrix_free (hzCovEst);
    gsl_matrix_free (haralickCovEst);
    gsl_matrix_free (A);
}

void
vis_sba_haralick_cov_2v_H (const gsl_matrix *sigmaCalib,
                           const gsl_vector *params, const gsl_matrix *u1,
                           const gsl_matrix *u2, const gsl_matrix *K,
                           const gsl_vector *distCoeffs, 
                           const gsl_vector *featscalei, const gsl_vector *featscalej,
                           gsl_matrix *rel_pose_cov)
{
    assert (u1->size2 == u2->size2);
    int npts = u1->size2;

    /* Compute hartley/zisserman covariance estimate : inv(JT inv(Sigma) J) */
    gsl_matrix *hzCovEst = gsl_matrix_calloc (8 + 2*npts, 8 + 2*npts);
    vis_sba_haralick_cov_hz_H (params, K, distCoeffs, featscalei, featscalej, hzCovEst);

    /* Compute the "A" term (see doxygen for description with latex) */
    gsl_matrix *A = computeATermHAlloc (params, u1, u2, K, distCoeffs, featscalei, featscalej);

    /* Do the matrix calculations */
    gsl_matrix *haralickCovEst = gsl_matrix_calloc (hzCovEst->size1, hzCovEst->size2);
    haralickEquation (hzCovEst, A, sigmaCalib, haralickCovEst);

    gsl_matrix_view rel_pose_cov_view = gsl_matrix_submatrix (haralickCovEst, 0, 0, 5, 5);
    gsl_matrix_memcpy (rel_pose_cov, &rel_pose_cov_view.matrix);

    gsl_matrix_free (hzCovEst);
    gsl_matrix_free (haralickCovEst);
    gsl_matrix_free (A);
}
