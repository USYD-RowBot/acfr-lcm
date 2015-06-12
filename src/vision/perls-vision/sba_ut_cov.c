#include "sba_ut_cov.h"

#include "perls-vision/sba.h"
#include "perls-vision/distortion.h"
#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-math/unscented_transform.h"
#include "perls-math/gsl_util_matrix.h"
#include "perls-math/gsl_util_vector.h"
#include "perls-math/gsl_util_blas.h"
#include "perls-math/dm.h"

#include <glib.h>

#define UT_POOL_THREADS_MAX 1

typedef struct ThreadDataRae
{
    gsl_matrix *u1_dist;
    gsl_matrix *u2_dist;
    gsl_vector *x21;
    gsl_vector *sigmaPoint;     /* represents calibration values */
    int index;                  /* so we know which sba output corresponds to which sigma
                                 * point */
} ThreadDataRae;


typedef struct ThreadDataH
{
    gsl_matrix *u1_dist;
    gsl_matrix *u2_dist;
    gsl_vector *x21;
    gsl_matrix *H;
    const perllcm_van_feature_t *fi;
    double d;
    gsl_vector *sigmaPoint;     /* represents calibration values */
    gsl_vector *featscalei;
    gsl_vector *featscalej;
    int index;                  /* so we know which sba output corresponds to which sigma
                                 * point */
} ThreadDataH;

typedef struct CommonData
{
    gsl_matrix *sbaOutputs;
    GMutex *mutex;
    GMutex *sbaMutex;
} CommonData;

/* Placeholder to allocate a sample calibration covariance if the user does not provide
 * one */
static gsl_matrix *
placeholderSigmaCalibAlloc()
{
    gsl_matrix *SigmaCalib = gsl_matrix_calloc (9, 9);
    gsl_vector_view _SigmaCalibDiag = gsl_matrix_diagonal (SigmaCalib);

    gsl_vector *SigmaCalibDiag = &_SigmaCalibDiag.vector;
    gsl_vector_set (SigmaCalibDiag, 0, 40.234979301808160);
    gsl_vector_set (SigmaCalibDiag, 1, 38.822745693779552);
    gsl_vector_set (SigmaCalibDiag, 2, 9.259693532351305 );
    gsl_vector_set (SigmaCalibDiag, 3, 27.625311265943740);
    gsl_vector_set (SigmaCalibDiag, 4, 0.000032012333624);
    gsl_vector_set (SigmaCalibDiag, 5, 0.000877222378770);
    gsl_vector_set (SigmaCalibDiag, 6, 0.000000506618851);
    gsl_vector_set (SigmaCalibDiag, 7, 0.000000382538938);
    gsl_vector_set (SigmaCalibDiag, 8, 0.002532289434731);

    return SigmaCalib;
}

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
hauvDump (const gsl_vector *params, int npts, const gsl_matrix *u1, const gsl_matrix *u2, const gsl_matrix *K,
          const gsl_vector *distCoeffs, const gsl_matrix *H)
{
    FILE *fp;
    char *s;
    s = malloc(128*sizeof(*s));

    char *homeDir = getenv("HOME");

    printf ("Dumping data.  %d points\n", npts);

    {
        sprintf (s, "%s/hauv-dump/matlab/Theta_%zu", homeDir, params->size);
        fp = fopen(s, "w");
        if (fp == NULL)
            PERROR ("Could not open file %s", s);
        gsl_vector_fprintf (fp, params, "%.60f");
        fclose(fp);

        sprintf (s, "%s/hauv-dump/matlab/u1_%zu_%zu", homeDir, u1->size1, u1->size2);
        fp = fopen(s, "w");
        gsl_matrix_fprintf (fp, u1, "%.60f");
        fclose(fp);

        sprintf (s, "%s/hauv-dump/matlab/u2_%zu_%zu", homeDir, u2->size1, u2->size2);
        fp = fopen(s, "w");
        gsl_matrix_fprintf (fp, u2, "%.60f");
        fclose(fp);

        gsl_vector *calibParams = stackCalibAlloc (K, distCoeffs);

        sprintf (s, "%s/hauv-dump/matlab/calibParams_%zu", homeDir, calibParams->size);
        fp = fopen(s, "w");
        gsl_vector_fprintf (fp, calibParams, "%.60f");
        fclose(fp);
        gsl_vector_free (calibParams);

        sprintf (s, "%s/hauv-dump/matlab/H_%zu_%zu", homeDir, H->size1, H->size2);
        fp = fopen(s, "w");
        gsl_matrix_fprintf (fp, H, "%.60f");
        fclose(fp);
    }

    {
        sprintf (s, "%s/hauv-dump/c/Theta", homeDir);
        fp = fopen(s, "w");
        if (fp == NULL)
            PERROR ("Could not open file %s", s);
        gsl_vector_fwrite (fp, params);
        fclose(fp);

        sprintf (s, "%s/hauv-dump/c/u1", homeDir);
        fp = fopen(s, "w");
        gsl_matrix_fwrite (fp, u1);
        fclose(fp);

        sprintf (s, "%s/hauv-dump/c/u2", homeDir);
        fp = fopen(s, "w");
        gsl_matrix_fwrite (fp, u2);
        fclose(fp);

        sprintf (s, "%s/hauv-dump/c/K", homeDir);
        fp = fopen(s, "w");
        gsl_matrix_fwrite (fp, K);
        fclose(fp);

        sprintf (s, "%s/hauv-dump/c/distCoeffs", homeDir);
        fp = fopen(s, "w");
        gsl_vector_fwrite (fp, distCoeffs);
        fclose(fp);

        sprintf (s, "%s/hauv-dump/c/H", homeDir);
        fp = fopen(s, "w");
        gsl_matrix_fwrite (fp, H);
        fclose(fp);
    }

    free (s);

}

static void
threadFuncRae (gpointer data, gpointer user_data)
{
    /* run sba */
    ThreadDataRae *threadData = (ThreadDataRae *)data;
    CommonData *commonData = (CommonData *)user_data;

    gsl_matrix *u1_dist = threadData->u1_dist;
    gsl_matrix *u2_dist = threadData->u2_dist;
    gsl_vector *sigmaPoint = threadData->sigmaPoint;
    gsl_vector *x21 = threadData->x21;

    gsl_matrix *K = gsl_matrix_calloc (3, 3);
    gsl_vector_view distCoeffs = gsl_vector_subvector (sigmaPoint, 4, 5);

    /* assign entries to K */
    gsl_matrix_set (K, 0, 0, gsl_vector_get (sigmaPoint, 0));
    gsl_matrix_set (K, 1, 1, gsl_vector_get (sigmaPoint, 1));
    gsl_matrix_set (K, 0, 2, gsl_vector_get (sigmaPoint, 2));
    gsl_matrix_set (K, 1, 2, gsl_vector_get (sigmaPoint, 3));
    gsl_matrix_set (K, 2, 2, 1);

    /* enforce triangulation constraint */
    gsl_matrix *u1_dist_triconst, *u2_dist_triconst;
    gsl_vector *rel_pose21 = gsl_vector_alloc (5);
    gsl_matrix *rel_pose_cov21 = gsl_matrix_alloc (5,5);
    double tri_min_dist, tri_max_dist;

    tri_min_dist = 0.0;
    tri_max_dist = 10.0;

    /* DEBUG: */
    /* gslu_matrix_printf (u1_dist, "u1"); */
    /* gslu_matrix_printf (u2_dist, "u2"); */
    /* gslu_matrix_printf (K, "K"); */

    vis_sba_2v_rae_enforce_tri_const_alloc (K, x21, u1_dist, u2_dist,
                                            tri_min_dist, tri_max_dist, &u1_dist_triconst, &u2_dist_triconst);

    gsl_matrix *X1_plot = NULL;
    vis_sba_2v_rae_nonlin_from_model_with_tri (K, &distCoeffs.vector,
            x21, u1_dist_triconst, u2_dist_triconst,
            10, FALSE,
            tri_min_dist, tri_max_dist,
            rel_pose21, rel_pose_cov21, NULL, FALSE,
            &X1_plot, commonData->sbaMutex, NULL, NULL);
    gsl_matrix_free (X1_plot);

    /* add to running total to compute sample covariance */
    g_mutex_lock (commonData->mutex);
    gsl_vector_view ithColumn = gsl_matrix_column (commonData->sbaOutputs, threadData->index);
    gsl_vector_memcpy (&(ithColumn.vector), rel_pose21);
    g_mutex_unlock (commonData->mutex);

    /* clean up */
    gsl_matrix_free (K);
    gsl_vector_free (sigmaPoint);
    gsl_vector_free (x21);
    gsl_vector_free (rel_pose21);
    gsl_matrix_free (rel_pose_cov21);
    gsl_matrix_free (u1_dist);
    gsl_matrix_free (u2_dist);
    free (threadData);
}

static void
threadFuncH (gpointer data, gpointer user_data)
{
    ThreadDataH *threadData = (ThreadDataH *)data;
    CommonData *commonData = (CommonData *)user_data;

    gsl_matrix *u1_dist = threadData->u1_dist;
    gsl_matrix *u2_dist = threadData->u2_dist;
    gsl_vector *sigmaPoint = threadData->sigmaPoint;
    gsl_vector *x21 = threadData->x21;
    gsl_matrix *H = threadData->H;
    const perllcm_van_feature_t *fi = threadData->fi;
    gsl_vector *featscalei = threadData->featscalei;
    gsl_vector *featscalej = threadData->featscalej;

    gsl_matrix *K = gsl_matrix_calloc (3, 3);
    gsl_vector_view distCoeffs = gsl_vector_subvector (sigmaPoint, 4, 5);

    /* assign entries to K */
    gsl_matrix_set (K, 0, 0, gsl_vector_get (sigmaPoint, 0));
    gsl_matrix_set (K, 1, 1, gsl_vector_get (sigmaPoint, 1));
    gsl_matrix_set (K, 0, 2, gsl_vector_get (sigmaPoint, 2));
    gsl_matrix_set (K, 1, 2, gsl_vector_get (sigmaPoint, 3));
    gsl_matrix_set (K, 2, 2, 1);

    /* run sba */
    gsl_vector *rel_pose21 = gsl_vector_alloc (5);
    gsl_matrix *rel_pose_cov21 = gsl_matrix_alloc (5,5);

    /* gslu_vector_printf (x21, "x21"); */
    vis_sba_2v_h_nonlin_from_model (H, &distCoeffs.vector, u1_dist, u2_dist, K, fi,
                                    x21, rel_pose21, rel_pose_cov21, NULL, FALSE, commonData->sbaMutex, NULL, NULL);

    /* add to running total to compute sample covariance */
    g_mutex_lock (commonData->mutex);
    gsl_vector_view ithColumn = gsl_matrix_column (commonData->sbaOutputs, threadData->index);
    /* gslu_vector_printf (rel_pose21, "rel_pose21"); */
    gsl_vector_memcpy (&(ithColumn.vector), rel_pose21);
    g_mutex_unlock (commonData->mutex);

    /* clean up */
    gsl_matrix_free (K);
    gsl_vector_free (sigmaPoint);
    gsl_vector_free (x21);
    gsl_vector_free (rel_pose21);
    gsl_matrix_free (rel_pose_cov21);
    gsl_matrix_free (u1_dist);
    gsl_matrix_free (u2_dist);
    gsl_matrix_free (H);
    free (threadData);
    gsl_vector_free (featscalei);
    gsl_vector_free (featscalej);
}

void
vis_sba_ut_cov_2v_H (const gsl_matrix *sigmaCalib,
                     const gsl_vector *params, const gsl_matrix *H, const gsl_matrix *u1_dist,
                     const gsl_matrix *u2_dist, const gsl_matrix *K,
                     const gsl_vector *distCoeffs, const perllcm_van_feature_t *fi,
                     const gsl_vector *featscalei, const gsl_vector *featscalej,
                     GMutex *sbaMutex, gsl_matrix *rel_pose_cov)
{
    gsl_vector *muCalib = gsl_vector_calloc (9);

    /* DEBUG: */
    /* hauvDump (params, u1_dist->size2, u1_dist, u2_dist, K, distCoeffs, H); */

    /* intrinsic parameters */
    gsl_vector_set (muCalib, 0, gsl_matrix_get (K, 0, 0));
    gsl_vector_set (muCalib, 1, gsl_matrix_get (K, 1, 1));
    gsl_vector_set (muCalib, 2, gsl_matrix_get (K, 0, 2));
    gsl_vector_set (muCalib, 3, gsl_matrix_get (K, 1, 2));

    /* lens distortion parameters */
    gsl_vector_set (muCalib, 4, gsl_vector_get (distCoeffs, 0));
    gsl_vector_set (muCalib, 5, gsl_vector_get (distCoeffs, 1));
    gsl_vector_set (muCalib, 6, gsl_vector_get (distCoeffs, 2));
    gsl_vector_set (muCalib, 7, gsl_vector_get (distCoeffs, 3));
    gsl_vector_set (muCalib, 8, gsl_vector_get (distCoeffs, 4));

    gsl_matrix *sigmaPoints;
    gsl_vector *meanWeights, *covWeights;
    unscented_transform_alloc (muCalib, sigmaCalib, NULL, &sigmaPoints, &meanWeights, &covWeights);

    /* initialize thread pool to run sba */
    CommonData *user_data = calloc (1, sizeof(*user_data));
    user_data->sbaOutputs = gsl_matrix_calloc (5, sigmaPoints->size2);
    user_data->mutex = g_mutex_new ();
    user_data->sbaMutex = sbaMutex;
    GThreadPool *pool = g_thread_pool_new (threadFuncH, user_data, UT_POOL_THREADS_MAX, TRUE, NULL);

    /* For each sigma point, kick off new thread */
    for (int i=0; i<sigmaPoints->size2; i++)
    {

        gsl_vector_const_view ithSigmaPoint = gsl_matrix_const_column (sigmaPoints, i);

        /* Create the thread data */
        ThreadDataH *threadData = calloc (1, sizeof(*threadData));
        threadData->u1_dist = gsl_matrix_calloc (u1_dist->size1, u1_dist->size2);
        threadData->u2_dist = gsl_matrix_calloc (u2_dist->size1, u2_dist->size2);
        threadData->x21 = gsl_vector_calloc (6);
        threadData->sigmaPoint = gsl_vector_calloc (ithSigmaPoint.vector.size);
        threadData->index = i;
        threadData->H = gsl_matrix_calloc (H->size1, H->size2);
        threadData->featscalei = gsl_vector_calloc (featscalei->size);
        threadData->featscalej = gsl_vector_calloc (featscalej->size);

        /* copy to thread data struct */
        gsl_matrix_memcpy (threadData->u1_dist, u1_dist);
        gsl_matrix_memcpy (threadData->u2_dist, u2_dist);
        gsl_vector_memcpy (threadData->sigmaPoint, &(ithSigmaPoint.vector));
        gsl_matrix_memcpy (threadData->H, H);
        gsl_vector_memcpy (threadData->featscalei, featscalei);
        gsl_vector_memcpy (threadData->featscalej, featscalej);
        threadData->fi = fi;

        gsl_vector_view x21_t = gsl_vector_subvector (threadData->x21, 0, 3);
        gsl_vector_view x21_rph = gsl_vector_subvector (threadData->x21, 3, 3);
        gsl_vector_const_view x21_ae = gsl_vector_const_subvector (params, 8, 2);
        gsl_vector_const_view x21_rph_5dof = gsl_vector_const_subvector (params, 10, 3);

        gsl_vector *x21_dm = gsl_vector_calloc (3);
        gsl_vector_set (x21_dm, 0, gsl_vector_get (&x21_ae.vector, 0));
        gsl_vector_set (x21_dm, 1, gsl_vector_get (&x21_ae.vector, 1));
        gsl_vector_set (x21_dm, 2, 1.0);

        /* compute the 6dof pose as the initial guess for the thread */
        dm_dm2trans_gsl (x21_dm, &x21_t.vector, NULL);
        gsl_vector_memcpy (&x21_rph.vector, &x21_rph_5dof.vector);

        g_thread_pool_push (pool, threadData, NULL);

        /* clean up */
        gsl_vector_free (x21_dm);
    }

    /* Wait for all the threads to finish */
    g_thread_pool_free (pool, FALSE, TRUE);

    /* now that threads are done, compute the covariance estiamte using std. ut
     * techniques */
    gsl_vector *muPrime = gsl_vector_calloc (5);
    gsl_matrix *sigmaPrime = gsl_matrix_calloc (5, 5);

    /* gslu_matrix_printf (user_data->sbaOutputs, "outputs"); */
    /* gslu_vector_printf (meanWeights, "meanWeights"); */
    /* gslu_vector_printf (covWeights, "covWeights"); */

    for (int i=0; i<sigmaPoints->size2; i++)
    {
        /* muPrime = muPrime + meanWeights(i)*y(:,i) */
        gsl_vector_const_view ithColumn = gsl_matrix_const_column (user_data->sbaOutputs, i);
        gsl_vector *ithColumnScaled = gsl_vector_calloc (ithColumn.vector.size);
        gsl_vector_memcpy (ithColumnScaled, &ithColumn.vector);
        gsl_vector_scale (ithColumnScaled, gsl_vector_get (meanWeights, i));
        gsl_vector_add (muPrime, ithColumnScaled);
        gsl_vector_free (ithColumnScaled);
    }

    for (int i=0; i<sigmaPoints->size2; i++)
    {
        /* sigmaPrime = sigmaPrime + covWeight(i)*(y(:,i) - muPrime)*(y(:,i) - muPrime)' */
        gsl_vector_const_view ithColumn = gsl_matrix_const_column (user_data->sbaOutputs, i);
        gsl_vector *v = gsl_vector_calloc (5);
        gsl_vector_memcpy (v, &(ithColumn.vector));
        gsl_vector_sub (v, muPrime);
        gsl_matrix *vvT = gslu_blas_vvT_alloc (v, v);
        gsl_matrix_scale (vvT, gsl_vector_get (covWeights, i));
        gsl_matrix_add (sigmaPrime, vvT);
        gsl_vector_free (v);
        gsl_matrix_free (vvT);
    }

    gsl_matrix_memcpy (rel_pose_cov, sigmaPrime);

    /* clean up */
    gsl_vector_free (muCalib);
    gsl_matrix_free (sigmaPoints);
    gsl_vector_free (meanWeights);
    gsl_vector_free (covWeights);
    gsl_matrix_free (user_data->sbaOutputs);
    gsl_vector_free (muPrime);
    gsl_matrix_free (sigmaPrime);
    g_mutex_free (user_data->mutex);
    free (user_data);
}

void
vis_sba_ut_cov_2v_Rae (const gsl_matrix *sigmaCalib,
                       const gsl_vector *params, const gsl_matrix *u1_dist,
                       const gsl_matrix *u2_dist, const gsl_matrix *K,
                       const gsl_vector *distCoeffs,
                       const gsl_vector *featscalei, const gsl_vector *featscalej,
                       GMutex *sbaMutex, gsl_matrix *rel_pose_cov)
{
    gsl_vector *muCalib = gsl_vector_calloc (9);

    /* DEBUG: */
    /* hauvDump (params, u1->size2, u1, u2, K, distCoeffs); */

    /* intrinsic parameters */
    gsl_vector_set (muCalib, 0, gsl_matrix_get (K, 0, 0));
    gsl_vector_set (muCalib, 1, gsl_matrix_get (K, 1, 1));
    gsl_vector_set (muCalib, 2, gsl_matrix_get (K, 0, 2));
    gsl_vector_set (muCalib, 3, gsl_matrix_get (K, 1, 2));

    /* lens distortion parameters */
    gsl_vector_set (muCalib, 4, gsl_vector_get (distCoeffs, 0));
    gsl_vector_set (muCalib, 5, gsl_vector_get (distCoeffs, 1));
    gsl_vector_set (muCalib, 6, gsl_vector_get (distCoeffs, 2));
    gsl_vector_set (muCalib, 7, gsl_vector_get (distCoeffs, 3));
    gsl_vector_set (muCalib, 8, gsl_vector_get (distCoeffs, 4));

    gsl_matrix *sigmaPoints;
    gsl_vector *meanWeights, *covWeights;
    unscented_transform_alloc (muCalib, sigmaCalib, NULL, &sigmaPoints, &meanWeights, &covWeights);

    /* initialize thread pool to run sba */
    CommonData *user_data = calloc (1, sizeof(*user_data));
    user_data->sbaOutputs = gsl_matrix_calloc (5, sigmaPoints->size2);
    user_data->mutex = g_mutex_new ();
    user_data->sbaMutex = sbaMutex;
    GThreadPool *pool = g_thread_pool_new (threadFuncRae, user_data, UT_POOL_THREADS_MAX, TRUE, NULL);

    /* For each sigma point, kick off new thread */
    for (int i=0; i<sigmaPoints->size2; i++)
    {

        gsl_vector_const_view ithSigmaPoint = gsl_matrix_const_column (sigmaPoints, i);

        /* Create the thread data */
        ThreadDataRae *threadData = calloc (1, sizeof(*threadData));
        threadData->u1_dist = gsl_matrix_calloc (u1_dist->size1, u1_dist->size2);
        threadData->u2_dist = gsl_matrix_calloc (u2_dist->size1, u2_dist->size2);
        threadData->x21 = gsl_vector_calloc (6);
        threadData->sigmaPoint = gsl_vector_calloc (ithSigmaPoint.vector.size);
        threadData->index = i;

        /* copy to thread data struct */
        gsl_matrix_memcpy (threadData->u1_dist, u1_dist);
        gsl_matrix_memcpy (threadData->u2_dist, u2_dist);
        gsl_vector_memcpy (threadData->sigmaPoint, &(ithSigmaPoint.vector));

        gsl_vector_view x21_t = gsl_vector_subvector (threadData->x21, 0, 3);
        gsl_vector_view x21_rph = gsl_vector_subvector (threadData->x21, 3, 3);
        gsl_vector_const_view x21_ae = gsl_vector_const_subvector (params, 5, 2);
        gsl_vector_const_view x21_rph_5dof = gsl_vector_const_subvector (params, 7, 3);

        gsl_vector *x21_dm = gsl_vector_calloc (3);
        gsl_vector_set (x21_dm, 0, gsl_vector_get (&x21_ae.vector, 0));
        gsl_vector_set (x21_dm, 1, gsl_vector_get (&x21_ae.vector, 1));
        gsl_vector_set (x21_dm, 2, 1.0);

        /* compute the 6dof pose as the initial guess for the thread */
        dm_dm2trans_gsl (x21_dm, &x21_t.vector, NULL);
        gsl_vector_memcpy (&x21_rph.vector, &x21_rph_5dof.vector);

        g_thread_pool_push (pool, threadData, NULL);

        /* clean up */
        gsl_vector_free (x21_dm);
    }

    /* Wait for all the jobs to finish */
    g_thread_pool_free (pool, FALSE, TRUE);

    /* now that threads are done, compute the covariance estiamte using std. ut
     * techniques */
    gsl_vector *muPrime = gsl_vector_calloc (5);
    gsl_matrix *sigmaPrime = gsl_matrix_calloc (5, 5);

    /* gslu_matrix_printf (user_data->sbaOutputs, "outputs"); */
    /* gslu_vector_printf (meanWeights, "meanWeights"); */
    /* gslu_vector_printf (covWeights, "covWeights"); */

    for (int i=0; i<sigmaPoints->size2; i++)
    {
        /* muPrime = muPrime + meanWeights(i)*y(:,i) */
        gsl_vector_const_view ithColumn = gsl_matrix_const_column (user_data->sbaOutputs, i);
        gsl_vector *ithColumnScaled = gsl_vector_calloc (ithColumn.vector.size);
        gsl_vector_memcpy (ithColumnScaled, &ithColumn.vector);
        gsl_vector_scale (ithColumnScaled, gsl_vector_get (meanWeights, i));
        gsl_vector_add (muPrime, ithColumnScaled);
        gsl_vector_free (ithColumnScaled);
    }

    for (int i=0; i<sigmaPoints->size2; i++)
    {
        /* sigmaPrime = sigmaPrime + covWeight(i)*(y(:,i) - muPrime)*(y(:,i) - muPrime)' */
        gsl_vector_const_view ithColumn = gsl_matrix_const_column (user_data->sbaOutputs, i);
        gsl_vector *v = gsl_vector_calloc (5);
        gsl_vector_memcpy (v, &(ithColumn.vector));
        gsl_vector_sub (v, muPrime);
        gsl_matrix *vvT = gslu_blas_vvT_alloc (v, v);
        gsl_matrix_scale (vvT, gsl_vector_get (covWeights, i));
        gsl_matrix_add (sigmaPrime, vvT);
        gsl_vector_free (v);
        gsl_matrix_free (vvT);
    }

    gsl_matrix_memcpy (rel_pose_cov, sigmaPrime);

    /* clean up */
    gsl_vector_free (muCalib);
    gsl_matrix_free (sigmaPoints);
    gsl_vector_free (meanWeights);
    gsl_vector_free (covWeights);
    gsl_matrix_free (user_data->sbaOutputs);
    gsl_vector_free (muPrime);
    gsl_matrix_free (sigmaPrime);
    g_mutex_free (user_data->mutex);
    free (user_data);
}
