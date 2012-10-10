#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <opencv/cv.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-common/error.h"
#include "perls-common/bot_util.h"

#include "opencv_util.h"
#include "calib.h"

perllcm_van_calib_t
vis_calib_load_config (BotParam *param, const char *rootkey)
{
    perllcm_van_calib_t calib = {0};

    if (!bot_param_has_key (param, rootkey)) {
        ERROR ("unable to find key `%s or %s.calib`", rootkey, rootkey);
        exit (-1);
    };

    //-----------------PARSED QUANTITIES-------------------------------//

    // matlab calibration file
    char mykey[1024];
    snprintf (mykey, sizeof mykey, "%s.mfile", rootkey);
    if (bot_param_has_key (param, mykey)) {
        char *mfile;
        bot_param_get_str (param, mykey, &mfile);
        printf ("mfile=%s\n", mfile);
        perllcm_van_calib_t calib = vis_calib_load_matlab (mfile);
        free (mfile);
        return calib;
    }

    // image size
    snprintf (mykey, sizeof mykey, "%s.nx", rootkey);
    calib.width = bot_param_get_int_or_fail (param, mykey);
    snprintf (mykey, sizeof mykey, "%s.ny", rootkey);
    calib.height = bot_param_get_int_or_fail (param, mykey);

    // focal length
    snprintf (mykey, sizeof mykey, "%s.fc", rootkey);
    if (bot_param_has_key (param, mykey))
        bot_param_get_double_array (param, mykey, calib.fc, 2);
    else {
        ERROR ("[%s] unable to parse focal length: fc\n", rootkey);
        exit (EXIT_FAILURE);
    }
    snprintf (mykey, sizeof mykey, "%s.fc_error", rootkey);
    if (bot_param_has_key (param, mykey)) {
        bot_param_get_double_array (param, mykey, calib.fc_std, 2);
        calib.fc_std[0] /= 3;
        calib.fc_std[1] /= 3;
    }
    else
        ERROR ("[%s] unable to parse focal length error: assuming fc_error=0.0\n", rootkey);

    // principal point
    snprintf (mykey, sizeof mykey, "%s.cc", rootkey);
    if (bot_param_has_key (param, mykey))
        bot_param_get_double_array (param, mykey, calib.cc, 2);
    else {
        ERROR ("[%s] unable to parse principal point: cc\n", rootkey);
        exit (EXIT_FAILURE);
    }
    snprintf (mykey, sizeof mykey, "%s.cc_error", rootkey);
    if (bot_param_has_key (param, mykey)) {
        bot_param_get_double_array (param, mykey, calib.cc_std, 2);
        calib.cc_std[0] /= 3;
        calib.cc_std[1] /= 3;
    }
    else
        ERROR ("[%s] unable to parse principal point error: assuming cc_error=0.0\n", rootkey);

    // skew coefficient
    snprintf (mykey, sizeof mykey, "%s.alpha_c", rootkey);
    if (!bot_param_has_key (param, mykey))
        ERROR ("[%s] unable to parse skew coefficient: assuming alpha_c=0.0\n", rootkey);
    calib.alpha_c = 0.0;
    bot_param_get_double (param, mykey, &calib.alpha_c);

    snprintf (mykey, sizeof mykey, "%s.alpha_c_error", rootkey);
    if (!bot_param_has_key (param, mykey))
        ERROR ("[%s] unable to parse skew coefficient error: assuming alpha_c_error=0.0\n", rootkey);
    calib.alpha_c_std = 0.0;
    bot_param_get_double (param, mykey, &calib.alpha_c_std);

    // distortion coeffs
    int len = 0;
    int n_kc = sizeof (calib.kc) / sizeof (calib.kc[0]);
    snprintf (mykey, sizeof mykey, "%s.kc", rootkey);
    if (bot_param_has_key (param, mykey)) {
        len = bot_param_get_array_len (param, mykey);
        if (len > n_kc) {
            ERROR ("[%s], too many distortion coeffs (len=%d>n_kc=%d)", rootkey, len, n_kc);
            exit (EXIT_FAILURE);
        }
        bot_param_get_double_array (param, mykey, calib.kc, len);
    }
    else
        ERROR ("[%s] unable to parse distortion coeff: assuming kc=0.0\n", rootkey);
    snprintf (mykey, sizeof mykey, "%s.kc_error", rootkey);
    if (bot_param_has_key (param, mykey)) {
        bot_param_get_double_array (param, mykey, calib.kc_std, len);
        for (int i=0; i<len; i++)
            calib.kc_std[i] /= 3;
    }
    else
        ERROR ("[%s] unable to parse distortion coeff error: assuming kc_error=0.0\n", rootkey);


    // distortion model
    char *kc_model;
    snprintf (mykey, sizeof mykey, "%s.kc_model", rootkey);
    if (bot_param_has_key (param, mykey))
        kc_model = bot_param_get_str_or_fail (param, mykey);
    else {
        ERROR ("[%s] unable to parse distortion model: assuming kc_model=\"radtan\"\n", rootkey);
        kc_model = botu_param_get_str_or_default (param, mykey, "radtan");
    }
    if (0 == strcasecmp (kc_model, "radial"))
        calib.kc_model = PERLLCM_VAN_CALIB_T_KC_MODEL_RADIAL;
    else if (0 == strcasecmp (kc_model, "radtan"))
        calib.kc_model = PERLLCM_VAN_CALIB_T_KC_MODEL_RADTAN;
    else if (0 == strcasecmp (kc_model, "spherical"))
        calib.kc_model = PERLLCM_VAN_CALIB_T_KC_MODEL_SPHERICAL;
    else if (0 == strcasecmp (kc_model, "full_map"))
        calib.kc_model = PERLLCM_VAN_CALIB_T_KC_MODEL_FULL_MAP;
    else {
        ERROR ("[%s] unknown distortion model: kc_model=%s\n", rootkey, kc_model);
        exit (EXIT_FAILURE);
    }

    //-----------------DERIVED QUANTITIES-------------------------------//
    calib.K[0] = calib.fc[0]; calib.K[1] = calib.alpha_c; calib.K[2] = calib.cc[0];
    calib.K[3] = 0.0;         calib.K[4] = calib.fc[1];   calib.K[5] = calib.cc[1];
    calib.K[6] = 0.0;         calib.K[7] = 0.0;           calib.K[8] = 1.0;

    gsl_matrix_view K = gsl_matrix_view_array (calib.K, 3, 3);
    gsl_matrix_view Kinv = gsl_matrix_view_array (calib.Kinv, 3, 3);
    gslu_matrix_inv (&Kinv.matrix, &K.matrix);

    calib.fov[0] = 2.0 * atan (calib.width/2.0 / calib.fc[0]);
    calib.fov[1] = 2.0 * atan (calib.height/2.0 / calib.fc[1]);

    return calib;
}


perllcm_van_calib_t
vis_calib_load_matlab (const char *filename)
{
    perllcm_van_calib_t calib = {0};

    /* WRITE ME !!!*/
    ERROR ("Write me!!");
    exit (EXIT_FAILURE);

    return calib;
}

vis_calib_view_cv_t
vis_calib_view_cv (perllcm_van_calib_t *calib)
{
    int n_kc = sizeof (calib->kc) / sizeof (calib->kc[0]);
    gsl_vector_view kc = gsl_vector_view_array (calib->kc, n_kc);
    gsl_matrix_view K = gsl_matrix_view_array (calib->K, 3, 3);
    gsl_matrix_view Kinv = gsl_matrix_view_array (calib->Kinv, 3, 3);
    vis_calib_view_cv_t cv = {
        .imageSize.width = calib->width,
        .imageSize.height = calib->height,
        .K = vis_cvu_gsl_matrix_to_cvmat_view (&K.matrix),
        .Kinv = vis_cvu_gsl_matrix_to_cvmat_view (&Kinv.matrix),
        .kc = vis_cvu_gsl_vector_to_cvmat_view (&kc.vector),
    };
    return cv;
}

vis_calib_const_view_cv_t
vis_calib_const_view_cv (const perllcm_van_calib_t *calib)
{
    int n_kc = sizeof (calib->kc) / sizeof (calib->kc[0]);
    gsl_vector_const_view kc = gsl_vector_const_view_array (calib->kc, n_kc);
    gsl_matrix_const_view K = gsl_matrix_const_view_array (calib->K, 3, 3);
    gsl_matrix_const_view Kinv = gsl_matrix_const_view_array (calib->Kinv, 3, 3);
    vis_calib_const_view_cv_t cv = {
        .imageSize.width = calib->width,
        .imageSize.height = calib->height,
        .K = vis_cvu_gsl_matrix_to_cvmat_view (&K.matrix),
        .Kinv = vis_cvu_gsl_matrix_to_cvmat_view (&Kinv.matrix),
        .kc = vis_cvu_gsl_vector_to_cvmat_view (&kc.vector),
    };
    return cv;
}


vis_calib_view_gsl_t
vis_calib_view_gsl (perllcm_van_calib_t *calib)
{
    int n_kc = sizeof (calib->kc) / sizeof (calib->kc[0]);
    vis_calib_view_gsl_t gsl = {
        .imageSize = {calib->width, calib->height},
        .K = gsl_matrix_view_array (calib->K, 3, 3),
        .Kinv = gsl_matrix_view_array (calib->Kinv, 3, 3),
        .kc  = gsl_vector_view_array (calib->kc, n_kc),
    };
    return gsl;
}

vis_calib_const_view_gsl_t
vis_calib_const_view_gsl (const perllcm_van_calib_t *calib)
{
    int n_kc = sizeof (calib->kc) / sizeof (calib->kc[0]);
    vis_calib_const_view_gsl_t gsl = {
        .imageSize = {calib->width, calib->height},
        .K = gsl_matrix_const_view_array (calib->K, 3, 3),
        .Kinv = gsl_matrix_const_view_array (calib->Kinv, 3, 3),
        .kc  = gsl_vector_const_view_array (calib->kc, n_kc),
    };
    return gsl;
}
