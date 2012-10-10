#include <math.h>
#include <string.h>

#include "perls-common/error.h"
#include "perls-common/units.h"

#include "ssc.h"

#include "perls-lcmtypes/senlcm_rph_t.h"

#include "rphcorr.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

rphcorr_t*
rphcorr_create (BotParam *param, char *rootkey)
{
    rphcorr_t *rphcorr = calloc (1, sizeof (rphcorr_t));

    char key[256] = {0};

    sprintf (key, "%s.rphcorr.publish", rootkey);
    bot_param_get_int (param, key, &rphcorr->publish);
    if (rphcorr->publish) {
        char *model_type = NULL;
        sprintf (key, "%s.rphcorr.model", rootkey);
        bot_param_get_str (param, key, &model_type);

        if (0==strcmp (model_type,"UVC")) {
            rphcorr->model = rphcorr_model_uvc;
            sprintf (key, "%s.rphcorr.coeffs", rootkey);
            int ncoeffs = bot_param_get_array_len (param, key);
            if (ncoeffs != RPHCORR_MODEL_UVC_NCOEFFS) {
                ERROR ("rphcorr model requires %d coeffs, not %d as in config", 
                        RPHCORR_MODEL_UVC_NCOEFFS, ncoeffs);
                return NULL;
            }
            rphcorr->ncoeffs = ncoeffs;
            rphcorr->coeffs = calloc (1, ncoeffs*sizeof (double));
            bot_param_get_double_array_or_fail (param, key, rphcorr->coeffs, ncoeffs);
            for (int i=0;i<ncoeffs;++i) rphcorr->coeffs[i] *= DTOR;
        }
        else if (0==strcmp (model_type,"NONE")) {
            rphcorr->model = rphcorr_model_none;
        }
        else {
            ERROR ("unrecognized heading correction model in config!");
            free (model_type);
            return NULL;
        }
        free (model_type);

        rphcorr->mag_var = bot_param_get_double_or_fail (param, "site.mag_var");
        rphcorr->mag_var *= DTOR;

        sprintf (key, "%s.rphcorr.channel", rootkey);
        rphcorr->channel = bot_param_get_str_or_fail (param, key);

        sprintf (key, "%s.x_vs", rootkey);
        bot_param_get_double_array_or_fail (param, key, rphcorr->x_vs, 6);
        for (int i=3;i<6;++i) rphcorr->x_vs[i] *= DTOR;
        double J_inv[6*6];
        ssc_inverse (rphcorr->x_sv, J_inv, rphcorr->x_vs);
    }
    else {
        rphcorr->ncoeffs = 0;
        rphcorr->channel = strdup ("");
        rphcorr->model = rphcorr_model_none;
    }
    return rphcorr;
}

void
rphcorr_destroy (rphcorr_t *rphcorr)
{
    if (rphcorr) {
        if (rphcorr->channel) free (rphcorr->channel);
        if (rphcorr->coeffs) free (rphcorr->coeffs);
        free (rphcorr);
    }
}

static void 
uvc_correction (double rph[3], const double raw_rph[3], rphcorr_t *rphcorr)
{
    // transmform sensor frame rph into vehicle frame
    double J_h2t[6*12], x_rv[6] = {0}, x_rs[6] = {0};
    memcpy (x_rs+3, raw_rph, 3*sizeof (double));
    ssc_head2tail (x_rv, J_h2t, x_rs, rphcorr->x_sv);

    // compute heading correction
    double h = x_rv[5];
    h += rphcorr->mag_var;
    double A = rphcorr->coeffs[0],
           B = rphcorr->coeffs[1],
           C = rphcorr->coeffs[2],
           D = rphcorr->coeffs[3],
           E = rphcorr->coeffs[4];
    double deviation = A + B*sin(h) + C*cos(h) + D*sin(2*(h)) + E*cos(2*(h));
    x_rv[5] = h + deviation;

    // transform corrected heading back into sensor frame
    ssc_head2tail (x_rs, J_h2t, x_rv, rphcorr->x_vs);
    memcpy (rph, x_rs+3, 3*sizeof (double));
}

void
publish_rphcorr (lcm_t *lcm, rphcorr_t *rphcorr, int64_t utime, const double raw_rph[3])
{
    if (!rphcorr->publish) return;

    double cor_rph[3] = {0};
    if (rphcorr->model == rphcorr_model_uvc) { // UVC model
        uvc_correction (cor_rph, raw_rph, rphcorr);
    }
    else if (rphcorr->model == rphcorr_model_none) {
        memcpy (cor_rph, raw_rph, 3*sizeof (double));
    }
    else {
        ERROR ("unrecognized heading correction model!");
        memcpy (cor_rph, raw_rph, 3*sizeof (double));
        return;
    }

    senlcm_rph_t msg = {0};
    msg.utime = utime;
    memcpy (msg.rph, cor_rph, 3*sizeof (double));
    senlcm_rph_t_publish (lcm, rphcorr->channel, &msg);
}
