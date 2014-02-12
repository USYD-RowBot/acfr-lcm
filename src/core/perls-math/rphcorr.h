#ifndef __PERLS_MATH_RPHCORR_H__
#define __PERLS_MATH_RPHCORR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <lcm/lcm.h>
#include <bot_param/param_client.h>

#define RPHCORR_MODEL_UVC_NCOEFFS 5

typedef enum _rphcorr_model_t rphcorr_model_t;
enum _rphcorr_model_t {
    rphcorr_model_uvc,
    rphcorr_model_none
};

typedef struct _rphcorr_t rphcorr_t;
struct _rphcorr_t {
    int publish;
    rphcorr_model_t model;

    int ncoeffs;
    double *coeffs;
    double mag_var;

    double x_sv[6];
    double x_vs[6];

    char *channel;
};

rphcorr_t*
rphcorr_create (BotParam *param, char *rootkey);

void
rphcorr_destroy (rphcorr_t *rphcorr);

void
publish_rphcorr (lcm_t *lcm, rphcorr_t *rphcorr, int64_t utime, const double raw_rph[3]);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_MATH_RPHCORR_H__
