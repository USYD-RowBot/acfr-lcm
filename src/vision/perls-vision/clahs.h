#ifndef __PERLS_VISION_CLAHS_H__
#define __PERLS_VISION_CLAHS_H__

#include <stddef.h>

#define VIS_CLAHS_RET_SUCCESS_PAD         (1)
#define VIS_CLAHS_RET_SUCCESS             (0)
#define VIS_CLAHS_RET_ERROR_NTILESC       (-1)
#define VIS_CLAHS_RET_ERROR_NTILESR       (-2)
#define VIS_CLAHS_RET_ERROR_DEPTH         (-3)
#define VIS_CLAHS_RET_ERROR_RANGEMIN      (-4)
#define VIS_CLAHS_RET_ERROR_RANGEMAX      (-5)
#define VIS_CLAHS_RET_ERROR_CLIPLIMIT     (-6)
#define VIS_CLAHS_RET_ERROR_MALLOC        (-7) // Not enough memory!, (try reducing nBins)
#define VIS_CLAHS_RET_ERROR_ALPHA         (-8)

#ifdef __cplusplus
extern "C" {
#endif


typedef enum
{
    VIS_CLAHS_DIST_UNIFORM=1,
    VIS_CLAHS_DIST_EXPONENTIAL=2,
    VIS_CLAHS_DIST_RAYLEIGH=3
} vis_clahs_dist_t;


typedef struct vis_clahs_opts vis_clahs_opts_t;
struct vis_clahs_opts
{
    size_t tiles[2];    /* Two-element vector of positive integers: [R C].
                          * [R C] specifies the number of tile rows and
                          * columns.  Both R and C must be at least 2. 
                          * The total number of image tiles is equal to R*C.
                          *
                          * Default: [8, 8].
                          */

    float cliplimit;     /* Real scalar from 0 to 1.
                          * 'clipLimit' limits contrast enhancement. Higher numbers 
                          * result in more contrast; clipLimit = 1 results in standard
                          * adaptive histogram equalization with no control limiting.
                          *
                          * Default: 0.01.
                          */


    size_t bins;         /* Positive integer scalar.
                          * Sets number of bins for the histogram used in building a
                          * contrast enhancing transformation. Higher values result 
                          * in greater dynamic range at the cost of slower processing
                          * speed.
                          *
                          * Default: 256.
                          */


    size_t range[2];     /* Two-element vector of min/max output intensity.
                          * Controls the range of the output image data.  If 
                          * 'rangeMin'='rangeMax', then the full range of the output
                          * image class is used based upon the specified bit depth
                          * (e.g., [0 255] for depth=8).
                          *
                          * Default: [0, 0]
                          */

    vis_clahs_dist_t dist;   /* Distribution can be one of three types: 
                              * CLAHS_UNIFORM, CLAHS_EXPONENTIAL, or CLAHS_RAYLEIGH.
                              * Sets desired histogram shape for the image tiles, by 
                              * specifying a distribution type.
                              *
                              * Default: 'uniform'.
                              */

    float alpha;         /* Nonnegative real scalar.
                          * 'alpha' is a distribution parameter, which can be supplied 
                          * when 'dist' is set to either CLAHS_RAYLIEGH or CLAHS_EXPONENTIAL
                          *
                          * Default: 0.4.
                          */
/*
 *   Notes
 *   -----
 *   - 'nTilesR' & 'nTilesC' specify the number of rectangular contextual regions (tiles)
 *     into which the image is divided. The contrast transform function is
 *     calculated for each of these regions individually. The optimal number of
 *     tiles depends on the type of the input image, and it is best determined
 *     through experimentation.
 *
 *   - The 'clipLimit' is a contrast factor that prevents over-saturation of the
 *     image specifically in homogeneous areas.  These areas are characterized
 *     by a high peak in the histogram of the particular image tile due to many
 *     pixels falling inside the same gray level range. Without the clip limit,
 *     the adaptive histogram equalization technique could produce results that,
 *     in some cases, are worse than the original image.
 *
 *   - CLAHS can use Uniform, Rayleigh, or Exponential distribution as
 *     the basis for creating the contrast transform function. The distribution
 *     that should be used depends on the type of the input image.
 *     For example, underwater imagery appears to look more natural when the
 *     Rayleigh distribution is used.
 */
};

/*
 * Image  - image pixel data, assumes single channel processing, 
 *          can be uint8_t or uint16_t (or their equivalents)
 * width  - image width in pixels
 * height - image height in pixels
 * depth  - pixel depth in bits, (e.g. 8 or 16)
 * opts   - clahs options structure, set to NULL to use default options
 */
int
vis_clahs (void *Image, size_t width, size_t height, size_t depth, const vis_clahs_opts_t *opts);

/*
 * Returns a clahs_opts_t struct initialized to default values
 */
vis_clahs_opts_t
vis_clahs_default_opts (void);

#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_CLAHS_H__
