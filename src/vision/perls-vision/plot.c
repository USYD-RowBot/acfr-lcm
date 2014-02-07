#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h> // needed for PRId64 macros
#include <assert.h>
#include <math.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <gsl/gsl_eigen.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_sort_vector.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_vector_short.h>

#include "perls-lcmtypes/perllcm_van_feature_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_cvsurf_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_siftgpu_t.h"
#include "perls-lcmtypes/perllcm_van_vlink_t.h"

#include "perls-common/error.h"
#include "perls-common/units.h"
#include "perls-math/dm.h"
#include "perls-math/fasttrig.h"
#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"

#include "homography.h"
#include "plot.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define SUMMARY_PLOT_NPTS_MAX  20
#define SHOW_ELLIPSE_AXIS      1    // ON / OFF

#define LINE_THICKNESS 3
#define FEAT_THICKNESS 1

#define DOT_SIZE 10
 
/* generate 8 basic color */
void 
_gen_basic_colors (CvScalar lcolors[8]) // b g r c m y k w
{ 
    /* RGB = [255 0 0]      = red
     * RGB = [0 255 0]      = green
     * RGB = [0 0 255]      = blue
     * RGB = [255 255 0]    = yellow
     * RGB = [255 0 255]    = magenta 
     * RGB = [0 255 255]    = cyan
     * RGB = [255 255 255]  = white
     * RGB = [0 0 0]        = black 
     */

    // cvcolor in BGR seq (this follows the matlab default)
    lcolors[0] = cvScalar (255, 0, 0, 0);      // blue
    lcolors[1] = cvScalar (0, 255, 0, 0);      // green 
    lcolors[2] = cvScalar (0, 0, 255, 0);      // red
    lcolors[3] = cvScalar (255, 0, 255, 0);    // magenta
    lcolors[4] = cvScalar (255, 255, 0, 0);    // cyan
    lcolors[5] = cvScalar (0, 255, 255, 0);    // yellow
    lcolors[6] = cvScalar (0, 0, 0, 0);        // black
    lcolors[7] = cvScalar (255, 255, 255, 0);  // white

}

void
vis_plot_add_utime (IplImage *image, 
                    int64_t utime, 
                    const CvPoint *_pt, 
                    const CvFont *_font, 
                    const CvScalar *_color)
{
    assert (image);

    CvPoint pt;
    if (_pt)
        pt = *_pt;
    else
        pt = cvPoint (0, 20);

    CvFont font;
    if (_font)
        font = *_font;
    else
        cvInitFont (&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1, 8);

    CvScalar color;
    if (_color)
        color = *_color;
    else
        color = CV_RGB (255, 255, 255);

    char utime_str[255];
    snprintf (utime_str, sizeof utime_str, "%"PRId64, utime);
    cvPutText (image, utime_str, pt, &font,  color);
}

void
vis_plot_add_text (IplImage *image, 
                   const char *text,
                   const CvPoint *_pt,
                   const CvFont *_font,
                   const CvScalar *_color)
{
    assert (image);

    CvPoint pt;
    if (_pt)
        pt = *_pt;
    else
        pt = cvPoint (0, 20);

    CvFont font;
    if (_font)
        font = *_font;
    else
        cvInitFont (&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1, 8);

    CvScalar color;
    if (_color)
        color = *_color;
    else
        color = CV_RGB (255, 255, 255);

    cvPutText (image, text, pt, &font,  color);
}


static IplImage *
_vis_plot_mkimgplot (const IplImage *image, 
                     float scale)
{
    CvSize size = {image->width*scale, image->height*scale};
    IplImage *img_scaled = cvCreateImage (size, IPL_DEPTH_8U, 1);
    IplImage *img_plot = cvCreateImage (size, IPL_DEPTH_8U, 3);
    cvResize (image, img_scaled, CV_INTER_LINEAR);
    cvCvtColor (img_scaled, img_plot, CV_GRAY2BGR);
    cvReleaseImage (&img_scaled);
    return img_plot;
}

static void
_vis_plot_feature_cvsurf (CvArr *img_plot, 
                          const perllcm_van_feature_t *f, 
                          const CvScalar color, 
                          const float scale)
{
    // plot options
    const int thickness = 1; // line thickness when pos number, -1 for filled circlea

    perllcm_van_feature_attr_cvsurf_t *attr = malloc (sizeof (*attr));
    if (perllcm_van_feature_attr_cvsurf_t_decode (f->attr, 0, f->attrsize, attr) != f->attrsize) {
        ERROR ("perllcm_van_feature_attr_cvsurf_t_decode() failed");
        free (attr);
        return;
    }


/*    gsl_permutation *perm = gsl_permutation_alloc (f->npts);
    gsl_vector_short_view size_vec = gsl_vector_short_view_array (attr->size, f->npts);
    gsl_sort_vector_short_index (perm, &size_vec.vector); // ascending order

    for (size_t n=0; n < f->npts; n++) {
        if (n<20) {
        int i = perm ? perm->data[f->npts-1-n] : n;
        const float u = f->u[i] * scale;
        const float v = f->v[i] * scale;
        const float radius = attr->size[i] * scale;
        const float dir = attr->dir[i];
        double s, c;
        fsincos (dir, &s, &c); 
        const CvPoint pt1 = cvPoint (u, v);
        const CvPoint pt2 = cvPoint (u + radius*c, v+radius*s);
        cvCircle (img_plot, pt1, radius, color, thickness, CV_AA, 0);
        cvLine (img_plot, pt1, pt2, color, thickness, CV_AA, 0);
        }
    }

    if (perm) gsl_permutation_free (perm);*/

    for (size_t n=0; n < f->npts; n++) {
        const float u = f->u[n] * scale;
        const float v = f->v[n] * scale;
        const float radius = attr->size[n] * scale;
        const float dir = attr->dir[n];
        double s, c;
        fsincos (dir, &s, &c); 
        const CvPoint pt1 = cvPoint (u, v);
        const CvPoint pt2 = cvPoint (u + radius*c, v-radius*s);

        cvCircle (img_plot, pt1, radius, color, thickness, CV_AA, 0);
        cvLine (img_plot, pt1, pt2, color, thickness, CV_AA, 0);

        //n=n+10;
    }
    perllcm_van_feature_attr_cvsurf_t_destroy (attr);
}

static void
_vis_plot_feature_siftgpu (CvArr *img_plot, 
                           const perllcm_van_feature_t *f, 
                           const CvScalar color, 
                           const float scale)
{
    // plot options
    const int thickness = FEAT_THICKNESS; // line thickness when pos number, -1 for filled circle

    perllcm_van_feature_attr_siftgpu_t *attr = malloc (sizeof (*attr));
    if (perllcm_van_feature_attr_siftgpu_t_decode (f->attr, 0, f->attrsize, attr) != f->attrsize) {
        ERROR ("perllcm_van_feature_attr_siftgpu_t_decode() failed");
        free (attr);
        return;        
    }
    for (size_t n=0; n < f->npts; n++) {
        const float u = f->u[n] * scale;
        const float v = f->v[n] * scale;
        const float radius = attr->s[n] * scale;
        const float dir = attr->o[n];
        double s, c;
        fsincos (dir, &s, &c); 
        const CvPoint pt1 = cvPoint (u, v);
        const CvPoint pt2 = cvPoint (u + radius*c, v+radius*s);
        cvCircle (img_plot, pt1, radius, color, thickness, CV_AA, 0);
        cvLine (img_plot, pt1, pt2, color, thickness, CV_AA, 0);
    }
    perllcm_van_feature_attr_siftgpu_t_destroy (attr);
}

static void
_vis_plot_feature_cvharris (CvArr *img_plot, 
                            const perllcm_van_feature_t *f, 
                            const CvScalar color, 
                            const float scale)
{
    // plot options
    const int thickness = -1; // line thickness when pos number, -1 for filled circle
    const float radius = 1;

    for (size_t n=0; n < f->npts; n++) {
        const float u = f->u[n] * scale;
        const float v = f->v[n] * scale;

        const CvPoint pt = cvPoint (u, v);
        cvCircle (img_plot, pt, radius, color, thickness, CV_AA, 0);
    }

}
void
vis_plot_feature (const IplImage *image, 
                  const perllcm_van_feature_t *f, 
                  const char *named_window, 
                  float scale, 
                  int64_t utime)
{
    fasttrig_init ();
    IplImage *img_plot = _vis_plot_mkimgplot (image, scale);

    switch (f->attrtype) {
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF:
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SURFGPU:
        _vis_plot_feature_cvsurf (img_plot, f, CV_RGB (0, 255, 0), scale);
        break;
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS:
        _vis_plot_feature_cvharris (img_plot, f, CV_RGB (255, 0, 0), scale);
        break;
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU:
        _vis_plot_feature_siftgpu (img_plot, f, CV_RGB (255, 0, 0), scale);
        break;
    default:
        ERROR ("unknown attrtype %d", f->attrtype);
    }
    
    if (utime)
        vis_plot_add_utime (img_plot, utime, NULL, NULL, NULL);
    
    cvNamedWindow (named_window, CV_WINDOW_AUTOSIZE);
    cvShowImage (named_window, img_plot);
    cvReleaseImage (&img_plot);
}

void
vis_plot_saliency (const perllcm_van_saliency_t *sal,
                   const IplImage *image,
                   const char *named_window,
                   const float scale)
{
    IplImage *canvas = _vis_plot_mkimgplot (image, scale);

    // add utime
    vis_plot_add_utime (canvas, sal->utime, NULL, NULL, NULL);

    // display saliency
    CvFont font;
    cvInitFont (&font, CV_FONT_HERSHEY_PLAIN, 1.0,1.0, 0, 2, 8);
    CvScalar green = CV_RGB (0, 255, 0);
    CvScalar red = CV_RGB (255, 0, 0);
    CvPoint local_pt = cvPoint (20, 50);
    CvPoint global_pt = cvPoint (20, 75);
    char local_str[30], global_str[30];

    // local
    snprintf (local_str, sizeof local_str, "L. Sal = %1.2f", sal->S_L);
    if (sal->is_S_L)
        cvPutText (canvas, local_str, local_pt, &font,  green);
    else
        cvPutText (canvas, local_str, local_pt, &font,  red);

    // global
    snprintf (global_str, sizeof global_str, "G. Sal = %1.2f", sal->S_G);
    if (sal->is_S_G)
        cvPutText (canvas, global_str, global_pt, &font,  green);
    else
        cvPutText (canvas, global_str, global_pt, &font,  red);
    
    cvNamedWindow (named_window, CV_WINDOW_AUTOSIZE);
    cvShowImage (named_window, canvas);
    cvReleaseImage (&canvas);
}

void
vis_plot_scene_prior (const IplImage *image, 
                      const perllcm_van_scene_prior_t *sp, 
                      const char *named_window,
                      int64_t utime, const float scale)
{
    //const float scale = 0.25;

    // make image of 3*width, 3*height in size at 0.25 scale and center image
    IplImage *img_small = _vis_plot_mkimgplot (image, scale);
    CvSize size = {img_small->width*3, img_small->height*3};
    IplImage *img_plot = cvCreateImage (size, IPL_DEPTH_8U, 3);
    cvZero (img_plot);
    cvSetImageROI (img_plot, cvRect (img_small->width, img_small->height, img_small->width, img_small->height));
    cvAdd (img_plot, img_small, img_plot, NULL);
    cvResetImageROI (img_plot);
    
    // cache colors & font
    CvScalar red = CV_RGB (255, 0, 0);
    CvScalar green = CV_RGB (0, 255, 0);
    CvScalar blue = CV_RGB (0, 0, 255);
    CvScalar yellow = CV_RGB (255, 255, 0);
    CvScalar white = CV_RGB (255, 255, 255);

    CvFont font;
    cvInitFont (&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1, 8);

    // plot bathy points
    for (size_t n=0; n<sp->npts; n++) {
        CvPoint pt = cvPoint (img_small->width + sp->u[n]*scale, 
                              img_small->height + sp->v[n]*scale);
        CvScalar color;
        switch (sp->id[n]) {
        case 1:
            color = red;
            break;
        case 2:
            color = green;
            break;
        case 3:
            color = blue;
            break;
        case 4:
            color = yellow;
            break;
        case -1:
        default:
            color = white;
        }
        char id_str[255];
        snprintf (id_str, sizeof id_str, "%d", sp->id[n]);
        cvPutText (img_plot, id_str, cvPoint (0, (2+sp->id[n])*20), &font, color);
        cvCircle (img_plot, pt, 2, color, -1, CV_AA, 0);
    }

    // plot legend
    if (utime)
        vis_plot_add_utime (img_plot, utime, NULL, &font, &white);

    cvPutText (img_plot, sp->locally_planar ? "PLANAR" : "3D", cvPoint (0, 40), &font, white);


    cvNamedWindow (named_window, CV_WINDOW_AUTOSIZE);
    cvShowImage (named_window, img_plot);
    cvReleaseImage (&img_small);
    cvReleaseImage (&img_plot);
}

/* This function has been imported 
 * from Rob Hess's sift code & modified 
 * Combines two images by scacking   
 *  @return Returns the image resulting from stacking
 */
static IplImage* 
vis_plot_stack_imgs (IplImage* img1, IplImage* img2, int stack_type)
{
    // stacked image to be returned
    IplImage* stacked = 0;

    if (stack_type == VIS_PLOT_STACK_VERT) { // stacking vertically img1 = top, img2 = bottom
        stacked = cvCreateImage (cvSize (MAX (img1->width, img2->width),
                                 img1->height + img2->height ),
                                 IPL_DEPTH_8U, img1->nChannels );

        cvZero (stacked);
        cvSetImageROI (stacked, cvRect (0, 0, img1->width, img1->height));
        cvAdd (img1, stacked, stacked, NULL);
        cvSetImageROI (stacked, cvRect (0, img1->height, img2->width, img2->height));
        cvAdd (img2, stacked, stacked, NULL);
        cvResetImageROI (stacked);

        return stacked;
    }
    else {
        stacked = cvCreateImage (cvSize (img1->width + img2->width,
                                 MAX(img1->height,img2->height)),
                                 IPL_DEPTH_8U, img1->nChannels);
        cvZero (stacked);
        cvSetImageROI (stacked, cvRect (0, 0, img1->width, img1->height));
        cvAdd (img1, stacked, stacked, NULL);
        cvSetImageROI (stacked, cvRect (img1->width, 0, img2->width, img2->height));
        cvAdd (img2, stacked, stacked, NULL);
        cvResetImageROI (stacked);

        return stacked;
    }
}

/* when two images are gray
 * stack images and convert it to color to overlay useful informations
 */
static IplImage* 
vis_plot_stack_imgs_cvt_color (IplImage* img1, IplImage* img2, int stack_type)
{
    IplImage* img_stack_rgb = 0;
    IplImage* img_stack = 0;
    img_stack = vis_plot_stack_imgs (img1, img2, stack_type);  // or STACK_VERT

    if (img_stack->nChannels == 1) { // gray scale 
        img_stack_rgb = cvCreateImage (cvGetSize (img_stack), IPL_DEPTH_8U, 3);
        cvCvtColor (img_stack, img_stack_rgb, CV_GRAY2BGR);  
    }
    else { // color 
        img_stack_rgb = cvCloneImage (img_stack);
    }

    cvReleaseImage (&img_stack);

    return img_stack_rgb;
}

void
vis_plot_pair (IplImage *img1,
               IplImage *img2,
               const char *named_window,
               float scale,
               int stack_type)
{
    assert (img1 && img2 && named_window);

    IplImage *img1_scaled = _vis_plot_mkimgplot (img1, scale);
    IplImage *img2_scaled = _vis_plot_mkimgplot (img2, scale);
    IplImage *stacked = vis_plot_stack_imgs (img1_scaled, img2_scaled, stack_type);
    cvNamedWindow (named_window, CV_WINDOW_AUTOSIZE);
    cvShowImage (named_window, stacked);
    cvReleaseImage (&stacked);
    cvReleaseImage (&img1_scaled);
    cvReleaseImage (&img2_scaled);
}

void
_vis_plot_inliers (IplImage *canvas, bool stack_type, size_t offset, gslu_index *clr_idx_vec,
                   const gslu_index *sel1, const gslu_index *sel2, 
                   const gsl_matrix *uv1, const gsl_matrix *uv2)
{
    // color code inliers
    CvScalar lcolors[4];
    lcolors[0] = cvScalar (0, 255, 0, 0);      // green 
    lcolors[1] = cvScalar (255, 255, 0, 0);    // cyan
    lcolors[2] = cvScalar (0, 255, 255, 0);    // yellow
    lcolors[3] = cvScalar (255, 255, 255, 0);  // white

    // color of the feature location (red dot)
    CvScalar red = cvScalar (0, 0, 255, 0);

    // plot options
    int thickness = LINE_THICKNESS;	        // line thickness 0~255
    int lineType = 8;
    int shift = 0;
    int radius = DOT_SIZE;
    int circle_thickness = -1;	// line thickness when pos number, -1 for filled circle

    int n_corr = sel1->size;

    // when no index provided, use default by setting all 0
    bool use_color_code = 1;
    if (!clr_idx_vec) {
        use_color_code = 0;
        clr_idx_vec = gslu_index_alloc (n_corr);
        gslu_index_set_zero (clr_idx_vec);
    }

    for (int i=0; i < n_corr; i++) {
        size_t clr_idx = gslu_index_get (clr_idx_vec, i);

        // draw in image1
        int idx1 = gslu_index_get (sel1, i);
        int idx2 = gslu_index_get (sel2, i);
        CvPoint pt1 = cvPoint (gsl_matrix_get (uv1, 0, idx1), gsl_matrix_get (uv1, 1, idx1));
        CvPoint pt2 = cvPoint (gsl_matrix_get (uv2, 0, idx2), gsl_matrix_get (uv2, 1, idx2));
        cvLine (canvas, pt1, pt2, lcolors[clr_idx], thickness, lineType, shift);
        cvCircle (canvas, pt1, radius, red, circle_thickness, CV_AA, 0);

        // draw in image2
        if (stack_type == VIS_PLOT_STACK_HORZ) {
            CvPoint pt1_on2 = cvPoint (gsl_matrix_get (uv1, 0, idx1)+offset, gsl_matrix_get (uv1, 1, idx1) );
            CvPoint pt2_on2 = cvPoint (gsl_matrix_get (uv2, 0, idx2)+offset, gsl_matrix_get (uv2, 1, idx2) );
            cvLine (canvas, pt1_on2, pt2_on2, lcolors[clr_idx], thickness, lineType, shift);
            cvCircle (canvas, pt2_on2, radius, red, circle_thickness, CV_AA, 0);
        }
        else {
            CvPoint pt1_on2 = cvPoint (gsl_matrix_get (uv1, 0, idx1), gsl_matrix_get (uv1, 1, idx1)+offset);
            CvPoint pt2_on2 = cvPoint (gsl_matrix_get (uv2, 0, idx2), gsl_matrix_get (uv2, 1, idx2)+offset);
            cvLine (canvas, pt1_on2, pt2_on2, lcolors[clr_idx], thickness, lineType, shift);
            cvCircle (canvas, pt2_on2, radius, red, circle_thickness, CV_AA, 0);
        }
    }

    if (!use_color_code)
        gslu_index_free (clr_idx_vec);
}

void
_vis_plot_outliers (IplImage *canvas, bool stack_type, size_t offset, gslu_index *clr_idx_vec,
                   const gslu_index *sel1, const gslu_index *sel2, 
                   const gsl_matrix *uv1, const gsl_matrix *uv2)
{
        
    // color code outliers
    /* CvScalar lcolors[4]; */
    /* lcolors[0] = cvScalar (0, 0, 255, 0);      // red  */
    /* lcolors[1] = cvScalar (255, 255, 0, 0);    // cyan */
    /* lcolors[2] = cvScalar (0, 255, 255, 0);    // yellow */
    /* lcolors[3] = cvScalar (255, 255, 255, 0);  // white */

    // color of the feature location (red dot)
    CvScalar red = cvScalar (0, 0, 255, 0);    

    // plot options
    int thickness = LINE_THICKNESS;	        // line thickness 0~255
    int lineType = 8;
    int shift = 0;

    int n_corr = sel1->size;
    int n = uv1->size2;

    // when no index provided, use default by setting all 0
    bool use_color_code = 1;
    if (!clr_idx_vec) {
        use_color_code = 0;
        clr_idx_vec = gslu_index_alloc (n);
        gslu_index_set_zero (clr_idx_vec);
    }

    // Draw outliers
    gslu_index *sel1_sort = gslu_index_dup (sel1);
    gslu_index *sel2_sort = gslu_index_dup (sel2);
    gsl_sort_vector_ulong (sel1_sort);
    gsl_sort_vector_ulong (sel2_sort);

    int in_idx = 0;
    for (int i=0; i<n; i++) {
        if ( gslu_index_get (sel1_sort, in_idx) != i) {
            // draw in image1
            CvPoint pt1 = cvPoint (gsl_matrix_get (uv1, 0, i), gsl_matrix_get (uv1, 1, i));
            CvPoint pt2 = cvPoint (gsl_matrix_get (uv2, 0, i), gsl_matrix_get (uv2, 1, i));
            cvLine (canvas, pt1, pt2, red, thickness, lineType, shift);

            // draw in image2
            if (stack_type == VIS_PLOT_STACK_HORZ) {
                CvPoint pt1_on2 = cvPoint (gsl_matrix_get (uv1, 0, i)+offset, gsl_matrix_get (uv1, 1, i) );
                CvPoint pt2_on2 = cvPoint (gsl_matrix_get (uv2, 0, i)+offset, gsl_matrix_get (uv2, 1, i) );
                cvLine (canvas, pt1_on2, pt2_on2, red, thickness, lineType, shift);
            }
            else {
                CvPoint pt1_on2 = cvPoint (gsl_matrix_get (uv1, 0, i), gsl_matrix_get (uv1, 1, i)+offset);
                CvPoint pt2_on2 = cvPoint (gsl_matrix_get (uv2, 0, i), gsl_matrix_get (uv2, 1, i)+offset);
                cvLine (canvas, pt1_on2, pt2_on2, red, thickness, lineType, shift);
            }
        }
        else
            if (in_idx < n_corr-1) in_idx++;
    } //draw outliers

    // clean up
    gslu_index_free (sel1_sort);
    gslu_index_free (sel2_sort);

    if (!use_color_code)
        gslu_index_free (clr_idx_vec);
}

void 
vis_plot_correspondences (IplImage *img1, IplImage *img2, 
                          const gslu_index *sel1, const gslu_index *sel2, 
                          const gsl_matrix *uv1, const gsl_matrix *uv2, 
                          bool stack_type, int in_or_out, const char *named_window,
                          gslu_index *clr_idx_vec, const float scale)
{
    // scale image to be seen properly on screen
    //const float scale = 0.5;

    // stack img1 and img2 together and convert to color if it is gray
    IplImage* img_plot = vis_plot_stack_imgs_cvt_color (img1, img2, stack_type);  // or STACK_VERT

    size_t offset = 0;
    if (stack_type == VIS_PLOT_STACK_HORZ)
        offset = img1->width;
    else
        offset = img1->height;

    if (in_or_out & VIS_PLOT_IN) { // PLOT INLIERS
        _vis_plot_inliers (img_plot, stack_type, offset, clr_idx_vec, sel1, sel2, uv1, uv2);
    }

    if (in_or_out & VIS_PLOT_OUT) { // PLOT OUTLIERS
        _vis_plot_outliers (img_plot, stack_type, offset, clr_idx_vec, sel1, sel2, uv1, uv2);
    }

    // resize for viewing
    CvSize size = {img_plot->width*scale, img_plot->height*scale};
    IplImage *img_scaled = cvCreateImage (size, IPL_DEPTH_8U, 3);
    cvResize (img_plot, img_scaled, CV_INTER_LINEAR);

    cvNamedWindow (named_window, CV_WINDOW_AUTOSIZE);
    cvShowImage (named_window, img_scaled);
    cvReleaseImage (&img_plot);
    cvReleaseImage (&img_scaled);
}

/* given line equation epl = [a, b, c]
 * returns pt1 and pt2 to draw a line in the boundary
 * offset value will be used to control stacked image
 */
void 
_line_bounds(double xminus, double xplus, double yminus, double yplus, 
             gsl_matrix *epl,                                           // 3 x 1 matrix
             CvPoint *pt1, CvPoint *pt2,                                // pt1 and pt2 indicate boundary points
             bool stack_type)
{
    // line equation: epl(1)*x + epl(2)*y + epl(3) = 0
    //                    a x + b y + c = 0

    double a = gsl_matrix_get (epl,0,0);
    double b = gsl_matrix_get (epl,1,0);
    double c = gsl_matrix_get (epl,2,0);
    double slope = fabs (a/b);

    double dx = fabs (xplus - xminus);
    double dy = fabs (yplus - yminus);
    double ratio = dy /dx;

    if ( fabs(a) < 1e-6 && fabs(b) < 1e-6 ) {
        // line equation is incorrect
        (*pt1) = cvPoint (0.0, 0.0);
        (*pt2) = cvPoint (0.0, 0.0);
    }
    else {
        if (slope < ratio) {
            if (stack_type == VIS_PLOT_STACK_HORZ) {
                (*pt1) = cvPoint (xminus, -(a*xminus + c)/b);
                (*pt2) = cvPoint (xplus, -(a*xplus  + c)/b);
            }
            else {
                (*pt1) = cvPoint (xminus, -(a*xminus + c)/b);
                (*pt2) = cvPoint (xplus, -(a*xplus  + c)/b);
            }

        }
        else {
            if (stack_type == VIS_PLOT_STACK_HORZ) {
                (*pt1) = cvPoint (-(b*yminus + c)/a, yminus);
                (*pt2) = cvPoint (-(b*yplus  + c)/a, yplus);
            }
            else {
                (*pt1) = cvPoint (-(b*yminus + c)/a, yminus);
                (*pt2) = cvPoint (-(b*yplus  + c)/a, yplus);
            }
        }
    }
}

void 
_draw_epipolar_line (IplImage* img, const gsl_matrix *F, const gsl_vector *uv, 
                     const int x_lb, const int x_ub, const int y_lb, const int y_ub, 
                     const CvScalar line_color, const bool stack_type)
{
    // plot options
    int thickness = LINE_THICKNESS;	    // line thickness 0~255
    int lineType = 8;
    int shift = 0;

    // epl = F * u=uv_h = [a b c], where ax +by+ c = 0 defines a line
    GSLU_MATRIX_VIEW (uv_h, 3,1, {0.0, 0.0, 1.0});
    GSLU_MATRIX_VIEW (epl, 3,1);
    gsl_matrix_set (&uv_h.matrix, 0,0, gsl_vector_get (uv,0));
    gsl_matrix_set (&uv_h.matrix, 1,0, gsl_vector_get (uv,1));
    gslu_blas_mm (&epl.matrix, F, &uv_h.matrix);

    CvPoint pt1, pt2;       // end points for a line    
    _line_bounds(x_lb+1, x_ub-1, y_lb+1, y_ub-1, &epl.matrix, &pt1, &pt2, stack_type);
    cvLine (img, pt1, pt2, line_color, thickness, lineType, shift);
}

/* Draws an ellipse, centered at mu, corresponding to
 * the confidence region determined by Sigma, given chi2(2).
 * [1] "Linear Algebra and its Applications", Gibert Strang, p.335
 * [2] http://opencv.willowgarage.com/documentation/drawing_functions.html?highlight=ellipse#cvEllipse
 */
void 
_draw_ellipse (IplImage *img, const gsl_vector *mu, const gsl_matrix *cov, const double k2, 
               const CvScalar color, const bool stack_type)
{

    if (!(gsl_vector_get (mu, 0) > 0.0 && gsl_vector_get (mu, 1) > 0.0))
        return;

    GSLU_VECTOR_VIEW (eval, 2);
    GSLU_MATRIX_VIEW (evec, 2, 2);
    GSLU_MATRIX_VIEW (work, 2, 2);

    gsl_matrix_memcpy (&work.matrix, cov);                       // because this changes matrix when eig decomposition
    gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc (2);
    gsl_eigen_symmv (&work.matrix, &eval.vector, &evec.matrix, w);
    gsl_eigen_symmv_free (w);
    gsl_eigen_symmv_sort (&eval.vector, &evec.matrix, GSL_EIGEN_SORT_ABS_DESC);

    int r1 = (int) sqrt (gsl_vector_get (&eval.vector, 0)*k2);    // semi-major ellipse axis
    int r2 = (int) sqrt (gsl_vector_get (&eval.vector, 1)*k2);    // semi-minor ellipse axis
    /* PJO: taking out minus sign makes search regions align with epipolar lines */
    double angle = atan (gsl_matrix_get (&evec.matrix, 1, 0) / gsl_matrix_get (&evec.matrix, 0, 0))*RTOD;

    //printf ("angle = %g\n", angle);
    // cvEllipse uses int format for axes (0~65535)
    if (2*r1 > 65535) r1 = 32766;
    if (2*r2 > 65535) r2 = 32766;

    if (r1 > 0 && r2 > 0) {
        CvSize axes = cvSize (r1, r2);
        CvPoint center = cvPoint (gsl_vector_get (mu, 0), gsl_vector_get (mu, 1));

        // plot param
        double start_angle = 0;
        double end_angle = 360; 
        int thickness=LINE_THICKNESS;
        int lineType=8;
        int shift=0;

        if (SHOW_ELLIPSE_AXIS) {
            /* PJO: taking out minus sign makes search regions align with epipolar lines */
            double theta = angle;
            CvPoint r1_pt1, r1_pt2;     // end points for a line of the second principal axis
            CvPoint r2_pt1, r2_pt2;       

            // semi-major axis
            r1_pt1 = cvPoint (center.x + r1*cos (theta*DTOR), center.y + r1*sin (theta*DTOR));
            r1_pt2 = cvPoint (center.x - r1*cos (theta*DTOR), center.y - r1*sin (theta*DTOR));

            if (r1_pt1.x > 0 && r1_pt1.y > 0 && r1_pt2.x > 0 && r1_pt2.y > 0)
                cvLine (img, r1_pt1, r1_pt2, color, thickness, lineType, shift);

            // semi-minor axis
            r2_pt1 = cvPoint (center.x + r2*cos ((90+theta)*DTOR), center.y + r2*sin ((90+theta)*DTOR));
            r2_pt2 = cvPoint (center.x - r2*cos ((90+theta)*DTOR), center.y - r2*sin ((90+theta)*DTOR));

            if (r2_pt1.x > 0 && r2_pt1.y > 0 && r2_pt2.x > 0 && r2_pt2.y > 0)
                cvLine (img, r2_pt1, r2_pt2, color, thickness, lineType, shift);
        }

        // TODO: ERROR Assertion failed (axes.width >= 0 && axes.height >= 0 && thickness <= 255 && 0 <= shift && shift <= XY_SHIFT) in ellipse
        cvEllipse (img, center, axes, angle, start_angle, end_angle, color, thickness, lineType, shift);
    }
}

void 
vis_plot_pccs_ellipses (IplImage *img1, IplImage *img2, 
                        const gsl_matrix *uv1, const gsl_matrix *uv2p, const gsl_matrix *cov2p, const gsl_matrix *uv2,
                        const gsl_matrix *F, const double chiSquare2dof,
                        bool stack_type, const char *named_window, const float scale, int64_t dt)
{
    CvScalar lcolors[8]; 
    _gen_basic_colors (lcolors); // [r g b y m c w k]

    IplImage *canvas1 = 0;
    IplImage *canvas2 = 0;

    // duplicate imgs to work with
    if (img1->nChannels == 1) { // gray scale 
        canvas1 = cvCreateImage (cvGetSize (img1), IPL_DEPTH_8U, 3);
        canvas2 = cvCreateImage (cvGetSize (img2), IPL_DEPTH_8U, 3);
        cvCvtColor (img1, canvas1, CV_GRAY2BGR);
        cvCvtColor (img2, canvas2, CV_GRAY2BGR);    
    }
    else { // color 
        canvas1 = cvCloneImage (img1);
        canvas2 = cvCloneImage (img2);
    }

    //float scale = 0.5;
    int nsamps = 8;

    GSLU_MATRIX_VIEW (FT, 3,3);
    GSLU_MATRIX_VIEW (cov_mat, 2, 2);
    GSLU_MATRIX_VIEW (invcov_mat, 2, 2);
    gsl_matrix_transpose_memcpy (&FT.matrix, F);

    // plot pose instantiated epipolar lines on I1 and I2
    // --------------------------------------------
    for (int i=0; i<nsamps; i++) {
        // draw lines from uv2p on I1
        gsl_vector_const_view uv2p_i = gsl_matrix_const_column (uv2p, i);
        _draw_epipolar_line (canvas1, &FT.matrix, &uv2p_i.vector, 0, img1->width, 0, MIN(img1->height,img2->height), lcolors[i], stack_type);

        // draw lines from uv1 on I2
        gsl_vector_const_view uv1_i = gsl_matrix_const_column (uv1, i);
        _draw_epipolar_line (canvas2, F, &uv1_i.vector, 0, img1->width, 0, MIN(img1->height,img2->height), lcolors[i], stack_type);
    }

    // plot interest points on I1 and I2
    // ----------------------------------------
    int radius = DOT_SIZE;
    int circle_thickness = -1;	// line thickness when pos number, -1 for filled circle

    for (int i=0; i<nsamps; i++) {
        cvCircle (canvas1, cvPoint (gsl_matrix_get (uv1, 0, i), gsl_matrix_get (uv1, 1, i)),
                  radius, lcolors[i], circle_thickness, CV_AA, 0);  // uv on I1

        // RME: don't plot center of predicted ellipse
        //cvCircle (canvas2, cvPoint (gsl_matrix_get (uv2p, 0, i), gsl_matrix_get (uv2p, 1, i)), 
        //          radius, lcolors[7], circle_thickness, CV_AA, 0);  // uvp on I2
    }

    // plot the search ellipse centered on (u2p,v2p) on I2
    // ----------------------------------------------------
    int n2 = uv2->size2;
    GSLU_VECTOR_VIEW (uv2_j,2);
    for (int i=0; i<nsamps; i++) {
        gsl_vector_const_view sigma_col = gsl_matrix_const_column (cov2p, i);
        gslu_vector_reshape (&cov_mat.matrix, &sigma_col.vector, CblasTrans);
        gslu_matrix_inv (&invcov_mat.matrix, &cov_mat.matrix);
        gsl_vector_const_view uv2p_i = gsl_matrix_const_column (uv2p, i);
        _draw_ellipse (canvas2, &uv2p_i.vector, &cov_mat.matrix, chiSquare2dof, lcolors[i], stack_type);

        #if 1 //plot the points from (u2,v2) which lie inside the ellipse
        for (int j=0; j < n2; ++j) {
            gsl_vector_const_view uv2_j = gsl_matrix_const_column (uv2, j); 
            double dist = gslu_vector_mahal_dist (&uv2p_i.vector, &uv2_j.vector, &invcov_mat.matrix);

            if (dist*dist < chiSquare2dof) { // inside ellipse, plot !
                cvCircle (canvas2, cvPoint (gsl_matrix_get (uv2, 0, j), gsl_matrix_get (uv2, 1, j)), 
                          radius, lcolors[i], circle_thickness, CV_AA, 0);  // uvp on I2
            }
        }
        #endif
    }

    // stack two canvases
    IplImage* img_plot = 0;
    img_plot = vis_plot_stack_imgs (canvas1, canvas2, stack_type);  // or STACK_VERT

    // show dt 
    CvFont font;
    cvInitFont (&font, CV_FONT_HERSHEY_PLAIN, 2.0, 2.0, 0, 2, 8);
    CvScalar red = CV_RGB (255, 0, 0);
    CvPoint str_pos = cvPoint (50, 50);

    char str[30];
    snprintf (str, sizeof str, "dt = %g", ((double)dt)/(1E6));
    cvPutText (img_plot, str, str_pos, &font,  red);

    // resize for viewing
    CvSize size = {img_plot->width*scale, img_plot->height*scale};
    IplImage *img_scaled = cvCreateImage (size, IPL_DEPTH_8U, 3);
    cvResize (img_plot, img_scaled, CV_INTER_LINEAR);


    cvNamedWindow (named_window, CV_WINDOW_AUTOSIZE);
    cvShowImage (named_window, img_scaled);
    cvReleaseImage (&img_plot);
    cvReleaseImage (&canvas1);
    cvReleaseImage (&canvas2);
    cvReleaseImage (&img_scaled);
}

void 
_vis_plot_summary_F  (IplImage *img1, IplImage *img2, 
                     const gsl_matrix *uv1, const gsl_matrix *uv2, const gsl_matrix *F,
                     bool stack_type, const char *named_window, const float scale)
{
    CvScalar lcolors[8]; 
    _gen_basic_colors (lcolors); // [r g b y m c w k]

    IplImage *canvas1 = 0;
    IplImage *canvas2 = 0;

    // duplicate imgs to work with
    if (img1->nChannels == 1) { // gray scale 
        canvas1 = cvCreateImage (cvGetSize (img1), IPL_DEPTH_8U, 3);
        canvas2 = cvCreateImage (cvGetSize (img2), IPL_DEPTH_8U, 3);
        cvCvtColor (img1, canvas1, CV_GRAY2BGR);
        cvCvtColor (img2, canvas2, CV_GRAY2BGR);    
    }
    else { // color 
        canvas1 = cvCloneImage (img1);
        canvas2 = cvCloneImage (img2);
    }

    //float scale = 0.5;
    int npts = uv1->size2;

    GSLU_MATRIX_VIEW (FT, 3,3);
    gsl_matrix_transpose_memcpy (&FT.matrix, F);

    // plot pose instantiated epipolar lines on I1 and I2
    // --------------------------------------------
    for (int i=0; i<npts; i++) {
        // draw lines from uv2p on I1
        gsl_vector_const_view uv2_i = gsl_matrix_const_column (uv2, i);
        _draw_epipolar_line (canvas1, &FT.matrix, &uv2_i.vector, 0, img1->width, 0, MIN(img1->height,img2->height), lcolors[i%8], stack_type);

        // draw lines from uv1 on I2
        gsl_vector_const_view uv1_i = gsl_matrix_const_column (uv1, i);
        _draw_epipolar_line (canvas2, F, &uv1_i.vector, 0, img1->width, 0, MIN(img1->height,img2->height), lcolors[i%8], stack_type);
    }

    // plot sample points on I1 and I2
    // ----------------------------------------
    int radius = DOT_SIZE;
    int circle_thickness = -1;	// line thickness when pos number, -1 for filled circle

    for (int i=0; i<npts; i++) {
        cvCircle (canvas1, cvPoint (gsl_matrix_get (uv1, 0, i), gsl_matrix_get (uv1, 1, i)),
                  radius, lcolors[i%8], circle_thickness, CV_AA, 0);  // uv1 on I1
        cvCircle (canvas2, cvPoint (gsl_matrix_get (uv2, 0, i), gsl_matrix_get (uv2, 1, i)),
                  radius, lcolors[i%8], circle_thickness, CV_AA, 0);  // uv2 on I2
    }

    // stack two canvases
    IplImage* img_plot = 0;
    img_plot = vis_plot_stack_imgs (canvas1, canvas2, stack_type);  // or STACK_VERT

    // resize for viewing
    CvSize size = {img_plot->width*scale, img_plot->height*scale};
    IplImage *img_scaled = cvCreateImage (size, IPL_DEPTH_8U, 3);
    cvResize (img_plot, img_scaled, CV_INTER_LINEAR);

    cvNamedWindow (named_window, CV_WINDOW_AUTOSIZE);
    cvShowImage (named_window, img_scaled);
    cvReleaseImage (&img_plot);
    cvReleaseImage (&canvas1);
    cvReleaseImage (&canvas2);
    cvReleaseImage (&img_scaled);
}

void 
_vis_plot_summary_H  (IplImage *img1, IplImage *img2, 
                     const gsl_matrix *uv1, const gsl_matrix *uv2, const gsl_matrix *H,
                     bool stack_type, const char *named_window, const float scale)
{
    CvScalar lcolors[8]; 
    _gen_basic_colors (lcolors); // [r g b y m c w k]

    IplImage *canvas1 = 0; 
    IplImage *canvas2 = 0;

    // duplicate imgs to work with
    if (img1->nChannels == 1) { // gray scale 
        canvas1 = cvCreateImage (cvGetSize (img1), IPL_DEPTH_8U, 3);
        canvas2 = cvCreateImage (cvGetSize (img2), IPL_DEPTH_8U, 3);
        cvCvtColor (img1, canvas1, CV_GRAY2BGR);
        cvCvtColor (img2, canvas2, CV_GRAY2BGR);    
    }
    else { // color 
        canvas1 = cvCloneImage (img1);
        canvas2 = cvCloneImage (img2);
    }

    //float scale = 0.5;
    int npts = uv1->size2;

    // plot sample points on I1 and I2
    // ----------------------------------------
    int radius = DOT_SIZE;
    int circle_thickness = -1;	// line thickness when pos number, -1 for filled circle

    GSLU_MATRIX_VIEW (Hinv, 3, 3);
    gslu_matrix_inv (&Hinv.matrix, H);
    gsl_matrix *uv1p = gsl_matrix_alloc (uv1->size1, uv1->size2);
    gsl_matrix *uv2p = gsl_matrix_alloc (uv2->size1, uv2->size2);

    vis_homog_project (H, uv1, uv1p);
    vis_homog_project (&Hinv.matrix, uv2, uv2p);

    for (int i=0; i<npts; i++) {
        cvCircle (canvas1, cvPoint (gsl_matrix_get (uv1, 0, i), gsl_matrix_get (uv1, 1, i)),
                  radius, lcolors[i%8], circle_thickness, CV_AA, 0);  // uv1 on I1
        cvCircle (canvas2, cvPoint (gsl_matrix_get (uv1p, 0, i), gsl_matrix_get (uv1p, 1, i)),
                  radius, lcolors[i%8], 1, CV_AA, 0);  // uv1p on I2

        cvCircle (canvas2, cvPoint (gsl_matrix_get (uv2, 0, i), gsl_matrix_get (uv2, 1, i)),
                  radius, lcolors[i%8], circle_thickness, CV_AA, 0);  // uv2 on I2
        cvCircle (canvas1, cvPoint (gsl_matrix_get (uv2p, 0, i), gsl_matrix_get (uv2p, 1, i)),
                  radius, lcolors[i%8], 1, CV_AA, 0);  // uv2p on I1
    }

    // stack two canvases
    IplImage* img_plot = 0;
    img_plot = vis_plot_stack_imgs (canvas1, canvas2, stack_type);  // or STACK_VERT

    // resize for viewing
    CvSize size = {img_plot->width*scale, img_plot->height*scale};
    IplImage *img_scaled = cvCreateImage (size, IPL_DEPTH_8U, 3);
    cvResize (img_plot, img_scaled, CV_INTER_LINEAR);

    cvNamedWindow (named_window, CV_WINDOW_AUTOSIZE);
    cvShowImage (named_window, img_scaled);
    cvReleaseImage (&img_plot);
    cvReleaseImage (&canvas1);
    cvReleaseImage (&canvas2);
    cvReleaseImage (&img_scaled);

    gslu_matrix_free (uv1p);
    gslu_matrix_free (uv2p);
}

// GNU PLOTS
//-----------------------------------------------------------------//
int 
vis_plot_coordinate_frame (const gsl_vector *center, const gsl_matrix *R, 
                           FILE *gp, int nframe, int z_axis_color)
{

    // center gsl_vector c & rotation matrix R = [r1 r2 r3]
    double cx = gsl_vector_get (center, 0);
    double cy = gsl_vector_get (center, 1);
    double cz = gsl_vector_get (center, 2);
    double r1x = gsl_matrix_get (R, 0,0); 
    double r1y = gsl_matrix_get (R, 0,1); 
    double r1z = gsl_matrix_get (R, 0,2);
    double r2x = gsl_matrix_get (R, 1,0);
    double r2y = gsl_matrix_get (R, 1,1);
    double r2z = gsl_matrix_get (R, 1,2);
    double r3x = gsl_matrix_get (R, 2,0);
    double r3y = gsl_matrix_get (R, 2,1);
    double r3z = gsl_matrix_get (R, 2,2);

    //fprintf (gp, "set label \"cam%d\" at first %g, first %g, first %g\n", nframe, cx,cy,cz);
    fprintf (gp, "set arrow %d from %g,%g,%g to %g,%g,%g ls 2\n"
                 ,3*nframe, cx,cy,cz, cx+r1x,cy+r1y,cz+r1z);
    fprintf (gp, "set arrow %d from %g,%g,%g to %g,%g,%g ls 2\n"
                 ,3*nframe+1, cx,cy,cz, cx+r2x,cy+r2y,cz+r2z);
    fprintf (gp, "set arrow %d from %g,%g,%g to %g,%g,%g ls %d\n"
                 ,3*nframe+2, cx,cy,cz, cx+r3x,cy+r3y,cz+r3z, 1+z_axis_color);

    nframe++;
    return nframe;
}

void
vis_plot_relpose (FILE *gp, const gsl_vector *x21, const double scale, const gsl_vector *nav)
{
    int _nframe = 1;

    // camera 1 is fixed
    GSLU_VECTOR_VIEW (x_cam1, 6, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); //180*DTOR, 0, 90*DTOR = z-down
    gsl_vector_const_view t1 = gsl_vector_const_subvector (&x_cam1.vector, 0, 3);
    gsl_vector_const_view rph1 = gsl_vector_const_subvector (&x_cam1.vector, 3, 3);
    GSLU_MATRIX_VIEW (R1, 3,3);
    so3_rotxyz (R1.matrix.data, rph1.vector.data);

    // camera 2 pose from x21
    GSLU_MATRIX_VIEW (R2, 3,3);
    GSLU_VECTOR_VIEW (t2, 3);

    // navigation prior
    GSLU_MATRIX_VIEW (nav_R2, 3,3);
    GSLU_VECTOR_VIEW (nav_t2, 3);

    if (x21->size == 5) {
        GSLU_VECTOR_VIEW (b_ext, 3,{0.0, 0.0, 1.0});
        GSLU_VECTOR_VIEW (t21, 3);
        GSLU_MATRIX_VIEW (R21, 3,3);
        gsl_vector_set (&b_ext.vector, 0, gsl_vector_get (x21, 0));
        gsl_vector_set (&b_ext.vector, 1, gsl_vector_get (x21, 1));
        gsl_vector_set (&b_ext.vector, 2, scale);     // last elem == scale
        dm_dm2trans (b_ext.vector.data, t21.vector.data, NULL);
        gsl_vector_const_view rph21 = gsl_vector_const_subvector (x21,2,3);
        so3_rotxyz (R21.matrix.data, rph21.vector.data);

        // Rw2 = Rw1*R21'
        // tw2 = -Rw1*R21'*t21 = -Rw2*t21
        gslu_blas_mmT (&R2.matrix, &R1.matrix, &R21.matrix);
        gslu_blas_mv (&t2.vector, &R2.matrix, &t21.vector);
        gsl_vector_scale (&t2.vector, -1.0);
    }
    else if (x21->size == 6) {
        GSLU_MATRIX_VIEW (R21, 3,3);
        gsl_vector_const_view t21 = gsl_vector_const_subvector (x21,0,3);
        gsl_vector_const_view rph21 = gsl_vector_const_subvector (x21,3,3);
        so3_rotxyz (R21.matrix.data, rph21.vector.data);

        // Rw2 = Rw1*R21'
        // tw2 = -Rw1*R21'*t21 = -Rw2*t21
        gslu_blas_mmT (&R2.matrix, &R1.matrix, &R21.matrix);
        gslu_blas_mv (&t2.vector, &R2.matrix, &t21.vector);
        gsl_vector_scale (&t2.vector, -1.0);
    }
    else
        printf ("relative pose should be either 5 or 6 dof\n");

    

    // set range
    double margin = 1.0; // meter
    double xmax = gsl_vector_get (&t2.vector, 0) > 0.0 ? gsl_vector_get (&t2.vector, 0)+margin :  margin;
    double xmin = gsl_vector_get (&t2.vector, 0) < 0.0 ? gsl_vector_get (&t2.vector, 0)-margin : -margin;
    double ymax = gsl_vector_get (&t2.vector, 1) > 0.0 ? gsl_vector_get (&t2.vector, 1)+margin :  margin;
    double ymin = gsl_vector_get (&t2.vector, 1) < 0.0 ? gsl_vector_get (&t2.vector, 1)-margin : -margin;
    double zmax = gsl_vector_get (&t2.vector, 2) > 0.0 ? gsl_vector_get (&t2.vector, 2)+margin :  margin;
    double zmin = gsl_vector_get (&t2.vector, 2) < 0.0 ? gsl_vector_get (&t2.vector, 2)-margin : -margin;

    fprintf (gp, "set xrange [%g:%g]\n", xmin, xmax);
    fprintf (gp, "set yrange [%g:%g]\n", ymin, ymax);
    fprintf (gp, "set zrange [%g:%g]\n", zmin, zmax);

    // plot coordinates 
    _nframe = vis_plot_coordinate_frame (&t1.vector, &R1.matrix, gp, _nframe, 0);
    _nframe = vis_plot_coordinate_frame (&t2.vector, &R2.matrix, gp, _nframe, 0);

    if (nav) {
        GSLU_MATRIX_VIEW (nav_R21, 3,3);
        gsl_vector_const_view nav_t21 = gsl_vector_const_subvector (nav,0,3);
        gsl_vector_const_view nav_rph21 = gsl_vector_const_subvector (nav,3,3);
        so3_rotxyz (nav_R21.matrix.data, nav_rph21.vector.data);

        // Rw2 = Rw1*R21'
        // tw2 = -Rw1*R21'*t21 = -Rw2*t21
        gslu_blas_mmT (&nav_R2.matrix, &R1.matrix, &nav_R21.matrix);
        gslu_blas_mv (&nav_t2.vector, &nav_R2.matrix, &nav_t21.vector);
        gsl_vector_scale (&nav_t2.vector, -1.0);

        _nframe = vis_plot_coordinate_frame (&nav_t2.vector, &nav_R2.matrix, gp, _nframe, 3);
    }

    fprintf (gp, "splot -4 notitle ls 0\n"); // plot ground

    // NOTE: 3D axis equal is not supported by gnuplot, only 2D so far.
    fprintf (gp, "set size ratio -1\n");     // 2D axis equal
    fprintf (gp, "set view equal\n");       

}

void 
vis_plot_3dpts (FILE *gp, const gsl_matrix *X)
{

    // X = 3 x n matrix 
    assert (X->size1 == 3);
    int n = X->size2;

    gsl_vector_const_view xdata = gsl_matrix_const_row (X, 0);
    gsl_vector_const_view ydata = gsl_matrix_const_row (X, 1);
    gsl_vector_const_view zdata = gsl_matrix_const_row (X, 2);

    double xmax = gsl_stats_max (xdata.vector.data, 1, n);
    double xmin = gsl_stats_min (xdata.vector.data, 1, n);
    double ymax = gsl_stats_max (ydata.vector.data, 1, n);
    double ymin = gsl_stats_min (ydata.vector.data, 1, n);
    double zmax = gsl_stats_max (zdata.vector.data, 1, n);
    double zmin = gsl_stats_min (zdata.vector.data, 1, n);

    //char buffer[100000]; // 100 buf per each 3d point (max 1000 points)
    char *buffer = calloc (1, sizeof (char) * 50 * (n+1));
    fprintf (gp, "set xrange [%g:%g]\n", xmin, xmax);
    fprintf (gp, "set yrange [%g:%g]\n", ymin, ymax);
    fprintf (gp, "set zrange [%g:%g]\n", zmin, zmax);

    int j = snprintf(buffer, 20, "splot \"< echo -e '");

    for (int i=0; i<n; i++) {
        j += snprintf(buffer+j, 40, "%2.2g %2.2g %2.2g", 
                      gsl_matrix_get (X,0,i),gsl_matrix_get (X,1,i),gsl_matrix_get (X,2,i));

        if (i!=n-1)
            j += snprintf(buffer+j, 5, "\\n ");
    }

    j += snprintf(buffer+j, 25, "'\" notitle lt 9 pt 7\n");
    fprintf (gp, "%s", buffer);

    free (buffer);

    // NOTE: 3D axis equal is not supported by gnuplot, only 2D so far.
    fprintf (gp, "set size ratio -1\n");     // 2D axis equal
    fprintf (gp, "set view equal\n");       
}

void 
vis_plot_relpose_3dpts (FILE *gp, const gsl_vector *x21, const gsl_matrix *X, const double scale, const gsl_vector *nav)
{
    int _nframe = 1;

    // camera 1 is fixed
    GSLU_VECTOR_VIEW (x_cam1, 6, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); //180*DTOR, 0, 90*DTOR = z-down
    gsl_vector_const_view t1 = gsl_vector_const_subvector (&x_cam1.vector, 0, 3);
    gsl_vector_const_view rph1 = gsl_vector_const_subvector (&x_cam1.vector, 3, 3);
    GSLU_MATRIX_VIEW (R1, 3,3);
    so3_rotxyz (R1.matrix.data, rph1.vector.data);

    // camera 2 pose from x21
    GSLU_MATRIX_VIEW (R2, 3,3);
    GSLU_VECTOR_VIEW (t2, 3);

    // navigation prior
    GSLU_MATRIX_VIEW (nav_R2, 3,3);
    GSLU_VECTOR_VIEW (nav_t2, 3);

    if (x21->size == 5) {
        GSLU_VECTOR_VIEW (b_ext, 3,{0.0, 0.0, 1.0});
        GSLU_VECTOR_VIEW (t21, 3);
        GSLU_MATRIX_VIEW (R21, 3,3);
        gsl_vector_set (&b_ext.vector, 0, gsl_vector_get (x21, 0));
        gsl_vector_set (&b_ext.vector, 1, gsl_vector_get (x21, 1));
        gsl_vector_set (&b_ext.vector, 2, scale);
        dm_dm2trans (b_ext.vector.data, t21.vector.data, NULL);
        gsl_vector_const_view rph21 = gsl_vector_const_subvector (x21,2,3);
        so3_rotxyz (R21.matrix.data, rph21.vector.data);

        // Rw2 = Rw1*R21'
        // tw2 = -Rw1*R21'*t21 = -Rw2*t21
        gslu_blas_mmT (&R2.matrix, &R1.matrix, &R21.matrix);
        gslu_blas_mv (&t2.vector, &R2.matrix, &t21.vector);
        gsl_vector_scale (&t2.vector, -1.0);
    }
    else if (x21->size == 6) {
        GSLU_MATRIX_VIEW (R21, 3,3);
        gsl_vector_const_view t21 = gsl_vector_const_subvector (x21,0,3);
        gsl_vector_const_view rph21 = gsl_vector_const_subvector (x21,3,3);
        so3_rotxyz (R21.matrix.data, rph21.vector.data);

        // Rw2 = Rw1*R21'
        // tw2 = -Rw1*R21'*t21 = -Rw2*t21
        gslu_blas_mmT (&R2.matrix, &R1.matrix, &R21.matrix);
        gslu_blas_mv (&t2.vector, &R2.matrix, &t21.vector);
        gsl_vector_scale (&t2.vector, -1.0);
    }
    else
        printf ("relative pose should be either 5 or 6 dof\n");

    if (nav) {
        GSLU_MATRIX_VIEW (nav_R21, 3,3);
        gsl_vector_const_view nav_t21 = gsl_vector_const_subvector (nav,0,3);
        gsl_vector_const_view nav_rph21 = gsl_vector_const_subvector (nav,3,3);
        so3_rotxyz (nav_R21.matrix.data, nav_rph21.vector.data);

        // Rw2 = Rw1*R21'
        // tw2 = -Rw1*R21'*t21 = -Rw2*t21
        gslu_blas_mmT (&nav_R2.matrix, &R1.matrix, &nav_R21.matrix);
        gslu_blas_mv (&nav_t2.vector, &nav_R2.matrix, &nav_t21.vector);
        gsl_vector_scale (&nav_t2.vector, -1.0);
    }

    // X = 3 x n matrix 
    assert (X->size1 == 3);
    int n = X->size2;
    gsl_vector_const_view xdata = gsl_matrix_const_row (X, 0);
    gsl_vector_const_view ydata = gsl_matrix_const_row (X, 1);
    gsl_vector_const_view zdata = gsl_matrix_const_row (X, 2);

    // set range
    double margin = 1.0; // meter
    double xmax_cam = gsl_vector_get (&t2.vector, 0) > 0.0 ? gsl_vector_get (&t2.vector, 0)+margin :  margin;
    double xmin_cam = gsl_vector_get (&t2.vector, 0) < 0.0 ? gsl_vector_get (&t2.vector, 0)-margin : -margin;
    double ymax_cam = gsl_vector_get (&t2.vector, 1) > 0.0 ? gsl_vector_get (&t2.vector, 1)+margin :  margin;
    double ymin_cam = gsl_vector_get (&t2.vector, 1) < 0.0 ? gsl_vector_get (&t2.vector, 1)-margin : -margin;
    double zmax_cam = gsl_vector_get (&t2.vector, 2) > 0.0 ? gsl_vector_get (&t2.vector, 2)+margin :  margin;
    double zmin_cam = gsl_vector_get (&t2.vector, 2) < 0.0 ? gsl_vector_get (&t2.vector, 2)-margin : -margin;
    double xmax_3pt = gsl_stats_max (xdata.vector.data, 1, n);
    double xmin_3pt = gsl_stats_min (xdata.vector.data, 1, n);
    double ymax_3pt = gsl_stats_max (ydata.vector.data, 1, n);
    double ymin_3pt = gsl_stats_min (ydata.vector.data, 1, n);
    double zmax_3pt = gsl_stats_max (zdata.vector.data, 1, n);
    double zmin_3pt = gsl_stats_min (zdata.vector.data, 1, n);
    double xmin = xmin_cam < xmin_3pt ? xmin_cam : xmin_3pt;
    double xmax = xmax_cam > xmax_3pt ? xmax_cam : xmax_3pt;
    double ymin = ymin_cam < ymin_3pt ? ymin_cam : ymin_3pt;
    double ymax = ymax_cam > ymax_3pt ? ymax_cam : ymax_3pt;
    double zmin = zmin_cam < zmin_3pt ? zmin_cam : zmin_3pt;
    double zmax = zmax_cam > zmax_3pt ? zmax_cam : zmax_3pt;
    fprintf (gp, "set xrange [%g:%g]\n", xmin, xmax);
    fprintf (gp, "set yrange [%g:%g]\n", ymin, ymax);
    fprintf (gp, "set zrange [%g:%g]\n", zmin, zmax);

    // plot coordinates 
    _nframe = vis_plot_coordinate_frame (&t1.vector, &R1.matrix, gp, _nframe, 0);
    _nframe = vis_plot_coordinate_frame (&t2.vector, &R2.matrix, gp, _nframe, 0);

    if (nav)  
        _nframe = vis_plot_coordinate_frame (&nav_t2.vector, &nav_R2.matrix, gp, _nframe, 3);

    // plot 3d points
    char *buffer = calloc (1, sizeof (char) * 50 * (n+1));
    int j = snprintf(buffer, 20, "splot \"< echo -e '");
    for (int i=0; i<n; i++) {
        j += snprintf(buffer+j, 40, "%2.2g %2.2g %2.2g", 
                      gsl_matrix_get (X,0,i),gsl_matrix_get (X,1,i),gsl_matrix_get (X,2,i));

        if (i!=n-1)
            j += snprintf(buffer+j, 5, "\\n ");
    }
    j += snprintf(buffer+j, 25, "'\" notitle lt 9 pt 7\n");
    fprintf (gp, "%s", buffer);

    free (buffer);

    // NOTE: 3D axis equal is not supported by gnuplot, only 2D so far.
    fprintf (gp, "set size ratio -1\n");     // 2D axis equal
    fprintf (gp, "set view equal\n");       
}



// PLOT_DEBUG_T
//-----------------------------------------------------------------//
gslu_index *
_vis_plot_clr_code_inliers_alloc (const perllcm_van_plot_debug_t *pd)
{
    if (!pd->n_in)
        return NULL;

    gslu_index *clr_idx_vec = gslu_index_alloc (pd->n_in);
    gslu_index_set_zero (clr_idx_vec);

    for (size_t ii=0; ii<pd->n_in; ii++) {
        if (pd->isel[ii] < pd->npts_each_type[0]){
            gslu_index_set (clr_idx_vec, ii, 0);
        }

        for (size_t n=1; n<pd->n_feat_types; n++) {
            if (pd->npts_each_type[n-1]<=pd->isel[ii] && pd->isel[ii]<pd->npts_each_type[n]) {
                gslu_index_set (clr_idx_vec, ii, n);
            }
        }        
    }

    return clr_idx_vec;
}

gslu_index *
_vis_plot_clr_code_pccs_alloc (const perllcm_van_plot_debug_t *pd)
{
    if (!pd->n_in_pccs)
        return NULL;

    gslu_index *clr_idx_vec = gslu_index_alloc (pd->n_in_pccs);
    for (size_t n=0; n<pd->n_feat_types; n++) {
        if (pd->npts_each_type[n]) {
            if (n==0) {
                gslu_index *subvec = gslu_index_alloc (pd->npts_each_type[n]);
                gslu_index_set_all (subvec, n);
                gslu_index_view clr_idx_vec_sub = gslu_index_subvector (clr_idx_vec, 0, pd->npts_each_type[n]);
                gslu_index_memcpy (&clr_idx_vec_sub.vector, subvec);
                gslu_index_free (subvec);
            }
            else {
                gslu_index *subvec = gslu_index_alloc (pd->npts_each_type[n]-pd->npts_each_type[n-1]);
                gslu_index_set_all (subvec, n);
                gslu_index_view clr_idx_vec_sub = gslu_index_subvector (clr_idx_vec, pd->npts_each_type[n-1], pd->npts_each_type[n]-pd->npts_each_type[n-1]);
                gslu_index_memcpy (&clr_idx_vec_sub.vector, subvec);
                gslu_index_free (subvec);
            }
        }
    }

    return clr_idx_vec;
}

void
vis_plot_pccs_debugplot (const perllcm_van_plot_debug_t *pd, 
                         IplImage *imgi, IplImage *imgj,
                         const gslu_index *sel1, const gslu_index *sel2, 
                         const gsl_matrix *uv1, const gsl_matrix *uv2,
                         const float scale)
{
    gslu_index *clr_idx_vec = _vis_plot_clr_code_pccs_alloc (pd);
    vis_plot_correspondences (imgi, imgj, sel1, sel2, uv1, uv2, 
                              VIS_PLOT_STACK_HORZ, VIS_PLOT_IN, PLOT_WIN_PUTCORR, 
                              clr_idx_vec, scale);

    gslu_index_free (clr_idx_vec);
}

void
_vis_plot_display_results_on_image (const perllcm_van_plot_debug_t *pd, IplImage *imgi)
{
    CvFont font;
    cvInitFont (&font, CV_FONT_HERSHEY_PLAIN, 3.0, 3.0, 0, 2, 8);
    CvScalar green = CV_RGB (0, 255, 0);
    CvScalar red = CV_RGB (255, 0, 0);
    CvScalar cyan = CV_RGB (0, 255, 255);
    CvPoint gic_pt = cvPoint (50, 50);
    CvPoint reg_pt = cvPoint (50, 100);

    char gic_str[30], reg_str[30];

    if (pd->model_gic == PERLLCM_VAN_PLOT_DEBUG_T_GIC_H)
        snprintf (gic_str, sizeof gic_str, "GIC: H");
    else if (pd->model_gic == PERLLCM_VAN_PLOT_DEBUG_T_GIC_F)
        snprintf (gic_str, sizeof gic_str, "GIC: F");
    else
        snprintf (gic_str, sizeof gic_str, "GIC: N/A");
    cvPutText (imgi, gic_str, gic_pt, &font,  cyan);

    if (pd->reg_result == PERLLCM_VAN_PLOT_DEBUG_T_REG_SUCC) {
        snprintf (reg_str, sizeof gic_str, "Reg: Y");
        cvPutText (imgi, reg_str, reg_pt, &font,  green);
    }
    else {
        // not registered. why?
        if (pd->errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_PCCS) {
            snprintf (reg_str, sizeof gic_str, "Reg: N (PCCS)");
        }
        else if (pd->errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_INLIERS_E ||
            pd->errmsg == PERLLCM_VAN_VLINK_T_MSG_MIN_INLIERS_H) {
            snprintf (reg_str, sizeof gic_str, "Reg: N (INLIERS)");
        }
        else if (pd->errmsg == PERLLCM_VAN_VLINK_T_MSG_SBA_E_ERROR ||
            pd->errmsg == PERLLCM_VAN_VLINK_T_MSG_SBA_H_ERROR) {
            snprintf (reg_str, sizeof gic_str, "Reg: N (SBA)");
        }
        else if (pd->errmsg == PERLLCM_VAN_VLINK_T_MSG_MDIST_NAV) {
            snprintf (reg_str, sizeof gic_str, "Reg: N (MAHAL)");
        }
        else if (pd->errmsg == PERLLCM_VAN_VLINK_T_MSG_TRI_CONST) {
            snprintf (reg_str, sizeof gic_str, "Reg: N (TRI)");
        }
        else
            snprintf (reg_str, sizeof gic_str, "Reg: N");
        cvPutText (imgi, reg_str, reg_pt, &font,  red);
    }
}

void
vis_plot_in_and_outliers_debugplot (const perllcm_van_plot_debug_t *pd, 
                                    IplImage *imgi, IplImage *imgj,
                                    gsl_matrix *uvi_sel, gsl_matrix *uvj_sel,
                                    int in_or_out, const float scale)
{
    // write gic and reg information on images
    //---------------------------------------------------
    _vis_plot_display_results_on_image (pd, imgi);

    // plot correspondences
    //---------------------------------------------------
    int stack_type = VIS_PLOT_STACK_HORZ;
    gslu_index *sel = gslu_index_alloc (pd->n_in);

    for (size_t ii=0; ii<pd->n_in; ii++)
        gslu_index_set (sel, ii, pd->isel[ii]);

    if (in_or_out == VIS_PLOT_INOUT) {
        vis_plot_correspondences (imgi, imgj, sel, sel, uvi_sel, uvj_sel,
                                  stack_type, VIS_PLOT_INOUT, PLOT_WIN_IN_OUT, NULL, scale);
    }
    else if (in_or_out == VIS_PLOT_IN) {
        gslu_index *clr_idx_vec = _vis_plot_clr_code_inliers_alloc (pd);
        vis_plot_correspondences (imgi, imgj, sel, sel, uvi_sel, uvj_sel,
                                  VIS_PLOT_STACK_HORZ, VIS_PLOT_IN, PLOT_WIN_IN, clr_idx_vec, scale);
        gslu_index_free (clr_idx_vec);
    }

    //clean up
    gslu_index_free (sel);

}

void
_vis_plot_display_nlink_on_image (const perllcm_van_plot_debug_t *pd, IplImage *imgi)
{
    CvFont font;
    cvInitFont (&font, CV_FONT_HERSHEY_PLAIN, 3.0, 3.0, 0, 2, 8);
    CvScalar red = CV_RGB (255, 0, 0);
    CvPoint str_pos = cvPoint (50, 50);

    char str[30];
    snprintf (str, sizeof str, "# link %d", pd->nlink_remaining);
    cvPutText (imgi, str, str_pos, &font,  red);
}

int
vis_plot_manual_verification_debugplot (const perllcm_van_plot_debug_t *pd, 
                                        IplImage *imgi, IplImage *imgj,
                                        gsl_matrix *uvi_sel, gsl_matrix *uvj_sel, 
                                        const float scale)
{
    // write gic and reg information on images
    //---------------------------------------------------
    _vis_plot_display_nlink_on_image (pd, imgi);

    // plot correspondences
    //---------------------------------------------------
    gslu_index *sel = gslu_index_alloc (pd->n_in);

    for (size_t ii=0; ii<pd->n_in; ii++)
        gslu_index_set (sel, ii, pd->isel[ii]);

    gslu_index *clr_idx_vec = _vis_plot_clr_code_inliers_alloc (pd);
    vis_plot_correspondences (imgi, imgj, sel, sel, uvi_sel, uvj_sel,
                              VIS_PLOT_STACK_HORZ, VIS_PLOT_IN, PLOT_WIN_VERIFY, clr_idx_vec, scale);

    cvMoveWindow(PLOT_WIN_VERIFY, 0, 100);

    gslu_index_free (clr_idx_vec);

    //clean up
    gslu_index_free (sel);

    int msg = PERLLCM_VAN_PLOT_DEBUG_T_MSG_INVALID;

    // and what do you think?
    char key = cvWaitKey(0); 

    switch (key) {
    case '\n':   // enter
    case 'y':    // yes
        //printf ("yes\n");
        msg = PERLLCM_VAN_PLOT_DEBUG_T_MSG_VALID;
        break;
    case '\b':  // backspace
    case 'n':   // no
        //printf ("no\n");
        msg = PERLLCM_VAN_PLOT_DEBUG_T_MSG_INVALID;
        break;
    case 'q':
        msg = PERLLCM_VAN_PLOT_DEBUG_T_MSG_QUIT;
        break;
    case 'b':
        msg = PERLLCM_VAN_PLOT_DEBUG_T_MSG_BATCH;
        break;
    default:
        //printf ("considered to be no\n");
        break;
    }

    return msg;

}

void
vis_plot_pccs_ellipses_plotdebug (const perllcm_van_plot_debug_t *pd, 
                                  IplImage *imgi, IplImage *imgj,
                                  gsl_matrix *uvi, gsl_matrix *uvj, 
                                  const bool vice_versa, const float scale, int64_t dt)
{
    if (!(imgi && imgj && uvi && uvj))
        return;

    // when pccs returned 0 for other error (ex. chol error), zero vector and matrices are returned in pd
    float check_uv1 = 0.0;
    for (size_t i=0; i<16; i++)
        check_uv1 += pd->uv1_sample[i];

    if (check_uv1 > 0) {
        GSLU_MATRIX_VIEW (F21, 3,3);
        GSLU_MATRIX_VIEW (F12, 3,3);
        gsl_matrix_const_view F21_view = gsl_matrix_const_view_array (pd->F21, 3,3);
        gsl_matrix_memcpy (&F21.matrix, &F21_view.matrix);
        gsl_matrix_transpose_memcpy (&F12.matrix, &F21.matrix);

        // read from pd and store it in gsl double format for plotting
        GSLU_MATRIX_VIEW (uv1_sample, 2,8);   GSLU_MATRIX_VIEW (uv2_sample, 2,8);
        GSLU_MATRIX_VIEW (uv2p_sample, 2,8);  GSLU_MATRIX_VIEW (uv1p_sample, 2,8);
        GSLU_MATRIX_VIEW (cov2p_sample, 4,8); GSLU_MATRIX_VIEW (cov1p_sample, 4,8);

        gsl_matrix_float_const_view uv1_sample_view = gsl_matrix_float_const_view_array (pd->uv1_sample, 2, 8);
        gsl_matrix_float_const_view uv2_sample_view = gsl_matrix_float_const_view_array (pd->uv2_sample, 2, 8);
        gsl_matrix_float_const_view uv2p_sample_view = gsl_matrix_float_const_view_array (pd->uv2p_sample, 2, 8);
        gsl_matrix_float_const_view uv1p_sample_view = gsl_matrix_float_const_view_array (pd->uv1p_sample, 2, 8);
        gsl_matrix_float_const_view cov2p_sample_view = gsl_matrix_float_const_view_array (pd->cov2p_sample, 4, 8);
        gsl_matrix_float_const_view cov1p_sample_view = gsl_matrix_float_const_view_array (pd->cov1p_sample, 4, 8);

        GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix_float, &uv1_sample_view.matrix, gsl_matrix, &uv1_sample.matrix);
        GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix_float, &uv2_sample_view.matrix, gsl_matrix, &uv2_sample.matrix);
        GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix_float, &uv2p_sample_view.matrix, gsl_matrix, &uv2p_sample.matrix);
        GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix_float, &uv1p_sample_view.matrix, gsl_matrix, &uv1p_sample.matrix);
        GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix_float, &cov2p_sample_view.matrix, gsl_matrix, &cov2p_sample.matrix);
        GSLU_MATRIX_TYPEA_TO_TYPEB (gsl_matrix_float, &cov1p_sample_view.matrix, gsl_matrix, &cov1p_sample.matrix);

        vis_plot_pccs_ellipses (imgi, imgj, &uv1_sample.matrix, &uv2p_sample.matrix, &cov2p_sample.matrix, uvj, &F21.matrix, pd->chiSquare2dof, 
                                VIS_PLOT_STACK_HORZ, PLOT_WIN_SEARCH_ELLIPSES1, scale, dt);

        if (vice_versa) {
            vis_plot_pccs_ellipses (imgj, imgi, &uv2_sample.matrix, &uv1p_sample.matrix, &cov1p_sample.matrix, uvi, &F12.matrix, pd->chiSquare2dof, 
                                    VIS_PLOT_STACK_HORZ, PLOT_WIN_SEARCH_ELLIPSES2, scale, dt);
        }
    }
}

void
vis_plot_summary_debugplot (const perllcm_van_plot_debug_t *pd, 
                            IplImage *imgi, IplImage *imgj,
                            gsl_matrix *uv1_pccs, gsl_matrix *uv2_pccs,
                            const float scale)
{
    // write gic and reg information on images
    _vis_plot_display_results_on_image (pd, imgi);

    int stack_type = VIS_PLOT_STACK_HORZ;

    // sample if n_inliers are large
    size_t sample_stride = 1; 
    size_t nsamp = MIN (pd->n_in, SUMMARY_PLOT_NPTS_MAX);

    gsl_matrix *uv1_in = gsl_matrix_alloc (2, nsamp);
    gsl_matrix *uv2_in = gsl_matrix_alloc (2, nsamp);

    if (pd->n_in > SUMMARY_PLOT_NPTS_MAX)
        sample_stride = round (pd->n_in / SUMMARY_PLOT_NPTS_MAX);

    for (size_t ii=0; ii<nsamp; ii++) {
        const size_t jj = pd->isel[sample_stride*ii];
        assert (jj < uv1_pccs->size2);
        gsl_vector_const_view c1 = gsl_matrix_const_column (uv1_pccs, jj);
        gsl_vector_const_view c2 = gsl_matrix_const_column (uv2_pccs, jj);
        gsl_matrix_set_col (uv1_in, ii, &c1.vector);
        gsl_matrix_set_col (uv2_in, ii, &c2.vector);
    }


    if (pd->model_gic == PERLLCM_VAN_PLOT_DEBUG_T_GIC_F) {
        GSLU_MATRIX_VIEW (F21, 3,3);
        gsl_matrix_const_view F21_view = gsl_matrix_const_view_array (pd->model, 3,3);
        gsl_matrix_memcpy (&F21.matrix, &F21_view.matrix);
        
        _vis_plot_summary_F (imgi, imgj, uv1_in, uv2_in, &F21.matrix,
                             stack_type, PLOT_WIN_SUMMARY, scale);
    }
    else { // pd->model_gic == PERLLCM_VAN_PLOT_DEBUG_T_GIC_H
        GSLU_MATRIX_VIEW (H, 3,3);
        gsl_matrix_const_view H_view = gsl_matrix_const_view_array (pd->model, 3,3);
        gsl_matrix_memcpy (&H.matrix, &H_view.matrix);
        
        _vis_plot_summary_H (imgi, imgj, uv1_in, uv2_in, &H.matrix,
                             stack_type, PLOT_WIN_SUMMARY, scale);
    }

    //clean up
    gslu_matrix_free (uv1_in);
    gslu_matrix_free (uv2_in);

}
