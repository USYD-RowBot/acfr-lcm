#ifndef __PERLS_VISION_TRIANGULATE_H__
#define __PERLS_VISION_TRIANGULATE_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "perls-math/gsl_util_index.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vis_triangulate vis_triangulate_t;
struct vis_triangulate 
{
    gsl_matrix *X1;
    gsl_vector *alpha;
    gsl_vector *beta;
    gsl_vector *gamma;
};

/*
function [X1,alpha,beta,gamma] = triangulate(R,t,u1,v1,u2,v2,K);
%TRIANGULATE find lest-squares scene structure.
%   X = TRIANGULATE(R,t,u1,v1,u2,v2,K) returns the [3 x N] matrix X
%   representing the scene structure as described in camera 1's
%   coordinate frame.  u1,v2 and u2,v2 are [N x 1] feature coordinates
%   measured in pixels.  K is the [3 x 3] camera calibration matrix.
%   The scene points X are defined as the lengths along camera rays r1
%   and r2 which minimizes the perpendicular distance between the ray
%   pairs.  Note that scene scale is measured in units of baseline magnitude.
%
%   [X,ALPHA,BETA,GAMMA] = TRIANGULATE(R,t,u1,v1,u2,v2,K) also returns
%   the [1 x N] vectors ALPHA, BETA, and GAMMA corresponding to the
%   length along rays r1 and r2 which minimize the perpendicular distance
%   and GAMMA is the corresponding minimal perpendicular distance.
%
%   R and t are defined such that the camera projection matrices are of the
%   form: P1 = K[I | 0]    P2 = K[R | t]
%
%   This algorithm is based upon:
%   Horn, B.K.P.  Relative Orientation, MIT A.I. Memo #994 September 1987
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-22-2003      rme         Created and written.  This
%                                implementation is also inspired by Oscar
%                                Pizarro's triangulate.m
%    03-06-2010      rme         Ported to C and verified correct output.
*/

// alpha, beta, gamma are optional outputs, set to NULL if you do not wish to compute them
void
vis_triangulate (const gsl_matrix *K, const gsl_matrix *R, const gsl_vector *t, const gsl_matrix *uv1, const gsl_matrix *uv2,    // inputs
                 gsl_matrix *X1, gsl_vector *alpha, gsl_vector *beta, gsl_vector *gamma);                                       // outputs

vis_triangulate_t *
vis_triangulate_alloc (const gsl_matrix *K, const gsl_matrix *R, const gsl_vector *t, const gsl_matrix *uv1, const gsl_matrix *uv2);

// free 
void
vis_triangulate_free (vis_triangulate_t *tri);

/* impose triangulate constraint
 * find 3D points between lower and upper bound
 * find lb < X(j,:) < ub, j=0 for x, j=1 for y, j=2 for z component
 * returns the indeces allocated with size of number of points accepted
 */
gslu_index *
vis_triangulate_constraint_alloc (const gsl_matrix *X, const int j, const double lb, const double ub);


#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_TRIANGULATE_H__
