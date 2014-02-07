#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/units.h"
#include "perls-common/bot_util.h"

#include "perls-math/gsl_util.h"

#include "renderers.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

// Pulled and modified from ford DGC-UMICH code

typedef struct _RendererCompass RendererCompass;
struct _RendererCompass {    
    BotRenderer renderer;
    BotCTrans   *ctrans;
};

static void
compass_free (BotRenderer *super)
{
    RendererCompass *self = (RendererCompass*) super->user;
    BotRenderer *renderer = &self->renderer;
    free (renderer->name);
    free (self);
}

GLint
mygluUnProject (double winx, double winy, double winz,
                const double model[16], const double proj[16], const int viewport[4],
                double *objx, double *objy, double *objz)
{
    GSLU_MATRIX_VIEW (p, 4, 4, {0});
    memcpy ((&p.matrix)->data, proj, 16 * sizeof (double));
    gsl_matrix_transpose (&p.matrix);

    GSLU_MATRIX_VIEW (m, 4, 4, {0});
    memcpy ((&m.matrix)->data, model, 16 * sizeof (double));
    gsl_matrix_transpose (&m.matrix);
    

    GSLU_MATRIX_VIEW (t, 4, 4, {0});
    gslu_blas_mm (&t.matrix, &p.matrix, &m.matrix);
    
    gslu_matrix_inv (&m.matrix, &t.matrix);    
    if (viewport[2] == 0 || viewport[3] == 0)
        return GL_FALSE;
    
    GSLU_VECTOR_VIEW (v, 4, {2 * (winx - viewport[0]) / viewport[2] - 1,
                             2 * (winy - viewport[1]) / viewport[3] - 1,
                             2 * winz - 1,
                             1});

    GSLU_VECTOR_VIEW (v2, 4, {0});
    
    gslu_blas_mv (&v2.vector, &m.matrix, &v.vector);
    
    *objx = gsl_vector_get (&v2.vector, 0) / gsl_vector_get (&v2.vector, 3);
    *objy = gsl_vector_get (&v2.vector, 1) / gsl_vector_get (&v2.vector, 3);
    *objz = gsl_vector_get (&v2.vector, 2) / gsl_vector_get (&v2.vector, 3);
    return GL_TRUE;
}

/**
 * Given a window position (in pixels) and a world z-coordinate,
 * compute the world x and y coordinates corresponding to that pixel.
 */
static int
window_pos_to_ground_pos (double *x, double *y,
                          double z, double winx, double winy)
{
    GLdouble model_matrix[16];
    GLdouble proj_matrix[16];
    GLint viewport[4];
    double x1, y1, z1, x2, y2, z2;

    glGetDoublev (GL_MODELVIEW_MATRIX, model_matrix);
    glGetDoublev (GL_PROJECTION_MATRIX, proj_matrix);
    glGetIntegerv (GL_VIEWPORT, viewport);

    if (mygluUnProject (winx, viewport[3]-winy, 0.0, model_matrix, proj_matrix,
                        viewport, &x1, &y1, &z1) == GL_FALSE)
        return -1;
    if (mygluUnProject (winx, viewport[3]-winy, 1.0, model_matrix, proj_matrix,
                        viewport, &x2, &y2, &z2) == GL_FALSE)
        return -1;

    if ((z1 > 0 && z2 > 0) || (z1 < 0 && z2 < 0))
        return -1;

    *x = x1 + (z-z1)/(z2-z1) * (x2-x1);
    *y = y1 + (z-z1)/(z2-z1) * (y2-y1);
    return 0;
}

/* Draws the letter N */
static void
draw_letter_N (void)
{
    glBegin (GL_QUADS);

    glVertex3f (-1.0, -1.0, 0);
    glVertex3f (-0.5, -1.0, 0);
    glVertex3f (-0.5, 1.0, 0);
    glVertex3f (-1.0, 1.0, 0);

    glVertex3f (1.0, -1.0, 0);
    glVertex3f (0.5, -1.0, 0);
    glVertex3f (0.5, 1.0, 0);
    glVertex3f (1.0, 1.0, 0);

    glVertex3f (-0.5, 1.0, 0);
    glVertex3f (-1.0, 1.0, 0);
    glVertex3f (0.5, -1.0, 0);
    glVertex3f (1.0, -1.0, 0);

    glEnd ();
}

static void
compass_draw (BotViewer *viewer, BotRenderer *renderer)
{
    
    //RendererCompass *self = (RendererCompass*) renderer;
    
    // default coords in (N-W-UP) rotate to our prefered (N-E-D)
    // for yaw just rotate about y
    glRotatef(-90.0,0.0,0.0,1.0);
        
    int viewport[4];
    int pos[2] = { -50, -50 };
    int radius = 35;
    double z = 0;
    double x = 0;
    double y = 0;
    double x2 = 0;
    double y2 = 0;
    double heading = 0,
    variance = 0;


    // setup -------------------------------------------------------------------
    
    // Find the world x,y coordinates for the compass, given we want it
    // near the side of the viewport.
    glGetIntegerv (GL_VIEWPORT, viewport);
    if (window_pos_to_ground_pos (&x, &y, z, viewport[2]+pos[0], viewport[3]+pos[1]) < 0)
        return;

    // Find the world x,y coordinates for the edge of the compass, so
    // we can compute how large to make it in world coordinates. 
    window_pos_to_ground_pos (&x2, &y2, z, viewport[2]+pos[0]+radius, viewport[3]+pos[1]);
    
    

    // Draw the compass at the right place, size, and orientation --------------

    glDisable (GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glPushMatrix ();
    glTranslatef (x, y, z);
    double scale = sqrt ((x2-x)*(x2-x) + (y2-y)*(y2-y));
    glScalef (scale, scale, scale);
    glRotatef (heading*RTOD + 90, 0, 0, 1);

    GLUquadricObj *q = gluNewQuadric ();

    /* Draw an error bar indicating one standard deviation of the
     * heading error.  If it's above the threshold, make it red. */

    glColor4f (0.8, 0.2, 0.2, 0.7);
    double std_deg = sqrt (variance) * RTOD;
    gluPartialDisk (q, 1.0, 1.1, 50, 1, 90 - std_deg, 2 * std_deg);
    
    /* Draw the circle for the compass */
    glColor4f (0.0, 0.0, 0.0, 0.6);
    gluDisk (q, 0, 1.0, 20, 1);
    gluDeleteQuadric (q);

    /* Draw the arrow on the compass */
    glLineWidth (2.0);
    glColor4f (1.0, 1.0, 1.0, 0.8);
    glBegin (GL_LINES);
    glVertex3f (-0.5, 0, 0);
    glVertex3f ( 0.5, 0, 0);

    glVertex3f (0, 0.5, 0);
    glVertex3f (0, -0.5, 0);

    glVertex3f (0.25, 0.25, 0);
    glVertex3f (0.5, 0, 0);

    glVertex3f (0.25, -0.25, 0);
    glVertex3f (0.5, 0, 0);
    glEnd ();

    /* Draw tick marks around the compass (15 degree increments) */
    glLineWidth (1.0);
    int i;
    for (i = 30; i <= 330; i += 15) {
        glPushMatrix ();
        glRotatef (i, 0, 0, 1);
        glBegin (GL_LINES);
        glVertex3f (0.95, 0, 0.0);
        glVertex3f (((i % 45) == 0) ? 0.75 : 0.85, 0, 0.0);
        glEnd ();
        glPopMatrix ();
    }

    // Draw the letter N 
    glColor4f (1.0, 0.0, 0.0, 1.0);
    glTranslatef (0.75, 0, 0);
    glScalef (0.2, 0.2, 0.2);
    glRotatef (90, 0, 0, 1);
    draw_letter_N();

    glDisable (GL_BLEND);
    glPopMatrix ();
}


void
setup_renderer_compass (BotViewer *viewer, int priority)
{
    RendererCompass *self = new RendererCompass ();
    BotRenderer *renderer = &self->renderer;

    renderer->name = strdup ("Compass");
    renderer->enabled = 1;

    renderer->draw = compass_draw;
    renderer->destroy = compass_free;
    renderer->widget = NULL;

    self->ctrans = bot_ctrans_new ();
    bot_viewer_add_renderer (viewer, renderer, priority);
}
