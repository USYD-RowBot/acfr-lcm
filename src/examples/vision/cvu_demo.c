#include "perls-math/gsl_util.h"
#include "perls-vision/opencv_util.h"

int
main (int argc, char *argv[])
{
    // OpenCV test matrix
    CvMat *cmat = cvCreateMat (4, 4, CV_64FC1);
    cvmSet (cmat, 0, 0, 1);
    cvmSet (cmat, 0, 1, 2);
    cvmSet (cmat, 0, 2, 3);
    cvmSet (cmat, 0, 3, 4);
    cvmSet (cmat, 1, 0, 5);
    cvmSet (cmat, 1, 1, 6);
    cvmSet (cmat, 1, 2, 7);
    cvmSet (cmat, 1, 3, 8);
    cvmSet (cmat, 2, 0, 9);
    cvmSet (cmat, 2, 1, 10);
    cvmSet (cmat, 2, 2, 11);
    cvmSet (cmat, 2, 3, 12);
    cvmSet (cmat, 3, 0, 13);
    cvmSet (cmat, 3, 1, 14);
    cvmSet (cmat, 3, 2, 15);
    cvmSet (cmat, 3, 3, 16);

    vis_cvu_matrix_printf (cmat, "cmat");
    vis_cvu_matrix_printfc (cmat, "cmat", NULL, CblasTrans);

    // convert to gsl_matrix (deep copy)
    gsl_matrix *gmat_copy = vis_cvu_cvmat_to_gsl_matrix_copy (cmat);
    gslu_matrix_printf (gmat_copy, "gmat_copy");
    gslu_matrix_printfc (gmat_copy, "gmat_copy", NULL, CblasTrans);

    // convert to gsl_matrix_view (light copy)
    gsl_matrix_view gmat_view = vis_cvu_cvmat_to_gsl_matrix_view (cmat);
    gslu_matrix_printf (&gmat_view.matrix, "gmat_view");

    // convert back to CvMat (deep copy)
    CvMat *cmat_copy = vis_cvu_gsl_matrix_to_cvmat_copy (&gmat_view.matrix);
    vis_cvu_matrix_printf (cmat_copy, "cmat_copy");

    // convert back to CvMat (light copy)
    CvMat cmat_view = vis_cvu_gsl_matrix_to_cvmat_view (gmat_copy);
    vis_cvu_matrix_printf (&cmat_view, "cmat_view");

    // GSL test vector
    gsl_vector *gvec = gsl_vector_alloc (3);
    for (size_t i=0; i<gvec->size; i++)
        gsl_vector_set (gvec, i, i);
    gslu_vector_printf (gvec, "gvec");

    // connvert to CvMat (deep copy)
    CvMat *cvec_copy = vis_cvu_gsl_vector_to_cvmat_copy (gvec);
    vis_cvu_vector_printf (cvec_copy, "cvec_copy");

    // convert to CvMat (light copy)
    CvMat cvec_view = vis_cvu_gsl_vector_to_cvmat_view (gvec);
    vis_cvu_vector_printf (&cvec_view, "cvec_view");

    // convert back to gsl_vector (light copy)
    gsl_vector_view gvec_view = vis_cvu_cvmat_to_gsl_vector_view (cvec_copy);
    gslu_vector_printf (&gvec_view.vector, "gvec_view");

    // free alloc'ed memory
    gsl_matrix_free (gmat_copy);
    gsl_vector_free (gvec);
    cvReleaseMat (&cmat);
    cvReleaseMat (&cmat_copy);
    cvReleaseMat (&cvec_copy);
}
