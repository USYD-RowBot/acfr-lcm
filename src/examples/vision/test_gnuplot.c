#include <stdio.h>
#include <stdlib.h>
#include "perls-math/math.h"
#include "perls-vision/vision.h"

int main(int argc, char *argv[])
{

    double scale = 1;

    gsl_matrix *X1 = gsl_matrix_alloc (3, 200);
    FILE * fX1 = fopen ("../share/examples/_test_corr_files/X.txt", "rb");
    gsl_matrix_fscanf (fX1, X1);
    fclose(fX1);

    double angle = 0.0;
    for (int i=0; i<3; i++)
    {
        FILE *gp = popen("gnuplot","w"); // 'gp' is the pipe descriptor
        fprintf (gp, "set size ratio -1\n");
        fprintf (gp, "set view equal\n");
        if (gp==NULL)
            printf("Error opening pipe to GNU plot. sudo apt-get install gnuplot \n");

        GSLU_VECTOR_VIEW (test, 5, {0.0,0.0,angle,0.0,0.0});  // 5 dof tes
        vis_plot_3dpts (gp, X1);
        fprintf (gp, "pause 1\n");

        vis_plot_relpose (gp, &test.vector, scale, NULL);
        fprintf (gp, "pause 1\n");

        vis_plot_relpose_3dpts (gp, &test.vector, X1, scale, NULL);
        fprintf (gp, "pause 1\n");
        angle=angle+0.1;

        fclose (gp);

    }

    // clean up
    gslu_matrix_free (X1);

    return 0;
}

