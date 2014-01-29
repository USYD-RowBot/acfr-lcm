#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include <sys/param.h>

// external linking req'd
#include <glib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/error.h"
#include "perls-common/glib_util.h"
#include "perls-common/lcm_util.h"

#include "perls-math/gsl_util.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_van_plink_t.h"
#include "perls-lcmtypes/perllcm_van_rdi_bathy_collection_t.h"
#include "perls-lcmtypes/perllcm_van_vlink_t.h"

#include "van_util.h"

void
vanu_isam_save_rtvan_appended (const char *filepath, GHashTable *camoffset_list, 
                               GHashTable *x_vc, GHashTable *S_ii)
{
    printf ("-- Saving to %s/graph.isam ...\n", filepath);

    char isam_tmp[MAXPATHLEN], rtvan_tmp[MAXPATHLEN];
    snprintf (isam_tmp, strlen (filepath)+12, "%s/graph.isam\n", filepath);
    snprintf (rtvan_tmp, strlen (filepath)+13, "%s/graph.rtvan\n", filepath);
    FILE *file_isam = fopen (isam_tmp, "r");
    FILE *file_rtvan = fopen (rtvan_tmp, "w");

    if (file_isam != NULL && file_rtvan != NULL) {
        char line [MAXPATHLEN];
        while ( fgets (line, sizeof line, file_isam) != NULL )  { // read a line
            // copy the existing line to new graph.isam file
            int len = strlen (line);
            if (line[len-1] != '\n') { 
                printf ("increase buffer size reading isam graph\n");
                return; 
            }
            line[len-1] = '\0';
            fprintf (file_rtvan, "%s", line);

            // parse it to add additional information
            size_t token_counter = 0;
            bool line_node = false;
            int64_t utime;
            char *pch = strtok (line," ,;(){}");
            while (pch != NULL) {
                if ((token_counter == 0) && (strcmp (line, "Pose3d_Node" ) == 0))
                    line_node = true;

                if (line_node && (token_counter == 1) && (atoi (pch) == 0))
                    line_node = false;
                
                if (line_node && (token_counter == (2+6))) {
                    // yes. this is a line for node. if token_counter is not 8, this could be other line starting with Pose3d_Node*
                    utime = atol (pch);

                    // write offset and sensor xform if available
                    double *offset = g_hash_table_lookup (camoffset_list, &utime);
                    perllcm_pose3d_t *x_vs = g_hash_table_lookup (x_vc, &utime);

                    if (offset && x_vs) {
                        // we don't have these value for nodes related to other sensors
                        fprintf (file_rtvan, " %g (%g, %g, %g, %g, %g, %g)", 
                                               *offset, x_vs->mu[0], x_vs->mu[1], x_vs->mu[2], 
                                                        x_vs->mu[3], x_vs->mu[4], x_vs->mu[5]);
                    }

                    // write covaraince for each node
                    gsl_matrix *S_vii = g_hash_table_lookup (S_ii, &utime);
                    if (S_vii) {
                        fprintf (file_rtvan, " {");
                        for (size_t i=0; i<6; i++) {
                            for (size_t j=0; j<6; j++) {
                                fprintf (file_rtvan, "%g",gsl_matrix_get (S_vii, i, j));
                                if (i!=5 || j!=5) fprintf (file_rtvan, ", ");
                            }
                        }
                        fprintf (file_rtvan, "}");
                    }

                }
                pch = strtok (NULL," ,;(){}");
                token_counter++;
            }

            fprintf (file_rtvan, "\n");
        } // reading line by line
    }

    // clean up
    fclose (file_isam);
    fclose (file_rtvan);
    //remove (isam_tmp);
    //rename (rtvan_tmp, isam_tmp);

    printf ("-- isam result has been saved to graph.isam\n");
}

void
vanu_isam_save_rtvan_appended_via_lcm (FILE *fid, se_save_isam_t *isam_line, 
                                       GHashTable *camoffset_list, GHashTable *x_vc, GHashTable *S_ii)
{
    if (!fid) {
        printf ("ERROR: graph.isam was not open properly.\n");
        return;
    }

    // file open and ready to write
    char line [MAXPATHLEN];
    strcpy (line, isam_line->line);

    // copy the original line received
    fprintf (fid, "%s", line);

    // add additional information to camera nodes
    if (isam_line->type == SE_SAVE_ISAM_T_TYPE_NODE) {
        int64_t utime = isam_line->id;
        //printf ("utime = %"PRId64"\n", utime);

        double *offset = g_hash_table_lookup (camoffset_list, &utime);
        perllcm_pose3d_t *x_vs = g_hash_table_lookup (x_vc, &utime);

        if (offset && x_vs) {
            // we don't have these value for nodes related to other sensors
            fprintf (fid, " %g (%g, %g, %g, %g, %g, %g)", 
                           *offset, x_vs->mu[0], x_vs->mu[1], x_vs->mu[2], 
                                    x_vs->mu[3], x_vs->mu[4], x_vs->mu[5]);
        }

        // write covaraince for each node
        gsl_matrix *S_vii = g_hash_table_lookup (S_ii, &utime);
        if (S_vii) {
            fprintf (fid, " {");
            for (size_t i=0; i<6; i++) {
                for (size_t j=0; j<6; j++) {
                    fprintf (fid, "%g",gsl_matrix_get (S_vii, i, j));
                    if (i!=5 || j!=5) fprintf (fid, ", ");
                }
            }
            fprintf (fid, "}");
        }
    }
    fprintf (fid, "\n");
    //printf ("isam : %s (type:%d,sensor:%d)\n", isam_line->line, isam_line->type, isam_line->sensor);    
}


GSList *
vanu_load_isam_graph (const char *filename, GSList *utimelist, 
                      GHashTable *camoffset_list, GHashTable *x_vc_hash, GHashTable *S_ii_hash,
                      size_t *n_nodes)
{
    FILE *file = fopen (filename, "r" );
    size_t dof = 6; 

    if (file != NULL) {
        char line [1024];
        while ( fgets (line, sizeof line, file) != NULL ) { // read a line
            //printf ("reading line: %s\n", line);
            size_t token_counter = 0;
            bool line_node = false;
            bool camera_node = false;
            int64_t utime = 0;

            perllcm_pose3d_t instant_x_vc = {};
            double camoffset = 0;
            double S_ii_tmp_array[36];
            gsl_matrix_view instant_S_ii = gsl_matrix_view_array (S_ii_tmp_array, 6, 6);
            gsl_matrix_set_zero (&instant_S_ii.matrix);
            size_t idx_x_vs = 0;
            size_t idx_S_ii = 0;
            char *pch = strtok (line," ,;(){}");
            while (pch != NULL) {
                if (token_counter == 0 && (strcmp(line, "Pose3d_Node" ) == 0))
                    line_node = true;

                if (line_node && (token_counter == 1) && (atoi (pch) == 0))
                    line_node = false;
                
                if (line_node && (token_counter == (2+dof))) 
                    utime = atol (pch);

                if (line_node && (token_counter == (2+dof+1))) {
                    camoffset = atof (pch);
                    camera_node = true;
                }

                if (line_node && ((2+dof+2) <= token_counter) && (token_counter < (2+dof+2+dof))) {
                    instant_x_vc.mu[idx_x_vs] = atof (pch);
                    idx_x_vs ++;
                }

                if (line_node && ( (2+dof+2+dof) <= token_counter && token_counter < (2+dof+2+dof+dof*dof))) {
                    S_ii_tmp_array[idx_S_ii] = atof(pch);
                    idx_S_ii ++;
                }

                pch = strtok (NULL," ,;(){}");
                token_counter++;
            }

            if (line_node && camera_node) {
                // verify if we read everything needed
                if (utime == 0) printf ("ERROR: no valid utime has been loaded\n");

                // store utime
                utimelist = g_slist_append (utimelist, gu_dup (&utime, sizeof utime));

                // store sensor transformation
                instant_x_vc.utime = utime;
                perllcm_pose3d_t *x_vc = calloc (1, sizeof(*x_vc));
                memcpy (x_vc, &instant_x_vc, sizeof(*x_vc));
                g_hash_table_insert (x_vc_hash, gu_dup (&utime, sizeof utime), x_vc);
                
                // store camera offset
                double *offset = g_malloc (sizeof(*offset));
                *offset = camoffset;
                g_hash_table_insert (camoffset_list, gu_dup (&utime, sizeof utime), offset);


                // store Sii
                //gslu_matrix_printf (&instant_S_ii.matrix, "Sii");
                gsl_matrix *S_ii = gsl_matrix_alloc (6,6);
                gsl_matrix_memcpy (S_ii, &instant_S_ii.matrix);
                g_hash_table_insert (S_ii_hash, gu_dup (&utime, sizeof utime), S_ii);

                (*n_nodes)++;
            } 
        } // read line by line

        fclose (file);
        return utimelist;
    }
    else {
        printf ("ERROR loading isam graph\n");
        return NULL;
    }
}

perllcm_van_corrset_t*
vanu_prepare_corrset (int64_t utime1, int64_t utime2, gsl_vector *z, gsl_matrix *R, gsl_matrix *uv1, gsl_matrix *uv2)
{
    assert (uv1->size1 == 2 && uv2->size1 == 2 && uv1->size2 == uv2->size2);
    
    perllcm_van_corrset_t *corr = calloc (1, sizeof (*corr));

    // assign timestamps
    corr->utime_i = utime1;
    corr->utime_j = utime2;

    // assign measurements
    gsl_vector_view z_view = gsl_vector_view_array (corr->z, 5);
    gsl_vector_memcpy (&z_view.vector, z);
    gsl_matrix_view R_view = gsl_matrix_view_array (corr->R, 5, 5);
    gsl_matrix_memcpy (&R_view.matrix, R);

    size_t npts = uv1->size2;
    corr->npts = npts;

    // assign inliers
    corr->u1 = malloc (sizeof (corr->u1) * npts); corr->v1 = malloc (sizeof (corr->v1) * npts);
    corr->u2 = malloc (sizeof (corr->u2) * npts); corr->v2 = malloc (sizeof (corr->v2) * npts);

    gsl_vector_float_view u1_view = gsl_vector_float_view_array (corr->u1, npts);
    gsl_vector_float_view v1_view = gsl_vector_float_view_array (corr->v1, npts);
    gsl_vector_float_view u2_view = gsl_vector_float_view_array (corr->u2, npts);
    gsl_vector_float_view v2_view = gsl_vector_float_view_array (corr->v2, npts);
    gsl_vector_view u1_vec = gsl_matrix_row (uv1, 0);
    gsl_vector_view v1_vec = gsl_matrix_row (uv1, 1);
    gsl_vector_view u2_vec = gsl_matrix_row (uv2, 0);
    gsl_vector_view v2_vec = gsl_matrix_row (uv2, 1);
    GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector, &u1_vec.vector, gsl_vector_float, &u1_view.vector);
    GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector, &v1_vec.vector, gsl_vector_float, &v1_view.vector);
    GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector, &u2_vec.vector, gsl_vector_float, &u2_view.vector);
    GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector, &v2_vec.vector, gsl_vector_float, &v2_view.vector);

    return corr;
} 

void
vanu_dvlpts_save2disk (const char *filepath, GSList *dvlpts_glist, 
                       perllcm_van_calib_t *calibUw_water, perllcm_van_calib_t *calibUw_air,
                       perllcm_van_calib_t *calibPeri_water, perllcm_van_calib_t *calibPeri_air)
{
    char bathy_path[PATH_MAX];
    snprintf (bathy_path, sizeof bathy_path, "%s/bathy.dat", filepath);
    printf ("%s\n", bathy_path);

    // prepare dvl_bathy
    perllcm_van_rdi_bathy_collection_t *bathy_list = calloc (1, sizeof (*bathy_list));
    //bathy_list->id = 
    size_t listlen = g_slist_length (dvlpts_glist);
    bathy_list->npts = listlen;
    bathy_list->bathy_v = malloc (sizeof (*bathy_list->bathy_v) * listlen);
    bathy_list->x_vc = malloc (sizeof (*bathy_list->x_vc) * listlen);
    bathy_list->calib_list = malloc (sizeof (*bathy_list->calib) * listlen);

    for (size_t i=0; i<listlen; i++) {
        perllcm_van_rdi_bathy_collection_t *bathy_i = g_slist_nth_data (dvlpts_glist, i);
        bathy_list->bathy_v[i] = bathy_i->bathy_v[0];
        bathy_list->x_vc[i] = bathy_list->x_vc[0];        
        bathy_list->calib_list[i] = bathy_list->calib_list[0];
    }

    // write calibration information
    bathy_list->ncalib = 4;
    bathy_list->calib = malloc (sizeof (*bathy_list->calib) * bathy_list->ncalib);
    bathy_list->calib[CALIB_UW_WATER] = *calibUw_water;
    bathy_list->calib[CALIB_UW_AIR] = *calibUw_air;
    bathy_list->calib[CALIB_PERI_WATER] = *calibPeri_water;
    bathy_list->calib[CALIB_PERI_AIR] = *calibPeri_air;

    // fwrite
    LCMU_FWRITE (bathy_path, bathy_list, perllcm_van_rdi_bathy_collection_t);
    printf ("-- bathy_list has been saved to %s\n", bathy_path);

    // clean up
    perllcm_van_rdi_bathy_collection_t_destroy (bathy_list);
}

void
vanu_corrset_save2disk (int64_t utime1, int64_t utime2, const char *logdir,
                          gsl_vector *z, gsl_matrix *R, gsl_matrix *uv1, gsl_matrix *uv2)
{
    // write z, R, uv1, uv2 into disk
    char corrsetdir[PATH_MAX];
    snprintf (corrsetdir, sizeof corrsetdir, "%s/corrset/%"PRId64"-%"PRId64".corr", logdir, utime1, utime2);
    printf ("%s\n", corrsetdir);

    perllcm_van_corrset_t *corr = vanu_prepare_corrset (utime1, utime2, z, R, uv1, uv2);
    LCMU_FWRITE (corrsetdir, corr, perllcm_van_corrset_t);
    perllcm_van_corrset_t_destroy (corr);
}

int
vanu_load_manual_plink (const char *filename, GHashTable *plinks)
{
    int nplink = 0;

    FILE *fid = fopen (filename, "r");
    if (!fid) {
        ERROR ("unable to read file %s!", filename);
        return nplink;
    }

    char line[128];

    if (fgets (line, sizeof line, fid) != NULL)
        sscanf (line, "%d", &nplink);

    if (nplink == 0)
        return nplink;
    
    // populate GList and return
    double mu[6] = {0.1, 0., 0., 0., 0., 0.};
    double s = 0.1;
    double Sigma[36] = {s, 0., 0., 0., 0., 0.,
                        0., s, 0., 0., 0., 0.,
                        0., 0., s,0., 0., 0.,
                        0., 0., 0., s, 0., 0.,
                        0., 0., 0., 0., s, 0.,
                        0., 0., 0., 0., 0., s};

    perllcm_pose3d_t x_cjci = {0};
    memcpy (x_cjci.mu, mu, sizeof (double) * 6);
    memcpy (x_cjci.Sigma, Sigma, sizeof (double) *36);

    while (fgets (line, sizeof line, fid) != NULL) {
        int64_t utime1 = 0, utime2 = 0;
        sscanf (line, "%ld %ld", &utime1, &utime2);

        //printf ("t1 = %"PRId64", t2 = %"PRId64"\n", utime1, utime2);

        // generate perllcm_van_link_t
        x_cjci.utime = utime2;
        perllcm_van_plink_t *plink = malloc (sizeof (*plink));
        plink->utime_i = utime1;
        plink->utime_j = utime2;
        plink->prior   = 1;
        plink->x_ji    = x_cjci;
        plink->link_id = 0;         // need to assign this just before proposing
        g_hash_table_insert (plinks, gu_dup (&utime2, sizeof utime2), plink);
    }

    printf ("%d manual plink have found and loaded.\n",nplink);

    // clean up
    fclose (fid);

    return nplink;
}

int
vanu_load_manual_plink_wo_prior (const char *filename, GHashTable *plinks)
{
    int nplink = 0;

    FILE *fid = fopen (filename, "r");
    if (!fid) {
        ERROR ("unable to read file %s!", filename);
        return nplink;
    }

    char line[128];

    if (fgets (line, sizeof line, fid) != NULL)
        sscanf (line, "%d", &nplink);

    if (nplink == 0)
        return nplink;
    
    while (fgets (line, sizeof line, fid) != NULL) {
        int64_t utime1 = 0, utime2 = 0;
        sscanf (line, "%ld %ld", &utime1, &utime2);

        //printf ("t1 = %"PRId64", t2 = %"PRId64"\n", utime1, utime2);

        // generate perllcm_van_link_t
        perllcm_van_plink_t *plink = malloc (sizeof (*plink));
        plink->utime_i = utime1;
        plink->utime_j = utime2;
        plink->prior   = 0;
        plink->link_id = 0;         // need to assign this just before proposing
        g_hash_table_insert (plinks, gu_dup (&utime2, sizeof utime2), plink);
    }

    printf ("%d manual plink have found and loaded.\n",nplink);

    // clean up
    fclose (fid);

    return nplink;
}

int
vanu_saliency_save2disk (const char *filename, GSList *utimelist, GHashTable *S_L_table, GHashTable *sumidf_list, GHashTable *vocablen_list, double sumidf_max)
{

    FILE *stream = fopen (filename, "w");
    if (!stream) {
        ERROR ("unable to create file %s!", filename);
        return EXIT_FAILURE;
    }

    size_t len = g_slist_length (utimelist);

    for (size_t i=0; i<len; i++) {
        GSList *event = g_slist_nth (utimelist, i);
        int64_t *utime = event->data;

        double *S_L = g_hash_table_lookup (S_L_table, utime);
        double local_saliency = 0.0;
        if (S_L) {
            local_saliency = *S_L;
        }

        double *sumidf = g_hash_table_lookup (sumidf_list, utime);
        double global_saliency = 0.0;
        if (sumidf) {
            global_saliency = (*sumidf)/sumidf_max;
        }

        size_t vocablen = 0.0;
        size_t *vocablen_tmp = g_hash_table_lookup (vocablen_list, utime);
        if (vocablen_tmp)
            vocablen = (*vocablen_tmp);

        fprintf (stream, "%"PRId64"\t%g\t%g\t%zu\n", (*utime), local_saliency, global_saliency, vocablen);
    }

    fclose (stream);

    return EXIT_SUCCESS;
}


int
vanu_link_info_save2disk (const char *logdir, GSList *linklist)
{
    char plinkfile[PATH_MAX];
    snprintf (plinkfile, sizeof plinkfile, "%s/links.dat", logdir);
    //printf ("%s\n", plinkfile);

    FILE *stream = fopen (plinkfile, "w");
    if (!stream) {
        ERROR ("unable to create file %s!", plinkfile);
        return EXIT_FAILURE;
    }

    size_t linklen = g_slist_length (linklist);
    for (size_t i=0; i<linklen; i++) {
        perllcm_van_vlink_t *link = g_slist_nth_data (linklist, i);
        bool success = false;
        if (link->type > 0) success = true;
        fprintf (stream, "%"PRId64"\t%"PRId64"\t%g\t%g\t%d\n", 
                 link->utime_i, link->utime_j, link->S_L, link->Ig, success);
    }
    fclose (stream);

    return EXIT_SUCCESS;
}

