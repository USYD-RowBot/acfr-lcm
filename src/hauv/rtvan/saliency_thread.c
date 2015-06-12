#include <stdio.h>
#include <stdlib.h>
#include <glib.h>

#include <gsl/gsl_math.h>

#include "perls-lcmtypes/hauv_bs_cnv_t.h"
#include "perls-lcmtypes/perllcm_van_feature_t.h"
#include "perls-lcmtypes/perllcm_van_saliency_t.h"
#include "perls-lcmtypes/perllcm_isam_cmd_t.h"

// perls lib
#include "perls-vision/vision.h"
#include "perls-common/common.h"
#include "perls-common/bot_util.h"
#include "perls-math/gsl_util.h"

// rtvan
#include "shared_memory.h"
#include "van_util.h"
#include "saliency_thread.h"

#define printf(format, ...)                                 \
    printf ("%-12s " format, "[saliency]", ## __VA_ARGS__)

#define SALIENCY_FILE "./saliency.dat"

#define EXTEND_LENGTH           5000
#define INVIDX_LENGTH           5000
#define UPDATE_ALL_SG_STEP      100

typedef struct invidx invidx_t;
struct invidx
{
    GSList    **index;          // list of imgnums (GsList)
    size_t      len;
    int         max_idx;      // initialized with -1
};

typedef struct thread_data thread_data_t;
struct thread_data
{
    GSList *sal_utimelist;   // this happens for all images

    GSList *vocab;           // GList of gsl_vector_float
    size_t vocab_len;

    double sal_new_vocab_sim_thresh;
    double sal_new_vocab_motion_thresh;
    double S_L_thresh;      // if S_L > S_L_thresh, it is locally salient
    double S_G_thresh;      // if S_G > S_G_thresh, it is globally salient

    // image bow generation statistics (assign word idx to feature i)
    gsl_vector_float *fi;       // ith feature
    gsl_vector_float *wi;       // ith word
    double feat_dist_min;       // current min dist
    size_t corr_word_idx;       // corr. word idx

    // to apply motion thresh
    double xyz_lv_curr[3];      // vehicle w.r.t. local-level (dynamic)
    GSList *xyz_lv_prev;        // list of double[3]
    double min_dxyz2;           // dx^2+ dy^2 + dz^2

    // self testing mode
    bool self_testing;

    // save image bow in file?
    bool save_image_bow;
    bool save_saliency;

    //GHashTable *S_L;           // save only when in self-testing mode
    gsl_vector *bowE;            // keep bowE    not S_L
    gsl_vector *sumidf;          // keep sumidf  not S_G
    gslu_index *vocablen_list;

    // incrementally increase the size of container by EXTEND_LENGTH
    gslu_index *nwoccur;         // nw, number of word occurance (idf)
    int         Nf;              // number of docs (images used in idf calc)
    int         imgcounter;
    double      sumidf_max;
    GSList     *image_bow_mask;  // list of boolean vector indicating wi occured in the image
    invidx_t   *invidx;

    size_t      _idx_foreach;

    // tmp track of stat
    //FILE *f_dt;
    //FILE *f_vocablen;
};

static gslu_index *
_extend_gslu_index (gslu_index *original)
{
    size_t curlen = original->size;
    gslu_index *extended = gslu_index_alloc (curlen+EXTEND_LENGTH);
    gslu_index_set_zero (extended);
    gslu_index_view extended_sub= gslu_index_subvector (extended, 0, curlen);
    gslu_index_memcpy (&extended_sub.vector, original);
    gslu_index_free (original);

    return extended;
}

static gsl_vector *
_extend_gsl_vector (gsl_vector *original)
{
    size_t curlen = original->size;
    gsl_vector *extended = gsl_vector_alloc (curlen+EXTEND_LENGTH);
    gsl_vector_set_zero (extended);
    gsl_vector_view extended_sub= gsl_vector_subvector (extended, 0, curlen);
    gsl_vector_memcpy (&extended_sub.vector, original);
    gslu_vector_free (original);

    return extended;

}

static double
_calc_sum_idf (gslu_boolean *image_bow_mask, gslu_index *nw, int Nf)
{
    // calculate sum of idf
    double sumidf = 0.0;    // sum of idf
    for (size_t wi=0; wi<image_bow_mask->size; wi++)
    {
        if (gslu_boolean_get (image_bow_mask, wi) > 0)   // wi occured
        {
            double idf = log2 (Nf/gslu_index_get (nw, wi));
            //printf ("for nz %zu nw = %zu Nf = %d idf = %f\n", wi, gslu_index_get (nw, wi), Nf, idf);
            sumidf = sumidf + idf;
        }
    }

    return sumidf;
}

static void
glist_destroyer (gpointer data, gpointer user)
{
    free (data);
}

static void
glist_gslu_boolean_destroyer (gpointer data, gpointer user)
{
    if (data) gslu_boolean_free (data);
}

static void
glist_calc_dist_vocab (gpointer data, gpointer user)
{
    gsl_vector_float *wi = (gsl_vector_float *) data;
    thread_data_t *tdata = (thread_data_t *) user;

    double feat_dist = vis_feature_get_simscore_gsl_float (tdata->fi, wi);

    if (feat_dist < tdata->feat_dist_min)
    {
        // update corresponding word
        tdata->feat_dist_min = feat_dist;
        tdata->wi = wi;
        //tdata->corr_word_idx = word_idx;
    }
    // printf ("min dist to %zu word = %g\n", corr_word, feat_dist_min);
}

static void
glist_calc_dist_xji (gpointer data, gpointer user)
{
    double *xyz_lv_prev = (double *) data;
    thread_data_t *tdata = (thread_data_t *) user;

    double dxyz2 = 0.0;             // dx^2+ dy^2 + dz^2
    for (size_t i=0; i<3; i++)
        dxyz2 += (tdata->xyz_lv_curr[i] - xyz_lv_prev[i]) * (tdata->xyz_lv_curr[i] - xyz_lv_prev[i]);

    if (tdata->min_dxyz2 > dxyz2)
        tdata->min_dxyz2 = dxyz2;

}

static void
g_list_update_sumidf (gpointer data, gpointer user)
{
    gslu_boolean *image_bow_mask_i = (gslu_boolean *) data;
    thread_data_t *tdata = (thread_data_t *) user;

    double sumidf_i = _calc_sum_idf (image_bow_mask_i, tdata->nwoccur, tdata->Nf);
    gsl_vector_set (tdata->sumidf, tdata->_idx_foreach, sumidf_i);

    // update sumidf max too!
    if (sumidf_i > tdata->sumidf_max)
        tdata->sumidf_max = sumidf_i;

    tdata->_idx_foreach ++;
}

static invidx_t*
init_invidx ()
{
    invidx_t *invidx = calloc (1, sizeof (*invidx));
    invidx->index = calloc (1, sizeof (GSList*)*INVIDX_LENGTH);
    invidx->len = INVIDX_LENGTH;
    invidx->max_idx = -1;

    return invidx;
}

static void
free_invidx (invidx_t *invidx)
{
    if (invidx)
    {
        free (invidx->index);
        free (invidx);
    }
}

static void
_update_sumidf_all (thread_data_t *tdata)
{
    tdata->_idx_foreach = 0;
    tdata->sumidf_max = 0.0;
    g_slist_foreach (tdata->image_bow_mask, &g_list_update_sumidf, tdata);
}

static void
_update_sumidf_invidx (thread_data_t *tdata)
{
    // @TODO: implement unique(imgnum)
    gslu_boolean *imgbow_cur = g_slist_nth_data (tdata->image_bow_mask, tdata->vocab_len-1);

    for (size_t wi=0; wi<tdata->vocab_len; wi++)
    {
        if (gslu_boolean_get (imgbow_cur, wi) > 0)   // occured in the current image
        {
            GSList *imglist = tdata->invidx->index[wi]; // imglist when wi occured

            for (size_t imgidx=0; imgidx < g_slist_length (imglist); imgidx++)
            {
                int *imgnum = g_slist_nth_data (imglist, imgidx);
                if (imgnum)
                {
                    gslu_boolean *image_bow_mask_i = g_slist_nth_data (tdata->image_bow_mask, (*imgnum));
                    double sumidf_i = _calc_sum_idf (image_bow_mask_i, tdata->nwoccur, tdata->Nf);

                    gsl_vector_set (tdata->sumidf, (*imgnum), sumidf_i);

                }
                else
                    printf ("ERROR: there is a missing imgnum in invidx imglist\n");
            }
        }
    }
}

static void
perllcm_van_feature_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_van_feature_t *f, void *user)
{
    thread_data_t *tdata = user;

    printf ("got surf (%d) for image %"PRId64"\n", f->npts, f->utime);

    // use only surf for saliency
    if (f->attrtype != PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF &&
            f->attrtype != PERLLCM_VAN_FEATURE_T_ATTRTYPE_SURFGPU)
        return;

    // separate utimelist (shm = camera node utime)
    // ------------------------------------------------------------------
    tdata->sal_utimelist = g_slist_append (tdata->sal_utimelist, gu_dup (&f->utime, sizeof f->utime));
    int imgnum_cur = tdata->imgcounter;
    tdata->imgcounter ++;
    if (tdata->imgcounter > tdata->sumidf->size)
    {
        tdata->sumidf = _extend_gsl_vector (tdata->sumidf);
        tdata->bowE = _extend_gsl_vector (tdata->bowE);
        tdata->vocablen_list = _extend_gslu_index (tdata->vocablen_list);
    }

    // this is a valid node OR self testing mode: compute saliency!
    if (f->npts == 0)  // not salient
    {
        // publish the local saliency score
        perllcm_van_saliency_t *saliency = calloc (1, sizeof (*saliency));
        saliency->utime = f->utime;
        saliency->feattype = f->attrtype;
        saliency->npts = f->npts;
        saliency->vocab_len = tdata->vocab_len;
        perllcm_van_saliency_t_publish (shm->lcm, VAN_SALIENCY_CHANNEL, saliency);
        perllcm_van_saliency_t_destroy (saliency);

        // save 0.0 to tdata then return
        gsl_vector_set (tdata->bowE, imgnum_cur, 0.0);
        gsl_vector_set (tdata->sumidf, imgnum_cur, 0.0);
        gslu_index_set (tdata->vocablen_list, imgnum_cur, tdata->vocab_len);
        return;
    }

    // motion
    // -----------------------------------------------------------------
    bool update_vocab = true;
    tdata->min_dxyz2 = GSL_POSINF;

    if (tdata->xyz_lv_prev)   // when null (=first time) just update
    {
        g_slist_foreach (tdata->xyz_lv_prev, &glist_calc_dist_xji, tdata);
        if (tdata->min_dxyz2 < tdata->sal_new_vocab_motion_thresh * tdata->sal_new_vocab_motion_thresh && tdata->vocab_len > 0)
            update_vocab = false;
    }

    if (update_vocab)   // valid node to check spatial distance
    {
        double *xyz_dup = calloc (1, sizeof(*xyz_dup));
        for (size_t i=0; i<3; i++) xyz_dup[i] = tdata->xyz_lv_curr[i];
        tdata->xyz_lv_prev = g_slist_append (tdata->xyz_lv_prev, xyz_dup);
    }

    // image represented in bow
    // -----------------------------------------------------------------
    gslu_index *image_bow = gslu_index_alloc (f->npts);
    gslu_index_set_zero (image_bow);

    // maximum mask length = current size of vocab + case when every feat is assigned to different word
    gslu_boolean *occurrence_mask = gslu_boolean_alloc (tdata->vocab_len+f->npts);
    gslu_boolean_set_zero (occurrence_mask);
    if (update_vocab) tdata->Nf ++;

    size_t nfeat = f->npts;
    int64_t t0, t1, dt;

    // we might need scale information
    perllcm_van_feature_attr_cvsurf_t *attr = malloc (sizeof (*attr));
    if (perllcm_van_feature_attr_cvsurf_t_decode (f->attr, 0, f->attrsize, attr) < 0)
    {
        ERROR ("perllcm_van_feature_attr_cvsurf_t_decode() failed");
        free (attr);
        return;
    }

    t0 = timestamp_now ();
    for (size_t feat_idx=0; feat_idx<nfeat; feat_idx++)
    {

        //printf ("size = %d\n", (attr->size[feat_idx]));

        // re-set tdata statistics
        tdata->feat_dist_min = GSL_POSINF;
        tdata->corr_word_idx = 0;
        gsl_vector_float_view fi = gsl_vector_float_view_array (f->keys[feat_idx], f->keylen);
        tdata->fi = &fi.vector;

        if (tdata->vocab_len > 0)   //if ((attr->size[feat_idx]) > 40)
        {
            // find the closest one from vocab
            g_slist_foreach (tdata->vocab, &glist_calc_dist_vocab, tdata);
            size_t closest_idx = g_slist_index (tdata->vocab, tdata->wi);

            // check what was min dist for feature fi
            if (tdata->feat_dist_min > tdata->sal_new_vocab_sim_thresh)
            {
                // add a new word to vocab
                gsl_vector_float *new_word = gsl_vector_float_alloc (f->keylen);
                gsl_vector_float_memcpy (new_word, &fi.vector);
                tdata->vocab = g_slist_append (tdata->vocab, new_word);
                tdata->vocab_len = g_slist_length (tdata->vocab);

                // if we are exceeding existinig nw, increase the size
                if (tdata->vocab_len > tdata->nwoccur->size)
                    tdata->nwoccur = _extend_gslu_index (tdata->nwoccur);

                // fi just has been assigned to a new word
                tdata->corr_word_idx = tdata->vocab_len - 1;

                // index for global saliency
                gslu_boolean_set (occurrence_mask, tdata->corr_word_idx, 1);
                gslu_index_set (tdata->nwoccur, tdata->corr_word_idx, 1);
                gslu_index_set (image_bow, feat_idx, tdata->corr_word_idx);
            }
            else
            {
                // assign word idx from exising vocab
                // tdata->corr_word_idx = closest_idx;
                gslu_index_set (image_bow, feat_idx, closest_idx);

                if (gslu_boolean_get (occurrence_mask, closest_idx) == 0)
                {
                    gslu_boolean_set (occurrence_mask, closest_idx, 1);

                    if (update_vocab)
                    {
                        int current_nw = gslu_index_get (tdata->nwoccur, closest_idx);
                        gslu_index_set (tdata->nwoccur, closest_idx, current_nw + 1);
                    }
                }
            }
        }
        else
        {
            printf ("No vocab exist. Initializing vocab...\n");
            // initialize the vocab by adding it
            gsl_vector_float *new_word = gsl_vector_float_alloc (f->keylen);
            gsl_vector_float_memcpy (new_word, &fi.vector);
            tdata->vocab = g_slist_append (tdata->vocab, new_word);
            tdata->vocab_len = g_slist_length (tdata->vocab);

            gslu_boolean_set (occurrence_mask, 0, 1);
            gslu_index_set (tdata->nwoccur, 0, 1);
            gslu_index_set (image_bow, feat_idx, 0);
        }
    }

    // Local saliency
    // -----------------------------------------------------------------
    //t0 = timestamp_now ();
    gslu_index *vocab_histc = gslu_index_alloc (tdata->vocab_len);
    gslu_index_set_zero (vocab_histc);
    for (size_t feat_idx=0; feat_idx<f->npts; feat_idx++)
    {
        size_t word_i = gslu_index_get (image_bow, feat_idx);
        size_t cur_count_word_i = gslu_index_get (vocab_histc, word_i);
        gslu_index_set (vocab_histc, word_i, cur_count_word_i+1);
    }

    double bowE = 0.0;
    for (size_t wi=0; wi<tdata->vocab_len; wi++)
    {
        size_t count_word_i = gslu_index_get (vocab_histc, wi);

        if (count_word_i > 0)
        {
            // sum over non-zero probability
            double p = (double) count_word_i / (double) f->npts;
            bowE = bowE - p * log2(p);
        }

    }
    gsl_vector_set (tdata->bowE, imgnum_cur, bowE);
    gslu_index_set (tdata->vocablen_list, imgnum_cur, tdata->vocab_len);
    double S_L = bowE / log2 ((double) tdata->vocab_len);

    // Global saliency
    // -----------------------------------------------------------------
    // copy image_bow_indicator and save for future update
    gslu_boolean *image_bow_mask = gslu_boolean_alloc (tdata->vocab_len);
    gslu_boolean_set_zero (image_bow_mask);
    gslu_boolean_view occ_mask_sub = gslu_boolean_subvector (occurrence_mask, 0, tdata->vocab_len);
    gslu_boolean_memcpy (image_bow_mask, &occ_mask_sub.vector);
    tdata->image_bow_mask = g_slist_append (tdata->image_bow_mask, image_bow_mask);

#if 0
    // maintain inverted indeces
    t2 = timestamp_now ();
    for (int wi=0; wi<tdata->vocab_len; wi++)
    {
        if (gslu_boolean_get (occurrence_mask, wi) > 0)   // occured in the current image
        {
            if (wi > tdata->invidx->max_idx)
            {
                // we don't have imglist for wi
                tdata->invidx->max_idx = wi;
                GSList *imglist_tmp = NULL;
                imglist_tmp = g_slist_append (imglist_tmp, gu_dup (&imgnum_cur, sizeof imgnum_cur));
                tdata->invidx->index[wi] = imglist_tmp;
            }
            else
            {
                // it is already in invidx
                GSList *imglist = tdata->invidx->index[wi];
                imglist = g_slist_append (imglist, gu_dup (&imgnum_cur, sizeof imgnum_cur));
            }
        }
    }
    dt = timestamp_now () - t2;
#endif

    // Do batch update
    // -----------------------------------------------------------------
    bool do_batch = false;
    if (imgnum_cur % UPDATE_ALL_SG_STEP == 0 && imgnum_cur > 0) do_batch = true;

    //printf ("alive batch %d\n", do_batch);
    double sumidf = 0.0;
    if (do_batch)
    {
        //gslu_boolean_printfc (image_bow_mask, "mask", "%g", CblasTrans);
        //gslu_index_printfc (tdata->nwoccur, "nw", "%g", CblasTrans);

        t1 = timestamp_now ();
        _update_sumidf_all (tdata);
        dt = timestamp_now () - t1;
        printf ("Updated sumidf for ALL image. dt=%"PRId64"\n", dt);
        sumidf = gsl_vector_get (tdata->sumidf, imgnum_cur);
    }
    else
    {
        //gslu_boolean_printfc (image_bow_mask, "mask", "%g", CblasTrans);
        //gslu_index_printfc (tdata->nwoccur, "nw", "%g", CblasTrans);
        // calculate the current global saliency
        //_update_sumidf_invidx (tdata); // with batch update, we do not need this?

        sumidf = _calc_sum_idf (image_bow_mask, tdata->nwoccur, tdata->Nf);
        gsl_vector_set (tdata->sumidf, imgnum_cur, sumidf);
        tdata->sumidf_max = gsl_vector_max (tdata->sumidf);
    }
    if (tdata->sumidf_max <= 0) tdata->sumidf_max = 1E-6;   // prevent div by 0

    // publish the saliency score
    double S_G = sumidf / tdata->sumidf_max;

    perllcm_van_saliency_t *saliency = calloc (1, sizeof (*saliency));
    saliency->utime = f->utime;
    saliency->feattype = f->attrtype;
    saliency->npts = f->npts;
    saliency->S_L = S_L;
    saliency->bowE = bowE;
    saliency->vocab_len = tdata->vocab_len;
    if (saliency->S_L > tdata->S_L_thresh)
        saliency->is_S_L = true;
    saliency->S_G = S_G;
    if (saliency->S_G > tdata->S_G_thresh)
        saliency->is_S_G = true;
    perllcm_van_saliency_t_publish (shm->lcm, VAN_SALIENCY_CHANNEL, saliency);
    perllcm_van_saliency_t_destroy (saliency);

    dt = timestamp_now () - t0;
    printf ("S_L = %2.2f S_G = %2.2f:\tnpts=%d\tN=%d\t|w|=%zu\tdt=%"PRId64"\n", S_L, S_G, f->npts, tdata->imgcounter, tdata->vocab_len, dt);

    // save image in bow rep
    if (tdata->save_image_bow)
    {
        char bowimage_fname[512];
        snprintf (bowimage_fname, sizeof bowimage_fname, "%s/%"PRId64".bow", shm->logdir, f->utime);
        FILE *fbow = fopen (bowimage_fname, "wb");
        fprintf (fbow, "%zu\n", tdata->vocab_len);
        fprintf (fbow, "%g\n", bowE);
        fprintf (fbow, "%g\n", S_L);
        gslu_index_fprintf (fbow, image_bow, "%d");
        fclose (fbow);
    }

    // clean up
    gslu_index_free (image_bow);
    gslu_boolean_free (occurrence_mask);
    gslu_index_free (vocab_histc);
    perllcm_van_feature_attr_cvsurf_t_destroy (attr);
}

static void
hauv_bs_cnv_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const hauv_bs_cnv_t *msg, void *user)
{
    thread_data_t *tdata = user;

    // we need xyz position (only cnv)
    tdata->xyz_lv_curr[0] = msg->x;
    tdata->xyz_lv_curr[1] = msg->y;
    tdata->xyz_lv_curr[2] = msg->z;
}

static void
perllcm_isam_cmd_t_cmd_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_isam_cmd_t *msg, void *user )
{
    thread_data_t *tdata = user;

    // option_cmd is from viewer
    if (msg->mode == PERLLCM_ISAM_CMD_T_MODE_SAVE && tdata->save_saliency)
    {
        printf ("saving saliency to ./saliency.dat\n");

        // global
        _update_sumidf_all (tdata);
        GHashTable *bowE_hash = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &free);
        GHashTable *sumidf_hash = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &free);
        GHashTable *vocablen_hash = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &free);
        for (size_t i=0; i<tdata->imgcounter; i++)
        {
            int64 *utime = g_slist_nth_data (tdata->sal_utimelist, i);
            double sumidf = gsl_vector_get (tdata->sumidf, i);
            double bowE = gsl_vector_get (tdata->bowE, i);
            size_t vocablen = gslu_index_get (tdata->vocablen_list, i);
            if (utime)
            {
                // printf ("%g %zu\n", sumidf, vocablen);
                g_hash_table_insert (sumidf_hash, gu_dup (utime, sizeof (*utime)), gu_dup (&sumidf, sizeof (sumidf)));
                g_hash_table_insert (bowE_hash, gu_dup (utime, sizeof (*utime)), gu_dup (&bowE, sizeof (bowE)));
                g_hash_table_insert (vocablen_hash, gu_dup (utime, sizeof (*utime)), gu_dup (&vocablen, sizeof (vocablen)));
            }

        }

        vanu_saliency_save2disk (SALIENCY_FILE, tdata->sal_utimelist, bowE_hash, sumidf_hash, vocablen_hash, tdata->sumidf_max);

        g_hash_table_unref (sumidf_hash);
        g_hash_table_unref (bowE_hash);
        g_hash_table_unref (vocablen_hash);
    }
}

gpointer
saliency_thread (gpointer user)
{
    printf ("Spawning\n");

    // saliency_thread state
    thread_data_t *tdata = calloc (1, sizeof (*tdata));

    // load from config
    tdata->sal_new_vocab_sim_thresh = 0.2;
    bot_param_get_double (shm->param, "rtvan.saliency_thread.sal_new_vocab_sim_thresh", &tdata->sal_new_vocab_sim_thresh);
    tdata->sal_new_vocab_motion_thresh = 0.5;
    bot_param_get_double (shm->param, "rtvan.saliency_thread.sal_new_vocab_motion_thresh", &tdata->sal_new_vocab_motion_thresh);
    tdata->S_L_thresh = 0.6;
    bot_param_get_double (shm->param, "rtvan.saliency_thread.S_L_thresh", &tdata->S_L_thresh);
    tdata->S_G_thresh = 0.5;
    bot_param_get_double (shm->param, "rtvan.saliency_thread.S_G_thresh", &tdata->S_G_thresh);

    tdata->self_testing = false;
    botu_param_get_boolean_to_bool (shm->param, "rtvan.post_process.selftest_saliency", &tdata->self_testing);
    if (tdata->self_testing)                  shm->active = true;

    tdata->save_image_bow = false;
    botu_param_get_boolean_to_bool (shm->param, "rtvan.post_process.save_image_bow", &tdata->save_image_bow);
    printf ("self testing %d\n", tdata->self_testing);

    tdata->save_saliency = false;
    botu_param_get_boolean_to_bool (shm->param, "rtvan.post_process.save_saliency", &tdata->save_saliency);

    tdata->bowE = gsl_vector_calloc (EXTEND_LENGTH);
    tdata->sumidf = gsl_vector_calloc (EXTEND_LENGTH);
    tdata->vocablen_list = gslu_index_alloc (EXTEND_LENGTH);
    gslu_index_set_zero (tdata->vocablen_list);

    tdata->nwoccur = gslu_index_alloc (EXTEND_LENGTH);
    gslu_index_set_zero (tdata->nwoccur);
    tdata->invidx = init_invidx ();

    // lcm subscriptions
    perllcm_van_feature_t_subscription_t *perllcm_van_feature_t_sub =
        perllcm_van_feature_t_subscribe (shm->lcm, VAN_WORDS_CHANNEL,
                                         &perllcm_van_feature_t_callback, tdata);

    hauv_bs_cnv_t_subscription_t *hauv_bs_cnv_t_sub =
        hauv_bs_cnv_t_subscribe (shm->lcm, "HAUV_BS_CNV", &hauv_bs_cnv_t_callback, tdata);

    perllcm_isam_cmd_t_subscription_t *perllcm_isam_cmd_t_cmd_sub =
        perllcm_isam_cmd_t_subscribe (shm->lcm, "CMD", &perllcm_isam_cmd_t_cmd_callback, tdata);

    while (!shm->done)
    {
        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        lcmu_handle_timeout (shm->lcm, &timeout);
    }

    // clean up
    hauv_bs_cnv_t_unsubscribe (shm->lcm, hauv_bs_cnv_t_sub);
    perllcm_van_feature_t_unsubscribe (shm->lcm, perllcm_van_feature_t_sub);
    perllcm_isam_cmd_t_unsubscribe (shm->lcm, perllcm_isam_cmd_t_cmd_sub);
    g_slist_foreach (tdata->vocab, &glist_destroyer, NULL);
    g_slist_foreach (tdata->xyz_lv_prev, &glist_destroyer, NULL);
    gslu_vector_free (tdata->sumidf);
    gslu_index_free (tdata->nwoccur);
    g_slist_foreach (tdata->image_bow_mask, &glist_gslu_boolean_destroyer, NULL);
    free_invidx (tdata->invidx);
    free (tdata);

    printf ("Exiting\n");
    g_thread_exit (0);
    return NULL;
}
