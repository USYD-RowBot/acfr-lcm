#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h> // needed for PRId64 macros

#include <opencv/cv.h>

#include "perls-common/cache.h"
#include "perls-common/timestamp.h"

#include "perls-lcmtypes/perllcm_van_example_t.h"

#define MAX_NNODES   10
#define IMGCACHE_MAX 10


static void
value_destroy_func (void *value)
{
    perllcm_van_example_t_destroy (value);
}

static void *
value_copy_func (const void *value)
{
    return perllcm_van_example_t_copy (value);
}

void
printf_key (cache_t *cache, int64_t key)
{
    perllcm_van_example_t *value = cache_pop (cache, key);
    if (value) {
        printf ("key=%"PRId64"\tvalue.utime=%"PRId64"\n", key, value->utime);
        free (value);
    }
    else
        printf ("key=%"PRId64" does not exist!\n", key);
}

static void
imgcache_value_destroy (void *value)
{
    IplImage *img = value;
    cvReleaseImage (&img);
}

static void *
imgcache_value_copy (const void *value)
{
    const IplImage *img = value;
    return cvCloneImage (img);
}

int
main (int argc, char *argv[])
{
    cache_t *cache = cache_new (MAX_NNODES, &value_copy_func, &value_destroy_func);


    // over populate it so things are pushed out of cache
    size_t i=0;
    for (i=0; i<10*MAX_NNODES; i++) {
        int64_t key = i;
        perllcm_van_example_t *value = malloc (sizeof (*value));
        value->utime = i;
        cache_push (cache, key, value);
        perllcm_van_example_t *value0 = cache_pop (cache, 0);
        perllcm_van_example_t_destroy (value0);
        printf ("i=%zd\tnnodes=%zd\n", i, cache_nnodes (cache));
    }

    // value stored MAX_NNODES iterations ago not there b/c the 0th key has persisted because of cache_pop(),
    // (normallay it would be if we hadn't made any calls to cache_pop())
    printf_key (cache, i-MAX_NNODES);

    // value associated with key=0 is still in the cache because of cache_pop()
    printf_key (cache, 0);

    cache_destroy (cache);

    // second cache on image
    cache_t *imgcache = cache_new (IMGCACHE_MAX, &imgcache_value_copy, &imgcache_value_destroy);

    IplImage *img = cvCreateImage (cvSize(1360,1024), IPL_DEPTH_8U, 1); // ~1.2MB per image 

    // if you see 100MB is taken by this proc --> there's mem leak
    // w/o leak ~20MB is used by this proc.
    for (int i=0; i<100; i++) {
        int64_t utime = i;
        cache_push (imgcache, utime, cvCloneImage (img));
        printf ("adding image to %d th node in the cache with max size %d\n", i, IMGCACHE_MAX);
    }

    cvWaitKey(0);
}
