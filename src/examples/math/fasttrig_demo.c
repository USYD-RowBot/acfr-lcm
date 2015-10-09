#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include "perls-common/timestamp.h"
#include "perls-math/fasttrig.h"

int
main (int argc, char *argv[])
{
    fasttrig_init ();

    if (1)
    {
        printf ("\nfasttrig accuracy test:\n");
        fasttrig_test_trig ();
        printf ("\n");
        fasttrig_test_arctrig ();
    }

    int64_t start, dt1, dt2;
    double s, c, t;
    const size_t MAX_ITERS = 10000000;

    printf ("\nfasttrig speed test (%lu iterations): math vs fasttrig (us)\n", (long unsigned int) MAX_ITERS);


    /* sincos */
    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        sincos (timestamp_now (), &s, &c);
    }
    dt1 = timestamp_now () - start;

    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        fsincos (timestamp_now (), &s, &c);
    }
    dt2 = timestamp_now () - start;
    printf ("fsincos: %10"PRId64" %10"PRId64"\n", dt1, dt2);


    /* fsin */
    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        s = sin (d);
    }
    dt1 = timestamp_now () - start;

    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        s = fsin (d);
    }
    dt2 = timestamp_now () - start;
    printf ("fsin:    %10"PRId64" %10"PRId64"\n", dt1, dt2);


    /* ftan */
    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        t = tan (d);
    }
    dt1 = timestamp_now () - start;

    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        t = ftan (d);
    }
    dt2 = timestamp_now () - start;
    printf ("ftan:    %10"PRId64" %10"PRId64"\n", dt1, dt2);


    /* facos */
    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        c = acos (d);
    }
    dt1 = timestamp_now () - start;

    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        c = facos (d);
    }
    dt2 = timestamp_now () - start;
    printf ("facos:   %10"PRId64" %10"PRId64"\n", dt1, dt2);

    /* fasin */
    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        s = asin (d);
    }
    dt1 = timestamp_now () - start;

    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        s = fasin (d);
    }
    dt2 = timestamp_now () - start;
    printf ("fasin:   %10"PRId64" %10"PRId64"\n", dt1, dt2);


    /* fatan2 */
    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        t = atan2 (d, d);
    }
    dt1 = timestamp_now () - start;

    start = timestamp_now ();
    for (size_t i=0; i<MAX_ITERS; i++)
    {
        int64_t d = timestamp_now ();
        t = fatan2 (d, d);
    }
    dt2 = timestamp_now () - start;
    printf ("fatan2:  %10"PRId64" %10"PRId64"\n", dt1, dt2);
}
