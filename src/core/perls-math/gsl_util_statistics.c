#include <gsl/gsl_sort.h>
#include <gsl/gsl_sort_vector.h>
#include <gsl/gsl_statistics.h>

#include "gsl_util_vector.h"
#include "gsl_util_statistics.h"

double
gslu_stats_median_array (double data[], size_t stride, size_t n)
{
    gsl_sort (data, stride, n);
    return gsl_stats_median_from_sorted_data (data, stride, n);
}


double
gslu_stats_median_vector (const gsl_vector *v)
{
    gsl_vector *b = gslu_vector_clone (v);
    gsl_sort_vector (b);
    double median = gsl_stats_median_from_sorted_data (b->data, b->stride, b->size);
    gslu_vector_free (b);
    return median;
}
