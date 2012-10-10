#include "est_pmf.h"

// initializes all index values to -1
void
init_index_t (perllcm_est_navigator_index_t *index)
{    
    index->proc_state_len = 0;

    index->u_len = 0; 
    
    // Translation
    index->x = -1;
    index->y = -1;
    index->z = -1;
    // Euler angle rotation
    index->r = -1;
    index->p = -1;
    index->h = -1;
    // body frame velocities
    index->v = -1;
    index->u = -1;
    index->w = -1;
    // body frame angular velocities
    index->a = -1;
    index->b = -1;
    index->c = -1;
    // world frame velocities
    index->x_dot = -1;
    index->y_dot = -1;
    index->z_dot = -1;
    // world frame angular velocities
    index->r_dot = -1;
    index->p_dot = -1;
    index->h_dot = -1;
}
