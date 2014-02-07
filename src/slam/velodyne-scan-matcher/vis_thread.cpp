#include <iostream>

#include "vis_thread.h"

#include "perls-common/timeutil.h"


void * 
vis_thread_f(void *user_data) {
    
    
    vis_thread_data_t *t_data = (vis_thread_data_t *)user_data;
    
    g_mutex_lock(t_data->vis_mutex);
    t_data->viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");
    t_data->viewer->setBackgroundColor (255, 255, 255);
    t_data->viewer->addCoordinateSystem (1.0);
    t_data->viewer->initCameraParameters ();
    //t_data->viewer->setCameraPosition (0, 0, 10, 0, 0, 1);
    t_data->viewer->spinOnce ();
    
    while (!t_data->viewer->wasStopped ()) {
        t_data->viewer->spinOnce (10);
        g_mutex_unlock(t_data->vis_mutex);
        timeutil_usleep (3e5);
        g_mutex_lock(t_data->vis_mutex);
    }
    
    
    return NULL;
}