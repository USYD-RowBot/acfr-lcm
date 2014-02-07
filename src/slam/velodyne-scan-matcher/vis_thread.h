#ifndef PERLS_VELODYNE_SCAN_MATCHER_VIS_THREAD_H
#define PERLS_VELODYNE_SCAN_MATCHER_VIS_THREAD_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>

//#include <gicp.h>
#include <glib.h>

typedef struct _vis_thread_data_t vis_thread_data_t;
struct _vis_thread_data_t {
    pcl::visualization::PCLVisualizer *viewer;
    GMutex *vis_mutex;
};


void * 
vis_thread_f(void *user_data);

#endif