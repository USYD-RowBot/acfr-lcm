#include <iostream>

#include "scan_match_thread.h"

void
pooldata_free (pool_data_t *pdata)
{
    perllcm_velodyne_laser_return_collection_t_destroy (pdata->lrc_i);
    perllcm_velodyne_laser_return_collection_t_destroy (pdata->lrc_j);
    free (pdata->channel_vlink);
    free (pdata);
}

void *
scan_match_thread_f (void *user_data) {
    
    pool_data_t *pdata = (pool_data_t *) user_data;
    
    std::cout << "[worker]\tProcessing (" << pdata->plink.utime_i << "," << pdata->plink.utime_j
        << ") dt = " << (pdata->plink.utime_i - pdata->plink.utime_j) << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j (new pcl::PointCloud<pcl::PointXYZ>);
    if (pdata->visualize || pdata->icp_params.use_icp || pdata->icp_params.use_gicp) {
        // put scans into pcl format    
        cloud_i->width    = pdata->lrc_i->num_lr;
        cloud_i->height   = 1;
        cloud_i->is_dense = false;
        cloud_i->points.resize (cloud_i->width * cloud_i->height);
        for (size_t i = 0; i < cloud_i->points.size (); ++i)
        {
          perllcm_velodyne_laser_return_t *lr = &(pdata->lrc_i->laser_returns[i]);
          cloud_i->points[i].x = lr->xyz[0];
          cloud_i->points[i].y = lr->xyz[1];
          cloud_i->points[i].z = lr->xyz[2];
        }
        
        cloud_j->width    = pdata->lrc_j->num_lr;
        cloud_j->height   = 1;
        cloud_j->is_dense = false;
        cloud_j->points.resize (cloud_j->width * cloud_j->height);
        for (size_t i = 0; i < cloud_j->points.size (); ++i)
        {
          perllcm_velodyne_laser_return_t *lr = &(pdata->lrc_j->laser_returns[i]);
          cloud_j->points[i].x = lr->xyz[0];
          cloud_j->points[i].y = lr->xyz[1];
          cloud_j->points[i].z = lr->xyz[2];
        }
    }
    
    // get prior
    double x_jis[6] = {0};
    ssc_head2tail (x_jis, NULL, pdata->plink.x_ji.mu, pdata->lrc_i->x_vs);
    double x_jsis[6] = {0};
    ssc_tail2tail (x_jsis, NULL, pdata->lrc_j->x_vs, x_jis);    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ji (new pcl::PointCloud<pcl::PointXYZ>);
    
    double x_jsis_est[6] = {0};
    bool valid_est = false;
        
    if (pdata->icp_params.use_icp) { // ICP
    
        // perform scan matching
        if (pdata->icp_params.debug) {
            std::cout << "[worker]\tStarting ICP ..." << std::endl;
        }
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputCloud(cloud_i);
        icp.setInputTarget(cloud_j);
        
        // Set the max correspondence distance  (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (pdata->icp_params.d_max);
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (pdata->icp_params.max_iters);
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (1e-8);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1);
        
        // set initial guess
        double R_isjs[9] = {0};
        double rph_jsis[3] = {x_jsis[3], x_jsis[4], x_jsis[5]};
        so3_rotxyz (R_isjs, rph_jsis);
        Eigen::Matrix4f T_jsis;
        T_jsis << R_isjs[0], R_isjs[1], R_isjs[2], x_jsis[0],
                  R_isjs[3], R_isjs[4], R_isjs[5], x_jsis[1],
                  R_isjs[6], R_isjs[7], R_isjs[8], x_jsis[2],
                  0,         0,         0,         1;
        icp.align(*cloud_ji, T_jsis);
        
        if (pdata->icp_params.debug) {
            std::cout << "[worker]\tT_jsis:" << std::endl << T_jsis << std::endl;
            std::cout << "[worker]\tICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;
        }
        
        // decompose the resutl
        Eigen::Matrix4f T_jsis_est = icp.getFinalTransformation(); 
        x_jsis_est[0] = T_jsis_est(0,3);
        x_jsis_est[1] = T_jsis_est(1,3);
        x_jsis_est[2] = T_jsis_est(2,3);
        double R_isjs_est[9] = {T_jsis_est(0,0), T_jsis_est(0,1), T_jsis_est(0,2),
                                T_jsis_est(1,0), T_jsis_est(1,1), T_jsis_est(1,2),
                                T_jsis_est(2,0), T_jsis_est(2,1), T_jsis_est(2,2)};
        so3_rot2rph (R_isjs_est, &x_jsis_est[3]);
        
        if (icp.hasConverged()) {
            valid_est = true;
            cout << "[worker]\tICP has converged in " << pdata->icp_params.max_iters << " iterations." << endl;
        } else {
            cout << "[worker]\tICP has NOT converged in " << pdata->icp_params.max_iters << " iterations." << endl;
        }
        
                
    } else if (0){ //pdata->icp_params.use_gicp) {
       
        // perform scan matching
        if (pdata->icp_params.debug) {
            std::cout << "[worker]\tStarting GICP ..." << std::endl;
        }
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputCloud(cloud_i);
        gicp.setInputTarget(cloud_j);
        
        //// Set the max correspondence distance  (e.g., correspondences with higher distances will be ignored)
        gicp.setMaxCorrespondenceDistance (pdata->icp_params.d_max);
        // Set the maximum number of iterations (criterion 1)
        gicp.setMaximumIterations (pdata->icp_params.max_iters);
        // Set maximum inner iterations
        gicp.setMaximumOptimizerIterations (8);
        // Set the transformation epsilon (criterion 2)
        gicp.setTransformationEpsilon (1e-8);
        // Set the euclidean distance difference epsilon (criterion 3)
        gicp.setEuclideanFitnessEpsilon (1);
        
        // set initial guess
        double R_isjs[9] = {0};
        double rph_jsis[3] = {x_jsis[3], x_jsis[4], x_jsis[5]};
        so3_rotxyz (R_isjs, rph_jsis);
        Eigen::Matrix4f T_jsis;
        T_jsis << R_isjs[0], R_isjs[1], R_isjs[2], x_jsis[0],
                  R_isjs[3], R_isjs[4], R_isjs[5], x_jsis[1],
                  R_isjs[6], R_isjs[7], R_isjs[8], x_jsis[2],
                  0,         0,         0,         1;
        gicp.align(*cloud_ji, T_jsis);
        
        if (pdata->icp_params.debug) {
            std::cout << "[worker]\tInitial XFM:" << std::endl << T_jsis << std::endl;
            std::cout << "[worker]\tgicp has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl;
            std::cout << "[worker]\tFinal XFM: " << std::endl << gicp.getFinalTransformation() << std::endl;
        }
        
        pcl::transformPointCloud (*cloud_i, *cloud_ji, T_jsis);
        
        // decompose the resutl
        Eigen::Matrix4f T_jsis_est = gicp.getFinalTransformation(); 
        x_jsis_est[0] = T_jsis_est(0,3);
        x_jsis_est[1] = T_jsis_est(1,3);
        x_jsis_est[2] = T_jsis_est(2,3);
        double R_isjs_est[9] = {T_jsis_est(0,0), T_jsis_est(0,1), T_jsis_est(0,2),
                                T_jsis_est(1,0), T_jsis_est(1,1), T_jsis_est(1,2),
                                T_jsis_est(2,0), T_jsis_est(2,1), T_jsis_est(2,2)};
        so3_rot2rph (R_isjs_est, &x_jsis_est[3]);
        
        if (gicp.hasConverged()) {
            valid_est = true;
            cout << "[worker]\tgicp has converged in less than " << pdata->icp_params.max_iters << " iterations." << endl;
        } else {
            cout << "[worker]\tgicp has NOT converged in " << pdata->icp_params.max_iters << " iterations." << endl;
        }

    } else if (pdata->icp_params.use_gicp) { //Segal's GICP implementation
    
        if (pdata->icp_params.debug) 
            std::cout << "[worker]\tStarting GICP ..." << std::endl;
        
        
        dgc::gicp::GICPPointSet ps_i, ps_j;
        dgc_transform_t T_jsis_0, T_jsis_delta, T_jsis_est;
        dgc_transform_identity(T_jsis_delta);
        
        // set initial guess
        dgc_transform_identity(T_jsis_0);
        dgc_transform_rotate_z(T_jsis_0, x_jsis[5]);
        dgc_transform_rotate_y(T_jsis_0, x_jsis[4]);
        dgc_transform_rotate_x(T_jsis_0, x_jsis[3]);
        dgc_transform_translate(T_jsis_0, x_jsis[0], x_jsis[1], x_jsis[2]);
    
        for (int i = 0; i < pdata->lrc_i->num_lr; ++i)
        {
          perllcm_velodyne_laser_return_t *lr = &(pdata->lrc_i->laser_returns[i]);
          dgc::gicp::GICPPoint ptmp;
          ptmp.x = lr->xyz[0];
          ptmp.y = lr->xyz[1];
          ptmp.z = lr->xyz[2];
          ps_i.AppendPoint(ptmp); 
        }
        for (int i = 0; i < pdata->lrc_j->num_lr; ++i)
        {
          perllcm_velodyne_laser_return_t *lr = &(pdata->lrc_j->laser_returns[i]);
          dgc::gicp::GICPPoint ptmp;
          ptmp.x = lr->xyz[0];
          ptmp.y = lr->xyz[1];
          ptmp.z = lr->xyz[2];
          ps_j.AppendPoint(ptmp); 
        }
        
        g_mutex_lock(pdata->gicp_mutex);
        if (pdata->icp_params.debug) 
            cout << "[worker]\tBuilding KDTree ..." << endl;
        ps_i.SetGICPEpsilon(pdata->icp_params.gicp_epsilon);
        ps_j.SetGICPEpsilon(pdata->icp_params.gicp_epsilon);
        ps_i.BuildKDTree();
        ps_j.BuildKDTree();
        if (pdata->icp_params.debug) 
            cout << "[worker]\tComputing surface normals/matrices ..." << endl;
        ps_i.ComputeMatrices();
        ps_j.ComputeMatrices();
        g_mutex_unlock(pdata->gicp_mutex);
        
        if(pdata->icp_params.debug) {
            // save data for debug/visualizations
            ps_i.SavePoints("pts1.dat");
            ps_i.SaveMatrices("mats1.dat");
            ps_j.SavePoints("pts2.dat");
            ps_j.SaveMatrices("mats2.dat");
        }
    
        // align the point clouds
        if (pdata->icp_params.debug) 
            cout << "[worker]\tAligning point cloud..." << endl;
        ps_j.SetDebug(pdata->icp_params.debug);
        ps_j.SetMaxIterationInner(8);
        ps_j.SetMaxIteration(pdata->icp_params.max_iters);
        g_mutex_lock(pdata->gicp_mutex);
        int iterations = ps_j.AlignScan(&ps_i, T_jsis_0, T_jsis_delta, pdata->icp_params.d_max);
        g_mutex_unlock(pdata->gicp_mutex);
        
        dgc_transform_copy(T_jsis_est, T_jsis_0);
        dgc_transform_left_multiply(T_jsis_est, T_jsis_delta);
        
        if(pdata->icp_params.debug) {
            dgc_transform_print(T_jsis_0, "T_jsis_0");
            dgc_transform_print(T_jsis_delta, "T_jsis_delta");
            dgc_transform_print(T_jsis_est, "T_jsis_est");
        }
        
        if (pdata->visualize) {
            cloud_ji->width    = ps_i.Size();
            cloud_ji->height   = 1;
            cloud_ji->is_dense = false;
            cloud_ji->points.resize (cloud_ji->width * cloud_ji->height);
            for (size_t i = 0; i < cloud_ji->points.size (); ++i)
            {
                double x = ps_i[i].x;
                double y = ps_i[i].y;
                double z = ps_i[i].z;
                dgc_transform_point(&x, &y, &z, T_jsis_est);
                cloud_ji->points[i].x = x;
                cloud_ji->points[i].y = y;
                cloud_ji->points[i].z = z;
            }
        }
        
        if(pdata->icp_params.debug) {
            ofstream fout("iterations.txt");
            if(!fout) {
              return 0;
            }
            fout << "Converged in " << iterations << " iterations." << endl;
            fout.close();
        }
        
        // decompose the resutl
        x_jsis_est[0] = T_jsis_est[0][3];
        x_jsis_est[1] = T_jsis_est[1][3];
        x_jsis_est[2] = T_jsis_est[2][3];
        double R_isjs_est[9] = {T_jsis_est[0][0], T_jsis_est[0][1], T_jsis_est[0][2],
                                T_jsis_est[1][0], T_jsis_est[1][1], T_jsis_est[1][2],
                                T_jsis_est[2][0], T_jsis_est[2][1], T_jsis_est[2][2]};
        so3_rot2rph (R_isjs_est, &x_jsis_est[3]);
        
        if (iterations < pdata->icp_params.max_iters) {
            valid_est = true;
            cout << "[worker]\tGICP has converged in " << iterations << " iterations." << endl;
        } else {
            cout << "[worker]\tGICP has not converged in " << pdata->icp_params.max_iters << " iterations." << endl;
        }
         
    }
    
    if (pdata->visualize) {
        g_mutex_lock(pdata->vis_t_data->vis_mutex);
        
        pcl::visualization::PCLVisualizer *viewer = pdata->vis_t_data->viewer;
        
        viewer->removeAllPointClouds ();
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud_j, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ> (cloud_j, red, "cloud_j");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_j");
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (cloud_i, 0, 0, 255);
        viewer->addPointCloud<pcl::PointXYZ> (cloud_i, blue, "cloud_i");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_i");
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cyan (cloud_ji, 0, 255, 255);
        viewer->addPointCloud<pcl::PointXYZ> (cloud_ji, cyan, "cloud_ji");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_ji");
        viewer->spinOnce (10);
        
        g_mutex_unlock(pdata->vis_t_data->vis_mutex);
    }
    
    // check the prior
    if (valid_est) {
        
        double x_sv_i[6] = {0};
        ssc_inverse (x_sv_i, NULL, pdata->lrc_i->x_vs);
        double x_sv_j[6] = {0};
        ssc_inverse (x_sv_j, NULL, pdata->lrc_j->x_vs);
        double x_js_i[6] = {0};
        ssc_head2tail (x_js_i, NULL, x_jsis_est, x_sv_i);
        double x_ji_est[6] = {0};
        ssc_tail2tail (x_ji_est, NULL, x_sv_j, x_js_i);
        
        gsl_vector_view mu_v = gsl_vector_view_array (pdata->plink.x_ji.mu, 6);
        gsl_vector_view x_ji_est_v = gsl_vector_view_array (x_ji_est, 6);
        gsl_matrix_view Sigma_v = gsl_matrix_view_array (pdata->plink.x_ji.Sigma, 6, 6);
        GSLU_MATRIX_VIEW (invCov, 6, 6);
        gslu_matrix_inv (&invCov.matrix, &Sigma_v.matrix);
        GSLU_INDEX_VIEW (c, 3, {3, 4, 5});
        double dist = gslu_vector_mahal_circ_dist (&x_ji_est_v.vector,
                                                   &mu_v.vector,
                                                   &invCov.matrix,
                                                   &c.vector);
        
        //double mdist = gsl_cdf_chisq_Pinv (1-2*gsl_cdf_ugaussian_P (-0.99), 6);
        double mdist = gsl_cdf_chisq_Pinv (pdata->mahal_dist_t, 6);
        
        if (dist < mdist) {
            cout << "[worker]\tPassed Mahalanobis distance check (";
            cout << pdata->mahal_dist_t <<") against prior: " << dist << " < " << mdist << endl;
            
            // publish a factor
            perllcm::isam_vlink_t vlink = {0};
            vlink.id1 = pdata->plink.utime_j;
            vlink.id2 = pdata->plink.utime_i;
            vlink.link_id = 0;
            vlink.sensor_id = perllcm::isam_vlink_t::SENSOR_LASER;
            vlink.n = 6;
            vlink.z.resize(vlink.n);
            memcpy (&(vlink.z[0]), x_ji_est, 6*sizeof (double));

            vlink.n2 = 6*6;
            vlink.R.resize(vlink.n2);
            memcpy (&(vlink.R[0]), pdata->R, 6*6*sizeof (double));
            vlink.link_type = perllcm::isam_vlink_t::LINK_POSE3D;
            vlink.accept = true;
            vlink.accept_code = perllcm::isam_vlink_t::CODE_ACCEPTED;
            
            lcm::LCM lcm;
            // check lcm connection
            if(!lcm.good()) {
                ERROR ("lcm_create () failed!");
            }
            lcm.publish (pdata->channel_vlink, &vlink);
            
        } else {
            cout << "[worker]\tFailed Mahalanobis distance check (";
            cout << pdata->mahal_dist_t <<") against prior: " << dist << " >= " << mdist << endl;
        }
    }
    

    pooldata_free (pdata);
    return NULL;
}