#include <stdio.h>
#include <stdlib.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_sort_vector.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Sparse>

#include <bot_lcmgl_client/lcmgl.h>
#include <opencv/cv.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-math/ssc.h"
#include "perls-math/gsl_util.h"
#include "perls-vision/camera.h"


#include "perls-lcmtypes++/perllcm/van_feature_collection_t.hpp"
#include "perls-lcmtypes++/perllcm/pvn_vis_map_t.hpp"
#include "perls-lcmtypes++/perllcm/viewer_image_pccs_t.hpp"

#include "pvn_util.h"

using namespace std;
using namespace Eigen;

void
pvnu_lrc_to_xyz (std::vector < std::vector <float> > *xyz,
                          const perllcm_velodyne_laser_return_collection_t *lrc) {
  
    // project points into camera frame
    xyz->resize (lrc->num_lrl);
    for (int j=0; j<lrc->num_lrl; j++) {
        (*xyz)[j].resize (3);
        (*xyz)[j][0] =  lrc->laser_returns_lite[j].xyz[0];
        (*xyz)[j][1] =  lrc->laser_returns_lite[j].xyz[1];
        (*xyz)[j][2] =  lrc->laser_returns_lite[j].xyz[2];
    }    
}

void
pvnu_exemplar_to_xyz (std::vector < std::vector <float> > *xyz,
                 const perllcm::pvn_eview_map_exemplar_t *mne) {
  
    xyz->resize (mne->npts);
    for (int j=0; j<mne->npts; j++) {
        (*xyz)[j].resize (3);
        (*xyz)[j][0] =  mne->x[j];
        (*xyz)[j][1] =  mne->y[j];
        (*xyz)[j][2] =  mne->z[j];
    }    
}



void
pvnu_vmc_to_xyz (std::vector < std::vector <float> > *xyz,
                          const perllcm::pvn_vis_map_cluster_t *vmc) {
  
    // project points into camera frame
    xyz->resize (vmc->npts);
    for (int j=0; j<vmc->npts; j++) {
        (*xyz)[j].resize (3);
        (*xyz)[j][0] =  vmc->x[j];
        (*xyz)[j][1] =  vmc->y[j];
        (*xyz)[j][2] =  vmc->z[j];
    }    
}

void
pvnu_project_to_image (std::vector < std::vector <float> > *uv,
                       std::vector <int> *proj_inds,
                       std::vector < std::vector <float> > *xyz_cam_out,
                       const std::vector < std::vector <float> > xyz,
                       const double x_w_ci[6],
                       const Eigen::Matrix3d K,
                       int max_height,
                       int max_width,
                       double max_range,
                       bool lcmgl) {
    
    std::vector < std::vector <float> > xyz_cam;
    if (x_w_ci != NULL) {
        xyz_cam.resize (xyz.size());
        for (size_t j=0; j<xyz.size(); j++) {
            xyz_cam[j].resize(3);
            double x_ci_feat[6] = {0};
            double x_w_feat[6] = {xyz[j][0], xyz[j][1], xyz[j][2], 0, 0, 0};
            ssc_tail2tail (x_ci_feat, NULL, x_w_ci, x_w_feat);
            xyz_cam[j][0] =  x_ci_feat[0];
            xyz_cam[j][1] =  x_ci_feat[1];
            xyz_cam[j][2] =  x_ci_feat[2];
        }
    } else {
        xyz_cam = xyz;
    }
    
    
    if (xyz_cam_out != NULL) {
        (*xyz_cam_out) = xyz_cam;
    }

    
    double half_fov = M_PI/2;
    const double s2_45 = sin (half_fov) * sin (half_fov);
    const double c2_45 = cos (half_fov) * cos (half_fov);
    std::vector <int> vis_inds;
    for (size_t j=0; j<xyz_cam.size(); j++) {
        // skip points that aren't in 90 degree cone in front of camera
        double cone_test = (pow (xyz_cam[j][0] ,2) + pow (xyz_cam[j][1] ,2))*c2_45;
        cone_test -= pow (xyz_cam[j][2] ,2)*s2_45;
        if ((xyz_cam[j][2] > 0 && cone_test <=0) && (max_range < 0 || xyz_cam[j][2] < max_range)) {
            vis_inds.push_back (j);
        }
    } 
    
    // project points into image plane
    for (size_t ind=0; ind<vis_inds.size(); ind++) {
        int j = vis_inds[ind];
        Eigen::Vector3d xyz_tmp;
        xyz_tmp << xyz_cam[j][0], xyz_cam[j][1], xyz_cam[j][2];
        Eigen::Vector3d uvw = K*xyz_tmp;
        uvw(0) = uvw(0)/uvw(2);
        uvw(1) = uvw(1)/uvw(2);
        // make sure it projects into the image
        if (uvw(0) > 0 && uvw(0) <= max_height &&
            uvw(1) > 0 && uvw(1) <= max_width) {
            (*proj_inds).push_back (j);
            std::vector <float> uv_tmp;
            uv_tmp.push_back (uvw(0));
            uv_tmp.push_back (uvw(1));
            (*uv).push_back (uv_tmp);
        } 
    } 
    
    if (lcmgl) {
        // lcmgl debug code for point projection (doesnt seem to be exactly working, only shows first point set)
        lcm::LCM lcm;
        bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm.getUnderlyingLCM(), "DEBUG");
        lcmglPushMatrix();
        lcmglRotated(180, 1, 0, 0);
        lcmglPointSize(2);
        lcmglBegin(LCMGL_POINTS);
        lcmglColor3f(0.0, 1.0, 1.0);
        for (size_t i=0; i<xyz_cam.size(); i++) {
            lcmglVertex3d(xyz_cam[i][0], xyz_cam[i][1], xyz_cam[i][2]);
        }
        lcmglEnd();
        
        lcmglBegin(LCMGL_POINTS);
        lcmglColor3f(1.0, 0.0, 0.0);
        for (size_t i=0; i<vis_inds.size(); i++) {
            int j = vis_inds[i];
            lcmglVertex3d(xyz_cam[j][0], xyz_cam[j][1], xyz_cam[j][2]);
        }
        lcmglEnd();
        
        lcmglBegin(LCMGL_POINTS);
        lcmglColor3f(0.0, 0.0, 1.0);
        for (size_t i=0; i<proj_inds->size(); i++) {
            int j = (*proj_inds)[i];
            lcmglVertex3d(xyz_cam[j][0], xyz_cam[j][1], xyz_cam[j][2]);
        }
        lcmglEnd();
        lcmglPopMatrix();
        bot_lcmgl_switch_buffer(lcmgl);
        bot_lcmgl_destroy(lcmgl);
        timeutil_usleep (1e6);
    }
}


perllcm::van_scene_prior_t
pvnu_pack_scene_prior (std::vector < std::vector <float> > xyz_cam,
                       std::vector < std::vector <float> > uv,
                       std::vector <int> *proj_inds,
                       int64_t utime) {
    
    perllcm::van_scene_prior_t sp = {0};
    
    sp.utime = utime;
    if (proj_inds != NULL) {
        sp.npts = proj_inds->size();
    } else {
        sp.npts = xyz_cam.size();
    }

    if (sp.npts > 0) {
        gsl_vector *tmpZ = gsl_vector_alloc (sp.npts);
        for (int j=0; j<sp.npts; j++) {
        
            int k;
            if (proj_inds != NULL) {
                k = (*proj_inds)[j];
            } else {
                k = j;
            }
        
            sp.id.push_back (0);
            sp.X.push_back (xyz_cam[k][0]);
            sp.Y.push_back (xyz_cam[k][1]);
            sp.Z.push_back (xyz_cam[k][2]);
            sp.u.push_back (uv[j][0]);
            sp.v.push_back (uv[j][1]);
            gsl_vector_set (tmpZ, j, xyz_cam[k][2]);
        }
        gsl_sort_vector (tmpZ);
        sp.Zmin = gsl_vector_get (tmpZ, 0);
        sp.Zmax = gsl_vector_get (tmpZ, tmpZ->size-1);
        sp.Z10  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.10);
        sp.Z20  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.20);
        sp.Z30  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.30);
        sp.Z40  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.40);
        sp.Z50  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.50);
        sp.Z60  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.60);
        sp.Z70  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.70);
        sp.Z80  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.80);
        sp.Z90  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.90);
        sp.Zmean = gsl_stats_mean (tmpZ->data, tmpZ->stride, tmpZ->size);
        sp.Zstd = gsl_stats_sd (tmpZ->data, tmpZ->stride, tmpZ->size);
        sp.Zabsdev = gsl_stats_absdev (tmpZ->data, tmpZ->stride, tmpZ->size);
    
        gsl_vector_free (tmpZ);

    } else {
        sp.npts = 0;
        sp.id.resize(0);
        sp.X.resize(0);
        sp.Y.resize(0);
        sp.Z.resize(0);
        sp.u.resize(0);
        sp.v.resize(0);
    }   
    
    return sp;
}

float
_median (std::vector < float > v)
{
    if (v.size() < 1)
        std::cout << "ERROR _median() expecting non-zero length array" << std::endl;
    
    if (v.size() < 2)
        return v[0];
    
    size_t n = ceil ((double)v.size() / 2.0);
    std::nth_element(v.begin(), v.begin()+n, v.end());
    
    if (v.size() % 2 != 0) {
        return (v[n] + v[n-1])/2.0;    
    } else {
        return v[n];    
    }
}

std::vector<int>
pvnu_median_outlier_rejection (std::vector<float> u, std::vector<float> v,
                               std::vector<float> up, std::vector<float> vp,
                               float *reproj_error) {
    
    
    std::vector<int> inliers;
    inliers.resize(0);
    std::vector<float> rpe; // reporjection error squared
    rpe.resize (u.size());
    for (size_t i=0; i<u.size(); i++) {
        rpe[i] = (pow (u[i]-up[i],2) + pow (v[i]-vp[i],2));
    }
    
    std::vector<float>rpe_tmp = rpe; //median function changes order so send a copy
    float rmean_est = _median (rpe_tmp); // the robust mean estimate
    
    if (rmean_est > 20*20) {
        //ERROR ("Estimated mean reprojection error > 20 px");
        return inliers;
    }
    
    std::vector<float> rpad; // reporjection abs deviation
    rpad.resize (u.size());
    for (size_t i=0; i<u.size(); i++) {
        rpad[i] = (fabs (rpe[i] - rmean_est));
    }
    float rstd_est =  1.48 * _median (rpad);

    double sum = 0.0;
    for (size_t i=0; i<u.size(); i++) {
        float cutoff = rstd_est*3.0; //3 standard deviations away
        if (rpe[i] < cutoff ) {
            sum += rpe[i];
            inliers.push_back(i);
        }
    }
    
    (*reproj_error) = sum/inliers.size();
    
    return inliers;
}

void
pvnu_image_map_matching (std::vector<int> *sel_map_out,
                        std::vector<int> *sel_img_out,
                        perllcm::van_feature_t *f_img,
                        perllcm::van_feature_t *f_map,
                        vector< vector<float> > xyz_map,
                        double x_w_ci[6],
                        double S_w_ci[6*6],
                        double K[3*3],
                        double sec_nn_thresh,
                        int use_pccs,
                        perllcm::viewer_image_pccs_t *vis_pccs_out) {
        
    int nf_map = f_map->npts;
    int nf_img = f_img->npts;
    
    Matrix < double, 2, 6, RowMajor > J_pose;
    Matrix < double, 2, 3, RowMajor > J_XYZ;
    Matrix < double, 6, 6, RowMajor > S_pose (S_w_ci);
    Matrix < double, 3, 3, RowMajor > S_XYZ = Matrix3d::Identity()*(0.05*0.05);
    
    Matrix < bool, Dynamic, Dynamic, RowMajor > cmatrix (nf_map, nf_img);
    if (use_pccs)
        cmatrix.setZero();
    else
        cmatrix.setConstant(1);
        
    vector <  Matrix < double, 2, 2, RowMajor > > S_uvs;

    // loop over map points projected into image
    //int64_t tic_pccs = timestamp_now();
    //int64_t inner_loop_time = 0;
    //int64_t sparse_insert_time = 0;
    double k2 = gsl_cdf_chisq_Pinv (0.9, 2);
    
    if (use_pccs) {
    
        for (int i=0; i<nf_map; i++) {
            
            Matrix < double, 2, 2, RowMajor > S_uv;
            Matrix < double, 2, 2, RowMajor > S_uv_inv;
            // calculate covariance
            Vector3d XYZ (xyz_map[i][0], xyz_map[i][1], xyz_map[i][2]); // use world frame point not camera frame point
            vis_camera_projection_jacobian(J_pose.data(), J_XYZ.data(), x_w_ci, K, XYZ.data());
            S_uv = J_pose * S_pose * J_pose.transpose();
            S_uv += J_XYZ * S_XYZ * J_XYZ.transpose();   //verified against original matlab code seems to be correct;
            S_uvs.push_back (S_uv);
            S_uv_inv = S_uv.inverse();
            double S00 = S_uv_inv(0,0);
            double S01 = S_uv_inv(0,1);
            double S10 = S_uv_inv(1,0);
            double S11 = S_uv_inv(1,1);
    
            for (int j=0; j<nf_img; j++) { // all the time wasting happens in this loop
                
                //int64_t tic_inner_loop = timestamp_now();
                
                // bounding ellipse is given by:
                // uv_error'*S_uv_inv*uv_error = chiSquare2dof
                // do the matrix multiplication manually because its a little faster and this loop gets called alot
                double ud = f_img->u[j] - f_map->u[i];
                double vd = f_img->v[j] - f_map->v[i];
                double epsilon = ud*ud*S00 + ud*vd*S10 + ud*vd*S01 + vd*vd*S11;
                
                if (epsilon <= k2) {
                    //int64_t tic_sparse_insert = timestamp_now();
                    cmatrix(i, j) = true;
                    //sparse_insert_time += (timestamp_now() - tic_sparse_insert);
                }
                //inner_loop_time += (timestamp_now() - tic_inner_loop);
            } 
        }
    }
    
    //int64_t toc_pccs = timestamp_now();   
    //int64_t tic_matching = timestamp_now();
    std::vector < int > sel_map;
    std::vector < int > sel_img;
    // find best matches using second nearest neighbor test
    // loop over the sparse matrix only visiting valid entries
    // we want image to map matches so thats why we do this col major
    for (int j=0; j<nf_img; j++) {
        
        int ind1, ind2;
        float dmin1=1e6, dmin2=1e6;
        Matrix <float, 128, 1 > key_img ( &(f_img->keys[j][0]) );
        
        for (int i=0; i<nf_map; i++) {          
            if (!cmatrix(i,j))
                continue;
                
            Matrix <float, 128, 1 > key_map ( &(f_map->keys[i][0]) );
            
            float dist = (key_map - key_img).transpose() * (key_map - key_img);
            
            if (dist < dmin1) {
                // swap 1st best with 2nd best
                dmin2 = dmin1;
                ind2 = ind1;
                // add new first best
                dmin1 = dist;
                ind1 = i;
            } else if (dist < dmin2) {
                dmin2 = dist;
                ind2 = i;
            }            
        }
        
    
        if (dmin1 < (sec_nn_thresh*sec_nn_thresh*dmin2) &&
            dmin1 < 1e5 && dmin2 < 1e5) { // make sure more than 2 features actually compared
    
            sel_map.push_back (ind1); 
            sel_img.push_back (j);
        }
    }
    
    //std::vector < int > sel_map;
    //std::vector < int > sel_img;
    //// find best matches using second nearest neighbor test
    //// loop over the sparse matrix only visiting valid entries
    //// we want image to map matches so thats why we do this col major
    //for (int i=0; i<nf_map; i++) {
    //    
    //    int ind1, ind2;
    //    float dmin1=1e6, dmin2=1e6;
    //    
    //    Matrix <float, 128, 1 > key_map ( &(f_map->keys[i][0]) );
    //    
    //    for (int j=0; j<nf_img; j++) {          
    //        
    //        if (!cmatrix(i,j))
    //            continue;
    //            
    //        Matrix <float, 128, 1 > key_img ( &(f_img->keys[j][0]) );
    //        
    //        float dist = (key_map - key_img).transpose() * (key_map - key_img);
    //        
    //        if (dist < dmin1) {
    //            // swap 1st best with 2nd best
    //            dmin2 = dmin1;
    //            ind2 = ind1;
    //            // add new first best
    //            dmin1 = dist;
    //            ind1 = j;
    //        } else if (dist < dmin2) {
    //            dmin2 = dist;
    //            ind2 = j;
    //        }            
    //    }
    //    
    //
    //    if (dmin1 < (sec_nn_thresh*sec_nn_thresh*dmin2) &&
    //        dmin1 < 1e5 && dmin2 < 1e5) { // make sure more than 2 features actually compared
    //
    //        sel_map.push_back (i); 
    //        sel_img.push_back (ind1);
    //    }
    //}
    //int64_t toc_matching = timestamp_now();
    
    if (vis_pccs_out != NULL) {
        perllcm::viewer_image_pccs_t vis_pccs;
        
        vis_pccs.num_corrs = sel_map.size();
        if (use_pccs) {
            vis_pccs.num_covp = vis_pccs.num_corrs;
        } else {
            vis_pccs.num_covp = 0;
        }
        
        for (int i=0; i<vis_pccs.num_corrs; i++) {
            vis_pccs.u.push_back (f_img->u[sel_img[i]]);
            vis_pccs.v.push_back (f_img->v[sel_img[i]]);
            vis_pccs.up.push_back (f_map->u[sel_map[i]]);
            vis_pccs.vp.push_back (f_map->v[sel_map[i]]);
        
            if (use_pccs) {
                vector < float > tmp (4,0);
                tmp[0] = S_uvs[sel_map[i]](0);
                tmp[1] = S_uvs[sel_map[i]](1);
                tmp[2] = S_uvs[sel_map[i]](2);
                tmp[3] = S_uvs[sel_map[i]](3);
                vis_pccs.covp.push_back (tmp);
            }
        }
    
        vis_pccs.type = perllcm::viewer_image_pccs_t::TYPE_PUTATIVE;
        (*vis_pccs_out) = vis_pccs;
    }
    
    //cout << "\t\t\t" << "Pccs time: \t\t" << (toc_pccs - tic_pccs)/1e6 << endl;
    //cout << "\t\t\t" << "  Pccs inner loop time:    " << (inner_loop_time)/1e6 << endl;
    //cout << "\t\t\t" << "  Pccs sparse insert time: " << (sparse_insert_time)/1e6 << endl;
    //cout << "\t\t\t" << "Matching time: \t\t" << (toc_matching - tic_matching)/1e6 << endl;

    (*sel_map_out) = sel_map;
    (*sel_img_out) = sel_img;
}


int
pvnu_lcm_handle_timeout (lcm::LCM *lcm, struct timeval *timeout)
{
    int fd = lcm->getFileno ();

    fd_set rfds;
    FD_ZERO (&rfds);
    FD_SET (fd, &rfds);

    int ret = select (fd + 1, &rfds, NULL, NULL, timeout);

    if (ret == -1) {
#ifdef LINUX
        if (errno == EINTR)
            return pvnu_lcm_handle_timeout (lcm, timeout);
#endif
        if (errno != EINTR)
            PERROR ("select()");
    }
    else if (ret == 0) {
        /* Timeout */
    }
    else {
        /* We have data. */
        if (FD_ISSET (fd, &rfds))
            lcm->handle ();
    }

    return ret;
}

