#include <fstream>  
#include <iostream> 
#include <time.h>
#include <dirent.h>


#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <bot_lcmgl_client/lcmgl.h>

#include "map_builder.h"

using namespace std;
using namespace Eigen;

void
PVNMapBuilder::run_build_vocab ()
{
    //
    //// load the camera channel names
    //CamConfig camconf = CamConfig (param, use_camera);
    //
    //int64_t tic, tic_start = timestamp_now ();
    //int pgr_cnt = 0;
    //int feat_cnt = 0;
    //
    //cv::Mat features(VOCAB_MAX_FEATURES, 128, CV_32F);
    //
    //set<int64_t> used_utimes;
    //    
    //int n_vimg_dirs = bot_param_get_array_len (param, "pvn-map-builder.vocab_img_dirs");
    //if (n_vimg_dirs <= 0) {
    //    cout << "[vocab]\tNo vocab image dirs set!" << endl;
    //    return;
    //}
    //
    //char **vimg_dirs = bot_param_get_str_array_alloc (param, "pvn-map-builder.vocab_img_dirs");
    //for (int i=0; i<n_vimg_dirs; i++) {
    //
    //    // open directory
    //    DIR *d;
    //    struct dirent *dir;
    //    d = opendir (vimg_dirs[i]);
    //    if (d == NULL) {
    //        ERROR ("Could not open vocab directory file directory: %s", vimg_dirs[i]);
    //        return;
    //    }
    //
    //    cout << "[vocab]\tProcessing dir: " << vimg_dirs[i] << endl;
    //    
    //    // loop over all files in directory
    //    while ((dir = readdir (d))) {
    //     
    //        int len = strlen (dir->d_name);
    //        if (strcmp (dir->d_name, "." ) == 0 || strcmp (dir->d_name, ".." ) == 0
    //            || dir->d_type == DT_DIR || strcmp (&(dir->d_name[len-4]), ".pgr") != 0) {
    //            //cout << "[vocab]\tSkipping: " << dir->d_name << endl;
    //            continue;
    //        }
    //        
    //        int64_t utime;
    //        sscanf (dir->d_name, "%ld.pgr", &utime);
    //        
    //        // make sure we aren't too close in time to an existing image
    //        if (used_utimes.empty()) {
    //            used_utimes.insert(utime);
    //        } else {
    //            pair<set<int64_t>::iterator,set<int64_t>::iterator> ret;
    //            ret = used_utimes.equal_range(utime);
    //            int64_t lb = *ret.first;
    //            int64_t ub = *ret.second;
    //            
    //            if (abs (lb - utime) >= VOCAB_MIN_USEC_DIST &&
    //                abs (ub - utime) >= VOCAB_MIN_USEC_DIST ) {
    //                
    //                used_utimes.insert(utime);
    //            } else {
    //                continue;    
    //            }
    //        }
    //        
    //        
    //        tic = timestamp_now();
    //        ImageData id = ImageData::ImageData (&camconf, vimg_dirs[i], use_camera, utime);
    //        if (id.load (&lcm)) {
    //            cout << "[vocab]\tNo image data found, skipping." << endl;
    //            continue;
    //        }
    //        cout << "\r" << "[vocab]\tLoaded image data for file: " << pgr_cnt;
    //        cout << ", " << feat_cnt << " total feats (" << (timestamp_now()-tic)/1e6 << " secs).       " << flush;
    //        
    //        
    //        for (int icam = 0; icam < LB3_NUM_CAMS; icam++) {
    //            // locate all the features
    //            perllcm::van_feature_collection_t *fc = &(id.feature_collections[icam]);
    //            if (fc->ntypes != 1) {
    //                ERROR ("Only expecting 1 feature type.");
    //                return;
    //            }
    //            
    //            perllcm::van_feature_t *f = &(fc->f[0]);
    //            if (f->attrtype != perllcm::van_feature_t::ATTRTYPE_SIFTGPU) {
    //                ERROR ("Only expecting SIFTGPU features.");
    //                return;
    //            }
    //            
    //            // store the features
    //            for (int j=0; j<f->npts; j++) {
    //                for (int k=0; k<f->keylen; k++)
    //                    features.at<float> (feat_cnt, k) = f->keys[j][k];
    //                feat_cnt++;
    //                
    //                if (feat_cnt >= VOCAB_MAX_FEATURES)
    //                break;
    //            }
    //            
    //            if (feat_cnt >= VOCAB_MAX_FEATURES)
    //                break;
    //        }
    //        pgr_cnt++;
    //        
    //        if (feat_cnt >= VOCAB_MAX_FEATURES)
    //            break;
    //    
    //    }
    //    cout << endl;
    //    
    //    closedir (d);
    //
    //}
    //bot_param_str_array_free (vimg_dirs);
    //
    //cout << "[vocab]\tLoaded " << feat_cnt << " features. (" << (timestamp_now()-tic_start)/1e6 << " secs)." << endl;
    //
    //// cluster the features ----------------------------------------------------
    //tic = timestamp_now();
    //cvflann::KMeansIndexParams kmeans_params (8, 11, cvflann::FLANN_CENTERS_KMEANSPP); // branching, itterations, init
    //cv::Mat clusters(VOCAB_MAX_SIZE, 128, CV_32F);
    //    
    ////// pack features
    ////for (int i=0; i<feat_cnt; i++) {
    ////    for (int j=0; j<128; j++) {
    ////        features.at<float> (i,j) = desc[i][j];
    ////    }
    ////}
    //
    //int num_vocab = 0;
    //if (features.rowRange(cv::Range(0,feat_cnt)).isContinuous()) {
    //    num_vocab = cv::flann::hierarchicalClustering < cvflann::L2<float> > (features.rowRange(cv::Range(0,feat_cnt)), clusters, kmeans_params);    
    //} else {
    //    cout << "[vocab]\tFeatures not continuous moving." << endl;
    //    cv::Mat features_c = features.rowRange(cv::Range(0,feat_cnt)).clone();
    //    features.create (1, 1, CV_32F); // should clear memory
    //    cout << "[vocab]\tStarting clustering." << endl;
    //    num_vocab = cv::flann::hierarchicalClustering < cvflann::L2<float> > (features_c, clusters, kmeans_params);    
    //}
    //
    //cout << "[vocab]\tClustered " << num_vocab << " vocab for " << feat_cnt << " features. (" << (timestamp_now()-tic)/1e6 << " secs)." << endl;
    //
    //perllcm::pvn_vis_vocab_t vis_vocab;
    //
    //vis_vocab.nvocab = num_vocab;
    //vis_vocab.keylen = 128;
    //vis_vocab.vocab.resize (vis_vocab.nvocab);
    //for (int i=0; i<num_vocab; i++) {
    //    vis_vocab.vocab[i].resize(vis_vocab.keylen);
    //    for (int j=0; j<128; j++) {
    //        vis_vocab.vocab[i][j] = clusters.at<float> (i,j);
    //    }
    //}
    //
    //char filename[PATH_MAX];
    //snprintf (filename, sizeof filename, "%s/%s.vocab", map_dir, map_name);
    //int32_t ret = pvnu_lcm_fwrite <perllcm::pvn_vis_vocab_t> (filename, vis_vocab);
    //if (ret < 0) {
    //    ERROR ("couldn't write %s to disk!", filename);
    //    return;
    //}
    //cout << "[vocab]\tVocab saved. (" << (timestamp_now()-tic_start)/1e6 << " secs)." << endl;
}