#include "map_builder.h"

using namespace std;

void
PVNMapBuilder::run_build_laser_map ()
{
    // load camera config
    std::vector<PVNCamConfig> camconfs;
    if (lm_color && !lm_force_lite) {
        ERROR("Currently can only find color for forced lite maps");
        return;
    }
    if (lm_color && lm_force_lite) {
        camconfs = pvn_cam_config_load_lb3 (param);
    }    
    
    // parse in graph
    char graph_pc[PATH_MAX];
    sprintf (graph_pc, "%s/pc.dat", graph_dir);
    cout << "[laser map]\tLoading graph: " << graph_pc << endl;
    perllcm_pose3d_collection_t *pc = NULL;
    int32_t ret = LCMU_FREAD (graph_pc, &pc, perllcm_pose3d_collection_t);
    if (ret < 0) {
        cout << "[laser map]\tCouldn't read " << graph_pc << " from disk. ret=" << ret << endl;
        return;
    }
    cout << "[laser map]\tFound " << pc->npose << " graph nodes." << endl;
    
    perllcm_pvn_laser_map_t *lmap = (perllcm_pvn_laser_map_t *)calloc (1, sizeof (*lmap));
    lmap->num_lrcs = pc->npose;
    lmap->lrcs = (perllcm_velodyne_laser_return_collection_t *)calloc (lmap->num_lrcs, sizeof (perllcm_velodyne_laser_return_collection_t));
    if (lm_color && lm_force_lite) {
        lmap->num_rgbcs = lmap->num_lrcs;
        lmap->rgbcs = (perllcm_color_byte_collection_t *)calloc (lmap->num_rgbcs, sizeof (perllcm_color_byte_collection_t));
    } else {
        lmap->num_rgbcs = 0;
        lmap->rgbcs = NULL;
    }
    
    double ll_deg[2] = {0};
    bot_param_get_double_array_or_fail (param, "site.orglatlon", ll_deg, 2);
    lmap->orglat = ll_deg[0]*DTOR;
    lmap->orglon = ll_deg[1]*DTOR;
    bot_param_get_double (param, "site.orgalt", &(lmap->orglon));
    
    lmap->xyz_min[0] = 1e10; lmap->xyz_min[1] = 1e10; lmap->xyz_min[2] = 1e10;
    lmap->xyz_max[0] = -1e10; lmap->xyz_max[1] = -1e10; lmap->xyz_max[2] = -1e10;
    lmap->total_lr = 0;   
    
    for (int i=0; i<lmap->num_lrcs; i++) {
        
        perllcm_velodyne_laser_return_collection_t *lrc = NULL;
        char filename[PATH_MAX];
        snprintf (filename, sizeof filename, "%s/%ld.lrc", scan_dir, pc->pose[i].utime);
        int32_t ret = LCMU_FREAD (filename, &lrc, perllcm_velodyne_laser_return_collection_t);
        if (ret < 0) {
            cout << "[laser map]\tCouldn't read " << filename << " from disk. ret=" << ret << endl;
            continue;
        }
       
        // assign a pose to this collection return
        lrc->has_pose = 1;
        double x_ws[6];
        ssc_head2tail (x_ws, NULL, pc->pose[i].mu, lrc->x_vs);
        memcpy (lrc->pose, x_ws, 6*sizeof (double));
        
        // setup lrc output
        perllcm_velodyne_laser_return_collection_t *lrc_out = &(lmap->lrcs[i]);
        perllcm_color_byte_collection_t *rgbc_out = NULL;
        if (lm_color && lm_force_lite) {
            rgbc_out = &(lmap->rgbcs[i]);
        }
        memcpy (lrc_out, lrc, sizeof (perllcm_velodyne_laser_return_collection_t));

        // set output sizes
        int num_lr = 0;
        int num_lrl = 0;
        if (lm_force_lite) {
            num_lr = 0;
            num_lrl = lrc->num_lrl + lrc->num_lr;
        } else {
            num_lr = lrc->num_lr;
            num_lrl = lrc->num_lrl;
        }
        
        if (lm_every_nth_return > 1) {
            num_lr = ceil((double)num_lr/(double)lm_every_nth_return);
            num_lrl = ceil((double)num_lrl/(double)lm_every_nth_return);            
        }
        
        lrc_out->num_lr = num_lr;
        lrc_out->num_lrl = num_lrl;
        lmap->total_lr += lrc_out->num_lr;
        lmap->total_lr += lrc_out->num_lrl;
        
        lrc_out->laser_returns = (perllcm_velodyne_laser_return_t *)
                                  calloc (lrc_out->num_lr, sizeof (perllcm_velodyne_laser_return_t));
        lrc_out->laser_returns_lite = (perllcm_velodyne_laser_return_lite_t *)
                                       calloc (lrc_out->num_lrl, sizeof (perllcm_velodyne_laser_return_lite_t));
        if (lm_color && lm_force_lite) {
            rgbc_out->n = lrc_out->num_lrl;
            rgbc_out->r = (uint8_t *) calloc (rgbc_out->n, sizeof (uint8_t));
            rgbc_out->g = (uint8_t *) calloc (rgbc_out->n, sizeof (uint8_t));
            rgbc_out->b = (uint8_t *) calloc (rgbc_out->n, sizeof (uint8_t));
            memset (rgbc_out->r, 0, rgbc_out->n*sizeof (uint8_t));
            memset (rgbc_out->g, 255, rgbc_out->n*sizeof (uint8_t));
            memset (rgbc_out->b, 255, rgbc_out->n*sizeof (uint8_t));
        }
                
        // find the min and max
        int k = 0;
        for (int j=0; j<lrc->num_lr; j++) {

            if (lm_every_nth_return > 1 && (j % lm_every_nth_return))
                continue;
            
            double x_wlr[6] = {0};
            double x_slr[6] = {lrc->laser_returns[j].xyz[0], 
                               lrc->laser_returns[j].xyz[1],
                               lrc->laser_returns[j].xyz[2], 0,0,0};
            ssc_head2tail (x_wlr, NULL, x_ws, x_slr);
            if (x_wlr[0] < lmap->xyz_min[0]) lmap->xyz_min[0] = x_wlr[0];
            if (x_wlr[1] < lmap->xyz_min[1]) lmap->xyz_min[1] = x_wlr[1];
            if (x_wlr[2] < lmap->xyz_min[2]) lmap->xyz_min[2] = x_wlr[2];
            if (x_wlr[0] > lmap->xyz_max[0]) lmap->xyz_max[0] = x_wlr[0];
            if (x_wlr[1] > lmap->xyz_max[1]) lmap->xyz_max[1] = x_wlr[1];
            if (x_wlr[2] > lmap->xyz_max[2]) lmap->xyz_max[2] = x_wlr[2];
            
            if (lm_force_lite) {
                lrc_out->laser_returns_lite[k].xyz[0] = lrc->laser_returns[j].xyz[0];
                lrc_out->laser_returns_lite[k].xyz[1] = lrc->laser_returns[j].xyz[1];
                lrc_out->laser_returns_lite[k].xyz[2] = lrc->laser_returns[j].xyz[2];
                lrc_out->laser_returns_lite[k].intensity = lrc->laser_returns[j].intensity;
            } else {
                lrc_out->laser_returns[k] = lrc->laser_returns[j];    
            }
            k++;
            
        }
        k = 0;
        for (int j=0; j<lrc->num_lrl; j++) {

            if (lm_every_nth_return > 1 && (j % lm_every_nth_return))
                continue;
            
            double x_wlr[6] = {0};
            double x_slr[6] = {lrc->laser_returns_lite[j].xyz[0], 
                               lrc->laser_returns_lite[j].xyz[1],
                               lrc->laser_returns_lite[j].xyz[2], 0,0,0};
            ssc_head2tail (x_wlr, NULL, x_ws, x_slr);
            if (x_wlr[0] < lmap->xyz_min[0]) lmap->xyz_min[0] = x_wlr[0];
            if (x_wlr[1] < lmap->xyz_min[1]) lmap->xyz_min[1] = x_wlr[1];
            if (x_wlr[2] < lmap->xyz_min[2]) lmap->xyz_min[2] = x_wlr[2];
            if (x_wlr[0] > lmap->xyz_max[0]) lmap->xyz_max[0] = x_wlr[0];
            if (x_wlr[1] > lmap->xyz_max[1]) lmap->xyz_max[1] = x_wlr[1];
            if (x_wlr[2] > lmap->xyz_max[2]) lmap->xyz_max[2] = x_wlr[2];
            
            if (lm_force_lite) {
                lrc_out->laser_returns_lite[lrc->num_lr+k] = lrc->laser_returns_lite[j];
            } else {
                lrc_out->laser_returns_lite[k] = lrc->laser_returns_lite[j];
            }
            k++;
            
        }
        
        if (lm_color && lm_force_lite) {
            
            std::vector<std::string> image_dirs;
            std::string dir_str (image_dir);
            for (int ii_cam=0; ii_cam<6; ii_cam++)
                image_dirs.push_back(dir_str);
            PVNImageData id(camconfs, image_dirs, pc->pose[i].utime, "pgr");
            id.load_orig_img (&lcm);
            // load image associated with this point

            std::multiset <int> updated;
            for (int ii_cam=0; ii_cam<6; ii_cam++) {
                
                // for debugging
                //char lcm_chan[1024] = {0};
                //sprintf(lcm_chan , "%s_UNDIST", camconfs[ii_cam].image_channel.c_str());
                //lcm.publish (lcm_chan, &(id.images[ii_cam]));
                
                // find transfrom betwen velodyne frame and camera frame
                double x_vel_ci[6] = {0};
                ssc_head2tail (x_vel_ci, NULL, camconfs[ii_cam].x_vel_h, camconfs[ii_cam].x_hs);
                
                // project laser points into image             
                std::vector < std::vector <float> > xyz;
                pvnu_lrc_to_xyz(&xyz, lrc_out);
                std::vector <int> proj_inds;
                std::vector < std::vector <float> > uv;
                std::vector < std::vector <float> > xyz_cam;
                pvnu_project_to_image (&uv, &proj_inds, &xyz_cam,
                                       xyz,
                                       x_vel_ci,
                                       camconfs[ii_cam].K,
                                       camconfs[ii_cam].height,
                                       camconfs[ii_cam].width,
                                       -1, false);
                
                // pack a scene prior and send it over lcm for visualization
                //perllcm::van_scene_prior_t sp = pvnu_pack_scene_prior (xyz_cam, uv, &proj_inds, pc->pose[i].utime);
                //char sp_chan[64]= {0};
                //sprintf (sp_chan, "SCENE_PRIOR_CAM%d", ii_cam);
                //lcm.publish (sp_chan, &sp);

                // get colors where avaliable
                for (size_t j=0; j<proj_inds.size(); j++) {
                    int u = uv[j][0], v = uv[j][1];
                    int row_stride = id.images[ii_cam].width*3;
                    int base_ind = v*row_stride + u*3;
     
                    if (camconfs[ii_cam].mask->imageData[v*camconfs[ii_cam].mask->width + u]) { // dont use if projecting into mask
                        
                        int N = updated.count(proj_inds[j]);
                        if (0 == N) { // first update

                            rgbc_out->r[proj_inds[j]] = id.images[ii_cam].data[base_ind + 0];
                            rgbc_out->g[proj_inds[j]] = id.images[ii_cam].data[base_ind + 1];
                            rgbc_out->b[proj_inds[j]] = id.images[ii_cam].data[base_ind + 2];
                            updated.insert (proj_inds[j]);
                        
                        }
                        //else if (N >= 1) {
                        //    
                        //    // average muddies the colors
                        //    //double alpha = (double)(N)/(double)(N+1);
                        //    //double beta = 1.0/(double)(N+1);
                        //    //rgbc_out->r[proj_inds[j]] = round (alpha*(double)rgbc_out->r[proj_inds[j]] + beta*(double)id.images[ii_cam].data[base_ind + 0]);
                        //    //rgbc_out->g[proj_inds[j]] = round (alpha*(double)rgbc_out->g[proj_inds[j]] + beta*(double)id.images[ii_cam].data[base_ind + 1]);
                        //    //rgbc_out->b[proj_inds[j]] = round (alpha*(double)rgbc_out->b[proj_inds[j]] + beta*(double)id.images[ii_cam].data[base_ind + 2]);
                        //    //updated.insert (proj_inds[j]);
                        //    
                        //    // keep the brightest color (not magnitude RGB but difference between colors)
                        //    //int diff_max_new = 0;
                        //    //int diff = abs(id.images[ii_cam].data[base_ind + 0] - id.images[ii_cam].data[base_ind + 1]);
                        //    //if (diff > diff_max_new) diff_max_new = diff;
                        //    //diff = abs(id.images[ii_cam].data[base_ind + 0] - id.images[ii_cam].data[base_ind + 2]);
                        //    //if (diff > diff_max_new) diff_max_new = diff;
                        //    //diff = abs(id.images[ii_cam].data[base_ind + 1] - id.images[ii_cam].data[base_ind + 2]);
                        //    //if (diff > diff_max_new) diff_max_new = diff;
                        //    //
                        //    //int diff_max_old = 0;
                        //    //diff = abs(rgbc_out->r[proj_inds[j]] - rgbc_out->g[proj_inds[j]]);
                        //    //if (diff > diff_max_old) diff_max_old = diff;
                        //    //diff = abs(rgbc_out->r[proj_inds[j]] - rgbc_out->b[proj_inds[j]]);
                        //    //if (diff > diff_max_old) diff_max_old = diff;
                        //    //diff = abs(rgbc_out->b[proj_inds[j]] - rgbc_out->g[proj_inds[j]]);
                        //    //if (diff > diff_max_old) diff_max_old = diff;
                        //    //
                        //    //if (diff_max_old < diff_max_new) {
                        //    //    rgbc_out->r[proj_inds[j]] = id.images[ii_cam].data[base_ind + 0];
                        //    //    rgbc_out->g[proj_inds[j]] = id.images[ii_cam].data[base_ind + 1];
                        //    //    rgbc_out->b[proj_inds[j]] = id.images[ii_cam].data[base_ind + 2];
                        //    //    updated.insert (proj_inds[j]);
                        //    //}
                        //    
                        //    // max magnitude
                        //    //int mag_new = id.images[ii_cam].data[base_ind + 0]
                        //    //              + id.images[ii_cam].data[base_ind + 1]
                        //    //              + id.images[ii_cam].data[base_ind + 2];
                        //    //int mag_old = rgbc_out->r[proj_inds[j]]
                        //    //              + rgbc_out->g[proj_inds[j]]
                        //    //              + rgbc_out->b[proj_inds[j]];
                        //    //              
                        //    //if (mag_old < mag_new) {
                        //    //    rgbc_out->r[proj_inds[j]] = id.images[ii_cam].data[base_ind + 0];
                        //    //    rgbc_out->g[proj_inds[j]] = id.images[ii_cam].data[base_ind + 1];
                        //    //    rgbc_out->b[proj_inds[j]] = id.images[ii_cam].data[base_ind + 2];
                        //    //    updated.insert (proj_inds[j]);
                        //    //}
                        //     
                        //}
                    }                    
                }
            }
        }

        cout << "Scan " << i << " of " << lmap->num_lrcs << endl;
        perllcm_velodyne_laser_return_collection_t_destroy (lrc);
    }

    
    cout << "[laser map]\tMap Build: Total Points = " << lmap->total_lr << endl;
    cout << "[laser map]\tMin xyz = " << lmap->xyz_min[0] << " " << lmap->xyz_min[1] << " " << lmap->xyz_min[2] << endl;
    cout << "[laser map]\tMax xyz = " << lmap->xyz_max[0] << " " << lmap->xyz_max[1] << " " << lmap->xyz_max[2] << endl;
    
    char filename[PATH_MAX];
    snprintf (filename, sizeof filename, "%s/%s-laser.lmap", map_dir, map_name);
    ret = LCMU_FWRITE (filename, lmap, perllcm_pvn_laser_map_t);
    if (ret < 0)
        ERROR ("couldn't write %s to disk!", filename);
        
    perllcm_pvn_laser_map_t_destroy (lmap);    
    
    
}
