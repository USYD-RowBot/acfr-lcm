#include <liblas/liblas.hpp>
#include <fstream>  
#include <iostream> 
#include <time.h>
#include <dirent.h>

#include "map_builder.h"

#include "perls-math/gsl_util.h"


using namespace std;

// find a timestamp for the scan file name, not sure exactly how to do this
// have day-of-year and year as well as what appears to be seconds into the data collection
// shouldnt really matter as long as we make a unique timestamp
// use the seconds associated with the first point
// right now does not appear to be based on the day the data was collected, maybe when it was processed?
int64_t
find_las_timestamp (liblas::Reader *reader) {
    
    liblas::Header const& header = reader->GetHeader();
    char ts[1024] = {0};
    sprintf (ts, "%d-%d", header.GetCreationYear (), header.GetCreationDOY ());
    struct tm t = {0};
    time_t epoch = {0};
    if ( strptime(ts, "%Y-%j", &t) != NULL )
      epoch = mktime(&t);
    else
        ERROR ("Unable to read timestamp");
    
    reader->ReadNextPoint();
    liblas::Point const& p = reader->GetPoint();
    
    epoch += p.GetTime ();
    reader->Reset ();
    
    return (int64_t) epoch*1e6 ;
    
    // not right
    //reader->ReadNextPoint();
    //liblas::Point const& p = reader->GetPoint();
    //uint64_t utime = p.GetTime() + 1e9;
    //reader->Reset();
    //return utime;
    
}

double
geodetic2geocentric_deg (double lat_deg) {       
        
        double lat_tmp = lat_deg*DTOR;
        
        double a = 6378137.0000;
        double b = 6356752.3142;
        double r = (b*b)/(a*a);

        lat_tmp = atan(r*tan(lat_tmp));
    
        return lat_tmp*RTOD;
}

void
PVNMapBuilder::process_las (char *filename, std::ofstream *fgraph){
    
    std::ifstream ifs;
    ifs.open(filename, std::ios::in | std::ios::binary);

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream (ifs);
    
    liblas::Header const& header = reader.GetHeader ();

    //std::cout << "Compressed: " << header.Compressed ();
    //std::cout << " Signature: " << header.GetFileSignature ();
    //std::cout << " Points count: " << header.GetPointRecordsCount () << std::endl;
    
    int64_t utime = find_las_timestamp (&reader);
    
    // generate random subsampling list
    uint32_t total_returns = header.GetPointRecordsCount();
    uint8_t *mask = (uint8_t *)calloc (total_returns, sizeof (*mask));
    memset (mask, 0, total_returns*sizeof (uint8_t));
    uint32_t num_returns = ceil ((double)total_returns*las_subsample_pct);

    bool subsamp = false;
    if (num_returns == total_returns)
        subsamp = false; 
    else {
        subsamp = true; 
        gsl_rng *r = gslu_rand_rng_alloc ();
        uint32_t *set = (uint32_t *)calloc (total_returns, sizeof (*set));
        uint32_t *subset = (uint32_t *)calloc (num_returns, sizeof (*subset));
        for (uint32_t i=0; i<total_returns; i++) {set[i] = i;}
        gsl_ran_choose (r, subset, num_returns, set, total_returns, sizeof (uint32_t));
        gsl_rng_free (r);
        for (uint32_t i=0; i<num_returns; i++)
            mask[subset[i]] = 1;
            
        free (set);    
        free (subset);        
    }
    
    // generate lrc ------------------------------------------------------------
    perllcm_velodyne_laser_return_collection_t *lrc = velodyne_calloc_laser_return_collection (0, num_returns);
    lrc->utime = utime;
    
    //double x_off = header.GetOffsetX ();
    //double y_off = header.GetOffsetY ();
    //double z_off = header.GetOffsetZ ();
    //double x_scale = header.GetScaleX ();
    //double y_scale = header.GetScaleY ();
    //double z_scale = header.GetScaleZ ();
    
    // using offset xyz as centroid doesnt give a good center pose for the scan
    // double ll_deg[2] = {header.GetOffsetY(), header.GetOffsetX()};
    // instead find lat lon alt centroid
    double lat_c = 0, lon_c = 0, alt_c = 0;
    while (reader.ReadNextPoint()) {   
        liblas::Point const& p = reader.GetPoint();
        lat_c += p.GetY()/(double)total_returns;
        lon_c += p.GetX()/(double)total_returns;
        alt_c += p.GetZ()/(double)total_returns;
    }
    double ll_deg[2] = {lat_c, lon_c};
    reader.Reset ();
    double yx[2] = {0};
    bot_gps_linearize_to_xy (llxy, ll_deg, yx);
    lrc->pose[0] = yx[1];
    lrc->pose[1] = yx[0];
    //lrc->pose[2] = - (header.GetOffsetZ() - org_alt); // in NED
    lrc->pose[2] = - (alt_c - org_alt); // in NED
    lrc->pose[3] = 0; lrc->pose[4] = 0; lrc->pose[5] = 0;
    lrc->has_pose = 1;
    memset (lrc->x_vs, 0, 6*sizeof (double));
    
    // write the entry in the isam graph file
    *fgraph << "Pose3d_Factor " << utime;
    *fgraph << " (" << lrc->pose[0] << ", " << lrc->pose[1] << ", " << lrc->pose[2] << "; 0, 0, 0) ";
    *fgraph << "{1e6,0,0,0,0,0,1e6,0,0,0,0,1e6,0,0,0,1e6,0,0,1e6,0,1e6}";
    *fgraph << endl;
    
    int i = 0, j = 0;
    while (reader.ReadNextPoint()) {
       
        if (subsamp && !mask[j]) {
            j++;
            continue;
        }
        j++;
        
        liblas::Point const& p = reader.GetPoint();
        
        perllcm_velodyne_laser_return_lite_t *lr = &(lrc->laser_returns_lite[i]);
        
        double ll_deg[2] = {p.GetY(), p.GetX()};
        double yx[2] = {0};
        bot_gps_linearize_to_xy (llxy, ll_deg, yx);
        // IN NED
        lr->xyz[0] = yx[1] - lrc->pose[0];
        lr->xyz[1] = yx[0] - lrc->pose[1];
        lr->xyz[2] = -(p.GetZ() - org_alt) - lrc->pose[2];
        lr->intensity = (uint8_t) p.GetIntensity();
        // using lite def so dont fill
        //lr->raw_range = 0;
        //lr->range = 0;
        //lr->ctheta = 0;
        //lr->theta = 0;
        //lr->phi = 0;
        //lr->physical = 0;
        //lr->logical = 0;
        //lr->motion_compensated = 1;
        //lr->utime = utime; // not correct could use GetTime () but doesnt really make sense

        // no classification data for navteq data
        // cout << p.GetClassification().GetClassName() << endl;

        i++;
    }

    if (i != lrc->num_lrl)
        ERROR ("Unexpected number of laser returns");
    
    char lrc_fname[PATH_MAX]={0};    
    snprintf (lrc_fname, sizeof lrc_fname, "%s/%ld.lrc", scan_dir, lrc->utime);
    int32_t ret = LCMU_FWRITE (lrc_fname, lrc, perllcm_velodyne_laser_return_collection_t);
    if (ret < 0)
        ERROR ("couldn't write %s to disk!", lrc_fname);
        
    perllcm_velodyne_laser_return_collection_t_destroy (lrc);
    free (mask);

}

void
PVNMapBuilder::run_build_seed_map ()
{    
    cout << "[seed map]\tLoading .las files in directory: " << las_dir << endl;
    
    // open directory
    DIR *d;
    struct dirent *dir;
    d = opendir (las_dir);
    if (d == NULL) {
        ERROR ("Could not open .las file directory: %s", las_dir);
        return;
    }

    char filename[PATH_MAX];
    sprintf (filename, "%s/isam.graph", las_dir);
    ofstream fgraph;
    fgraph.open (filename);
    
    // loop over all files in directory
    while ((dir = readdir (d))) {
     
        int len = strlen (dir->d_name);
        if (strcmp (dir->d_name, "." ) == 0 || strcmp (dir->d_name, ".." ) == 0
            || dir->d_type == DT_DIR || strcmp (&(dir->d_name[len-4]), ".las") != 0) {
            cout << "[seed map]\tSkipping: " << dir->d_name << endl;
            continue;
        }
        
        cout << "[seed map]\tProcessing: " << dir->d_name << endl;
        sprintf (filename, "%s/%s", las_dir, dir->d_name);

        process_las (filename, &fgraph);
    
    }
    
    fgraph.close();
}

