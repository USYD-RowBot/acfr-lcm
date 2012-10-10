#include <sys/stat.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "tmpcor.h"


void
PVNTempCor::run_preprocess (char *dir) {
    
    // parse in the meta data
    std::string dir_str (dir);
    int ncams = count_num_cams (dir);
    vector<int64_t> utimes = read_utimes (dir);
    cout << "[webcam]\tRead Metadata in: " << dir << endl;
    cout << "[webcam]\tNum Cams = " << ncams << ", Num Samples = " << utimes.size() << endl;
    
    // loop over each utime
    for (size_t ii_utime=0; ii_utime<utimes.size(); ii_utime++) {
        
        int64_t utime = utimes[ii_utime];
        
        if (ii_utime % every_nth)
            continue;
        
        // read conditions for this utime
        std::string conds_file = dir_str + "/conds/" + boost::lexical_cast<std::string>(utime) + ".cond";
        perllcm::pvn_conditions_t conds = read_conditions (conds_file);
        std::string conds_file_out = dir_str + "/lcm_conds/" + boost::lexical_cast<std::string>(utime) + ".cond";
        pvnu_lcm_fwrite <perllcm::pvn_conditions_t> (conds_file_out.c_str(), conds);
    
        
    }

}