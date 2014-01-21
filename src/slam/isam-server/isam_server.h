#ifndef __ISAM_SERVER_H__
#define __ISAM_SERVER_H__

#include <string>
#include <vector>
#include <map>
#include <queue>
#include <iostream>
#include <stdexcept>
#include <sys/param.h>

#include <glib.h>
#include <isam/isam.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_isam_add_node_t.h"
#include "perls-lcmtypes/perllcm_isam_vlink_t.h"
#include "perls-lcmtypes/perllcm_isam_glc_factor_t.h"
#include "perls-lcmtypes/perllcm_isam_return_state_t.h"
#include "perls-lcmtypes/perllcm_isam_request_state_t.h"
#include "perls-lcmtypes/perllcm_isam_cmd_t.h"
#include "perls-lcmtypes/perllcm_isam_return_state_t.h"
#include "perls-lcmtypes/perllcm_isam_graph_vis_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#include "perls-common/getopt.h"

#include "perls-isam/gparser.h"
#include "perls-isam/isam_util.h"
#include "perls-isam/user_factors.h"

// threads
#include "interf_thread.h"
#include "fileio_thread.h"


using namespace std;
using namespace isam;

typedef enum _verbose_level_t
{
    VERB_QUITE          = 0,  // quite mode
    VERB_UPDATE         = 1,  // display update status
    VERB_FACTOR_NODE    = 2,  // display factor/node add status
    VERB_STATE          = 4,  // display state request/return status
    VERB_LCM            = 8,  // display lcm interface level
    VERB_ISAM           = 16, // print isam stat (#nodes/#
} verbose_level_t;


typedef struct _fileio_tdata_t fileio_tdata_t;
struct _fileio_tdata_t
{
    lcm_t *lcm;
    const char *input_fname;
    GAsyncQueue *gq;
    bool *active;
    int  verbose;
};

typedef struct _interf_tdata_t interf_tdata_t;
struct _interf_tdata_t
{
    lcm_t *lcm;
    GAsyncQueue *gq;
    const char *chname_add_node;
    const char *chname_add_node_ack;
    const char *chname_return_st;
    const char *chname_request_st;
    const char *chname_vlink;
    const char *chname_plink;
    const char *chname_init_isam;
    const char *chname_cmd;
    bool *done; //used to signal thread kill
    int  verbose;
};


class isamServer {
public:

    isamServer ();
    isamServer (int argc, char *argv[]);
    ~isamServer();
    
    void run ();
    
    bool done;

    int m_verbose;      // verbose level from gopt. see "verbose_level_t"

private:

    lcm_t *m_lcm;                       
    
    // flags
    bool m_active;
    bool filein_on;
  
    // thread id's
    GThread *tid_interf_thread;
    GThread *tid_fileio_thread;
  
    // queues
    GAsyncQueue *m_gq; // glib asynchronous queue for lcm messages
    
    // access to flags
    bool is_active () {return m_active;}
    void activate () {m_active = true;} //@TODO, this should be a command message
 
    void fwrite_graph (const char *mucov_fname, const char *pc_fname);
    void save_cputime (const char *cputime_fname);
    
    // channel names
    const char *m_chname_add_node;
    const char *m_chname_add_node_ack;
    const char *m_chname_return_st;
    const char *m_chname_request_st;
    const char *m_chname_vlink;
    const char *m_chname_vis;
    const char *m_chname_init_isam;
    const char *m_chname_cmd; 
    
    const char *input_fname;
    
    void add_node          (const perllcm_isam_add_node_t *msg); 
    void add_node_pose3d   (const perllcm_isam_add_node_t *msg); 
    void add_node_plane3d  (const perllcm_isam_add_node_t *msg); 
    void add_plane3d_prior (const perllcm_isam_vlink_t *vlink);
    void add_prior         (const perllcm_isam_vlink_t *vlink);
    int  add_factor_pose3d (Pose3d_Node *node1, Pose3d_Node *node2, const perllcm_isam_vlink_t *vlink);
    int  add_factor_plane3d (Pose3d_Node *node1, Plane3d_Node *node2, const perllcm_isam_vlink_t *vlink);
    int  add_factor     (const perllcm_isam_vlink_t *vlink);
    void add_partial    (const perllcm_isam_vlink_t *vlink);
    
    void process_queues (void);
    void process_cmd (perllcm_isam_cmd_t *cmd);
    void process_state_request (perllcm_isam_request_state_t *strq);
    perllcm_isam_return_state_t * return_states (std::map<int64_t, isam::Pose3d_Node*> &nodes, std::vector<int64_t> rqlist, int rqtype);

    // local copy of the graph
    isam::Slam     m_slam;
    Pose3d_Factor *m_originPrior;
    Pose3d_Node   *m_origin;    
    Pose3d_Node   *m_last_node; // The last node added to the estimation
    
    std::map<int64_t, isam::Pose3d_Node*> m_pose3d_nodes;           // Nodes indexed by id (id can be utime)
    std::map<int64_t, isam::Plane3d_Node*> m_plane3d_nodes;         // Nodes indexed by id (id can be utime)
    std::map<int64_t, isam::Point3d_Node*> m_point3d_nodes;         // Nodes indexed by id (id can be utime)
    /* not supported in current version of isam in third-party */
    /* std::map<int64_t, isam::Anchor3d_Node*> m_anchor3d_nodes;       // Nodes indexed by id (id can be utime) */
        
    // vis
    void pub_vis (void);
    
    std::vector<int64_t> m_links_i;
    std::vector<int64_t> m_links_j;
    std::vector<int32_t> m_links_sensor_id;         // see perllcm_isam_vlink_t
    double m_cov_recent[6*6];
    
    void
    update_vis_factors (int64_t id1, int64_t id2, int32_t sensor_id) {
        m_links_i.push_back (id1);
        m_links_j.push_back (id2);
        m_links_sensor_id.push_back (sensor_id); // see perllcm_isam_vlink_t
    }

    // config and options
    getopt_t    *m_gopt;
    BotParam    *m_param;
    
    // batch processing options
    char *mucov_fname;
    char *graph_fname;
    char *pc_fname;

    // utimelist for valid nodes
    GList *utimelist;

    // post processing
    bool                        m_save_cputime;
    std::map<int64_t, int64_t>  m_cputime_dat;

};


#endif //__ISAM_SERVER_H__
