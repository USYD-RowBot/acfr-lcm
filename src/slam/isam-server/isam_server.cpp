#include <iostream>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/glib_util.h"
#include "perls-common/lcm_util.h"

#include "isam_server.h"

using namespace Eigen;

#define HAUV_PROJECT 0

static void
gq_free (gpointer element)
{
    queue_element_t *qe = (queue_element_t *) element;
    
    switch (qe->type) {
        case QE_VLINK:
            perllcm_isam_vlink_t_destroy ((perllcm_isam_vlink_t *) qe->msg);        
        break;
        case QE_NODE:
            perllcm_isam_add_node_t_destroy ((perllcm_isam_add_node_t *) qe->msg);
        break;
        case QE_CMD:
            perllcm_isam_cmd_t_destroy ((perllcm_isam_cmd_t *) qe->msg);
        break;
        case QE_STATE:
            perllcm_isam_request_state_t_destroy ((perllcm_isam_request_state_t *) qe->msg);        
        break;
        case QE_INIT:
            perllcm_isam_init_t_destroy ((perllcm_isam_init_t *) qe->msg);
        break;
        case QE_GLC:
            perllcm_isam_glc_factor_t_destroy ((perllcm_isam_glc_factor_t *) qe->msg);
        break;
        default:
            ERROR ("Unknown queue element type !");
    }
    free (qe);
}

static gint
gq_sort (gconstpointer a, gconstpointer b, gpointer user_data)
{
    const queue_element_t *A = (queue_element_t *)a;
    const queue_element_t *B = (queue_element_t *)b;
    
    if (A->utime > B->utime)
        return 1;
    else if (A->utime == B->utime) {
        if (A->type != B->type) {
            if (A->type == QE_NODE && (B->type == QE_VLINK || B->type == QE_GLC))// nodes before factors (very important)
                return -1;
            else if (A->type == QE_VLINK && (B->type == QE_NODE || B->type == QE_GLC))
                return 1;
            else if (A->type == QE_STATE) { // state requests last 
                if (B->type == QE_CMD)
                    return -1;
                else
                    return 1;
            }
            else if (B->type == QE_STATE) {// state requests last
                if (A->type == QE_CMD)
                    return 1;
                else
                    return -1;
            }
            else
                return 0; // unspecified case
        } 
        else
            return 0;
    } 
    else
        return -1;
}

isamServer::isamServer (int argc, char *argv[]) 
{
    done = 0;

    // lcm
    m_lcm = lcm_create (NULL);
    if (!m_lcm)
        throw runtime_error("[server]\tERROR: NULL lcm init.");

    // parse our options
    m_gopt = getopt_create ();
    getopt_add_description (m_gopt, "iSAM Server:\n./isam-server -f sample.graph -s or ./isam-server");
    getopt_add_help    (m_gopt, NULL);
    getopt_add_string  (m_gopt, 'f', "fname",      "",             "input isam graph file");
    getopt_add_int     (m_gopt, 'v', "verbose",    "1",            "verbose level (0~5)");

    if (!getopt_parse (m_gopt, argc, argv, 0)) {
        getopt_do_usage (m_gopt, "FILE");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (m_gopt, "help")) {
        getopt_do_usage (m_gopt, "FILE");
        exit (EXIT_SUCCESS);
    }
    
    // verbose level
    int _verbose_level = getopt_get_int (m_gopt, "verbose");
    switch (_verbose_level) {
        case 0: m_verbose = VERB_QUITE; break;
        case 1: m_verbose = VERB_UPDATE; break;
        case 2: m_verbose = VERB_UPDATE | VERB_FACTOR_NODE; break;
        case 3: m_verbose = VERB_UPDATE | VERB_FACTOR_NODE | VERB_STATE; break;
        case 4: m_verbose = VERB_UPDATE | VERB_FACTOR_NODE | VERB_STATE | VERB_LCM; break;
        case 5: m_verbose = VERB_UPDATE | VERB_FACTOR_NODE | VERB_STATE | VERB_LCM | VERB_ISAM; break;
        default: m_verbose = VERB_QUITE; break;
    }

    // do you want to load from a file?
    input_fname = getopt_get_string (m_gopt, "fname");
    if (input_fname[0] == '\0' || input_fname == NULL)
        filein_on = false;
    else
        filein_on = true;

    // parse our config file
    m_param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!m_param) {
      ERROR ("Could not get config parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
      exit(EXIT_FAILURE);
    }
    
    // load properties from config file
    Properties prop = isamu_init_slam (m_param);

    m_slam.set_properties(prop);
    std::cout << "[server]\tSet properties from config file" << std::endl;

    // create glib asynchronous queue for factors and nodes to be processed
    m_gq = g_async_queue_new_full (&gq_free);

    // assign channel name
    m_chname_add_node      = bot_param_get_str_or_fail (m_param, "isamServer.lcm_channels.ADD_NODE_CHANNEL");
    m_chname_add_node_ack  = bot_param_get_str_or_fail (m_param, "isamServer.lcm_channels.ADD_NODE_ACK_CHANNEL");
    m_chname_return_st     = bot_param_get_str_or_fail (m_param, "isamServer.lcm_channels.RETURN_ST_CHANNEL");
    m_chname_request_st    = bot_param_get_str_or_fail (m_param, "isamServer.lcm_channels.REQUEST_ST_CHANNEL");
    m_chname_vlink         = bot_param_get_str_or_fail (m_param, "isamServer.lcm_channels.VLINK_CHANNEL");
    m_chname_init_isam     = bot_param_get_str_or_fail (m_param, "isamServer.lcm_channels.INIT_ISAM_CHANNEL");
    m_chname_cmd           = bot_param_get_str_or_fail (m_param, "isamServer.lcm_channels.CMD_CHANNEL");
    m_chname_vis           = bot_param_get_str_or_fail (m_param, "isamServer.lcm_channels.VIS_CHANNEL");

    // post processing
    if (-1==bot_param_get_boolean (m_param, "isamServer.post_process.save_cputime", (int*) &m_save_cputime))
        m_save_cputime = false;
        
    mucov_fname = NULL;
    graph_fname = NULL;
    pc_fname = NULL;
    utimelist = NULL;
    if (0!=bot_param_get_str (m_param, "isamServer.post_process.mucov_fname", &mucov_fname))
        mucov_fname = NULL;
    if (0!=bot_param_get_str (m_param, "isamServer.post_process.graph_fname", &graph_fname))
        graph_fname = NULL;
    if (0!=bot_param_get_str (m_param, "isamServer.post_process.pc_fname", &pc_fname))
        pc_fname = NULL;

    memset (m_cov_recent, 0, 6*6*sizeof (double));

    std::cout << "[init]\t\tinitialized." << std::endl;
}

isamServer::~isamServer ()
{
    // free memory in reverse order of allocation
    getopt_destroy (m_gopt);

    // remove destory vlink async queue
    g_async_queue_unref (m_gq);    

    //g_slist_foreach (utimelist, &glist_destroyer, NULL);
    
    lcm_destroy (m_lcm);
}


void
isamServer::run (void) {
    
    interf_tdata_t interf_tdata = {0};
    interf_tdata.lcm = m_lcm;
    interf_tdata.gq = m_gq;
    interf_tdata.chname_add_node = m_chname_add_node;
    interf_tdata.chname_add_node_ack = m_chname_add_node_ack;
    interf_tdata.chname_return_st = m_chname_return_st;
    interf_tdata.chname_request_st = m_chname_request_st;
    interf_tdata.chname_vlink = m_chname_vlink;
    interf_tdata.chname_init_isam = m_chname_init_isam;
    interf_tdata.chname_cmd = m_chname_cmd;
    interf_tdata.done = &done;
    interf_tdata.verbose = m_verbose;
    
    tid_interf_thread = g_thread_create (&interf_thread, &interf_tdata, 1, NULL);

    m_active = true;
    if (filein_on) {
        m_active = false;
        
        fileio_tdata_t fileio_tdata = {0};
        fileio_tdata.lcm = m_lcm;
        fileio_tdata.input_fname = input_fname;
        fileio_tdata.gq = m_gq;
        fileio_tdata.active = &m_active;
        fileio_tdata.verbose = m_verbose;
        
        tid_fileio_thread = g_thread_create (&fileio_thread, &fileio_tdata, 1, NULL);        
    }    
        
    int64_t last_vis_pub = 0;
    while (!done) {
        
        if (is_active ()) {
            process_queues ();
        }
        
        if (abs (last_vis_pub - timestamp_now ()) > 1e6) {
            pub_vis ();
            last_vis_pub = timestamp_now ();
        }
    }

    // wait for children threads to die
    if (tid_interf_thread)
        g_thread_join (tid_interf_thread);

    if (filein_on && tid_fileio_thread)
        g_thread_join (tid_fileio_thread);
}

void
isamServer::fwrite_graph (const char *mucov_fname, const char *pc_fname) 
{
    size_t n_nodes = m_pose3d_nodes.size();

    perllcm_pose3d_collection_t *pc = (perllcm_pose3d_collection_t*) calloc (1, sizeof (*pc));
    pc->utime = 0;
    pc->npose = n_nodes;
    pc->pose = (perllcm_pose3d_t*) malloc (n_nodes * sizeof(perllcm_pose3d_t));

    std::list< std::list< isam::Node* > > covrq_nodes;  

    std::map<int64_t, isam::Pose3d_Node*>::iterator it_node;
    int i = 0;
    for (it_node = m_pose3d_nodes.begin(); it_node != m_pose3d_nodes.end(); it_node++) {
        Pose3d_Node* node = it_node->second; // the value in the key value pair
        perllcm_pose3d_t *p = &pc->pose[i];
        
        if (NULL != node) {
            p->utime = it_node->first;
            p->mu[0] = node->value().x();
            p->mu[1] = node->value().y();
            p->mu[2] = node->value().z();
            p->mu[5] = node->value().yaw();
            p->mu[4] = node->value().pitch();
            p->mu[3] = node->value().roll();

            std::list< isam::Node* > nodes;            
            nodes.push_back(node);
            covrq_nodes.push_back(nodes);
        }
        i++;
    }

    // request block diagonal covariance
    std::list<MatrixXd> m = m_slam.covariances().marginal(covrq_nodes);

    int node_idx = 0;
    for (std::list<MatrixXd>::iterator it = m.begin(); it != m.end(); it++) {
        int k = 0; 
        perllcm_pose3d_t *p = &pc->pose[node_idx];
        MatrixXd a = *it;
        int row, col;
        for (int i=0;i<a.rows(); i++) {
            for (int j=0; j<a.cols(); j++) {
                row = i, col = j;
                if (i == 3) row = 5;
                if (i == 5) row = 3;
                if (j == 3) col = 5;
                if (j == 5) col = 3;
                
                p->Sigma[k++] = a(row,col);
            }
        }
        node_idx++;
    }   

    // write into the disk

    // (1) fwrite binary file of pose3d collection
    if (pc_fname)
        LCMU_FWRITE (pc_fname, pc, perllcm_pose3d_collection_t);

    // (2) readable textfile of mu and covariance
    if (mucov_fname)
        isamu_state_fprintf (mucov_fname, pc, utimelist);

    // clean up
    perllcm_pose3d_collection_t_destroy (pc);
}

void
isamServer::save_cputime (const char *cputime_fname)
{
    if (m_save_cputime)
        isamu_cputime_fprintf (cputime_fname, m_cputime_dat);
}

void
isamServer::process_queues (void)
{    
    int64_t timeout = 1e3;

    // process queue
    bool quit = 0;
    bool do_update = 0;
    while (!quit) {
        GTimeVal end_time;
        g_get_current_time (&end_time);
        g_time_val_add (&end_time, timeout);
        g_async_queue_sort (m_gq, &gq_sort, NULL);
        queue_element_t *qe = (queue_element_t *)g_async_queue_timed_pop (m_gq, &end_time);
    
        if (qe != NULL) {
                    
            switch (qe->type) {
            case QE_VLINK: {
                perllcm_isam_vlink_t *vlink = (perllcm_isam_vlink_t *)qe->msg;
                //std::cout << "in the queue (factor) = " << qe->utime << std::endl;

                if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_PRIOR)
                    add_prior (vlink);
                else if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_PLANE3D_PRIOR)
                    add_plane3d_prior (vlink);
                else if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_ZPR_PARTIAL
                         || vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_XYZ_PARTIAL
                         || vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_H_PARTIAL
                         || vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_RP_PARTIAL
                         || vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_Z_PARTIAL
                         || vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_XY_PARTIAL) {
                    add_partial (vlink);
                }
                else {
                    add_factor (vlink);
                    // vis
                    update_vis_factors (vlink->id1, vlink->id2, vlink->sensor_id);
                }
                do_update |= 1;

                break;
            }
            case QE_GLC: {
                perllcm_isam_glc_factor_t *glc_factor = (perllcm_isam_glc_factor_t *)qe->msg;
                
                vector<isam::Pose3d_Node*> p3d_nodes;
                for (int i=0; i<glc_factor->np; i++) 
                    p3d_nodes.push_back (dynamic_cast<Pose3d_Node*>(m_pose3d_nodes[glc_factor->ids[i]]));
                
                isamu_add_glc_factor (glc_factor, &m_slam, p3d_nodes);
                
                if (m_verbose & VERB_FACTOR_NODE) {
                    std::cout << "[server]\tglc factor between ";
                    for (int i=0; i<glc_factor->np; i++)
                        std::cout << glc_factor->ids[i] << " ";
                    std::cout << " added." << std::endl;
                }
                
                // for each pair update vis
                for (int i=0; i<glc_factor->np; i++) {
                    for (int j=i+1; j<glc_factor->np; j++) {
                        update_vis_factors (glc_factor->ids[i], glc_factor->ids[j], PERLLCM_ISAM_VLINK_T_SENSOR_GLC);
                    }
                }
            
                do_update |= 1;

                break;
            }
            case QE_NODE:
                add_node ((perllcm_isam_add_node_t *) qe->msg); 
                break;
            case QE_CMD:
                process_cmd ((perllcm_isam_cmd_t *) qe->msg);
                break;
            case QE_STATE: {
                //std::cout << "in the queue (state) = " << qe->utime << std::endl;

                if (do_update) {
                    int64_t tic = timestamp_now ();
                    if (m_verbose & VERB_UPDATE) std::cout << "[server]\tUpdating graph started w/ " << m_pose3d_nodes.size() << " nodes." << std::endl;
                    m_slam.update();
                    if (m_verbose & VERB_UPDATE) std::cout << "[server]\tUpdating graph done (" << (timestamp_now () - tic)/1000000.0 << " sec). " << std::endl;
                    do_update = 0;
                }
                //@TODO figure out how to make a local copy of m_slam and kick off covariance recovery in a seperate thread

                process_state_request ((perllcm_isam_request_state_t *) qe->msg);      
                break;
            }
            case QE_INIT: {
                isam::Properties prop = isamu_init_slam ((perllcm_isam_init_t *) qe->msg);
                m_slam.set_properties(prop);
                break;
            }
            default:
                ERROR ("Unknown queue element type !");
            }
            
            // pop then clean
            gq_free (qe);
        }
        else
            quit = 1;
    }

    if (do_update) {
        int64_t tic = timestamp_now ();
        if (m_verbose & VERB_UPDATE) std::cout << "[server]\tUpdating graph started w/ " << m_pose3d_nodes.size() << " nodes." << std::endl;
        //sleep(10);
        m_slam.update();
        if (m_verbose & VERB_UPDATE) std::cout << "[server]\tUpdating graph done (" << (timestamp_now () - tic)/1000000.0 << " sec). " << std::endl;
        do_update = 0;
    }
}

void
isamServer::process_cmd (perllcm_isam_cmd_t *cmd)
{    
    // on save
    if (cmd->mode == PERLLCM_ISAM_CMD_T_MODE_SAVE) {
        const char *filepath = cmd->savepath;
        // write matlab readable file
        char *mucov_fullpath = NULL;
        char *pc_fullpath = NULL;
        if (mucov_fname) {
            mucov_fullpath = (char*) malloc (MAXPATHLEN * sizeof(char));
            snprintf (mucov_fullpath, strlen (filepath)+strlen(mucov_fname)+2, "%s/%s\n", filepath, mucov_fname);
        }
        if (pc_fname) {
            pc_fullpath = (char*) malloc (MAXPATHLEN * sizeof(char));
            snprintf (pc_fullpath, strlen (filepath)+strlen (pc_fname)+2, "%s/%s\n", filepath, pc_fname);    
        }

        if (mucov_fname && pc_fname) {
            std::cout << "[server]\t\tsaving state ...";
            fwrite_graph (mucov_fullpath, pc_fullpath);
            std::cout << "done. "<< std::endl;
        }

        // write isam original graph file
        char *graph_fullpath = NULL;
        if (graph_fname) {
            graph_fullpath = (char*) malloc (MAXPATHLEN * sizeof(char));
            snprintf (graph_fullpath, strlen (filepath)+strlen(graph_fname)+2, "%s/%s\n", filepath, graph_fname);

            std::cout << "[server]\t\tsaving isam graph to: " << graph_fullpath << "...." ;
            m_slam.save (graph_fullpath);
            std::cout << "done. "<< std::endl;
        }

        // save into disk
        //char cputime_fullpath[MAXPATHLEN];
        //snprintf (cputime_fullpath, strlen (filepath)+13, "%s/cputime.dat\n", filepath);
        //std::cout << "[server]\tsaving cputime ...";
        //save_cputime (cputime_fullpath);
        //std::cout << "done. "<< std::endl;
    }

    // load a graph
    if (cmd->mode == PERLLCM_ISAM_CMD_T_MODE_LOAD) {

        m_active = false;
        // graph parser
        gParser *gp = new gParser (m_lcm, cmd->graphfile);
        if (!gp)
            std::cout << "[isam loading]\tError: init_gp()" << std::endl;
        
        gp->set_gq (m_gq);
        gp->fread_graph ();        
        std::cout << "[isam loading]\tDone parsing: " << g_async_queue_length (m_gq) << " nodes and vlinks." << std::endl;

        delete gp;

        m_active = true;
    }

    // on forced batch
    if (cmd->mode == PERLLCM_ISAM_CMD_T_MODE_BATCH) {
        int64_t tic = timestamp_now ();
        if (m_verbose & VERB_UPDATE) std::cout << "[server]\tBatch updating graph started." << std::endl;
        m_slam.batch_optimization();
        if (m_verbose & VERB_UPDATE) std::cout << "[server]\tBatch updating graph (" << (timestamp_now () - tic)/1000000.0 << " sec) done." << std::endl;
        if (m_verbose & VERB_ISAM) m_slam.print_stats ();
    }

}


void
isamServer::process_state_request (perllcm_isam_request_state_t *strq) 
{

    if (strq->n < 1)
        return;

    std::vector<int64_t> valid_rqlist;

    // check valid m_pose3d_nodes
    for (int i=0; i<strq->n; i++) {
        int64_t v = strq->variables[i];
        Pose3d_Node* node = (m_pose3d_nodes.find(v)==m_pose3d_nodes.end())?NULL:m_pose3d_nodes[v];
      
        if (node)
            valid_rqlist.push_back (v);
    }
    size_t valid_n = valid_rqlist.size();

    if (valid_n < 1)
        return;

    if ((m_verbose & VERB_STATE) && (valid_rqlist.size() > 1))
        std::cout << "[server]\tRerturning state started for utime " << strq->utime << " with id " <<  valid_rqlist.at(valid_rqlist.size()-1) << std::endl;

    perllcm_isam_return_state_t* strt = return_states (m_pose3d_nodes, valid_rqlist, strq->state_type);

    if (strt) {
        strt->requester = strq->requester;
        perllcm_isam_return_state_t_publish (m_lcm, m_chname_return_st, strt);
        perllcm_isam_return_state_t_destroy (strt);
    }
}

void
isamServer::add_node (const perllcm_isam_add_node_t *msg)
{
    switch (msg->node_type) {
    case PERLLCM_ISAM_ADD_NODE_T_NODE_POSE3D:
        add_node_pose3d (msg);
        break;
    case PERLLCM_ISAM_ADD_NODE_T_NODE_PLANE3D:
        add_node_plane3d (msg);
        break;
    }
}

void
isamServer::add_node_pose3d (const perllcm_isam_add_node_t *msg) 
{
    Pose3d_Node *new_node = isamu_add_node_pose3d (msg, &m_slam);

    m_pose3d_nodes[msg->id] = new_node;

    if (msg->utime != 0)
        utimelist = g_list_append (utimelist, gu_dup (&msg->utime, sizeof msg->utime));
    
    if (m_verbose & VERB_FACTOR_NODE) 
        std::cout << "[server]\tnode " << msg->id << " added." << std::endl;

    m_last_node = new_node;
}

void
isamServer::add_node_plane3d (const perllcm_isam_add_node_t *msg) 
{
    Plane3d_Node *new_node = isamu_add_node_plane3d (msg, &m_slam);

    m_plane3d_nodes[msg->id] = new_node;

    if (msg->utime != 0)
        utimelist = g_list_append (utimelist, gu_dup (&msg->utime, sizeof msg->utime));
    
    if (m_verbose & VERB_FACTOR_NODE) 
        std::cout << "[server]\tnode " << msg->id << " added." << std::endl;

    /* m_last_node = new_node; */
}

void
isamServer::add_plane3d_prior (const perllcm_isam_vlink_t *vlink)
{
    // parse from vlink msg
    isam::Plane3d plane = isamu_van2isam_plane3d (vlink->z);
    Eigen::MatrixXd sqrtinf0 = isamu_van2isam_sqrtinf3d  (vlink->R);

    // look for an existing node
    Plane3d_Node* new_plane_node;
    if (m_plane3d_nodes.find(vlink->id2) == m_plane3d_nodes.end()) {
        new_plane_node = new Plane3d_Node();
        
        m_slam.add_node(new_plane_node);
        
        m_plane3d_nodes[vlink->id2] = new_plane_node;
        utimelist = g_list_append (utimelist, gu_dup (&vlink->id2, sizeof vlink->id2));
    }
    else
        new_plane_node = m_plane3d_nodes[vlink->id2];
    
    // generate a factor
    Plane3d_Factor *factor = new Plane3d_Factor(new_plane_node, plane, SqrtInformation(sqrtinf0));

    m_slam.add_factor(factor);

    if (m_verbose & VERB_FACTOR_NODE)
        std::cout << "\n[server]\tadded prior to plane: node " << vlink->id2 << std::endl;
}

void
isamServer::add_prior (const perllcm_isam_vlink_t *vlink)
{
    // parse from vlink msg
    isam::Pose3d Pose0 = isamu_van2isam_pose3d (vlink->z);
    Eigen::MatrixXd sqrtinf0 = isamu_van2isam_sqrtinf3d  (vlink->R);

    // look for an existing node
    Pose3d_Node* new_pose_node;
    if (m_pose3d_nodes.find(vlink->id2) == m_pose3d_nodes.end()) {
        new_pose_node = new Pose3d_Node();
        
        m_slam.add_node(new_pose_node);
        
        m_pose3d_nodes[vlink->id2] = new_pose_node;
        utimelist = g_list_append (utimelist, gu_dup (&vlink->id2, sizeof vlink->id2));
    }
    else
        new_pose_node = m_pose3d_nodes[vlink->id2];
    
    // generate a factor
    m_originPrior = new Pose3d_Factor(new_pose_node, Pose0, SqrtInformation(sqrtinf0));

    // update map
    m_slam.add_factor(m_originPrior);

    // update stats
    m_origin = new_pose_node;
    m_last_node = new_pose_node;

    if (m_verbose & VERB_FACTOR_NODE)
        std::cout << "\n[server]\tinitialized with prior: node " << vlink->id2 << std::endl;
}

/* Add factors connecting two pose3d nodes (could be odo, sonar, camera, etc...) */
int
isamServer::add_factor_pose3d (Pose3d_Node *node1, Pose3d_Node *node2, const perllcm_isam_vlink_t *vlink)
{
    // 1st node shouldn't be null
    if (node1 == NULL) {
        char errorText[255];
        sprintf(errorText, "ERROR: [isamserver.cpp] first node null in adding factor %lu",vlink->id2); // # links in the queue
        throw runtime_error (errorText);
    }

    // 2nd node could be null for odometry constraints
    int ack_on = 0;
    if (node2 == NULL) {
        node2 = new Pose3d_Node();
        m_pose3d_nodes[vlink->id2] = node2;
        utimelist = g_list_append (utimelist, gu_dup (&vlink->id2, sizeof vlink->id2));
        m_slam.add_node (node2);
        m_last_node = node2;

        ack_on = 1;
    }

    isamu_add_factor (vlink, &m_slam, node1, node2);

    return ack_on;
}

int
isamServer::add_factor_plane3d (Pose3d_Node *node1, Plane3d_Node *node2, const perllcm_isam_vlink_t *vlink)
{
    /* 1st node shouldn't be null */
    if (node1 == NULL) {
        char errorText[255];
        sprintf(errorText, "ERROR: [isamserver.cpp] first node null in adding factor %lu",vlink->id2); // # links in the queue
        throw runtime_error (errorText);
    }

    /* 2nd node can be null */
    int ack_on = 0;
    if (node2 == NULL) {
        node2 = new Plane3d_Node();
        m_plane3d_nodes[vlink->id2] = node2;
        utimelist = g_list_append (utimelist, gu_dup (&vlink->id2, sizeof vlink->id2));
        m_slam.add_node (node2);
        /* m_last_ndoe = node2; */

        ack_on = 1;
    }

    isamu_add_factor (vlink, &m_slam, node1, node2);

    return ack_on;
}

/* Switch yard for adding factors */
int
isamServer::add_factor (const perllcm_isam_vlink_t *vlink) 
{    
    int ack_on = 0;
    int64_t id1 = vlink->id1;
    int64_t id2 = vlink->id2;

    if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_POSE3D || 
        vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_POSE3DB || 
        vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_SONAR2D) {
        Pose3d_Node* node1 = (m_pose3d_nodes.find(id1)==m_pose3d_nodes.end())?NULL:m_pose3d_nodes[id1];
        Pose3d_Node* node2 = (m_pose3d_nodes.find(id2)==m_pose3d_nodes.end())?NULL:m_pose3d_nodes[id2];
        ack_on = add_factor_pose3d (node1, node2, vlink);
    }
    else if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_PLANE3D) {
        Pose3d_Node* node1 = (m_pose3d_nodes.find(id1)==m_pose3d_nodes.end())?NULL:m_pose3d_nodes[id1];
        Plane3d_Node* node2 = (m_plane3d_nodes.find(id2)==m_plane3d_nodes.end())?NULL:m_plane3d_nodes[id2];
        ack_on = add_factor_plane3d (node1, node2, vlink);
    }

    // verbose
    if (m_verbose & VERB_FACTOR_NODE) {
        if (vlink->sensor_id == PERLLCM_ISAM_VLINK_T_SENSOR_ODOMETRY)
            std::cout << "[server]\todo factor between " << id1 << " and " << id2 << " added." << std::endl;
        else if (vlink->sensor_id == PERLLCM_ISAM_VLINK_T_SENSOR_CAMERA)
            std::cout << "[server]\tcam factor between " << id1 << " and " << id2 << " added." << std::endl;
        else if (vlink->sensor_id == PERLLCM_ISAM_VLINK_T_SENSOR_LASER)
            std::cout << "[server]\tlaser factor between " << id1 << " and " << id2 << " added." << std::endl;
        else if (vlink->sensor_id == PERLLCM_ISAM_VLINK_T_SENSOR_SONAR)
            std::cout << "[server]\tsonar factor between " << id1 << " and " << id2 << " added." << std::endl;
    }
    return ack_on;
}

void
isamServer::add_partial (const perllcm_isam_vlink_t *vlink)
{
    int64_t id = vlink->id2;

    Pose3d_Node* node = (m_pose3d_nodes.find(id)==m_pose3d_nodes.end())?NULL:m_pose3d_nodes[id];

    // node shouldn't be null
    if (node == NULL) {
        char errorText[255];
        sprintf(errorText, "ERROR: [isamserver.cpp] node null in adding partial factor %lu",vlink->id2); // # links in the queue
        throw runtime_error(errorText);
    }
    
    isamu_add_partial (vlink, &m_slam, node, m_verbose & VERB_FACTOR_NODE);
   
}

perllcm_isam_return_state_t* 
isamServer::return_states (std::map<int64_t, isam::Pose3d_Node*> &nodes, std::vector<int64_t> valid_rqlist, int state_type)
{
    // process the queue so that we are caught up before we try to return states
    int64_t tic = timestamp_now ();

    perllcm_isam_return_state_t* strt = (perllcm_isam_return_state_t *)calloc(1, sizeof (perllcm_isam_return_state_t));
    strt->utime = timestamp_now ();
    strt->n = 0;
    strt->mu = NULL;
    strt->timestamps = NULL;
    strt->m = 0;
    strt->covariance = NULL;
    
    strt->state_type = state_type;

    // populate utimelist
    size_t valid_n = valid_rqlist.size();
    strt->n = valid_n;
    strt->timestamps = (int64_t *) calloc (strt->n, sizeof (int64_t));
    memcpy (strt->timestamps, &(valid_rqlist[0]), strt->n * sizeof (int64_t));

    // cov request list
    std::list< isam::Node* >              nodes_fullcov_rq;
    std::list< std::list< isam::Node* > > nodes_blockcov_rq;
    std::list<std::pair<Node*, Node*> >   nodes_rightcol_rq;
    int64_t time_recent = valid_rqlist.back();
    Pose3d_Node* node_recent = (nodes.find(time_recent)==nodes.end())?NULL:nodes[time_recent];

    // memory alloc for mu
    if (state_type & PERLLCM_ISAM_REQUEST_STATE_T_POSE) {
        strt->k = valid_n;
        strt->mu = (double **)malloc (valid_n * sizeof (double*));
    }

    // populate state for valid nodes
    for (size_t i=0; i<valid_n; i++) {
        int64_t v = valid_rqlist.at(i);
        Pose3d_Node* node = (nodes.find(v)==nodes.end())?NULL:nodes[v];

        if (node) { // fail-safe
            if (state_type & PERLLCM_ISAM_REQUEST_STATE_T_POSE) {
                strt->mu[i] = (double *)malloc (6 * sizeof (double));
                strt->mu[i][0] = node->value().x();
                strt->mu[i][1] = node->value().y();
                strt->mu[i][2] = node->value().z();
                strt->mu[i][5] = node->value().yaw();
                strt->mu[i][4] = node->value().pitch();
                strt->mu[i][3] = node->value().roll();
            }

            // possible request list
            if (state_type & PERLLCM_ISAM_REQUEST_STATE_T_COV_FULL)
                nodes_fullcov_rq.push_back (node);
            else if (state_type & PERLLCM_ISAM_REQUEST_STATE_T_COV_BLOCK) {
                // node_lists_t node_lists;
                std::list< isam::Node* > nodes_tmp;
                nodes_tmp.push_back(node);
                nodes_blockcov_rq.push_back(nodes_tmp);
            }
            else if (state_type & PERLLCM_ISAM_REQUEST_STATE_T_COV_RIGHTCOL) {
                //node_lists_t node_lists;
                if (node_recent)
                    nodes_rightcol_rq.push_back(std::make_pair(node, node_recent));
            }
        }
    }

    // populate cov
    double *cov = NULL;
    if (state_type & PERLLCM_ISAM_REQUEST_STATE_T_COV_FULL) {
        // populate full cov
        Eigen::MatrixXd m = m_slam.covariances().marginal (nodes_fullcov_rq);

        int cols = valid_n*6;
        int rows = valid_n*6;

        cov = (double *)calloc (cols*rows, sizeof (double));    
        strt->m = cols*rows;       
        strt->covariance = cov;
        int k = 0;
        int row, col;
        for (int i=0; i<m.rows(); i++) {
            for (int j=0; j<m.cols(); j++) {
                row = i, col = j;
                if (i % 6 == 3) row = i+2;
                if (i % 6 == 5) row = i-2;
                if (j % 6 == 3) col = j+2;
                if (j % 6 == 5) col = j-2;
                cov[k++] = m (row,col);
            }
        }        
    }
    else if (state_type & PERLLCM_ISAM_REQUEST_STATE_T_COV_BLOCK) {
        // populate block cov
        std::list<MatrixXd> m = m_slam.covariances().marginal (nodes_blockcov_rq);

        strt->m = valid_n*36;
        cov = (double *)calloc (valid_n*36, sizeof (double));
        strt->covariance = cov;

        int k = 0;
        for (std::list<MatrixXd>::iterator it = m.begin(); it != m.end(); it++) {
            MatrixXd a = *it;
            int row, col;
            for (int i=0;i<a.rows(); i++) {
                for (int j=0; j<a.cols(); j++) {
                    row = i, col = j;
                    if (i == 3) row = 5;
                    if (i == 5) row = 3;
                    if (j == 3) col = 5;
                    if (j == 5) col = 3;
                    cov[k++] = a(row,col);
                }
            }
        }        
    }
    else if (state_type & PERLLCM_ISAM_REQUEST_STATE_T_COV_RIGHTCOL) {
        
        std::list<MatrixXd> m = m_slam.covariances().access (nodes_rightcol_rq);

        strt->m = valid_n*36;
        cov = (double *)calloc (valid_n*36, sizeof (double));
        strt->covariance = cov;

        int k = 0;
        for (std::list<MatrixXd>::iterator it = m.begin(); it != m.end(); it++) {
            MatrixXd a = *it;
            int row, col;
            for (int i=0;i<a.rows(); i++) {
                for (int j=0; j<a.cols(); j++) {
                    row = i, col = j;
                    if (i == 3) row = 5;
                    if (i == 5) row = 3;
                    if (j == 3) col = 5;
                    if (j == 5) col = 3;
                    cov[k++] = a(row,col);
                }
            }
        }
        memcpy (m_cov_recent, &(cov[(valid_n-1)*36]), 6*6*sizeof (double));
    }
    else { // SE_REQUEST_STATE_T_COV_NONE
        strt->m = 0;       
        strt->covariance = NULL;
    }

    if (m_verbose & VERB_STATE) 
        std::cout << "[server]\tRerturning state done (" << (timestamp_now () - tic)/1000000.0 << " sec). " << std::endl;
    return strt;
}

void
isamServer::pub_vis (void) 
{
    size_t n_nodes = m_pose3d_nodes.size();
    if (n_nodes > 0) {
        perllcm_isam_graph_vis_t *igv = (perllcm_isam_graph_vis_t *)calloc (1, sizeof (*igv));
        igv->utime = 0;
        igv->nnodes = n_nodes;
        igv->mu = (double **)malloc (n_nodes * sizeof (double*));
        igv->node_id = (int64_t *)malloc (n_nodes * sizeof (int64_t));
            
        std::map<int64_t, isam::Pose3d_Node*>::iterator it;
        int i = 0;
        for (it = m_pose3d_nodes.begin(); it != m_pose3d_nodes.end(); it++) {
            Pose3d_Node* node = NULL;
            node = it->second; // the value in the key value pair
            if (NULL != node) {
                igv->mu[i] = (double *)malloc (6 * sizeof (double));
                igv->mu[i][0] = node->value().x();
                igv->mu[i][1] = node->value().y();
                igv->mu[i][2] = node->value().z();
                igv->mu[i][5] = node->value().yaw();
                igv->mu[i][4] = node->value().pitch();
                igv->mu[i][3] = node->value().roll();
                igv->node_id[i] = it->first;
                i++;
            }
        }
            
        igv->nlinks = m_links_i.size();
        igv->links_i = (int64_t *)calloc (igv->nlinks, sizeof (int64_t));
        memcpy (igv->links_i,   &(m_links_i[0]), igv->nlinks * sizeof (int64_t));
        igv->links_j = (int64_t *)calloc (igv->nlinks, sizeof (int64_t));
        memcpy (igv->links_j,   &(m_links_j[0]), igv->nlinks * sizeof (int64_t));
        igv->link_sensor_id = (int32_t *)calloc (igv->nlinks, sizeof (int32_t));
        memcpy (igv->link_sensor_id, &(m_links_sensor_id[0]), igv->nlinks * sizeof (int32_t));
            
        memcpy (igv->covariance, m_cov_recent, 6*6*sizeof(double));
            
        perllcm_isam_graph_vis_t_publish (m_lcm, m_chname_vis, igv);
        perllcm_isam_graph_vis_t_destroy (igv);
    }
}
