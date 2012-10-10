#include <iostream>

#include "interf_thread.h"

using namespace std;

class InterfThread {
    private:

        char *channel_velodyne;
        char *channel_position;
        char *channel_drop_scan;
        char *channel_plink;

        perllcm::position_t pose;
        int64_t last_pose_utime;
        
        // velodyne setup
        double subsample_pct;
        double scan_start_stop_deg[2];
        velodyne_calib_t *calib;
        velodyne_laser_return_collector_t *collector;
        
        // buffer to catch the last couple of scans so that when we get a drop
        // node message we can go back in time and find the closest one
        BotPtrCircular   *velodyne_data_delay_circ;
        GMutex *circ_mutex;
        
        GAsyncQueue *gq;

        //lcm callbacks
        void
        drop_scan_cb (const lcm::ReceiveBuffer *rbuf,
                      const std::string &chan,
                      const perllcm::heartbeat_t *msg);
        void
        position_t_cb (const lcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const perllcm::position_t *msg);
        
        void
        velodyne_t_cb (const lcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const senlcm::velodyne_t *msg);
        
        void
        plink_t_cb (const lcm::ReceiveBuffer *rbuf,
                    const std::string &chan,
                    const perllcm::isam_plink_collection_t *msg);

    public:   

        InterfThread (interf_thread_data_t *t_data);
        ~InterfThread ();

        lcm::LCM *lcm;   
        BotParam *param;        
        
        int *done;
        
        void
        run (void);
        
};


void
InterfThread::drop_scan_cb (const lcm::ReceiveBuffer *rbuf,
                            const std::string &chan,
                            const perllcm::heartbeat_t *msg) {
    
    //std::cout << "[interf]\tDrop scan requested at " << msg->utime << std::endl;
    
    // add to queue
    queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (queue_element_t *));
    perllcm::heartbeat_t *copy = new perllcm::heartbeat_t (*msg);
    qe->msg = copy;
    qe->utime = msg->utime;
    qe->type = QE_DROP_SCAN;
    g_async_queue_push (gq, qe);

}

void
InterfThread::plink_t_cb (const lcm::ReceiveBuffer *rbuf,
                          const std::string &chan,
                          const perllcm::isam_plink_collection_t *msg){
    
    //std::cout << "[interf]\tLinks proposed at " << msg->utime << std::endl;
    
    for (int i=0; i<msg->nlinks ;i++) {
        // add to queue
        queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (queue_element_t *));
        perllcm::isam_plink_t *copy = new perllcm::isam_plink_t (msg->plink[i]);
        qe->msg = copy;
        qe->utime = (copy->utime_i >= copy->utime_j) ? copy->utime_i: copy->utime_j;
        qe->type = QE_PLINK;
        g_async_queue_push (gq, qe);
    }

}


void
InterfThread::position_t_cb (const lcm::ReceiveBuffer *rbuf,
                             const std::string &chan,
                             const perllcm::position_t *msg) {

    pose.utime = msg->utime;
    memcpy (pose.xyzrph, msg->xyzrph, sizeof (double) * 6);
    memcpy (pose.xyzrph_cov, msg->xyzrph_cov, sizeof (double) * 6*6);
    memcpy (pose.xyzrph_dot, msg->xyzrph_dot, sizeof (double) * 6);    
}

void
InterfThread::velodyne_t_cb (const lcm::ReceiveBuffer *rbuf,
                             const std::string &chan,
                             const senlcm::velodyne_t *msg) {
    
    // only push motion data if we are starting a new collection or there is a new pose
    int do_push_motion = 0;
    
    if (msg->packet_type == senlcm::velodyne_t::TYPE_DATA_PACKET) {
        
        perllcm_velodyne_laser_return_collection_t *lrc =
            velodyne_decode_data_packet_subsamp(calib, (uint8_t *)&(msg->data[0]), msg->datalen, msg->utime, subsample_pct);
            
        int ret = velodyne_collector_push_laser_returns (collector, lrc);
            
        velodyne_free_laser_return_collection (lrc);
        
        if (VELODYNE_COLLECTION_READY == ret) {
        
            perllcm_velodyne_laser_return_collection_t *lrc =
                velodyne_collector_pull_collection (collector);
            
            g_mutex_lock (circ_mutex);
            bot_ptr_circular_add (velodyne_data_delay_circ, lrc);
            g_mutex_unlock (circ_mutex);
            
            //starting a new collection
            do_push_motion = 1;
        }
    }
    
    // check if we have a new pose 
    if (0 == last_pose_utime || pose.utime != last_pose_utime) {

        // new pose
        do_push_motion = 1;
        last_pose_utime = pose.utime;
    }
    
    if (do_push_motion) {

        // push new motion onto collector
        perllcm_position_t state = {0};

        state.utime = pose.utime;
        memcpy (state.xyzrph, &(pose.xyzrph[0]), 6*sizeof (double));
        memcpy (state.xyzrph_dot, &(pose.xyzrph_dot[0]), 6*sizeof (double));

        velodyne_collector_push_state (collector, state);
        do_push_motion = 0;
    }
    
}

InterfThread::InterfThread (interf_thread_data_t *t_data) {
    
    lcm = t_data->lcm;
    param = t_data->param;
    done = t_data->done;
    circ_mutex = t_data->circ_mutex;
    velodyne_data_delay_circ = t_data->velodyne_data_delay_circ;
    gq = t_data->gq;
    
    // init
    last_pose_utime = 0;
    pose.utime = 0;
    memset (pose.xyzrph, 0, sizeof (double) * 6);
    memset (pose.xyzrph_cov, 0, sizeof (double) * 6*6);
    memset (pose.xyzrph_dot, 0, sizeof (double) * 6);
    
    // load from config file
    channel_position = bot_param_get_str_or_fail (param, "velodyne-scan-matcher.position_channel");
    channel_drop_scan = bot_param_get_str_or_fail (param, "velodyne-scan-matcher.drop_scan_channel");
    channel_plink = bot_param_get_str_or_fail (param, "velodyne-scan-matcher.plink_channel");
    channel_velodyne = bot_param_get_str_or_fail (param, "sensors.velodyne.channel");

    char *velodyne_model = bot_param_get_str_or_fail (param, "sensors.velodyne.model");
    char *calib_file = bot_param_get_str_or_fail (param, "sensors.velodyne.intrinsic_calib_file");
    subsample_pct = 1.0;
    bot_param_get_double (param, "velodyne-scan-matcher.subsample_pct", &subsample_pct);
    
    if (0 == strcmp (velodyne_model, VELODYNE_HDL_32E_MODEL_STR)) {
        calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_32E, calib_file);
    } else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR)){
        calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S1, calib_file);
    } else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S2_MODEL_STR)){
        calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S2, calib_file);    
    } else {
        ERROR ("ERROR: Unknown Velodyne model \'%s\'", velodyne_model);
    }
    free (velodyne_model);
    free (calib_file);
    
    double x_vs[6];
    bot_param_get_double_array_or_fail (param, "sensors.velodyne.x_vs", x_vs, 6);
    x_vs[3] = x_vs[3] * UNITS_DEGREE_TO_RADIAN;
    x_vs[4] = x_vs[4] * UNITS_DEGREE_TO_RADIAN;
    x_vs[5] = x_vs[5] * UNITS_DEGREE_TO_RADIAN;

    if (2 == bot_param_get_double_array (param, "velodyne-scan-matcher.scan_start_stop_deg", scan_start_stop_deg, 2)){
        collector = velodyne_laser_return_collector_create (0,
                                scan_start_stop_deg[0]*UNITS_DEGREE_TO_RADIAN,
                                scan_start_stop_deg[1]*UNITS_DEGREE_TO_RADIAN,
                                x_vs); 
    } else {
        collector = velodyne_laser_return_collector_create (1, 0, 0, x_vs); // full scan
    }
    
}


InterfThread::~InterfThread () {
    velodyne_calib_free (calib);
    velodyne_laser_return_collector_free (collector);
}


void
InterfThread::run (void) {
    
    // subscribe to lcm channels
    lcm->subscribe(channel_velodyne, &InterfThread::velodyne_t_cb, this);
    lcm->subscribe(channel_position, &InterfThread::position_t_cb, this);
    lcm->subscribe(channel_drop_scan, &InterfThread::drop_scan_cb, this);
    lcm->subscribe(channel_plink, &InterfThread::plink_t_cb, this);
    
    while (!(*done)) {
        lcm->handle();  
    }
    
}


void * 
interf_thread_f(void *user_data) {
    
    interf_thread_data_t *t_data = (interf_thread_data_t *)user_data;
    
    InterfThread it = InterfThread::InterfThread(t_data);
    
    it.run ();
    
    return NULL;
}

