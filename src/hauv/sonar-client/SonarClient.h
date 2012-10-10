#ifndef _HAUV_SONARCLIENT_H
#define _HAUV_SONARCLIENT_H

#include <list>
#include <map>
#include <string>
#include <deque>

#include <boost/date_time.hpp>
#include <boost/thread.hpp>

#include <opencv/cv.h>

#include <Eigen/Core>
#include <lcm/lcm-cpp.hpp>
#include <isam/isam.h>
#include <bot_lcmgl_client/lcmgl.h>

#include "perls-common/units.h"
#include "perls-common/bot_util.h"
#include "perls-common/timestamp.h"
#include "perls-hauv/vehicletrajectory.h"

// lcmtypes
#include "perls-lcmtypes++/hauv/bs_rnv_2_t.hpp"
#include "perls-lcmtypes++/hauv/bs_cnv_t.hpp"
#include "perls-lcmtypes++/hauv/bs_dvl_2_t.hpp"
#include "perls-lcmtypes++/hauv/bs_pit_t.hpp"
#include "perls-lcmtypes++/hauv/didson_t.hpp"
#include "perls-lcmtypes++/hauv/wp_goto_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_add_node_ack_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_request_state_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_return_state_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_cmd_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_plink_t.hpp"

#include "ndt.h"
#include "scanbuilder.h"
#include "didson.h"

/**
 * A single sonar frame that contains the necessary information to compute
 * registration between different frames.
 */
class Frame
{
  public:
    explicit Frame(const State & state,
                   hauv::NDT* ndt,
                   const std::vector<isam::Point2d> & points,
                   IplImage* sonarImage);
    ~Frame();

    /**
     * Return the vehicle state at the time this frames was captured.
     */
    State state() {return m_state;}

    /**
     * The sonar image in Cartesian coordinates.
     */
    IplImage* image() {return m_sonarImage;}

    /**
     * Returns the feature points that have been extracted from the
     * sonar frame.
     */
    std::vector<isam::Point2d> & scan() {return m_scan;}

    /**
     * Return the NDT for this frame.
     */
    hauv::NDT* ndt() {return m_ndt;}
  private:
    // Vehicle state for this frame
    State m_state;

    // (raw frame)
    // Sonar image
    IplImage* m_sonarImage;

    // NDT
    hauv::NDT* m_ndt;

    // Extracted points
    std::vector<isam::Point2d> m_scan;
};

/**
 * Maintains a collection of frames and
 * provides various methods of accessing those
 * frames.
 */
class Map
{
  public:
    /**
     * Create a new map instance.
     */
    Map();

    /**
     * Destruct the map and release all memory used.
     * Deletes all the frames in the map.
     */
    ~Map();

    /**
     * Add a new frame to the map. The map object
     * receives ownership of the frame and is
     * responsible for destroying it.
     */
    void addFrame(Frame* frame);

    /**
     * Return the frame that is closest to the given pose.
     *
     * @param pose a pose that is used as a reference.
     *
     * @return the closest frame or 0 if no frame has been
     *         added to the map.
     */
    Frame* findClosestFrame(const isam::Pose3d & pose);

    /**
     * Return the frame that is closest to a given time.
     *
     * @param utime a time used as a reference.
     *
     * @return the closest frame or 0 if no frame has been
     *         added to the map.
     */
    Frame* findClosestFrame(int64_t utime);

    /**
     * Return a frame at the given time.
     *
     * @param utime  time in microseconds.
     * @return the frame at a given time or 0 if
     *         no frame is found.
     */
    Frame* getFrame(int64_t utime);

    /**
     * Return the number of frames in the map.
     */
    size_t getFrameCount() const {return m_frames.size();}

    /**
     * Remove all frames from the map.
     */
    void clear();
  private:
    typedef std::map<int64_t, Frame*> FrameMap;
    FrameMap m_frames;
};


class SonarClient
{
  public:
    SonarClient (lcm::LCM& lcm);
    ~SonarClient ();
    
    void run();
    
    /**
     * Processes incoming registrations
     */
    void processRegistrations();

    void onMouseDown(int event, int x, int y, int flags);

    // lcm callbacks
    void isam_cmd_t_callback (const lcm::ReceiveBuffer *rbuf, 
                              const std::string&  channel,
                              const perllcm::isam_cmd_t *msg);
    void add_node_ack_t_callback (const lcm::ReceiveBuffer *rbuf,
                                  const std::string &channel,
                                  const perllcm::isam_add_node_ack_t *msg);
    void return_state_t_callback (const lcm::ReceiveBuffer *rbuf,
                                  const std::string &channel,
                                  const perllcm::isam_return_state_t *msg);

    void hauv_bs_rnv_2_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                   const std::string& channel, 
                                   const hauv::bs_rnv_2_t *msg);
    void hauv_bs_pit_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                 const std::string& channel, 
                                 const hauv::bs_pit_t *msg);
    void hauv_bs_cnv_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                 const std::string& channel, 
                                 const hauv::bs_cnv_t *msg);
    void hauv_bs_dvl_2_t_callback (const lcm::ReceiveBuffer *rbuf,
                                   const std::string& channel, 
                                   const hauv::bs_dvl_2_t *msg);
    void hauv_didson_t_callback (const lcm::ReceiveBuffer *rbuf,  
                                 const std::string&  channel,
                                 const hauv::didson_t *msg );
    void wp_goto_t_callback (const lcm::ReceiveBuffer *rbuf, 
                             const std::string&  channel,
                             const hauv::wp_goto_t *msg);

    // Global variables
    CvFont g_font;

    // channel name read from config
    char *m_sonar_add_node_channel;
    char *m_add_node_ack_channel;
    char *m_isam_rq_st_channel;
    char *m_isam_rt_st_channel;
    char *m_vlink_channel;
    char *m_wp_goto_channel;
    char *m_isam_cmd_channel;

  private:
    /**
     */
    void registration(const perllcm::isam_plink_t *msg);

    /**
     * Logs a single registration
     *
     * @registrationid   Id of registration
     * @reference        Frame being registered to
     * @currentState     The current state of the vehicle
     * @points           The points extracted from the current view
     * @matchResult      Results of registration from current view to the reference frame
     * @score            The score from that registration
     * @currentImage     An cartesian image of the current sonar view
     * @ms               The scale from pixels to meters
     * @correction       The difference between the registration and predicted registration
     */
    void saveRegistration (int registrationId,
                           Frame* reference,
                           State* currentState,
                           std::vector<isam::Point2d>* points,
                           hauv::ScanMatch & matchResult,
                           double score,
                           IplImage* currentImage,
                           MapScale ms,
                           Eigen::Vector3f correction);

    Frame* addFrame (hauv::didson_t* didsonFrame, IplImage* sonarImage);
    void addScan (IplImage* image,
                  const std::vector<isam::Point2d> & points,
                  const isam::Pose2d & pose,
                  const CvScalar & color,
                  const MapScale & ms);

    void showTargets();
    double getTime();
    void sendImageColor (bot_lcmgl_t* lcmgl, const uint8_t* image, int width, int height, int stride, double hpos, double vpos);
    void sendImageBW (bot_lcmgl_t* lcmgl, const uint8_t* gray, int width, int height, int stride, double hpos, double vpos);
    
    void proposelink (int64_t utime1, int64_t utime2, double delta[6]);

    lcm::LCM *m_lcm;
    bool m_finished;

    BotParam *m_param;
    
    std::deque<perllcm::isam_plink_t*> m_links;
    std::deque<hauv::didson_t*> m_didson;
    VehicleTrajectory m_states;
    // Mutex to protect access to the trajectory.
    boost::mutex m_mutex_states;

    int m_registrationId;

    ScanBuilderRedCloud m_scanBuilder;
    //ScanBuilderOceanus2 m_scanBuilder;
    //ScanBuilderSeafloor m_scanBuilder;

    DidsonFrame* m_didsonFrame;
    Map m_map;
    std::string m_outputDirectory;
    std::fstream m_targetFile;
    Frame* m_lastFrame;

    boost::mutex m_mutex;
    boost::condition_variable m_newdata;

    /// Timestamp of the last frame added.
    int64_t m_last_utime;

    /// Total number of registrations
    size_t m_reg_total;
    /// Successful registrations
    size_t m_reg_success;

    boost::mutex m_mutex_targets;
    std::deque<Frame*> m_targets;

    bot_lcmgl_t* lcmgl_reg;
    bot_lcmgl_t* lcmgl_sonar;
    bot_lcmgl_t* lcmgl_target;

    bool m_active;

    // The time when last node was added;
    double m_node_added_time;

    // utime list of sonar node
    std::deque<int64_t> m_utimelist;

    // sonar link proposal range [m]
    double m_plink_sonar_thresh;
};

#endif
