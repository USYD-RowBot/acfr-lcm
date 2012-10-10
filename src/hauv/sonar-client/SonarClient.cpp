#include <iostream>
#include <stdexcept>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <Eigen/Dense>

#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <isam/isam.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-hauv/vehicletrajectory.h"

// lcmtypes
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_vlink_t.hpp"

#include "didson.h"
#include "ndt.h"
#include "scanbuilder.h"

#include "SonarClient.h"

#include "perls-math/ssc.h"

using namespace std;

#define PLINK_DIST 1

// parameters
const unsigned int MIN_POINTS = 600;
const unsigned int MAX_POINTS = 12050;
const double       NDT_CELL_SIZE = 0.25;
const double       NDT_MIN_EIGEN_RATIO = 0.05;
const double       FRAME_DISTANCE_XY = 0.3;
const double       FRAME_DISTANCE_TH = 0.09;  // approx 5 deg
const int          DECIMATION_FACTOR = 4;
const double       MAX_FRAME_DISTANCE = 4.0;
//const double MIN_SCORE = 0.05/DECIMATION_FACTOR;    // The score needed to accept a registration
//const double MIN_SCORE = 0.07/DECIMATION_FACTOR;    // The score needed to accept a registration

const double MIN_SCORE = 0.10;    // The score needed to accept a registration
//const double MIN_SCORE = 0.08;    // The score needed to accept a registration

// SonarClient
//--------------------------------------------------------------
SonarClient::SonarClient (lcm::LCM& lcm) : m_finished(false),
                                           m_registrationId(0),
                                           m_didsonFrame(0),
                                           m_lastFrame(0),
                                           m_last_utime(0),
                                           m_reg_total(0),
                                           m_reg_success(0),
                                           m_active(false),
                                           m_node_added_time(0)
{
  
    m_lcm = &lcm;

    // read from config
    m_param = bot_param_new_from_file(BOTU_PARAM_DEFAULT_CFG);

    if (0!=bot_param_get_str (m_param, "hauv-client.lcm_channels.SONAR_ADD_NODE_CHANNEL", &m_sonar_add_node_channel))
        throw runtime_error("hauv-client.lcm_chanenls.SONAR_ADD_NODE_CHANNEL not found in config file");

    if (0!=bot_param_get_str (m_param, "isamServer.lcm_channels.ADD_NODE_ACK_CHANNEL", &m_add_node_ack_channel))
        throw runtime_error("isamServer.lcm_channels.ADD_NODE_ACK_CHANNEL not found in config file");

    if (0!=bot_param_get_str (m_param, "isamServer.lcm_channels.REQUEST_ST_CHANNEL", &m_isam_rq_st_channel))
        throw runtime_error("isamServer.lcm_channels.REQUEST_ST_CHANNEL not found in config file");

    if (0!=bot_param_get_str (m_param, "isamServer.lcm_channels.RETURN_ST_CHANNEL", &m_isam_rt_st_channel))
        throw runtime_error("isamServer.lcm_channels.RETURN_ST_CHANNEL not found in config file");

    if (0!=bot_param_get_str (m_param, "isamServer.lcm_channels.VLINK_CHANNEL", &m_vlink_channel))
        throw runtime_error("isamServer.lcm_channels.VLINK_CHANNEL not found in config file");

    if (0!=bot_param_get_str (m_param, "isamServer.lcm_channels.CMD_CHANNEL", &m_isam_cmd_channel))
        throw runtime_error("isamServer.lcm_channels.CMD_CHANNEL not found in config file");

    if (0!=bot_param_get_str (m_param, "hauv-client.wp_nav.GOTO_CHANNEL", &m_wp_goto_channel))
        throw runtime_error("hauv-client.wp_nav.GOTO_CHANNEL not found in config file");

    if (0!=bot_param_get_double (m_param, "sonar-client.propose_link.plink_thresh", &m_plink_sonar_thresh))
        throw runtime_error("sonar-client.propose_link.plink_thresh not found in config file");

    // lcmgl uses lcm_t * but this uses lcmtypes++
    lcmgl_sonar = bot_lcmgl_init(m_lcm->getUnderlyingLCM(), "Sonar");
    lcmgl_reg = bot_lcmgl_init(m_lcm->getUnderlyingLCM(), "Sonar - Registration");
    lcmgl_target = bot_lcmgl_init(m_lcm->getUnderlyingLCM(), "Sonar - Target");

    m_outputDirectory = "./results";
    std::cout << "Output directory: " << m_outputDirectory << std::endl;

    // Lets check if directory exists
    struct stat st;
    if(stat(m_outputDirectory.c_str(),&st) != 0) {
        if (mkdir(m_outputDirectory.c_str(), S_IRWXU) != 0) {
            std::cout << "Failed creating directory " << m_outputDirectory << std::endl;
            perror("Failed creating directory");
            exit(-1);
        }
    }

    // Get a timestamp to create a folder for the current run
    time_t t;
    struct tm *tmp;
    t = time(NULL);
    tmp = localtime(&t);
    char outstr[200];
    if (strftime(outstr, sizeof(outstr), "%Y%m%d-%H%M%S", tmp) == 0) {
        perror("strftime returned 0");
        exit(-1);
    }
    m_outputDirectory += std::string("/") + outstr;
    if (mkdir(m_outputDirectory.c_str(), S_IRWXU) != 0) {
        std::cout << "Failed creating directory " << m_outputDirectory << std::endl;
        perror("Failed creating directory");
        exit(-1);
    }
    std::cout << "Current working directory: " << m_outputDirectory << std::endl;

    std::string filename = m_outputDirectory+std::string("/targets.txt");
    std::cout << "Target file: " << filename << std::endl;
    m_targetFile.open(filename.c_str(),std::fstream::out);
}

SonarClient::~SonarClient()
{
    m_targetFile.close();

    bot_lcmgl_destroy(lcmgl_reg);
    bot_lcmgl_destroy(lcmgl_sonar);
}

void 
SonarClient::run()
{
    // start lcm subscription
    while (0 == m_lcm->handle() && !m_finished);
    return;
}

void 
SonarClient::processRegistrations()
{
    while (!m_finished) {
        int key = cvWaitKey(33);
        if ((key&0xff) == 's') {
            std::cout << "Save current frame" << std::endl;

        } else if ((key&0xff) == 'r') {
        } else if ((key&0xff) == 27) {
            m_finished = true;
            continue;
        }

        size_t count = 0;
        perllcm::isam_plink_t* link = 0;

        size_t didsonCount = 0;
        hauv::didson_t* lastDidsonFrame = 0;

        // Wait for new data
        {
            boost::mutex::scoped_lock lock(m_mutex);

            // Wait until we get new data
            //while (!m_links.size() && ! m_didson.size())
            //{
            //    m_newdata.timed_wait(lock,t);
            //}
            // We got new data check both queues

            // Wait for condition for a fixed time
            boost::posix_time::milliseconds t(200);
            count = m_links.size();
            didsonCount = m_didson.size();

            if(count==0 && didsonCount==0) {
                // Wait if no data
                m_newdata.timed_wait(lock,t);
            }

            count = m_links.size();
            if (count) {
                link = m_links.back();
                m_links.pop_back();
            }

            didsonCount = m_didson.size();
            if (didsonCount) {
                lastDidsonFrame = m_didson.front();
                m_didson.pop_front();
            }
        }

        if (link) {
            registration(link);
            delete link;
        }

        if (lastDidsonFrame) {
            if (m_didsonFrame == 0) {
                m_didsonFrame = new DidsonFrame(lastDidsonFrame);
                m_didsonFrame->makeNewImage(300);
            } else {
                m_didsonFrame->setFrame(lastDidsonFrame);
            }
            MapScale ms = m_didsonFrame->getMapScale();
            IplImage* sonarImage = cvCreateImage(cvSize(ms.width, ms.height), IPL_DEPTH_8U, 1);
            IplImage* displayImage = cvCreateImage(cvSize(ms.width, ms.height), IPL_DEPTH_8U, 3);

            memcpy(sonarImage->imageData, m_didsonFrame->getImage(), ms.width*ms.height);

            Frame* f = addFrame(lastDidsonFrame, sonarImage);

            // Send sonar frames every two seconds
#if 0
            double current_time = getTime();
            if (fabs(current_time-m_node_added_time) > 2.0)
                {
                    se_add_node_t node;
                    node.utime = lastDidsonFrame->time_first_packet;
                    node.node_type = se::add_node_t::NODE_POSE3D;
                    node.sensor_id = se::add_node_t::NODE_ODO;
                    se::add_node_t::publish(m_lcm, "SE_ADD_NODE_CLIENT", &node);
                    m_node_added_time = current_time;
                }
#endif
            cvCvtColor(sonarImage, displayImage, CV_GRAY2RGB);

            IplImage* displayImage2 = cvCloneImage(displayImage);
            char statusText[255];
            sprintf(statusText, "#links: %zd", count); // # links in the queue
            cvPutText(displayImage, statusText, cvPoint(10,20), &g_font, cvScalar(255,255,255));
            sprintf(statusText, "#frames: %zd", didsonCount); // # frames in the queue
            cvPutText(displayImage, statusText, cvPoint(10,40), &g_font, cvScalar(255,255,255));

            sprintf(statusText, "#reg total: %zd", m_reg_total); // # frames in the queue
            cvPutText(displayImage, statusText, cvPoint(10,60), &g_font, cvScalar(255,255,255));
            sprintf(statusText, "#reg success: %zd", m_reg_success); // # frames in the queue
            cvPutText(displayImage, statusText, cvPoint(10,80), &g_font, cvScalar(255,255,255));
            sprintf(statusText, "#frames stored: %zd", m_map.getFrameCount()); // # frames in the queue
            cvPutText(displayImage, statusText, cvPoint(10,100), &g_font, cvScalar(255,255,255));

            cvShowImage("SonarImage", displayImage);

            if (f) {
                std::cout << "Points: "<< f->scan().size() << std::endl;
                addScan(displayImage2, f->scan(), isam::Pose2d(0,0,0), cvScalar(0,0,255,0), ms);
                cvShowImage("DisplayImage", displayImage2);

                sendImageColor(lcmgl_sonar, (unsigned char*)displayImage2->imageData, displayImage->width, displayImage->height,
                               displayImage->widthStep, 0.0, 0.0);

                boost::mutex::scoped_lock lock(m_mutex_targets);
                m_lastFrame = f;
            }
            cvReleaseImage(&sonarImage);
            cvReleaseImage(&displayImage);
            cvReleaseImage(&displayImage2);
            delete lastDidsonFrame;
        }
    }
    m_finished = true;
}


void 
SonarClient::onMouseDown(int event, int x, int y, int flags)
{
  //TODO: make this function work correctly
    if (0) {
        State s = m_lastFrame->state();
        // Save information from last frame
        m_targetFile << s.utime << " "
                     << s.rel_x << " "
                     << s.rel_y << " "
            ;

        m_targetFile << std::endl;

        boost::mutex::scoped_lock lock(m_mutex_targets);
        m_targets.push_back(m_lastFrame);
        if (m_targets.size()>10) {
            m_targets.pop_front();
        }
    }

    //showTargets();
}

// private functions come next
void 
SonarClient::registration(const perllcm::isam_plink_t *msg)
{
    Frame* A = m_map.getFrame(msg->utime_i);
    Frame* B = m_map.getFrame(msg->utime_j);

    hauv::ScanMatch matchResult;
    double matchScore;

    bool accepted = false;
    if (A && B)
        {
            // Find transformation from frame A to frame B
            hauv::NDT* ndtA = A->ndt();
            //    std::vector<isam::Point2d> & pointsB = B->scan();
            std::vector<isam::Point2d> pointsB;
            for (size_t i=0; i<B->scan().size(); ++i) if ((i%DECIMATION_FACTOR) == 0) pointsB.push_back(B->scan()[i]);

            int pointCount = (pointsB.size()+A->scan().size()/DECIMATION_FACTOR);

            // Try to get a registration with keyframe
#if 0
            // IJRR settings
            hauv::ScanMatch registrationResult[50];
            int n = 0;
            double dt = 0.20; // Step size used
            for (double y = -2.0*dt; y<=2.0*dt; y+=dt) {
                for (double x = -3.0*dt; x<=3.0*dt; x+=dt) {
#else
            // original Seneca settings
            hauv::ScanMatch registrationResult[27];
            int n = 0;
            double dt = 0.15; // Step size used
            for (double y = -2.0*dt; y<=2.0*dt; y+=dt) {
                for (double x = -2.0*dt; x<=2.0*dt; x+=dt) {
#endif
                    //for (double th = -0.1; x<=0.1; x+=0.1)
                    {
                        double th = 0.0;
                        ndtA->findTransformation(pointsB, isam::Pose2d(x,y,th), registrationResult[n++], 3);
                    }
                }
            }

            // Find the best score of those tested
            hauv::ScanMatch matchResult = registrationResult[0];
            double maxScore = ndtA->calculateScore(pointsB,matchResult.pose)/pointCount;
            matchScore = maxScore/(pointsB.size()+A->scan().size());
            for (int i=1;i<n;i++) {
                hauv::ScanMatch & r = registrationResult[i];
                double score = ndtA->calculateScore(pointsB,r.pose)/pointCount;
                if (score > maxScore) {
                    matchResult = r;
                    matchScore = score/pointCount;
                    maxScore = score;
                }
            }

            // Iterate on the best match
            ndtA->findTransformation(pointsB, matchResult.pose, matchResult, 10);
            matchScore = ndtA->calculateScore(pointsB,matchResult.pose)/pointCount;
            if (matchScore > MIN_SCORE) {
                accepted = true;

                /*
                // Calculate correction
                Pose2d reg = matchResult.pose;
                State ref = A->state();
                State dst = B->state();
                Pose3d poseA(ref.x, ref.y, ref.z, ref.heading, ref.pitch, ref.roll);
                Pose3d poseB(dst.x, dst.y, dst.z, dst.heading, dst.pitch, dst.roll);
                Pose3d dlt = poseB.ominus(poseA);

                double sonar = ref.pitch_sonar;
                double dvl = ref.pitch_dvl;

                Eigen::Vector3f d; d << dlt.x(), dlt.y(), dlt.z();
                Eigen::Matrix3f R;
                R          = Eigen::AngleAxisf(-sonar, Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(dvl+M_PI/2.0, Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ());
                Eigen::Matrix3f T;
                T          = Eigen::AngleAxisf(dlt.roll(), Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(dlt.pitch(), Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(dlt.yaw(), Eigen::Vector3f::UnitZ());

                d = R.inverse() * d;

                Eigen::Vector3f v; v << 1.0, 0.0, 0.0; // x - axis
                // v is in sonar coordinates
                v = R * v;
                // v is in vehicle coordinates
                v = T * v;
                // v is in new frame
                v = R.inverse() * v;
                // v is in sonar coordinates
                // Get angle from v.x, v.y

                double dlt_heading = atan2(v(1), v(0));
                */
                isam::Pose2d reg = matchResult.pose;
                State ref = A->state();
                State dst = B->state();
                isam::Pose3d poseA(ref.x, ref.y, ref.z, ref.heading, ref.pitch, ref.roll);
                isam::Pose3d poseB(dst.x, dst.y, dst.z, dst.heading, dst.pitch, dst.roll);

                isam::Pose3d sonar_frame1 = isam::Pose3d(0.,0.,0., M_PI/2.0, 0., 0.);
                sonar_frame1 = sonar_frame1.oplus(isam::Pose3d(0.2, 0., 0., 0., 0., ref.pitch_dvl - M_PI/2.0));
                sonar_frame1 = sonar_frame1.oplus(isam::Pose3d(0., 0., 0., 0., ref.pitch_sonar, 0.));

                isam::Pose3d sonar_frame2 = isam::Pose3d(0.,0.,0., M_PI/2.0, 0., 0.);
                sonar_frame2 = sonar_frame2.oplus(isam::Pose3d(0.2, 0., 0., 0., 0., dst.pitch_dvl - M_PI/2.0));
                sonar_frame2 = sonar_frame2.oplus(isam::Pose3d(0., 0., 0., 0., dst.pitch_sonar, 0.));

                isam::Pose3d dlt = (poseB.oplus(sonar_frame2)).ominus(poseA.oplus(sonar_frame1));

                Eigen::Vector3f correction;
                correction << abs(reg.x()-dlt.x()), abs(reg.y()-dlt.y()),  abs(reg.t()-dlt.yaw());

                std::cout << "Registration: " << std::endl
                          << "Registration: Ref: " << ref.x << " " << ref.y << " " << ref.z << std::endl
                          << "Registration: Dst: " << dst.x << " " << dst.y << " " << dst.z << std::endl
                          << "Registration: Dlt: " << dlt.x() << " " << dlt.y() << " " << dlt.yaw() << std::endl
                    //                << "Registration: Pre: " << d(0) << " " << d(1) << " " << " t: " << dlt_heading << " " << d(2) << std::endl
                          << "Registration: Reg: " << reg.x() << " " << reg.y() << " t: " << reg.t() << std::endl
                          << "Registration: Dif: " << correction(0) << " " << correction(1) << " " << correction(2)
                          << std::endl;
                // ----------------

                // Save registration results
                State currentState = B->state();
                saveRegistration(m_registrationId++, A, &currentState, &pointsB,
                                 matchResult, matchScore,
                                 B->image(), m_didsonFrame->getMapScale(), correction);

            }

            // Display registration
            MapScale ms = m_didsonFrame->getMapScale();
            IplImage* displayImage = cvCreateImage(cvSize(ms.width, ms.height), IPL_DEPTH_8U, 3);
            cvCvtColor(A->image(), displayImage, CV_GRAY2RGB);

            //cvSetImageROI(regImage, cvRect(0, height, width, height));
            //cvAdd(displayImage,regImage,regImage);

            addScan(displayImage, A->scan(), isam::Pose2d(0,0,0),cvScalar(255,255,255,0),ms);
            addScan(displayImage, pointsB, isam::Pose2d(0,0,0),cvScalar(0,0,255,0),ms);
            if (accepted) {
                addScan(displayImage, pointsB, matchResult.pose, cvScalar(0,255,0,0),ms);
            } else {
                addScan(displayImage, pointsB, matchResult.pose, cvScalar(255,255,0,0),ms);
            }

            // Add some text information on the registration
            char statusText[512];

            sprintf(statusText,"Score: %lf",matchScore);
            if (matchScore>MIN_SCORE) {
                cvPutText(displayImage, statusText, cvPoint(10,20), &g_font, cvScalar(0,255,0));
            } else {
                cvPutText(displayImage, statusText, cvPoint(10,20), &g_font, cvScalar(0,0,255));
            }
            sprintf(statusText, "Total: %lf", matchScore*pointsB.size());
            cvPutText(displayImage, statusText, cvPoint(10,40), &g_font, cvScalar(255,255,255));

            sprintf(statusText, "Count: %zd", pointsB.size());
            cvPutText(displayImage, statusText, cvPoint(10,60), &g_font, cvScalar(255,255,255));

            sprintf(statusText, "X: %lf",matchResult.pose.x());
            cvPutText(displayImage, statusText, cvPoint(10,80), &g_font, cvScalar(255,255,255));
            sprintf(statusText, "Y: %lf",matchResult.pose.y());
            cvPutText(displayImage, statusText, cvPoint(10,100), &g_font, cvScalar(255,255,255));
            sprintf(statusText, "T: %lf",matchResult.pose.t()/M_PI*180);
            cvPutText(displayImage, statusText, cvPoint(10,120), &g_font, cvScalar(255,255,255));

            cvShowImage("MatchImage", displayImage);
            cvReleaseImage(&displayImage);

        }
    else
        {
            // @todo give a reason why not accepted
        }

    double m[3];
    double cov[9];
    m[0] = matchResult.pose.x();
    m[1] = matchResult.pose.y();
    m[2] = matchResult.pose.t();
    for (int i=0;i<9; i++) cov[i] = 0.0;
    // x,y variance (0.1m)^2
    cov[0] = cov[4] = 0.01;
    // th variance
    cov[8] = (1.0/180.0*M_PI)*(1.0/180.0*M_PI);

    // Reply to the proposed link
    perllcm::isam_vlink_t l;
    l.utime = timestamp_now ();
    l.id1 = msg->utime_i;
    l.id2 = msg->utime_j;
    l.n = 3;
    l.n2 = 9;
    l.link_type = perllcm::isam_vlink_t::LINK_SONAR2D;
    l.sensor_id = perllcm::isam_vlink_t::SENSOR_SONAR;
    std::copy(m, m+3, std::back_inserter(l.z));
    std::copy(cov, cov+9, std::back_inserter(l.R));

    // @todo configure the sonar offset from the DVL (currentl 20cm to starboard)
    State s1 = m_states.getState(msg->utime_i);
    State s2 = m_states.getState(msg->utime_j);

    isam::Pose3d sonar_frame1 = isam::Pose3d(0.,0.,0., M_PI/2.0, 0., 0.);
    sonar_frame1 = sonar_frame1.oplus(isam::Pose3d(0.2, 0., 0., 0., 0., s1.pitch_dvl - M_PI/2.0));
    sonar_frame1 = sonar_frame1.oplus(isam::Pose3d(0., 0., 0., 0., s1.pitch_sonar, 0.));

    isam::Pose3d sonar_frame2 = isam::Pose3d(0.,0.,0., M_PI/2.0, 0., 0.);
    sonar_frame2 = sonar_frame2.oplus(isam::Pose3d(0.2, 0., 0., 0., 0., s2.pitch_dvl - M_PI/2.0));
    sonar_frame2 = sonar_frame2.oplus(isam::Pose3d(0., 0., 0., 0., s2.pitch_sonar, 0.));

    // 2012.02.02 ak: perllcm isam plink does not have link_id
    //l.publisher_id = 2;            // A Sonar
    //l.link_id = msg->link_id;  

    // dynamic sensor transform (note: perllcm_isam pose is in [x,y,z,r,p,yaw] order
    l.dynamic_xvs = 1;
    l.x_vs1[0] = sonar_frame1.x();     l.x_vs1[1] = sonar_frame1.y();     l.x_vs1[2] = sonar_frame1.z();
    l.x_vs1[3] = sonar_frame1.roll();  l.x_vs1[4] = sonar_frame1.pitch(); l.x_vs1[5] = sonar_frame1.yaw();
    l.x_vs2[0] = sonar_frame2.x();     l.x_vs2[1] = sonar_frame2.y();     l.x_vs2[2] = sonar_frame2.z();
    l.x_vs2[3] = sonar_frame2.roll();  l.x_vs2[4] = sonar_frame2.pitch(); l.x_vs2[5] = sonar_frame2.yaw();

    if (accepted) {
        l.accept = 1;
        l.accept_code = perllcm::isam_vlink_t::CODE_ACCEPTED;
        //l.comment = (char *) "accepted";
    } else {
        l.accept = 0;
        l.accept_code = perllcm::isam_vlink_t::CODE_INVALID_MODEL;
        //l.comment = (char *) "rejected";
    }
    m_lcm->publish(m_vlink_channel, &l);

    ++m_reg_total;
    if (accepted) ++m_reg_success;
}

void 
SonarClient::saveRegistration (int registrationId,
                               Frame* reference,
                               State* currentState,
                               std::vector<isam::Point2d>* points,
                               hauv::ScanMatch & matchResult,
                               double score,
                               IplImage* currentImage,
                               MapScale ms,
                               Eigen::Vector3f correction)
{
    State state = reference->state();
    State* refState = &state;
    //    Pose2d_Node* refNode = refState->node();
    //    Pose2d_Node* currentNode = currentState->node();
    /*
    // Write to log file
    g_registrationFile << registrationId << " "
    << currentState->time() << " "
    << currentNode->value().x() << " "
    << currentNode->value().y() << " "
    << currentNode->value().t() << " "
    << points->size() << " "
    << refState->time() << " "
    << refNode->value().x() << " "
    << refNode->value().y() << " "
    << refNode->value().t() << " "
    << reference->scan().size() << " "
    << matchResult.pose.x() << " "
    << matchResult.pose.y() << " "
    << matchResult.pose.t() << " "
    << matchResult.covariance[0][0] << " "   // xx
    << matchResult.covariance[1][1] << " "   // yy
    << matchResult.covariance[2][2] << " "   // tt
    << matchResult.covariance[0][1] << " "   // xy
    << matchResult.covariance[0][2] << " "   // xt
    << matchResult.covariance[1][2] << " "   // yt
    << matchResult.scoreStart << " "
    << matchResult.score << " "
    << score << " "
    << std::endl;
    */
    // Write image(s)
    // Combine results in a single image
    int width = currentImage->width;
    int height = currentImage->height;

    CvSize sonarImageSize = cvSize(3 * width, 2 * height);
    IplImage* regImage = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 3);
    IplImage* displayImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

    cvZero(regImage);
    cvZero(displayImage);

    char imageFilename[500];
    sprintf(imageFilename, "%s/reg-%d.png",m_outputDirectory.c_str(), registrationId);
    std::cout << imageFilename << std::endl;

    // Reference image

    // Current image
    // Reference - extracted features
    // Current - extracted features
    // Reference, Current - extracted features, after registration

    // Add current view at bottom
    cvCvtColor(currentImage, displayImage, CV_GRAY2RGB);

    cvSetImageROI(regImage, cvRect(0, height, width, height));
    cvAdd(displayImage,regImage,regImage);

    addScan(displayImage, *points, isam::Pose2d(0,0,0),cvScalar(0,0,255,0),ms);
    cvSetImageROI(regImage, cvRect(width, height, width, height));
    cvAdd(displayImage,regImage,regImage);

    // Add reference view at top
    cvCvtColor(reference->image(), displayImage, CV_GRAY2RGB);

    cvSetImageROI(regImage, cvRect(0, 0, width, height));
    cvAdd(displayImage,regImage,regImage);

    addScan(displayImage, reference->scan(), isam::Pose2d(0,0,0),cvScalar(0,255,0,0),ms);
    cvSetImageROI(regImage, cvRect(width, 0, width, height));
    cvAdd(displayImage,regImage,regImage);

    char statusText[512];

    // Add combined view
    addScan(displayImage, *points, matchResult.pose,cvScalar(0,0,255,0),ms);

    // Send registration to viewer
    sprintf(statusText, "Diff time: %lf",refState->utime/1E6-currentState->utime/1E6);
    cvPutText(displayImage, statusText, cvPoint(10,20), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "dx: %lf",correction(0));
    cvPutText(displayImage, statusText, cvPoint(10,40), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "dy: %lf",correction(1));
    cvPutText(displayImage, statusText, cvPoint(10,60), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "dt: %lf",correction(2));
    cvPutText(displayImage, statusText, cvPoint(10,80), &g_font, cvScalar(255,255,255));
    sendImageColor(lcmgl_reg, (unsigned char*)displayImage->imageData, displayImage->width,
                   displayImage->height, displayImage->widthStep, 0.0, 0.75);

    std::cout << "correction "
              << (refState->utime/1E6-currentState->utime/1E6) << " "
              << correction(0)<< " "
              << correction(1) << " "
              << correction(2) << std::endl;

    cvSetImageROI(regImage, cvRect(2*width, 0, width, height));
    cvAdd(displayImage,regImage,regImage);

    cvSetImageROI(regImage, cvRect(2*width, height, width, height));

    sprintf(statusText,"Score: %lf",score);
    cvPutText(regImage, statusText, cvPoint(10,20), &g_font, cvScalar(0,255,0));

    sprintf(statusText, "Total: %lf", score*points->size());
    cvPutText(regImage, statusText, cvPoint(10,40), &g_font, cvScalar(255,255,255));

    sprintf(statusText, "Count: %zd",points->size());
    cvPutText(regImage, statusText, cvPoint(10,60), &g_font, cvScalar(255,255,255));

    sprintf(statusText, "X: %lf",matchResult.pose.x());
    cvPutText(regImage, statusText, cvPoint(10,80), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "Y: %lf",matchResult.pose.y());
    cvPutText(regImage, statusText, cvPoint(10,100), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "T: %lf",matchResult.pose.t()/M_PI*180);
    cvPutText(regImage, statusText, cvPoint(10,120), &g_font, cvScalar(255,255,255));

    sprintf(statusText, "CXX*1000: %lf",1000.0*(matchResult.covariance[0][0]));
    cvPutText(regImage, statusText, cvPoint(10,140), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "CYY*1000 %lf",1000.0*(matchResult.covariance[1][1]));
    cvPutText(regImage, statusText, cvPoint(10,160), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "CTT*1000: %lf",1000.0*(matchResult.covariance[2][2]));
    cvPutText(regImage, statusText, cvPoint(10,180), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "CXY*1000: %lf",1000.0*(matchResult.covariance[0][1]));
    cvPutText(regImage, statusText, cvPoint(10,200), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "CXT*1000: %lf",1000.0*(matchResult.covariance[0][2]));
    cvPutText(regImage, statusText, cvPoint(10,220), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "CYT*1000: %lf",1000.0*(matchResult.covariance[1][2]));
    cvPutText(regImage, statusText, cvPoint(10,240), &g_font, cvScalar(255,255,255));

    sprintf(statusText, "Cur time: %lf",currentState->utime/1E6);
    cvPutText(regImage, statusText, cvPoint(10,260), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "Ref time: %lf",refState->utime/1E6);
    cvPutText(regImage, statusText, cvPoint(10,280), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "Diff time: %lf",currentState->utime/1E6 - refState->utime/1E6);
    cvPutText(regImage, statusText, cvPoint(10,300), &g_font, cvScalar(255,255,255));

    sprintf(statusText, "DX: %lf",correction(0));
    cvPutText(regImage, statusText, cvPoint(10,320), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "DY: %lf",correction(1));
    cvPutText(regImage, statusText, cvPoint(10,340), &g_font, cvScalar(255,255,255));
    sprintf(statusText, "DT: %lf",correction(2));
    cvPutText(regImage, statusText, cvPoint(10,360), &g_font, cvScalar(255,255,255));

    //    double d = dist(reference->node()->value(), currentState->node()->value());
    //    sprintf(statusText, "Dist: %lf",d);
    //    cvPutText(regImage, statusText, cvPoint(10,320), &g_font, cvScalar(255,255,255));

    cvSetImageROI(regImage, cvRect(0, 0, regImage->width, regImage->height));
    cvShowImage("RegistrationImage", regImage);

    // Save image
    // TODO: create a key to turn this on/off
    cvSaveImage(imageFilename, regImage);

    cvReleaseImage(&regImage);
    cvReleaseImage(&displayImage);
}

Frame* 
SonarClient::addFrame(hauv::didson_t* didsonFrame, IplImage* sonarImage)
{
    // Check time stamp and if any other frame is already in the vicinity

    int64_t t = didsonFrame->time_first_packet;
    State s;

    int count = 0;
    {
        boost::mutex::scoped_lock lock(m_mutex_states);
        while(!m_states.canInterpolate(t)) {
            lock.unlock();
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            if (count++ > 5) return 0;  // timeout;

            lock.lock();
        }
    }

    {
        if (!m_states.canInterpolate(t)) {
            std::cout << "Failure to interpolate: " << t << std::endl;
            exit(-1);
        }
    }

    {
        boost::mutex::scoped_lock lock(m_mutex_states);
        s = m_states.getState(t);
    }

    if (t-m_last_utime < 0.5 * 1E6) return 0;

    // Frame distance check
    // note (hj) -  temporarily disabling distance check
    //              because of how link proposals are handled
    /*
      Frame* nearest = m_map.findClosestFrame(Pose3d(s.x,s.y,s.z,s.heading,s.pitch,s.roll));
      if (nearest)
      {
      State ns = nearest->state();
      if (    abs(ns.x-s.x) < 0.5
      && abs(ns.y-s.y) < 0.5
      && abs(ns.z-s.z) < 0.5
      && abs(ns.heading-s.heading) < 5.0/180.0*M_PI
      )
      {
      return 0;
      }
      }
    */

    std::vector<isam::Point2d>* points = m_scanBuilder.getPoints(didsonFrame);

    if (points->size() < MIN_POINTS) {
        delete points;
        return 0;
    }

    hauv::NDT* ndt = new hauv::NDT(*points, NDT_CELL_SIZE, NDT_MIN_EIGEN_RATIO);
    Frame* f = new Frame(s, ndt, *points, sonarImage);
    m_map.addFrame(f);

    m_last_utime = t;

    // Publish an LCM add message
    std::cout << "-- Add a new node " << t << std::endl;
    perllcm::heartbeat_t node;
    node.utime = t;
    m_lcm->publish(m_sonar_add_node_channel, &node);
    m_node_added_time = getTime();

    // @todo Send keyframe to the viewer

    return f;
}

void 
SonarClient::addScan(IplImage* image,
                     const std::vector<isam::Point2d> & points,
                     const isam::Pose2d & pose,
                     const CvScalar & color,
                     const MapScale & ms)
{
    // Position offsets of sensor
    // TODO: Put transformation between sensor and vehicle frame into config
    double x_offset =  0.0;
    double y_offset =  0.0; // 0.20;

    // Draw a scan
    for (size_t k = 0; k<points.size(); ++k) {
        isam::Point2d p = pose.transform_from(points[k]);
        double x,y;

        x = p.x()-x_offset;
        y = p.y()-y_offset;

        int i = ((-x)/ms.hs + ms.y0);  // Vertical
        int j = (( y)/ms.ws + ms.x0);  // Horizontal
        if (i>=0 && i<image->height && j>=0 && j<image->width) {
            image->imageData[i*image->widthStep + j*image->nChannels + 0] = color.val[0];
            image->imageData[i*image->widthStep + j*image->nChannels + 1] = color.val[1];
            image->imageData[i*image->widthStep + j*image->nChannels + 2] = color.val[2];
        }
    }
}


void SonarClient::showTargets()
{
    int width = 200;
    int height = 180;
    boost::mutex::scoped_lock lock(m_mutex_targets);

    IplImage* targetsImage = cvCreateImage(cvSize(2*width, 5*height), IPL_DEPTH_8U, 3);
    cvZero(targetsImage);

    // cvCvtColor(A->image(), displayImage, CV_GRAY2RGB);
    char statusText[512];

    //State s = m_lastFrame->state();
    // Save information from last frame
    //m_targetFile << s.utime << " "
    //             << s.rel_x << " "
    //             << s.rel_y << " "

    int n = 0;
    for (std::deque<Frame*>::iterator it = m_targets.begin(); it != m_targets.end(); ++it)
        {
            Frame* f = *it;
            State s = f->state();

            int col = n%2;
            int row = n/2;

            cvSetImageROI(targetsImage, cvRect(col*width, height*row, width, height-20));
            IplImage* thumbnail = cvCreateImage(cvSize(f->image()->width, f->image()->height), IPL_DEPTH_8U, 3);
            cvCvtColor(f->image(), thumbnail, CV_GRAY2RGB);
            cvResize(thumbnail, targetsImage);

            cvSetImageROI(targetsImage, cvRect(0,0,targetsImage->width, targetsImage->height));
            sprintf(statusText,"x:%.2f y:%.2f", s.rel_x, s.rel_y);
            cvPutText(targetsImage, statusText, cvPoint(10+col*width, height*(1+row) - 10), &g_font, cvScalar(0,255,0));

            ++n;

            cvReleaseImage(&thumbnail);
        }

    cvSetImageROI(targetsImage, cvRect(0, 0, targetsImage->width, targetsImage->height));
    cvShowImage("TargetsImage", targetsImage);
    cvReleaseImage(&targetsImage);
}

double
SonarClient::getTime()
{
    return timestamp_to_double(timestamp_now());
}

void
SonarClient::sendImageColor(bot_lcmgl_t* lcmgl, const uint8_t* image, int width, int height, int stride, double hpos, double vpos)
{
    bot_lcmgl_matrix_mode(lcmgl, GL_PROJECTION);
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_load_identity(lcmgl);
    bot_lcmgl_ortho(lcmgl, -1, 1, -1, 1, -1, 1);
    bot_lcmgl_matrix_mode(lcmgl, GL_MODELVIEW);
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_load_identity(lcmgl);
    bot_lcmgl_push_attrib(lcmgl, GL_DEPTH_BUFFER_BIT);
    bot_lcmgl_disable(lcmgl, GL_DEPTH_TEST);
    bot_lcmgl_color3f(lcmgl, 1,1,1);
    int color_texid = bot_lcmgl_texture2d(lcmgl, image,
                                          width, height, stride,
                                          BOT_LCMGL_RGB, BOT_LCMGL_UNSIGNED_BYTE, BOT_LCMGL_COMPRESS_NONE);

    float w = 0.5;
    float h = (float)w * ((float)height/width);

    bot_lcmgl_texture_draw_quad(lcmgl, color_texid,
                                1-w + hpos, 1   - vpos, 0,
                                1-w + hpos, 1-h - vpos, 0,
                                1   + hpos, 1-h - vpos, 0,
                                1   + hpos, 1   - vpos, 0);

    bot_lcmgl_pop_attrib(lcmgl);
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_matrix_mode(lcmgl, GL_PROJECTION);
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_matrix_mode(lcmgl, GL_MODELVIEW);

    bot_lcmgl_switch_buffer(lcmgl);
}

void 
SonarClient::sendImageBW(bot_lcmgl_t* lcmgl, const uint8_t* gray, int width, int height, int stride, double hpos, double vpos)
{
    bot_lcmgl_matrix_mode(lcmgl, GL_PROJECTION);
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_load_identity(lcmgl);
    bot_lcmgl_ortho(lcmgl, -1, 1, -1, 1, -1, 1);
    bot_lcmgl_matrix_mode(lcmgl, GL_MODELVIEW);
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_load_identity(lcmgl);
    bot_lcmgl_push_attrib(lcmgl, GL_DEPTH_BUFFER_BIT);
    bot_lcmgl_disable(lcmgl, GL_DEPTH_TEST);
    bot_lcmgl_color3f(lcmgl, 1,1,1);
    int gray_texid = bot_lcmgl_texture2d(lcmgl, gray,
                                         width, height, stride,
                                         BOT_LCMGL_LUMINANCE, BOT_LCMGL_UNSIGNED_BYTE, BOT_LCMGL_COMPRESS_NONE);

    float w = 0.5;
    float h = (float)w * ((float)height/width);

    bot_lcmgl_texture_draw_quad(lcmgl, gray_texid,
                                1-w + hpos, 1   - vpos, 0,
                                1-w + hpos, 1-h - vpos, 0,
                                1   + hpos, 1-h - vpos, 0,
                                1   + hpos, 1   - vpos, 0);

    bot_lcmgl_pop_attrib(lcmgl);
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_matrix_mode(lcmgl, GL_PROJECTION);
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_matrix_mode(lcmgl, GL_MODELVIEW);

    bot_lcmgl_switch_buffer(lcmgl);
}

// lcm callbacks
void 
SonarClient::add_node_ack_t_callback (const lcm::ReceiveBuffer *rbuf,
                                      const std::string &channel,
                                      const perllcm::isam_add_node_ack_t *msg)
{
    //std::cout << "ack back on " << msg->utime << std::endl;

    if (msg->sensor_id & perllcm::isam_vlink_t::SENSOR_SONAR) {

        // bookkeeping utimelist for valid sonar nodes
        m_utimelist.push_back (msg->utime);

        // request entire pose (mu) for link proposal
        perllcm::isam_request_state_t *rq_st = new perllcm::isam_request_state_t();
        rq_st->utime = timestamp_now ();
        rq_st->requester = perllcm::isam_vlink_t::SENSOR_SONAR;
        rq_st->state_type = perllcm::isam_request_state_t::POSE;
        rq_st->n          = m_utimelist.size();

        std::deque<int64_t>::const_iterator it;
        for (it = m_utimelist.begin(); it != m_utimelist.end(); it++)
            {
                int64_t sonar_node_utime = *it;
                rq_st->variables.push_back (sonar_node_utime);        
            }
        m_lcm->publish (m_isam_rq_st_channel, rq_st);
    }
}

void
SonarClient::proposelink (int64_t utime1, int64_t utime2, double delta[6])
{
    //double cov[36];
    //for(int i=0;i<36;i++)cov[i]=0.0;
    //cov[0] = cov[7] = cov[14] = 0.01*0.01;//0.001*0.001;
    //cov[21] = cov[28] = cov[35] = 0.01*0.01;//(0.1*DTOR)*(0.1*DTOR);

    boost::mutex::scoped_lock lock(m_mutex);
    perllcm::isam_plink_t *link = new perllcm::isam_plink_t ();
    link->utime_i = utime1;
    link->utime_j = utime2;
    link->prior = 1;
    //link->x_ji;  // sonar does not use actual mu and cov
    link->link_id = perllcm::isam_vlink_t::LINK_SONAR2D;
    link->sensor_id = perllcm::isam_vlink_t::SENSOR_SONAR;
    m_links.push_back (link);
    m_newdata.notify_one();
}

void 
SonarClient::return_state_t_callback (const lcm::ReceiveBuffer *rbuf,
                                      const std::string &channel,
                                      const perllcm::isam_return_state_t *msg)
{
    if (msg->requester != perllcm::isam_vlink_t::SENSOR_SONAR) return;

    size_t n = msg->n;
    size_t npair = n - 1;

    int64_t utime_j = msg->timestamps[npair];
    //std::cout << "mu return: " << n << " nodes in the graph for the last node w/ t = " << utime_j << std::endl;
    if (n < 2) return;

    // current utime and pose, j=current idx i=past nodes
    double x_lvj[6], x_lvi[6], delta[6];
    std::copy(msg->mu[npair].begin(), msg->mu[npair].end(), x_lvj);
    //printf ("[%g,%g,%g,%g,%g,%g]\n",x_lvj[0],x_lvj[1],x_lvj[2],x_lvj[3],x_lvj[4],x_lvj[5]);

    // then, do normal candidates
    size_t num_request = 0;
    for (size_t i=0; i<npair; i++) { // for each x_lci
        int64_t utime_i = msg->timestamps[i];
        std::copy(msg->mu[i].begin(), msg->mu[i].end(), x_lvi);
        ssc_tail2tail (delta, NULL, x_lvj, x_lvi);      // relative vehicle pose

        double dist2 = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2];
        int64_t dt   = fabs (utime_j - utime_i);
        double range_thresh2 = PLINK_DIST * PLINK_DIST;

        if ((dist2 < range_thresh2) && dt < 60.0*1e6) {
            proposelink (utime_i, utime_j, delta);
            num_request++;
        }
    } // for each x_lvi
}

void 
SonarClient::isam_cmd_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                  const std::string&  channel,
                                  const perllcm::isam_cmd_t *msg)
{
    // SAVE
    // -----------------------------------------------------------
    if (msg->mode == perllcm::isam_cmd_t::MODE_SAVE) {
    }
    // LOAD
    // -----------------------------------------------------------
    if (msg->mode == perllcm::isam_cmd_t::MODE_LOAD) {
    }

    if (msg->mode == perllcm::isam_cmd_t::MODE_START) {
        m_active = true;
    }

    if (msg->mode == perllcm::isam_cmd_t::MODE_WAIT) {
        m_active = false;
    }
}

void 
SonarClient::wp_goto_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                 const std::string&  channel,
                                 const hauv::wp_goto_t *msg)
{
    std::cout << "on_wp_goto: " << msg->mode << " " << msg->utime_target << std::endl;
    // find closest frame
    Frame* f = m_map.findClosestFrame(msg->utime_target);

    if (f!=NULL) {
        // send the image to the viewer
        IplImage* img = f->image();
        sendImageBW(lcmgl_target, (unsigned char*)img->imageData, img->width, img->height, img->widthStep, -1.5, 0. );
    }
}


void 
SonarClient::hauv_bs_pit_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                     const std::string& channel, 
                                     const hauv::bs_pit_t *msg)
{  
    boost::mutex::scoped_lock lock(m_mutex_states);
    m_states.add(msg);
}

void 
SonarClient::hauv_bs_rnv_2_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                       const std::string& channel, 
                                       const hauv::bs_rnv_2_t *msg)
{  
    boost::mutex::scoped_lock lock(m_mutex_states);
    m_states.add(msg);
}

void 
SonarClient::hauv_bs_cnv_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                     const std::string& channel, 
                                     const hauv::bs_cnv_t *msg)
{
    boost::mutex::scoped_lock lock(m_mutex_states);
    m_states.add(msg);              
}

void 
SonarClient::hauv_bs_dvl_2_t_callback (const lcm::ReceiveBuffer *rbuf,
                                       const std::string& channel, 
                                       const hauv::bs_dvl_2_t *msg)
{
    boost::mutex::scoped_lock lock(m_mutex_states);
    m_states.add(msg);
}

void
SonarClient::hauv_didson_t_callback (const lcm::ReceiveBuffer *rbuf, 
                                     const std::string&  channel,
                                     const hauv::didson_t *msg )
{
    if (m_active) {
        boost::mutex::scoped_lock lock(m_mutex);
        hauv::didson_t *didson = new hauv::didson_t();
        *didson = *msg;
        m_didson.push_back(didson);
        m_newdata.notify_one();
    }
}

// Frame and map methods
//--------------------------------------------------------------
Frame::Frame(const State & state,
             hauv::NDT* ndt,
             const std::vector<isam::Point2d> & points,
             IplImage* sonarImage) :
    m_state(state),
    m_ndt(ndt),
    m_scan(points)
{
    m_sonarImage = cvCloneImage(sonarImage);
}

Frame::~Frame()
{
    cvReleaseImage(&m_sonarImage);
    delete m_ndt;
}

// Map methods
Map::Map()
{
}

Map::~Map()
{
    clear();
}

void 
Map::addFrame(Frame* frame)
{
    m_frames[frame->state().utime] = frame;
}

Frame* 
Map::findClosestFrame(const isam::Pose3d & pose)
{
    FrameMap::iterator it;
    Frame* closest = 0;
    double d = -1.0;

    for (it = m_frames.begin(); it != m_frames.end(); ++it) {
        Frame* f = (*it).second;
        State s = f->state();
        isam::Pose3d p(s.x, s.y, s.z, s.heading, s.pitch, s.roll );
        isam::Pose3d delta = pose.ominus(p);
        // @todo might consider scaling angles vs translation in distance measurement
        double r =   delta.x()*delta.x() + delta.y()*delta.y() + delta.z()*delta.z()
            + delta.yaw()*delta.yaw() + delta.pitch()*delta.pitch() + delta.roll()*delta.roll();

        if (d<0.0 || r<d)
            {
                d = r;
                closest = f;
            }
    }
    return closest;
}

Frame* 
Map::findClosestFrame(int64_t utime)
{
    FrameMap::iterator it;
    Frame* closest = 0;
    double d = -1.0;

    for (it = m_frames.begin(); it != m_frames.end(); ++it) {
        Frame* f = (*it).second;

        State s = f->state();
        double r = fabs(s.img_utime-utime);
        if (d<0.0 || r<d)
            {
                d = r;
                closest = f;
            }
    }
    return closest;
}

Frame* 
Map::getFrame(int64_t utime)
{
    FrameMap::iterator it = m_frames.find(utime);
    if (it != m_frames.end())
        return (*it).second;
    else
        return 0;
}

void 
Map::clear()
{
    for (FrameMap::iterator it = m_frames.begin(); it != m_frames.end(); ++it)
        {
            delete (*it).second;
        }
    m_frames.clear();
}

