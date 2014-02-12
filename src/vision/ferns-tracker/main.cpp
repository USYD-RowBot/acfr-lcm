/*** ***/
//PERL
//Joe DeGol, UMich SROP
/*** ***/


/*** Includes ***/
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include <signal.h>
#include <dirent.h>
#include <inttypes.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/perllcm_ferns_tracker_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#include "perls-math/fasttrig.h"
#include "perls-math/so3.h"

#include "ferns/planar_pattern_detector_builder.h"
#include "ferns/template_matching_based_tracker.h"

#define DEBUG 0

/*** namespaces ***/
using namespace std;
using namespace cv;


/*** define config structure loaded from cfg file ***/
typedef struct _config_t config_t;
struct _config_t 
{
    //camera intrinsics
    CvMat *intrinsic;
    CvMat *invIntrinsic;
    CvMat *distortion;
	
    //lcm channels
    char bot_core_image_t_channel[256];
    char ferns_track_pose_t_channel[256];
	
    //model image
    string model_image;
	
    //ferns-tracker-parameters
    int max_num_pts_on_model;
    int num_of_generated_images_to_find_stable_pts;
    double min_num_of_views_rate;
    int patch_size;
    int yape_radius;
    int number_of_octaves;
    int number_of_ferns;
    int number_of_tests_per_fern;
    int number_of_samples_for_refinement;
    int number_of_samples_for_test;
    int max_num_pts_to_detect;
    
    //target parameters
    double target_length;
    double target_width;
};


/*** define state structure ***/
typedef struct _state_t state_t;
struct _state_t
{
    //lcm
    lcm_t *lcm;
	
    //image frames
    IplImage *color_frame, *frame, *gray_frame;
    IplImage *u_map_x, *u_map_y;
	
    //Matrices
    double XYZ[3];
    double RPH[3];
    homography06 *myH;
	
    //detector, tracker
    planar_pattern_detector *detector;
    template_matching_based_tracker *tracker;
	
    //primitives
    int64 timer;
    int64 now;
    bool fileOut;
    bool done;
    bool is_daemon;
    bool show_tracking;
    double fps;
	
    //output file
    ofstream myfile;
};


/*** Init the state & config structures ***/
state_t state;
config_t config = {NULL,NULL,NULL,"","","",0,0,0,0,0,0,0,0,0,0,0,0.0,0.0};



/***************************/
//draws rectangle
/***************************/
static void
draw_quadrangle (IplImage *frame, 
                 int u0, int v0, int u1, int v1, int u2, int v2, int u3, int v3, 
                 CvScalar color, int thickness = 1)
{
    cvLine (frame, cvPoint (u0, v0), cvPoint (u1, v1), color, thickness);
    cvLine (frame, cvPoint (u1, v1), cvPoint (u2, v2), color, thickness);
    cvLine (frame, cvPoint (u2, v2), cvPoint (u3, v3), color, thickness);
    cvLine (frame, cvPoint (u3, v3), cvPoint (u0, v0), color, thickness);
}

//wrappers to draw quadrangle
static void
draw_detected_position(IplImage *frame, planar_pattern_detector *detector)
{
    draw_quadrangle (frame,
                     detector->detected_u_corner[0], detector->detected_v_corner[0],
                     detector->detected_u_corner[1], detector->detected_v_corner[1],
                     detector->detected_u_corner[2], detector->detected_v_corner[2],
                     detector->detected_u_corner[3], detector->detected_v_corner[3],
                     cvScalar (0,255,0), 3);
}


/********************************/
//Finding Pose
/********************************/
static void
decomposeHomography2 (CvMat *H_L) //method follows paper
{
    //H_L = [h1 h2 h3]
    CvMat *h1 = cvCreateMat (3, 1, CV_32FC1);
    CvMat *h2 = cvCreateMat (3, 1, CV_32FC1);
    CvMat *h3 = cvCreateMat (3, 1, CV_32FC1);
    cvGetCol (H_L, h1, 0);
    cvGetCol (H_L, h2, 1);
    cvGetCol (H_L, h3, 2);

    //r1 = lambda * K^-1 * h1
    int lambda = 1;
    CvMat *r1 = cvCreateMat (3, 1, CV_32FC1);
    cvMatMul (config.invIntrinsic, h1, r1);
    cvConvertScale (r1, r1, lambda);
	
    //r2 = lambda * K^-1 * h2
    CvMat *r2 = cvCreateMat (3, 1, CV_32FC1);
    cvMatMul (config.invIntrinsic, h2, r2);
    cvConvertScale (r2, r2, lambda);
	
    //r3 = r1 x r2
    CvMat *r3 = cvCreateMat (3, 1, CV_32FC1);
    cvCrossProduct (r1, r2, r3);
	
    //R = [r1 r2 r3]
    CvMat *R  = cvCreateMat (3, 3, CV_32FC1);
    CvMat *Q  = cvCreateMat (3, 3, CV_32FC1);
    CvMat *S  = cvCreateMat (3, 3, CV_32FC1);
    CvMat *U  = cvCreateMat (3, 3, CV_32FC1);
    CvMat *V  = cvCreateMat (3, 3, CV_32FC1);
    cvmSet (Q, 0, 0, cvmGet (r1, 0, 0)); cvmSet (Q, 0, 1, cvmGet (r2, 0, 0)); cvmSet(Q, 0, 2, cvmGet (r3, 0, 0));
    cvmSet (Q, 1, 0, cvmGet (r1, 1, 0)); cvmSet (Q, 1, 1, cvmGet (r2, 1, 0)); cvmSet(Q, 1, 2, cvmGet (r3, 1, 0));
    cvmSet (Q, 2, 0, cvmGet (r1, 2, 0)); cvmSet (Q, 2, 1, cvmGet (r2, 2, 0)); cvmSet(Q, 2, 2, cvmGet (r3, 2, 0));
    cvSVD (Q, S, U, V);
    cvMatMul (U, V, R);
	
    //t = lambda * K^-1 * h3
    CvMat *t  = cvCreateMat (3, 1, CV_32FC1);
    cvMatMul (config.invIntrinsic, h3, t);
	
    //lambda = 1/||K^-1*h1|| = 1/||K^-1*h2||
    CvMat *lambdaTemp = cvCreateMat (3, 1, CV_32FC1);
    cvMatMul (config.invIntrinsic, h1, lambdaTemp);
    //magnitude of lambdaTemp
    lambda = 1.0 / sqrtf ( pow(cvmGet(lambdaTemp,0,0),2) + pow(cvmGet(lambdaTemp,1,0),2) + pow(cvmGet(lambdaTemp,2,0),2) );
    cout<<lambda<<endl;
    cvConvertScale (t, t, lambda);
	
    //rotation to roll pitch heading
    double R_data[9];
    for (int i=0, k=0; i<3; i++)
        for (int j=0; j<3; j++, k++)
            R_data[k] = cvmGet (R, i, j);
    so3_rot2rph (R_data, state.RPH);

#if DEBUG
    cout<<"R:\n"
        <<cvmGet(R,0,0)<<" "<<cvmGet(R,0,1)<<" "<<cvmGet(R,0,2)<<endl
        <<cvmGet(R,1,0)<<" "<<cvmGet(R,1,1)<<" "<<cvmGet(R,1,2)<<endl
        <<cvmGet(R,2,0)<<" "<<cvmGet(R,2,1)<<" "<<cvmGet(R,2,2)<<endl<<endl;
	
    cout<<"T:\n"
        <<cvmGet(t,0,0)<<" "<<cvmGet(t,1,0)<<" "<<cvmGet(t,2,0)<<endl<<endl;
#endif
	
    state.XYZ[0] = cvmGet (t, 0, 0);
    state.XYZ[1] = cvmGet (t, 1, 0);
    state.XYZ[2] = cvmGet (t, 2, 0);

    //clean up
    cvReleaseMat (&r1);
    cvReleaseMat (&r2);
    cvReleaseMat (&r3);
    cvReleaseMat (&t); 
    cvReleaseMat (&h1);
    cvReleaseMat (&h2);
    cvReleaseMat (&h3);
    cvReleaseMat (&R); 
    cvReleaseMat (&Q); 
    cvReleaseMat (&S); 
    cvReleaseMat (&U); 
    cvReleaseMat (&V); 
}

#if 0 //disable for now
static void
findPose (void) //method uses opencv function call cvFindExtrinsicCameraParams2
{
    /*** find number of stable points found by detector ***/
    int size = 1;
    for (int i=0; i<state.detector->number_of_model_points; i++) {
        if (state.detector->model_points[i].class_score > 0)
            size++;
    }
	
    /*** make matrices from size ***/
    CvMat *objectPoints = cvCreateMat (size, 3, CV_32FC1);
    CvMat *imagePoints = cvCreateMat (size, 2, CV_32FC1);
	
    /*** populate objectPoints & modelPoints from detector information ***/
    for (int i=0, j=0; i<state.detector->number_of_model_points; i++) {
        if (state.detector->model_points[i].class_score > 0) {
            cvmSet (objectPoints, j, 0, state.detector->model_points[i].fr_u());
            cvmSet (objectPoints, j, 1, state.detector->model_points[i].fr_v());
            cvmSet (objectPoints, j, 2, 0);
            cvmSet (imagePoints, j, 0, state.detector->model_points[i].potential_correspondent->fr_u());
            cvmSet (imagePoints, j, 1, state.detector->model_points[i].potential_correspondent->fr_v());
            j++;
        }
    }
	
    /*** finds pose from object and image points ***/
    CvMat *t = cvCreateMat (3, 1, CV_32FC1);
    CvMat *r = cvCreateMat (3, 1, CV_32FC1);
    CvMat *R = cvCreateMat (3, 3, CV_32FC1);
    cvFindExtrinsicCameraParams2 (objectPoints, imagePoints, config.intrinsic, config.distortion, r, t);
    cvRodrigues2 (r, R, 0); //converts r from rodriguez to rotation matrix R
	
    /*** release matrices ***/
    cvReleaseMat (&r);
    cvReleaseMat (&t);
    cvReleaseMat (&R);
    cvReleaseMat (&objectPoints);
    cvReleaseMat (&imagePoints);
}
#endif

/*************************************************************************/
// Calls the detector for detection and calls the drawings functions
/*************************************************************************/
static void
detect_and_draw (int64_t utime)
{
    /*** Look for target ***/
    state.detector->detect (state.frame, state.myH);
    cvCvtColor (state.frame, state.color_frame, CV_GRAY2BGR ); //convert frame to color

    if (state.detector->pattern_is_detected) { //target found
        /*** init tracker with detector corner points of target ***/
        state.tracker->initialize (state.detector->detected_u_corner[0], state.detector->detected_v_corner[0],
                                   state.detector->detected_u_corner[1], state.detector->detected_v_corner[1],
                                   state.detector->detected_u_corner[2], state.detector->detected_v_corner[2],
                                   state.detector->detected_u_corner[3], state.detector->detected_v_corner[3]);
		
        /*** if TRUE: output detected coordinates to output file ***/	
        if( state.fileOut ) {
            state.myfile << 
                utime <<" : "<< state.detector->detected_u_corner[0] <<" "<< state.detector->detected_v_corner[0] <<"\n"<<
                utime <<" : "<< state.detector->detected_u_corner[1] <<" "<< state.detector->detected_v_corner[1] <<"\n"<<
                utime <<" : "<< state.detector->detected_u_corner[2] <<" "<< state.detector->detected_v_corner[2] <<"\n"<<
                utime <<" : "<< state.detector->detected_u_corner[3] <<" "<< state.detector->detected_v_corner[3] <<"\n";
        }
		
        /*** draw border on detected target ***/
        draw_detected_position (state.color_frame, state.detector);
		
        /*** Find pose ***/
        decomposeHomography2 (state.myH);
        //findPose ();
    }
    else { //target not found
        /*** if TRUE: output 0 coordinates to output file ***/
        if (state.fileOut) {
            state.myfile << utime <<" : 0.00000 0.00000\n"
                         << utime <<" : 0.00000 0.00000\n"
                         << utime <<" : 0.00000 0.00000\n"
                         << utime <<" : 0.00000 0.00000\n";
        }
    }
	
    /*** Show frame with detected target highlighted if found ***/
    if (state.show_tracking) {
        cvShowImage ("ferns-tracking", state.color_frame);
        if (utime == -9)
            cvWaitKey (0); //utime == -9 is for singleImage, wait until user closes image
        else
            cvWaitKey (10);
    }
}


/******************************************/
//send message for lcm publishing
/******************************************/
static void
send_message (lcm_t *lcm, int64_t utime, bool found)
{
    perllcm_ferns_tracker_t msg = {0};
    msg.utime = utime;
    msg.found = found;
    msg.pose[0] = state.XYZ[0];
    msg.pose[1] = state.XYZ[1];
    msg.pose[2] = state.XYZ[2];
    msg.pose[3] = state.RPH[0];
    msg.pose[4] = state.RPH[1];
    msg.pose[5] = state.RPH[2];
    msg.number_of_inliers = state.detector->number_of_inliers;
    msg.number_of_corr = state.detector->number_of_correspondences;
    msg.rms_error = state.detector->inlier_rms_rproj_error;
    for (int i=0, k=0; i<3; i++)
        for (int j=0; j<3; j++, k++)
            msg.H[k] = cvmGet (state.myH, i, j);

    perllcm_ferns_tracker_t_publish (lcm, config.ferns_track_pose_t_channel, &msg);
}



/*****************************/
//mesage received handler
/*****************************/
static void
my_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
            const bot_core_image_t *msg, void *user)
{   
    /*** gray_frame = incoming image data, frame = undistorted ***/
    state.gray_frame->imageData = (char *) msg->data;
    state.gray_frame->imageDataOrigin = state.gray_frame->imageData;
    cvRemap (state.gray_frame, state.frame, state.u_map_x, state.u_map_y, 
             CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));
    
    /*** find target and highlight it ***/
    detect_and_draw (msg->utime);
    
    /*** calculates fps and outputs ***/
#if DEBUG
    state.now = cvGetTickCount();
    state.fps = 1e6 * cvGetTickFrequency() / double (state.now-state.timer);
    state.timer = state.now;
    clog << "Detection frame rate: " << state.fps << " fps\r";
#endif

    /*** send message to lcmOut ***/
    send_message (state.lcm, msg->utime, state.detector->pattern_is_detected);
}


/****************************************************/
//image handler, handles images for inputMode 1 & 2
/****************************************************/
static void
imageHandler (int64_t flag)
{
    /*** undistort ***/
    cvRemap (state.gray_frame, state.frame, state.u_map_x, state.u_map_y,
             CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));
    
    /*** find target and highlight it ***/
    detect_and_draw (flag); //-9 for utime, used for waiting for user input when showing image
    
    /*** send message to lcmOut ***/
    send_message (state.lcm, 0, state.detector->pattern_is_detected);
}


/******************************/
//interupt handler
/******************************/
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("Quitting...\n");
    if (state.done) {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        state.done = true;
}


/******************************************/
//Loads config file, camera intrinsics
/******************************************/
static void
ferns_track_load_cfg (void)
{
    /*** Init matrices ***/
    config.intrinsic = cvCreateMat(3,3,CV_32FC1);
    config.invIntrinsic = cvCreateMat(3,3,CV_32FC1);
    config.distortion = cvCreateMat(5,1,CV_32FC1);
  
    /*** Open ferns-track.cfg with bot param ***/
    BotParam *param = bot_param_new_from_file ("../config/ferns-track.cfg");
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", "../config/ferns-track.cfg");
        exit (EXIT_FAILURE);
    }    
    
    /*** read in intrinsic and distortion from cfg file ***/
    double K[9];
    bot_param_get_double_array (param, "ferns.intrinsic", K, 9);
    for (int i=0, k=0; i<3; i++)
        for (int j=0; j<3; j++, k++)
            cvmSet (config.intrinsic, i, j, K[k]);
    cvInvert (config.intrinsic, config.invIntrinsic);
  
    double D[5];
    bot_param_get_double_array (param, "ferns.distortion", D, 5);
    for (int i=0; i<5; i++)
        cvmSet (config.distortion, i, 0, D[i]);

  
    /*** read in LCM input and output channels ***/
    strcpy (config.bot_core_image_t_channel, bot_param_get_str_or_fail (param, "ferns.bot_core_image_t_channel"));
    strcpy (config.ferns_track_pose_t_channel, bot_param_get_str_or_fail (param, "ferns.ferns_track_pose_t_channel"));
  
    /*** model image ***/
    config.model_image = bot_param_get_str_or_fail (param, "ferns.model_image");
  
    /*** ferns default parameters ***/
    bot_param_get_int (param, "ferns.max_num_pts_on_model", &config.max_num_pts_on_model);
    bot_param_get_int (param, "ferns.num_of_generated_images_to_find_stable_pts", &config.num_of_generated_images_to_find_stable_pts);
    bot_param_get_double (param, "ferns.min_num_of_views_rate", &config.min_num_of_views_rate);
    bot_param_get_int (param, "ferns.patch_size", &config.patch_size);
    bot_param_get_int (param, "ferns.yape_radius", &config.yape_radius);
    bot_param_get_int (param, "ferns.number_of_octaves", &config.number_of_octaves);
    bot_param_get_int (param, "ferns.number_of_ferns", &config.number_of_ferns);
    bot_param_get_int (param, "ferns.number_of_tests_per_fern", &config.number_of_tests_per_fern);
    bot_param_get_int (param, "ferns.number_of_samples_for_refinement", &config.number_of_samples_for_refinement);
    bot_param_get_int (param, "ferns.number_of_samples_for_test", &config.number_of_samples_for_test);
    bot_param_get_int (param, "ferns.max_num_pts_to_detect", &config.max_num_pts_to_detect);
    bot_param_get_double (param, "ferns.target_length", &config.target_length);
    bot_param_get_double (param, "ferns.target_width", &config.target_width);
  
    cout<<endl<<"K:" <<endl;
    cout<< cvmGet (config.intrinsic, 0, 0) <<" "<< cvmGet (config.intrinsic, 0, 1) <<" "<< cvmGet (config.intrinsic, 0, 2) <<endl;
    cout<< cvmGet (config.intrinsic, 1, 0) <<" "<< cvmGet (config.intrinsic, 1, 1) <<" "<< cvmGet (config.intrinsic, 1, 2) <<endl;
    cout<< cvmGet (config.intrinsic, 2, 0) <<" "<< cvmGet (config.intrinsic, 2, 1) <<" "<< cvmGet (config.intrinsic, 2, 2) <<endl<<endl;

    cout<<"distCoeffs:"<<endl;
    cout<< cvmGet (config.distortion, 0, 0)<<" "<< cvmGet (config.distortion, 1, 0)<<" "<< cvmGet (config.distortion, 2, 0)<<" "<< 
           cvmGet (config.distortion, 3, 0)<<" "<< cvmGet (config.distortion, 4, 0)<<endl<<endl;
}



int
main (int argc, char *argv[])
{
    /*** locals ***/
    string logFile = "fernlog.txt";
    affine_transformation_range range;
    char singleImage[NAME_MAX];
    char dirName[PATH_MAX];
    int inputMode = 0; //0 = lcm, 1 = dir of images, 2 = single image
  
    /*** init struct primitives ***/
    state.timer = cvGetTickCount();
    state.fileOut = false;
    state.done = false;
    state.is_daemon = false;
    state.show_tracking = false;
  
    /*** read config file for camera intrinsics***/
    ferns_track_load_cfg ();
  
    /*** install custom signal handler ***/
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
  
  
    /*** Read in the command line options ***/
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "Ferns Planar Tracker: Outputs the pose of the camera relative to the tracked image plane.");
    getopt_add_bool   (gopt, '\0', "daemon",       0,  "Run as system daemon");
    getopt_add_bool   (gopt, 'h',  "help",         0,  "Display Help");
    getopt_add_bool   (gopt, 's',  "show",         0,  "Opens Window to show tracking");
    getopt_add_string (gopt, 't',  "train",        "", "Input image for training or loading");
    getopt_add_string (gopt, 'i',  "channel-in",   "", "LCM input channel");
    getopt_add_string (gopt, 'o',  "channel-out",  "", "LCM output channel");
    getopt_add_string (gopt, 'd',  "directory",    "", "Directory of images");
    getopt_add_string (gopt, 'I',  "image",        "", "Single input image");
    getopt_add_string (gopt, 'f',  "logFile",      "", "File to log output");
  
    //ferns parameters, the defaults are set based on config file if possible, if not, defaults are set here.
    int tmpi=0;   double tmpf=0;   char tmps[32]={'\0'};

    getopt_add_spacer (gopt, "\nFerns Parameters");
    tmpi = (config.max_num_pts_on_model != 0) ? config.max_num_pts_on_model : 400;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'M', "maxModelPts", tmps, "Max_num_pts_on_model");
  
    tmpi = (config.num_of_generated_images_to_find_stable_pts != 0) ? config.num_of_generated_images_to_find_stable_pts : 5000;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'G', "genImages", tmps, "Num_of_generated_images_to_find_stable_points");

    tmpf = (config.min_num_of_views_rate != 0) ? config.min_num_of_views_rate : 0.0;
    snprintf (tmps, sizeof tmps, "%f", tmpf);
    getopt_add_double (gopt, 'V', "viewsRate", tmps, "Min_num_of_views_rate");
  
    tmpi = (config.patch_size != 0) ? config.patch_size : 32;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'P', "patchSize", tmps, "Patch_size");
  
    tmpi = (config.yape_radius != 0) ? config.yape_radius : 7;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'Y', "yapeRadius", tmps, "Yape_radius");
  
    tmpi = (config.number_of_octaves != 0) ? config.number_of_octaves : 4;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'O', "octaves", tmps, "Num_of_octaves");
  
    tmpi = (config.number_of_ferns != 0) ? config.number_of_ferns : 30;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'F', "ferns", tmps, "Num_of_ferns");
  
    tmpi = (config.number_of_tests_per_fern != 0) ? config.number_of_tests_per_fern : 12;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'T', "fernTests", tmps, "Num_of_tests_per_fern");
  
    tmpi = (config.number_of_samples_for_refinement != 0) ? config.number_of_samples_for_refinement : 10000;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'R', "refinementSamples", tmps, "Num_of_samples_for_refinement");
  
    tmpi = (config.number_of_samples_for_test != 0) ? config.number_of_samples_for_test : 200;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'S', "testSamples", tmps, "Num_of_samples_for_test");
  
    tmpi = (config.max_num_pts_to_detect != 0) ? config.max_num_pts_to_detect : 1000;
    snprintf (tmps, sizeof tmps, "%d", tmpi);
    getopt_add_int (gopt, 'D', "detectPts", tmps, "Max_num_pts_to_detect");
  
    tmpf = (config.target_length != 0) ? config.target_length : 0.0;
    snprintf (tmps, sizeof tmps, "%f", tmpf);
    getopt_add_double (gopt, 'L', "targetLength", tmps, "Length of trained target image");
  
    tmpf = (config.target_width != 0) ? config.target_width : 0.0;
    snprintf (tmps, sizeof tmps, "%f", tmpf);
    getopt_add_double (gopt, 'W', "targetWidth", tmps, "Width of trained target image");
  
    getopt_add_example (gopt, "%s", argv[0]);

    //help then exit
    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len != 0) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage  (gopt, NULL);
        exit (EXIT_SUCCESS);
    }
    else
        fasttrig_init ();
  
    //train or load?
    if (getopt_has_flag (gopt, "train")) {
        config.model_image = getopt_get_string (gopt, "train");
        printf ("\nCmd Line Option: Training image input: %s\n", config.model_image.c_str());
    }
    else
        printf ("Training image: %s\n", config.model_image.c_str());
  
    //Input from LCM, Directory of images, or single image
    if (getopt_has_flag (gopt, "channel-in")) {
        strcpy (config.bot_core_image_t_channel, getopt_get_string (gopt, "channel-in"));
        printf ("Cmd Line Option: Receiving input from LCM Channel: %s\n", config.bot_core_image_t_channel);
        inputMode = 0;
    }
    else if (getopt_has_flag (gopt, "directory")) {
        strcpy (dirName, getopt_get_string (gopt, "directory"));
        printf ("Cmd Line Option: Receiving input from directory of images: %s\n", dirName);
        inputMode = 1;
    }
    else if (getopt_has_flag (gopt, "image")) {
        strcpy (singleImage, getopt_get_string (gopt, "image"));
        printf ("Cmd Line Option: Receiving input from single image: %s\n", singleImage);
        state.show_tracking = true;
        inputMode = 2;
    }
    else {
        if (0 < strlen (config.bot_core_image_t_channel))
            cout << "LCM In Channel: " << config.bot_core_image_t_channel << endl;
        else {
            ERROR ("Must enter an input LCM channel, directory of images, or single image");
            exit (EXIT_FAILURE);
        }
    }
  
    //output options
    if (getopt_has_flag (gopt, "channel-out")) {
        strcpy (config.ferns_track_pose_t_channel, getopt_get_string (gopt, "channel-out"));
        printf ("Cmd Line Option: Sending output to LCM Channel: %s\n", config.ferns_track_pose_t_channel);
    }
    else
        cout << "LCM Out Channel: " << config.ferns_track_pose_t_channel << endl;

    if (getopt_has_flag (gopt, "logFile")) {
        logFile = getopt_get_string (gopt, "logFile");
        state.fileOut = true;
        printf ("Logging output to logFile: %s\n", logFile.c_str());
    }
    if (getopt_get_bool (gopt, "show")) {
        state.show_tracking = true;
        printf ("Show Tracking: ON");
    }
    cout<<endl<<endl;

  
    /*** ferns parameters ***/
    config.max_num_pts_on_model                       = getopt_get_int (gopt, "maxModelPts");
    config.num_of_generated_images_to_find_stable_pts = getopt_get_int (gopt, "genImages"); 
    config.min_num_of_views_rate                      = getopt_get_double (gopt, "viewsRate"); 
    config.patch_size                                 = getopt_get_int (gopt, "patchSize"); 
    config.yape_radius                                = getopt_get_int (gopt, "yapeRadius"); 
    config.number_of_octaves                          = getopt_get_int (gopt, "octaves"); 
    config.number_of_ferns                            = getopt_get_int (gopt, "ferns"); 
    config.number_of_tests_per_fern                   = getopt_get_int (gopt, "fernTests"); 
    config.number_of_samples_for_refinement           = getopt_get_int (gopt, "refinementSamples"); 
    config.number_of_samples_for_test                 = getopt_get_int (gopt, "testSamples"); 
    config.max_num_pts_to_detect                      = getopt_get_int (gopt, "detectPts");
    config.target_length                              = getopt_get_double (gopt, "targetLength");
    config.target_width                               = getopt_get_double (gopt, "targetWidth");
  
    cout<<"Ferns Parameters: "<<config.max_num_pts_on_model<<" "<<config.num_of_generated_images_to_find_stable_pts<<" "<<
        config.min_num_of_views_rate<<" "<<config.patch_size<<" "<<config.yape_radius<<" "<<config.number_of_octaves<<
        " "<<config.number_of_ferns<<" "<<config.number_of_tests_per_fern<<" "<<config.number_of_samples_for_refinement<<
        " "<<config.number_of_samples_for_test<<" "<<config.max_num_pts_to_detect<<" "<<config.target_length<<" "<<
        config.target_width<<endl<<endl;
  
    /*** start as daemon if asked ***/
    if (getopt_get_bool (gopt, "daemon")) {
        daemon_fork ();
        state.is_daemon = true;
    }
    else
        state.is_daemon = false;

  
    /*** Init images, distortion Matrices, and distortion Maps ***/
    state.myH = new homography06 (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    state.color_frame = cvCreateImage (cvSize (640, 480), IPL_DEPTH_8U, 3);
    state.gray_frame = cvCreateImage (cvSize (640, 480), IPL_DEPTH_8U, 1);
    state.frame = cvCreateImage (cvSize (640, 480), IPL_DEPTH_8U, 1);
    state.u_map_x = cvCreateImage (cvSize( 640, 480), IPL_DEPTH_32F, 1);
    state.u_map_y = cvCreateImage (cvSize( 640, 480), IPL_DEPTH_32F, 1);
    cvInitUndistortMap (config.intrinsic, config.distortion, state.u_map_x, state.u_map_y);

    /***  LCM init section  ***/
    state.lcm = lcm_create (NULL);
    if (!state.lcm) {
        cout<<"Error creating LCM object"<<endl;
        exit (EXIT_FAILURE);
    }

    /*** init detector ***/
    state.detector = planar_pattern_detector_builder::build_with_cache (config.model_image.c_str(),
                                                                        &range,
                                                                        config.max_num_pts_on_model,
                                                                        config.num_of_generated_images_to_find_stable_pts,
                                                                        config.min_num_of_views_rate,
                                                                        config.patch_size, 
                                                                        config.yape_radius,
                                                                        config.number_of_octaves, 
                                                                        config.number_of_ferns, 
                                                                        config.number_of_tests_per_fern,
                                                                        config.number_of_samples_for_refinement,
                                                                        config.number_of_samples_for_test);
    state.detector->set_maximum_number_of_points_to_detect (config.max_num_pts_to_detect);


    /*** init tracker and train image if needed ***/
    state.tracker = new template_matching_based_tracker ();
    string trackerfn = config.model_image + string (".tracker_data");
  
    if (!state.tracker->load (trackerfn.c_str ())) {
        cout << "Image data not found. Training image instead."<<endl;
        cout << "Training template matching..."<<endl;
        state.tracker->learn (state.detector->model_image,
                              5, // number of used matrices (coarse-to-fine)
                              40, // max motion in pixel used to train to coarser matrix
                              20, 20, // defines a grid. Each cell will have one tracked point.
                              state.detector->u_corner[0], state.detector->v_corner[1], state.detector->u_corner[2], state.detector->v_corner[2],
                              40, 40, // neighbordhood for local maxima selection
                              10000 // number of training samples
            );
        state.tracker->save (trackerfn.c_str ());
    }
    state.tracker->initialize ();

  
    if (state.show_tracking)
        cvNamedWindow ("ferns-tracking", 1);

    /*** open file for logging ***/
    if (state.fileOut)
        state.myfile.open (logFile.c_str (), ios::out | ios::app);
	
    /*** Input from lcm, dir, or image ***/
    bot_core_image_t_subscribe (state.lcm, config.bot_core_image_t_channel, &my_handler, NULL);
    if (inputMode == 0) {//LCM, subscribe and wait
        cout<<"Start LCM In for tracking"<<endl;
        while (!state.done) {
            struct timeval timeout = {0};
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            lcmu_handle_timeout (state.lcm, &timeout);
        }
    }
    else if (inputMode == 1) {//Dir of images
        cout<<"Opening Directory for tracking"<<endl;
	
        //locals
        DIR *dir = NULL;
        int i = 0;
        struct dirent *ent;
        char fileName[256] = "";
        dir = opendir (dirName);
        vector<string> list(10000);
        vector<string>::iterator it;
    
        if (dir) {//dir opened
            while ((ent = readdir (dir)) != NULL && !state.done) {//run through files
                list[i] = std::string(ent->d_name);
                i++;
            }
            sort (list.begin(),list.end());//sort them
		
            for (it=list.begin(); it!=list.end(); ++it)  {
                //create full path of image
                strcat (fileName, dirName);
                strcat (fileName, (*it).c_str());
                
                //load image
                state.gray_frame = cvLoadImage (fileName, CV_LOAD_IMAGE_GRAYSCALE);
		
                //if image was loaded, call tracker
                if (state.gray_frame != 0)
                    imageHandler (timestamp_now());
                fileName[0] = '\0';
            }
            closedir (dir);
        } 
        else {//dir not opened
            ERROR ("Could not open directory: %s", dirName);
            exit (EXIT_FAILURE);
        }
    }
    else {//single image
        cout<<"Single Image Test"<<endl;
	
        //opening image
        state.gray_frame = cvLoadImage (singleImage, CV_LOAD_IMAGE_GRAYSCALE);
        if (state.gray_frame == 0) {
            ERROR ("Cannot load file %s!\n", singleImage);
            exit (EXIT_FAILURE);
        }
    
        // Run tracker
        imageHandler (-9);
    
    }

  
    /*** Shutdown ***/
    clog << endl;
    delete state.detector;
    delete state.tracker;
    if (state.show_tracking)
        cvDestroyWindow ("ferns-tracking");
  
    if (state.fileOut)
        state.myfile.close();

    lcm_destroy (state.lcm);
  
    /*cvReleaseImage (&state.color_frame); //causing a seg fault with cntrl+c, doesnt really matter if they are released anyway
      cvReleaseImage (&state.gray_frame);
      cvReleaseImage (&state.frame);*/ 
    cvReleaseImage (&state.u_map_x);
    cvReleaseImage (&state.u_map_y);
    cvReleaseMat (&config.intrinsic);
    cvReleaseMat (&config.invIntrinsic);
    cvReleaseMat (&config.distortion);

    printf ("Goodbye\n");  
    exit (EXIT_SUCCESS);
}
