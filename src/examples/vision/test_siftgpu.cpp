#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// opencv 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cxcore.h>
// opengl
#include <GL/gl.h>
#include <GL/glu.h>

#ifdef SIFTGPU_FOUND
#include <siftgpu/SiftGPU.h>
#endif

// for timing
#include <sys/time.h>
#include <time.h>

#include "perls-vision/clahs.h"
#include "perls-vision/opencv_util.h"

#define DESCRIPTOR_SIZE 128
#define FEAT_SURF 1
#define FEAT_SIFTGPU 2
#define MAX_FEAT_NUM 1000

using namespace std;

// for timing: tic and toc
int tic_sec, tic_usec;

typedef struct{
    // length of feature vector
    int Nf;
    int type; // FEAT_SURF 1, FEAT_SIFTGPU 2, FEAT_HARRIS 4
  
    // depth prior from bathy
    double	Z[MAX_FEAT_NUM];
    double	covZ[MAX_FEAT_NUM];

    // what we want to keep
    CvMat *uv;    // N x 2 matrix
    CvMat *key;   // N x 128 matrix


}tFeature;

void tic()
{
    struct timeval now;
    // TIC
    gettimeofday(&now, NULL);
    tic_sec = now.tv_sec;
    tic_usec = now.tv_usec;
}

void toc()
{
    // TOC
    float time;
    int toc_sec, toc_usec;
    struct timeval now;

    gettimeofday(&now, NULL);
    toc_sec = now.tv_sec;
    toc_usec = now.tv_usec;
    time = (toc_sec*1000000+toc_usec)-(tic_sec*1000000+tic_usec);
    //printf("time taken = %3.0f usec\n",time);
    printf("%3.4fs\n",time/1000000);

}

void plotFeature(int fignum, IplImage* img, tFeature* feature)
{
    // full size image is large so x 1/scale
    int scale = 2;
    int i;
    int n = feature->Nf;

    // plot options
    int radius = 3;
    int thickness = -1;	// line thickness when pos number, -1 for filled circle

    IplImage* img_plot = 0;
    img_plot = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U, 3);
    cvCvtColor(img, img_plot, CV_GRAY2BGR);

    for (i = 0; i < n; ++i) {
        //const CvSURFPoint* p = (CvSURFPoint*)cvGetSeqElem(feature->kp, i);
        float u = cvmGet(feature->uv,i,0);
        float v = cvmGet(feature->uv,i,1);
        cvCircle(img_plot, cvPoint(u,v), radius, cvScalar(0,0,255,0), thickness,8,0); //0x66, 0x66, 0x66, 0x66
    }

    // resize for viewing
    IplImage* img_plot_resize = 0;
    CvSize dsize = cvGetSize(img);
    dsize.width = dsize.width/scale; dsize.height = dsize.height/scale;
    img_plot_resize = cvCreateImage(dsize,IPL_DEPTH_8U, 3);
    cvResize(img_plot, img_plot_resize, CV_INTER_LINEAR);

    char title[50];
    snprintf(title, 50, "Feature_Plot (%d)",fignum);
    cvNamedWindow( title, CV_WINDOW_AUTOSIZE );
    cvShowImage( title, img_plot_resize);
    cvWaitKey(0);
}

void initialize_image(IplImage* srcimg, IplImage* init_img)
{

    IplImage* imggray = NULL;
    if (srcimg->nChannels == 3) {
        imggray = cvCreateImage (cvGetSize(srcimg),srcimg->depth, 1);
        cvCvtColor (srcimg, imggray, CV_BGR2GRAY);
    }
    else { 
        imggray = cvCloneImage (srcimg);
    }

    IplImage *clahsimg = imggray;
    vis_clahs_opts_t opts = vis_clahs_default_opts ();
    opts.cliplimit = 0.0075;  //getopt_get_double (gopt, "cliplimit"),
    opts.bins = 1024;        //getopt_get_int (gopt, "nbins"),
    opts.tiles[0] = 8, opts.tiles[1] = 10;
    opts.dist = VIS_CLAHS_DIST_RAYLEIGH;
    int ret = vis_clahs (clahsimg->imageData, clahsimg->width, clahsimg->height, clahsimg->depth, &opts);
    if (ret < 0 ) {
        printf ("clahs error, ret=%d\n", ret);
    }

    IplImage *img_8bit = cvCreateImage (cvGetSize (srcimg), IPL_DEPTH_8U, 1);
    cvConvertImage (clahsimg, img_8bit, 0);

    CvMat* K_matrix;
    K_matrix = cvCreateMat(3,3,CV_32FC1);
    cvmSet (K_matrix,0,0, 1717.612682549220835); cvmSet(K_matrix,0,1, 0); cvmSet(K_matrix,0,2,716.481702212280538);
    cvmSet (K_matrix,1,0, 0); cvmSet(K_matrix,1,1, 1722.093475350950030); cvmSet(K_matrix,1,2,553.074067198867510);
    cvmSet (K_matrix,2,0, 0); cvmSet(K_matrix,2,1, 0); cvmSet(K_matrix,2,2, 1);

    CvMat* distCoeffs;
    distCoeffs = cvCreateMat (1,5,CV_32FC1);
    cvmSet (distCoeffs,0,0, 0.144810177630367); 
    cvmSet (distCoeffs,0,1, 0.698350635155968); 
    cvmSet (distCoeffs,0,2, 0.006548501158010);
    cvmSet (distCoeffs,0,3, -0.006986461621033); 
    cvmSet (distCoeffs,0,4, 0.000000000000000); 
	cvUndistort2 (img_8bit, init_img, K_matrix, distCoeffs);

    // clean up
    cvReleaseImage (&imggray);
    cvReleaseImage (&img_8bit);
    cvReleaseMat (&K_matrix);
    cvReleaseMat (&distCoeffs);

}

#ifdef SIFTGPU_FOUND
void siftgpu_to_tFeature(SiftGPU *sift, tFeature* feat)
{
    // returned key points (u,v) and 128 descriptor
    // tFeature has CvMat *uv and CvMat *key; 

    int m, d;

    vector<float > descriptors;
    vector<SiftGPU::SiftKeypoint> keys;	
    // SiftKeypoint { float x, y, s, o; //x, y, scale, orientation. }
    int num = feat->Nf;
    keys.resize(num);	descriptors.resize(DESCRIPTOR_SIZE*num);
    sift->GetFeatureVector(&keys[0], &descriptors[0]);

    // SIFTGPU has some duplicated points (~100)
    //int unique_num = 1;
    //vector<int> unique_idx_list;
    //unique_idx_list.push_back(0);
    //
    //bool duplicated = false;
    //float tol = 1.0;  
    //for (i=0; i<num; i++) {
    //  SiftGPU::SiftKeypoint kp = keys.at(i);
    //  float x = kp.x; float y = kp.y;
    //
    //  duplicated = false;
    //  for (j=0; j<unique_num; j++) {
    //    int idx = unique_idx_list.at(j);
    //    SiftGPU::SiftKeypoint kp_cpr = keys.at(idx);
    //    float x_cpr = kp_cpr.x; float y_cpr = kp_cpr.y;
    //    if ( x == x_cpr  && y == y_cpr )
    //      duplicated = true;
    //  }
    //  
    //  if(!duplicated) {
    //    unique_idx_list.push_back(i);
    //    unique_num++;
    //    cout << "unique_num = " << unique_num << endl;
    //  }
    //  cout << x <<", "<<y<< endl;
    //}

    if (num > MAX_FEAT_NUM) {
        m = MAX_FEAT_NUM;
        feat->Nf = m;
    }
    else
        m = num; 

    // allocate memory for uv and key
    feat->uv =  cvCreateMat(m,2,CV_32FC1); 
    feat->key =  cvCreateMat(m,DESCRIPTOR_SIZE,CV_32FC1); 

    for (int i=0; i<m; i++) {
        SiftGPU::SiftKeypoint kp = keys.at(i);
        float x = kp.x;
        float y = kp.y;
        cvmSet(feat->uv ,i, 0, x);
        cvmSet(feat->uv ,i, 1, y);

        for (d = 0; d < DESCRIPTOR_SIZE; d++)
            cvmSet(feat->key,i,d,descriptors.at(DESCRIPTOR_SIZE*i+d));
    }
}

int compute_feature(IplImage* img, tFeature* feat, SiftGPU  *sift = NULL)
{
    // RUN SIFTGPU
    int siftres = sift->RunSIFT(img->width, img->height, img->imageData, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    int num = 0;

    if (siftres) {
        num = sift->GetFeatureNum();
        feat->Nf = num;
        feat->type = FEAT_SIFTGPU;
        // store it to tFeature
        siftgpu_to_tFeature(sift, feat);

        printf("found %d features \n ",feat->Nf);
        return num;

    }
    else {
        printf("ERROR in SIFTGPU");
        return -1;
    }
}
#endif



int main(int argc, char** argv)
{
    // RUN
    IplImage* img_raw1 = cvLoadImage ("../share/examples/_test_corr_files/color1.jpg", CV_LOAD_IMAGE_UNCHANGED );
    IplImage* img_8bit_undist1 = 0;
    IplImage* img_raw2 = cvLoadImage ("../share/examples/_test_corr_files/color2.jpg", CV_LOAD_IMAGE_UNCHANGED );
    IplImage* img_8bit_undist2 = 0;

    img_8bit_undist1 = cvCreateImage(cvGetSize(img_raw1),IPL_DEPTH_8U, 1);
    img_8bit_undist2 = cvCreateImage(cvGetSize(img_raw2),IPL_DEPTH_8U, 1);
    initialize_image(img_raw1, img_8bit_undist1);
    initialize_image(img_raw2, img_8bit_undist2);

    // -------------------------------------------------------------- //
    // [Compute Features] SURF or SIFT or Harris
    // -------------------------------------------------------------- //
    printf( "[Compute Features]\n");
    tFeature feat1; tFeature feat2;

#ifdef SIFTGPU_FOUND
    SiftGPU  *sift = new SiftGPU;
    char * siftargv[] = {"-fo", "-1", "-v", "1", "-tc", "900"};//
    int siftargc = sizeof(siftargv)/sizeof(char*);
  	sift->ParseParam(siftargc, siftargv);
    if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) return -1;
 	
    compute_feature(img_8bit_undist1, &feat1, sift);
 	compute_feature(img_8bit_undist2, &feat2, sift);

    plotFeature(1,img_8bit_undist1, &feat1);
    plotFeature(2,img_8bit_undist2, &feat2);

#else
    printf ("NEED SIFTGPU_FOUND = ON\n");
    return -1;
#endif

    printf(" -------- Done ! -----------\n");

    // Clean up
    cvReleaseImage( &img_raw1 );
    cvReleaseImage( &img_raw2 );
    cvReleaseImage( &img_8bit_undist1 );
    cvReleaseImage( &img_8bit_undist2 );

}
