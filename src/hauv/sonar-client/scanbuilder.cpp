/**
 * @file scanbuilder.cpp
 * @brief A class that extracts feature points from sonar images
 * @author Hordur Johannsson
 * @version $Id:  $
 */

#include <Eigen/Core>
#include <opencv/cv.h>

#include "scanbuilder.h"

using namespace isam;
using namespace cv;

ScanBuilderA::ScanBuilderA() : didsonFrame(NULL), filteredImage(NULL), filteredImage2(NULL)
{
    CvSize sonarImageSize = cvSize(96, 512);
    sonarFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);  
    filteredFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1); 
    filteredFrame2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);  
}

ScanBuilderA::~ScanBuilderA()
{
    cvReleaseImage(&sonarFrame);
    cvReleaseImage(&filteredFrame);
    cvReleaseImage(&filteredFrame2);
    cvReleaseImage(&filteredImage);
    cvReleaseImage(&filteredImage2);
    delete didsonFrame;
}

std::vector<isam::Point2d>* ScanBuilderA::getPoints(hauv::didson_t* didsonData)
{
    // NOTE: could the dimension be variable, i.e. not 96x512
    if (didsonFrame == NULL) {
        didsonFrame = new DidsonFrame(didsonData);
        didsonFrame->makeNewImage(300);
    } else {
        didsonFrame->setFrame(didsonData);
    }	
    IplImage* sonarImage;

    MapScale ms = didsonFrame->getMapScale();
    sonarImage = cvCreateImage(cvSize(ms.width, ms.height), IPL_DEPTH_8U, 1);	
    if (filteredImage2 == NULL) {
        CvSize sonarImageSize = cvSize(ms.width, ms.height);
        filteredImage = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
        filteredImage2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    }
    memcpy(sonarFrame->imageData, didsonData->m_cData, 96*512);

    // Filter image

    // 1. Calculate a gradient for the image
    //float shadowFilter[11] = {-1.0, -1.0, -1.0, -1.0, -1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[11] = {0.0, 0.0, 0.0, 2.0, 2.0, 3.0,-2.0, -1.0, -1.0, -1.0, -1.0};
    float shadowFilter[13] = {0.0,0.0, 0.0, 0.0, 2.0, 2.0, 2.0,-2.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[11] = {0.0, 0.0, 0.0, 2.0, 2.0, 2.0,-2.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[9] = {-1.0, -1.0, -1.0, -1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[9] = {0.0, 0.0, 0.0, 0.0, 4.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[7] = {0.0, 0.0, 0.0, 3.0, -1.0, -1.0, -1.0};
    CvMat shadowKernel = cvMat(13, 1, CV_32FC1, shadowFilter);
    cvFilter2D(sonarFrame, filteredFrame, &shadowKernel);

    // 2. Create a histogram and label top percentage as highlights or points of interest
    //			The goal is to pick out consistent features in the sonar image
    int mint, maxt;
    float v;
    int histSizes[1] = {256};
    float g_ranges[] = {0, 255};
    float* ranges = {g_ranges};
    CvHistogram* hist = cvCreateHist(1, histSizes, CV_HIST_ARRAY, &ranges, 1);
    cvCalcHist(&filteredFrame, hist);
    cvNormalizeHist(hist, 1.0);

    // Accumulate the histogram
    mint = -1; maxt = -1;
    v = 0.0;
    for (int i=0;i<256;i++) {
        v = v + cvQueryHistValue_1D(hist, i);
        if (mint<0 && v >= 0.3) mint = i;
        if (maxt<0 && v >= 0.35) maxt = i;
    }
		
    cvReleaseHist(&hist);	
    cvThreshold(filteredFrame, filteredFrame, maxt, 1, CV_THRESH_BINARY);
	
    // 3. Throw out spurious pixel by counting the number of highlights in 
    //		a box surrounding the pixel
    // Count filter
	
    float countFilter[100];
    CvMat countKernel = cvMat(3, 6, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvFilter2D(filteredFrame, filteredFrame2, &countKernel);
    cvThreshold(filteredFrame2, filteredFrame2, 10, 255, CV_THRESH_BINARY);
	
    // 4. Project the sensor data from measurment space (polar) into cartesian coordinate frame
    //			NOTE: This is an unceratain transformation because we do not know the altitude/elevation
    //						of the points in measurement space. We project assuming elevation 0.
    filteredFrame2->imageData[0] = 0;
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame2->imageData, (uint8_t*)filteredImage->imageData);
    didsonFrame->copyFrameToImage((uint8_t*)sonarFrame->imageData, (uint8_t*)sonarImage->imageData);

    //float countFilter[100];
    cvThreshold(filteredImage, filteredImage, 1, 1, CV_THRESH_BINARY);

    CvMat countKernel2 = cvMat(10, 10, CV_32FC1, countFilter);
    cvSet(&countKernel2, cvScalar(1.0));
    cvFilter2D(filteredImage, filteredImage2, &countKernel2);
    cvThreshold(filteredImage2, filteredImage2, 25, 255, CV_THRESH_BINARY);

    // 5. Extract the remaining highlight pixels and return as a vector of points
    // Extract points from filteredImage2
    //std::cout << "y0: " << ms.y0 << std::endl;
    std::vector<isam::Point2d>* points = new std::vector<isam::Point2d>();

    for (int i=0; i<ms.height;i++) {
        const char* ptr1 = (const char*)(filteredImage->imageData + i*ms.width);
        const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
        const char* ptrImage = (const char*)(sonarImage->imageData + i*ms.width);

        for (int j=0; j<ms.width; j++) {
            if (*ptr != 0 && *ptr1 != 0 && *ptrImage > 40) {
                points->push_back(isam::Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
            }
            ptr++;
            ptr1++;
            ptrImage++;
        }
    }

    cvReleaseImage(&sonarImage);
    return points;
}


/// Optimized for tank operations
ScanBuilderB::ScanBuilderB() : didsonFrame(NULL)
{
    CvSize sonarImageSize = cvSize(96, 512);
    sonarFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
}

ScanBuilderB::~ScanBuilderB()
{
    cvReleaseImage(&sonarFrame);
    cvReleaseImage(&filteredFrame);
    cvReleaseImage(&filteredFrame2);
    cvReleaseImage(&filteredImage2);
    delete didsonFrame;
}

std::vector<isam::Point2d>* ScanBuilderB::getPoints(hauv::didson_t* didsonData)
{
    // NOTE: could the dimension be variable, i.e. not 96x512
    if (didsonFrame == NULL) {
        didsonFrame = new DidsonFrame(didsonData);
        didsonFrame->makeNewImage(300);
    } else {
        didsonFrame->setFrame(didsonData);
    }	
    MapScale ms = didsonFrame->getMapScale();
    if (filteredImage2 == NULL) {
        CvSize sonarImageSize = cvSize(ms.width, ms.height);
        filteredImage2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    }
    memcpy(sonarFrame->imageData, didsonData->m_cData, 96*512);

    // Filter image

    // 1. Calculate a gradient for the image
    float smoothFilter[9];
    CvMat smoothKernel = cvMat(3, 3, CV_32FC1, smoothFilter);
    cvSet(&smoothKernel, cvScalar(0.0));
    cvmSet(&smoothKernel, 1, 1, 1.0);
    cvFilter2D(sonarFrame, filteredFrame, &smoothKernel);

    //float shadowFilter[11] = {-1.0, -1.0, -1.0, -1.0, -1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 5.0,-1.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[9] = {-1.0, -1.0, -1.0, -1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[9] = {0.0, 0.0, 0.0, 0.0, 4.0, -1.0, -1.0, -1.0, -1.0};
    float shadowFilter[7] = { -1.0, -1.0, -1.0, 3.0, 0.0, 0.0, 0.0 };
    CvMat shadowKernel = cvMat(7, 1, CV_32FC1, shadowFilter);
    cvFilter2D(filteredFrame, filteredFrame2, &shadowKernel);

    // 2. Create a histogram and label top percentage as highlights or points of interest
    //			The goal is to pick out consistent features in the sonar image
    //int mint;
    int maxt;
    float v;
    int histSizes[1] = {256};
    float g_ranges[] = {0, 255};
    float* ranges = {g_ranges};
    CvHistogram* hist = cvCreateHist(1, histSizes, CV_HIST_ARRAY, &ranges, 1);
    cvCalcHist(&filteredFrame2, hist);
    cvNormalizeHist(hist, 1.0);

    // Accumulate the histogram
    //mint = -1; 
    maxt = -1;
    v = 0.0;
    for (int i=0;i<256;i++) {
        v = v + cvQueryHistValue_1D(hist, i);
        if (maxt<0 && v >= 0.95) maxt = i;
    }
		
    cvReleaseHist(&hist);	
    cvThreshold(filteredFrame2, filteredFrame2, maxt, 1, CV_THRESH_BINARY);
	
    // 3. Throw out spurious pixel by counting the number of highlights in 
    //		a box surrounding the pixel
    // Count filter
    float countFilter[100];
    CvMat countKernel = cvMat(10, 10, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvFilter2D(filteredFrame2, filteredFrame, &countKernel);
    cvThreshold(filteredFrame, filteredFrame, 30, 255, CV_THRESH_BINARY);

    // 4. Project the sensor data from measurment space (polar) into cartesian coordinate frame
    //			NOTE: This is an unceratain transformation because we do not know the altitude/elevation
    //						of the points in measurement space. We project assuming elevation 0.
    filteredFrame->imageData[0] = 0;
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame->imageData, (uint8_t*)filteredImage2->imageData);

    // 5. Extract the remaining highlight pixels and return as a vector of points
    // Extract points from filteredImage2
    //std::cout << "y0: " << ms.y0 << std::endl;
    std::vector<isam::Point2d>* points = new std::vector<isam::Point2d>();
    for (int i=0; i<ms.height;i++) {
        const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
        for (int j=0; j<ms.width; j++) {
            if (*ptr != 0) {
                points->push_back(isam::Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
            }
            ptr++;
        }
    }

    return points;
}




/// Optimized for ship hull operations
ScanBuilderShip::ScanBuilderShip() : didsonFrame(NULL)
{
    CvSize sonarImageSize = cvSize(96, 512);
    sonarFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
}

ScanBuilderShip::~ScanBuilderShip()
{
    cvReleaseImage(&sonarFrame);
    cvReleaseImage(&filteredFrame);
    cvReleaseImage(&filteredFrame2);
    cvReleaseImage(&filteredImage2);
    delete didsonFrame;
}

std::vector<isam::Point2d>* ScanBuilderShip::getPoints(hauv::didson_t* didsonData)
{
    // NOTE: could the dimension be variable, i.e. not 96x512
    if (didsonFrame == NULL) {
        didsonFrame = new DidsonFrame(didsonData);
        didsonFrame->makeNewImage(200);
    } else {
        didsonFrame->setFrame(didsonData);
    }	
    MapScale ms = didsonFrame->getMapScale();
    if (filteredImage2 == NULL) {
        CvSize sonarImageSize = cvSize(ms.width, ms.height);
        filteredImage2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    }
    memcpy(sonarFrame->imageData, didsonData->m_cData, 96*512);

    // Filter image

    // 1. Calculate a gradient for the image
    float smoothFilter[9];
    CvMat smoothKernel = cvMat(3, 3, CV_32FC1, smoothFilter);
    cvSet(&smoothKernel, cvScalar(0.0));
    cvmSet(&smoothKernel, 1, 1, 1.0);
    cvFilter2D(sonarFrame, filteredFrame, &smoothKernel);

    //float shadowFilter[11] = {-1.0, -1.0, -1.0, -1.0, -1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 5.0,-1.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[9] = {-1.0, -1.0, -1.0, -1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[9] = {0.0, 0.0, 0.0, 0.0, 4.0, -1.0, -1.0, -1.0, -1.0};
    float shadowFilter[7] = { -1.0, -1.0, -1.0, 3.0, 0.0, 0.0, 0.0 };
    //float shadowFilter[7] = { 0.0,0.0,0.0,3.0,-1.0, -1.0, -1.0};
    CvMat shadowKernel = cvMat(7, 1, CV_32FC1, shadowFilter);
    cvFilter2D(filteredFrame, filteredFrame2, &shadowKernel);

    // 2. Create a histogram and label top percentage as highlights or points of interest
    //			The goal is to pick out consistent features in the sonar image
    //int mint;
    int maxt;
    float v;
    int histSizes[1] = {256};
    float g_ranges[] = {0, 255};
    float* ranges = {g_ranges};
    CvHistogram* hist = cvCreateHist(1, histSizes, CV_HIST_ARRAY, &ranges, 1);
    cvCalcHist(&filteredFrame2, hist);
    cvNormalizeHist(hist, 1.0);

    // Accumulate the histogram
    //mint = -1;
    maxt = -1;
    v = 0.0;
    for (int i=0;i<256;i++) {
        v = v + cvQueryHistValue_1D(hist, i);
        if (maxt<0 && v >= 0.90) maxt = i;
    }
		
    cvReleaseHist(&hist);	
    cvThreshold(filteredFrame2, filteredFrame2, maxt, 1, CV_THRESH_BINARY);
	
    // 3. Throw out spurious pixel by counting the number of highlights in 
    //		a box surrounding the pixel
    // Count filter
    float countFilter[100];
    CvMat countKernel = cvMat(10, 10, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvFilter2D(filteredFrame2, filteredFrame, &countKernel);
    cvThreshold(filteredFrame, filteredFrame, 25, 255, CV_THRESH_BINARY);

    // 4. Project the sensor data from measurment space (polar) into cartesian coordinate frame
    //			NOTE: This is an unceratain transformation because we do not know the altitude/elevation
    //						of the points in measurement space. We project assuming elevation 0.
    filteredFrame->imageData[0] = 0;
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame->imageData, (uint8_t*)filteredImage2->imageData);

    // 5. Extract the remaining highlight pixels and return as a vector of points
    // Extract points from filteredImage2
    //std::cout << "y0: " << ms.y0 << std::endl;
    std::vector<isam::Point2d>* points = new std::vector<isam::Point2d>();
    for (int i=0; i<ms.height;i++) {
        const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
        for (int j=0; j<ms.width; j++) {
            if (*ptr != 0) {
                points->push_back(isam::Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
            }
            ptr++;
        }
    }

    return points;
}


/// Optimized for seafloor operations
ScanBuilderSeafloor::ScanBuilderSeafloor() : didsonFrame(NULL)
{
    CvSize sonarImageSize = cvSize(96, 512);
    sonarFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
}

ScanBuilderSeafloor::~ScanBuilderSeafloor()
{
    cvReleaseImage(&sonarFrame);
    cvReleaseImage(&filteredFrame);
    cvReleaseImage(&filteredFrame2);
    cvReleaseImage(&filteredImage2);
    delete didsonFrame;
}

std::vector<isam::Point2d>* ScanBuilderSeafloor::getPoints(hauv::didson_t* didsonData)
{
    // NOTE: could the dimension be variable, i.e. not 96x512
    if (didsonFrame == NULL) {
        didsonFrame = new DidsonFrame(didsonData);
        didsonFrame->makeNewImage(300);
    } else {
        didsonFrame->setFrame(didsonData);
    }	
    MapScale ms = didsonFrame->getMapScale();
    if (filteredImage2 == NULL) {
        CvSize sonarImageSize = cvSize(ms.width, ms.height);
        filteredImage2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    }
    memcpy(sonarFrame->imageData, didsonData->m_cData, 96*512);

    // Filter image

    // 1. Calculate a gradient for the image
    float smoothFilter[9];
    CvMat smoothKernel = cvMat(3, 3, CV_32FC1, smoothFilter);
    cvSet(&smoothKernel, cvScalar(0.0));
    cvmSet(&smoothKernel, 1, 1, 1.0);
    cvFilter2D(sonarFrame, filteredFrame, &smoothKernel);

    //float shadowFilter[11] = {-1.0, -1.0, -1.0, -1.0, -1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float shadowFilter[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 5.0,-1.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[9] = {-1.0, -1.0, -1.0, -1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[9] = {0.0, 0.0, 0.0, 0.0, 4.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[7] = { -1.0, -1.0, -1.0, 3.0, 0.0, 0.0, 0.0 };
    //float shadowFilter[7] = { 0.0,0.0,0.0,3.0,-1.0, -1.0, -1.0};
    //CvMat shadowKernel = cvMat(7, 1, CV_32FC1, shadowFilter);
    CvMat shadowKernel = cvMat(11, 1, CV_32FC1, shadowFilter);
    cvFilter2D(filteredFrame, filteredFrame2, &shadowKernel);

    // 2. Create a histogram and label top percentage as highlights or points of interest
    //			The goal is to pick out consistent features in the sonar image
    //int mint;
    int maxt;
    float v;
    int histSizes[1] = {256};
    float g_ranges[] = {0, 255};
    float* ranges = {g_ranges};
    CvHistogram* hist = cvCreateHist(1, histSizes, CV_HIST_ARRAY, &ranges, 1);
    cvCalcHist(&filteredFrame2, hist);
    cvNormalizeHist(hist, 1.0);

    // Accumulate the histogram
    //mint = -1;
    maxt = -1;
    v = 0.0;
    for (int i=0;i<256;i++) {
        v = v + cvQueryHistValue_1D(hist, i);
        if (maxt<0 && v >= 0.9) maxt = i;
    }
		
    cvReleaseHist(&hist);	
    cvThreshold(filteredFrame2, filteredFrame2, maxt, 1, CV_THRESH_BINARY);
	
    // 3. Throw out spurious pixel by counting the number of highlights in 
    //		a box surrounding the pixel
    // Count filter
    float countFilter[100];
    CvMat countKernel = cvMat(10, 10, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvFilter2D(filteredFrame2, filteredFrame, &countKernel);
    cvThreshold(filteredFrame, filteredFrame, 25, 255, CV_THRESH_BINARY);

    // 4. Project the sensor data from measurment space (polar) into cartesian coordinate frame
    //			NOTE: This is an unceratain transformation because we do not know the altitude/elevation
    //						of the points in measurement space. We project assuming elevation 0.
    filteredFrame->imageData[0] = 0;
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame->imageData, (uint8_t*)filteredImage2->imageData);

    // 5. Extract the remaining highlight pixels and return as a vector of points
    // Extract points from filteredImage2
    //std::cout << "y0: " << ms.y0 << std::endl;
    std::vector<isam::Point2d>* points = new std::vector<isam::Point2d>();
    for (int i=0; i<ms.height;i++) {
        const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
        for (int j=0; j<ms.width; j++) {
            if (*ptr != 0) {
                points->push_back(isam::Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
            }
            ptr++;
        }
    }

    return points;
}


ScanBuilderOceanus::ScanBuilderOceanus() : didsonFrame(NULL), filteredImage(NULL), filteredImage2(NULL)
{
    CvSize sonarImageSize = cvSize(96, 512);
    sonarFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    lastThreshold = -1; 

    cvNamedWindow( "BeforeImage", CV_WINDOW_AUTOSIZE);
    cvNamedWindow( "FilterImage", CV_WINDOW_AUTOSIZE);
    cvNamedWindow( "FilterImage2", CV_WINDOW_AUTOSIZE);
    cvNamedWindow( "FilterImage3", CV_WINDOW_AUTOSIZE);

}

ScanBuilderOceanus::~ScanBuilderOceanus()
{
    cvReleaseImage(&sonarFrame);
    cvReleaseImage(&filteredFrame);
    cvReleaseImage(&filteredFrame2);
    cvReleaseImage(&filteredImage);
    cvReleaseImage(&filteredImage2);
    delete didsonFrame;
}

std::vector<isam::Point2d>* ScanBuilderOceanus::getPoints(hauv::didson_t* didsonData)
{
    // NOTE: could the dimension be variable, i.e. not 96x512
    if (didsonFrame == NULL) {
        didsonFrame = new DidsonFrame(didsonData);
        didsonFrame->makeNewImage(300);
    } else {
        didsonFrame->setFrame(didsonData);
    }	
    IplImage* sonarImage;

    MapScale ms = didsonFrame->getMapScale();
    sonarImage = cvCreateImage(cvSize(ms.width, ms.height), IPL_DEPTH_8U, 1);	
    if (filteredImage2 == NULL) {
        CvSize sonarImageSize = cvSize(ms.width, ms.height);
        filteredImage = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
        filteredImage2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    }
    memcpy(sonarFrame->imageData, didsonData->m_cData, 96*512);

    // Filter image
    cvSmooth(sonarFrame, filteredFrame2, CV_GAUSSIAN, 5, 1);

    // 1. Calculate a gradient for the image
    //float shadowFilter[11] = {-1.0, -1.0, -1.0, -1.0, -1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[11] = {0.0, 0.0, 0.0, 2.0, 2.0, 3.0,-2.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[13] = {0.0,0.0, 0.0, 0.0, 2.0, 2.0, 2.0,-2.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    float shadowFilter[13] = {0.0,0.0, 0.0, 0.0, 0.0, 1.0, 5.0,-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[11] = {0.0, 0.0, 0.0, 2.0, 2.0, 2.0,-2.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[9] = {-1.0, -1.0, -1.0, -1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[9] = {0.0, 0.0, 0.0, 0.0, 4.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[7] = {0.0, 0.0, 0.0, 3.0, -1.0, -1.0, -1.0};
    //CvMat shadowKernel = cvMat(13, 1, CV_32FC1, shadowFilter);
    CvMat shadowKernel = cvMat(13, 1, CV_32FC1, shadowFilter);
    cvFilter2D(filteredFrame2, filteredFrame, &shadowKernel);

    //cvShowImage("BeforeImage", sonarFrame);
    //cvShowImage("FilterImage", filteredFrame);

    // 2. Create a histogram and label top percentage as highlights or points of interest
    //			The goal is to pick out consistent features in the sonar image
    int mint, maxt;
    float v;
    int histSizes[1] = {256};
    float g_ranges[] = {0, 255};
    float* ranges = {g_ranges};
    CvHistogram* hist = cvCreateHist(1, histSizes, CV_HIST_ARRAY, &ranges, 1);
    cvCalcHist(&filteredFrame, hist);
    cvNormalizeHist(hist, 1.0);

    // Accumulate the histogram
    mint = -1; maxt = -1;
    v = 0.0;
    for (int i=0;i<256;i++) {
        v = v + cvQueryHistValue_1D(hist, i);
        if (mint<0 && v >= 0.3) mint = i;
        //		if (maxt<0 && v >= 0.35) maxt = i;
        if (maxt<0 && v >= 0.95) maxt = i;
    }
    double a = 0.1;
    if (lastThreshold == -1.0) lastThreshold = maxt;
    else lastThreshold = (1.0-a)*lastThreshold + maxt*a;
    cvReleaseHist(&hist);	
    cvThreshold(filteredFrame, filteredFrame, (int)lastThreshold-1, 1, CV_THRESH_BINARY);

    //cvShowImage("FilterImage2", filteredFrame);

	
    // 3. Throw out spurious pixel by counting the number of highlights in 
    //		a box surrounding the pixel
    // Count filter
	
    float countFilter[100];
    CvMat countKernel = cvMat(4, 6, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvFilter2D(filteredFrame, filteredFrame2, &countKernel);
    cvThreshold(filteredFrame2, filteredFrame2, 10, 255, CV_THRESH_BINARY);
	
    // 4. Project the sensor data from measurment space (polar) into cartesian coordinate frame
    //			NOTE: This is an unceratain transformation because we do not know the altitude/elevation
    //						of the points in measurement space. We project assuming elevation 0.
    filteredFrame2->imageData[0] = 0;
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame2->imageData, (uint8_t*)filteredImage->imageData);
    didsonFrame->copyFrameToImage((uint8_t*)sonarFrame->imageData, (uint8_t*)sonarImage->imageData);

    //float countFilter[100];
    cvThreshold(filteredImage, filteredImage, 1, 1, CV_THRESH_BINARY);

    CvMat countKernel2 = cvMat(10, 10, CV_32FC1, countFilter);
    cvSet(&countKernel2, cvScalar(1.0));
    cvFilter2D(filteredImage, filteredImage2, &countKernel2);
    cvThreshold(filteredImage2, filteredImage2, 15, 255, CV_THRESH_BINARY);

    // 5. Extract the remaining highlight pixels and return as a vector of points
    // Extract points from filteredImage2
    //std::cout << "y0: " << ms.y0 << std::endl;
    std::vector<isam::Point2d>* points = new std::vector<isam::Point2d>();

    for (int i=0; i<ms.height;i++) {
        const char* ptr1 = (const char*)(filteredImage->imageData + i*ms.width);
        const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
        const char* ptrImage = (const char*)(sonarImage->imageData + i*ms.width);

        for (int j=0; j<ms.width; j++) {
            //if (*ptr != 0 && *ptr1 != 0 && *ptrImage > 40) {
            //if (*ptr != 0 && *ptr1 != 0 && *ptrImage > 80) {
            if (*ptr != 0 && *ptr1 != 0  && *ptrImage > 40) {
                points->push_back(isam::Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
            }
            ptr++;
            ptr1++;
            ptrImage++;
        }
    }

    cvReleaseImage(&sonarImage);
    return points;
}

/** Oceanus 2 **/
ScanBuilderOceanus2::ScanBuilderOceanus2() : didsonFrame(NULL), filteredImage(NULL), filteredImage2(NULL)
{
    CvSize sonarImageSize = cvSize(96, 512);
    sonarFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    lastThreshold = -1; 

    cvNamedWindow( "BeforeImage", CV_WINDOW_AUTOSIZE);
    cvNamedWindow( "FilterImage", CV_WINDOW_AUTOSIZE);
    cvNamedWindow( "FilterImage2", CV_WINDOW_AUTOSIZE);
    cvNamedWindow( "FilterImage3", CV_WINDOW_AUTOSIZE);

}

ScanBuilderOceanus2::~ScanBuilderOceanus2()
{
    cvReleaseImage(&sonarFrame);
    cvReleaseImage(&filteredFrame);
    cvReleaseImage(&filteredFrame2);
    cvReleaseImage(&filteredImage);
    cvReleaseImage(&filteredImage2);
    delete didsonFrame;
}

std::vector<isam::Point2d>* ScanBuilderOceanus2::getPoints(hauv::didson_t* didsonData)
{
    // NOTE: could the dimension be variable, i.e. not 96x512
    if (didsonFrame == NULL) {
        didsonFrame = new DidsonFrame(didsonData);
        didsonFrame->makeNewImage(300);
    } else {
        didsonFrame->setFrame(didsonData);
    }	
    IplImage* sonarImage;

    MapScale ms = didsonFrame->getMapScale();
    sonarImage = cvCreateImage(cvSize(ms.width, ms.height), IPL_DEPTH_8U, 1);	
    if (filteredImage2 == NULL) {
        CvSize sonarImageSize = cvSize(ms.width, ms.height);
        filteredImage = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
        filteredImage2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    }
    memcpy(sonarFrame->imageData, didsonData->m_cData, 96*512);

    // Filter image
    cvSmooth(sonarFrame, filteredFrame2, CV_GAUSSIAN, 5, 5, 3);

    // 1. Calculate a gradient for the image
    float shadowFilter[11] = {-1.0, -1.0, -1.0, -1.0, -1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[11] = {0.0, 0.0, 0.0, 2.0, 2.0, 3.0,-2.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[13] = {0.0,0.0, 0.0, 0.0, 2.0, 2.0, 2.0,-2.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[13] = {0.0,0.0, 0.0, 0.0, 2.0, 2.0, 2.0,-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[11] = {0.0, 0.0, 0.0, 2.0, 2.0, 2.0,-2.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[9] = {-1.0, -1.0, -1.0, -1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[9] = {0.0, 0.0, 0.0, 0.0, 4.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[9] = {-1.0,-1.0,-1.0,-1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[7] = {0.0, 0.0, 0.0, 3.0, -1.0, -1.0, -1.0};
    //CvMat shadowKernel = cvMat(13, 1, CV_32FC1, shadowFilter);
    //CvMat shadowKernel = cvMat(sizeof(shadowFilter)/4, 1, CV_32FC1, shadowFilter);
    CvMat shadowKernel = cvMat(1, sizeof(shadowFilter)/4, CV_32FC1, shadowFilter);
    cvFilter2D(filteredFrame2, filteredFrame, &shadowKernel);

    //cvShowImage("BeforeImage", sonarFrame);
    //cvShowImage("FilterImage", filteredFrame);

    // 2. Create a histogram and label top percentage as highlights or points of interest
    //			The goal is to pick out consistent features in the sonar image
    int mint, maxt;
    float v;
    int histSizes[1] = {256};
    float g_ranges[] = {0, 255};
    float* ranges = {g_ranges};
    CvHistogram* hist = cvCreateHist(1, histSizes, CV_HIST_ARRAY, &ranges, 1);
    cvCalcHist(&filteredFrame, hist);
    cvNormalizeHist(hist, 1.0);

    // Accumulate the histogram
    mint = -1; maxt = -1;
    v = 0.0;
    for (int i=0;i<256;i++) {
        v = v + cvQueryHistValue_1D(hist, i);
        if (mint<0 && v >= 0.3) mint = i;
        //		if (maxt<0 && v >= 0.35) maxt = i;
        if (maxt<0 && v >= 0.97) maxt = i;
    }
    double a = 0.1;
    if (lastThreshold == -1.0) lastThreshold = maxt;
    else lastThreshold = (1.0-a)*lastThreshold + maxt*a;
    cvReleaseHist(&hist);	
    cvThreshold(filteredFrame, filteredFrame, (int)lastThreshold-1, 1, CV_THRESH_BINARY);

    //cvShowImage("FilterImage2", filteredFrame);

	
    // 3. Throw out spurious pixel by counting the number of highlights in 
    //		a box surrounding the pixel
    // Count filter
	
    float countFilter[100];
    CvMat countKernel = cvMat(4, 6, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvFilter2D(filteredFrame, filteredFrame2, &countKernel);
    cvThreshold(filteredFrame2, filteredFrame2, 10, 255, CV_THRESH_BINARY);
	
    // 4. Project the sensor data from measurment space (polar) into cartesian coordinate frame
    //			NOTE: This is an unceratain transformation because we do not know the altitude/elevation
    //						of the points in measurement space. We project assuming elevation 0.
    filteredFrame2->imageData[0] = 0;
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame2->imageData, (uint8_t*)filteredImage->imageData);
    didsonFrame->copyFrameToImage((uint8_t*)sonarFrame->imageData, (uint8_t*)sonarImage->imageData);

    //float countFilter[100];
    cvThreshold(filteredImage, filteredImage, 1, 1, CV_THRESH_BINARY);

    CvMat countKernel2 = cvMat(10, 10, CV_32FC1, countFilter);
    cvSet(&countKernel2, cvScalar(1.0));
    cvFilter2D(filteredImage, filteredImage2, &countKernel2);
    cvThreshold(filteredImage2, filteredImage2, 15, 255, CV_THRESH_BINARY);

    // 5. Extract the remaining highlight pixels and return as a vector of points
    // Extract points from filteredImage2
    //std::cout << "y0: " << ms.y0 << std::endl;
    std::vector<isam::Point2d>* points = new std::vector<isam::Point2d>();

    for (int i=0; i<ms.height;i++) {
        const char* ptr1 = (const char*)(filteredImage->imageData + i*ms.width);
        const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
        const char* ptrImage = (const char*)(sonarImage->imageData + i*ms.width);

        for (int j=0; j<ms.width; j++) {
            //if (*ptr != 0 && *ptr1 != 0 && *ptrImage > 40) {
            //if (*ptr != 0 && *ptr1 != 0 && *ptrImage > 80) {
            if (*ptr != 0 && *ptr1 != 0  && *ptrImage > 60) {
                points->push_back(isam::Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
            }
            ptr++;
            ptr1++;
            ptrImage++;
        }
    }

    cvReleaseImage(&sonarImage);
    return points;
}


/// Optimized for seafloor operations
ScanBuilderSeafloor2::ScanBuilderSeafloor2() : didsonFrame(NULL)
{
    CvSize sonarImageSize = cvSize(96, 512);
    sonarFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
}

ScanBuilderSeafloor2::~ScanBuilderSeafloor2()
{
    cvReleaseImage(&sonarFrame);
    cvReleaseImage(&filteredFrame);
    cvReleaseImage(&filteredFrame2);
    cvReleaseImage(&filteredImage);
    cvReleaseImage(&filteredImage2);
    delete didsonFrame;
}

std::vector<isam::Point2d>* ScanBuilderSeafloor2::getPoints(hauv::didson_t* didsonData)
{
    // NOTE: could the dimension be variable, i.e. not 96x512
    if (didsonFrame == NULL) {
        didsonFrame = new DidsonFrame(didsonData);
        didsonFrame->makeNewImage(300);
    } else {
        didsonFrame->setFrame(didsonData);
    }	
    MapScale ms = didsonFrame->getMapScale();
    if (filteredImage2 == NULL) {
        CvSize sonarImageSize = cvSize(ms.width, ms.height);
        filteredImage = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
        filteredImage2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    }
    memcpy(sonarFrame->imageData, didsonData->m_cData, 96*512);

    // Filter image

    // 1. Calculate a gradient for the image
    float smoothFilter[9];
    CvMat smoothKernel = cvMat(3, 3, CV_32FC1, smoothFilter);
    cvSet(&smoothKernel, cvScalar(0.0));
    cvmSet(&smoothKernel, 1, 1, 1.0);
    cvFilter2D(sonarFrame, filteredFrame, &smoothKernel);

    //float shadowFilter[11] = {-1.0, -1.0, -1.0, -1.0, -1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float shadowFilter[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 5.0,-1.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[9] = {-1.0, -1.0, -1.0, -1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    //float shadowFilter[9] = {0.0, 0.0, 0.0, 0.0, 4.0, -1.0, -1.0, -1.0, -1.0};
    //float shadowFilter[7] = { -1.0, -1.0, -1.0, 3.0, 0.0, 0.0, 0.0 };
    //float shadowFilter[7] = { 0.0,0.0,0.0,3.0,-1.0, -1.0, -1.0};
    //CvMat shadowKernel = cvMat(7, 1, CV_32FC1, shadowFilter);
    CvMat shadowKernel = cvMat(11, 1, CV_32FC1, shadowFilter);

    //cvFilter2D(filteredFrame, filteredFrame2, &shadowKernel);
    cvSmooth( filteredFrame, filteredFrame2, CV_MEDIAN, 5, 1);
    cvSmooth( filteredFrame2, filteredFrame, CV_GAUSSIAN, 5, 1);
    //  cvSmooth( filteredFrame2, filteredFrame, CV_BILATERAL, 5, 5);
    //  cvCopy(filteredFrame2,filteredFrame);
    cvFilter2D(filteredFrame, filteredFrame2, &shadowKernel);

    // 2. Create a histogram and label top percentage as highlights or points of interest
    //			The goal is to pick out consistent features in the sonar image
    //int mint;
    int maxt;
    float v;
    int histSizes[1] = {256};
    float g_ranges[] = {0, 255};
    float* ranges = {g_ranges};
    CvHistogram* hist = cvCreateHist(1, histSizes, CV_HIST_ARRAY, &ranges, 1);
    cvCalcHist(&filteredFrame2, hist);
    cvNormalizeHist(hist, 1.0);

    // Accumulate the histogram
    //mint = -1; 
    maxt = -1;
    v = 0.0;
    for (int i=0;i<256;i++) {
        v = v + cvQueryHistValue_1D(hist, i);
        if (maxt<0 && v >= 0.90) maxt = i;
    }
		 
    cvReleaseHist(&hist);	
    cvThreshold(filteredFrame2, filteredFrame2, maxt, 1, CV_THRESH_BINARY);
    //	cvThreshold(filteredFrame2, filteredFrame, maxt, 1, CV_THRESH_BINARY);

    // 3. Throw out spurious pixel by counting the number of highlights in 
    //		a box surrounding the pixel
    // Count filter
    float countFilter[100];
    //	CvMat countKernel = cvMat(10, 10, CV_32FC1, countFilter);
    CvMat countKernel = cvMat(9, 1, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvFilter2D(filteredFrame2, filteredFrame, &countKernel);
    cvThreshold(filteredFrame, filteredFrame, 5, 255, CV_THRESH_BINARY);

    countKernel = cvMat(10, 10, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvThreshold(filteredFrame, filteredFrame, 25, 255, CV_THRESH_BINARY);

    // 4. Project the sensor data from measurment space (polar) into cartesian coordinate frame
    //			NOTE: This is an unceratain transformation because we do not know the altitude/elevation
    //						of the points in measurement space. We project assuming elevation 0.
    filteredFrame->imageData[0] = 0;
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame->imageData, (uint8_t*)filteredImage2->imageData);
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame2->imageData, (uint8_t*)filteredImage->imageData);

    //  cvErode(filteredImage2, filteredImage2);
    //  cvErode(filteredImage, filteredImage);
    int CVCLOSE_ITR = 1;
    cvMorphologyEx( filteredImage, filteredImage, 0, 0, CV_MOP_OPEN, CVCLOSE_ITR );
    cvMorphologyEx( filteredImage, filteredImage, 0, 0, CV_MOP_CLOSE, CVCLOSE_ITR );

    cvMorphologyEx( filteredImage2, filteredImage2, 0, 0, CV_MOP_OPEN, CVCLOSE_ITR );
    cvMorphologyEx( filteredImage2, filteredImage2, 0, 0, CV_MOP_CLOSE, CVCLOSE_ITR );

    // 5. Extract the remaining highlight pixels and return as a vector of points
    // Extract points from filteredImage2
    //std::cout << "y0: " << ms.y0 << std::endl;

    std::vector<isam::Point2d>* points = new std::vector<isam::Point2d>();

    for (int i=0; i<ms.height-100;i++) {
        const char* ptr1 = (const char*)(filteredImage->imageData + i*ms.width);
        const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
        //		const char* ptrImage = (const char*)(sonarImage->imageData + i*ms.width);

        for (int j=0; j<ms.width; j++) {
            if (*ptr != 0 && *ptr1 != 0) { // && *ptrImage > 40) {
                points->push_back(isam::Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
            }
            ptr++;
            ptr1++;
            //			ptrImage++;
        }
    }

    /*
      std::vector<Point2d>* points = new std::vector<Point2d>();
      for (int i=0; i<ms.height;i++) {
      const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
      for (int j=0; j<ms.width; j++) {
      if (*ptr != 0) {
      points->push_back(Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
      }
      ptr++;
      }
      }
    */
    return points;
}

/// Optimized for seafloor operations
ScanBuilderRedCloud::ScanBuilderRedCloud() : didsonFrame(0), filteredImage(0), filteredImage2(0)
{
    CvSize sonarImageSize = cvSize(96, 512);
    sonarFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);	
    filteredFrame3 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);
}

ScanBuilderRedCloud::~ScanBuilderRedCloud()
{
    cvReleaseImage(&sonarFrame);
    cvReleaseImage(&filteredFrame);
    cvReleaseImage(&filteredFrame2);
    cvReleaseImage(&filteredFrame3);
    cvReleaseImage(&filteredImage);
    cvReleaseImage(&filteredImage2);
    cvReleaseImage(&filteredImage3);
    delete didsonFrame;
}

std::vector<isam::Point2d>* ScanBuilderRedCloud::getPoints(hauv::didson_t* didsonData)
{
    // NOTE: could the dimension be variable, i.e. not 96x512
    if (didsonFrame == NULL) {
        didsonFrame = new DidsonFrame(didsonData);
        didsonFrame->makeNewImage(300);
    } else {
        didsonFrame->setFrame(didsonData);
    }
    MapScale ms = didsonFrame->getMapScale();

    if (filteredImage2 == NULL) {
        CvSize sonarImageSize = cvSize(ms.width, ms.height);
        filteredImage = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);
        filteredImage2 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);
        filteredImage3 = cvCreateImage(sonarImageSize, IPL_DEPTH_8U, 1);
    }
    memcpy(sonarFrame->imageData, didsonData->m_cData, 96*512);
    // Filter image

    // 1. Calculate a gradient for the image
    float smoothFilter[9];
    CvMat smoothKernel = cvMat(3, 3, CV_32FC1, smoothFilter);
    cvSet(&smoothKernel, cvScalar(0.0));
    cvmSet(&smoothKernel, 1, 1, 1.0);
    cvFilter2D(sonarFrame, filteredFrame, &smoothKernel);

    //float shadowFilter[11] = {-1.0, -1.0, -1.0, -1.0, -1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ///float shadowFilter[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 5.0, -1.0, -1.0, -1.0, -1.0, -1.0};

    //float shadowFilter[9] = {-1.0, -1.0, -1.0, -1.0, 4.0, 0.0, 0.0, 0.0, 0.0};
    float shadowFilter[9] = {0.0, 0.0, 0.0, 0.0, 4.0, -1.0, -1.0, -1.0, -1.0};

    //float shadowFilter[7] = { -1.0, -1.0, -1.0, 3.0, 0.0, 0.0, 0.0 };
    //float shadowFilter[7] = { 0.0,0.0,0.0,3.0,-1.0, -1.0, -1.0};
    //CvMat shadowKernel = cvMat(7, 1, CV_32FC1, shadowFilter);
    CvMat shadowKernel = cvMat(9, 1, CV_32FC1, shadowFilter);
    //CvMat shadowKernel = cvMat(11, 1, CV_32FC1, shadowFilter);

    //cvFilter2D(filteredFrame, filteredFrame2, &shadowKernel);
    cvSmooth( filteredFrame, filteredFrame2, CV_MEDIAN, 5, 1);
    cvSmooth( filteredFrame2, filteredFrame, CV_GAUSSIAN, 5, 1);
    //  cvSmooth( filteredFrame2, filteredFrame, CV_BILATERAL, 5, 5);
    //  cvCopy(filteredFrame2,filteredFrame);
    cvFilter2D(filteredFrame, filteredFrame2, &shadowKernel);

    // 2. Create a histogram and label top percentage as highlights or points of interest
    //			The goal is to pick out consistent features in the sonar image
    //int mint;
    int maxt;
    float v;
    int histSizes[1] = {256};
    float g_ranges[] = {0, 255};
    float* ranges = {g_ranges};
    CvHistogram* hist = cvCreateHist(1, histSizes, CV_HIST_ARRAY, &ranges, 1);
    cvCalcHist(&filteredFrame2, hist);
    cvNormalizeHist(hist, 1.0);

    // Accumulate the histogram
    //mint = -1;
    maxt = -1;
    v = 0.0;
    for (int i=0;i<256;i++) {
        v = v + cvQueryHistValue_1D(hist, i);
        if (maxt<0 && v >= 0.85) maxt = i;
    }
		 
    cvReleaseHist(&hist);
    cvThreshold(filteredFrame2, filteredFrame2, maxt, 1, CV_THRESH_BINARY);

    // 3. Throw out spurious pixel by counting the number of highlights in
    //		a box surrounding the pixel
    // Count filter
    float countFilter[100];
    CvMat countKernel = cvMat(9, 1, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvFilter2D(filteredFrame2, filteredFrame, &countKernel);
    cvThreshold(filteredFrame, filteredFrame, 5, 1, CV_THRESH_BINARY);

    // Detect long streaks and kill
    /*
      for (int y=0; y<512; ++y) {
      int max_streak = int(96 * 0.3);
      int max_total_streak = int(96 * 0.4);
      int current_streak = 0;
      int total_streak = 0;
      char* row = (char*)(filteredFrame->imageData + y*96);
      for (int x=0; x<96; ++x) {
      if (row[x] > 0) {
      ++current_streak;
      ++total_streak;
      }
      else current_streak = 0;

      if (current_streak>max_streak || total_streak>max_total_streak) {
      for (int i=0; i<96; ++i) row[i] = 0;
      break;
      }
      }
      }
    */

    //    cvThreshold(filteredFrame3, filteredFrame3, 1, 1, CV_THRESH_BINARY);
    //    cvFilter2D(filteredFrame3, filteredFrame, &countKernel);
    //    cvThreshold(filteredFrame, filteredFrame, 1, 255, CV_THRESH_BINARY);

    // 4. Project the sensor data from measurement space (polar) into Cartesian coordinate frame
    //			NOTE: This is an uncertain transformation because we do not know the altitude/elevation
    //						of the points in measurement space. We project assuming elevation 0.
    filteredFrame->imageData[0] = 0;
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame->imageData, (uint8_t*)filteredImage2->imageData);
    didsonFrame->copyFrameToImage((uint8_t*)filteredFrame2->imageData, (uint8_t*)filteredImage->imageData);

    //  cvErode(filteredImage2, filteredImage2);
    //  cvErode(filteredImage, filteredImage);
    int CVCLOSE_ITR = 1;
    cvMorphologyEx( filteredImage, filteredImage, 0, 0, CV_MOP_OPEN, CVCLOSE_ITR );
    cvMorphologyEx( filteredImage, filteredImage, 0, 0, CV_MOP_CLOSE, CVCLOSE_ITR );

    cvMorphologyEx( filteredImage2, filteredImage2, 0, 0, CV_MOP_OPEN, CVCLOSE_ITR );
    cvMorphologyEx( filteredImage2, filteredImage2, 0, 0, CV_MOP_CLOSE, CVCLOSE_ITR );


    countKernel = cvMat(5, 10, CV_32FC1, countFilter);
    cvSet(&countKernel, cvScalar(1.0));
    cvThreshold(filteredImage2, filteredImage3, 0, 1, CV_THRESH_BINARY);
    cvFilter2D(filteredImage3, filteredImage2, &countKernel);
    cvThreshold(filteredImage2, filteredImage2, 20, 1, CV_THRESH_BINARY);

    cvFilter2D(filteredImage2, filteredImage3, &countKernel);
    cvThreshold(filteredImage3, filteredImage2, 20, 255, CV_THRESH_BINARY);

    // 5. Extract the remaining highlight pixels and return as a vector of points
    // Extract points from filteredImage2

    std::vector<isam::Point2d>* points = new std::vector<isam::Point2d>();

    // Mask top and bottom (TODO: should be in sensor coordinates)
    double masktop = 50;
    double maskbottom = 50;

    for (int i=masktop; i<ms.height-maskbottom;i++) {
        const char* ptr1 = (const char*)(filteredImage->imageData + i*ms.width);
        const char* ptr = (const char*)(filteredImage2->imageData + i*ms.width);
        //		const char* ptrImage = (const char*)(sonarImage->imageData + i*ms.width);

        for (int j=0; j<ms.width; j++) {
            if (*ptr != 0 && *ptr1 != 0) { // && *ptrImage > 40) {
                points->push_back(isam::Point2d((-i+ms.y0)*ms.hs, (j-ms.x0)*ms.ws));
            }
            ptr++;
            ptr1++;
        }
    }

    return points;
}

