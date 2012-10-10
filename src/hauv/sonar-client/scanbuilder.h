#ifndef _HAUV_SCANBUILDER_H
#define _HAUV_SCANBUILDER_H

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <vector>

#include <isam/Point2d.h>
#include <isam/Pose2d.h>

#include "didson.h"

/**
 * @file scanbuilder.h
 * @brief A class that extracts feature points from sonar images
 * @author Hordur Johannsson
 * @version $Id:  $
 */

class ScanBuilder
{
  public:
    virtual ~ScanBuilder() {};
    /**
     * Returns points representing the scan.
     */
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData) = 0;
};

class ScanBuilderA : public ScanBuilder
{
  public:
    ScanBuilderA();
    virtual ~ScanBuilderA();
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData);
  private:
    DidsonFrame* didsonFrame;
    IplImage* sonarFrame;
    IplImage* filteredFrame;
    IplImage* filteredFrame2;
    IplImage* filteredImage;
    IplImage* filteredImage2;
};

// Optimized for tank operations
class ScanBuilderB : public ScanBuilder
{
  public:
    ScanBuilderB();
    virtual ~ScanBuilderB();
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData);
  private:
    DidsonFrame* didsonFrame;
    IplImage* sonarFrame;
    IplImage* filteredFrame;
    IplImage* filteredFrame2;
    IplImage* filteredImage2;
};


// Optimized for ship hull operations
class ScanBuilderShip : public ScanBuilder
{
  public:
    ScanBuilderShip();
    virtual ~ScanBuilderShip();
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData);
  private:
    DidsonFrame* didsonFrame;
    IplImage* sonarFrame;
    IplImage* filteredFrame;
    IplImage* filteredFrame2;
    IplImage* filteredImage2;
};

// Optimized for seafloor (at the sailing pavilion)
class ScanBuilderSeafloor : public ScanBuilder
{
  public:
    ScanBuilderSeafloor();
    virtual ~ScanBuilderSeafloor();
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData);
  private:
    DidsonFrame* didsonFrame;
    IplImage* sonarFrame;
    IplImage* filteredFrame;
    IplImage* filteredFrame2;
    IplImage* filteredImage2;
};


// Optimized for seafloor (at the sailing pavilion)
// This is version 2, previous version was used for run3 dataset
//  on 2009/11/30
class ScanBuilderSeafloor2 : public ScanBuilder
{
  public:
    ScanBuilderSeafloor2();
    virtual ~ScanBuilderSeafloor2();
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData);
  private:
    DidsonFrame* didsonFrame;
    IplImage* sonarFrame;
    IplImage* filteredFrame;
    IplImage* filteredFrame2;
    IplImage* filteredImage;
    IplImage* filteredImage2;
};

class ScanBuilderOceanus : public ScanBuilder
{
  public:
    ScanBuilderOceanus();
    virtual ~ScanBuilderOceanus();
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData);
  private:
    DidsonFrame* didsonFrame;
    IplImage* sonarFrame;
    IplImage* filteredFrame;
    IplImage* filteredFrame2;
    IplImage* filteredImage;
    IplImage* filteredImage2;
    double lastThreshold;
};

class ScanBuilderOceanus2 : public ScanBuilder
{
  public:
    ScanBuilderOceanus2();
    virtual ~ScanBuilderOceanus2();
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData);
  private:
    DidsonFrame* didsonFrame;
    IplImage* sonarFrame;
    IplImage* filteredFrame;
    IplImage* filteredFrame2;
    IplImage* filteredImage;
    IplImage* filteredImage2;
    double lastThreshold;
};

// Tune during the operations on the Red Cloud
//
class ScanBuilderRedCloud : public ScanBuilder
{
  public:
    ScanBuilderRedCloud();
    virtual ~ScanBuilderRedCloud();
    virtual std::vector<isam::Point2d>* getPoints(hauv::didson_t* didsonData);
  private:
    DidsonFrame* didsonFrame;
    IplImage* sonarFrame;
    IplImage* filteredFrame;
    IplImage* filteredFrame2;
    IplImage* filteredFrame3;
    IplImage* filteredImage;
    IplImage* filteredImage2;
    IplImage* filteredImage3;
};

#endif
