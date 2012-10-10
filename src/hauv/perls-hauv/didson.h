#ifndef _HAUV_DIDSON_H
#define _HAUV_DIDSON_H

/**
 * @file didson.h
 * @brief A library to work with didson data
 * @author Hordur Johannsson
 * @version $Id:  $
 */

#include <cmath>
#include "perls-lcmtypes/hauv_didson_t.h"

/**
 * Stores the parameters of the transformation from DIDSON measurement space
 * into a cartesian image coordinate system. 
 */
class MapScale
{
public:
	float ws;		// The horizontal scale of each pixel in meters
	float hs;   // The vertical scale of each pixel in meters
	int y0;			// Y - origin of the data in image coordinates
	int x0;     // X - origin of the data in image coordinates
	int width;  // Width of the image
	int height; // Height of the image
};

/**
 * A class to work with a single DIDSON frame
 */
class DidsonFrame
{
public:
	DidsonFrame(hauv_didson_t* frame);
	~DidsonFrame();
	void makeNewImage(int imagexsize);

	void setFrame(hauv_didson_t* frame);

	void copyFrameToImage(uint8_t* data, uint8_t* image);

	const uint8_t*  getImage() const { return image; } 
	const hauv_didson_t* getFrame() const { return frame; }
	MapScale getMapScale() const { return mapscale; }

    /**
    * Converts from image space coordinates into measurement coordinates
    *
    * @return 1 if mapping exists and 0 otherwise
    */
    int getPolar(int x, int y, int & r, int & th);

    /**
    * Returns the maximum and minimum ranges for the frame
    */
    void getRanges(float & minrange, float & maxrange);

private:
    // Private methods
    DidsonFrame() {}  // Can not construct an DidsonFrame without a frame

    int lensDistortion(int nbeams, float theta);

    /**
    * Calculates a map from cartesian to polar coordinates
    *
    * ixsize  - number of pixels in horizontal direction in image space
    * rmax    - maximum range in meters
    * rmin    - minimum range in meters
    * halffov - one-half of sector field of view in degrees
    * nbins   - number of range bins in sample space
    */
    int* mapscan(int ixsize, double rmax, double rmin, double halffov, int nbeams, int nbins, int* iysizeOut);

    // Private member variables
    int* map;
    hauv_didson_t *frame;
    uint8_t* image;
    int numbeams;
    int smooth;
    MapScale mapscale;

    // Stores the images sizes from last call to makeNewImage
    int m_imagexsize;
    int m_imageysize;

    // The ranges of the scan
    float m_minrange;
    float m_maxrange;
};

#endif
