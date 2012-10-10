/**
 * @file didson.cpp
 * @brief A library to work with didson data
 * @author Hordur Johannsson
 * @version $Id:  $
 */

#include "didson.h"

// Constants
const float degtorad = 3.14159/180.;
const float radtodeg = 180./3.14159;

DidsonFrame::DidsonFrame(hauv::didson_t* frame)
{
    this->frame = frame;
    numbeams = 96;
    smooth = 1;		// TODO: support smooth 1,4,8
    image = NULL;
    map = NULL;
    m_imagexsize = 0;
    m_imageysize = 0;

    // TODO: There relly should be a call to setFrame
    m_minrange = frame->m_nWindowStart * 0.375;
    double windowLength[4] = {1.125, 2.25, 4.5, 9.0};
    m_maxrange = m_minrange + windowLength[frame->m_nWindowLength];
}

DidsonFrame::~DidsonFrame()
{
    delete image;
    //	delete frame;
    delete map;
}

int DidsonFrame::getPolar(int x, int y, int & r, int & th)
{
    int idx = y*m_imagexsize + x;
    int d_idx = map[idx]; // index into frame
    // Invalid x,y are mapped to entry zero (NOTE: we actually loose 1 point)
    if (d_idx == 0) return 0;

    r = d_idx / 96;
    th = d_idx % 96;
    return 1;
}

void DidsonFrame::getRanges(float & minrange, float & maxrange) 
{
    minrange = m_minrange;
    maxrange = m_maxrange;
}

void DidsonFrame::setFrame(hauv::didson_t* newFrame)
{
    // TODO: Check if frame size has changed and recreate map

    //	if (frame != NULL) delete frame;
    frame = newFrame;

    // Calculate ranges in meters
    m_minrange = frame->m_nWindowStart * 0.375;
    double windowLength[4] = {1.125, 2.25, 4.5, 9.0};
    m_maxrange = m_minrange + windowLength[frame->m_nWindowLength];

    for (int i=0;i<mapscale.width*mapscale.height;i++) {
        image[i] = frame->m_cData[map[i]];
    }
}

void DidsonFrame::copyFrameToImage(uint8_t* data, uint8_t* image)
{
    for (int i=0;i<mapscale.width*mapscale.height;i++) {
        image[i] = data[map[i]];
    }
}

int DidsonFrame::lensDistortion(int nbeams, float theta)
{
    float* a = NULL; 
    float factor = 0.0;
    float a48[4] = {0.0015, -0.0036, 1.3351, 24.0976}; 
    float a189[4] = {0.0015, -0.0036, 1.3351, 24.0978};
    float a96[4] = {0.0030, -0.0055, 2.6829, 48.04};
    float a381[4] = {0.0030, -0.0055, 2.6829, 48.04};

    switch (nbeams) {
    case 48:
        factor = 1.0;
        a = a48;
        break;
    case 189:
        factor = 4.026;
        a = a189;
        break;
    case 96:
        factor = 1.012;
        a = a96;
        break;
    case 381:
        factor = 4.05;
        a = a381;
        break;
    }
    return (int)round(factor*(  a[0]*theta*theta*theta
                                + a[1]*theta*theta 
                                + a[2]*theta
                                + a[3] + 1));
}

/**
 * Calculates a map from cartesian to polar coordinates
 * 
 * ixsize  - number of pixels in horizontal direction in image space
 * rmax    - maximum range in meters
 * rmin    - minimum range in meters
 * halffov - one-half of sector field of view in degrees
 * nbins   - number of range bins in sample space
 */
int* DidsonFrame::mapscan(int ixsize, double rmax, double rmin, double halffov, int nbeams, int nbins, int* iysizeOut )
{
    // conversion of Matlab DIDSON code from Soundmetrics ---
    //float d2 = rmax*cos(halffov*degtorad); // distance from point scan touches image boundary to origin
    float d3 = rmin*cos(halffov*degtorad); // bottom of image frame to r,theta origin in meters
    float c1 = 511/(rmax-rmin); // precalculation of constants used in do loop below
    //float c2 = (nbeams-1)/(2*halffov);
    float gamma = ixsize/(2*rmax*sin(halffov*degtorad)); // Ratio of pixel number to position in meters

    int iysize = (int)(gamma*(rmax - d3) + 0.5); // number of pixels in image in vertical direction

    int* svector = new int[ixsize*iysize]; // Stores the index map

    // ix,iy   - coordinates of a pixel in image space
    for (int iy = 1;iy<=iysize;iy++)
        {
            for (int ix = 1;ix<=ixsize;ix++)
                {
                    float x = ((ix-1) - ixsize/2)/gamma; // Convert from pixels to meters

                    float z = 0.0;
                    float y = rmax - (iy-1)/gamma; // Convert from pixels to meters

                    float r = sqrt(y*y + x*x + z*z); // Convert to polar coordinates
                    float theta = radtodeg*atan2(x,y); // Theta is in degrees
                    int binnum = (int)((r - rmin)*c1 + 1.5); // the rangebin number
                    int beamnum = lensDistortion(nbeams, theta); // Remove the lens distortation using empirical formula
                    int pos = 1;
                    if ((beamnum > 0) && (beamnum <= nbeams) && (binnum > 0) && (binnum <= nbins)) {
                        pos = (binnum-1)*nbeams + beamnum;
                    }
                    svector[(iy-1)*ixsize + ix - 1] = pos-1;
                }
        }
    *iysizeOut = iysize;
    return svector;
}

void DidsonFrame::makeNewImage(int imagexsize)
{
    float minrange = frame->m_nWindowStart * 0.375;
    double windowLength[4] = {1.125, 2.25, 4.5, 9.0};
    float maxrange = minrange + windowLength[frame->m_nWindowLength];
 
    int nbeams = numbeams*smooth - smooth + 1;
    int imageysize = 0;

    if (map != NULL) delete map;
    map = mapscan(imagexsize, maxrange, minrange, 14.4, nbeams, 512, &imageysize);

    m_imagexsize = imagexsize;
    m_imageysize = imageysize;

    float ws = 2.0*maxrange*sin(14.25*degtorad)/imagexsize; // widthscale meters/pixels
    float hs = (maxrange - minrange*cos(14.25*degtorad))/imageysize; // heightscale meters/pixels
    int y0 = maxrange/hs; // origin in height direction
    int x0 = imagexsize/2; // origin in width direction

    mapscale.ws = ws;
    mapscale.hs = hs;
    mapscale.y0 = y0;
    mapscale.x0 = x0;
    mapscale.width = imagexsize;
    mapscale.height = imageysize;

    if (image != NULL) delete image; 
    image = new uint8_t[imagexsize*imageysize];
    frame->m_cData[0] = 0;

    for (int i=0;i<imagexsize*imageysize;i++) {
        image[i] = frame->m_cData[map[i]];
    }
}
