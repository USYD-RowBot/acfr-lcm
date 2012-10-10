/*
 *  These functions implement Contrast Limited Adaptive Histogram
 *  Specification (CLAHS).  The main routine (clahs) expects an input
 *  image that is stored contiguously in memory; the clahs() output
 *  image overwrites the original input image and has the same minimum
 *  and maximum values (which must be provided by the user).
 *
 *  This code is largely based upon source from:
 *
 *  ANSI C code from the article
 *  "Contrast Limited Adaptive Histogram Equalization"
 *  by Karel Zuiderveld, karel@cv.ruu.nl
 *  in "Graphics Gems IV", Academic Press, 1994
 *
 * and changes made by Jonathan Howland (jhowland@whoi.edu) to accept
 * a specifier as to the type of histogram to use.  This actually turns
 * CLAHE into CLAHS, histogram specification. 
 *
 * reference:
 * Digital Image Processing by Gonzalez and Woods, pp 180+, Handbook
 * ofComputer Vision Algorithms in Image Algebra by Ritter and Wilson,
 * p 67.
 *
 * 2010 Ryan Eustice eustice@umich.edu
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <gsl/gsl_math.h>

#include "clahs.h"

typedef enum {IMAGE_8U=8, IMAGE_16U=16} image_type_t;

const size_t MAX_NTILESR = 128;  // max. # contextual regions in row-direction
const size_t MAX_NTILESC = 128;  // max. # contextual regions in col-direction



/* To speed up histogram clipping, the input image [Min,Max] is scaled down to
 * [0, nBins-1]. This function calculates the LUT. */
static void
MakeLut (uint16_t *LUT, size_t Min, size_t Max, size_t nBins, size_t NR_OF_GREY)
{
    const size_t BinSize = (1 + (Max - Min) / nBins);
    for (size_t i = 0; i < NR_OF_GREY; i++) {
        if (i < Min)
            LUT[i] = 0;
        else if (i <= Max)
            LUT[i] = (i - Min) / BinSize;
        else
            LUT[i] = nBins-1;
    }
}

/* This function classifies the greylevels present in the array image into
 * a greylevel histogram. The LookupTable specifies the relationship
 * between the greyvalue of the pixel (typically between 0 and 255) and
 * the corresponding bin in the histogram (usually containing only 128 bins).
 */
static void
MakeHistogram8U (uint8_t *Image, size_t XRes, size_t SizeX, size_t SizeY,
                 uint32_t *Histogram, size_t NrGreylevels, uint16_t *LookupTable)
{
    memset (Histogram, 0L, sizeof(0L)*NrGreylevels); // clear histogram
    
    uint8_t *Tile = Image;
    for (size_t i=0; i < SizeY; i++) {
        uint8_t *TileEnd = Tile + SizeX;
        while (Tile < TileEnd)
            Histogram[LookupTable[*Tile++]]++;
        /* point to next row of tile */
        TileEnd += XRes;
        Tile = TileEnd - SizeX;
    }
}

static void
MakeHistogram16U (uint16_t *Image, size_t XRes, size_t SizeX, size_t SizeY,
                  uint32_t *Histogram, size_t NrGreylevels, uint16_t *LookupTable)
{
    memset (Histogram, 0L, sizeof(0L)*NrGreylevels); // clear histogram

    uint16_t *Tile = Image;
    for (size_t i=0; i < SizeY; i++) {
        uint16_t *TileEnd = Tile + SizeX;
        while (Tile < TileEnd)
            Histogram[LookupTable[*Tile++]]++;
        /* point to next row of tile */
        TileEnd += XRes;
        Tile = TileEnd - SizeX;
    }
}

/* This function performs clipping of the histogram and redistribution of bins.
 * The histogram is clipped and the number of excess pixels is counted. Afterwards
 * the excess pixels are equally redistributed across the whole histogram (providing
 * the bin count is smaller than the cliplimit).
 */
static void
ClipHistogram (uint32_t *Histogram, size_t NrGreylevels, size_t ClipLimit)
{
    /* 1) calculate total number of excess pixels */
    size_t NrExcess = 0;
    for (size_t i = 0; i < NrGreylevels; i++) {
        long BinExcess = Histogram[i] - ClipLimit;
        if (BinExcess > 0)
            NrExcess += BinExcess; // excess in current bin
    }

    /* 2) clip histogram and redistribute excess pixels in each bin */
    size_t BinIncr = NrExcess / NrGreylevels; // average bin increment
    size_t Upper = ClipLimit - BinIncr; // bins largers than Upper set to cliplimit
    for (size_t i=0; i < NrGreylevels; i++) {
        if (Histogram[i] > ClipLimit)
            Histogram[i] = ClipLimit; // clip bin
        else if (Histogram[i] > Upper) { // high bin count
                NrExcess -= Histogram[i] - Upper;
                Histogram[i] = ClipLimit;
        }
        else {
            NrExcess -= BinIncr;
            Histogram[i] += BinIncr;
        }
    }

    /* 3) redistribute remaining excess */
    while (NrExcess) {
        uint32_t *EndPointer = Histogram + NrGreylevels;
        uint32_t *Histo = Histogram;
        while (NrExcess && Histo < EndPointer) {
            size_t StepSize = NrGreylevels / NrExcess;
            StepSize = (StepSize > 1) ? StepSize : 1; // stepsize at least 1
            for (uint32_t *BinPointer=Histo; BinPointer < EndPointer && NrExcess; BinPointer += StepSize) {
                if (*BinPointer < ClipLimit) { // reduce excess
                    (*BinPointer)++;
                    NrExcess--; 
                }
            }
            Histo++; // restart redistributing on other bin location
        }
    }
}


/* This function calculates the equalized lookup table (mapping) by
 * cumulating the input histogram. Note: lookup table is rescaled in range [Min..Max].
 */
static void
MapHistogram (uint32_t *Histogram, size_t Min, size_t Max, size_t NrGreylevels,
              size_t NrOfPixels, vis_clahs_dist_t heq_type, double alpha)
{
    size_t Sum = 0;
    const double eps = DBL_EPSILON;
    size_t valSpread = Max - Min;

    switch (heq_type) {
    case VIS_CLAHS_DIST_UNIFORM: 
    {
        const float Scale = ((float)valSpread) / ((float)NrOfPixels);
        for (size_t i = 0; i < NrGreylevels; i++) {
            Sum += Histogram[i];
            Histogram[i] = GSL_MIN (Min + Sum*Scale, Max);
        }
    }
    break;

    case VIS_CLAHS_DIST_EXPONENTIAL:
    {
        /* pdf = alpha*exp(-alpha*t)*U(t)
         * cdf = 1-exp(-alpha*t) 
         */
        double vmax = 1.0 - exp (-alpha);
        for (size_t i=0; i < NrGreylevels; i++) {
            Sum += Histogram[i];
            double val = (vmax*Sum) / ((float)NrOfPixels);
            if (val >= 1) val = 1.0-eps; // avoid log(0)
            double temp = -1.0/alpha * log (1.0-val);
            Histogram[i] = GSL_MIN (Min + temp * valSpread, Max);
        }
    }   
    break;

    case VIS_CLAHS_DIST_RAYLEIGH: 
    {
        /* suitable for underwater imagery
         * pdf = (t./alpha^2).*exp(-t.^2/(2*alpha^2))*U(t)
         * cdf = 1-exp(-t.^2./(2*alpha^2)) 
         */
        double temp;
        double hconst = 2.0 * alpha * alpha;
        double vmax = 1.0 - exp (-1.0/hconst);
        for (size_t i=0; i < NrGreylevels; i++) {
            Sum += Histogram[i];
            double val = (vmax*Sum) / (float)NrOfPixels;
            if (val >= 1) val = 1.0-eps; // avoid log(0)
            temp = sqrt (-hconst * log (1.0-val));
            Histogram[i] = GSL_MIN (Min + temp*valSpread, Max);
        }
    }
    break;
    }

}

/* This function calculates the new greylevel assignments of pixels within a submatrix
 * of the image with size XSize and YSize. This is done by a bilinear interpolation
 * between four different mappings in order to eliminate boundary artifacts.
 *
 * Image     - pointer to input/output image
 * ImageType - IMAGE_8U or IMAGE_16U
 * XRes      - resolution of image in x-direction
 * Map*      - mappings of greylevels from histograms
 * XSize     - uiXSize of image submatrix
 * YSize     - uiYSize of image submatrix
 * LUT	     - lookup table containing mapping greyvalues to bins
 */
static void
Interpolate (void *pImage, image_type_t ImageType, size_t XRes, 
             uint32_t *MapLU, uint32_t *MapRU, uint32_t *MapLB, uint32_t *MapRB,
             size_t XSize, size_t YSize, uint16_t *LUT)
{
    const size_t Incr = XRes - XSize; // pointer increment after processing row
    const size_t NormFactor = XSize*YSize;

    switch (ImageType) {
    case IMAGE_8U:
    {
        uint8_t *Image = pImage;
        for (size_t YCoef = 0, YInvCoef = YSize; YCoef < YSize; YCoef++, YInvCoef--, Image += Incr) {
            for (size_t XCoef = 0, XInvCoef = XSize; XCoef < XSize; XCoef++, XInvCoef--) {
                uint8_t GreyValue = LUT[*Image];
                *Image++ = (YInvCoef * (XInvCoef*MapLU[GreyValue] + XCoef*MapRU[GreyValue]) +
                               YCoef * (XInvCoef*MapLB[GreyValue] + XCoef*MapRB[GreyValue]) ) / NormFactor;
            }
        }
    }
    break;

    case IMAGE_16U:
    {
        uint16_t *Image = pImage;
        for (size_t YCoef = 0, YInvCoef = YSize; YCoef < YSize; YCoef++, YInvCoef--, Image += Incr) {
            for (size_t XCoef = 0, XInvCoef = XSize; XCoef < XSize; XCoef++, XInvCoef--) {
                uint16_t GreyValue = LUT[*Image];
                *Image++ = (YInvCoef * (XInvCoef*MapLU[GreyValue] + XCoef*MapRU[GreyValue]) +
                               YCoef * (XInvCoef*MapLB[GreyValue] + XCoef*MapRB[GreyValue]) ) / NormFactor;
            }
        }
    }
    break;
    }
}

/* check if image needs to be padded; pad if necessary;
 * padding occurs if any dimensions of a single tile is an odd number
 * and/or when image dimensions are not divisible by the selected number of tiles
 */
static void *
PadImage (void *Image, size_t width, size_t height, image_type_t ImageType,
          size_t NrX, size_t NrY, size_t *XSize, size_t *YSize, size_t *XRes, size_t *YRes)
{
    *XSize = width / NrX;
    *YSize = height / NrY;

    bool rowDiv = ((height % NrY) == 0);
    bool colDiv = ((width % NrX) == 0);

    bool rowEven = ((*YSize % 2) == 0);
    bool colEven = ((*XSize % 2) == 0);

    void *pImage = NULL;
    if (rowDiv && colDiv && rowEven && colEven) {
        pImage = Image;
        *XRes = width;
        *YRes = height;
    }
    else {
        size_t padRow=0, padCol=0;

        if (!rowDiv) {
            *YSize = ceil ((double)height / (double)NrY);
            padRow = (*YSize)*NrY - height;
        }

        if (!colDiv) {
            *XSize = ceil ((double)width / (double)NrX);
            padCol = (*XSize)*NrX - width;
        }
        
        // check if tile dimensions are even numbers
        rowEven = ((*YSize % 2) == 0);
        colEven = ((*XSize % 2) == 0);

        if (!rowEven) padRow += NrY;
        if (!colEven) padCol += NrX;
        
        *YRes = height + padRow;
        *XRes = width + padCol;

        *YSize = *YRes / NrY;
        *XSize = *XRes / NrX;

        const int bpp = ImageType / 8; // bytes per pixel
        pImage = malloc ((*XRes) * (*YRes) * bpp);

        // copy and symmetrically pad bottom row and right column image data
        uint8_t *Image8U = Image;
        uint8_t *pImage8U = pImage;
        for (size_t i=0; i<height; i++, Image8U+=width*bpp, pImage8U+=(*XRes)*bpp) {
            memcpy (pImage8U, Image8U, width*bpp);
            for (size_t j=width, k=width-1; j<*XRes; j++, k--)
                memcpy (pImage8U+j*bpp, Image8U+k*bpp, bpp);
        }
        uint8_t *prevRow = pImage8U-(*XRes)*bpp;
        for (size_t i=height; i<*YRes; i++, pImage8U+=(*XRes)*bpp, prevRow-=(*XRes)*bpp)
            memcpy (pImage8U, prevRow, (*XRes)*bpp);
    }

    return pImage;
}


int
vis_clahs (void *Image, size_t width, size_t height, size_t depth, const vis_clahs_opts_t *_opts)
{
    // determine image storage type
    image_type_t ImageType;
    if (depth <= 8)
        ImageType = IMAGE_8U;
    else if (depth <= 16)
        ImageType = IMAGE_16U;
    else
        return VIS_CLAHS_RET_ERROR_DEPTH;

    vis_clahs_opts_t opts;
    if (_opts == NULL)
        opts = vis_clahs_default_opts ();
    else
        memcpy (&opts, _opts, sizeof (vis_clahs_opts_t));

    // check if we should set output intensity to full range for image type
    const size_t NR_OF_GREY = pow (2, depth);
    if (opts.range[0] == opts.range[1]) {
        opts.range[0] = 0; 
        opts.range[1] = NR_OF_GREY - 1;
    }

    // error check input args
    if (opts.tiles[1] < 2 || opts.tiles[1] > MAX_NTILESC)  // at least 4 contextual regions req'd
        return VIS_CLAHS_RET_ERROR_NTILESC;
    if (opts.tiles[0] < 2 || opts.tiles[0] > MAX_NTILESR) 
        return VIS_CLAHS_RET_ERROR_NTILESR;
    if (opts.range[1] > NR_OF_GREY)                      // output range too large for image type
        return VIS_CLAHS_RET_ERROR_RANGEMAX;
    if (opts.range[0] > opts.range[1]) 
        return VIS_CLAHS_RET_ERROR_RANGEMIN;
    if (opts.cliplimit < 0.0 || opts.cliplimit > 1.0)    // must be in the range [0, 1]
        return VIS_CLAHS_RET_ERROR_CLIPLIMIT;
    if (!(opts.alpha > 0 ))
        return VIS_CLAHS_RET_ERROR_ALPHA;

    // map to KZ's notation for ease of code translation
    const vis_clahs_dist_t heq_type = opts.dist;
    const double alpha = opts.alpha;
    const size_t NrBins = opts.bins;
    const size_t NrY = opts.tiles[0]; // row-dir
    const size_t NrX = opts.tiles[1]; // col-dir
    const size_t Min = opts.range[0];
    const size_t Max = opts.range[1];
    size_t XRes, YRes, XSize, YSize;
    void *pImage = PadImage (Image, width, height, ImageType, NrX, NrY, &XSize, &YSize, &XRes, &YRes);
    const size_t NrPixels = XSize*YSize; // # of pixels per tile    

    // pointer to histogram mappings
    uint32_t *MapArray = malloc (sizeof(uint64_t) * NrX * NrY * NrBins);
    if (!MapArray) // Not enough memory! (try reducing opt.nBins)
        return VIS_CLAHS_RET_ERROR_MALLOC;

    // calculate actual cliplimit
    const size_t minClipLimit = ceil ((float)NrPixels / (float)NrBins);
    const size_t ClipLimit = minClipLimit + round (opts.cliplimit * (NrPixels - minClipLimit));

    // make lookup table used for mapping input greyvalues to output bins
    uint16_t LUT[NR_OF_GREY];
    MakeLut (LUT, Min, Max, NrBins, NR_OF_GREY);

    
    // calculate greylevel mappings for each contextual region
    switch (ImageType) {
    case IMAGE_8U:
    {
        uint8_t *ImPointer8U = pImage;
        for (int Y = 0; Y < NrY; Y++) {
            for (int X = 0; X < NrX; X++, ImPointer8U += XSize) {
                uint32_t *Hist = &MapArray[NrBins * (Y * NrX + X)];
                MakeHistogram8U (ImPointer8U, XRes, XSize, YSize, Hist, NrBins, LUT);
                ClipHistogram (Hist, NrBins, ClipLimit);
                MapHistogram (Hist, Min, Max, NrBins, NrPixels, heq_type, alpha);
            }
            ImPointer8U += (YSize - 1) * XRes; // skip ahead to next row of tiles
        }
    }
    break;

    case IMAGE_16U:
    {
        uint16_t *ImPointer16U = pImage;
        for (size_t Y = 0; Y < NrY; Y++) {
            for (size_t X = 0; X < NrX; X++, ImPointer16U += XSize) {
                uint32_t *Hist = &MapArray[NrBins * (Y * NrX + X)];
                MakeHistogram16U (ImPointer16U, XRes, XSize, YSize, Hist, NrBins, LUT);
                ClipHistogram (Hist, NrBins, ClipLimit);
                MapHistogram (Hist, Min, Max, NrBins, NrPixels, heq_type, alpha);
            }
            ImPointer16U += (YSize - 1) * XRes; // skip ahead to next row of tiles
        }
    }
    break;
    }

    // interpolate greylevel mappings to get CLAHS image
    uint8_t *ImPointer = pImage;    // 8-bit pointer
    const int bpp = ImageType / 8; // bytes per pixel
    for (size_t Y = 0; Y <= NrY; Y++) {
        size_t SubY, YU, YB;
        if (Y == 0) { /* top row */
            SubY = YSize/2; YU = 0; YB = YU;
        }
        else if (Y == NrY) { /* bottom row */
            SubY = YSize/2; YU = NrY-1; YB = YU;
        }
        else {
            SubY = YSize; YU = Y-1; YB = YU+1;
        }

        for (size_t X = 0; X <= NrX; X++) {
            size_t SubX, XL, XR;
            if (X == 0) { /* left column */
                SubX = XSize/2; XL = 0; XR = 0;
            }
            else if (X == NrX) { /* right column */
                SubX = XSize/2; XL = NrX-1; XR = XL;
            }
            else {
                SubX = XSize; XL = X-1; XR = XL+1;
            }

            uint32_t *LU = &MapArray[NrBins * (YU * NrX + XL)];
            uint32_t *RU = &MapArray[NrBins * (YU * NrX + XR)];
            uint32_t *LB = &MapArray[NrBins * (YB * NrX + XL)];
            uint32_t *RB = &MapArray[NrBins * (YB * NrX + XR)];

            Interpolate (ImPointer, ImageType, XRes, LU, RU, LB, RB, SubX, SubY, LUT);
            ImPointer += SubX * bpp;  /* set pointer on next matrix */
        }
        ImPointer += (SubY - 1)*XRes * bpp;
    }

    if (pImage != Image) {
        // copy padded image back to source image
        switch (ImageType) {
        case IMAGE_8U:
        {
            uint8_t *Image8U = Image;
            uint8_t *pImage8U = pImage;
            for (size_t i=0; i<height; i++, Image8U+=width, pImage8U+=XRes)
                memcpy (Image8U, pImage8U, width*sizeof(uint8_t));
        }
        break;

        case IMAGE_16U:
        {
            uint16_t *Image16U = Image;
            uint16_t *pImage16U = pImage;
            for (size_t i=0; i<height; i++, Image16U+=width, pImage16U+=XRes)
                memcpy (Image16U, pImage16U, width*sizeof(uint16_t));
        }
        break;
        }
        free (pImage);
        free (MapArray);
        return VIS_CLAHS_RET_SUCCESS_PAD;
    }

    free (MapArray);
    return VIS_CLAHS_RET_SUCCESS;
}


vis_clahs_opts_t
vis_clahs_default_opts (void)
{
    vis_clahs_opts_t opts = {
        .tiles = {8, 8},
        .cliplimit = 0.01,
        .bins = 256,
        .range = {0, 0},
        .dist = VIS_CLAHS_DIST_UNIFORM,
        .alpha = 0.4,
    };
    return opts;
};
