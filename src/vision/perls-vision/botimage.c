#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <tiffio.h>
#include <inttypes.h> // needed for PRId64 macros

#include <glib.h>
#include <prosilica/PvApi.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "opencv_util.h"
#include "botimage.h"

int
vis_botimage_is_gray (const bot_core_image_t *bot)
{
    switch (bot->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_GRAY16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32:
        return 1;
    default:
        return 0;
    }
}

int
vis_botimage_is_bayer (const bot_core_image_t *bot)
{
    switch (bot->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB:
        return 1;
    default:
        return 0;
    }
}

int
vis_botimage_is_color (const bot_core_image_t *bot)
{
    switch (bot->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_RGB16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUYV:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV420:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV411P:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_I420:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_NV12:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_RGB16:
        return 1;
    default:
        return 0;
    }
}


IplImage *
vis_botimage_to_iplimage_copy (const bot_core_image_t *bot) {
    IplImage img = vis_botimage_to_iplimage_view (bot);
    return cvCloneImage (&img);
}

IplImage
vis_botimage_to_iplimage_view (const bot_core_image_t *bot) {

    IplImage img;
    CvSize size = {bot->width, bot->height};

    switch (bot->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
        cvInitImageHeader (&img, size, IPL_DEPTH_8U, 1, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16:
        cvInitImageHeader (&img, size, IPL_DEPTH_16U, 1, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB:
        cvInitImageHeader (&img, size, IPL_DEPTH_8U, 1, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB:
        cvInitImageHeader (&img, size, IPL_DEPTH_16U, 1, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR:
        cvInitImageHeader (&img, size, IPL_DEPTH_8U, 3, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_RGB16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16:
        cvInitImageHeader (&img, size, IPL_DEPTH_16U, 3, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA:
        cvInitImageHeader (&img, size, IPL_DEPTH_8U, 4, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUYV:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV420:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV411P:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_I420:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_NV12:
        cvInitImageHeader (&img, size, IPL_DEPTH_8U, 3, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_GRAY16:
        cvInitImageHeader (&img, size, IPL_DEPTH_16S, 1, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_RGB16:
        cvInitImageHeader (&img, size, IPL_DEPTH_16S, 3, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32:
        cvInitImageHeader (&img, size, IPL_DEPTH_32F, 1, IPL_ORIGIN_TL, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_INVALID:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_ANY:
        ERROR ("unimplemented pixel format");
        exit (-1);
        break;
    default:
        ERROR ("unrecognized pixel format");
        exit (-1);
    }
    cvSetData (&img, bot->data, bot->row_stride);

    return img;
}

IplImage *
vis_botimage_to_iplimage_convert (const bot_core_image_t *bot, vis_bot2cv_color_t code)
{
    IplImage img_cv = vis_botimage_to_iplimage_view (bot);
    CvSize size = cvGetSize (&img_cv);

    IplImage *img_out = NULL, *img_tmp = NULL;
    switch (code) {
    case VIS_BOT2CVGRAY:
        img_out = cvCreateImage (size, img_cv.depth, 1);
        if (vis_botimage_is_bayer (bot))
            img_tmp = cvCreateImage (size, img_cv.depth, 3);
        break;

    case VIS_BOT2CVBGR:
        img_out = cvCreateImage (size, img_cv.depth, 3);
        break;

    default:
        ERROR ("unrecognized conversion type");
        exit (-1);
    }


    switch (bot->pixelformat) {
    // gray
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCopy (&img_cv, img_out, NULL);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_GRAY2BGR);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16:
        goto unimplemented;
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_GRAY16:
        goto unimplemented;
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32:
        goto unimplemented;
        break;

    // color
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_out, CV_RGB2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_RGB2BGR);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_out, CV_BGR2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCopy (&img_cv, img_out, NULL);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_RGB16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_out, CV_RGB2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_RGB2BGR);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_out, CV_RGBA2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_RGBA2BGR);
            break;            
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_out, CV_BGRA2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_BGRA2BGR);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUYV:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV420:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV411P:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_I420:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_NV12:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_RGB16:
        goto unimplemented;
        break;

    // bayer
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_tmp, CV_BayerBG2BGR);
            cvCvtColor (img_tmp, img_out, CV_BGR2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_BayerBG2BGR);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_tmp, CV_BayerGB2BGR);
            cvCvtColor (img_tmp, img_out, CV_BGR2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_BayerGB2BGR);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_tmp, CV_BayerGR2BGR);
            cvCvtColor (img_tmp, img_out, CV_BGR2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_BayerGR2BGR);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB:
        switch (code) {
        case VIS_BOT2CVGRAY:
            cvCvtColor (&img_cv, img_tmp, CV_BayerRG2BGR);
            cvCvtColor (img_tmp, img_out, CV_BGR2GRAY);
            break;
        case VIS_BOT2CVBGR:
            cvCvtColor (&img_cv, img_out, CV_BayerRG2BGR);
            break;
        }
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB:
        goto unimplemented;
        break;

    unimplemented:
        ERROR ("unimplemented pixel format");
        exit (-1);
        break;
    default:
        ERROR ("unrecognized pixel format");
        exit (-1);
    }

    if (img_tmp)
        cvReleaseImage (&img_tmp);

    return img_out;
}

bot_core_image_t *
vis_iplimage_to_botimage_copy (const IplImage *ipl) 
{
    bot_core_image_t bot = vis_iplimage_to_botimage_view (ipl);
    return bot_core_image_t_copy (&bot);
}

bot_core_image_t
vis_iplimage_to_botimage_view (const IplImage *ipl) 
{
    bot_core_image_t bot;

    bot.utime = 0;
    bot.width = ipl->width;
    bot.height = ipl->height;
    bot.size = 0;
    bot.data = NULL;
    bot.nmetadata = 0;

    int Bpp = 0;
    if (ipl->depth == IPL_DEPTH_8U || ipl->depth == IPL_DEPTH_8S)
        Bpp = 1;
    else if (ipl->depth == IPL_DEPTH_16U || ipl->depth == IPL_DEPTH_16S)
        Bpp = 2;
    else
        printf ("Unsupported iplimage depth\n");

    int nchannel = ipl->nChannels;
    bot.row_stride = nchannel * Bpp * ipl->width;

    if (nchannel == 1) {
        if (ipl->depth == IPL_DEPTH_8U)
            bot.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
        else if (ipl->depth == IPL_DEPTH_16U)
            bot.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16;

        //case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB:
        //    cvInitImageHeader (&img, size, IPL_DEPTH_8U, 1, IPL_ORIGIN_TL, 4);
        //case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB:
        //    cvInitImageHeader (&img, size, IPL_DEPTH_16U, 1, IPL_ORIGIN_TL, 4);
    }
    else if (nchannel == 3) {
        if (ipl->depth == IPL_DEPTH_8U)
            bot.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR;
        else if (ipl->depth == IPL_DEPTH_16U)
            bot.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16;
    }
    else 
        printf ("Unknown channel \n");

    bot.size = ipl->imageSize;
    bot.data = (uint8_t*) ipl->imageData;

    return bot;
}


static int
_vis_pvframe_to_botimage (bot_core_image_t *dest, const tPvFrame *src, int64_t utime, bool copy)
{
    bot_core_image_t *bot = dest;
    const tPvFrame *Frame = src;

    // sane defaults
    memset (bot, 0, sizeof (*bot));
    bot->utime = utime;
    bot->width = Frame->Width;
    bot->height = Frame->Height;
    bot->size = 0;
    bot->data = NULL;
    bot->nmetadata = 0;

    int Bpp = (Frame->BitDepth == 12 ? 2 : 1); // bytes per pixel
    if (Frame->Format == ePvFmtMono8  || Frame->Format == ePvFmtBayer8 ||
        Frame->Format == ePvFmtMono16 || Frame->Format == ePvFmtBayer16)
        bot->row_stride = Bpp * Frame->Width;
    else
        bot->row_stride = 3 * Bpp * Frame->Width; // rgb, yuv


    // translate tPvFrame's Format to bot's pixelformat
    switch (Frame->Format) {
        // mono
    case ePvFmtMono8:   
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
        break;
    case ePvFmtMono16:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16;
        break;

        // bayer8
    case ePvFmtBayer8:  
        switch (Frame->BayerPattern) {
        case ePvBayerRGGB: 
            bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB;
            break;
        case ePvBayerGBRG: 
            bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG;
            break;
        case ePvBayerGRBG: 
            bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG;
            break;
        case ePvBayerBGGR: 
            bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR;
            break;
        default: 
            ERROR ("unrecognized BayerPattern");
            return -1;
        }
        break;

        // bayer16
    case ePvFmtBayer16:
        switch (Frame->BayerPattern) { 
        case ePvBayerRGGB: 
            bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB;
            break;
        case ePvBayerGBRG:
            bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG;
            break;
        case ePvBayerGRBG:
            bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG;
            break;
        case ePvBayerBGGR:
            bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR;
            break;
        default: 
            ERROR ("unrecognized BayerPattern");
            return -1;            
        }
        break;

        // color
    case ePvFmtRgb24:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
        break;
    case ePvFmtRgb48:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16;
        break;
    case ePvFmtYuv411:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1;
        break;
    case ePvFmtYuv422:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY;
        break;
    case ePvFmtYuv444:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2;
        break;
    case ePvFmtBgr24:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR;
        break;
    case ePvFmtRgba32:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA;
        break;
    case ePvFmtBgra32:
        bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA;
        break;
    default:
        ERROR ("unrecognized pixelformat");
        return -1;        
    }

    bot->size = Frame->ImageBufferSize;
    if (copy) {
        bot->data = malloc (Frame->ImageBufferSize);
        if (!bot->data) {
            ERROR ("malloc() failed");
            bot->size = 0;
            return -1;
        }
        else
            memcpy (bot->data, Frame->ImageBuffer, Frame->ImageBufferSize);
    }
    else
        bot->data = Frame->ImageBuffer;

    return 0;
}

bot_core_image_t *
vis_pvframe_to_botimage_copy (const tPvFrame *src, int64_t src_utime)
{
    bot_core_image_t *dest = malloc (sizeof (*dest));
    if (0 == _vis_pvframe_to_botimage (dest, src, src_utime, 1))
        return dest;
    else
        return NULL;
}

bot_core_image_t
vis_pvframe_to_botimage_view (const tPvFrame *src, int64_t src_utime)
{
    bot_core_image_t dest = {0};
    if (0 == _vis_pvframe_to_botimage (&dest, src, src_utime, 0))
        return dest;
    else {
        bot_core_image_t foo = {0};
        return foo;
    }
}


static int
_vis_botimage_to_pvframe (tPvFrame *dest, const bot_core_image_t *src, bool copy)
{
    tPvFrame *Frame = dest;
    const bot_core_image_t *bot = src;

    // sane defaults
    memset (Frame, 0, sizeof (*Frame));
    Frame->Status = ePvErrSuccess;
    Frame->ImageSize = 0;
    Frame->AncillarySize = 0;
    Frame->Width = bot->width;
    Frame->Height = bot->height;
    Frame->RegionX = 0;
    Frame->RegionY = 0;
    Frame->Format = ePvFmtMono8; // default, overwritten if not Mono8
    Frame->BitDepth = 8;         // default, overwritten if >8-bit
    Frame->BayerPattern = ePvBayerRGGB; // default, overwritten if not RGGB
    Frame->FrameCount = 0;
    Frame->TimestampLo = 0;
    Frame->TimestampHi = 0;


    // translate bot pixel format
    switch (bot->pixelformat) {
        // mono
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY: 
        Frame->Format = ePvFmtMono8;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16:
        Frame->Format = ePvFmtMono16;
	Frame->BitDepth = 12;
        break;

        // bayer8
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB:
        Frame->Format = ePvFmtBayer8;
        Frame->BayerPattern = ePvBayerRGGB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG:
        Frame->Format = ePvFmtBayer8;
        Frame->BayerPattern = ePvBayerGBRG;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG:
        Frame->Format = ePvFmtBayer8;
        Frame->BayerPattern = ePvBayerGRBG;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR:
        Frame->Format = ePvFmtBayer8;
        Frame->BayerPattern = ePvBayerBGGR;
        break;

        // bayer16
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB:
        Frame->Format = ePvFmtBayer16;
        Frame->BayerPattern = ePvBayerRGGB;
	Frame->BitDepth = 12;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG:
        Frame->Format = ePvFmtBayer16;
        Frame->BayerPattern = ePvBayerGBRG;
	Frame->BitDepth = 12;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG:
        Frame->Format = ePvFmtBayer16;
        Frame->BayerPattern = ePvBayerGRBG;
	Frame->BitDepth = 12;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR:
        Frame->Format = ePvFmtBayer16;
        Frame->BayerPattern = ePvBayerBGGR;
	Frame->BitDepth = 12;
        break;

        // color
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        Frame->Format = ePvFmtRgb24;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16:
        Frame->Format = ePvFmtRgb48;
	Frame->BitDepth = 12;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1:
        Frame->Format = ePvFmtYuv411;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY:
        Frame->Format = ePvFmtYuv422;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2:
        Frame->Format = ePvFmtYuv444;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR:
        Frame->Format = ePvFmtBgr24;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA:
        Frame->Format = ePvFmtRgba32;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA:
        Frame->Format = ePvFmtBgra32;
        break;

    default: 
        ERROR ("no equivalent pixelformat in pvFrame [%d]", bot->pixelformat);
        return -1;
    }

    // copy or point at bot's image buffer?
    Frame->ImageSize = Frame->ImageBufferSize = bot->size;
    if (copy) {
        Frame->ImageBuffer = malloc (bot->size);
        if (!Frame->ImageBuffer) {
            ERROR ("malloc() failed");
            Frame->ImageBufferSize = 0;
            return -1;
        }
        else
            memcpy (Frame->ImageBuffer, bot->data, bot->size);
    }
    else
        Frame->ImageBuffer = bot->data;

    return 0;
}


tPvFrame *
vis_botimage_to_pvframe_copy (const bot_core_image_t *src)
{
    tPvFrame *dest = malloc (sizeof (*dest));
    if (0 == _vis_botimage_to_pvframe (dest, src, 1))
        return dest;
    else
        return NULL;
}

tPvFrame
vis_botimage_to_pvframe_view (const bot_core_image_t *src)
{
    tPvFrame dest = {0};
    if (0 == _vis_botimage_to_pvframe (&dest, src, 0))
        return dest;
    else {
        tPvFrame foo = {0};
        return foo;
    }
}


int
vis_botimage_write_tiff (const bot_core_image_t *bot, const char *filename, const char *channel, 
                         const char *description, uint32_t compression)
{
    // TIFF pixel properties
    int32_t bpp=8;  // bits per pixel
    int32_t cpp=1;  // channels per pixel
    int32_t photometric=PHOTOMETRIC_MINISBLACK, fillorder=FILLORDER_MSB2LSB;
    const char *unsupported_format;
    switch (bot->pixelformat) { // keep in sync with bot_core_image_t.lcm
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY:
        unsupported_format = "UYVY";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUYV:
        unsupported_format = "YUYV";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1:
        unsupported_format = "IYU1";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2:
        unsupported_format = "IYU2";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV420:
        unsupported_format = "YUV420";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV411P:
        unsupported_format = "YUV411P";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_I420:
        unsupported_format = "I420";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_NV12:
        unsupported_format = "NV12";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
        bpp=8, cpp=1, photometric=PHOTOMETRIC_MINISBLACK;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        bpp=8, cpp=3, photometric=PHOTOMETRIC_RGB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR:
        unsupported_format = "BGR";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA:
        unsupported_format = "RGBA";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA:
        unsupported_format = "BGRA";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB:
        bpp=8, cpp=1, photometric=PHOTOMETRIC_MINISBLACK;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB:
        bpp=16, cpp=1, photometric=PHOTOMETRIC_MINISBLACK, fillorder=FILLORDER_MSB2LSB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB:
        bpp=16, cpp=1, photometric=PHOTOMETRIC_MINISBLACK, fillorder=FILLORDER_LSB2MSB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
        unsupported_format = "MJPEG";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16:
        bpp=16, cpp=1, photometric=PHOTOMETRIC_MINISBLACK, fillorder=FILLORDER_MSB2LSB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16:
        bpp=16, cpp=1, photometric=PHOTOMETRIC_MINISBLACK, fillorder=FILLORDER_LSB2MSB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_RGB16:
        bpp=16, cpp=3, photometric=PHOTOMETRIC_RGB, fillorder=FILLORDER_MSB2LSB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16:
        bpp=16, cpp=3, photometric=PHOTOMETRIC_RGB, fillorder=FILLORDER_LSB2MSB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_GRAY16:
        unsupported_format = "BE_SIGNED_GRAY16";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_RGB16:
        unsupported_format = "BE_SIGNED_RGB16";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32:
        unsupported_format = "FLOAT_GRAY32";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_INVALID:
        unsupported_format = "INVALID";
        goto unsupported_pixelformat;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_ANY:
        unsupported_format = "ANY";
        goto unsupported_pixelformat;
        break;
    default:
        unsupported_format = "UNRECOGNIZED_PIXELFORMAT";
        goto unsupported_pixelformat;
    }

    // open TIFF file
    TIFF *tif = TIFFOpen (filename, "w");
    if (!tif) {
        ERROR ("TIFFOpen() failed");
        return -1;
    }

    // TIFF generic attributes
    TIFFSetField (tif, TIFFTAG_IMAGEWIDTH, bot->width);
    TIFFSetField (tif, TIFFTAG_IMAGELENGTH, bot->height);
    TIFFSetField (tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField (tif, TIFFTAG_ROWSPERSTRIP, 1);
    TIFFSetField (tif, TIFFTAG_FILLORDER, fillorder);
    TIFFSetField (tif, TIFFTAG_PHOTOMETRIC, photometric);
    TIFFSetField (tif, TIFFTAG_SAMPLESPERPIXEL, cpp);
    TIFFSetField (tif, TIFFTAG_BITSPERSAMPLE, bpp);

    // TIFF compression attributes
    uint32_t type = compression & 0xffff0000;
    uint32_t quality = compression & 0x0000ffff;
    switch (type) {
    case VIS_BOTIMAGE_TIFF_COMPRESSION_NONE:
        TIFFSetField (tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
        break;
    case VIS_BOTIMAGE_TIFF_COMPRESSION_DEFLATE:
        TIFFSetField (tif, TIFFTAG_COMPRESSION, COMPRESSION_ADOBE_DEFLATE);
        TIFFSetField (tif, TIFFTAG_PREDICTOR, PREDICTOR_HORIZONTAL);
        break;
    case VIS_BOTIMAGE_TIFF_COMPRESSION_LZW:
        TIFFSetField (tif, TIFFTAG_COMPRESSION, COMPRESSION_LZW);
        TIFFSetField (tif, TIFFTAG_PREDICTOR, PREDICTOR_HORIZONTAL);
        break;
    case VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG:
        if (bpp != 8) {
            ERROR ("COMPRESSION_JPEG does not support 16-bit imagery");
            goto tiff_error;
        }
        else {
            TIFFSetField (tif, TIFFTAG_COMPRESSION, COMPRESSION_JPEG);
            TIFFSetField (tif, TIFFTAG_JPEGQUALITY, quality);
            TIFFSetField (tif, TIFFTAG_ROWSPERSTRIP, 64); /* must be multiple of 8 for jpeg */
        }
        break;
    default:
        ERROR ("unrecognized compression type");
        goto tiff_error;
    }

    // TIFF bot-specific attributes
    char buf[1024];
    snprintf (buf, sizeof buf, "pixelformat=%d", bot->pixelformat);
    TIFFSetField (tif, TIFFTAG_SOFTWARE, buf);

    timeutil_strftime (buf, sizeof buf, "%Y:%m:%d %H:%M:%S", bot->utime);
    TIFFSetField (tif, TIFFTAG_DATETIME, buf);

    snprintf (buf, sizeof buf, "%"PRId64".%06"PRId64, 
              timestamp_seconds (bot->utime), timestamp_useconds (bot->utime));
    TIFFSetField (tif, TIFFTAG_COPYRIGHT, buf);

    gethostname (buf, sizeof buf);
    TIFFSetField (tif, TIFFTAG_HOSTCOMPUTER, buf);

    if (channel)
        TIFFSetField (tif, TIFFTAG_ARTIST, channel);
    else
        TIFFSetField (tif, TIFFTAG_ARTIST, "");

    if (description)
        TIFFSetField (tif, TIFFTAG_IMAGEDESCRIPTION, description);
    else
        TIFFSetField (tif, TIFFTAG_IMAGEDESCRIPTION, "");

    // write TIFF buffer to disk
    uint8_t *row_ptr = bot->data;
    for (size_t i=0; i<bot->size/bot->row_stride; i++) {
        if (!TIFFWriteScanline (tif, row_ptr, i, 0)) {
            ERROR ("error writing TIFF buffer to disk");
            goto tiff_error;            
        }
        else
            row_ptr += bot->row_stride;
    }
    TIFFClose (tif);

    return 0;

  unsupported_pixelformat:
    ERROR ("unsupported pixelformat: %s", unsupported_format);
    return -1;    

  tiff_error:
    TIFFClose (tif);
    return -1;
}

int
vis_botimage_read_tiff (bot_core_image_t **_bot, char **channel, char **description, const char *filename,
                        bool ignore_bot_tif_tags)
{
    // open TIFF file
    TIFF *tif = TIFFOpen (filename, "r");
    if (!tif) {
        ERROR ("TIFFOpen() failed");
        return -1;
    }
    
    // TIFF generic attributes
    unsigned int width = 0;
    if (!TIFFGetField (tif, TIFFTAG_IMAGEWIDTH, &width) || width < 1) {
        ERROR ("Either undefined or unsupported IMAGEWIDTH, width=%d", width);
        goto on_error;
    }

    unsigned int height = 0;
    if (!TIFFGetField (tif, TIFFTAG_IMAGELENGTH, &height) || height < 1) {
        ERROR ("Either undefined or unsupported IMAGELENGTH, height=%d", height);
        goto on_error;
    }

    unsigned short planarconfig = 0;
    if (!TIFFGetField (tif, TIFFTAG_PLANARCONFIG, &planarconfig) || planarconfig != PLANARCONFIG_CONTIG) {
        ERROR ("Either undefined or unsupported PLANARCONFIG, planarconfig=%d", planarconfig);
        goto on_error;
    }

    unsigned short photometric = 0;
    if (!TIFFGetField (tif, TIFFTAG_PHOTOMETRIC, &photometric)) {
        ERROR ("Either undefined or unsupported PHOTOMETRIC, photometric=%d", photometric);
        goto on_error;
    }    

    unsigned short cpp = 0; // channels per pixel
    if (!TIFFGetField (tif, TIFFTAG_SAMPLESPERPIXEL, &cpp) || !(cpp == 1 || cpp == 3)){
        ERROR ("Either undefined or unsupported SAMPLESPERPIXEL, cpp=%d", cpp);
        goto on_error;
    }
    
    unsigned short bpp = 0; // bits per pixel
    if (!TIFFGetField (tif, TIFFTAG_BITSPERSAMPLE, &bpp) || !(bpp == 8 || bpp == 16)) {
        ERROR ("Either undefined or unsupported BITSPERSAMPLE, bpp=%d", bpp);
        goto on_error;
    }

    // TIFF bot-specific attributes
    unsigned int rowsperstrip = 0;
    unsigned short fillorder = 0;
    unsigned short compression = 0;
    const char *buf = NULL;
    int pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_INVALID;//909199180;
    int64_t utime = -1, utime_sec = -1, utime_usec = -1;

    if (ignore_bot_tif_tags) {
        // we only need pixel format
        uint16 depth, nchannel;
        if (!TIFFGetField (tif, TIFFTAG_BITSPERSAMPLE, &depth)) {
            ERROR ("Either undefined or unsupported IMAGEDEPTH, depth=%d", depth);
            goto on_error;
        }

        if (!TIFFGetField (tif, TIFFTAG_SAMPLESPERPIXEL, &nchannel)) {
            ERROR ("Either undefined or unsupported NCHANNEL, nchannel=%d", nchannel);
            goto on_error;
        }

        // assessing bot image_t format
        if (depth == 8 && nchannel == 1)
            pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
        else if (depth == 8 && nchannel == 3)
            pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
        else if (depth == 16 && nchannel == 1)
            pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16;
        else if (depth == 16 && nchannel == 3)
            pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16;
        else
            ERROR ("Either undefined or unsupported pixel format from tiff tag");

    }
    else { // use bot specific tiff tags
        if (!TIFFGetField (tif, TIFFTAG_ROWSPERSTRIP, &rowsperstrip) || rowsperstrip != 1) {
            ERROR ("Either undefined or unsupported ROWSPERSTRIP, rowsperstrip=%d", rowsperstrip);
            goto on_error;
        }

        if (!TIFFGetField (tif, TIFFTAG_FILLORDER, &fillorder)) {
            ERROR ("Either undefined or unsupported FILLORDER, fillorder=%d", fillorder);
            goto on_error;
        }
        
        if (!TIFFGetField (tif, TIFFTAG_COMPRESSION, &compression) || 
            !(compression == COMPRESSION_NONE || compression == COMPRESSION_LZW ||
              compression == COMPRESSION_JPEG || compression == COMPRESSION_ADOBE_DEFLATE)) {
            ERROR ("Either undefined or unsupported COMPRESSION, compression=%d", compression);
            goto on_error;
        }

        if (!TIFFGetField (tif, TIFFTAG_SOFTWARE, &buf) || 
            (1 != sscanf (buf, "pixelformat=%d", &pixelformat))) {
            ERROR ("Either undefined or unsupported SOFTWARE, buf=%s, pixelformat=%d", buf, pixelformat);
            goto on_error;
        }

        if (!TIFFGetField (tif, TIFFTAG_COPYRIGHT, &buf) || 
            (2 != sscanf (buf, "%"PRId64".%06"PRId64, &utime_sec, &utime_usec))) {
            ERROR ("Either undefined or unsupported COPYRIGHT, utime_sec=%"PRId64", utime_usec=%06"PRId64,
                   utime_sec, utime_usec);
            goto on_error;
        }
        utime = utime_sec*1000000 + utime_usec;

        if (!TIFFGetField (tif, TIFFTAG_ARTIST, &buf)) {
            ERROR ("Either undefined or unsupported ARTIST, buf=%s", buf);
            goto on_error;
        }

        if (!TIFFGetField (tif, TIFFTAG_IMAGEDESCRIPTION, &buf)) {
            ERROR ("Either undefined or unsupported IMAGEDESCRIPTION, buf=%s", buf);
            goto on_error;
        }
    }
    
    if (channel)
        *channel = strdup (buf);

    if (description)
        *description = strdup (buf);

    // read TIFF buffer from disk
    bot_core_image_t *bot = *_bot = g_malloc0 (sizeof (*bot));
    bot->width = width;
    bot->height = height;
    bot->row_stride = bot->width * cpp * (bpp / 8);
    bot->pixelformat = pixelformat;
    bot->size = bot->row_stride * bot->height;
    bot->data = g_malloc0 (bot->size);
    bot->utime = utime;

    uint8_t *row_ptr = bot->data;
    for (size_t i=0; i<bot->height; i++) {
        if (!TIFFReadScanline (tif, row_ptr, i, 0)) {
            ERROR ("error reading TIFF buffer from disk");
            bot_core_image_t_destroy (bot);
            goto on_error;
        }
        else
            row_ptr += bot->row_stride;
    }
    TIFFClose (tif);

    return 0;

  on_error:
    TIFFClose (tif);
    return -1;
}


int
vis_botimage_bayerfilt (bot_core_image_t **dest, const bot_core_image_t *src)
{
    /* is this a bayer pattern bot image?, if so map it to a tPvFrame,
     * do the interpolation there, and then map it back
     */
    switch (src->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB:
        break;
    default:
        *dest = bot_core_image_t_copy (src);
        return 1;
    }

    tPvFrame srcFrame = vis_botimage_to_pvframe_view (src);
    if (srcFrame.ImageBuffer == NULL) {
        ERROR ("unable to map to a tPvFrame");
        return -1;
    }
    const int npixels = srcFrame.Height * srcFrame.Width;

    tPvFrame destFrame = srcFrame;
    if (srcFrame.Format == ePvFmtBayer8) {
        destFrame.Format = ePvFmtRgb24;
        destFrame.ImageBufferSize = 3 * npixels * sizeof (uint8_t);
        destFrame.ImageBuffer = g_malloc (destFrame.ImageBufferSize);
        uint8_t *RGB = destFrame.ImageBuffer;
        PvUtilityColorInterpolate (&srcFrame, RGB, RGB+1, RGB+2, 2, 0);
    }
    else if (srcFrame.Format == ePvFmtBayer16) {
        destFrame.Format = ePvFmtRgb48;
        destFrame.ImageBufferSize = 3 * npixels * sizeof (uint16_t);
        destFrame.ImageBuffer = g_malloc (destFrame.ImageBufferSize);
        uint16_t *RGB = destFrame.ImageBuffer;
        PvUtilityColorInterpolate (&srcFrame, RGB, RGB+1, RGB+2, 2, 0);
    }
    else {
        ERROR ("non-bayer Frame format [%d]", srcFrame.Format);
        return -1;
    }

    *dest = vis_pvframe_to_botimage_copy (&destFrame, src->utime);
    free (destFrame.ImageBuffer);
    if (dest == NULL) {
        ERROR ("unable to map to a bot_core_image_t");
        return -1;
    }

    return 0;
}

int
vis_botimage_16_to_8 (bot_core_image_t **dest, const bot_core_image_t *src)
{
    bool bigendian = 0;
    int npixels=src->width*src->height, nchannels=1, pixelformat=0;
    switch (src->pixelformat) { // keep in sync with bot_core_image_t.lcm
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_BGGR:
        bigendian = 1;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GBRG:
        bigendian = 1;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GRBG:
        bigendian = 1;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB:
        bigendian = 1;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR:
        bigendian = 0;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG:
        bigendian = 0;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG:
        bigendian = 0;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB:
        bigendian = 0;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16:
        bigendian = 1;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16:
        bigendian = 0;
        nchannels = 1;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_RGB16:
        bigendian = 1;
        nchannels = 3;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16:
        bigendian = 0;
        nchannels = 3;
        pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_GRAY16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_RGB16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32:
    default:
        *dest = bot_core_image_t_copy (src);
        return 1;
    }

    *dest = bot_core_image_t_copy (src);
    free ((*dest)->data);
    (*dest)->pixelformat = pixelformat;
    (*dest)->size = src->size / 2;
    (*dest)->row_stride = src->row_stride / 2;
    (*dest)->data = g_malloc ((*dest)->size * sizeof (uint8_t));
    const uint16_t *pixel16 = (uint16_t *) src->data;
    uint8_t *pixel8 = (*dest)->data;
    for (int i=0; i<(nchannels*npixels); i++, pixel16++, pixel8++) {
        if (bigendian)
            *pixel8 = (*pixel16) & 0x00ff;
        else
            *pixel8 = (*pixel16) >> 8;
    }

    return 0;
}


size_t
vis_botimage_filename (char *filename, size_t len, const char *format, int64_t utime, uint32_t framecount)
{
    const char *formatend = format + strlen (format);
    char format2[1024], tmp[1024];

    // handle %f arg if present
    char *Istr = strstr (format, "%f");
    if (Istr != NULL) {
        if (Istr > format) {
            memset (tmp, '\0', sizeof (tmp));
            strncpy (tmp, format, Istr - format);
            sprintf (format2, "%s%d%s", tmp, framecount, Istr+2 < formatend ? Istr+2 : "");
        }
        else {
            sprintf (format2, "%d%s", framecount, format+2);
        }
    }
    else
        strcpy (format2, format);

    return timeutil_strftime (filename, len, format2, utime);
}
