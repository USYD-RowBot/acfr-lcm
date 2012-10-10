#include <iostream>
#include <fstream>

#include <boost/algorithm/string.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

//#include "perls-lcmtypes/perllcm_van_calib_t.h"


#include "perls-vision/ladybug3_util.h"
#include "perls-vision/camera.h"
#include "perls-vision/calib.h"
#include "perls-vision/distortion.h"
#include "perls-vision/opencv_util.h"
#include "perls-vision/botimage.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/error.h"

#include "perls-pvn/pvn_util.h"
#include "pvn_image_data.h"


#define LB3_NUM_CAMS 6
#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)


static inline bool
file_exists(const char *filename)
{
  ifstream ifile(filename);
  return ifile;
}


IplImage *
pvn_botimage_to_iplimage (const bot_core::image_t bot) {
    
    IplImage *img;
    CvSize size = {bot.width, bot.height};

    switch (bot.pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
        img = cvCreateImage (size, IPL_DEPTH_8U, 1);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16:
        img = cvCreateImage (size, IPL_DEPTH_16U, 1);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB:
        img = cvCreateImage (size, IPL_DEPTH_8U, 1);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_BGGR:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GBRG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GRBG:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB:
        img = cvCreateImage (size, IPL_DEPTH_16U, 1);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR:
        img = cvCreateImage (size, IPL_DEPTH_8U, 3);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_RGB16:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16:
        img = cvCreateImage (size, IPL_DEPTH_16U, 3);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA:
        img = cvCreateImage (size, IPL_DEPTH_8U, 4);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUYV:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV420:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV411P:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_I420:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_NV12:
        img = cvCreateImage (size, IPL_DEPTH_8U, 3);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_GRAY16:
        img = cvCreateImage (size, IPL_DEPTH_16S, 1);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_RGB16:
        img = cvCreateImage (size, IPL_DEPTH_16S, 3);
        break;

    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32:
        img = cvCreateImage (size, IPL_DEPTH_32F, 1);
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
    memcpy (img->imageData, &bot.data[0], bot.size);

    return img;
}

bot_core::image_t
pvn_iplimage_to_botimage (const IplImage *ipl) 
{
    bot_core::image_t bot;

    bot.utime = 0;
    bot.width = ipl->width;
    bot.height = ipl->height;
    bot.size = 0;
    bot.data.resize(0);
    bot.nmetadata = 0;

    int Bpp = 0;
    if ((size_t)ipl->depth == IPL_DEPTH_8U || (size_t)ipl->depth == IPL_DEPTH_8S)
        Bpp = 1;
    else if ((size_t)ipl->depth == IPL_DEPTH_16U || (size_t)ipl->depth == IPL_DEPTH_16S)
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
    }
    else if (nchannel == 3) {
        if (ipl->depth == IPL_DEPTH_8U)
            bot.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
        else if (ipl->depth == IPL_DEPTH_16U)
            bot.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16;
    }
    else 
        printf ("Unknown channel \n");

    bot.size = ipl->imageSize;
    bot.data.resize (bot.size);
    memcpy (&bot.data[0], ipl->imageData, bot.size*sizeof(uint8_t));

    return bot;
}


std::vector<PVNCamConfig>
pvn_cam_config_load_lb3 (BotParam *param) {
    
    std::vector<PVNCamConfig> out;
    for (int i=0; i<LB3_NUM_CAMS; i++) {
        PVNCamConfig tmp;
        tmp.LoadLB3Config (param, i);
        out.push_back(tmp);
    }
    return out;
}



PVNCamConfig::PVNCamConfig () {

}

int
PVNCamConfig::LoadLB3Config (BotParam *param, int cam_num) {
    
    bot_param_get_double_array_or_fail (param, "sensors.velodyne.x_vel_lb3", x_vel_h, 6);
    x_vel_h[3] = x_vel_h[3]*DTOR;
    x_vel_h[4] = x_vel_h[4]*DTOR;
    x_vel_h[5] = x_vel_h[5]*DTOR;
    
    bot_param_get_double_array_or_fail (param, "ladybug.x_vs", x_vh, 6);
    x_vh[3] = x_vh[3]*DTOR;
    x_vh[4] = x_vh[4]*DTOR;
    x_vh[5] = x_vh[5]*DTOR;

    char *image_channel_tmp = NULL;
    char *feature_channel_tmp = NULL;
    char *cam_dir_tmp = NULL;
    
    char key[1024] = {0};
    sprintf (key, "cam%d-feature.image_channel", cam_num);
    bot_param_get_str (param, key, &(image_channel_tmp));
    sprintf (key, "cam%d-feature.feature_channel", cam_num);
    bot_param_get_str (param, key, &(feature_channel_tmp));
    sprintf (key, "cam%d-feature.cache_dir", cam_num);
    bot_param_get_str (param, key, &(cam_dir_tmp));
    sprintf (key, "ladybug.cam%d.x_hs", cam_num);
    bot_param_get_double_array_or_fail (param, key, x_hs, 6);
    x_hs[3] = x_hs[3]*DTOR;
    x_hs[4] = x_hs[4]*DTOR;
    x_hs[5] = x_hs[5]*DTOR;
    
    // read K matrix
    double fc[2] = {0};
    sprintf (key, "ladybug.cam%d.cam_calib.fc", cam_num);
    bot_param_get_double_array_or_fail (param, key, fc, 2);
    double cc[2] = {0};
    sprintf (key, "ladybug.cam%d.cam_calib.cc", cam_num);
    bot_param_get_double_array_or_fail (param, key, cc, 2);
    K << fc[0], 0,      cc[0],
         0,     fc[1],  cc[1],
         0,     0,      1;
    
    // read image size
    sprintf (key, "ladybug.cam%d.cam_calib.nx", cam_num);
    height = bot_param_get_double_or_fail (param, key);
    sprintf (key, "ladybug.cam%d.cam_calib.ny", cam_num);
    width = bot_param_get_double_or_fail (param, key);
    
    if (image_channel_tmp == NULL || feature_channel_tmp == NULL || cam_dir_tmp == NULL) {
        ERROR ("image_channel, feature_channel and/or cache_dir not found for camera");
        return -1;
    }
    
    if (image_channel_tmp  != NULL) {
        std::string tmp1 (image_channel_tmp);
        image_channel = tmp1;
        free (image_channel_tmp);
    }
    if (feature_channel_tmp  != NULL){
        std::string tmp2 (feature_channel_tmp);
        feature_channel = tmp2;
        free (feature_channel_tmp );
    }
    if (cam_dir_tmp != NULL){
        std::string tmp3 (cam_dir_tmp);
        cam_dir = tmp3;
        free (cam_dir_tmp );
    }
    
    sprintf (key, "ladybug.cam%d.cam_calib", cam_num);
    perllcm_van_calib_t calib = vis_calib_load_config (param, key);

    // idealy would add this functionality to distortion.c/h and calib.c/h but
    // would need change of lcmdef to includes pointers which breaks current
    // rtvan implementation, also changes def of feature collection, and bathy collection
    if (calib.kc_model == PERLLCM_VAN_CALIB_T_KC_MODEL_FULL_MAP)  {
    
        vis_cvu_map_t *map = (vis_cvu_map_t *)malloc (sizeof (*map)); // memory leak
        map->mapu = cvCreateMat (calib.height, calib.width, CV_32FC1);
        map->mapv = cvCreateMat (calib.height, calib.width, CV_32FC1);

        snprintf (key, sizeof key, "ladybug.cam%d.cam_calib.undist_map_path", cam_num);
        char *undist_map_path = botu_param_get_str_or_default (param, key, "");
        //snprintf (key, sizeof mykey, "%s.dist_map_path", cfgkey);
        //char *dist_map_path = botu_param_get_str_or_default (param, key, "");
        
        perllcm_vis_cvu_map_t *lcm_map;
        LCMU_FREAD (undist_map_path, &lcm_map, perllcm_vis_cvu_map_t);
            
        for (int i=0; i<lcm_map->height; i++) {
            for (int j=0; j<lcm_map->width; j++) {
                cvmSet (map->mapu, i, j, lcm_map->mapu[i*lcm_map->width + j]);
                cvmSet (map->mapv, i, j, lcm_map->mapv[i*lcm_map->width + j]);    
            }
        }
        
        vis_calib_const_view_cv_t cv = vis_calib_const_view_cv (&calib);
        IplImage *Imask = cvCreateImage (cv.imageSize, IPL_DEPTH_8U, 1);
        IplImage *I = cvCreateImage (cv.imageSize, IPL_DEPTH_8U, 1);
    
        cvSet (I, cvScalarAll (255), NULL);
        cvRemap (I, Imask, map->mapu, map->mapv, 
                 CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));
        
        cvReleaseImage (&I);
        
        udist_map = map;
        mask = Imask;
        
        
        
    } else {    
        udist_map = vis_undistort_map (&calib);
        mask = vis_undistort_mask (&calib);
    }
    
    return 0;
}


int
PVNCamConfig::LoadMonoConfig (BotParam *param, std::string cam_key, std::string feature_key) {
    
    char key[1024] = {0};
    sprintf (key, "%s.x_vs", cam_key.c_str());

    bot_param_get_double_array_or_fail (param, key, x_vh, 6);
    x_vh[3] = x_vh[3]*DTOR;
    x_vh[4] = x_vh[4]*DTOR;
    x_vh[5] = x_vh[5]*DTOR; 

    char *image_channel_tmp = NULL;
    char *feature_channel_tmp = NULL;
    char *cam_dir_tmp = NULL;
    sprintf (key, "%s.image_channel", feature_key.c_str());
    bot_param_get_str (param, key, &(image_channel_tmp));
    sprintf (key, "%s.feature_channel", feature_key.c_str());
    bot_param_get_str (param, key, &(feature_channel_tmp));
    sprintf (key, "%s.cache_dir", feature_key.c_str());
    bot_param_get_str (param, key, &(cam_dir_tmp));
    // x_hs is identity for non-lb3 cameras
    memset(x_hs, 0, 6*sizeof (double));
    
    // read K matrix
    double fc[2] = {0};
    sprintf (key, "%s.cam_calib.fc", cam_key.c_str());
    bot_param_get_double_array_or_fail (param, key, fc, 2);
    double cc[2] = {0};
    sprintf (key, "%s.cam_calib.cc", cam_key.c_str());
    bot_param_get_double_array_or_fail (param, key, cc, 2);
    K << fc[0], 0,      cc[0],
         0,     fc[1],  cc[1],
         0,     0,      1;
    
    // read image size
    sprintf (key, "%s.cam_calib.nx", cam_key.c_str());
    height = bot_param_get_double_or_fail (param, key);
    sprintf (key, "%s.cam_calib.ny", cam_key.c_str());
    width = bot_param_get_double_or_fail (param, key);
    
    if (image_channel_tmp == NULL || feature_channel_tmp == NULL || cam_dir_tmp == NULL) {
        ERROR ("image_channel, feature_channel and/or cache_dir not found for camera");
        return -1;
    }
    
    if (image_channel_tmp  != NULL) {
        std::string tmp1 (image_channel_tmp);
        image_channel = tmp1;
        free (image_channel_tmp);
    }
    if (feature_channel_tmp  != NULL){
        std::string tmp2 (feature_channel_tmp);
        feature_channel = tmp2;
        free (feature_channel_tmp );
    }
    if (cam_dir_tmp != NULL){
        std::string tmp3 (cam_dir_tmp);
        cam_dir = tmp3;
        free (cam_dir_tmp );
    }
    
    return 0;
}

PVNCamConfig::~PVNCamConfig () {

}




bool
PVNImageData::data_ready (void) {
    for (int i=0; i<num_images; i++) {
        if (!received[i])
            return false;
    }
    return true;
}

void
PVNImageData::feature_cb (const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                          const perllcm::van_feature_collection_t *msg) {
    int cam_index = -1;
    for (int i=0; i<num_images; i++) {
        if (camconf[i].feature_channel.compare(chan) == 0) {
            cam_index = i;
            break;
        }
    }
    if (-1 == cam_index) {
        ERROR ("No camera found for this feature collection!");
        return;
    }
    
    feature_collections[cam_index] = *msg;
    received[cam_index] = true;
}

int
PVNImageData::load_cached (lcm::LCM *lcm) {
    
    // used if multiple images are collected at same utime () otherwise wont cache right.
    int64_t offset_utime = utime + usec_offset; 
    
    // make check if all the files exist before trying to load them
    bool cache_files_exist = true;
    for (int icam = 0; icam < num_images; icam++) {
        char feat_file[PATH_MAX]={0};
        sprintf (feat_file, "%s/%ld.feat", camconf[icam].cam_dir.c_str(), offset_utime);
        char ppm_file[PATH_MAX]={0};
        sprintf (ppm_file, "%s/%ld.ppm", camconf[icam].cam_dir.c_str(), offset_utime);
        
        if (!file_exists (feat_file) || !file_exists (ppm_file))
            cache_files_exist &= false; 
    }
    
    if(!cache_files_exist)
        return -1;
    
    // load and send
    for (int icam = 0; icam < num_images; icam++) {       
        
        char feat_file[PATH_MAX]={0};
        sprintf (feat_file, "%s/%ld.feat", camconf[icam].cam_dir.c_str(), offset_utime);
        
        perllcm::van_feature_collection_t fc;
        int32_t ret = pvnu_lcm_fread<perllcm::van_feature_collection_t> (feat_file, &fc);
        if (ret < 0) {
            ERROR ("couldn't read %s from disk!", feat_file);
            return -1;
        }
        feature_collections[icam] = fc;
        // publish features for viewer
        lcm->publish (camconf[icam].feature_channel.c_str(), &fc);
        
        // load image and publish it (for viewer)
        IplImage *ud_img = vis_cvu_iplimg_load (camconf[icam].cam_dir.c_str(), offset_utime);
        if (ud_img) {
            bot_core_image_t *botimg = vis_iplimage_to_botimage_copy (ud_img);
            botimg->utime = offset_utime;
            char chan[32];
            sprintf (chan, "%s_UNDIST", camconf[icam].image_channel.c_str());
            bot_core_image_t_publish (lcm->getUnderlyingLCM(), chan, botimg);
            bot_core_image_t_destroy (botimg);
            cvReleaseImage (&ud_img);
        }
    }    
    
    return 0;   

}

int
PVNImageData::load_std_fmt (std::vector<bot_core::image_t> &images) {
    
    // used if multiple images are collected at same utime () otherwise wont cache right.
    int64_t offset_utime = utime + usec_offset; 
    
    images.resize(num_images);
    for (int icam=0; icam<num_images; icam++) {
        
        char filename[PATH_MAX]={0};
        sprintf (filename, "%s/%ld.%s", image_dirs[icam].c_str(), utime, file_extension.c_str());
    
        IplImage *ipl = cvLoadImage (filename, CV_LOAD_IMAGE_UNCHANGED);
        if (ipl) {
            
            bot_core::image_t botimg;
            botimg.utime = offset_utime;
            botimg.width = ipl->width;
            botimg.height = ipl->height;
            botimg.size = ipl->imageSize;
            botimg.nmetadata = 0;
            
            int Bpp = 0;
            if ((unsigned)ipl->depth == IPL_DEPTH_8U || (unsigned)ipl->depth == IPL_DEPTH_8S)
                Bpp = 1;
            else if ((unsigned)ipl->depth == IPL_DEPTH_16U || (unsigned)ipl->depth == IPL_DEPTH_16S)
                Bpp = 2;
            else
                printf ("Unsupported iplimage depth\n");
            
            int nchannel = ipl->nChannels;
            botimg.row_stride = nchannel * Bpp * ipl->width;
            
            if (nchannel == 1) {
                if (ipl->depth == IPL_DEPTH_8U){
                    botimg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
                } else if (ipl->depth == IPL_DEPTH_16U) {
                    botimg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16;
                }
            } else if (nchannel == 3) {
                if (ipl->depth == IPL_DEPTH_8U) {
                    //botimg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR;
                    cvCvtColor (ipl, ipl, CV_BGR2RGB);
                    botimg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
                } else if (ipl->depth == IPL_DEPTH_16U) {
                    botimg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16;
                }
            }
            else 
            printf ("Unknown channel \n");
            
            
            
            botimg.data.resize(botimg.size);
            memcpy (&botimg.data[0], (uint8_t*)ipl->imageData, botimg.size);

            images[icam] = botimg;
            cvReleaseImage (&ipl);
            
        } else {
            ERROR("Failed to open: %s", filename);
            return -1;
        }
    }
    
    return 0;   
}

int
PVNImageData::load_pgr (std::vector<bot_core::image_t> &images, bool color) {
    
    // used if multiple images are collected at same utime () otherwise wont cache right.
    int64_t offset_utime = utime + usec_offset; 
    
    images.resize(num_images);
    
    if (LB3_NUM_CAMS != num_images) {
        ERROR ("Number of images incorrect for LB3 %d != %d", num_images, LB3_NUM_CAMS);
        return -1;   
    }
    
           
    // load image from file
    char pgr_file[PATH_MAX]={0};
    sprintf (pgr_file, "%s/%ld.pgr", image_dirs[0].c_str(), utime);
    
    FILE *fp = fopen (pgr_file, "rb");
    if (fp == NULL) {
        ERROR ("Error reading the pgr file: %s", pgr_file);
        return -1;
    } 
    
    //read header
    LB3StreamHeaderInfo_t header;
    lb3_read_stream_header (fp, &header);
    
    //read calibration data
    lb3_read_calib_data (fp, &header, image_dirs[0].c_str());
    
    // make sure there is only one image
    if (header.numImages != 1) {
        ERROR ("Expecting only 1 image in pgr file: %s", pgr_file);
        return -1;
    }
    
    // for LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_JPEG
    int width = 808;
    int height = 616;
    int decompressed_width = 1616;
    int decompressed_height = 1232;

    //read jpeg header
    LB3JpegHeaderInfo_t jpegHeader;
    lb3_read_jpeg_header (fp, &jpegHeader);    

    //read jpeg blocks
    unsigned char *decompressed_bayer[LB3_NUM_CAMS][4];
    unsigned char *bayer[LB3_NUM_CAMS][4];
    for (int i=0; i<LB3_NUM_CAMS; i++) {
        for (int k = 0; k < 4; k++) {
            bayer[i][k] = (unsigned char *)malloc (jpegHeader.jpegDataSize[i][k]);
            decompressed_bayer[i][k] = (unsigned char *)malloc (width*height); 
        }
    }

    for (int i=0; i<LB3_NUM_CAMS; i++) {
        for (int k = 0; k < 4; k++) {
            //Read compressed bayer pattern
            fseek (fp, jpegHeader.jpegDataOffset[i][k], SEEK_SET);
            size_t ret = fread (bayer[i][k], jpegHeader.jpegDataSize[i][k], 1, fp);
            if (ret == 0)
                ERROR ("Read Error");
            //decompress bayer pattern
            lb3_decompress_jpeg (bayer[i][k], jpegHeader.jpegDataSize[i][k], decompressed_bayer[i][k], width, height);
        }
    }
    
    //perform bayer conversion to get the color image
    for (int cam=0; cam<LB3_NUM_CAMS; cam++) {
    
        //create a bayer pattern buffer from these blocks
        unsigned char *bayer_buffer = (unsigned char *)malloc (decompressed_width*decompressed_height);
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                //Set the R value
                bayer_buffer[2*i*decompressed_width + 2*j]             = decompressed_bayer[cam][3][i*width + j]; 
                //Set the G value
                bayer_buffer[2*i*decompressed_width + (2*j + 1)]       = decompressed_bayer[cam][2][i*width + j];
                //set the G value
                bayer_buffer[(2*i + 1)*decompressed_width + 2*j]       = decompressed_bayer[cam][1][i*width + j]; 
                //set the B value 
                bayer_buffer[(2*i + 1)*decompressed_width + (2*j + 1)] = decompressed_bayer[cam][0][i*width + j];
            }
        }
        
        //Now do bayer conversion on the bayer buffer
        IplImage *src = cvCreateImage (cvSize (decompressed_width, decompressed_height), IPL_DEPTH_8U, 1);
        src->imageData = (char *) bayer_buffer;        
        IplImage *dst = NULL; 
        
        if (color) {
            dst = cvCreateImage (cvSize (decompressed_width, decompressed_height), IPL_DEPTH_8U, 3);    
            cvCvtColor (src, dst, CV_BayerBG2RGB);
        } else {
            dst = cvCreateImage (cvSize (decompressed_width, decompressed_height), IPL_DEPTH_8U, 1);
            IplImage *tmp = cvCreateImage (cvSize (decompressed_width, decompressed_height), IPL_DEPTH_8U, 3);    
            cvCvtColor (src, tmp, CV_BayerBG2RGB);
            cvCvtColor (tmp, dst, CV_RGB2GRAY);
            cvReleaseImage (&tmp);
        }
        char * buffer = (char *) dst->imageData;
        
        // pack image_t structure
        bot_core::image_t img;
        img.utime = offset_utime;
        img.width = decompressed_width;
        img.height = decompressed_height;
        if (color) {
            img.row_stride = decompressed_width*3;
            img.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
            img.size = decompressed_width*decompressed_height*3;
        } else {
            img.row_stride = decompressed_width;
            img.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY;
            img.size = decompressed_width*decompressed_height;
        }
        img.data.resize(img.size);
        memcpy (&img.data[0], buffer, img.size);
        img.nmetadata = 0;
        images[cam] = img;
        
        free (bayer_buffer);
        cvReleaseImage (&src);
        cvReleaseImage (&dst);
    }
    
    //Free memory
    for (int i = 0; i < LB3_NUM_CAMS; i++) {
        for (int k = 0; k < 4; k++) {
            free (bayer[i][k]);
            free (decompressed_bayer[i][k]);
        }
    }
    
    return 0;
}

vector<bot_core::image_t>
PVNImageData::undistort_images (vector<bot_core::image_t> images) {
    
    vector<bot_core::image_t> images_out;
    images_out.resize(num_images);
    for (int i=0; i<num_images; i++) {
    
       
        if (camconf[i].udist_map != NULL) {
            
            IplImage *img = pvn_botimage_to_iplimage (images[i]);
            IplImage *img_warp = cvCreateImage (cvGetSize (img), img->depth, img->nChannels);
            vis_cvu_warp_image (img, img_warp, camconf[i].udist_map);
            images_out[i] = pvn_iplimage_to_botimage (img_warp);
            images_out[i].utime = images[i].utime;
            cvReleaseImage (&img);
            cvReleaseImage (&img_warp);
            
        } else {
            images_out[i] = images[i];
        }        
        
    }
    
    return images_out;
}

vector<bot_core::image_t>
PVNImageData::resize_images (std::vector<bot_core::image_t> images, int resize[2]) {
    
    vector<bot_core::image_t> images_out;
    images_out.resize(num_images);
    for (int i=0; i<num_images; i++) {
    
       
        if (camconf[i].udist_map != NULL) {
            
            IplImage *img = pvn_botimage_to_iplimage (images[i]);
            IplImage *img_resize = cvCreateImage (cvSize(resize[0],resize[1]), img->depth, img->nChannels);
            cvResize(img, img_resize, CV_INTER_LINEAR);
            images_out[i] = pvn_iplimage_to_botimage (img_resize);
            images_out[i].utime = images[i].utime;
            cvReleaseImage (&img);
            cvReleaseImage (&img_resize);
            
        } else {
            images_out[i] = images[i];
        }        
        
    }
    
    return images_out;    
        
}


PVNImageData::PVNImageData (std::vector<PVNCamConfig> _camconf, std::vector<std::string> _image_dirs,
                            int64_t _utime, std::string _file_extension, int _usec_offset) {
    
    num_images = _camconf.size();
    image_dirs = _image_dirs;
    utime = _utime;
    usec_offset = _usec_offset;
    file_extension = _file_extension;
    camconf = _camconf;
    std::vector<bool> tmp(num_images, false);
    received = tmp;
    feature_collections.resize(num_images);
    boost::to_lower(file_extension);
}

PVNImageData::~PVNImageData (void) {
    
}

int    
PVNImageData::load_features (lcm::LCM *lcm, int resize[2]) {
   
    // try to short circuit, check if the files exists in the features directory, if so use them.
    if (0 == load_cached (lcm)) {
        return 0;
    }
    
    vector<bot_core::image_t> images_tmp;
    if (0 == file_extension.compare("pgr")) {
        if (load_pgr (images_tmp)) 
            return -1;
    } else if (0 == file_extension.compare("tiff") ||
               0 == file_extension.compare("tif")  ||
               0 == file_extension.compare("jpg")  ||
               0 == file_extension.compare("jpeg") ||
               0 == file_extension.compare("ppm")  ||
               0 == file_extension.compare("png")) {
        if (load_std_fmt (images_tmp))
            return -1;
    } else {
        
    }
    
    if (resize != NULL) {
        images_tmp = resize_images (images_tmp, resize);
    }

    int max_try = 3;
    int try_cnt = 0;
    while (try_cnt < max_try) {
    
        // subscribe to callbacks
        lcm::Subscription *feat_subs[num_images];
    
        for (int i=0; i<num_images; i++) {
            feat_subs[i] = lcm->subscribe(camconf[i].feature_channel, &PVNImageData::feature_cb, this);
        }
    
        // send and wait for reply
        for (int i=0; i<num_images; i++) {
            lcm->publish (camconf[i].image_channel, &(images_tmp[i]));      
        }
        
        // wait for response
        struct timeval to;
        to.tv_sec = 10;
        to.tv_usec = 0;
        while (!data_ready () && (to.tv_sec > 0 || to.tv_usec > 0)) {
            pvnu_lcm_handle_timeout (lcm, &to);
        }
    
        // unsubscribe
        for (int i=0; i<num_images; i++) {
            if (feat_subs[i] != NULL)
                lcm->unsubscribe (feat_subs[i]);      
        }
    
        if (!data_ready ()) {
            try_cnt++;
        } else {
            return 0;
        }
    
    }

    ERROR ("Failed to load feature data!");
    return -1;
}

int    
PVNImageData::load_orig_img (lcm::LCM *lcm) {   
    
    if (0 == file_extension.compare("pgr")) {
        vector<bot_core::image_t> images_tmp;
        if (load_pgr (images_tmp, 1)) 
            return -1;
        
        images = undistort_images (images_tmp);
        
    } else if (0 == file_extension.compare("tiff") ||
               0 == file_extension.compare("tif")  ||
               0 == file_extension.compare("jpg")  ||
               0 == file_extension.compare("jpeg") ||
               0 == file_extension.compare("ppm")  ||
               0 == file_extension.compare("png")) {
        if (load_std_fmt (images))
            return -1;
    }
    
    return 0;

    
}


perllcm::pvn_eview_map_exemplar_t
PVNImageData::to_pvn_eview_map_exemplar_t (perllcm::pose3d_t x_n_e) {
    
    perllcm::pvn_eview_map_exemplar_t mne = {0};
    
    mne.utime = utime + usec_offset;
    mne.x_n_e = x_n_e;
    
    mne.max_xyz[0] = -1e6; mne.max_xyz[1] = -1e6; mne.max_xyz[2] = -1e6;
    mne.min_xyz[0] = 1e6; mne.min_xyz[1] = 1e6; mne.min_xyz[2] = 1e6;
    mne.npts = 0;
    mne.npts_uv = 0;
    mne.keylen = feature_collections[0].f[0].keylen;
    for (int i=0; i<num_images;  i++) {
        
        if (feature_collections[i].ntypes > 1) {
            ERROR ("Ignoring more that 1 feature type!");
        }
        perllcm::van_feature_t *f = &feature_collections[i].f[0];  
        if (f->attrtype != perllcm::van_feature_t::ATTRTYPE_SIFTGPU) {
            ERROR ("Only expecting SIFTGPU features.");
        }
    
        for (int j=0; j<f->npts; j++) {
            
            // todo add in 3D data
            //double x_v_ci[6] = {0}; // vehicle to camera i
            //ssc_head2tail (x_v_ci, NULL, camconf[i]->x_vh, camconf[i]->x_hs);
            //double x_v_f[6] = {0};
            //ssc_head2tail (x_v_f, NULL, x_v_ci, x_ci_f);
            //mne.x.push_back(x_v_f[0]);
            //mne.y.push_back(x_v_f[1]);
            //mne.z.push_back(x_v_f[2]);
            //
            //if (x_v_f[0] > mne.max_xyz[0]) mne.max_xyz[0] = x_v_f[0];
            //if (x_v_f[1] > mne.max_xyz[1]) mne.max_xyz[1] = x_v_f[1];
            //if (x_v_f[2] > mne.max_xyz[2]) mne.max_xyz[2] = x_v_f[2];
            //if (x_v_f[0] < mne.min_xyz[0]) mne.min_xyz[0] = x_v_f[0];
            //if (x_v_f[1] < mne.min_xyz[1]) mne.min_xyz[1] = x_v_f[1];
            //if (x_v_f[2] < mne.min_xyz[2]) mne.min_xyz[2] = x_v_f[2];
            mne.x.push_back(0.0);
            mne.y.push_back(0.0);
            mne.z.push_back(0.0);
        }
        mne.npts += f->npts;
        mne.npts_uv += f->npts;
        
        mne.keys.insert (mne.keys.end(), f->keys.begin(), f->keys.end());
        mne.u.insert (mne.u.end(), f->u.begin(), f->u.end());
        mne.v.insert (mne.v.end(), f->v.begin(), f->v.end());
        vector <int> tmp (f->npts, 1);
        mne.vocab_id.insert(mne.vocab_id.end(), tmp.begin(), tmp.end());
        
        // load conditions externally
        perllcm::pvn_conditions_t conditions = {0};
        mne.conditions = conditions;
        
        // set initial used time to 0
        // mne.utime_last_match = 0;
        // use initilization time as used time
        mne.utime_last_match = mne.utime;
        
    }   
    
    
    return mne;
}

cv::Mat
PVNImageData::to_opencv_mat (void) {
    
    // count the number of points
    int npts = 0;
    int keylen = 0;
    for (int i=0; i<num_images;  i++) {
        
        if (feature_collections[i].ntypes > 1) {
            ERROR ("Ignoring more that 1 feature type!");
        }
        perllcm::van_feature_t *f = &feature_collections[i].f[0];  
        if (f->attrtype != perllcm::van_feature_t::ATTRTYPE_SIFTGPU) {
            ERROR ("Only expecting SIFTGPU features.");
        }
        
        npts += f->npts;
        keylen = f->keylen;
            
    }
    
    cv::Mat desc (npts, keylen, CV_32F);
    
    // fill matrix
    int ii = 0;
    for (int i=0; i<num_images; i++) {
        
        perllcm::van_feature_t *f = &feature_collections[i].f[0];  
        
        for (int j=0; j<f->npts; j++) {
            for (int k=0; k<keylen; k++) {
                
                desc.at<float> (ii, k) = f->keys[j][k];
                
            }
            ii++;
        }
    }
    
    return desc;
}


int
PVNImageData::get_num_feats (void) {
    
    int npts = 0;
    for (int i=0; i<num_images;  i++) {
        
        if (feature_collections[i].ntypes > 1) {
            ERROR ("Ignoring more that 1 feature type!");
        }
        perllcm::van_feature_t *f = &feature_collections[i].f[0];  
        if (f->attrtype != perllcm::van_feature_t::ATTRTYPE_SIFTGPU) {
            ERROR ("Only expecting SIFTGPU features.");
        }
        
        npts += f->npts;
    }
    return npts;
}