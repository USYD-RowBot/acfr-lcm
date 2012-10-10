#ifndef __LADYBUG3_UTIL_H__
#define __LADYBUG3_UTIL_H__


#ifdef __cplusplus
extern "C" {
#endif

/**
 * These enums are for saving the data in a format readable by ladybug software
 * These are copied from "ladybug.h" in PGR software package.
 */
typedef enum LadybugDataFormat {
    // This format involves interleaving every image from each of the 6  
    // image sensors on a pixel by pixel basis.  Each pixel is in its raw  
    // 8bpp format. This is the only 6 image format supported by the Ladybug. 
    // This format is not supported by Ladybug2. 
    LADYBUG_DATAFORMAT_INTERLEAVED, 
  
    // This format produces a single image buffer that has each sensor's image  
    // one after the other. Again, each pixel is in its raw 8bpp format.  This  
    // format is only supported by the Ladybug2.  This format is not supported 
    // by the original Ladybug. 
    LADYBUG_DATAFORMAT_SEQUENTIAL, 
  
    // This format is similar to the LADYBUG_DATAFORMAT_SEQUENTIAL  
    // except that the entire buffer is JPEG compressed.  This format is  
    // intended for use with cameras that have black and white sensors. It is  
    // not supported by the original Ladybug. 
    LADYBUG_DATAFORMAT_SEQUENTIAL_JPEG, 
  
    // In addition to separating the images sequentially, this format separates  
    // each individual image into its 4 individual Bayer channels (Green, Red, 
    // Blue and Green - not necessarily in that order). This format is only  
    // supported by Ladybug2. 
    LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL, 
  
    // This format is very similar to  
    // LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL except that the transmitted 
    // buffer is JPEG compressed. This format is only supported by Ladybug2. 
    LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_JPEG, 
  
    // This format is similar to LADYBUG_DATAFORMAT_SEQUENTIAL. The height 
    // of the image is only half of that in LADYBUG_DATAFORMAT_SEQUENTIAL 
    // format. This format is only supported by Ladybug3. 
    LADYBUG_DATAFORMAT_SEQUENTIAL_HALF_HEIGHT, 
  
    // This format is similar to LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_JPEG. 
    // The height of each individual Bayer channel image is only one fourth  
    // of the original Bayer channel image. This format is only supported by  
    // Ladybug3. 
    LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_HALF_HEIGHT_JPEG, 
  
    // The number of possible data formats. 
    LADYBUG_NUM_DATAFORMATS, 
  
    // Hook for "any usable video mode". 
    LADYBUG_DATAFORMAT_ANY, 
     
    // Unused member to force this enumeration to compile to 32 bits. 
    LADYBUG_DATAFORMAT_FORCE_QUADLET = 0x7FFFFFFF, 

} LadybugDataFormat_t; 

typedef enum LadybugResolution { 
     // 128x96 pixels. Not supported. 
     LADYBUG_RESOLUTION_128x96, 
     // 256x192 pixels. Not supported. 
     LADYBUG_RESOLUTION_256x192, 
     // 512x384 pixels. Not supported. 
     LADYBUG_RESOLUTION_512x384, 
     // 640x480 pixels. Not supported. 
     LADYBUG_RESOLUTION_640x480, 
     // 1024x768 pixels. Ladybug2 camera. 
     LADYBUG_RESOLUTION_1024x768, 
     // 1216x1216 pixels. Not supported. 
     LADYBUG_RESOLUTION_1216x1216, 
     // 1616x1216 pixels. Not supported. 
     LADYBUG_RESOLUTION_1616x1216, 
     // 1600x1200 pixels, Not supported. 
     LADYBUG_RESOLUTION_1600x1200, 
     // 1616x1232 pixels. Ladybug3 camera.  
     LADYBUG_RESOLUTION_1616x1232, 
  
     // Number of possible resolutions. 
     LADYBUG_NUM_RESOLUTIONS, 
     // Hook for "any usable resolution". 
     LADYBUG_RESOLUTION_ANY, 
                                            
     // Unused member to force this enumeration to compile to 32 bits. 
     LADYBUG_RESOLUTION_FORCE_QUADLET = 0x7FFFFFFF, 
} LadybugResolution_t; 

typedef enum LadybugBayerPattern {
    LADYBUG_BGGR, 
    LADYBUG_GBRG,
    LADYBUG_GRBG, 
    LADYBUG_RGGB,
    LADYBUG_DEFAULT,
    LADYBUG_STIPPLED_FORCE_QUADLET = 0x7FFFFFFF, 
} LadybugBayerPattern_t;

typedef struct LB3StreamHeaderInfo LB3StreamHeaderInfo_t;
struct LB3StreamHeaderInfo {
    uint32_t versionNum;
    uint32_t framerate;
    uint32_t baseSerialNum;
    uint32_t headSerialNum;
    uint32_t paddingBlock;
    LadybugDataFormat_t dataFormat;
    LadybugResolution_t resolution;
    LadybugBayerPattern_t bayerPattern;
    uint32_t configDataSize;
    uint32_t numImages;
    uint32_t numIndex_M;
    uint32_t interval_K;
    uint32_t streamDataOffset;
    uint32_t gpsDataOffset;
    uint32_t gpsDataSize;
};

typedef struct LB3JpegHeaderInfo LB3JpegHeaderInfo_t;
struct LB3JpegHeaderInfo {
    uint64_t timestamp;
    uint32_t dataSize;
    uint32_t fingerPrint;
    uint32_t versionNumber;
    uint32_t timestamp_sec;
    uint32_t timestamp_musec;
    uint32_t seqId;
    uint32_t refreshRate;
    uint32_t gain[6];
    uint32_t whiteBalance;
    uint32_t bayerGain;
    uint32_t bayerMap;
    uint32_t brightness;
    uint32_t gamma;
    uint32_t headSerialNum;
    uint32_t shutter[6];
    uint32_t gpsOffset;
    uint32_t gpsSize;
    uint32_t jpegDataOffset[6][4];
    uint32_t jpegDataSize[6][4];
};
 
/**
 * The LB3 camera can either be run on full resolution or half resolution
 */ 
typedef enum LB3video_mode {
    LB3_FULL_RES_UNCOMPRESSED, // 1600 x 1200
    LB3_HALF_RES_UNCOMPRESSED,  // 1600 x 600
    LB3_FULL_RES_JPEG, //This is color separated 808x616, 4 images per camera, 
    //1 image per bayer channel
    LB3_HALF_RES_JPEG  //This is color separated 808x308
} LB3video_mode_t;

/**
 * This function decompresses the jpeg buffer
 */
int 
lb3_decompress_jpeg (const unsigned char *src, unsigned long size, unsigned char *dest,
                     const uint32_t width, const uint32_t height);

/**
 * Reverses the byte order to big-endian
 */
uint32_t
lb3_reverse_int (uint32_t i);

/**
 * This function reads the header from the file
 */
int 
lb3_read_stream_header (FILE *fp, LB3StreamHeaderInfo_t *header);

/**
 * Read Calibration data
 */
void
lb3_read_calib_data (FILE *fp, LB3StreamHeaderInfo_t *header, const char *folder);

/**
 * This function reads the jpeg header
 */
void
lb3_read_jpeg_header (FILE *fp, LB3JpegHeaderInfo_t *jpegHeader);

#ifdef __cplusplus
}
#endif

#endif //__LADYBUG3_UTIL_H__
