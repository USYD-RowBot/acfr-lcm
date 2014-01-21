#ifndef __PERLS_VISION_BOTIMAGE_H__
#define __PERLS_VISION_BOTIMAGE_H__

#include <stdbool.h>
#include <stdint.h>
#include <tiffio.h> // required for COMPRESSION_XXX #defines
#include <opencv/cv.h>
#include <prosilica/PvApi.h>

#include "perls-lcmtypes/bot_core_image_t.h"


#ifdef __cplusplus
extern "C" {
#endif

enum vis_bot_colormap {
    VIS_BOT2CVGRAY,
    VIS_BOT2CVBGR,
};
typedef enum vis_bot_colormap vis_bot2cv_color_t;

int
vis_botimage_is_gray (const bot_core_image_t *bot);

int
vis_botimage_is_bayer (const bot_core_image_t *bot);

int
vis_botimage_is_color (const bot_core_image_t *bot);

IplImage *
vis_botimage_to_iplimage_copy (const bot_core_image_t *bot);

IplImage
vis_botimage_to_iplimage_view (const bot_core_image_t *bot);

IplImage *
vis_botimage_to_iplimage_convert (const bot_core_image_t *bot, vis_bot2cv_color_t code);

bot_core_image_t *
vis_iplimage_to_botimage_copy (const IplImage *ipl);

bot_core_image_t
vis_iplimage_to_botimage_view (const IplImage *ipl);


/* The following two functions map between bot_core_image_t and tPvFrame image structs.
 * returns 0 on success, -1 on error
 */
bot_core_image_t *
vis_pvframe_to_botimage_copy (const tPvFrame *src, int64_t src_utime);

bot_core_image_t
vis_pvframe_to_botimage_view (const tPvFrame *src, int64_t src_utime);


tPvFrame *
vis_botimage_to_pvframe_copy (const bot_core_image_t *src);

tPvFrame
vis_botimage_to_pvframe_view (const bot_core_image_t *src);


/* The following two functions write and read bot_core_image_t to/from a tiff file.
 * channel=(optional) LCM channel name, o/w set to NULL
 * returns 0 on success, -1 on error
 * 
 * In the case of BOTU_TIFF_COMPRESSION_JPEG, the user can OR the compression flag
 * with the desired level of jpeg quality, e.g. BOTU_TIFF_COMPRESSION_JPEG | 95,
 * would specify jpeg compression at 95% quality
 */
#define VIS_BOTIMAGE_TIFF_COMPRESSION_NONE     (COMPRESSION_NONE << 16)
#define VIS_BOTIMAGE_TIFF_COMPRESSION_LZW      (COMPRESSION_LZW << 16)
#define VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG     (COMPRESSION_JPEG << 16)
#define VIS_BOTIMAGE_TIFF_COMPRESSION_DEFLATE  (COMPRESSION_DEFLATE << 16)
int
vis_botimage_write_tiff (const bot_core_image_t *bot, const char *filename, const char *channel, 
                         const char *description, uint32_t compression);

int
vis_botimage_read_tiff (bot_core_image_t **bot, char **channel, char **description, const char *filename,
                        bool ignore_bot_tif_tags);

/* This function performs Bayer color conversion.  If the image is non-bayer, it just copies the src to dest.
 * returns 0 on success, 1 if non-bayer image, -1 on error
 */
int
vis_botimage_bayerfilt (bot_core_image_t **dest, const bot_core_image_t *src);

/* This function converts a 16-bit image to an 8-bit image.  If the image is 8-bit, it just copies the src to dest.
 * returns 0 on success, 1 if 8-bit already, -1 on error
 */
int
vis_botimage_16_to_8 (bot_core_image_t **dest, const bot_core_image_t *src);


/* Generates an image file filename. Same syntax as strftime, but adds %i conversion character option
 * for microseconds and %f conversion character for framenumber.
 */
size_t
vis_botimage_filename (char *filename, size_t len, const char *format, int64_t utime, uint32_t framecount);


#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_BOTIMAGE_H__
