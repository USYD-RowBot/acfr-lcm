#ifndef __PROSILICA_H__
#define __PROSILICA_H__

#include <stdio.h>
#include <stdbool.h>
#include <tiffio.h>
#include <PvApi.h>

#include "perls-common/error.h"
#include "perls-lcmtypes/senlcm_prosilica_t.h"

#define PROSILICA_MAX_PACKET_SIZE          8228
#define PROSILICA_STREAMBYTESPERSECOND_MAX 124000000

/* note the ", ## args"
 * This macro works even if no variable argument list is given -- the precompile will remove the comma ","
 */
#define PROSILICA_ERROR(err, format, ...)                               \
    fprintf (stderr, "%s:%s:%d:%s " format "\n",                        \
             __SRC_FILE__, __func__, __LINE__,                          \
             err ? prosilica_error_string ((tPvErr) err) : "", ## __VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pvattribute pvattribute_t;
struct pvattribute {
    tPvDatatype datatype;
    char        label[256];
    char        value[256];
};

typedef struct prosilica_bandwidth prosilica_bandwidth_t;
struct prosilica_bandwidth {
    tPvUint32 StreamBytesPerSecond;
    tPvUint32 Used;
    double    Percentage;
};

/* Convert tPvErr type to string
 * returns string pointer on success, NULL on error
 */
const char *
prosilica_error_string (tPvErr err);

/* query camera's internal Timestamp count
   returns 0 on success, -1 on error
 */
int
prosilica_get_timestamp (tPvHandle handle, unsigned long *TimestampLo, unsigned long *TimestampHi);

/* query camera attribute 'label'
 * returns 0 success, -1 error
 */
pvattribute_t
prosilica_get_attribute (tPvHandle handle, const char* label);

/* sets camera attribute 'label' to string 'value' 
 * returns 0 success, -1 error 
 */
int
prosilica_set_attribute (tPvHandle handle, const char* label, const char* value);


/* returns pointer on success, NULL on error */
senlcm_prosilica_t *
prosilica_get_pvattributes (tPvHandle handle);

void
prosilica_set_pvattributes (tPvHandle handle, const senlcm_prosilica_t *pvatt);

/* loads a camera attribute file
 * returns 0 success, -1 failure */
int
prosilica_load_attfile (tPvHandle handle, const char *attfile);

/* loads a CameraConfigFile specified by index
   set index=0 for 'Factory'
   return 0 success, -1 error */
int
prosilica_load_config (tPvHandle handle, int index);

/* returns 1 if online; 0 if offline */
int
prosilica_camera_online (unsigned long uid);


/* auto computes and configures camera for optimal StreamBytesPerSecond
 * returns StreamBytesPerSecond on success, -1 error */
int
prosilica_multicam (tPvHandle handle, int ncameras);


/* returns percentage of allocated bandwidth used with the current frame settings
 */
int
prosilica_bandwidth_used (tPvHandle handle, prosilica_bandwidth_t *bw);

/* returns an array of tPvFrame objects, NULL on error 
 */
tPvFrame *
prosilica_alloc_frames (tPvHandle handle, int nframes);

/* duplicates an array of tPvFrame objects
 * returns a pointer to the duplicated Frames, NULL on error
 */
tPvFrame *
prosilica_dup_frames (tPvFrame *Frames, int nframes);

/* frees an array of tPvFrame objects */
void
prosilica_free_frames (tPvFrame *Frames, int nframes);


/* returns the number of pixels in the ImageBuffer */
size_t
prosilica_npixels (tPvFrame *Frame);

/* bit shift the image frame by 4-bits
 * return 0 success, -1 error
 */
int
prosilica_bitshift_frame (tPvFrame *Frame);

/* returns 0 success, -1 error
   compression = {COMPRESSION_NONE; COMPRESSION_LZW; COMPRESSION_JPEG, COMPRESSION_DEFLATE} // defined in <tiffio.h>
   
   quality = {0-100}, only valid if compression is none
*/
int
prosilica_save_tiff (const tPvFrame *Frame, unsigned long uid, const char *dirname, const char *filename, 
                     bool color_interpolate, unsigned short compression, unsigned short quality);

/* Generates an image file filename. Same syntax as strftime, but adds %i conversion character option
 * for microseconds and %f conversion character for framenumber.
 */
size_t
prosilica_gen_filename (char *filename, size_t len, const char *format, int64_t utime, uint32_t framecount);


#ifdef __cplusplus
}
#endif

#endif // __PROSILICA_H__
