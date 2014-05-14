#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <sys/types.h>
#include <jpeglib.h>
#include <jerror.h>

#include <jconfig.h>		/* auto configuration options */
#define JCONFIG_INCLUDED	/* so that jpeglib.h doesn't do it again */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include "ladybug3_util.h"

/*
 * Copyright (C) 1994-1996, Thomas G. Lane.
 * This file is part of the Independent JPEG Group's software.
 * For conditions of distribution and use, see the accompanying README file.
 *
 * This file contains decompression data source routines for the case of
 * reading JPEG data from a memory buffer that is preloaded with the entire
 * JPEG file.  This would not seem especially useful at first sight, but
 * a number of people have asked for it.
 * This is really just a stripped-down version of jdatasrc.c.  Comparison
 * of this code with jdatasrc.c may be helpful in seeing how to make
 * custom source managers for other purposes.
 */
/* this is not a core library module, so it doesn't define JPEG_INTERNALS */
/* Expanded data source object for memory input */

typedef struct lb3_source_mgr lb3_source_mgr_t;
struct lb3_source_mgr {
    struct jpeg_source_mgr pub;	/* public fields */
    JOCTET eoi_buffer[2];       /* a place to put a dummy EOI */
};
typedef lb3_source_mgr_t *lb3_src_ptr;

/*
 * Initialize source --- called by jpeg_read_header
 * before any data is actually read.
 */
void
lb3_init_source (j_decompress_ptr cinfo)
{
    /* No work, since jpeg_memory_src set up the buffer pointer and count.
     * Indeed, if we want to read multiple JPEG images from one buffer,
     * this *must* not do anything to the pointer.
     */
}

/*
 * Fill the input buffer --- called whenever buffer is emptied.
 *
 * In this application, this routine should never be called; if it is called,
 * the decompressor has overrun the end of the input buffer, implying we
 * supplied an incomplete or corrupt JPEG datastream.  A simple error exit
 * might be the most appropriate response.
 *
 * But what we choose to do in this code is to supply dummy EOI markers
 * in order to force the decompressor to finish processing and supply
 * some sort of output image, no matter how corrupted.
 */
int
lb3_fill_input_buffer (j_decompress_ptr cinfo)
{
    lb3_src_ptr src = (lb3_src_ptr) cinfo->src;

    WARNMS (cinfo, JWRN_JPEG_EOF);

    /* Create a fake EOI marker */
    src->eoi_buffer[0] = (JOCTET) 0xFF;
    src->eoi_buffer[1] = (JOCTET) JPEG_EOI;
    src->pub.next_input_byte = src->eoi_buffer;
    src->pub.bytes_in_buffer = 2;

    return TRUE;
}

/*
 * Skip data --- used to skip over a potentially large amount of
 * uninteresting data (such as an APPn marker).
 *
 * If we overrun the end of the buffer, we let fill_input_buffer deal with
 * it.  An extremely large skip could cause some time-wasting here, but
 * it really isn't supposed to happen ... and the decompressor will never
 * skip more than 64K anyway.
 */
void
lb3_skip_input_data (j_decompress_ptr cinfo, long num_bytes)
{
    lb3_src_ptr src = (lb3_src_ptr) cinfo->src;

    if (num_bytes > 0) {
      while (num_bytes > (long) src->pub.bytes_in_buffer) {
        num_bytes -= (long) src->pub.bytes_in_buffer;
        (void) lb3_fill_input_buffer (cinfo);
        /* note we assume that fill_input_buffer will never return FALSE,
         * so suspension need not be handled.
         */
      }
      src->pub.next_input_byte += (size_t) num_bytes;
      src->pub.bytes_in_buffer -= (size_t) num_bytes;
    }
}

/*
 * An additional method that can be provided by data source modules is the
 * resync_to_restart method for error recovery in the presence of RST markers.
 * For the moment, this source module just uses the default resync method
 * provided by the JPEG library.  That method assumes that no backtracking
 * is possible.
 */

/*
 * Terminate source --- called by jpeg_finish_decompress
 * after all data has been read.  Often a no-op.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */
void
lb3_term_source (j_decompress_ptr cinfo)
{
    /* no work necessary here */
}

/*
 * Prepare for input from a memory buffer.
 */
void lb3_jpeg_memory_src (j_decompress_ptr cinfo, const JOCTET * buffer, size_t bufsize)
{
    lb3_src_ptr src;

    /* The source object is made permanent so that a series of JPEG images
     * can be read from a single buffer by calling jpeg_memory_src
     * only before the first one.
     * This makes it unsafe to use this manager and a different source
     * manager serially with the same JPEG object.  Caveat programmer.
     */
    if (cinfo->src == NULL) {	/* first time for this JPEG object? */
        cinfo->src = (struct jpeg_source_mgr *)
          (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
            			  sizeof (lb3_source_mgr_t));
    }

    src = (lb3_src_ptr) cinfo->src;
    src->pub.init_source = lb3_init_source;
    src->pub.fill_input_buffer = lb3_fill_input_buffer;
    src->pub.skip_input_data = lb3_skip_input_data;
    src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
    src->pub.term_source = lb3_term_source;

    src->pub.next_input_byte = buffer;
    src->pub.bytes_in_buffer = bufsize;
}

/**
 * This function decompresses the jpeg buffer
 */
int
lb3_decompress_jpeg (const unsigned char *src, unsigned long size,  unsigned char *dest, 
                     const uint32_t width, const uint32_t height)
{
    
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    /* Now we can initialize the JPEG decompression object. */
    jpeg_create_decompress (&cinfo);
    lb3_jpeg_memory_src (&cinfo, src, size);
    
    /* Step 3: read file parameters with jpeg_read_header() */
    jpeg_read_header (&cinfo, TRUE);
    
    /* Step 5: Start decompressor */
    jpeg_start_decompress (&cinfo);
    
    uint32_t row_stride = cinfo.output_width * cinfo.output_components;
    //printf("rowstride = %d\n", row_stride);
    //*dest = (unsigned char*) malloc(row_stride*cinfo.output_height);

    //decoded_buffer.resize(row_stride*cinfo.output_height);

    JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)
      ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
    
    uint32_t scan_line=0;
    while (cinfo.output_scanline < cinfo.output_height) {
        //printf("cinfo.output_scanline = %d, cinfo.output_height = %d\n"
        //             , cinfo.output_scanline, cinfo.output_height);
	jpeg_read_scanlines (&cinfo,buffer,1);
	memcpy (dest+scan_line*row_stride,buffer[0],row_stride);
	scan_line++;
    }
    
    jpeg_finish_decompress (&cinfo);
    jpeg_destroy_decompress (&cinfo);    

    return 0;
}

/**
 * Reverses the byte order to big-endian
 */
uint32_t
lb3_reverse_int (uint32_t i)
{
    unsigned char c1, c2, c3, c4;

    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;

    return ((uint32_t)c1 << 24) + ((uint32_t)c2 << 16) + ((uint32_t)c3 << 8) + c4;
}

/**
 * This function reads the header from the file
 */
int 
lb3_read_stream_header (FILE *fp, LB3StreamHeaderInfo_t *header)
{
    // Signature	
    fscanf (fp,"PGRLADYBUGSTREAM");
    fread (&header->versionNum, 4, 1, fp);
    fread (&header->framerate, 4, 1, fp);
    fread (&header->baseSerialNum, 4, 1, fp);
    fread (&header->headSerialNum, 4, 1, fp);
    uint32_t reserve = 0x0000;
    for (int i = 0 ; i < 25; i++)
        fread (&reserve, 4, 1, fp);

    fread (&header->paddingBlock, 4, 1, fp);
    fread (&header->dataFormat, 4, 1, fp);
    fread (&header->resolution, 4, 1, fp);
    fread (&header->bayerPattern, 4, 1, fp);
    fread (&header->configDataSize, 4, 1, fp);
    fread (&header->numImages, 4, 1, fp);
    fread (&header->numIndex_M, 4, 1, fp);
    fread (&header->interval_K, 4, 1, fp);
    fread (&header->streamDataOffset, 4, 1, fp);
    fread (&header->gpsDataOffset, 4, 1, fp);
    fread (&header->gpsDataSize, 4, 1, fp);
    //Reserved space is 848 bytes 
    //Also need to fill space for Image index [0] to Image index [M-1]
    //Basically fill the space from position 0xA0 to 0xBF0
    for (int i = 0; i < 723; i++)
        fread (&reserve, 4, 1, fp);
 
    //|| Image index [0] || 4 unsigned int || Offset of image 0 ||
    //Since numIndex_M is fixed to 0
    fread (&header->streamDataOffset, 4, 1, fp);
    return 0;   
}

/**
 * Read Calibration data and save it as a .cal file
 */
void
lb3_read_calib_data (FILE *fp, LB3StreamHeaderInfo_t *header, const char *folder)
{
    uint32_t a;
    char filename[1024];
    sprintf (filename, "%s/ladybug%d.cal", folder, header->headSerialNum);
    FILE *calib_fp = fopen (filename, "wb");
    int i = 0;
    while (i < header->configDataSize) {
        fread (&a, 4, 1, fp); 
        fwrite (&a, 4, 1, calib_fp); 
        i = i + 4;
    }
    
    fclose (calib_fp);
}

/**
 * This function reads the jpeg header
 */
void
lb3_read_jpeg_header (FILE *fp, LB3JpegHeaderInfo_t *jpegHeader)
{
    uint32_t reserve = 0x0000;
    fread (&jpegHeader->timestamp, 8, 1, fp);
    //fread (&reserve, 4, 1, fp);
    fread (&jpegHeader->dataSize, 4, 1, fp);
    fread (&reserve, 4, 1, fp);
    fread (&jpegHeader->fingerPrint, 4, 1, fp);
    fread (&jpegHeader->versionNumber, 4, 1, fp);
    fread (&jpegHeader->timestamp_sec, 4, 1, fp);
    fread (&jpegHeader->timestamp_musec, 4, 1, fp);
    fread (&jpegHeader->seqId, 4, 1, fp);
    fread (&jpegHeader->refreshRate, 4, 1, fp);
    for (int i = 0; i < 6; i++)
        fread (&jpegHeader->gain[i], 4, 1, fp);
    fread (&jpegHeader->whiteBalance, 4, 1, fp);
    fread (&jpegHeader->bayerGain, 4, 1, fp);
    fread (&jpegHeader->bayerMap, 4, 1, fp);
    fread (&jpegHeader->brightness, 4, 1, fp);
    fread (&jpegHeader->gamma, 4, 1, fp);
    fread (&jpegHeader->headSerialNum, 4, 1, fp);
    for (int i = 0; i < 6; i++)
        fread (&jpegHeader->shutter[i], 4, 1, fp);
    //reserved 24 + 632 + 56 = 712 = 178*4 bytes
    for (int i = 0; i < 178; i++)
        fread (&reserve, 4, 1, fp);
    fread (&jpegHeader->gpsOffset, 4, 1, fp);
    fread (&jpegHeader->gpsSize, 4, 1, fp);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 4; j++) {
            fread (&jpegHeader->jpegDataOffset[i][j], 4, 1, fp);
            fread (&jpegHeader->jpegDataSize[i][j], 4, 1, fp);
            jpegHeader->jpegDataOffset[i][j] = lb3_reverse_int (jpegHeader->jpegDataOffset[i][j]);
            jpegHeader->jpegDataSize[i][j] = lb3_reverse_int (jpegHeader->jpegDataSize[i][j]);
            //printf("jpeg offset [%d][%d] = %d\n", i, j,  jpegHeader->jpegDataOffset[i][j]);
            //printf("jpeg size   [%d][%d] = %d\n", i, j,  jpegHeader->jpegDataSize[i][j]);
        }
    }
    //printf ("Timestamp = %"PRId64"\n", jpegHeader->timestamp);
}

