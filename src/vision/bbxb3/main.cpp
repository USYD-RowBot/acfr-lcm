#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//#include "perls-common/timestamp.h"

#include "bumblebeexb3.h"

#define DECOMPRESS_JPEG 0

/*clean_exit:
   dc1394_capture_stop( camera );
   dc1394_video_set_transmission( camera, DC1394_OFF );
   dc1394_camera_free( camera );
   exit( 0 );
*/


int
writePpm (const char    *szFilename,
          unsigned char *pucBuffer,
          int		width,
          int		height)
{
  FILE *stream = fopen (szFilename, "wb");
  if (!stream) {
     perror ("Can't open image file");
     return 1;
  }

  fprintf (stream, "P6\n%u %u 255\n", width, height);
  fwrite (pucBuffer, 3*width, height, stream);
  fclose (stream);
  return 0;
}

int
writePgm (const char    *szFilename,
          unsigned char *pucBuffer,
          int		width,
          int		height)
{
  FILE *stream = fopen (szFilename, "wb");
  if (!stream) {
     perror ("Can't open image file");
     return 1;
  }

  fprintf (stream, "P5\n%u %u 255\n", width, height);
  fwrite (pucBuffer, width, height, stream);
  fclose (stream);
  return 0;
}



int
main(int argc, char *argv[])
{
    BB3camera_t *bb3camera = bb3_init ();
    dc1394error_t err;
    
    if (bb3camera) {
        printf ("**************** Success ********************\n");
        //printf ("Camera Model: %s\nCamera Vendor:  %s one shot capable = %d\n",
                //bb3camera->camera->model,bb3camera->camera->vendor, bb3camera->camera->one_shot_capable);
       	err = bb3_setup(bb3camera);
    	if (err != DC1394_SUCCESS) {
            printf ("setup failed\n");
            return -1;
	}
        err = bb3_start_video_transmission (bb3camera);
	if (err != DC1394_SUCCESS) {
            fprintf(stderr, "Failed to start video transmission\n"); 
            bb3_stop_video_transmission (bb3camera);
            bb3_camera_free (bb3camera); 
            return err;
	}
	sleep(6);
        // dc1394video_frame_t *frame;
        //float fps = 0.0;
        int count = 0;
        printf ("starting frame capture\n");

        //dc1394error_t status;
/*
        status = dc1394_capture_dequeue (bb3camera->camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
	
        BB3StreamHeaderInfo_t header = {
            .versionNum = , //4
            .framerate = , //2
            .headSerialNum = 9410017,//from the box
            .paddingBlock = frame->padding_bytes,
            .dataFormat = ,
            .resolution = ,
            .bayerPattern = ,
            .configDataSize = , //189740,
            .numImages = 100,
            .numIndex_M = 0x01,
            .interval_K = 0x32,
            .streamDataOffset = , //0x0002f200, //848 + 8 + 2048 + header.configDataSize;
            .gpsDataOffset = 0,
            .gpsDataSize = 0,
        };

        status = dc1394_capture_enqueue (bb3camera->camera, frame);
    
        FILE *pgr_fp = fopen("test-000000.pgr","wb+"); 
        bb3_write_stream_header_info (pgr_fp, header);
        bb3_write_calib_data (pgr_fp, "bumblebeexb39410017.cal");
        //int cam, k;
        char BASENAME[256];
        //char filename[512];
        //FILE *fd;
        sprintf(BASENAME, "RAW-");*/
  	 // size of capture buffer
        unsigned int nBufferSize = bb3camera->nRows * bb3camera->nCols * bb3camera->nBytesPerPixel;
        // allocate a buffer to hold the de-interleaved images
        unsigned char *pucDeInterlacedBuffer = new unsigned char[ nBufferSize ];//

        while (count < 1) {//change
            if (bb3camera->bColor) {
                unsigned char *pucRGBBuffer = new unsigned char[ 3 * nBufferSize ];//new
                unsigned char *pucGreenBuffer = new  unsigned char[ nBufferSize ];//new
                unsigned char *pucRightRGB  = NULL;
                unsigned char *pucLeftRGB   = NULL;
                unsigned char *pucCenterRGB = NULL;

	      // get the images from the capture buffer and do all required processing
	      extractImagesColor (&(*bb3camera),
				  DC1394_BAYER_METHOD_NEAREST,
				  pucDeInterlacedBuffer,
				  pucRGBBuffer,
				  pucGreenBuffer,
				  &pucRightRGB,
				  &pucLeftRGB,
				  &pucCenterRGB);

	      // write the color images to file

	      if (!writePpm ("right.ppm", pucRightRGB, bb3camera->nCols, bb3camera->nRows))
                  printf ("wrote right.ppm\n");
	      if (!writePpm ("left.ppm", pucLeftRGB, bb3camera->nCols, bb3camera->nRows))
                  printf( "wrote left.ppm\n");
	      if (pucCenterRGB != pucLeftRGB)
                  if (!writePpm ("center.ppm", pucCenterRGB, bb3camera->nCols, bb3camera->nRows))
                      printf("wrote center.ppm\n");

	      delete[] pucRGBBuffer;
	      delete[] pucGreenBuffer;
            }
            else {
                unsigned char *pucRightMono  = NULL;
                unsigned char *pucLeftMono   = NULL;
                unsigned char *pucCenterMono = NULL;
                //get the images from the capture buffer and do all required processing
                extractImagesMono (&(*bb3camera),
                                   pucDeInterlacedBuffer,
                                   &pucRightMono,
                                   &pucLeftMono,
                                   &pucCenterMono);
                
                
                // write the greyscale images to file

                if (!writePgm ("right.pgm", pucRightMono, bb3camera->nCols, bb3camera->nRows))
                    printf ("wrote right.pgm\n");
                if (!writePgm ("left.pgm", pucLeftMono, bb3camera->nCols, bb3camera->nRows))
                    printf ("wrote left.pgm\n");
                if (pucCenterMono != pucLeftMono)
                    if (!writePgm ("center.pgm", pucCenterMono, bb3camera->nCols, bb3camera->nRows))
                        printf ("wrote center.pgm\n");
            }
        }
        delete[] pucDeInterlacedBuffer;
    }
    else
        printf ("Failure\n");

    printf ("Done \n");
    fflush (stdout);
    bb3_stop_video_transmission (bb3camera);
    bb3_camera_free (bb3camera);  
    
    return 1;
}
