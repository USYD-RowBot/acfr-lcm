#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/select.h>
#include <dc1394/dc1394.h>
#include <dc1394/log.h>
#include <dc1394/utils.h>
#include <dc1394/register.h>
#include <dc1394/control.h>


#include "bumblebeexb3.h"



dc1394error_t
getSensorInfo (dc1394camera_t *camera,
	       int            *pbColor,
	       unsigned int   *pnRows,
	       unsigned int   *pnCols)
{

   uint32_t value;
   dc1394error_t err;
   err = dc1394_get_control_register (camera, 0x1f28, &value);
   if (err != DC1394_SUCCESS)
      return err;

   unsigned char info = 0xf & value;

   switch (info) {
      case 0xA:	// color 640x480
	 *pbColor	= 1;
	 *pnRows	= 480;
	 *pnCols	= 640;
	 break;
      case 0xB:	// mono 640x480
	 *pbColor	= 0;
	 *pnRows	= 480;
	 *pnCols	= 640;
	 break;
      case 0xC:	// color 1024x768
	 *pbColor	= 1;
	 *pnRows	= 768;
	 *pnCols	= 1024;
	 break;
      case 0xD:	// mono 1024x768
	 *pbColor	= 0;
	 *pnRows	= 768;
	 *pnCols	= 1024;
	 break;
      case 0xE:	// color 1280x960
	 *pbColor	= 1;
	 *pnRows	= 960;
	 *pnCols	= 1280;
	 break;
      case 0xF:	// mono 1280x960
	 *pbColor	= 0;
	 *pnRows	= 960;
	 *pnCols	= 1280;
	 break;
      default:
	 // unknown sensor!
	 printf ("Bad sensro info.\n");
	 return DC1394_FAILURE;
   }

   return err;
}

void
dc1394_deinterlace_green (unsigned char *src, 
			  unsigned char *dest, 
			  unsigned int width, 
			  unsigned int height)
{
  register int i = (width*height)-2;
  register int g = ((width*height)/3)-1;

  while (i >= 0)
    dest[g--] = src[i-=3];
}

void
extractImagesColor (BB3camera_t *bb3camera, 
		    dc1394bayer_method_t bayerMethod,
		    unsigned char *pucDeInterleaved,
		    unsigned char *pucRGB,
		    unsigned char *pucGreen,
		    unsigned char **ppucRightRGB,
		    unsigned char **ppucLeftRGB,
		    unsigned char **ppucCenterRGB) 
{
   dc1394error_t err;
   dc1394video_frame_t *frame;
   err = dc1394_capture_dequeue (bb3camera->camera,
				 DC1394_CAPTURE_POLICY_WAIT,
				 &frame);
   if (err != DC1394_SUCCESS) {
       fprintf (stderr, "extractImagesColor - cannot dequeue image!\n");
       return;
   }

   unsigned char *pucGrabBuffer = frame->image;

   if (bb3camera->nBytesPerPixel == 2) {
       // de-interlace the 16 bit data into 2 bayer tile pattern images
       dc1394_deinterlace_stereo (pucGrabBuffer,
                                  pucDeInterleaved,
                                  bb3camera->nCols,
                                  2*bb3camera->nRows);
       // extract color from the bayer tile image
       // note: this will alias colors on the top and bottom rows
       dc1394_bayer_decoding_8bit (pucDeInterleaved,
                                   pucRGB,
                                   bb3camera->nCols,
                                   2*bb3camera->nRows,
                                   bb3camera->bayerTile,
                                   bayerMethod);
       // now deinterlace the RGB Buffer to extract the green channel
       // The green channel is a quick and dirty approximation to the mono
       // equivalent of the image and can be used for stereo processing
       dc1394_deinterlace_green (pucRGB,
                                 pucGreen,
                                 bb3camera->nCols,
                                 6*bb3camera->nRows);
       *ppucRightRGB 	= pucRGB;
       *ppucLeftRGB 	= pucRGB + 3 * bb3camera->nRows * bb3camera->nCols;
       *ppucCenterRGB	= *ppucLeftRGB;
   }
   else
       fprintf (stderr, "Set Bytes per to 2 !\n");
      
   // return buffer for use
   dc1394_capture_enqueue (bb3camera->camera, frame);
   return;
}

BB3camera_t * 
bb3_init (void)
{
    BB3camera_t *bb3camera = (BB3camera_t*) malloc (sizeof (*bb3camera));
    dc1394camera_t *cam;
    dc1394_t *dc1394 = dc1394_new ();
    if (!dc1394) {
        printf ("Returning dc1394 is null\n"); 
        return NULL;
    }

    dc1394camera_list_t *camlist;
    if (dc1394_camera_enumerate (dc1394, &camlist) < 0) {
        dc1394_free (dc1394);
        printf ("No camera found\n");
        return NULL;
    }
  
   //loop over all firewire cameras available
    for (unsigned int i = 0; i < camlist->num; i++) {
        cam = dc1394_camera_new (dc1394, camlist->ids[i].guid);
        printf ("cam->model = %s cam->vendor = %s\n", cam->model,cam->vendor);
// May need to check for BUMBLEBEEXB3 vs ladybug in ladybug file ///////////////////////////////////
        if (strncmp (cam->model, "Bumblebee XB3", 13)) { //finds Point grey camera
            dc1394_camera_free (cam);
            cam = NULL;
            continue;
        }
        //snprintf (name, sizeof (name), "%s %s", cam->vendor, cam->model);
        //printf("Found: %s\n", name);
        break;
    }
  
    dc1394_camera_free_list (camlist);
    if (!cam) {
        printf ("Failed to find bumblebeexb3 camera");
        dc1394_free (dc1394);
        return NULL;
    }

    // show some information about the camera we're using
    dc1394_camera_print_info (cam, stdout);
    
    //dc1394_free (dc1394);
   
    bb3camera->camera = cam;
    bb3camera->dc1394 = dc1394; 

    dc1394error_t status;

    status = getSensorInfo (cam,
                            &bb3camera->bColor,
                            &bb3camera->nRows,
                            &bb3camera->nCols);
     if (status != DC1394_SUCCESS) {
          fprintf (stderr, "Could not query the Sensor Info Register!\n");
          return NULL;// status;
     }


     // load the factory defaults - this is auto-everything
     status = dc1394_memory_load (bb3camera->camera, 0);
     if (status != DC1394_SUCCESS) {
          fprintf (stderr, "Can't load default memory channel\n");
          return NULL;// status;
     }

     bb3camera->nBytesPerPixel = 2;
     return bb3camera;
}


dc1394error_t 
bb3_set_operation_mode (BB3camera_t *bb3camera, dc1394operation_mode_t mode)
{
    dc1394error_t status;
    status = dc1394_video_set_operation_mode (bb3camera->camera, mode);
    DC1394_ERR_RTN (status, "setting operation mode");
    return status;
}

/*This function sets the isochronous speed */
dc1394error_t
bb3_set_iso_speed (BB3camera_t *bb3camera, dc1394speed_t speed)
{
    //For speed greater than 400 Mbps you need to set the operation mode to 1394B
    dc1394error_t status;
    status = dc1394_video_set_iso_speed (bb3camera->camera, speed);
    DC1394_ERR_RTN (status, "setting speed");
    return status;
}

/*This function sets video mode */
dc1394error_t
bb3_set_video_mode (BB3camera_t *bb3camera)
{
    dc1394error_t status;
    status = dc1394_video_set_mode( bb3camera->camera, DC1394_VIDEO_MODE_FORMAT7_3 );
    DC1394_ERR_RTN (status, "setting video mode");
    return status;
}

/**
 *This function sets the jpeg quality of the camera. 
 *quality = 1 to 100
 */
dc1394error_t
bb3_set_jpeg_quality (BB3camera_t *bb3camera, uint8_t quality)
{
    dc1394error_t status;
    dc1394video_mode_t mode;
    status = dc1394_video_get_mode (bb3camera->camera, &mode);
    uint32_t tempwidth, tempheight;
    status = dc1394_format7_get_image_size (bb3camera->camera, mode, &tempwidth, &tempheight);
    DC1394_ERR_RTN (status, "setting image size");
    printf ("Inside set jpeg quality : width = %d, height = %d\n", tempwidth, tempheight);
  
    //set image size
    int height = tempheight*quality/100;
    printf ("new height = %d\n", height);
    status = dc1394_format7_set_image_size (bb3camera->camera, mode, tempwidth, height);
    
    uint32_t recbpp;
    status = dc1394_format7_get_recommended_packet_size (bb3camera->camera, mode, &recbpp);
    DC1394_ERR_RTN (status, "getting recommended bytes per packet\n");
    printf ("recommended bytes per packet: %d\n", recbpp);
  
    status = dc1394_format7_set_packet_size (bb3camera->camera, mode, recbpp);
    DC1394_ERR_RTN (status, "setting bytes per packet\n");
    return status;
}



/**
 * This function starts the video transmission, i.e. data starts transferring and image buffer
 * starts getting filled
 */
dc1394error_t
bb3_start_video_transmission (BB3camera_t *bb3camera)
{
    dc1394error_t stat;
   
    // start data transmission
    stat = dc1394_video_set_transmission (bb3camera->camera, DC1394_ON);
    DC1394_ERR_RTN (stat, "enabling video transmission");

   dc1394switch_t status = DC1394_OFF;

   for (int i = 0; i <= 6; i++) {
      usleep (50000);
      stat = dc1394_video_get_transmission (bb3camera->camera, &status);
      if (stat != DC1394_SUCCESS) {
          fprintf (stderr, "Status not obtained\n");
          return stat;
      }
      if (status != DC1394_OFF)
          break;

      if(i == 6) {
	 fprintf (stderr, "Camera failed to turn on!\n");
	 return DC1394_FAILURE;
      }
   }
   
   return DC1394_SUCCESS;
}


dc1394error_t
getBayerTile (dc1394camera_t *camera, dc1394color_filter_t *bayerPattern)
{
    uint32_t val;
    dc1394error_t err; 
    err = dc1394_get_control_register (camera, 0x1040 , &val);
    if (err != DC1394_SUCCESS)
        return err;

    switch (val) {
    case 0x59595959:
        *bayerPattern = (dc1394color_filter_t) 0;
        break;
    case 0x52474742:
        *bayerPattern = DC1394_COLOR_FILTER_RGGB;
        break;
    case 0x47425247:
        *bayerPattern = DC1394_COLOR_FILTER_GBRG;
        break;
    case 0x47524247:
        *bayerPattern = DC1394_COLOR_FILTER_GRBG;
        break;
    case 0x42474752:
        *bayerPattern = DC1394_COLOR_FILTER_BGGR;
        break;
    default:
        break;
    }

    return err;
}	

dc1394error_t
bb3_setup (BB3camera_t *bb3camera)
{
    dc1394error_t err;
	
    dc1394color_coding_t code;// = DC1394_COLOR_CODING_MONO16;
	
    /*err = dc1394_memory_load(bb3camera->camera, 0);
      if(err != DC1394_SUCCESS){
      fprintf(stderr, "Can't load default memory channel\n");
      return err;
      }*/
    err = dc1394_set_control_register (bb3camera->camera, 0x1048 , 0x80000000);
    if (err != DC1394_SUCCESS) {
        fprintf (stderr, "Can't set endian\n");
        return err;
    }
    if (bb3camera->bColor) {
        code = DC1394_COLOR_CODING_RAW16;
        fprintf (stderr, "Color code = RAW16\n");
    }
    else {
        code = DC1394_COLOR_CODING_MONO16;
        fprintf (stderr, "Color code = MONO16\n");
    }
		
    err = bb3_set_operation_mode (bb3camera, DC1394_OPERATION_MODE_1394B);
    if (err != DC1394_SUCCESS) {
        fprintf (stderr, "Failed to set operational mode\n");
        return err;
    }
    err = bb3_set_iso_speed (bb3camera, DC1394_ISO_SPEED_800);
    if (err != DC1394_SUCCESS) {
        fprintf (stderr, "Failed to set ISO Speed \n");
        return err;
    }
    err = bb3_set_video_mode (bb3camera);
    if (err != DC1394_SUCCESS) {
        fprintf (stderr, "Failed to set video mode \n");
        return err;
    }

    err = dc1394_format7_set_roi (bb3camera->camera, DC1394_VIDEO_MODE_FORMAT7_3, code, 
                                  DC1394_USE_MAX_AVAIL, 0, 0, bb3camera->nCols, bb3camera->nRows);
    if (err != DC1394_SUCCESS) {
        fprintf (stderr, "Can't setup BB XB3 capture\n");
        return err;
    }
    err = dc1394_capture_setup (bb3camera->camera, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
    if (err != DC1394_SUCCESS) {
        fprintf (stderr, "Can't setup BB3 capture\n");
        return err;
    }
    err = getBayerTile (bb3camera->camera, &bb3camera->bayerTile);
    if (err != DC1394_SUCCESS) {
        fprintf (stderr, "Could not query the Bayer Tile Reg\n");
        return err;
    }
    return DC1394_SUCCESS;
}

/**
 * This function writes the bumblebeexb3 stream header info into the file
 */
void
bb3_write_stream_header_info (FILE *fp, BB3StreamHeaderInfo_t header)
{
    // Signature	
    fprintf (fp,"PGRLADYBUGSTREAM");
    fwrite (&header.versionNum, 4, 1, fp);
    fwrite (&header.framerate, 4, 1, fp);
    fwrite (&header.headSerialNum, 4, 1, fp);
    uint32_t reserve = 0x0000;
    for (int i = 0 ; i < 25; i++)
        fwrite (&reserve, 4, 1, fp);

    fwrite (&header.paddingBlock, 4, 1, fp);
    //fwrite (&header.dataFormat, 4, 1, fp);
    //fwrite (&header.resolution, 4, 1, fp);
    //fwrite (&header.bayerPattern, 4, 1, fp);
    fwrite (&header.configDataSize, 4, 1, fp);
    fwrite (&header.numImages, 4, 1, fp);
    fwrite (&header.numIndex_M, 4, 1, fp);
    fwrite (&header.interval_K, 4, 1, fp);
    fwrite (&header.streamDataOffset, 4, 1, fp);
    fwrite (&header.gpsDataOffset, 4, 1, fp);
    fwrite (&header.gpsDataSize, 4, 1, fp);
    for (int i = 0; i < 723; i++)
        fwrite (&reserve, 4, 1, fp);

    fwrite (&header.streamDataOffset, 4, 1, fp);
    //fwrite (&reserve, 4, 212, fp);
    //fwrite (&reserve, 4, 512, fp);
}
/**
 * This function writes the calib data into the file stream
 */
void
bb3_write_calib_data (FILE *fp, const char *calib_file_name)
{
    //uint32_t a = 0;
    FILE *calib_fp = fopen (calib_file_name, "rb");
    // while (fread (&a, 4, 1, calib_fp), !feof (calib_fp) && !ferror(calib_fp)) {
    //     fwrite (&a, 4, 1, fp); 
    //printf ("I read %d\n", a);
    // }
  
    uint32_t reserve = 0;
    for (int i = 0; i < 57; i++)
        fwrite (&reserve, 4, 1, fp);
  
    if (feof (calib_fp))
        printf ("End of file was reached.\n");

    if (ferror (calib_fp))
        printf ("An error occurred.\n");

    fclose (calib_fp);
}

/**
 * This function frees the parameter
 */
void
bb3_camera_free (BB3camera_t *bb3camera)
{
    dc1394_camera_free (bb3camera->camera);
    dc1394_free (bb3camera->dc1394);
}


/**
 * This function stops the video transmission
 */
dc1394error_t
bb3_stop_video_transmission (BB3camera_t *bb3camera)
{
    dc1394error_t status;
    status = dc1394_video_set_transmission (bb3camera->camera, DC1394_OFF);
    DC1394_ERR_RTN (status, "disabling video transmission");

    // release capture resources
    status=dc1394_capture_stop (bb3camera->camera);
    DC1394_ERR_RTN (status, "Could not stop capture");

    //dc1394_camera_free (camera);

    return status;
}

void
extractImagesMono (BB3camera_t   *bb3camera, 
		   unsigned char *pucDeInterleaved,
		   unsigned char **ppucRightMono8,
		   unsigned char **ppucLeftMono8,
		   unsigned char **ppucCenterMono8) 
{

    dc1394error_t err;
    // RC7
    dc1394video_frame_t *frame;
    err = dc1394_capture_dequeue (bb3camera->camera,
                                  DC1394_CAPTURE_POLICY_WAIT,
                                  &frame);
    if (err != DC1394_SUCCESS) {
        fprintf (stderr, "extractImagesColor - cannot dequeue image!\n");
        return;
    }

    unsigned char *pucGrabBuffer = frame->image;

    unsigned char *right  = NULL;
    unsigned char *left   = NULL;
    unsigned char *center = NULL;
    if (bb3camera->nBytesPerPixel == 2) {
        // de-interlace the 16 bit data into 2 mono images
        dc1394_deinterlace_stereo (pucGrabBuffer,
                                   pucDeInterleaved,
                                   bb3camera->nCols,
                                   2*bb3camera->nRows);
        right = pucDeInterleaved;
        left  = pucDeInterleaved + bb3camera->nRows * bb3camera->nCols;
        center= left;
    }
    else {
        fprintf (stderr, "ERROR: Set Bytes per pixel to 2!\n");
        /*    dc1394_deinterlace_rgb( pucGrabBuffer,
              pucDeInterleaved,
              bb3camera->nCols,
              3*bb3camera->nRows );

              // NOTE: this code needs to be double checked.
              // Currently 3-bytes-per-pixel is not activatable in this example
              right 	= pucDeInterleaved;
              center  	= pucDeInterleaved + bb3camera->nRows * bb3camera->nCols;
              left	= pucDeInterleaved + 2 * bb3camera->nRows * bb3camera->nCols;*/
    }
      
    *ppucRightMono8  = right;
    *ppucLeftMono8   = left;
    *ppucCenterMono8 = center;

    
    // return buffer for use
    dc1394_capture_enqueue (bb3camera->camera, frame);

    return;
}



dc1394error_t
bb3_set_frame_rate (BB3camera_t *bb3camera, float framerate)
{
    //Check if framerate is out of range then set the max framerate available
    //for that mode
    dc1394error_t status;
    dc1394video_mode_t mode;
    float maxframerate;
    status = dc1394_video_get_mode (bb3camera->camera, &mode);

    if (mode == DC1394_VIDEO_MODE_FORMAT7_0) {
        maxframerate = 6.5;
        if (framerate > maxframerate) {
            framerate = maxframerate;
            printf ("Max frame rate supported for BB3_FULL_RES_UNCOMPRESSED mode is %f\n", framerate);
        }
    }

    else if (mode == DC1394_VIDEO_MODE_FORMAT7_2) {
        maxframerate = 13.0;
        if (framerate > maxframerate) {
            framerate = maxframerate;
            printf ("Max frame rate supported for BB3_HALF_RES_UNCOMPRESSED mode is %f\n", framerate);
        }
    }
  
    else if (mode == DC1394_VIDEO_MODE_FORMAT7_7) {
        maxframerate = 15.0;
        if (framerate > maxframerate) {
            framerate = maxframerate;
            printf ("Max frame rate supported for BB3_FULL_RES_JPEG mode is %f\n", framerate);
        }
    }
  
    else if (mode == DC1394_VIDEO_MODE_FORMAT7_3) {
        maxframerate = 30.0;
        if (framerate > maxframerate) {
            framerate = maxframerate;
            printf ("Max frame rate supported for BB3_HALF_RES_JPEG mode is %f\n", framerate);
        }
    }
  

    //Control frame rate with Absolute value control register
    uint32_t value=0;
    value = 0xC2000000; 
    status = dc1394_set_control_register (bb3camera->camera, 0x83C , value); //0x83C = BB3_REGISTER_FRAME_RATE_CONTROL
    DC1394_ERR_RTN (status, "Cannot set the frame rate register value");


    //copy the bits of IEEE*4 32-bit floating point number to unsigned integer.
    uint32_t ui;
    memcpy (&ui, &framerate, sizeof (ui));
    status = dc1394_set_control_register (bb3camera->camera, 0x968, ui); //0x968 = BB3_REGISTER_ABS_FRAME_RATE_CONTROL
    DC1394_ERR_RTN (status, "Cannot set the frame rate register value");
    return status;
}

