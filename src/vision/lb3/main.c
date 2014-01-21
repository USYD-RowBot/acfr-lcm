#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <inttypes.h>
#include <sys/select.h>
#include <dc1394/dc1394.h>
#include <bot_core/bot_core.h>

#include "perls-lcmtypes/bot_core_image_sync_t.h"
#include "perls-lcmtypes/bot_core_image_t.h"

#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#include "perls-vision/botimage.h"

#include "perls-vision/ladybug3_util.h"
#include "ladybug3.h"

#define MAX_FILE_SIZE 2000000000
#define MAX_CIRC_BUF_SIZE 100


/**
 * This structure contains the parameters to be entered by user
 * from command line.
 */
typedef struct ConfigParams ConfigParams_t;
struct ConfigParams {
    float framerate;
    int compression_mode;
    float quality;
    int resolution;
    char *folder;
    char *calib_file;
    char folderName[1024];
    char basename[64];
    int lcm_flag;
    int pgr_flag;
    int software_compression;
    int auto_max_shutter;
    float max_shutter;
};

/**
 * This struct maintains the current state.
 */
typedef struct _LB3state LB3state_t;
struct _LB3state
{
    ConfigParams_t configParams;
    LB3StreamHeaderInfo_t header;
    LB3camera_t *lb3cam;
    //Circular buffer for holding frame data
    BotCircular *circBufData;
    pthread_mutex_t mutex;
    pthread_cond_t cond_buffer_ready;
    int record_flag;
    lcm_t *lcm;
};

//Global exit flag
int g_record_flag; 
int g_exit_write_thread;

void
print_help (int exval, char **argv) 
{
    printf("Usage:%s [-h] [-l] [-f OUTPUT FOLDER] [-r FRAMERATE] [-m VIDEO MODE (1/2/3/4)] [-q QUALITY (0-100)] [-s SOFTWARE COMPRESSION (1-100)\n\n", argv[0]);
   
    printf("  -h                               print this help and exit\n");
    printf("  -l                               publish thumbnails to LCM\n");
    printf("  -f FOLDER                        set folder where data is logged\n");
    printf("  -c LB3 CALIB FILE                LB3 calibration file obtained from manufacturers (needed when saving in .pgr format)\n");
    printf("  -r FRAMERATE (1-30)              set the framerate (default is 5)\n");
    printf("  -q QUALITY (0-100)               set JPEG quality. default is 75.\n");
    printf("  -s SOFTWARE COMPRESSION (1-100)  enable software compression with quality (1-100) (Default: No software compression).\n");
    printf("  -m VIDEO MODE(1/2/3/4)           set the video mode (Default is 3)\n     1 = Full res (1616x1232) Uncompressed\n     2 = Half res (1616x616)  Uncompressed\n     3 = Full res (1616x1232) JPEG\n     4 = Half res (1616x616)  JPEG\n");
    printf("  -i MAX INTEGRATION TIME          set the maximum integration time (in seconds)\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, ConfigParams_t *configParams)
{
    int opt;
    //Initialize the global parameters to default values
    configParams->framerate = 5;
    configParams->compression_mode = 1;
    configParams->quality = 95;
    configParams->resolution = 1;
    configParams->folder = NULL;
    configParams->calib_file = NULL;
    configParams->lcm_flag = 0;
    configParams->pgr_flag = 1;
    configParams->software_compression = -1;
    configParams->auto_max_shutter = 1;
    int vid_mode = -1;
    
    while ((opt = getopt (argc, argv, "hlf:c:r:m:q:s:i:")) != -1) {
        switch(opt) {
            case 'h':
                print_help (0, argv);
                break;
            case 'l':
                configParams->lcm_flag = 1;
                break;
            case 'f':
                //get the folder
                configParams->folder = (char*)optarg;
                break;
            case 'c':
                //get the calib file
                configParams->calib_file = (char*)optarg;
                break;
            case 'r':
                //get the framerate
                configParams->framerate = atof (optarg);
                
                if(configParams->framerate < 0 || configParams->framerate > 30) {
                   printf("Error: Supported framerate = [0, 30]\n");
                   print_help (1, argv);
                }
                break;
            case 'm':
                //get the compression mode
                vid_mode = atoi(optarg);
                if(vid_mode == 1) {
                      //Full res Uncompressed
                      configParams->compression_mode = 0;
                      configParams->resolution = 1;
                } 
                else if (vid_mode == 2) {
                      //Half res Uncompressed
                      configParams->compression_mode = 0;
                      configParams->resolution = 0;
                 }
                 else if (vid_mode == 3) {
                      //Full res JPEG
                      configParams->compression_mode = 1;
                      configParams->resolution = 1;
                  }
                  else if (vid_mode == 4) {
                      //Half res JPEG
                      configParams->compression_mode = 1;
                      configParams->resolution = 0;
                  }
                  break;
            case 'q':
                //get the compression quality
                configParams->quality = atoi (optarg);
                if(configParams->quality < 0 || configParams->quality > 100) {
                    printf("Error: Supported quality = [0, 100]\n");
                    print_help (1, argv);
                }
                break;
            case 's':
                //get the resolution 
                configParams->software_compression = atoi(optarg);
                if(configParams->resolution  < 0 && configParams->resolution > 100) {
                    printf("Error: Only 1-100 supported for resolution.\n"); 
                    print_help (1, argv);
                }
                break;
            case 'i':
                configParams->max_shutter = atof (optarg);
                configParams->auto_max_shutter = 0;
printf ("max_shutter = %f \n", configParams->max_shutter);
	        break;
            case ':':
                fprintf (stderr, "Error - Option `%c' needs a value\n\n", optopt);
                print_help (1, argv);
                exit (0);
                break;
            case '?':
                fprintf (stderr, "Error - No such option: `%c'\n\n", optopt);
                print_help (1, argv);
        }
    }
    
    if (configParams->folder == NULL) {
        printf ("Enter folder path to store the data\n");
        print_help (1, argv);
    }
    
    if (configParams->calib_file == NULL) {
        printf ("Enter calibration file for the LB3 camera\n");
        print_help (1, argv);
    }
    
    /* 
    // print all remaining options
    */
    for (; optind < argc; optind++)
        printf ("argument: %s\n", argv[optind]);
    
}

/**
 * deletes spaces from the string passed to it
 */
void 
delspace(char *Str)
{
    int Pntr = 0;
    int Dest = 0;
    while (Str [Pntr])
    {
        if (Str [Pntr] != ' ' && Str [Pntr] != ':')
           Str [Dest++] = Str [Pntr];
        else
           Str [Dest++] = '-';
        Pntr++;
    }
    Pntr--;
    Str [Pntr] = '\0';
}

/**
 * This function saves color image as tiff after doing software compression
 */
void
save_color_image_as_tiff (unsigned char *buffer, bot_core_image_t *bot, 
                          int width, int height, LB3state_t *state, int64_t utime)
{
    // Color Conversion assuming bayer pattern RGGB
    // vis_bayer_edge_sense (buffer, rgbbuffer, width, height*6, VIS_BAYER_PATTERN_RGGB);
    IplImage *src = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 1);
    src->imageData = (char *) buffer;
    IplImage *dst = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 3);
    cvCvtColor (src, dst, CV_BayerBG2RGB);
    unsigned char *rgbbuffer = (unsigned char *) dst->imageData;
   
    bot->utime = utime;

    //For each camera image in the buffer
    for (int i = 0; i < 6; i++) { 
        bot->data = rgbbuffer;
        
        //Create the file with utime as the filename  
        char filename[256];
        sprintf (filename, "%s/Cam%d/%"PRId64".tiff", state->configParams.folderName, i, utime);
        
        //save tiff image with software JPEG compression
        unsigned int compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG | state->configParams.software_compression;
        vis_botimage_write_tiff (bot, filename, NULL, NULL, compression);
        rgbbuffer = rgbbuffer + width*height*3;
    }
    
    rgbbuffer = rgbbuffer - width*height*3*6;
    cvReleaseImage (&src);
    cvReleaseImage (&dst);
}


void *
write_thread (void *ptr)
{
  LB3state_t *LB3state = (LB3state_t* )ptr;
  char filename[1024];
  FILE *pgr_fp = NULL;
  uint32_t file_size = 0;

  //Allocate memory for holding bayer blocks sizes
  uint32_t **bayersize;
  bayersize = malloc (sizeof (uint32_t*)*6);
  for(int j = 0; j < 6; j++)
      bayersize[j] = malloc (sizeof (uint32_t)*4);
  //Allocate memory for the bayer pattern image
  unsigned char ***bayer;
  bayer = malloc (sizeof (unsigned char**)*6);
  for(int i = 0; i < 6; i++)
      bayer[i] = malloc (sizeof (unsigned char*)*4);

  //Allocate the buffer
  unsigned char *data = (unsigned char*) malloc (LB3state->circBufData->element_size);

  //If software compression 
  int width = -1, height = -1;
  char cmd[512];
  LB3StreamHeaderInfo_t *header = &LB3state->header; 
  if (header->dataFormat == LADYBUG_DATAFORMAT_SEQUENTIAL) {
      width = 1616; 
      height = 1232; 
  }
  else if (header->dataFormat == LADYBUG_DATAFORMAT_SEQUENTIAL_HALF_HEIGHT) {
      width = 1616; 
      height = 616; 
  }
  if (width == -1 || height == -1) {
      printf ("Unknown Data Format\n");
  }
  unsigned char *rgbbuffer = NULL;
  bot_core_image_t *bot = NULL;
  if (LB3state->configParams.software_compression > 0) {
      //Allocate buffers
      //rgbbuffer = (unsigned char *) malloc (width*height*6*3);
      bot = (bot_core_image_t *) malloc (sizeof (bot_core_image_t));
      //Create bot_core_image_t  
      bot->width = width;
      bot->height = height;
      bot->row_stride = 3*width;
      bot->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB; 
      bot->size = width*height*3;
      for (int i = 0; i < 6; i++) {
          //Create Folders
          sprintf (cmd, "mkdir %s/Cam%d/", LB3state->configParams.folderName, i);
          system (cmd);
      }
  }
 
  while (g_record_flag) {
       //Lock mutex
       pthread_mutex_lock (&LB3state->mutex);
       if( bot_circular_is_empty (LB3state->circBufData)) { 
           //Wait for data
           //pthread condition wait
           pthread_cond_wait (&LB3state->cond_buffer_ready, &LB3state->mutex);
           pthread_mutex_unlock (&LB3state->mutex);
       }
       else { 
           //Get frame from the circular buffer 
           //Get data
           bot_circular_pop_tail (LB3state->circBufData, data); 
           //Get timestamp
           uint64_t embeded_sync_time;
           memcpy (&embeded_sync_time, data, sizeof(uint64_t));          
           uint64_t dc1394_time;
           memcpy (&dc1394_time, data + sizeof(uint64_t), sizeof(uint64_t));          
           uint64_t host_time;
           memcpy (&host_time, data + 2*sizeof(uint64_t), sizeof(uint64_t));
           
           //publish bot_core_image_sync message
           bot_core_image_sync_t timesync;
           timesync.utime = embeded_sync_time;
           bot_core_image_sync_t_publish (LB3state->lcm, "LB3_IMAGESYNC", &timesync);
           
           //Unlock Mutex
           pthread_mutex_unlock (&LB3state->mutex);
          
           // if jpeg data then 
           if (LB3state->configParams.compression_mode) {
               //Create the file with utime as the filename  
               sprintf (filename, "%s/%"PRId64".pgr", LB3state->configParams.folderName, embeded_sync_time);
               pgr_fp = fopen (filename,"wb+"); 
               if (pgr_fp == NULL) {
                   printf ("Error: Cannot open file %s to write data\n", filename);
                   continue;
               }
               
               //write the stream header info
               lb3_write_stream_header_info (pgr_fp, LB3state->header);
               
               //write the calibration data
               lb3_write_calib_data (pgr_fp, LB3state->configParams.calib_file);

               //get the current file size
               file_size = (uint32_t)ftell (pgr_fp); 
               //printf("file_size = %x\n", file_size);

               //This is the position from where the image data starts in the stream file.
               LB3state->header.streamDataOffset = file_size;
           
               // a) get the bayer data and its sizes and create the jpeg header
               LB3JpegHeaderInfo_t jpgHeader;
               jpgHeader.timestamp = embeded_sync_time;
               jpgHeader.dataSize = lb3_reverse_int (LB3state->circBufData->element_size);
               jpgHeader.fingerPrint = lb3_reverse_int (0xCAFEBABE); //From LB3 document
               jpgHeader.versionNumber = lb3_reverse_int (4);
               jpgHeader.timestamp_sec = 0;
               jpgHeader.timestamp_musec = 0;
               jpgHeader.seqId = 1;
               jpgHeader.refreshRate = 0; //Dummy
               //Dummy values
               for(int j = 0; j < 6; j++) {
                   jpgHeader.gain[j] = 0;
                   jpgHeader.shutter[j] = 0;
               }
               jpgHeader.whiteBalance = 0;
               jpgHeader.bayerGain = 0;
               jpgHeader.bayerMap = 0;
               jpgHeader.brightness = 0;
               jpgHeader.gamma = 0;
               jpgHeader.headSerialNum = lb3_reverse_int (LB3state->header.headSerialNum);
               jpgHeader.gpsOffset = 0; //No Gps
               jpgHeader.gpsSize = 0;   //No Gps
               
               //get 24 jpeg blocks sizes
               lb3_get_jpeg_bayer_pattern (data, bayer, bayersize);

               uint32_t offset = 1024 + file_size; 
               for (int cam = 0; cam < 6; cam++) {
                   for(int k = 0; k < 4; k++) {
                       jpgHeader.jpegDataOffset[cam][k] = lb3_reverse_int (offset);
                       jpgHeader.jpegDataSize[cam][k] = lb3_reverse_int (bayersize[cam][k]);
                       offset = offset + bayersize[cam][k]; 
                   }
               }
               // b) write the jpeg header 
               lb3_write_jpeg_header (pgr_fp, &jpgHeader);
               // c) write the data
               for (int cam = 0; cam < 6; cam++) {
                 for (int k = 0; k < 4; k++) {
                   fwrite (bayer[cam][k], bayersize[cam][k], 1, pgr_fp);
                   free (bayer[cam][k]);
                 }
               }  
           }
           else {
               if (LB3state->configParams.software_compression > 0) {
                   // Perform Bayer color conversion 
                   // Create a bot_core_image_t structure
                   // Fill it with the image data
                   // compress the image 
                   // save it as a tiff file
                   save_color_image_as_tiff (data, bot, width, height, LB3state, embeded_sync_time); 
               }
               else { 
                   //Create the file with utime as the filename  
                   sprintf (filename, "%s/%"PRId64".pgr", LB3state->configParams.folderName, embeded_sync_time);
                   pgr_fp = fopen (filename,"wb+"); 
                   if (pgr_fp == NULL) {
                       printf ("Error: Cannot open file %s to write data\n", filename);
                       continue;
                   }
                   
                   //write the stream header info
                   lb3_write_stream_header_info (pgr_fp, LB3state->header);
                   
                   //write the calibration data
                   lb3_write_calib_data (pgr_fp, LB3state->configParams.calib_file);

                   //get the current file size
                   file_size = (uint32_t)ftell (pgr_fp); 
                   //printf("file_size = %x\n", file_size);

                   //This is the position from where the image data starts in the stream file.
                   LB3state->header.streamDataOffset = file_size;
                   // Simply write the data
                   fwrite (data, LB3state->circBufData->element_size, 1, pgr_fp);
               }
           }
 
           if(pgr_fp != NULL) {
               //update header 
               fseek (pgr_fp, 0, SEEK_SET );
               LB3state->header.numImages = 1;
               lb3_write_stream_header_info (pgr_fp, LB3state->header);
               fseek (pgr_fp, 0, SEEK_END);
               
               //Close the file
               fflush (pgr_fp);
               fclose (pgr_fp); 
           }
       }
  }
   
  //Free data
  free (data);
  for (int j = 0; j < 6; j++)
      free (bayersize[j]);
  free (bayersize);
  for (int i = 0; i < 6; i++)
      free (bayer[i]);  
  free (bayer);
 
  if (rgbbuffer)
      free (rgbbuffer);

  if (bot)
      free (bot);
 
  //this flag tells the main thread that write thread exitted successfully. 
  g_exit_write_thread = 1;
  return 0; 
}

/**
 * This thread creates and runs the lcm instance
 */
void*
lcm_thread (void *ptr)
{
    LB3state_t *state = (LB3state_t *)ptr;
    state->lcm = lcm_create (NULL);
    while (1) {
        struct timeval timeout = {
            .tv_sec = 1,
            .tv_usec = 0,
        };
        lcmu_handle_timeout (state->lcm, &timeout);
    } 
}

void 
my_signal_handler (int signum) 
{
    //Stop the recording and finalize the file
    g_record_flag = 0;
    signal (signum, SIG_IGN);
    printf ("\nCtrl+C pressed: Closing Application\n");
    signal (SIGINT, my_signal_handler);
    signal (SIGTERM, my_signal_handler);
}


int
main (int argc, char *argv[])
{
    //Parse command line arguments;
    ConfigParams_t configParams;
    parse_args (argc, argv, &configParams);
    
    //Initialize the camera
    LB3camera_t *lb3cam = lb3_init ();
    
    if (lb3cam == NULL) {
        printf ("Closing.... \n");
        return 0;
    }
  
    g_record_flag = 1; 
    g_exit_write_thread = -1;
     
    //Get the size of calibration file
    FILE *calib_fptr = fopen (configParams.calib_file, "r");
    if (calib_fptr != NULL) {
        fseek (calib_fptr, 0, SEEK_END);
    }
    else {
        printf ("Give a valid calibration file\n");
        return 0;
    }
    uint32_t configDataSize = ftell (calib_fptr);
    fclose (calib_fptr);
    
    //create sub-folder by current time under the parent folder
    //get the current date and time 
    time_t theTime;
    time (&theTime);   // get the calendar time
    struct tm *t = localtime (&theTime);  // convert to local
    char currTime[512];
    strcpy (currTime, asctime (t));
    delspace (currTime);

    sprintf (configParams.folderName,"%s/%s", configParams.folder, currTime);
    char cmd[512];
    sprintf (cmd,"mkdir %s",configParams.folderName);

    //create a folder by this name in the parent folder
    system (cmd);  

 
    dc1394video_frame_t *frame;
    LB3state_t LB3state; 
    float fps = 0.0; 
    LB3video_mode_t vidMode;
    LadybugDataFormat_t lb3dataformat;
    
    signal (SIGINT, my_signal_handler);
    signal (SIGTERM, my_signal_handler);

    /* int width = -1; */
    /* int height = -1; */
    if (lb3cam) {
        printf ("Camera Model: %s\nCamera Vendor:  %s\n",lb3cam->cam->model,lb3cam->cam->vendor);
        lb3_set_operation_mode (lb3cam, DC1394_OPERATION_MODE_1394B);
        lb3_set_iso_speed (lb3cam, DC1394_ISO_SPEED_800);
        //Configure the camera based on input parameters 
        //set the video mode
        if (configParams.compression_mode == 0) { //Uncompressed   
            if (configParams.resolution == 0) {
                vidMode = LB3_HALF_RES_UNCOMPRESSED;
                lb3dataformat = LADYBUG_DATAFORMAT_SEQUENTIAL_HALF_HEIGHT;
                sprintf (configParams.basename, "HALF_RES_UNCOMPRESSED");
                /* width = 1616; height = 616; */
            }
            else {
                vidMode = LB3_FULL_RES_UNCOMPRESSED;
                lb3dataformat = LADYBUG_DATAFORMAT_SEQUENTIAL;
                sprintf (configParams.basename, "FULL_RES_UNCOMPRESSED");
                /* width = 1616; height = 1232; */
            }
        }
        else {    
            if (configParams.resolution == 0) {
                vidMode = LB3_HALF_RES_JPEG;
                lb3dataformat = LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_HALF_HEIGHT_JPEG; 
                sprintf (configParams.basename, "HALF_RES_JPEG");
                /* width = 1616; height = 616; */
            }
            else {
                vidMode = LB3_FULL_RES_JPEG;
                lb3dataformat = LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_JPEG;  
                sprintf (configParams.basename, "FULL_RES_JPEG");
                /* width = 1616; height = 1232; */
            }
        }
        
        lb3_set_video_mode (lb3cam, vidMode);
        
        //set the jpeg quality if recording in jpeg compression mode
        if(configParams.compression_mode)
            lb3_set_jpeg_quality (lb3cam, configParams.quality);
        
        //set the framerate
        lb3_set_frame_rate (lb3cam, configParams.framerate);
        lb3_embed_timestamp_into_frame (lb3cam);
        lb3_start_video_transmission (lb3cam);
        
        if(!configParams.auto_max_shutter)
	    lb3_set_max_shutter (lb3cam, configParams.max_shutter);

        int mask[6] = {1,1,1,1,1,0};
	lb3_set_ae_mask (lb3cam, mask); 

    }
    else {
        printf ("Failure\n");
        return 0;
    }
    
    LB3state.lb3cam = lb3cam;
        
    dc1394error_t status;
    status = dc1394_capture_dequeue (lb3cam->cam, DC1394_CAPTURE_POLICY_WAIT, &frame);
    //create the header
    LB3state.header.versionNum = 4;
    LB3state.header.framerate = configParams.framerate;
    LB3state.header.baseSerialNum = 8511563; //from the box
    LB3state.header.headSerialNum = 8511563;
    LB3state.header.paddingBlock = frame->padding_bytes;
    LB3state.header.dataFormat = lb3dataformat; 
    LB3state.header.resolution = LADYBUG_RESOLUTION_1616x1232;
    LB3state.header.bayerPattern = LADYBUG_RGGB;
    LB3state.header.configDataSize = configDataSize; //189740; //This is the size of the calib file
    LB3state.header.numImages = 1; //Dummy (This is updated when frames are written to the file)
    LB3state.header.numIndex_M = 0x01;
    LB3state.header.interval_K = 0x32;
    //0xBF0 is from the Ladybug Stream data format documentation
    LB3state.header.streamDataOffset = 0xBF0 + 0x16 + configDataSize; //0x0002f200; 
    LB3state.header.gpsDataOffset = 0;
    LB3state.header.gpsDataSize = 0;
    status = dc1394_capture_enqueue (lb3cam->cam, frame);
       
    //Initialize LB3state
    LB3state.circBufData = bot_circular_new (MAX_CIRC_BUF_SIZE, frame->total_bytes);
    LB3state.record_flag = 1;
    LB3state.configParams = configParams; 
    
    //Initialize the buffer ready condition
    pthread_cond_init (&LB3state.cond_buffer_ready, NULL);
    //Initialize the mutex
    pthread_mutex_init (&LB3state.mutex, NULL);

    //Create a write thread which writes data into disk.
    int iwret = 0; 
    pthread_t wthread; 
    iwret = pthread_create (&wthread, NULL, write_thread, (void*)&LB3state);
    if (iwret != 0)
        return 1;
       
    //FIXME
    //Create a lcm thread which publishes botsync on network
    int ilret;
    pthread_t lthread; 
    ilret = pthread_create (&lthread, NULL, lcm_thread, (void*)&LB3state);  
    if (ilret != 0)
        return 1;
 
    uint64_t utime_prev = timestamp_now ();
  
    //Grab frame and store them into a circular buffer
    while (g_record_flag) { 
       
	//Get frame from the ring buffer 
        status = dc1394_capture_dequeue (lb3cam->cam, DC1394_CAPTURE_POLICY_WAIT, &frame);
        DC1394_ERR_RTN (status,"capturing frame");
        uint64_t utime_curr = timestamp_now ();
 
        pthread_mutex_lock (&LB3state.mutex);
        // write the timestamp in the frame data itself
        uint64_t embeded_sync_time;
        lb3_get_sync_embedded_timestamp (lb3cam, frame, &embeded_sync_time);
        memcpy (frame->image, &embeded_sync_time, sizeof(uint64_t)); 
        memcpy (frame->image + sizeof(uint64_t), &frame->timestamp, sizeof(uint64_t)); 
        memcpy (frame->image + 2*sizeof(uint64_t), &utime_curr, sizeof(uint64_t)); 

        utime_curr = embeded_sync_time;
        //printf ("%lld %lld %lld\n", embeded_sync_time, dc1394_time, host_time);
        // write data into circular buffer 
        bot_circular_push_head (LB3state.circBufData, frame->image);  
        
        //Enqueue the frame to ring buffer need to do this once you have copied 
        //the frame data
        status = dc1394_capture_enqueue (lb3cam->cam, frame);
        DC1394_ERR_RTN (status,"releasing buffer");

        //If the queue was empty signal the write thread that there is data to read now
        if (bot_circular_size (LB3state.circBufData) == 1)
           pthread_cond_signal (&LB3state.cond_buffer_ready);

        pthread_mutex_unlock (&LB3state.mutex); 
         
        //checking the FPS
        const double alpha = 0.5;
        double dt = (utime_curr - utime_prev)*1e-6;
        fps = alpha*fps + (1-alpha)*1/dt;
        printf ("\rBuffer State = %d FPS = %f dt = %f", LB3state.circBufData->len, fps, dt); fflush (stdout);
        if (LB3state.circBufData->len == MAX_CIRC_BUF_SIZE) 
            printf ("\nOOPS !! Buffer is full. Will have to drop/overwrite the oldest frame in the buffer\n"); 

        utime_prev = utime_curr;
    }

    printf ("\nFinalizing......\n");
    while (g_exit_write_thread != 1)
        sleep (1);
    
    lb3_stop_video_transmission (lb3cam);
    lb3_camera_free (lb3cam);
    lcm_destroy (LB3state.lcm);
    printf ("\nDone \n"), fflush (stdout);
    return 0;
}
