#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
//#include <turbojpeg.h>
#include <jpeglib.h>
//#include <opencv2/core/core_c.h>
//#include <opencv2/imgproc/imgproc_c.h>
//#include <opencv2/highgui/highgui_c.h>
#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-vision/botimage.h"
#include "perls-common/lcm_util.h"
#include "perls-lcmtypes/acfrlcm_compressed_image_t.h"

typedef struct
{
    lcm_t *lcm;
    int count;
    int is_bayer;
    char *out_channel;
    int quality;
} state_t;

int program_exit;
#define MAX_BUFFER_SIZE 1024*1024*10
static void
bot_core_image_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                         const bot_core_image_t *image, void *user)
{
    state_t *state = user;
    if(state->count < 9)
        state->count++;
    else
    {
    
//        printf("Image: W=%d H=%d RS=%d PF=%d S=%d\n", image->width, image->height, image->row_stride, image->pixelformat, image->size);
    
         acfrlcm_compressed_image_t cimage;     // the structure to send
         int step = 4;
         int output_width = image->width / step; 
         int output_height =image->height / step;
         short *image_buffer;
         unsigned char *image_8bit;
         unsigned char *output_buffer = NULL;
            unsigned long output_size;

         // Using libjpeg
        struct jpeg_compress_struct cinfo = {0};
        struct jpeg_error_mgr jerr;
            
//         tjhandle jhandle = tjInitCompress();
//         if(jhandle == NULL) 
//            printf("Error creating Turbo JPEG compressor\n");
//         else
/*         {
            // first if its a bayer camera we want to turn it into an RGB image first
            short *image_buffer;
            unsigned char *image_8bit;
            unsigned char *output_buffer = NULL;
            unsigned long output_size;
            int output_width = 640, output_height = 480;
            
            IplImage *img = cvCreateImageHeader(cvSize(image->width, image->height), IPL_DEPTH_16U, 1);
            cvSetData(img, image->data, CV_AUTOSTEP);
            IplImage *img_resized;
*/
            

/*            
            if(state->is_bayer)
            {
               // Convert to RGB
               IplImage *img_rgb = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_16U, 3);
               cvCvtColor(img, img_rgb, CV_BayerBG2RGB);
               // Resize to 640 x 480
               img_resized = cvCreateImage(cvSize(output_width, output_height), IPL_DEPTH_16U, 3);
               cvResize(img_rgb, img_resized, CV_INTER_LINEAR);
               image_buffer = (short *)img_resized->imageData;                             // pointer to the first element
               image_8bit = (unsigned char *)malloc(output_width * output_height * 3);     // array for 8 bit version
               for(int i=0; i<(output_width * output_height * 3); i++)
                  image_8bit[i] = image_buffer[i] >> 8;

               // Using libjpeg
               cinfo.in_color_space = JCS_RGB;
                 
//               int ret = tjCompress2(jhandle, image_8bit, output_width, output_width * 3, output_height, TJPF_RGB, &output_buffer, &output_size, TJSAMP_422, state->quality, 0);  
//               if(ret == -1)
//                  printf("Error: %s\n", tjGetErrorStr());
                    
               cimage.is_rgb = 1;
               cvReleaseImage(&img_rgb);
                
            }
            else
            {    
               img_resized = cvCreateImage(cvSize(output_width, output_height), IPL_DEPTH_16U, 1);
               cvResize(img, img_resized, CV_INTER_LINEAR);
               // first convert the image to 8-bit as all jpegs are 8-bit
*/               image_buffer = (short *)image->data;
             //image_buffer = (short *)img_resized->imageData;
               image_8bit = (unsigned char *)malloc(output_width * output_height);
               //for(int i=0; i<(output_width * output_height); i++)
               //   image_8bit[i] = image_buffer[i] >> 8;
               int count = 0;
                for(int i=0; i<image->height; i+=step)
                    for(int j=0; j<image->width; j+=step)
                        image_8bit[count++] = image_buffer[j + i * image->width] >> 8;
               

               
               // Using turbojpeg            
//               int ret = tjCompress2(jhandle, image_8bit, output_width, output_width, output_height, TJPF_GRAY, &output_buffer, &output_size, TJSAMP_GRAY, state->quality, 0);  
//               if(ret == -1)
//                  printf("Error: %s\n", tjGetErrorStr());
                    
               cimage.is_rgb = 0;
            //}

            // compress the 8 bit image usinf libjpeg
            JSAMPROW row_ptr[1];
            int row_stride;
            cinfo.err = jpeg_std_error(&jerr);
            jpeg_create_compress(&cinfo);
            jpeg_mem_dest(&cinfo, &output_buffer, &output_size);
            
            cinfo.image_width = output_width; //img_resized->width;               
            cinfo.image_height = output_height; //img_resized->height;
            cinfo.input_components = 1; //img_resized->nChannels;
            
            jpeg_set_defaults(&cinfo);
            jpeg_set_quality(&cinfo, state->quality, 1);
            jpeg_start_compress(&cinfo, 1);
            row_stride = output_width; //img_resized->width * img_resized->nChannels;

            while (cinfo.next_scanline < cinfo.image_height) {
                row_ptr[0] = &image_8bit[cinfo.next_scanline * row_stride];
                jpeg_write_scanlines(&cinfo, row_ptr, 1);
            }

            jpeg_finish_compress(&cinfo);
            jpeg_destroy_compress(&cinfo);


                    
            // Fill in the rest of the data and send it on it's way
            cimage.utime = image->utime;
            cimage.size = output_size;
            cimage.image = output_buffer;
            acfrlcm_compressed_image_t_publish(state->lcm, state->out_channel, &cimage);
            
            // Clean up
//            tjFree(output_buffer);
//            tjDestroy(jhandle);
            free(image_8bit);
//            cvReleaseImage(&img_resized);
//            cvReleaseImageHeader(&img);
//       } 

        state->count = 0;      
    }
    
    return;
}

void signal_handler(int sigNum) 
{
    // do a safe exit
    program_exit = 1;
}

int main(int argc, char **argv)
{
    state_t state;
    
    if(argc < 2) {
        printf("Usage: cam-compress <-b> <-q quality> [in channel] [out channel]\n");
        printf("   -b  Bayer image\n");
        printf("   -q  JPEG compression quality 0 - 100, default 25\n");
        return 0;
    }

    state.is_bayer = 0;
    state.quality = 25;    
    char opt;
    while((opt = getopt(argc, argv, "bq:")) != -1) {
        if(opt == 'b')
            state.is_bayer = 1;
        if(opt == 'q') 
            state.quality = atoi(optarg);
    }
    state.out_channel = argv[optind + 1];

    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);
    
    // start lcm
    
    state.lcm = lcm_create(NULL);
        
    state.count = 0;
    printf("Subscribing to channel %s\n", argv[optind]);
    printf("Publishing to channel %s\n", state.out_channel);
    
    // subscribe to the image channel
    bot_core_image_t_subscribe(state.lcm, argv[optind], &bot_core_image_t_callback, &state);
    
//    cvNamedWindow( "CompressDebug", CV_WINDOW_AUTOSIZE);
    
    // loop and hadle LCM
    while (!program_exit) {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        lcmu_handle_timeout(state.lcm, &tv);
    }
    return 0;
}       
    
