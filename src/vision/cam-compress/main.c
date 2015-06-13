#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
//#include <jpeglib.h>
#include <turbojpeg.h>
#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-common/lcm_util.h"
#include "perls-lcmtypes/acfrlcm_compressed_image_t.h"
#include "bayer.h"

typedef struct
{
    lcm_t *lcm;
    int count;
    int is_bayer;
    char *out_channel;
    int quality;
    int rate;
} state_t;

int program_exit;
#define MAX_BUFFER_SIZE 1024*1024*10
static void
bot_core_image_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                           const bot_core_image_t *image, void *user)
{
    state_t *state = user;
    if(state->count < state->rate)
        state->count++;
    else
    {

//        printf("Image: W=%d H=%d RS=%d PF=%d S=%d\n", image->width, image->height, image->row_stride, image->pixelformat, image->size);

        acfrlcm_compressed_image_t cimage;     // the structure to send
        int step = 1;
        int output_width = image->width / step;
        int output_height =image->height / step;
        short *image_buffer;
        unsigned char *image_8bit = NULL, *image_8bit_rgb = NULL, *image_8bit_out = NULL;
        unsigned char *output_buffer = NULL;
        unsigned long output_size;



        // Convert to 8bit as all JPEGs are 8 bit, scale at the same time
        image_buffer = (short *)image->data;
        image_8bit = (unsigned char *)malloc(output_width * output_height);
        int count = 0;
        for(int i=0; i<image->height; i+=step)
            for(int j=0; j<image->width; j+=step)
                image_8bit[count++] = image_buffer[j + i * image->width] >> 8;


        int pixel_format;

        // If the imag is Bayer we need to convert it to RGB
        if(state->is_bayer)
        {
            // Convert to RGB
            image_8bit_rgb = (unsigned char *)malloc(output_width * output_height * 3);
            gp_bayer_decode(image_8bit, output_width, output_height, image_8bit_rgb, BAYER_TILE_RGGB);
            pixel_format = TJPF_RGB;
            cimage.is_rgb = 1;
            image_8bit_out = image_8bit_rgb;
        }
        else
        {
            pixel_format = TJPF_GRAY;
            cimage.is_rgb = 0;
            image_8bit_out = image_8bit;
        }

        tjhandle jpeg_compress = tjInitCompress();
        tjCompress2(jpeg_compress, image_8bit_out, output_width, 0, output_height,
                    pixel_format,
                    &output_buffer, &output_size, TJSAMP_444, state->quality,
                    TJFLAG_FASTDCT);

        // Fill in the rest of the data and send it on it's way
        cimage.utime = image->utime;
        cimage.size = output_size;
        cimage.image = output_buffer;
        acfrlcm_compressed_image_t_publish(state->lcm, state->out_channel, &cimage);

        free(image_8bit);
        if(image_8bit_rgb != NULL)
            free(image_8bit_rgb);
        tjDestroy(jpeg_compress);
        tjFree(output_buffer);

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

    if(argc < 2)
    {
        printf("Usage: cam-compress <-b> <-q quality> [in channel] [out channel]\n");
        printf("   -b  Bayer image\n");
        printf("   -q  JPEG compression quality 0 - 100, default 25\n");
        return 0;
    }

    state.is_bayer = 0;
    state.quality = 25;
    state.rate = 10;
    char opt;
    while((opt = getopt(argc, argv, "r:bq:")) != -1)
    {
        switch(opt)
        {
        case 'b':
            state.is_bayer = 1;
            break;
        case 'q':
            state.quality = atoi(optarg);
            break;
        case 'r':
            state.rate = atoi(optarg);
            break;
        }
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

    // loop and hadle LCM
    while (!program_exit)
    {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        lcmu_handle_timeout(state.lcm, &tv);
    }
    return 0;
}

