#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <opencv/cv.h>

#include "perls-lcmtypes/bot_core_image_t.h"

#include "perls-vision/ladybug3_util.h"
#include "perls-vision/botimage.h"
#include "perls-common/error.h"

typedef struct _InputParams InputParams_t;
struct _InputParams {
    char *folder;
    char *basename;
    int num_files;
    int overwrite;
    unsigned int compression;
};

void
print_help (int exval, char **argv) 
{
    printf ("Usage:%s [-h] [-f FOLDER]\n\n", argv[0]);
    printf ("  -h         print this help and exit\n");
    printf ("  -f FOLDER  set folder where data is logged\n");
    printf ("  -o         overwrite files with same name\n");
    printf ("  -c         TIFF file compression scheme {none,lzw,deflate,jpeg,jpeg:quality}\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, InputParams_t *inputParams)
{
    int opt;
    //Initialize the global parameters to default values
    inputParams->folder = NULL;
    inputParams->basename = NULL;
    inputParams->num_files = 1;
    inputParams->overwrite = 0;

    const char *compression_str = NULL;
    
    while ((opt = getopt (argc, argv, "hof:c:")) != -1) {
        switch (opt) {
            case 'h':
                print_help (0, argv);
                break;
            case 'o':
                inputParams->overwrite = 1;
                break;
            case 'f':
                //get the folder
                inputParams->folder = (char *)optarg;
                break;
	    case 'c':
                //get the compression string
		compression_str = (char *)optarg;
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
    
    unsigned int quality = 95;
    if (compression_str ==  NULL || 0 == strcasecmp (compression_str, "none"))
        inputParams->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_NONE;
    else if (0 == strcasecmp (compression_str, "lzw"))
        inputParams->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_LZW;
    else if (0 == strcasecmp (compression_str, "deflate"))
        inputParams->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_DEFLATE;
    else if (0 == strcasecmp (compression_str, "jpeg"))
        inputParams->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG | quality;
    else if (0 == strncasecmp (compression_str, "jpeg:", 5) &&
             1 == sscanf (compression_str, "jpeg:%u", &quality) &&
             0 < quality && quality <= 100)
        inputParams->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG | quality;
    else {
        fprintf (stderr, "unrecognized compression format\n");
        exit (EXIT_FAILURE);
    }
    
    if (inputParams->folder == NULL) {
        printf ("Enter folder path where data is logged\n");
        print_help (1, argv);
    }
    
    /* 
    // print all remaining options
    */
    for (; optind < argc; optind++)
        printf ("argument: %s\n", argv[optind]);
    
}

int
file_exists (char *fileName)
{
    struct stat buf;
    int i = stat (fileName, &buf);

    if (i == 0)
        return 1;
    else
        return 0;
}

/**
 * This function reads the uncompressed image data from the file
 */
int
save_uncompressed_color_image_data (FILE *fp, LB3StreamHeaderInfo_t *header, InputParams_t *inputParams, int file_index)
{
    int width = -1, height = -1;
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
        return -1;
    }
    
    //Allocate memory for the data
    int size = width*height*6 + header->paddingBlock;
    unsigned char *buffer = (unsigned char *) malloc (size);

    //read buffer from file
    //read timestamp
    int bytes_read = fread (buffer, size, 1, fp);    
    if (bytes_read == 0) {
        printf ("Error 0 bytes read\n");
        return -1;
    }
    
    //Get timestamp
    uint64_t timestamp;
    memcpy (&timestamp, buffer, sizeof(uint64_t));          
   
    uint8_t *rgbbuffer; // = (unsigned char *) malloc (size*3);  
    // Color Conversion assuming bayer pattern RGGB
    IplImage *src = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 1);
    src->imageData = (char *) buffer;
    IplImage *dst = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 3);
    cvCvtColor (src, dst, CV_BayerBG2RGB);
    rgbbuffer = (uint8_t *) dst->imageData;
    //vis_bayer_edge_sense (buffer, rgbbuffer, width, height*6, VIS_BAYER_PATTERN_RGGB);

    //Write the color image corresponding to each camera to a file under the 
    //respective folder
    for (int i = 0; i < 6; i++) {
        char imageName[1024];
        sprintf (imageName,"%s/Cam%d/%"PRId64".tiff", inputParams->folder, i, timestamp);
        //sprintf (imageName,"%s/Cam%d/%s-%06d.ppm", inputParams->folder, i, inputParams->basename, file_index);
        if (!file_exists (imageName) || inputParams->overwrite) {
            
	    // old ppm code
	    //FILE *imagefile = fopen (imageName, "w");
            //fprintf (imagefile, "P6\n%u %u 255\n", width, height);
            //fwrite (rgbbuffer, width*height, 3, imagefile);
            //rgbbuffer = rgbbuffer + 3*width*height;
            //fclose (imagefile);
	    
	    bot_core_image_t img;
	    img.utime = timestamp;
	    img.width = width;
	    img.height = height;
	    img.row_stride = width*3.0;
	    img.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
	    img.size = width*height*3;
	    img.data = rgbbuffer;
	    img.nmetadata = 0;
	    img.metadata = NULL;

	    if (0 != vis_botimage_write_tiff (&img, imageName, NULL, NULL, inputParams->compression)) {
		ERROR ("unable to write file %s", imageName);
	    }
	    
        }
    }
 
    rgbbuffer = rgbbuffer - 6*3*width*height;
    //delete the buffers
    free (buffer);
    //free (rgbbuffer);
    cvReleaseImage (&src);
    cvReleaseImage (&dst);
    return 0; 
}

/**
 * This function reads the jpeg compressed image data from the file
 */
void
decompress_jpeg_and_save_color_image_data (FILE *fp, LB3StreamHeaderInfo_t *header, InputParams_t *inputParams, int file_index)
{
    int width, height, decompressed_width, decompressed_height;
    if (header->dataFormat == LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_JPEG) {
        decompressed_width = 1616; 
        decompressed_height = 1232; 
        width = 808; height = 616; 
    }
    else if (header->dataFormat == LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_HALF_HEIGHT_JPEG) {
        decompressed_width = 1616; 
        decompressed_height = 616;
        width = 808; height = 308; 
    }
    //FIXME
    LB3JpegHeaderInfo_t jpegHeader;
    //read jpeg header
    lb3_read_jpeg_header (fp, &jpegHeader);    
    
    //read jpeg blocks
    unsigned char *decompressed_bayer[6][4];
    unsigned char *bayer[6][4];
    for (int cam = 0; cam < 6; cam++) {
        for (int k = 0; k < 4; k++) {
            bayer[cam][k] = (unsigned char *) malloc (jpegHeader.jpegDataSize[cam][k]);
            decompressed_bayer[cam][k] = malloc (width*height); 
        }
    }

 
    for (int cam = 0; cam < 6; cam++) {
        for (int k = 0; k < 4; k++) {
            //Read compressed bayer pattern
            fseek (fp, jpegHeader.jpegDataOffset[cam][k], SEEK_SET);
            fread (bayer[cam][k], jpegHeader.jpegDataSize[cam][k], 1, fp);
            //decompress bayer pattern
            lb3_decompress_jpeg (bayer[cam][k], jpegHeader.jpegDataSize[cam][k], decompressed_bayer[cam][k], width, height); 
        }
    }
     
    //perform bayer conversion to get the color image
    for (int cam = 0; cam < 6; cam++) {

	char imageName[1024];
        //sprintf(imageName,"%s/Cam%d/%s-%06d.ppm", inputParams->folder, cam, inputParams->basename, file_index);
        sprintf (imageName,"%s/Cam%d/%"PRId64".tiff", inputParams->folder, cam, jpegHeader.timestamp);
        if (file_exists (imageName) & !inputParams->overwrite) {
            printf ("Skipping Cam %d\n", cam);
	    continue;
        }


         //create a bayer patter buffer from these blocks
        unsigned char *bayer_buffer = malloc (decompressed_width*decompressed_height);
        uint8_t *rgb_buffer; // = malloc (decompressed_width*decompressed_height*3);
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
        //vis_bayer_edge_sense (bayer_buffer, rgb_buffer, decompressed_width, decompressed_height, VIS_BAYER_PATTERN_RGGB);
        IplImage *src = cvCreateImage (cvSize (decompressed_width, decompressed_height), IPL_DEPTH_8U, 1);
        src->imageData = (char *) bayer_buffer;
        IplImage *dst = cvCreateImage (cvSize (decompressed_width, decompressed_height), IPL_DEPTH_8U, 3);
        cvCvtColor (src, dst, CV_BayerBG2RGB);
        rgb_buffer = (uint8_t *) dst->imageData;
        
	//save the image 
        //FILE* imagefile = fopen (imageName, "w");
        //fprintf (imagefile, "P6\n%u %u 255\n", decompressed_width, decompressed_height);
        //fwrite (rgb_buffer, decompressed_width*decompressed_height, 3, imagefile);
        //fclose (imagefile);
	bot_core_image_t img;
	img.utime = jpegHeader.timestamp;
	img.width = decompressed_width;
	img.height = decompressed_height;
	img.row_stride = decompressed_width*3.0;
	img.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
	img.size = decompressed_width*decompressed_height*3;
	img.data = rgb_buffer;
	img.nmetadata = 0;
	img.metadata = NULL;

	if (0 != vis_botimage_write_tiff (&img, imageName, NULL, NULL, inputParams->compression)) {
	    ERROR ("unable to write file %s", imageName);
	}
	
	printf ("Done Cam %d\n", cam);
        free (bayer_buffer);
        //free (rgb_buffer);
        cvReleaseImage (&src);
        cvReleaseImage (&dst);
    }

    //Free memory
    for (int cam = 0; cam < 6; cam++) {
        for (int k = 0; k < 4; k++) {
            free (bayer[cam][k]);
            free (decompressed_bayer[cam][k]);
        }
    }
    printf ("Memory released \n");
    fflush (stdout);
}

int
main (int argc, char **argv)
{
    InputParams_t inputParams;
    parse_args (argc, argv, &inputParams);
   
    //Create 6 sub-folders to save individual images from each Cam
    for (int i = 0; i < 6; i++) {
        char cmd[1024];
        sprintf (cmd, "mkdir %s/Cam%d", inputParams.folder, i);
        system (cmd);
    }

    DIR *d;
    struct dirent *dir;
    d = opendir (inputParams.folder);
    if (d == NULL) 
        return 1;
 
    //Loop over all files
    //Find the number of .pgr files in the folder 
    int num_images = 0;
    while ((dir = readdir (d))) {
        
        int len = strlen (dir->d_name);
        if (strcmp (dir->d_name, "." ) == 0 || strcmp (dir->d_name, ".." ) == 0
            || dir->d_type == DT_DIR || strcmp (&(dir->d_name[len-4]), ".pgr") != 0) {
            printf ("Skipping: %s \n", dir->d_name);
            continue;
        }
 
        //Read ith file
        char filename[1024];
        sprintf (filename, "%s/%s", inputParams.folder, dir->d_name);
        printf ("Now Reading %s\n", filename);
        
        FILE *fptr = fopen (filename, "rb");
        if (fptr == NULL) {
            printf ("Error reading the logfile: %s\n", filename);
            return -1;
        } 
        
        //read header
        LB3StreamHeaderInfo_t header;
        lb3_read_stream_header (fptr, &header);

        //read calibration data
        lb3_read_calib_data (fptr, &header, inputParams.folder); 
        
        //Read image data and save it as color images
        for (int j = 0; j < header.numImages; j++) {
            if (header.dataFormat == LADYBUG_DATAFORMAT_SEQUENTIAL || 
                header.dataFormat == LADYBUG_DATAFORMAT_SEQUENTIAL_HALF_HEIGHT) {
                // Perform Bayer color conversion and save raw data as ppm files
                //printf ("\rNow Saving image %d", num_images); fflush(stdout); 
                save_uncompressed_color_image_data (fptr, &header, &inputParams, num_images);
                //num_images++;
             } 
             else {
                //Read jpeg compressed data
                //Decompress the data
                //Do bayer conversion
                //Save uncompressed data as raw ppm   
                decompress_jpeg_and_save_color_image_data (fptr, &header, &inputParams, num_images);
                //num_images++;
             }
        }
        fclose (fptr); 
    } 
    return 0;
}
