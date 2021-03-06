/* View a camera LCM stream
 *
 * Christian Lees
 * ACFR
 * 19/12/13
 */
#include <lcm/lcm-cpp.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "perls-lcmtypes++/bot_core/image_t.hpp"
#include "perls-lcmtypes++/acfrlcm/compressed_image_t.hpp"
#include <signal.h>
#include <iostream>
#include <unistd.h>
#include <turbojpeg.h>

using namespace std;
using namespace bot_core;
using namespace cv;
using namespace acfrlcm;    

int isBayer;
int isCompressed;

void onImage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const image_t *lcmImage, lcm::LCM *lcm) {
    // extract the image and display it 
    Mat img(Size(lcmImage->width, lcmImage->height), CV_16U, (void *)&lcmImage->data[0]);
    if(isBayer)
        cvtColor(img, img, CV_BayerBG2BGR);
    imshow("LCMDisplay", img);
    waitKey(10);
}

void onCompressedImage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const compressed_image_t *lcmImage, lcm::LCM *lcm) {
    int width, height, jpeg_sub_samp;
    int ret;

    tjhandle jhandle = tjInitDecompress();
    ret = tjDecompressHeader2(jhandle, (unsigned char *)&lcmImage->image[0], lcmImage->size, &width, &height, &jpeg_sub_samp);
    if(ret == -1) {
        cerr << "Error: " << tjGetErrorStr() << endl;
        return;
    }
    if(lcmImage->is_rgb) {
        unsigned char *image = (unsigned char *)malloc(width * height * sizeof(unsigned char) * 3);
        ret = tjDecompress2(jhandle, (unsigned char *)&lcmImage->image[0], lcmImage->size, image, width, width * 3, height, TJPF_RGB, 0);
        if(ret == -1) {
            cerr << "Error: " << tjGetErrorStr() << endl;
            return;
        }
        Mat img(Size(width, height), CV_8UC3, (void *)image);    
        imshow("LCMDisplay", img);
        free(image);
    }
    else {
        unsigned char *image = (unsigned char *)malloc(width * height * sizeof(unsigned char));
        ret = tjDecompress2(jhandle, (unsigned char *)&lcmImage->image[0], lcmImage->size, image, width, width, height, TJPF_GRAY, 0);
        if(ret == -1) {
            cerr << "Error: " << tjGetErrorStr() << endl;
            return;
        }
        Mat img(Size(width, height), CV_8U, (void *)image);
        imshow("LCMDisplay", img);
        free(image);
    }
    
    waitKey(10);
    tjDestroy(jhandle);

}

int mainExit;
void signal_handler(int sig)
{
    mainExit = 1;
}

    
int main(int argc, char **argv) {
    if(argc < 2) {
        cerr << "Usage: cam-viewer <-b> <-c> [channel]" << endl;
        return 0;
    }
    isBayer = 0;
    isCompressed = 0;
    char opt;
    while((opt = getopt(argc, argv, "bc")) != -1) {
        if(opt == 'b') {
            isBayer = 1;
        }
        if(opt == 'c') {
            isCompressed = 1;
        }
    }

    cout << "isBayer: " << isBayer << " Channel: " << argv[optind] << endl;
    
    mainExit = 0;
    signal(SIGINT, signal_handler);
    
    lcm::LCM lcm;
    // subscribe to the image channel
    if(isCompressed)
        lcm.subscribeFunction(argv[optind], onCompressedImage, &lcm);
    else       
        lcm.subscribeFunction(argv[optind], onImage, &lcm);
        
    namedWindow("LCMDisplay", WINDOW_AUTOSIZE);
    
    int fd = lcm.getFileno();
    fd_set rfds;
    while(!mainExit)
    {
        FD_ZERO (&rfds);
        FD_SET (fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select (fd + 1, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
            lcm.handle();
    }
    
    return 1;
}
