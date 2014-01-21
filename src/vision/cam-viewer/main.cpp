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
#include <signal.h>
#include <iostream>
#include <unistd.h>

using namespace std;
using namespace bot_core;
using namespace cv;

int isBayer;

void onImage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const image_t *lcmImage, lcm::LCM *lcm) {
    // extract the image and display it 
    Mat img(Size(lcmImage->width, lcmImage->height), CV_16U, (void *)&lcmImage->data[0]);
    if(isBayer)
        cvtColor(img, img, CV_BayerBG2BGR);
    imshow("LCMDisplay", img);
    waitKey(10);
}


int mainExit;
void signal_handler(int sig)
{
    mainExit = 1;
}

    
int main(int argc, char **argv) {
    if(argc < 2) {
        cerr << "Usage: cam-viewer <-b> [channel]" << endl;
        return 0;
    }
    int opts = 0;
    isBayer = 0;
    char opt;
    while((opt = getopt(argc, argv, "b")) != -1) {
        if(opt == 'b') {
            isBayer = 1;
            opts++;
        }
    }

    cout << "isBayer: " << isBayer << " Channel: " << argv[optind] << endl;
    
    mainExit = 0;
    signal(SIGINT, signal_handler);
    
    lcm::LCM lcm;
    // subscribe to the image channel
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
