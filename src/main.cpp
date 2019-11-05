#include "kinectgrabber.h"

using namespace zerovoid;
int main(int argc, char *argv[])
{
    KinectLogger *testlogger = new KinectLogger();
    if (testlogger->start() != true) {
        delete testlogger;
    }
    KinectGrabber kinect;
    if (kinect.start() == false) {
        return -1;
    }
    cv::Mat colorMat, depthMat, irMat;
    cv::Mat blurMat, bilMat, bmMat;
    while(kinect.shutdown == false) {
        if (kinect.waitNewFrame(10) == false) {
            break;
        }
        kinect.getFrames(&colorMat, &depthMat, nullptr);
        cv::medianBlur(depthMat, depthMat, 5);
        cv::blur(depthMat, blurMat, cv::Size(3,3));
        cv::bilateralFilter(depthMat, bilMat, 5, 10, 10, cv::BORDER_DEFAULT);
        //cv::bilateralFilter(bmMat, bmMat, 5, 10, 10, cv::BORDER_DEFAULT);
        cv::imshow("dep", depthMat / 4096.0f);
        cv::imshow("blur", blurMat / 4096.0f);
        cv::imshow("bi", bilMat / 4096.0f);
        //cv::imshow("bm", bmMat / 4096.0f);
        //cv::imshow("ir", irMat / 4096.0f);
        kinect.closeCheck();
    }
    kinect.close();
    return 0;
}
