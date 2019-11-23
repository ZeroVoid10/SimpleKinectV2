#include "kinectgrabber.h"

// ORB SLAM 2
#include <iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

using namespace zerovoid;

int main(int argc, char *argv[]) {
    KinectLogger *testlogger = new KinectLogger();    
    if (testlogger->start() != true) {
        delete testlogger;
    }
    KinectGrabber kinect;
    if (kinect.start() == false) {
        return -1;
    }
    cv::Mat colorMat, depthMat, irMat;
    while(kinect.shutdown == false) {
        if (kinect.waitNewFrame(10) == false) {
            break;
        }
        kinect.getFrames(&colorMat, &depthMat, nullptr);
        cv::imshow("color", colorMat);
        cv::imshow("dep", depthMat / 4096.0f);
        //cv::imshow("ir", irMat / 4096.0f);
        kinect.closeCheck();
    }
    kinect.close();
    return 0;
}
