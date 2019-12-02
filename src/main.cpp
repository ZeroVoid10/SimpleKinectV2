#include "kinectgrabber.h"

using namespace zerovoid;
int fast_arg = 10;
cv::Ptr<cv::FastFeatureDetector> ptrFAST = cv::FastFeatureDetector::create(fast_arg);
cv::Ptr<cv::ORB> ptrORB = cv::ORB::create(fast_arg, 1.2, 8);
void on_fast_arg_change(int, void*);

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
    cv::Mat blurMat, bilMat, bmMat;
    cv::Mat grayMat, keyMat;
    int arg = 20;
    std::vector<cv::KeyPoint> keyPoints;
    cv::namedWindow("key", 0);
    cv::createTrackbar("fast_arg", "key", &fast_arg, 100, on_fast_arg_change);
    //cv::Ptr<cv::FastFeatureDetector> ptrFAST = cv::FastFeatureDetector::create(fast_arg);
    while(kinect.shutdown == false) {
        if (kinect.waitNewFrame(10) == false) {
            break;
        }
        kinect.getFrames(&colorMat, &depthMat, nullptr);
        cv::flip(colorMat, colorMat, 1);
        cv::cvtColor(colorMat, grayMat, cv::COLOR_RGB2GRAY);
        cv::cvtColor(colorMat, keyMat, cv::COLOR_RGB2GRAY);
        //ptrORB->detect(grayMat, keyPoints);
        cv::medianBlur(depthMat, depthMat, 5);
        cv::blur(depthMat, blurMat, cv::Size(3,3));
        cv::bilateralFilter(depthMat, bilMat, 5, 10, 10, cv::BORDER_DEFAULT);

        ptrFAST->detect(keyMat, keyPoints);
        cv::drawKeypoints(keyMat, keyPoints, keyMat, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
        //cv::bilateralFilter(bmMat, bmMat, 5, 10, 10, cv::BORDER_DEFAULT);
        //cv::imshow("dep", depthMat / 4096.0);
        cv::imshow("gray", grayMat);
        cv::imshow("key", keyMat);
        //cv::imshow("blur", blurMat / 4096.0f);
        //cv::imshow("bi", bilMat / 4096.0f);
        //cv::imshow("bm", bmMat / 4096.0f);
        //cv::imshow("ir", irMat / 4096.0f);
        kinect.closeCheck();
    }
    kinect.close();
    return 0;
}

void on_fast_arg_change(int arg, void*) {
    ptrFAST->setThreshold(arg);
    ptrORB->setFastThreshold(arg);
}
