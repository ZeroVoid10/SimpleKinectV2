#include "kinectgrabber.h"


#include <fstream>
#include <ctime>
#include <iostream>

using namespace zerovoid;

KinectGrabber::KinectGrabber() : 
    listener(libfreenect2::Frame::Color|libfreenect2::Frame::Depth|libfreenect2::Frame::Ir),
    shutdown(false),
    undistorted(512, 424, 4),
    registered(512, 424, 4),
    depth2rgb(1920, 1082, 4) {
    serial = "";
    dev = nullptr;
    pipeline = nullptr;
}

bool KinectGrabber::start(void) {
    if (prepare() == false) {
        return false;
    }
    if (!dev->start()) {
        return false;
    }
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    registration = new libfreenect2::Registration(dev->getIrCameraParams(),
                                                  dev->getColorCameraParams());
    return true;
}

void KinectGrabber::close(void) {
    dev->stop();
    dev->close();
    delete registration;
}

void KinectGrabber::closeCheck(void) {
    int key = cv::waitKey(1);
    shutdown = shutdown || (key > 0 &&((key & 0xff) == 27));
    listener.release(frames);
}

bool KinectGrabber::waitNewFrame(int sec) {
    if (!listener.waitForNewFrame(frames, sec*1000)) {
        std::cout << "timeout!" << std::endl;
        return false;
    }
    return true;
}

void KinectGrabber::getFrames(cv::Mat* colorMat, cv::Mat* depthMat, cv::Mat* irMat) {
    libfreenect2::Frame *rgb = nullptr, *depth = nullptr, *ir = nullptr;
    if (colorMat != nullptr) {
        rgb = frames[libfreenect2::Frame::Color];
        cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC4, rgb->data).copyTo(*colorMat);
    }
    if (depthMat != nullptr) {
        depth = frames[libfreenect2::Frame::Depth];
        cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data).copyTo(*depthMat);
    }
    if (irMat != nullptr) {
        ir = frames[libfreenect2::Frame::Ir];
        cv::Mat((int)ir->height, (int)ir->width, CV_32FC1, ir->data).copyTo(*irMat);
    }
    if (colorMat != nullptr && depthMat != nullptr) {
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
    }
}

bool KinectGrabber::prepare(void) {
    if (nect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return false;
    }
    if (serial == "") {
        serial = nect2.getDefaultDeviceSerialNumber();
    }
    pipeline = new libfreenect2::OpenGLPacketPipeline();
    dev = nect2.openDevice(serial);
    dev->setIrAndDepthFrameListener(&listener);
    dev->setColorFrameListener(&listener);
    return true;
}

// Kinect Logger 
KinectLogger::KinectLogger() {
    std::time_t t = std::time(nullptr);
    std::tm *p = std::localtime(&t);
    std::string filename = std::to_string(p->tm_year - 100) +
                           (((p->tm_mon + 1) < 10)? "0":"") + (std::to_string(p->tm_mon + 1)) +
                           ((p->tm_mday < 10)? "0":"") + std::to_string(p->tm_mday) + "-" +
                           ((p->tm_hour < 10)? "0":"") + std::to_string(p->tm_hour) + ":" +
                           ((p->tm_min < 10)? "0":"") + std::to_string(p->tm_min) + ":" +
                           ((p->tm_sec < 10)? "0":"") + std::to_string(p->tm_sec);
    logfile_.open("./log/" + filename + ".log");
    level_ = Debug;
}

bool KinectLogger::good(void) {
    return logfile_.is_open() && logfile_.good();
}

void KinectLogger::log(Level level, const std::string &mes) {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "]" << mes << std::endl;
}

bool KinectLogger::start(void) {
    if (good()) {
        libfreenect2::setGlobalLogger(this);
    } else {
        // delete this; // 语法ok, 但不知道这样安不安全
        return false;
    }
    return true;
}
