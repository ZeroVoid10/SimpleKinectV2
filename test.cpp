#include <iostream>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#include <fstream>
#include <cstdlib>

// custom logger
class TestLogger: public libfreenect2::Logger {
    private:
        std::ofstream logfile_;
    public:
        TestLogger(const char *filename) {
            if (filename) {
                logfile_.open(filename);
            }
            level_ = Debug;
        }
        bool good() {
            return logfile_.is_open() && logfile_.good();
        }
        virtual void log(Level level, const std::string &mes) {
            logfile_ << "[" << libfreenect2::Logger::level2str(level) << "]" << mes << std::endl;
        }
};

int main(int argc, char *argv[])
{
    // file logger
    TestLogger *testlogger = new TestLogger("./log/test.log");
    if (testlogger->good()) {
        libfreenect2::setGlobalLogger(testlogger);
    } else {
        delete testlogger;
    }

    // Configuration
    bool enable_rgb = true, enable_depth = true;
    
    // Initialize and Discover Devices
    libfreenect2::Freenect2 nect2;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::PacketPipeline *pipeline = nullptr;
    std::string serial = "";

    if (nect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    if (serial == "") {
        serial = nect2.getDefaultDeviceSerialNumber();
    }

    // Open and Configure the Device
    int types = libfreenect2::Frame::Color |  libfreenect2::Frame::Depth | libfreenect2::Frame::Ir;
    //int types = libfreenect2::Frame::Depth | libfreenect2::Frame::Ir;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

    pipeline = new libfreenect2::OpenGLPacketPipeline();
    dev = nect2.openDevice(serial);
    dev->setIrAndDepthFrameListener(&listener);
    dev->setColorFrameListener(&listener);

    // Start the Device
    if (enable_rgb && enable_depth) {
        if (!dev->start()) {
            return -1;
        }
    } else {
        if (!dev->startStreams(enable_rgb, enable_depth)) {
            std::cout << "start stream error" << std::endl;
            return -1;
        }
    }

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    // Receive Image Frames
    libfreenect2::Registration *registration = 
                new libfreenect2::Registration(dev->getIrCameraParams(),dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512,424, 4), depth2rgb(1920, 1080+2, 4);
    cv::Mat depthMat, irMat, colorMat, unidisMat, rgbd, rgbd2;
    cv::Mat cDepthMat, blurDepthMat, lastMat, deltaMat;
    cv::Mat cDepthMat2, outlineMat, mask;
    cv::Mat resBinMat = (cv::Mat_<uchar>(424, 512));
    bool protonect_shutdown = false, first_frame = true;
    float x,y,z, color;
    cv::Vec3b red;
    red[0] = 0;
    red[1] = 0;
    red[2] = 255;

    cv::VideoWriter writer;
    cv::Mat ker = (cv::Mat_<float>(5,5)<< 1/32,1/32,1/32,1/32,1/32, 
                                          1/32,1/8,1/8,1/8,1/32, 
                                          1/32,1/8,0.1,1/8,1/32, 
                                          1/32,1/8,1/8,1/8,1/32, 
                                          1/32,1/32,1/32,1/32,1/32);
    /*
    writer.open("test_video.mp4", 
                //cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
                cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
                30.0, 
                cv::Size(512, 424), 
                true);
                */
    while(!protonect_shutdown) {
        if (!listener.waitForNewFrame(frames, 10*1000)) {
            std::cout << "timeout!" << std::endl;
            return -1;
        }
        //listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC4, rgb->data).copyTo(colorMat);
        //cv::Mat((int)ir->height, (int)ir->width, CV_32FC1, ir->data).copyTo(irMat);
        cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data).copyTo(depthMat);
        cv::blur(depthMat, deltaMat, cv::Size(3,3));
        cv::medianBlur(depthMat, blurDepthMat, 5);
        cv::GaussianBlur(blurDepthMat, blurDepthMat, cv::Size(3,3), 1.5);
        cv::Sobel(blurDepthMat, outlineMat, 
                  CV_32FC1, 1, 1, 3, 0.4, 128);
        //cv::GaussianBlur(blurDepthMat, blurDepthMat, cv::Size(3, 3), 2);
        outlineMat = blurDepthMat - outlineMat;
        //cv::medianBlur(rgbd, rgbd, 5);
        cv::cvtColor(depthMat / 4096.0f, cDepthMat, cv::COLOR_GRAY2RGB);
        if (first_frame) {
            first_frame = false;
            blurDepthMat.copyTo(lastMat);
            //lastMat = depthMat;
        }
        deltaMat = lastMat - blurDepthMat;
        //cv::blur(deltaMat, deltaMat, cv::Size(5,5));
        blurDepthMat.copyTo(lastMat);

        cDepthMat.copyTo(cDepthMat2);
        cv::Mat resMat;
        cDepthMat.copyTo(resMat);
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        for (int col = 0; col < depth->width; ++col) {
            for (int row = 0; row < depth->height; ++row) {
                registration->getPointXYZ(&undistorted, row, col, x, y, z);        
                if ( deltaMat.at<float>(row, col) > 25 && deltaMat.at<float>(row, col) < 1000) {
                    cv::line(cDepthMat, cv::Point(col, row), cv::Point(col, row), cv::Scalar(0, 0, 255));
                    if (y < 0.3 && z > 0.4 && z < 5) {
                        cv::line(cDepthMat2, cv::Point(col, row), cv::Point(col, row), cv::Scalar(0, 0, 255));
                        resBinMat.at<uchar>(row, col) = 255;
                    } else {
                        resBinMat.at<uchar>(row, col) = 0;
                    }
                } else {
                    deltaMat.at<float>(row, col) = 0;
                    resBinMat.at<uchar>(row, col) = 0;
                }
            }
        }

        //cv::filter2D(resBinMat, resMat, resMat.depth(), ker);

        cv::flip(depthMat, depthMat, 1);
        cv::flip(deltaMat, deltaMat, 1);
        //cv::flip(colorMat, colorMat, 1);
        cv::flip(cDepthMat, cDepthMat, 1);
        cv::flip(cDepthMat2, cDepthMat2, 1);
        cv::flip(blurDepthMat, blurDepthMat, 1);
        cv::flip(resBinMat, resBinMat, 1);
        cv::flip(resMat, resMat, 1);

        std::vector<std::vector<cv::Point>> counters;
        cv::findContours(resBinMat, counters, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        auto it = counters.begin();
        while (it != counters.end()) {
            if (it->size() < 65 || it->size() > 200) {
                it = counters.erase(it);
            } else {
                ++it;
            }
        }
        //cv::Mat resMat(cv::Size(512, 424), CV_8U, cv::Scalar(255));
        cv::drawContours(resMat, counters, -1, cv::Scalar(0, 0, 255),2);
        /*
        cv::line(cDepthMat, cv::Point(0, 0), cv::Point(511, 423), cv::Scalar(255, 0, 0));
        cv::line(cDepthMat, cv::Point(0, 423), cv::Point(511, 0), cv::Scalar(255, 0, 0));
        cv::line(cDepthMat, cv::Point(256, 212), cv::Point(256, 212), cv::Scalar(0, 0, 255));
        */

        //writer << colorMat;
        

        /* 单纯距离标色
        for (int row = 0; row < depth->width; ++row) {
            for (int col = 0; col < depth->height; ++col) {
            if (blurDepthMat.at<float>(col, row) > 500 && blurDepthMat.at<float>(col, row) < 4000) {
                    cv::line(cDepthMat, cv::Point(row, col), cv::Point(row, col), cv::Scalar(0, 0, 255));
                    //cDepthMat.at<cv::Vec3b>(col, row) = red;
                }
            }
        }
        */

        //cv::imshow("color", colorMat);
        //cv::imshow("depth", depthMat / 4096.0f);
        cv::imshow("color depth", cDepthMat);
        cv::imshow("mask", cDepthMat2);
        cv::imshow("deltaMat", deltaMat / -4096.0);
        cv::imshow("blur depth", blurDepthMat / 4096.0f);
        cv::imshow("resbin", resBinMat);
        cv::imshow("res", resMat);
        //cv::imshow("outline", outlineMat / 4096.0f);

        //std::cout << "val " << std::setw(10) << depthMat.at<float>(256, 212) << "\r";

        //cv::imshow("depthraw", depthMat);
        //cv::imshow("depth0.5", depthMat / 512.0f);
        //cv::imshow("depth1", depthMat / 1024.0f);
        //cv::imshow("depth2", depthMat / 2048.0f);
        //cv::imshow("depth8", depthMat / 9192.0f);
        //cv::imshow("ir", irMat / 4096.0f);
        //cv::imshow("color", colorMat);


        /*
        float x,y,z;
        registration->getPointXYZ(&undistorted,undistorted.height/2, undistorted.width/2,
                                    x, y, z);
        */
        
        //std::cout << "x: " << x << " y: " << y << " z: " << z << "\r";
        
        /*
        // 
        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(unidisMat);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
        cv::imshow("undistorted", unidisMat / 4096.0f);
        cv::imshow("registered", rgbd);
        cv::imshow("depth2RGB", rgbd2 / 4096.0f);
        */

        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key &0xff) == 27));

        listener.release(frames);
    }

    // Stop the Device
    dev->stop();
    dev->close();
    // writer.release();

    std::cout << "Ends" << std::endl; 
    delete registration;
    return 0;
}

