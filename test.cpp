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
    cv::Mat cDepthMat, blurDepthMat;
    bool protonect_shutdown = false;
    cv::Vec3b red;
    red[0] = 0;
    red[1] = 0;
    red[2] = 255;

    cv::VideoWriter writer;
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
        cv::flip(depthMat, depthMat, 1);
        cv::cvtColor(depthMat / 4096.0f, cDepthMat, cv::COLOR_GRAY2RGB);
        /*
        cv::line(cDepthMat, cv::Point(0, 0), cv::Point(511, 423), cv::Scalar(255, 0, 0));
        cv::line(cDepthMat, cv::Point(0, 423), cv::Point(511, 0), cv::Scalar(255, 0, 0));
        cv::line(cDepthMat, cv::Point(256, 212), cv::Point(256, 212), cv::Scalar(0, 0, 255));
        */

        cv::medianBlur(depthMat, blurDepthMat, 5);
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::medianBlur(rgbd, rgbd, 5);
        //writer << colorMat;


        for (int row = 0; row < depth->width; ++row) {
            for (int col = 0; col < depth->height; ++col) {
            if (blurDepthMat.at<float>(col, row) > 500 && blurDepthMat.at<float>(col, row) < 4000) {
                    cv::line(cDepthMat, cv::Point(row, col), cv::Point(row, col), cv::Scalar(0, 0, 255));
                    //cDepthMat.at<cv::Vec3b>(col, row) = red;
                }
            }
        }

        cv::flip(colorMat, colorMat, 1);
        cv::imshow("color", colorMat);
        cv::imshow("depth4", depthMat / 4096.0f);
        cv::imshow("color depth", cDepthMat);
        cv::imshow("blur depth", blurDepthMat / 4096.0f);

        std::cout << "val " << std::setw(10) << depthMat.at<float>(256, 212) << "\r";

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

