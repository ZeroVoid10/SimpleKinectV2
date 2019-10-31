#include <iostream>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral.h>

#include <fstream>
#include <cstdlib>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

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

int main(int argc, char *argv[]) {
    std::cout << "Clout Point Test" << std::endl;
    // file logger
    TestLogger *testlogger = new TestLogger("./log/cptest.log");
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

    cv::Mat depthMat, colorMat, unidisMat, rgbd, dst;
    cv::Mat blurDepthMat;
    float x,y,z, color;

    pcl::visualization::CloudViewer viewer("CloudPointTest");
    pcl::PassThrough<PointT> pass;

    bool protonect_shutdown = false;
    while(!protonect_shutdown) {
        if (!listener.waitForNewFrame(frames, 10*1000)) {
            std::cout << "timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC4, rgb->data).copyTo(colorMat);
        cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data).copyTo(depthMat);

        //cv::medianBlur(depthMat / 4096.0f, blurDepthMat, 5);
        cv::imshow("color", colorMat);
        //cv::imshow("depth4", depthMat / 4096.0f);

        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        PointCloud::Ptr cloud(new PointCloud);
        PointCloud::Ptr cloud_filtered(new PointCloud);
        /*
        pcl::FastBilateralFilter<PointT> filter;
        filter.setSigmaS(5);
        filter.setSigmaR(5e-3);
        */
        for (int m = 0; m < 512; m++) {
            for (int n = 0; n < 424; n++) {
                PointT p;
                PointT p2;
                registration->getPointXYZRGB(&undistorted, &registered, n, m, x, y, z, color);
                const uint8_t *c = reinterpret_cast<uint8_t*>(&color);
                    p.z = -z;
                    p.x = -x;
                    p.y = -y;
                    p.b = c[0];
                    p.g = c[1];
                    p.r = c[2];
                cloud->points.push_back(p);
            }
        }
        
        // filter
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.setFilterLimitsNegative(true);
        pass.filter(*cloud_filtered);

        viewer.showCloud(cloud_filtered);
        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key &0xff) == 27));
        listener.release(frames);
    }

    // Stop the Device
    dev->stop();
    dev->close();
    delete registration;
    
    std::cout << "Ends" << std::endl; 
    return 0;
}

