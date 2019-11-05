#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <map>

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

    pipeline = new libfreenect2::OpenCLKdePacketPipeline();
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
    cv::Mat blurDepthMat, blurDepthMat2;
    float x,y,z, color;

    // Filter
    pcl::visualization::CloudViewer viewer("CloudPointTest");
    //pcl::FastBilateralFilter<PointT> fbf;
    pcl::PassThrough<PointT> pass;

    pass.setFilterFieldName("z");
    pass.setFilterLimits(-5.0, -0.5);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.4, 4.0); // 过滤掉地板
    //pass.setFilterFieldName("x");
    //pass.setFilterLimits(-2, 2);
    //pass.setFilterLimitsNegative(true);
    
    /*
    fbf.setSigmaS(5);
    fbf.setSigmaR(5e-3);
    */

    bool protonect_shutdown = false;
    bool first_frame = true;
    PointCloud::Ptr lastCloud(new PointCloud);
    lastCloud->width = 512;
    lastCloud->height = 424;
    lastCloud->resize(512*424);
    lastCloud->is_dense = false;

    while(!protonect_shutdown) {
        if (!listener.waitForNewFrame(frames, 10*1000)) {
            std::cout << "timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC4, rgb->data).copyTo(colorMat);
        cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data).copyTo(depthMat);
        cv::flip(colorMat, colorMat, 1); // 左右翻转方便展示看
        cv::flip(depthMat, depthMat, 1);

        cv::medianBlur(depthMat, blurDepthMat, 5);
        //cv::medianBlur(depthMat, blurDepthMat2, 5);
        //cv::imshow("color", colorMat);
        cv::imshow("depth4", depthMat / 4096.0f);
        cv::imshow("blur /", blurDepthMat / 4096.0f);
        //cv::imshow("blur", blurDepthMat2 / 4096.0f);

        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        PointCloud::Ptr cloud(new PointCloud);
        //cloud->width = depth->width;
        //cloud->height = depth->height;
        cloud->height = 1;
        //cloud->resize(depth->width * depth->height);
        cloud->is_dense = false;
        //cloud->points.resize(cloud->width*cloud->height);
        PointCloud::Ptr cloud_filtered(new PointCloud);
        cloud_filtered->width = depth->width;
        cloud_filtered->height = depth->height;
        cloud_filtered->is_dense = false;


        //std::map<PointT, std::vector<int>> dep2pc;

        // formate registered to pcl cloud point
        for (int m = 0; m < depth->height; m++) {
            for (int n = 0; n < depth->width; n++) {
                unsigned int index = m*depth->width + n;
                //std::vector<int> depPoint(2);
                //depPoint[0] = m;
                //depPoint[1] = n;
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
                //dep2pc[p] = depPoint;
                //cloud->points[index] = p;
            }
        }

        //pcl::PLYWriter writer;
        cloud->width =cloud->points.size();
        std::fstream fs;
        std::stringstream ss;
        ss << "test.txt";
        fs.open("test.txt", std::fstream::out);
        for (int i = 0; i < cloud->points.size(); i++) {
            fs << cloud->points[i].x << "\t";
            fs << cloud->points[i].y << "\t";
            fs << cloud->points[i].z << "\n";
        }
        fs.close();

        //std::cout<< cloud->width*cloud->height << " " << cloud->points.size() << std::endl;
        //writer.write("test.ply", *cloud);

        /* passthrough filter */
        pass.setInputCloud(cloud);
        pass.filter(*cloud_filtered);

        /* FastBilateralFilter */
        //fbf.setInputCloud(cloud_filtered);
        //fbf.applyFilter(*cloud_filtered);

        viewer.showCloud(cloud_filtered);
        //viewer.showCloud(cloud);
        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key &0xff) == 27)) || viewer.wasStopped();

        listener.release(frames);
    }

    // Stop the Device
    dev->stop();
    dev->close();
    delete registration;
    
    std::cout << "Ends" << std::endl; 
    return 0;
}

