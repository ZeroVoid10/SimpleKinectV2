/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		kinectgrabber.hpp
 * Description:		Kinect V2 Brabber
 * Author:			ZeroVoid
 * Version:			0.1.0
 * Data:			2019/11/04 Mon 19:51
 * Encoding:		UTF-8
 *******************************************************************************/

#ifndef KINECT_GRABBER_H
#define KINECT_GRABBER_H 

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>
#include <opencv2/opencv.hpp>

#include <fstream>

namespace zerovoid {
    class KinectGrabber {
    public:
        KinectGrabber ();

        bool start(void);
        void close(void);
        void closeCheck(void);
        bool waitNewFrame(int sec);
        void getFrames(cv::Mat* colorMat, cv::Mat* depthMat, cv::Mat* irMat);
        
        libfreenect2::FrameMap frames;
        libfreenect2::Registration *registration;
        libfreenect2::Frame undistorted, registered, depth2rgb;
        bool shutdown;

    private:
        std::string serial;
        libfreenect2::Freenect2 nect2;
        libfreenect2::Freenect2Device *dev;
        libfreenect2::PacketPipeline *pipeline;
        libfreenect2::SyncMultiFrameListener listener;

        bool prepare(void);
    };

    class KinectLogger: public libfreenect2::Logger {
    public:
        KinectLogger();
        bool good(void);
        virtual void log(Level level, const std::string &mes);
        bool start(void);
    private:
        std::ofstream logfile_;
    };
} /* kinectgrabber */ 

#endif /* ifndef KINECT_GRABBER_H */
