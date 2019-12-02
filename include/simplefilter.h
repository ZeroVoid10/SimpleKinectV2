#ifndef _SIMPLEFILTER
#define _SIMPLEFILTER 

#include <iostream>

#include <opencv2/opencv.hpp>
#include <libfreenect2/registration.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral.h>

namespace zerovoid {

    class SimpleFilter {
    private:
        libfreenect2::Registration registration;
        libfreenect2::Frame undistorted, registered, depth2rgb;
    public:
        SimpleFilter();
    };

} // namespace zerovoid

#endif /* ifndef _SIMPLEFILTER */
