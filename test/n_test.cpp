#include "kinectgrabber.h"

using namespace zerovoid;
int main(int argc, char *argv[])
{
    KinectLogger *logger = new KinectLogger();
    if (logger->good()) {
        libfreenect2::setGlobalLogger(logger);
    } else {
        delete logger;
    }
    return 0;
}
