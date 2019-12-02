#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char *argv[]) {
    cv::Mat src_img1, src_img2;
    cv::Mat img1, img2;
    cv::Rect half(0, 480, 1920, 1080-480);
    src_img1 = cv::imread("../img/sample/rx_gray5.png");
    src_img2 = cv::imread("../img/sample/rx_gray6.png");
    img1 = cv::Mat(src_img1, half);
    img2 = cv::Mat(src_img2, half);
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;
    cv::Ptr<cv::Feature2D> ptrSURF = cv::xfeatures2d::SURF::create(2000.0);
    cv::Ptr<cv::FastFeatureDetector> ptrFAST = cv::FastFeatureDetector::create(50);

    ptrFAST->detect(img1, kp1);
    ptrFAST->detect(img2, kp2);
    ptrSURF->compute(img1, kp1, des1);
    ptrSURF->compute(img2, kp2, des2);
    
    cv::BFMatcher matcher(cv::NORM_L2, true);
    std::vector<cv::DMatch> matches;
    matcher.match(des1, des2, matches);

    cv::Mat matMat;
    cv::namedWindow("mat", 0);
    cv::drawMatches(img1, kp1, 
                    img2, kp2, 
                    matches, matMat, 
                    cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255));
    cv::imshow("mat", matMat);

    cv::waitKey(0);
    return 0;
}

