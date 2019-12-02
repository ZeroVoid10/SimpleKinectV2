#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main(void) {
    ofstream fout("cal_res.txt");
    string base_filename = "./img/cal_1/";

    int img_cnt = 0;
    Size img_size;
    Size board_size = Size(8, 11);
    vector<Point2f> img_point_buf;
    vector<vector<Point2f>> img_point_seq;
    int file_index = 1;
    string filename;
    for(;img_cnt < 15; file_index++, img_cnt++) {
        filename = base_filename + to_string(file_index) + ".png";
        Mat imgInput = imread(filename);
        std::cout << "img" << file_index << std::endl;
        
        if (img_cnt == 0) {
            img_size.width = imgInput.cols;
            img_size.height = imgInput.rows;
            std::cout << "img size wxh: " << img_size.width << "x" << img_size.height << std::endl;
        }

        if (findChessboardCorners(imgInput, board_size, img_point_buf) == 0) {
            std::cout << "Can not find chessboard corners" << std::endl;
            exit(1);
        } else {
            Mat gray;
            cvtColor(imgInput, gray, cv::COLOR_RGB2GRAY);
            cv::find4QuadCornerSubpix(gray, img_point_buf, Size(11,11));
            img_point_seq.push_back(img_point_buf);
            cv::drawChessboardCorners(gray, board_size, img_point_buf, true);
            imshow("cal_" + to_string(file_index), gray);
            cv::imwrite(base_filename + to_string(file_index) + "_fine.png", gray);
            waitKey(300);
        }
    }
    std::cout << "Total " << img_cnt << " pictures" << std::endl;
    std::cout << "Find corner done" << std::endl;
    std::cout << "Start calibrating..." << std::endl;

    vector<vector<cv::Point3f>> obj_points;
    Mat cameraMat = cv::Mat(3,3, CV_32FC1, cv::Scalar::all(0));
    Mat distCoffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
    std::vector<int> point_cnts;
    vector<Mat> tvecsMat;
    vector<Mat> rvecsMat;

    int i,j,t;
    for (t = 0; t < img_cnt; t++) {
        vector<Point3f> tmpPointSet;
        for (i = 0; i < board_size.height; i++) {
            for (j = 0; j < board_size.width; j++) {
                Point3f realPoint;
                realPoint.x = i*0.6;
                realPoint.y = j*0.6;
                realPoint.z = 0;
                tmpPointSet.push_back(realPoint);
            }
        }
        obj_points.push_back(tmpPointSet);
    }

    for (i = 0; i < img_cnt; i++) {
        point_cnts.push_back(board_size.width*board_size.height);
    }

    cv::calibrateCamera(obj_points, img_point_seq, img_size, cameraMat, distCoffs, rvecsMat, tvecsMat, 0);
    std::cout << "Calibration done!" << std::endl;
    std::cout << "cameraMat: " << cameraMat << std::endl;
    std::cout << "distCoffs" << distCoffs << std::endl;


    return 0;
}

