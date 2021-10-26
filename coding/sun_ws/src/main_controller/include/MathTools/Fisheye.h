//
// Created by dji on 2021/8/18.
//

#include <iostream>
#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>*/

using namespace cv;
#ifndef _CAMERA_H
#define _CAMERA_H


cv::Mat modify(cv::Mat src);



void modify(cv::Mat* maps){
    cv::Mat intrinsic_matrix, new_Matrix;

    //摄像机内参数
    intrinsic_matrix = (Mat_<double>(3, 3) << 197.1264832685997, 0, 313.4094670802276, 0, 197.2824909727301, 247.5478545068283, 0, 0, 1);
    
    //镜头畸变参数
    cv::Vec4d distortion_coeffs;
    distortion_coeffs[0] = 0.624356;
    distortion_coeffs[0] = 0.0593305;
    distortion_coeffs[0] = -0.29334;
    distortion_coeffs[0] = 0.136311;

    Mat R= Mat::eye(3, 3, CV_32F);
    Size image_size;
    image_size.width = 640;
    image_size.height = 480;
    Mat mapx = Mat(image_size, CV_32FC1);
    Mat mapy = Mat(image_size, CV_32FC1);
    new_Matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
    initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
    maps[0] = mapx;
    maps[1] = mapy;
}

#endif
