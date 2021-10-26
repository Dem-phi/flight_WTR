#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <time.h>

using namespace std;
using namespace Eigen;


// Particle-Pair Search Method
class PPS{
public:
    bool is_debugging = true;
    cv::Mat src;
    int x, y, PPnum;
    int BinaryTreshold, resize_length, max_iter;
    
    std::vector<Vector2f> FeaturePoints;
    std::vector<Vector2f> findFeaturePoints();

    PPS(cv::Mat src, int BinaryThreshold = 150, int resize_length = 500, int ParticlePair_num = 40, int max_iter = 10);
    ~PPS();
};



PPS::PPS(cv::Mat src, int BinaryThreshold, int resize_length, int ParticlePair_num, int max_iter){
    float scaler = (float)resize_length/src.cols;
    cv::resize(src, src, cv::Size(), scaler, scaler);
    cv::GaussianBlur(src, src, cv::Size(5,5), 5);
    if(this->is_debugging){
        cv::Mat src_ori = src;
        cv::imshow("original_img", src_ori);
    }
    cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
    vector<cv::Mat> hsv_planes;
    cv::split(src, hsv_planes);
    // cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    cv::threshold(hsv_planes[1], hsv_planes[1], 36, 255, cv::THRESH_BINARY_INV);
    src = hsv_planes[1];
    
    
    this->src = src;
    this->x = src.cols;
    this->y = src.rows;
    this->PPnum = ParticlePair_num;
    this->BinaryTreshold = BinaryThreshold;
    this->resize_length = resize_length;
    this->max_iter = max_iter;
    return;
}

PPS::~PPS(){
}

std::vector<Vector2f> PPS::findFeaturePoints(){
    std::vector<Vector2f> FeaturePoints;
    srand( (unsigned)time(NULL));

    // generate random points
    std::vector<cv::Point> BlackPoints, WhitePoints;
    while(BlackPoints.size() < this->PPnum || WhitePoints.size() < this->PPnum){
        int tmp_x = (int)this->x*(rand()/(float)RAND_MAX);
        int tmp_y = (int)this->y*(rand()/(float)RAND_MAX);
        if(this->src.at<uchar>(tmp_y, tmp_x) > this->BinaryTreshold){
            WhitePoints.push_back(cv::Point(tmp_x, tmp_y));
        }else{
            BlackPoints.push_back(cv::Point(tmp_x, tmp_y));
        }
    }
    while(WhitePoints.size() > BlackPoints.size()){
        WhitePoints.pop_back();
    }
    while(BlackPoints.size() > WhitePoints.size()){
        BlackPoints.pop_back();
    }

    // dividing iteration
    for(int iter = 0; iter < this->max_iter; iter ++){
        for(int i = 0; i < BlackPoints.size(); i++){
            int tmp_x = (int)(BlackPoints[i].x + WhitePoints[i].x)/2;
            int tmp_y = (int)(BlackPoints[i].y + WhitePoints[i].y)/2;
            if( this->src.at<uchar>(tmp_y, tmp_x) > this->BinaryTreshold){
                WhitePoints[i] = cv::Point(tmp_x, tmp_y);
            }else{
                BlackPoints[i] = cv::Point(tmp_x, tmp_y);
            }
        }
    }

    if(this->is_debugging){
        // draw image
        for(int i = 0; i < this->PPnum; i++){
            //cv::circle(this->src, WhitePoints[i], 5, cv::Scalar(50, 0, 255, 0));
            cv::rectangle(this->src, cv::Rect(WhitePoints[i].x, WhitePoints[i].y, 5, 5), cv::Scalar(120));
        }
        for(int i = 0; i < this->PPnum; i++){
            cv::circle(this->src, cv::Point(BlackPoints[i].x,BlackPoints[i].y), 5, cv::Scalar(120, 255, 0, 0));
        }
        cv::imshow("PPS_img", this->src);
    }

    // compute feature points
    for(int i = 0; i < this->PPnum; i++){
        FeaturePoints.push_back(
            Vector2f(
                (int)(BlackPoints[i].x + WhitePoints[i].x)/2, 
                (int)(BlackPoints[i].y + WhitePoints[i].y)/2
            )
        );
    }
    return FeaturePoints;
}