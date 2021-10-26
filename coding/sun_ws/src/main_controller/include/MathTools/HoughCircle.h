#ifndef _HOUGH_CIRCLE_
#define _HOUGH_CIRCLE_

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;


/* 
Hough circle detection
@param None
 */
class HoughCircle{
public:
    cv::Mat src;
    float min_radius = 0, max_radius = 150;
    int param1 = 100, param2 = 50;
    vector<cv::Vec3f> finded_circles;
    bool low_detect = false;

    void run(cv::Mat src);

};


/* 
@brief Load and preprocess the image. After processing, you can visit finded_circles for result
@param cv::Mat src
 */
void HoughCircle::run(cv::Mat src){
    float shrink_scale = 500.0/src.cols;
    cv::resize(src, src, cv::Size(), shrink_scale, shrink_scale);
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(src, src, cv::Size(5,5), 3);
    if(this->low_detect){
        cv::HoughCircles(src, this->finded_circles, cv::HOUGH_GRADIENT, 1, 80, 100, 40, this->min_radius, this->max_radius);    
    }else{
        cv::HoughCircles(src, this->finded_circles, cv::HOUGH_GRADIENT, 1, 80, 140, 40, this->min_radius, this->max_radius);
    }
    return;
}



#endif