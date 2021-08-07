#ifndef _POLYFILTER_RECTANGLE_SEARCHER_
#define _POLYFILTER_RECTANGLE_SEARCHER_

// #define DEBUGGING


#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <stdarg.h>
#include <stdlib.h>
#include <common_parameters.h>

using namespace std;
using namespace Eigen;

/* 
@brief Poly-filter searching method to locate the center of a rectangle
@param none
 */
class PFS{
public:
    cv::Mat src;

    vector<Vector2f> centers;
    bool is_finded = false;

    float cosine(cv::Point center, cv::Point a, cv::Point b);

    void rectangle_detect(cv::Mat src);


    PFS();
    ~PFS();
};

PFS::PFS(){
}

PFS::~PFS(){
}

float PFS::cosine(cv::Point center, cv::Point a, cv::Point b){
    Vector2f c = Vector2f(a.x-center.x, a.y-center.y);
    Vector2f d = Vector2f(b.x-center.x, b.y-center.y);
    return (c[0]*d[0]+c[1]*d[1])/(c.norm()*d.norm());
}

/* 
@brief find the center of a rectangle
you may get more than one possible center,
@param cv::Mat src
 */
void PFS::rectangle_detect(cv::Mat src){
    this->src = src;
    this->centers.clear();
    vector<cv::Point> corners;

    // preproduce the image
    float shrink_scale = 500.0/src.cols;
    cv::resize(this->src, this->src, cv::Size(),shrink_scale, shrink_scale);   // resize
    this->src.copyTo(src);
    
    // BGR2GRAY and binary threshold
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    cv::threshold(src, src, 160, 255, cv::THRESH_BINARY);
    cv::erode(src, src, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9,9)));
    cv::dilate(src, src, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)));

    // contours checking
    vector<vector<cv::Point>> contours;
    cv::findContours(src, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    // poly filter
    vector<vector<cv::Point>> polys;
    for(auto contour:contours){
        vector<cv::Point> poly;
        cv::approxPolyDP(contour, poly, 15, true);
        polys.push_back(poly);
        #ifdef DUBUGGING
        for(auto point:poly){
            cv::circle(src, point, 2, sun::COLOR_DRAW, 2);
        }
        #endif
    }

    #ifdef DUBUGGING
    cout << "Poly number: " << polys.size() << endl;
    cv::imshow("src", src);
    #endif

    // Rectangle check 
    vector<vector<cv::Point>> Rect;
    for(auto poly:polys){
        if(poly.size() != 4){
            continue;
        }
        float max_cosine = -10;
        float tmp_cos;
        tmp_cos = cosine(poly[0], poly[3], poly[1]);
        if( tmp_cos > max_cosine){ max_cosine = tmp_cos; }
        tmp_cos = cosine(poly[1], poly[0], poly[2]);
        if( tmp_cos > max_cosine){ max_cosine = tmp_cos; }
        tmp_cos = cosine(poly[2], poly[1], poly[3]);
        if( tmp_cos > max_cosine){ max_cosine = tmp_cos; }
        tmp_cos = cosine(poly[3], poly[2], poly[0]);
        if( tmp_cos > max_cosine){ max_cosine = tmp_cos; }
        cout << max_cosine << endl;
        cout << cos(60*2.0*CV_PI/360.0) << endl;
        if(max_cosine < cos(60*2.0*CV_PI/360.0) && sqrt(max((poly[0]-poly[1]).dot(poly[0]-poly[1]), (poly[1]-poly[2]).dot(poly[1]-poly[2]))) < 0.95*max(src.cols, src.rows)){
            Rect.push_back(poly);
            this->centers.push_back(Vector2f(
                (poly[0].x + poly[1].x + poly[2].x + poly[3].x)/4.0,
                (poly[0].y + poly[1].y + poly[2].y + poly[3].y)/4.0
            ));
            this->is_finded = true;
        }
    }
    if(this->centers.size() == 0){
        this->is_finded = false;
    }

    #ifdef DEBUGGING
    if(result.is_finded == false){
        cout << "not found" << endl;
        return result;
    }

    for(auto each:Rect){
        cv::line(in_src, each[0], each[1], Acolor, 3);
        cv::line(in_src, each[1], each[2], Acolor, 3);
        cv::line(in_src, each[2], each[3], Acolor, 3);
        cv::line(in_src, each[3], each[0], Acolor, 3);
    }

    imshow("center", in_src);

    
    cv::waitKey(50);
    cv::destroyAllWindows();
    #endif
    return;
}





#endif