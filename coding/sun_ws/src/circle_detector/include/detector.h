#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

#define resize_length 500
#define threshold_perc 0.2

class CircleDetector{
    

public:
    CircleDetector(cv::Mat src);
    cv::Mat src;
    std::vector<std::vector<int>> feature_points;
    std::vector<std::vector<int>> GetSidePoints(std::vector<int> center);
    bool FindFeaturePoints(std::vector<int> test_point);
    bool IfCrossThreshold(int side, int center);
    int x;
    int y;
    void Run();
};

// convert the source img to GRAY img, and resize to 500 for maxside length
CircleDetector::CircleDetector(cv::Mat src){
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    float scaler = (float)resize_length/src.cols;
    cv::resize(src, src, cv::Size(), scaler, scaler);
    cv::GaussianBlur(src, src, cv::Size(5,5), 5);
    cv::threshold(src, src, 140, 255, cv::THRESH_BINARY);
    this->src = src;
    this->x = src.cols;
    this->y = src.rows;
    return;
}

// Get the points set which lies around the center point 
std::vector<std::vector<int>> CircleDetector::GetSidePoints(std::vector<int> center){
    std::vector<std::vector<int>> Points(16);
    int x = center[0]; 
    int y = center[1]; 
    Points[0].push_back(x);
    Points[0].push_back(y-3);
    Points[1].push_back(x+1);
    Points[1].push_back(y-3);
    Points[2].push_back(x+2);
    Points[2].push_back(y-2);
    Points[3].push_back(x+3);
    Points[3].push_back(y-1);
    Points[4].push_back(x+3);
    Points[4].push_back(y);
    Points[5].push_back(x+3);
    Points[5].push_back(y+1);
    Points[6].push_back(x+2);
    Points[6].push_back(y+2);
    Points[7].push_back(x+1);
    Points[7].push_back(y+3);
    Points[8].push_back(x);
    Points[8].push_back(y+3);
    Points[9].push_back(x-1);
    Points[9].push_back(y+3);
    Points[10].push_back(x-2);
    Points[10].push_back(y+2);
    Points[11].push_back(x-3);
    Points[11].push_back(y+1);
    Points[12].push_back(x-3);
    Points[12].push_back(y);
    Points[13].push_back(x-3);
    Points[13].push_back(y-1);
    Points[14].push_back(x-2);
    Points[14].push_back(y-2);
    Points[15].push_back(x-1);
    Points[15].push_back(y-3);
    return Points;
}

bool CircleDetector::IfCrossThreshold(int side, int center){
    if ((float)abs(side - center)/center > threshold_perc){
        return true;
    }
    else{
        return false;
    }
}

// check if the point is a feature point, if it is, push it in to the feature_points
bool CircleDetector::FindFeaturePoints(std::vector<int> test_point){
    int false_point = 0;
    int quick_check_flag = 0;
    std::vector<std::vector<int>> SidePoints(16);
    SidePoints = this->GetSidePoints(test_point);

    // quick_check 0 4 8 12
    for( int i = 0,j = 0; i <= 12; i += 4, j++){
        if(this->IfCrossThreshold(
            this->src.at<uchar>(SidePoints[i][0], SidePoints[i][1]),
            this->src.at<uchar>(test_point[0], test_point[1])
        )){
            quick_check_flag ++;
        }
        else{
            false_point = i;
        }
    }
    if(quick_check_flag < 3){ return false; }

    // complete_check
    quick_check_flag = 0;
    for(int i = 0; i < 16; i++){
        if(this->IfCrossThreshold(
            this->src.at<uchar>(SidePoints[i][0], SidePoints[i][1]),
            this->src.at<uchar>(test_point[0], test_point[1])
        )){
            quick_check_flag ++;
        }
        else{
            quick_check_flag = 0;
        }
        if( quick_check_flag == 12) {
            this->feature_points.push_back(test_point);
            cout << "Find a feature point" << endl;
            return true;
        }       
    }
    return false;
}

void CircleDetector::Run(){
    for(int i = 3; i < this->x-2; i++){
        for(int j = 3; j < this->y-2; j++){
            int tuple[2]   = {i,j};
            this->FindFeaturePoints(std::vector<int>(tuple, tuple+2));
        }
    }

    return;
}