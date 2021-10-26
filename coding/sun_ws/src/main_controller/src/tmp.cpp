#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <HoughCircle.h>
#include <PFS.h>
#include <PPS.h>
#include <MMCS.h>
#include <vector>

using namespace std;
using namespace cv;

float height = 0;

void Callback_pose(const geometry_msgs::PoseStampedConstPtr & msg){
    height = msg->pose.position.z;
    return;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 10, Callback_pose
    );

    cv::VideoCapture camera(0);
    Mat src;
    if(camera.isOpened()){
        cout << "open camera successfully" << endl;
    }else{
        cout << "fail to open camera" << endl;
        exit(0);
    }
    camera.read(src);
    int cols = src.cols, rows = src.rows;
    Vector2f center = Vector2f(cols/2, rows/2);
    vector<Vector2f> feature_points;


    HoughCircle model_Hough;
    PFS model_PFS;
    MMCS model_MMCS(feature_points, center);

    int keyvalue = 0;

    while(cv::waitKey(1) != 27){
        ros::spinOnce();
        cout << "Height = " << height << "m" << endl;
        camera.read(src);
        imshow("window",src);
        model_Hough.run(src);
        if(model_Hough.finded_circles.size() == 0){continue;}
        float delta_x = abs(model_Hough.finded_circles[0][0]-center[0]);
        float delta_y = abs(model_Hough.finded_circles[0][1]-center[1]);
        cout << "delta_x = " << delta_x << "\tdelta_y = " << delta_y << endl;
        if(delta_x < 10 && delta_y < 10 && (abs(height-1.0)<0.05 || abs(height-0.3)<0.05)){
            cout << "Reach target height and complete circle docking. Begin to capture.";
        }
    }
    PPS model_PPS(src);
    feature_points = model_PPS.findFeaturePoints();
    model_MMCS.discard_noise(0.0, 50.0);
    cout << "discarding" << endl;
    cv::imwrite("/home/dji/Test_v.png", src);
    cout << "Done!" << endl;
    camera.release();

    return 0;
}





















