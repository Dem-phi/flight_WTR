#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace Eigen;

Quaternionf q_init, q_now, Q_now;
bool is_init = false;

void imu_Callback(const sensor_msgs::ImuConstPtr & msg){
    if(!is_init){
        q_init = Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        is_init = true;
        return;
    }
    Quaternionf q_r;
    q_r.vec() = Vector3f(msg->orientation.x, msg->orientation.y, msg->orientation.z);
    q_r.w() = msg->orientation.w;
//    Quaternionf q_tmp = q_init*q_r*q_init.conjugate();
//    q_now.w() = msg->orientation.w;
//    q_now.vec() = q_tmp.vec();
    Q_now = q_r*q_init.conjugate();
    cout << Q_now.x() << " " << Q_now.y() << " " << Q_now.z() << " " << Q_now.w() << endl;
}

ros::Subscriber imu_sub;

int main(int argc, char** argv){
    ros::init(argc, argv,"test_node");
    ros::NodeHandle nh;

    imu_sub = nh.subscribe("/mavros/imu/data", 10, imu_Callback);

//    Quaternionf a, b, c;
//    a.vec() = Vector3f(0,0,1.0)*sin(3.14159/4);
//    a.w() = cos(3.14159/4);
//
//    b.vec() = Vector3f(1,0,0);
//    b.w() = 0;
//
//    c = a*b*a.conjugate();
//
//    cout << c.x() << " " << c.y() << " " << c.z() << " " << c.w() << endl;

    ros::spin();

    return 0;


}
