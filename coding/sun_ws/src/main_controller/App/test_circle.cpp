//
// Created by demphi-dji on 4/17/21.
//
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>

ros::Publisher pub_circles;

int main(int argc, char**argv)
{
    ros::init(argc, argv, "node_circle_test");
    ros::NodeHandle nh;
    pub_circles = nh.advertise<geometry_msgs::Vector3>("/sun/circles", 10);
    geometry_msgs::Vector3 circles;
    circles.x = 200;
    circles.y = 200;
    ros::Rate rate(50);
    while(1){
        pub_circles.publish(circles);
        rate.sleep();
    }

}