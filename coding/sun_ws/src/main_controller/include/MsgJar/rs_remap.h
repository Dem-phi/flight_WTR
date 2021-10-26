//
// Created by demphi on 9/16/21.
//

#ifndef MAIN_CONTROLLER_RS_REMAP_H
#define MAIN_CONTROLLER_RS_REMAP_H

/**
 * @author      :demphi
 * @date        :Created in 9/16/21
 * @brief       :${description}
 * @param       :${parameters}
 */

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>

class rs_remap {
public:
    ros::NodeHandle nh;
    ros::Subscriber sub_camera, sub_laser;
    ros::Publisher pub_vision;

    geometry_msgs::PoseStamped msg;

    bool use_t265_height = false;

    rs_remap(ros::NodeHandle &nh);
    void cameraCallback(const nav_msgs::OdometryConstPtr & msg);
    void laserCallback(const std_msgs::Int32ConstPtr & msg);

    void run();
};

rs_remap::rs_remap(ros::NodeHandle &nh){
    this->nh = nh;
    this->sub_camera = this->nh.subscribe("/camera/odom/sample", 10, &rs_remap::cameraCallback, this);
    this->sub_laser = this->nh.subscribe("/sun/height", 10, &rs_remap::laserCallback, this);
    this->pub_vision = this->nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
}

void rs_remap::cameraCallback(const nav_msgs::OdometryConstPtr &msg){
    this->msg.pose.position.x = msg->pose.pose.position.x;
    this->msg.pose.position.y = msg->pose.pose.position.y;
    this->msg.pose.orientation = msg->pose.pose.orientation;
    if(this->use_t265_height){
        this->msg.pose.position.z = msg->pose.pose.position.z;
//        ROS_ERROR("Using t265");
    }
    this->msg.header.stamp = ros::Time::now();
    this->msg.header.frame_id = "map";
    return;
}

void rs_remap::laserCallback(const std_msgs::Int32ConstPtr &msg){
    int init_height = 100;
    if(!this->use_t265_height){
        this->msg.pose.position.z = float((msg->data - init_height))/1000.0;
//        ROS_ERROR("Using laser");
    }
    this->msg.header.stamp = ros::Time::now();
    this->msg.header.frame_id = "map";
    return;
}

void rs_remap::run(){
    ros::spinOnce();
    ros::spinOnce();
    this->pub_vision.publish(this->msg);
    return;
}

#endif
