/*
 * @Author: Demphi
 * @Date: 2021-09-14 18:12:43
 * @email: demphi339413361@outlook.com
 * @Description: Add you want
 * @Github: https://github.com/Dem-phi
 */
#ifndef _RS_REMAP_
#define _RS_REMAP_


#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <nlohmann/json.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Eigen;

typedef struct Ac_data{
    float position[3];
    float velocity[3];
    float quaternion[4];
    float eular[3];
}Ac_data;

float qx,qy,qz,qw;
void IMU_Callback(const sensor_msgs::ImuConstPtr & msg){
    qx = msg->orientation.x;
    qy = msg->orientation.y;
    qz = msg->orientation.z;
    qw = msg->orientation.w;
    return;
}



class Remaper{
public:
    ros::NodeHandle nh;
    serial::Serial sp;
    ros::Publisher vision_pose_pub;
    ros::Subscriber imu_sub;
    geometry_msgs::PoseStamped msg;

    char read_buffer[1024];
    int read_index = 0;
    bool is_reading = false;
    int is_init = 0;

    Ac_data ac_data;

    void read_ac_data();
    void clear_buffer(int num = 9999);
    void set_serial_port(string port_name, int baudrate);
    Remaper(ros::NodeHandle &nh);
};

Remaper::Remaper(ros::NodeHandle &nh){
    this->nh = nh;
    this->vision_pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    this->imu_sub = this->nh.subscribe("/mavros/imu/data", 10, IMU_Callback);
}

void Remaper::clear_buffer(int num){
    if( num == 9999 ){
        for(int i = 0; i < 1024; i++){
            this->read_buffer[i] = '\0';
        }
    }else{
        for(int i = 0; i < num; i++){
            this->read_buffer[i] = '\0';
        }
    }
}

void Remaper::set_serial_port(string port_name, int baudrate){
    this->sp.setPort(port_name);
    this->sp.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    this->sp.setTimeout(to);
    this->sp.open();
    return;
}

void Remaper::read_ac_data(){
    ros::spinOnce();
    if(!this->sp.isOpen()){
        ROS_ERROR("Serial is not opened!");
        return;
    }
    if(this->sp.available() <= 0){
        // ROS_WARN("No data found!");
        return;
    }
    uint8_t tmp_char;
    this->sp.read(&tmp_char, 1);
    if(this->is_reading == false){
        if(tmp_char == (int)'{'){
            this->is_reading = true;
            this->read_buffer[0] = '{';
            this->read_index = 1;
        }else{
            return;
        }
    }else{
        if(tmp_char == (int)'{'){
            this->clear_buffer(this->read_index);
            this->read_buffer[0] = '{';
            this->read_index = 1;
            return;
        }
        if(tmp_char == (int)'}'){
            this->is_reading = false;
            this->read_buffer[this->read_index] = '}';
            this->read_index += 1;
            this->read_buffer[this->read_index] = '\0';
            // process
            string words = this->read_buffer;
            nlohmann::json json = nlohmann::json::parse(words);

            this->ac_data.position[0] = json["position"][0];
            this->ac_data.position[0] /= 100.0;
            this->ac_data.position[1] = json["position"][1];
            this->ac_data.position[1] /= 100.0;
            this->ac_data.position[2] = json["position"][2];
            this->ac_data.position[2] /= 100.0;
            this->ac_data.velocity[0] = json["velocity"][0];
            this->ac_data.velocity[0] /= 100.0;
            this->ac_data.velocity[1] = json["velocity"][1];
            this->ac_data.velocity[1] /= 100.0;
            this->ac_data.velocity[2] = json["velocity"][2];
            this->ac_data.velocity[2] /= 100.0;
            this->ac_data.quaternion[0] = json["quaternion"][0];
            this->ac_data.quaternion[1] = json["quaternion"][1];
            this->ac_data.quaternion[2] = json["quaternion"][2];
            this->ac_data.quaternion[3] = json["quaternion"][3];
            this->ac_data.eular[0] = json["eular"][0];
            this->ac_data.eular[1] = json["eular"][1];
            this->ac_data.eular[2] = json["eular"][2];

//            float theta = this->ac_data.eular[2]/180.0*3.14159;
            float theta = 0;
            float init_w = 0;
            if(this->is_init == 100){
                theta = this->ac_data.eular[2];
                ROS_INFO("ACfly init successfully.");
                is_init ++;
            }else if(this->is_init < 100){
                is_init ++;
            };

//            float px = this->ac_data.position[0]*cos(theta) + this->ac_data.position[1]*sin(theta);
//            float py = this->ac_data.position[1]*cos(theta) - this->ac_data.position[0]*sin(theta);
//            float vx = this->ac_data.velocity[0]*cos(theta) + this->ac_data.velocity[1]*sin(theta);
//            float vy = this->ac_data.velocity[1]*cos(theta) - this->ac_data.velocity[0]*sin(theta);
//
//            this->ac_data.position[0] = px;
//            this->ac_data.position[1] = py;
//            this->ac_data.velocity[0] = vx;
//            this->ac_data.velocity[1] = vy;

            msg.pose.position.x = this->ac_data.position[0];
            msg.pose.position.y = this->ac_data.position[1];
            msg.pose.position.z = this->ac_data.position[2];

            msg.pose.orientation.x = qx;
            msg.pose.orientation.y = qy;
            msg.pose.orientation.z = qz;
            msg.pose.orientation.w = qw;

#ifdef DEBUG
            cout << this->ac_data.eular[2] << " "<< px << " " << py << " " <<msg.pose.position.z << endl;
#endif

            this->clear_buffer(this->read_index);
            this->read_index = 0;
            this->msg.header.frame_id = "body";
            this->msg.header.stamp = ros::Time::now();
            this->vision_pose_pub.publish(this->msg);

        }else{
            this->read_buffer[this->read_index] = (char)tmp_char;
            this->read_index ++;
        }
    }


}

#endif