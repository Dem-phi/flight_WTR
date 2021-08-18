#ifndef _MSGJAR_
#define _MSGJAR_

#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <common_parameters.h>
#include <ros/ros.h>
using namespace std;
/* 
MsgJar is built to simplify ros_msg/serial_msg/bluetooth_msg and so on.

*/

typedef struct StateInfo{
    bool connected;                     // is connected
    bool armed;                         // is unlocked
    bool gogogo;                        // use the button of Raspberry Pi to control UAV for auto takeoff
    float height;                       // height data from laser
    int loading;                        // use rc SD channel to load cylinder
    int load_angle;                     // use rc V1 to decide the angle of each servo
    int emergency_land;                 // if needs emergency land
    int manual_takeoff;                 // use Manual Control for safety
    string mode;                        // flight mode, OFFBOARD eg.
    geometry_msgs::Vector3 linear;      // linear velocity
    geometry_msgs::Vector3 angular;     // angular velocity
    geometry_msgs::Vector3 vel_info;    // velocity commend in manual mode 
    geometry_msgs::Pose cur_pose;       // position info provided by t265 vision fusion
}StateInfo;


// TODO update info from serial port
void update_serial_data(StateInfo* state_info){
    /* code */
    return;
}
// TODO update info from booth device
void update_bluetooth_data(StateInfo* state_info){
    /* code */

    return;
}

/**
 * @brief Velocity publisher with constraints. It would be safer.
 * @param {ros::Publisher*} pub
 * @param {geometry_msgs::TwistStamped} msg
 * @return none
 */
void publish_vel(ros::Publisher* pub, geometry_msgs::TwistStamped msg){
    float linear_speed = sqrt(
        msg.twist.linear.x*msg.twist.linear.x +
        msg.twist.linear.y*msg.twist.linear.y + 
        msg.twist.linear.z*msg.twist.linear.z 
    );
    float angular_speed = sqrt(
        msg.twist.angular.x*msg.twist.angular.x + 
        msg.twist.angular.y*msg.twist.angular.y + 
        msg.twist.angular.z*msg.twist.angular.z 
    );
    if(linear_speed > sun::MAX_LINEAR_VEL){
        float shrink_scale = sun::MAX_LINEAR_VEL/linear_speed;
        msg.twist.linear.x *= shrink_scale;
        msg.twist.linear.y *= shrink_scale;
        msg.twist.linear.z *= shrink_scale;
    }
    if(angular_speed > sun::MAX_ANGULAR_VEL){
        float shrink_scale = sun::MAX_ANGULAR_VEL/angular_speed;
        msg.twist.angular.x *= shrink_scale;
        msg.twist.angular.y *= shrink_scale;
        msg.twist.angular.z *= shrink_scale;
    }
    pub->publish(msg);
    return;
}



#endif