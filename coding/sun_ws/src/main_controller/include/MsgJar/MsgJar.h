#ifndef _MSGJAR_
#define _MSGJAR_

#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <common_parameters.h>
using namespace std;
/* 
MsgJar is built to simplify ros_msg/serial_msg/bluetooth_msg and so on.



*/

typedef struct StateInfo{
    bool connected;                     // is connected
    bool armed;                         // is unlocked
    string mode;                        // flight mode, OFFBOARD eg.
    geometry_msgs::Vector3 linear;      // linear velocity
    geometry_msgs::Vector3 angular;     // angular velocity
    float height;                       // height data from laser
}StateInfo;

void update_serial_data(StateInfo* state_info){
    /* code */
    return;
}

void update_bluetooth_data(StateInfo* state_info){
    /* code */
    return;
}

/* 
Publish velocity with constraints
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