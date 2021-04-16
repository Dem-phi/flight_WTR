/*
 * @Author: Demphi
 * @Date: 2021-03-07
 * @email: demphi339413361@outlook.com
 * @Description: ros serial stm32
 * @Github: https://github.com/Dem-phi
 */

/*
how to start:
roscore
rosparam set joy_node/dev "/dev/input/jsX"
rosrun joy joy_node

A B X Y /joy->buttons[0 1 2 3]      Y means fiying
axes[1] left jointed arm UPAndDown(1->-1)*/

#ifndef SERIAL_CTL_H
#define SERIAL_CTL_H
#define capture_value 90

#include <iostream>

//ROS
#include <ros/ros.h>
#include <serial/serial.h>

//ROS msgs
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>

/*
55 aa size 00[0,1,2,3] 00[capture_value] 00[is_release] crc8 0d 0a
数据头55 aa + 数据字节数size + 数据 + 校验位crc8 + 数据尾0d0a
*/

namespace ctl{

    class SerialNode{
    private:
        int cnt;
        int get_start;
        ros::NodeHandle node_;
        ros::Subscriber joy_sub_;
        serial::Serial ser;
        ros::Timer serial_node_;

        typedef enum{
            flying,
            localtion_A,
            localtion_B,
            localtion_C
        }flight_t;
        flight_t flight_stauts;//defalut value

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void empty_mode();
        //void release_A_mode();
        //void release_B_mode();
        //void release_C_mode();
        void main_servo_ctl(const ros::TimerEvent&);

        const unsigned char header[2] = {0x55, 0xaa};
        const unsigned char ender[2] = {0x0d, 0x0a};
        void serialInit();
        void release_start(int j);//send data to stm32 to control servo
        unsigned char getCrc8(unsigned char *ptr, unsigned short len);
        
    public:
        SerialNode(ros::NodeHandle& nh);
        ~SerialNode(){};

    };
}


#endif //SERIAL_CTL_H

