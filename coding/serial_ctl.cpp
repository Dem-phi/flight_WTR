/*
 * @Author: Demphi
 * @Date: 2021-02-25 21:06:50
 * @email: demphi339413361@outlook.com
 * @Description: Add you want
 * @Github: https://github.com/Dem-phi
 */

#include <serial_ctl.h>

using namespace ctl;

void SerialNode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    bool is_mode_change = false;
    int _stauts_;
    if (msg->buttons[3]!=0 && get_start == 0){
        get_start = 1;
        ROS_INFO("serial msg transmit start!");
    }
    if(get_start == 1) {
        if (msg->buttons[3] == 0 && flight_stauts == flying && !is_mode_change) {
            for (int i = 0; i < 3; i++) {
                if (msg->buttons[i] != 0) {
                    _stauts_ = i + 2; //_stauts_ defalut value is 0
                }
            }
            if (_stauts_ != 0) {
                switch (_stauts_) {
                    case 2:
                        flight_stauts = localtion_A;
                        break;
                    case 3:
                        flight_stauts = localtion_B;
                        break;
                    case 4:
                        flight_stauts = localtion_C;
                        break;
                }
                is_mode_change = true;
            }
        }else if (msg->buttons[3] != 0 && flight_stauts != flying) {
            flight_stauts = flying;
            _stauts_ = 0;
            is_mode_change = false;
        }
    }
}

void SerialNode::empty_mode(){
    if (cnt++>50){
        cnt=0;
        ROS_INFO("[SERVO STAUTS]: 00 00 00 ---------- Waiting for releasing");
    }
    unsigned char buf[10] = {0};
    int i ,length;
        //set msg header
        for (i = 0; i < 2; i++){
            buf[i] = header[i];
        }
        //set size
        length = 3;
        buf[2] = length;

        //set control servo
        buf[3 + i] = 0x00;
        buf[4]=char(capture_value);
        buf[5]=0x00;
        //set msg checkbit and ender
        buf[3+length] = getCrc8(buf, 3+length);
        buf[3+length+1] = ender[0];
        buf[3+length+2] = ender[1];
        //send msg
        ser.write(buf, 10);

}

void SerialNode::main_servo_ctl(const ros::TimerEvent&){
    switch (flight_stauts){
        case flying:{
            empty_mode();
            break;
        }
        case localtion_A:{
            release_start(1);
            break;
        }
        case localtion_B:{
            release_start(2);
            break;
        }
        case localtion_C:{
            release_start(3);
            break;
        }
        default:{
            empty_mode();
        }
    }
    ros::spinOnce();
}

SerialNode::SerialNode(ros::NodeHandle& nh){
    node_ = nh;
    flight_stauts = flying;
    // Define the subscriber of controller
    joy_sub_ = nh.subscribe("/joy", 10, &SerialNode::joyCallback, this);
    serialInit();
    serial_node_ = node_.createTimer(ros::Duration(0.02), &SerialNode::main_servo_ctl, this);
    ROS_INFO("INIT SERIAL NODE SUCCESS!");
    ros::spinOnce();
}

void SerialNode::serialInit(){
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(115200);
    serial::Timeout to=serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
    if(ser.isOpen()){
        ROS_INFO("Serial Port is opened");
    }
    else{
        while (!ser.isOpen()){
            ROS_INFO("Error, Serial Port is failed!");
        }
    }
}

//send data to stm32 to control servo
void SerialNode::release_start(int j){
    unsigned char buf[10] = {0};
    int i ,length;
    if(j!=0){
        //set msg header
        for (i = 0; i < 2; i++){
            buf[i] = header[i];
        }
        //set size
        length = 3;
        buf[2] = length;
        //set control servo
        switch (j){
        case 1:
            buf[3]=0x01;
            ROS_INFO("[SERVO STAUTS]: 01 %d 01 ---------- Servo 1 release", capture_value);
            break;
        case 2:
            buf[3]=0x02;
            ROS_INFO("[SERVO STAUTS]: 02 %d 01 ---------- Servo 2 release", capture_value);
            break;
        case 3:
            buf[3]=0x03;
            ROS_INFO("[SERVO STAUTS]: 03 %d 01 ---------- Servo 3 release", capture_value);
            break;
        }
        buf[4]=char(capture_value);
        buf[5]=0x01;
        //set msg checkbit and ender
        buf[3+length] = getCrc8(buf, 3+length);
        buf[3+length+1] = ender[0];
        buf[3+length+2] = ender[1];

        //send msg
        ser.write(buf, 10);
        //ROS_INFO("serial transmit success");
    }
}

unsigned char SerialNode::getCrc8(unsigned char *ptr, unsigned short len){
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;              //每次先与需要计算的数据异或,计算完指向下一数据
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)            //判断低位是不是1
                crc=(crc>>1)^0x8C;  //向右移动一位，与10001100异或
            else
                crc >>= 1;          //最低位为0时，数据整体往右移动1位
        }
    }
    return crc;
}