#ifndef _POSEOFFLOADWORKER_H
#define _POSEOFFLOADWORKER_H

#include <StateWorker.h>
#include <geometry_msgs/PoseStamped.h>

class PoseOffloadWorker : public StateWorker {
public:
    ros::NodeHandle nh;
    ros::Publisher pub_goal_pose;
    ros::Publisher servo_pub;
    geometry_msgs::Vector3 servo_msg;
    geometry_msgs::PoseStamped goal_pose;
    bool ignore_xy = false;
    bool is_init = false;
    int area;
    int convergence_counter = 0;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    PoseOffloadWorker(ros::NodeHandle &nh, double x, double y, double z, int area);
    ~PoseOffloadWorker();
};

PoseOffloadWorker::PoseOffloadWorker(ros::NodeHandle &nh, double x, double y, double z, int area){
    if(abs(x - 1000) < 10 || abs(y-1000) < 10){
        this->ignore_xy = true;
    }
    this->goal_pose.header.frame_id = "body";
    this->nh = nh;
    this->servo_pub = nh.advertise<geometry_msgs::Vector3>("/sun/servo_ctl", 10);
    this->pub_goal_pose = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    this->area = area;
    if(abs(x)>5 || abs(y)>5 || abs(z)>3){
        x = 0.0;
        y = 0.0;
        z = 0.1;
    }
    this->goal_pose.pose.position.x = x;
    this->goal_pose.pose.position.y = y;
    this->goal_pose.pose.position.z = z;

    return;
}

PoseOffloadWorker::~PoseOffloadWorker(){
}

void PoseOffloadWorker::run(StateInfo state_info){
    ROS_INFO("Fire!!!!!");
    if(!this->is_init){
        goal_pose.pose.orientation = state_info.init_quater;
        this->is_init = true;
    }
    goal_pose.header.stamp = ros::Time::now();
    if(this->ignore_xy){
        goal_pose.pose.position.x = state_info.cur_pose.position.x;
        goal_pose.pose.position.y = state_info.cur_pose.position.y;
    }
    this->pub_goal_pose.publish(this->goal_pose);
    if( abs(this->goal_pose.pose.position.x - state_info.cur_pose.position.x) < sun::POSITION_TOLERANCE_X &&
        abs(this->goal_pose.pose.position.y - state_info.cur_pose.position.y) < sun::POSITION_TOLERANCE_Y &&
        abs(this->goal_pose.pose.position.z - state_info.cur_pose.position.z) < sun::POSITION_TOLERANCE_Z ){
        this->convergence_counter ++;
    }else{
        this->convergence_counter = 0;
    }
    return;
}

bool PoseOffloadWorker::is_finished(){
    if(this->convergence_counter >= sun::MAX_CONVERGENCE_COUNTER/2 && this->convergence_counter < sun::MAX_CONVERGENCE_COUNTER){
        switch (this->area) {
            case 1:
                this->servo_msg.x = 10.0;
                this->servo_msg.y = 120.0;
                this->servo_msg.z = 120.0;
                break;
            case 2:
                this->servo_msg.x = 10.0;
                this->servo_msg.y = 10.0;
                this->servo_msg.z = 120.0;
                break;
            case 3:
                this->servo_msg.x = 10.0;
                this->servo_msg.y = 10.0;
                this->servo_msg.z = 10.0;
                break;
        }
        this->servo_pub.publish(this->servo_msg);
        ROS_INFO("Fire");
        return false;
    }
    else if(this->convergence_counter >= sun::MAX_CONVERGENCE_COUNTER){
        return true;
    }
    else{
        return false;
    }
}

#endif
