#ifndef _POSITIONWORKER_H
#define _POSITIONWORKER_H

#include <StateWorker.h>
#include <geometry_msgs/PoseStamped.h>

/*!
 * @brief fly to certain area in position state
 * @param int goal_type, for different goal
 *
 */
class PositionWorker: public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_goal_pose;
    geometry_msgs::PoseStamped goal_pose;
    bool ignore_xy = false;
    bool is_init = false;

    int convergence_counter = 0;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    PositionWorker(ros::NodeHandle &nh, double x, double y, double z);
    ~PositionWorker();
};

PositionWorker::PositionWorker(ros::NodeHandle &nh, double x, double y, double z){
    if(abs(x - 1000) < 10 || abs(y-1000) < 10){
        this->ignore_xy = true;
    }
    this->goal_pose.header.frame_id = "body";
    this->nh = nh;
    this->pub_goal_pose = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
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

PositionWorker::~PositionWorker(){
}

void PositionWorker::run(StateInfo state_info){
    ROS_INFO("Position!!!!!");
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

bool PositionWorker::is_finished(){
    if(this->convergence_counter >= sun::MAX_CONVERGENCE_COUNTER){
        return true;
    }
    else{
        return false;
    }
}

#endif
