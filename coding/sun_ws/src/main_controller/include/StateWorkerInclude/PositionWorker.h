#ifndef _POSITIONWORKER_H
#define _POSITIONWORKER_H

#include <StateWorker.h>
#include <PositionTools.h>

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

    PositionTools* position_tools = new PositionTools();

    int goal_type;
    int convergence_counter;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    PositionWorker(ros::NodeHandle &nh, int goal_type);
    ~PositionWorker();
};

PositionWorker::PositionWorker(ros::NodeHandle &nh, int goal_type_){
    goal_pose.header.frame_id = "body";
    this->nh = nh;
    this->pub_goal_pose = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    this->goal_type = goal_type_;
    this->nh.param<float>("/B_area/x",sun::B_AREA_X,0.0);
    this->nh.param<float>("/B_area/y",sun::B_AREA_Y,-2.2);
    this->nh.param<float>("/C_area/x",sun::C_AREA_X,2.0);
    this->nh.param<float>("/C_area/y",sun::C_AREA_Y,-2.2);
    switch (goal_type) {
        case sun::GOAL_A :
            goal_pose.pose.position.x = sun::A_AREA_X;
            goal_pose.pose.position.y = sun::A_AREA_Y;
            goal_pose.pose.position.z = sun::A_AREA_Z;
            break;
        case sun::GOAL_B :
            goal_pose.pose.position.x = sun::B_AREA_X;
            goal_pose.pose.position.y = sun::B_AREA_Y;
            goal_pose.pose.position.z = sun::B_AREA_Z;
            break;
        case sun::GOAL_C :
            goal_pose.pose.position.x = sun::C_AREA_X;
            goal_pose.pose.position.y = sun::C_AREA_Y;
            goal_pose.pose.position.z = sun::C_AREA_Z;
            break;
        case sun::GOAL_H:
            goal_pose.pose.position.x = 0;
            goal_pose.pose.position.y = 0;
            goal_pose.pose.position.z = 1.0;
            break;
    }
}

PositionWorker::~PositionWorker(){
}

void PositionWorker::run(StateInfo state_info){
    goal_pose.header.stamp = ros::Time::now();
    pub_goal_pose.publish(goal_pose);
    if(position_tools->IsArrive(state_info.cur_pose, goal_pose.pose)){
        this->convergence_counter++;
    }
    else{
        this->convergence_counter=0;
    }

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
