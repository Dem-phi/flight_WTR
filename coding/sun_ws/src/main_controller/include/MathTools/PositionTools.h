#ifndef _POSITIONTOOLS_H
#define _POSITIONTOOLS_H

#include <common_parameters.h>

/**
 * @author      :demphi
 * @date        :Created in 8/12/21
 * @brief       :PositionTools provides some function that needed in PositionWorker
 * @param       :geometry_msgs::Pose current_pose, goal_pose
 */
class PositionTools{
public:
    bool IsArrive(geometry_msgs::Pose cur_pose_, geometry_msgs::Pose goal_pose_);
    PositionTools();
    ~PositionTools();

private:
    geometry_msgs::Pose error;
};

PositionTools::PositionTools(){

}

PositionTools::~PositionTools(){
}

/*!
 * @brief to judge if UAV arrive
 * @param cur_pose_
 * @param goal_pose_
 */
bool PositionTools::IsArrive(geometry_msgs::Pose cur_pose_, geometry_msgs::Pose goal_pose_){
    error.position.x = cur_pose_.position.x - goal_pose_.position.x;
    error.position.y = cur_pose_.position.y - goal_pose_.position.y;
    error.position.z = cur_pose_.position.z - goal_pose_.position.z;
    if( abs(error.position.x) > sun::TOLERANCE_X ||
        abs(error.position.y) > sun::TOLERANCE_Y ||
        abs(error.position.z) > sun::TOLERANCE_Z){
        return false;
    }
    return true;
}

#endif
