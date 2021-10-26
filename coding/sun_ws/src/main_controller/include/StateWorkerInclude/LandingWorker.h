#ifndef _LANDINGWORKER_
#define _LANDINGWORKER_


#include <StateWorker.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <time.h>

int is_landed_state = 0;
void landCallback(const mavros_msgs::ExtendedStateConstPtr & msg){
    is_landed_state = msg->landed_state;
    return;
}


/* 
Landing. 
@param None
 */

class LandingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    ros::Subscriber sub_land;
    geometry_msgs::TwistStamped msg_vel;

    int convergence = 0;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    LandingWorker(ros::NodeHandle &nh);
    ~LandingWorker();
};

LandingWorker::LandingWorker(ros::NodeHandle &nh){
    this->nh = nh;
    this->pub_vel = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10
    );
    this->sub_land = this->nh.subscribe("/mavros/extended_state", 10, landCallback);
    this->msg_vel.twist.linear.x = 0;
    this->msg_vel.twist.linear.y = 0;
    this->msg_vel.twist.linear.z = -0.3;
    this->is_working = true;
}

LandingWorker::~LandingWorker(){
}

void LandingWorker::run(StateInfo state_info){
    ROS_INFO("Landing!!!!!!!!!!");
    ros::spinOnce();
    this->pub_vel.publish(this->msg_vel);
    if(state_info.cur_pose.position.z < 0.1){
        this->convergence ++;
    }
//    if(state_info.armed == false && state_info.mode == "LAND"){
//        this->is_working = false;
//    }
    return;
}

bool LandingWorker::is_finished() {
//    cout << is_landed_state << endl;
//    if (is_landed_state == 1) {
//        return true;
//    } else {
//        return false;
//    }
    if(this->convergence > 150){
        return true;
    }else{
        return false;
    }
}

#endif