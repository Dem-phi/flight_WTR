#ifndef _LANDINGWORKER_
#define _LANDINGWORKER_


#include <StateWorker.h>
#include <mavros_msgs/State.h>

/* 
Landing. 
@param None
 */
class LandingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    geometry_msgs::TwistStamped msg_vel;


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
    this->msg_vel.twist.linear.x = 0;
    this->msg_vel.twist.linear.y = 0;
    this->msg_vel.twist.linear.z = -0.3;
    this->is_working = true;
}

LandingWorker::~LandingWorker(){
}

void LandingWorker::run(StateInfo state_info){
    ROS_INFO("Landing!!!!!!!!!!");
    this->pub_vel.publish(this->msg_vel);
//    if(state_info.armed == false && state_info.mode == "LAND"){
//        this->is_working = false;
//    }

    return;
}

bool LandingWorker::is_finished(){
    return !is_working;
}

#endif