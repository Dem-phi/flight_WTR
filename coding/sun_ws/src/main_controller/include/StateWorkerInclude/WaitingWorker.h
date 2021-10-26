#ifndef _WAITINGWORKER_
#define _WAITINGWORKER_

#include <StateWorker.h>

/* 
Stay still, this state will never end
@param None
 */
class WaitingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    geometry_msgs::TwistStamped msg_vel;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    WaitingWorker(ros::NodeHandle &nh);
    ~WaitingWorker();
};

WaitingWorker::WaitingWorker(ros::NodeHandle &nh){
    this->nh = nh;
    this->pub_vel = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10
    );
    this->msg_vel.twist.linear.x = 0;
    this->msg_vel.twist.linear.y = 0;
    this->msg_vel.twist.linear.z = 0;
}

WaitingWorker::~WaitingWorker(){
}

void WaitingWorker::run(StateInfo state_info){
    this->pub_vel.publish(this->msg_vel);
    return;
}

bool WaitingWorker::is_finished(){
    return false;
}

#endif