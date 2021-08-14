#ifndef _MANUALWORKER_
#define _MANUALWORKER_

#include <StateWorker.h>


class ManualWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_vel_;
    geometry_msgs::TwistStamped msg_vel;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();



    ManualWorker(ros::NodeHandle &nh);
    ~ManualWorker();
};

ManualWorker::ManualWorker(ros::NodeHandle &nh){
    this->nh = nh;
    this->pub_vel_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10
            );
    this->is_working = true;
}

ManualWorker::~ManualWorker()
{
}

void ManualWorker::run(StateInfo state_info){
#if IF_USE_MANNUL
    this->msg_vel.header.frame_id = "body";
    this->msg_vel.header.stamp = ros::Time::now();
    this->msg_vel.twist.linear.x = state_info.vel_info.linear_x;
    this->msg_vel.twist.linear.y = state_info.vel_info.linear_y;
    this->msg_vel.twist.linear.z = state_info.vel_info.linear_z;
    this->pub_vel_.publish(msg_vel);
#endif
    cout << "ManualWorker is running" << endl;
    return;
}

bool ManualWorker::is_finished(){
    cout << "ManualWorker is finished" << endl;
    return false;
}

#endif