#ifndef _OFFLOADINGWORKER_
#define _OFFLOADINGWORKER_

#include <StateWorker.h>
#include <time.h>


class OffloadingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher servo_pub;
    geometry_msgs::Vector3 servo_msg;
    int area;
    int limit_HZ;
    bool is_init = false;
    time_t timer;
    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    OffloadingWorker(ros::NodeHandle &nh, int area);
    ~OffloadingWorker();
};

OffloadingWorker::OffloadingWorker(ros::NodeHandle &nh, int area){
    this->nh = nh;
    this->area = area;
    this->servo_pub = nh.advertise<geometry_msgs::Vector3>("/sun/servo_ctl", 10);
}

OffloadingWorker::~OffloadingWorker(){
}

void OffloadingWorker::run(StateInfo state_info){
    this->limit_HZ++;
    if(!this->is_init){
        this->is_init = true;
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
    }
}

bool OffloadingWorker::is_finished(){
    if(this->limit_HZ >= 100){
        cout << "OffloadingWorker is finished" << endl;
        return true;
    }
    return false;
}

#endif