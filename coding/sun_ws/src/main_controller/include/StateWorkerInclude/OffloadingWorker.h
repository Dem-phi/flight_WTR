#ifndef _OFFLOADINGWORKER_
#define _OFFLOADINGWORKER_

#include <StateWorker.h>


class OffloadingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher servo_pub;
    geometry_msgs::Vector3 servo_msg;
    int area;
    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    OffloadingWorker(ros::NodeHandle &nh, int _area);
    ~OffloadingWorker();
};

OffloadingWorker::OffloadingWorker(ros::NodeHandle &nh, int _area){
    this->nh = nh;
    this->area = _area;
    this->servo_pub = nh.advertise<geometry_msgs::Vector3>("/sun/servo_ctl", 10);
}

OffloadingWorker::~OffloadingWorker()
{
}

void OffloadingWorker::run(StateInfo state_info){
    cout << "OffloadingWorker is running" << endl;
    switch (area) {
        case 1:
            servo_msg.x = 1;
            break;
        case 2:
            servo_msg.y = 1;
            break;
        case 3:
            servo_msg.z = 1;
            break;
    }
    servo_pub.publish(servo_msg);
    return;
}

bool OffloadingWorker::is_finished(){
    cout << "OffloadingWorker is finished" << endl;
    return true;
}

#endif