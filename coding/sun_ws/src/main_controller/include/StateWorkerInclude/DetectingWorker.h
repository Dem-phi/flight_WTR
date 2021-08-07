#ifndef _DETECTINGWORKER_
#define _DETECTINGWORKER_

#include <StateWorker.h>


class DetectingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    DetectingWorker(ros::NodeHandle &nh);
    ~DetectingWorker();
};

DetectingWorker::DetectingWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

DetectingWorker::~DetectingWorker(){
}

void DetectingWorker::run(StateInfo state_info){
    cout << "DetectingWorker is running" << endl;
    return;
}

bool DetectingWorker::is_finished(){
    cout << "DetectingWorker is finished" << endl;
    return true;
}

#endif