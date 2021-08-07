#ifndef _WAITINGWORKER_
#define _WAITINGWORKER_

#include <StateWorker.h>


class WaitingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    WaitingWorker(ros::NodeHandle &nh);
    ~WaitingWorker();
};

WaitingWorker::WaitingWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

WaitingWorker::~WaitingWorker()
{
}

void WaitingWorker::run(StateInfo state_info){
    cout << "WaitingWorker is running" << endl;
    return;
}

bool WaitingWorker::is_finished(){
    cout << "WaitingWorker is finished" << endl;
    return true;
}

#endif