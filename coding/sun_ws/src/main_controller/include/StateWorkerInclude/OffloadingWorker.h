#ifndef _OFFLOADINGWORKER_
#define _OFFLOADINGWORKER_

#include <StateWorker.h>


class OffloadingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    OffloadingWorker(ros::NodeHandle &nh);
    ~OffloadingWorker();
};

OffloadingWorker::OffloadingWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

OffloadingWorker::~OffloadingWorker()
{
}

void OffloadingWorker::run(StateInfo state_info){
    cout << "OffloadingWorker is running" << endl;
    return;
}

bool OffloadingWorker::is_finished(){
    cout << "OffloadingWorker is finished" << endl;
    return true;
}

#endif