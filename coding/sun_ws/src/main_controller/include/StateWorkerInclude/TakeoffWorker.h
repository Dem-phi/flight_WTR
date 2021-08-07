#ifndef _TAKEOFFWORKER_
#define _TAKEOFFWORKER_

#include <StateWorker.h>


class TakeoffWorker:public StateWorker
{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    TakeoffWorker(ros::NodeHandle &nh);
    ~TakeoffWorker();
};

TakeoffWorker::TakeoffWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

TakeoffWorker::~TakeoffWorker()
{
}

void TakeoffWorker::run(StateInfo state_info){
    cout << "TakeoffWorker is running" << endl;
    return;
}

bool TakeoffWorker::is_finished(){
    cout << "TakeoffWorker is finished" << endl;
    return true;
}

#endif