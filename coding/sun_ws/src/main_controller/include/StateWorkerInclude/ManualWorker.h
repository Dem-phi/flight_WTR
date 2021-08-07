#ifndef _MANUALWORKER_
#define _MANUALWORKER_

#include <StateWorker.h>


class ManualWorker:public StateWorker
{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    ManualWorker(ros::NodeHandle &nh);
    ~ManualWorker();
};

ManualWorker::ManualWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

ManualWorker::~ManualWorker()
{
}

void ManualWorker::run(StateInfo state_info){
    cout << "ManualWorker is running" << endl;
    return;
}

bool ManualWorker::is_finished(){
    cout << "ManualWorker is finished" << endl;
    return true;
}

#endif