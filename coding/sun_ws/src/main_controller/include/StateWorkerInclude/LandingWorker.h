#ifndef _LANDINGWORKER_
#define _LANDINGWORKER_


#include <StateWorker.h>


class LandingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    LandingWorker(ros::NodeHandle &nh);
    ~LandingWorker();
};

LandingWorker::LandingWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

LandingWorker::~LandingWorker()
{
}

void LandingWorker::run(StateInfo state_info){
    cout << "LandingWorker is running" << endl;
    return;
}

bool LandingWorker::is_finished(){
    cout << "LandingWorker is finished" << endl;
    return true;
}

#endif