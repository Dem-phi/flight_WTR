#ifndef _ALWORKER_
#define _ALWORKER_

#include <StateWorker.h>

class Adaptive_LearningWorker:public StateWorker
{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    Adaptive_LearningWorker(ros::NodeHandle &nh);
    ~Adaptive_LearningWorker();
};

Adaptive_LearningWorker::Adaptive_LearningWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

Adaptive_LearningWorker::~Adaptive_LearningWorker(){
}

void Adaptive_LearningWorker::run(StateInfo state_info){
    cout << "Adaptive_LearningWorker is running" << endl;
    return;
}

bool Adaptive_LearningWorker::is_finished(){
    cout << "Adaptive_LearningWorker is finished" << endl;
    return true;
}

#endif