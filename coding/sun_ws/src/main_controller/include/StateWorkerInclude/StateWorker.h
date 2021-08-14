#ifndef _STATEWORKER_
#define _STATEWORKER_


#include <iostream>
#include <ros/ros.h>
#include <vector>
#include "Reference/common_include.h"
#include <common_defination.h>
#include <common_parameters.h>
#include <MsgJar.h>


using namespace std;

class StateWorker{
public:
    ros::NodeHandle nh;
    virtual void run(StateInfo state_info) = 0;
    virtual bool is_finished() = 0;
    bool is_working = false;

    StateWorker(){};
};

#endif