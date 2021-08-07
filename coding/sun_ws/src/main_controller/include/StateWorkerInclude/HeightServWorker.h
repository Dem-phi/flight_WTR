#ifndef _HEIGHTSERVWORKER_
#define _HEIGHTSERVWORKER_


#include <StateWorker.h>
#include <PIDcontroller.h>
#include <geometry_msgs/TwistStamped.h>


class HeightServWorker:public StateWorker
{
public:
    ros::NodeHandle nh;

    ros::Publisher pub_vel;
    geometry_msgs::TwistStamped msg_vel;
    float max_speed, Err, expected_height;
    PIDcontroller* model_PID = new PIDcontroller(sun::HEIGHTLOCKING_PID_PARAMS);

    int convergence_counter;
    

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    HeightServWorker(ros::NodeHandle &nh, float expected_height, float max_speed);
    ~HeightServWorker();
};

HeightServWorker::HeightServWorker(ros::NodeHandle &nh, float expected_height, float max_speed){
    this->nh = nh;
    this->max_speed = max_speed;
    this->pub_vel = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10
    );
    this->model_PID->PID_params = sun::HEIGHTLOCKING_PID_PARAMS;
    this->expected_height = expected_height;
    return;
}

HeightServWorker::~HeightServWorker(){
    delete this->model_PID;
}

void HeightServWorker::run(StateInfo state_info){
    this->Err = this->expected_height - state_info.height;
    Vector2f input = this->model_PID->run(Vector2f(this->Err, 0));
    if(input[0] > this->max_speed){
        input[0] = max_speed;
    }
    this->msg_vel.twist.linear.z = input[0];
    publish_vel(&this->pub_vel, this->msg_vel);
    return;
}

bool HeightServWorker::is_finished(){
    if(this->Err < sun::HEIGHTLOCKING_TOLERANCE){
        this->convergence_counter ++;
    }else{
        this->convergence_counter = 0;
    }
    if(this->convergence_counter > sun::MAX_CONVERGENCE_COUNTER){
        return true;
    }
    return false;
}

#endif