#ifndef _TAKEOFFWORKER_
#define _TAKEOFFWORKER_

#include <StateWorker.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <PIDcontroller.h>


/* 
takeoff and fly to a given height
@param float height
 */
class TakeoffWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    geometry_msgs::TwistStamped msg_vel;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    PIDcontroller* model_PID = new PIDcontroller(sun::HEIGHTLOCKING_PID_PARAMS);

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    ros::ServiceClient set_mode_client, arming_client;

    float expected_height;
    int convergence_counter;


    TakeoffWorker(ros::NodeHandle &nh, float height);
    ~TakeoffWorker();
};

TakeoffWorker::TakeoffWorker(ros::NodeHandle &nh, float height){
    this->nh = nh;
    this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    this->offb_set_mode.request.custom_mode = "OFFBOARD";
    this->arm_cmd.request.value = true;
    this->expected_height = height;
    this->pub_vel = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10
    );
}

TakeoffWorker::~TakeoffWorker(){
    delete this->model_PID;
}

void TakeoffWorker::run(StateInfo state_info){
    ros::Time last_request = ros::Time::now();

    if( state_info.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    }else{
        if( !state_info.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }

    float Err = this->expected_height - state_info.height;
    if(Err < sun::HEIGHTLOCKING_TOLERANCE){
        this->convergence_counter ++;
    }else{
        this->convergence_counter = 0;
    }
    Vector2f input = this->model_PID->run(Vector2f(Err, 0));
    this->msg_vel.twist.linear.z = input[0];
    publish_vel(&this->pub_vel, this->msg_vel);

    return;
}

bool TakeoffWorker::is_finished(){
    if(this->convergence_counter > sun::MAX_CONVERGENCE_COUNTER){
        return true;
    }else{
        return false;
    }
}

#endif