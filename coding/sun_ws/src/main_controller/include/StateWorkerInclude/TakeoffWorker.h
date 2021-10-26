#ifndef _TAKEOFFWORKER_
#define _TAKEOFFWORKER_

#include <StateWorker.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <PIDcontroller.h>

/**
  * @brief: takeoff and fly to a given height
  * @param: float height
 */
class TakeoffWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_vel, pub_pose;
    geometry_msgs::TwistStamped msg_vel;
    geometry_msgs::PoseStamped takeoff_pose;
    ros::Time last_request;
    bool is_init = false;
    int count = 0;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    PIDcontroller* model_PID = new PIDcontroller(sun::HEIGHTLOCKING_PID_PARAMS);

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    ros::ServiceClient set_mode_client, arming_client;

    float expected_height;
    int convergence_counter = 0;


    TakeoffWorker(ros::NodeHandle &nh, float height);
    ~TakeoffWorker();
};

TakeoffWorker::TakeoffWorker(ros::NodeHandle &nh, float height){
    this->nh = nh;
    this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    this->offb_set_mode.request.custom_mode = "OFFBOARD";
    this->arm_cmd.request.value = true;
    this->expected_height = height;
    this->pub_pose = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    this->pub_vel = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10
    );
    this->last_request = ros::Time::now();

}

TakeoffWorker::~TakeoffWorker(){
    delete this->model_PID;
}

void TakeoffWorker::run(StateInfo state_info){
    // Auto takeoff
    if(state_info.gogogo){
        ROS_INFO("Ready to fly");
        // delay 8 seconds
        if(this->count++ >= 750){
            if( state_info.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                this->last_request = ros::Time::now();
                }else{
                if( !state_info.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    this->last_request = ros::Time::now();
                }
            }
        }
    }
    //Maunal control
    if(state_info.manual_takeoff < 1500){
        ROS_INFO("Takeoff!!!!");
        if(!this->is_init){
            this->takeoff_pose.pose.position = state_info.cur_pose.position;
            this->takeoff_pose.pose.orientation = state_info.init_quater;
            this->takeoff_pose.pose.position.z += this->expected_height;
            this->is_init = true;
        }
        /*
         * position control of takeoff mode
         */
        takeoff_pose.header.stamp = ros::Time::now();
        pub_pose.publish(takeoff_pose);
        if( abs(this->takeoff_pose.pose.position.x - state_info.cur_pose.position.x) < sun::POSITION_TOLERANCE_X &&
            abs(this->takeoff_pose.pose.position.y - state_info.cur_pose.position.y) < sun::POSITION_TOLERANCE_Y &&
            abs(this->takeoff_pose.pose.position.z - state_info.cur_pose.position.z) < sun::POSITION_TOLERANCE_Z ){
            this->convergence_counter ++;
        }else{
            this->convergence_counter = 0;
        }
    }

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