#include <iostream>
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

using namespace std;

mavros_msgs::State state_info;
void StateCallback(const mavros_msgs::StateConstPtr &msg){
    cout << "Updating State_info" << endl;
    state_info = *msg;
    return;
}

geometry_msgs::PoseStamped takeoff_pose;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;
ros::ServiceClient set_mode_client, arming_client;
ros::Publisher pub_pos;
ros::Subscriber sub_state;


int main(int argc, char** argv){
    ros::init(argc, argv, "tmp_node");
    ros::NodeHandle nh;

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    pub_pos = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    sub_state = nh.subscribe("/mavros/state", 10, StateCallback);

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    takeoff_pose.pose.position.z = 0.2;
    ros::Time last_request;

    ros::Rate rate(20.0);

    while(ros::ok() && !state_info.connected){
        cout << "Waiting for connection" << endl;
        ros::spinOnce();
        rate.sleep();
    }

    for(int i = 0; i < 100; i++){
        cout << "Sending precommands "<< i+1 << endl;
        pub_pos.publish(takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }

    last_request = ros::Time::now();
    while(true){
        if( state_info.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }else{
            if( !state_info.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        pub_pos.publish(takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }






    /* 
        Why I can not type words in Chinese???
        This cpp file is almost the same as the official offboard example. I tried it for figuring out whether this damn problem 
        occurs on our main program and the result is NOT. EVEN this official one can not change the mode to OFFBOARD.
        
        (You can run 'rosrun main_controller tmp' for a test on it)
        The test gives the same conclusion that all of the requests for Mode-Change service respond a bool True but the mode won't
        change with no errors shown on.
        It's for sure or I should suppose that the problem occurs for reason concerning some hardware damege and may be PX4.

        All the clues I can provide you with. Hope you would fix it tonight :). 
    
    
    
     */































    return 0;
}