#ifndef MAIN_CONTROLLER_MAV_FSM_H
#define MAIN_CONTROLLER_MAV_FSM_H

#include "common_include.h"
#include "calculate.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/BatteryState.h"
//#include "recognition/circles_msg.h"
#include "mavros_msgs/ExtendedState.h"
#include "tf/tf.h"
#include "std_msgs/Float32MultiArray.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "std_msgs/String.h"
#include <iostream>

#define enum_to_string(x) #x
#define poseMode_test_times 150
#define grasp_servo_A 70
#define grasp_servo_B 20
#define grasp_servo_C 150
#define release_servo_A 100
#define release_servo_B 80
#define release_servo_C 100
#define mode_info_interval 3
namespace wtr {
    class MavFsmNode {
    private:
        int cnt, v_cnt;
        int if_on_ground;
        int test_times;
        int rc_msg_land_;
        bool is_init_pose_;
        std_msgs::String remap_target_state, t265_ctl_state;
        mavros_msgs::State cur_state;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        ros::NodeHandle node_;
        ros::Subscriber extended_state_sub_, vision_sub_, battery_sub_, rc_sub_, rec_sub_, circle_sub_, state_sub_;
        ros::Publisher t265_ctl_pub_,local_pos_pub_, speed_pub_, pose_pub_, servo_pub_, remap_pub_;
        ros::ServiceClient arming_client, set_mode_client;
        ros::Timer mav_fsm_;
        geometry_msgs::Pose init_pose_, cur_pose_;
        geometry_msgs::PoseStamped takeoff_pose_, target_pose_;
        geometry_msgs::TwistStamped land_signal_, vel_signal_;
        geometry_msgs::Vector3 circle_position, servo_state;
        geometry_msgs::Vector3 std_circle_position_0, std_circle_position_1, std_circle_position_2;


        struct if_use_yaml_t{
            bool if_use_B;
            bool if_use_C;
        }if_use_yaml;

        struct goalPose_t {
            geometry_msgs::Pose A_pose;
            geometry_msgs::Pose B_pose;
            geometry_msgs::Pose C_pose;
            geometry_msgs::Pose H_pose;
            geometry_msgs::Pose error_pose;
        } goalPose;

        struct tolerance_t {
            double vision_x;
            double vision_y;
            double pose_x;
            double pose_y;
            double pose_z;
            double angle_x;
            double angle_y;
            double angle_z;
            double angle_w;
        }tolerance;

        struct fly_params_t {
            Vec3 k_position, k_velocity, i_position;
            Vec4 limit_vel_max;
            double k_thrust;
        } fp;

        typedef enum {
            takeoff = 0,
            pose,
            velocity,
            land,
            waiting,
        } workstate_t;
        workstate_t work_state_;

        typedef enum {
            rc_control = 0,
            target_A,
            target_B,
            target_C,
            back,
        } target_t;
        target_t plan_target;

        /* self Function */
//      void InitAndInfo(int count, workstate_t* t_state);

        std_msgs::String Pub_remap_msg(target_t t_msg){
            remap_target_state.data = enum_to_string(t_msg);
            remap_pub_.publish(remap_target_state);
        }

        geometry_msgs::Pose update_error(geometry_msgs::Pose cur_pos_, geometry_msgs::Pose set_pos_) const{
            geometry_msgs::Pose error;
            error.position.x = cur_pose_.position.x - set_pos_.position.x;
            error.position.y = cur_pose_.position.y - set_pos_.position.y;
            error.position.z = cur_pose_.position.z - set_pos_.position.z;
            return error;
        }

        bool if_in_tolerance(geometry_msgs::Pose error_pos_) const{
            if( abs(error_pos_.position.x) > tolerance.pose_x ||
                abs(error_pos_.position.y) > tolerance.pose_y ||
                abs(error_pos_.position.z) > tolerance.pose_z){
                return false;
            }
            return true;
        }

        void InitAndSwitchMode(int *t_time ,workstate_t *t_state, target_t *t_target){
            *t_time = 0;
            if(*t_state == land){
                *t_state = takeoff;
                ROS_INFO("Change to \033[35mTake off Mode\033[0m.");
                switch (*t_target) {
                    case target_A:
                        *t_target = target_B;
                        ROS_INFO("\033[31m Target to B Area\033[0m.");
                        break;
                    case target_B:
                        *t_target = target_C;
                        ROS_INFO("\033[31m Target to C Area\033[0m.");
                        break;
                    case target_C:
                        *t_target = back;
                        ROS_INFO("\033[31m Target to Home\033[0m.");
                        break;
                    default:
                        ROS_INFO("Error! Wrong target");
                        break;
                }
            }
            else{
                ROS_INFO("Error! Use wrong function!");
                return;
            }
        }

        void InitAndSwitchMode(int *t_time, workstate_t *t_state){
            *t_time = 0;
            switch (*t_state) {
                case pose:
                    ROS_INFO("Change to \033[35mVelocity Mode\033[0m.");
                    *t_state = velocity;
                    break;
                case velocity:
                    ROS_INFO("Change to \033[35mLand Mode\033[0m.");
                    *t_state = land;
                    break;
                case takeoff:
                    ROS_INFO("Change to \033[35mPose Mode\033[0m.");
                    *t_state = pose;
                    break;
                case waiting:
                    ROS_INFO("Change to \033[35mTake off Mode\033[0m.");
                    break;
                default:
                    ROS_INFO("Error! Use wrong function!");
                    break;
            }
        }

        /* world to relative transformer of targets*/
        geometry_msgs::Pose World2Relative(geometry_msgs::Pose world_pose_, geometry_msgs::Pose axis_pose_){
            geometry_msgs::Pose relative_pose;
            relative_pose.position.x = world_pose_.position.x - axis_pose_.position.x;
            relative_pose.position.y = world_pose_.position.y - axis_pose_.position.y;
            return relative_pose;
        }


        /* Callback Functions */
        void rcCallback(const mavros_msgs::RCInConstPtr &msg);

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void batCallback(const sensor_msgs::BatteryStateConstPtr &msg);

        void stateCallback(const mavros_msgs::StateConstPtr &msg);

        void extented_stateCallback(const mavros_msgs::ExtendedStateConstPtr &msg);

        void visionCallback(const geometry_msgs::Vector3::ConstPtr &msg);    //camera underneath

        /* Main */
        void main_FSM(const ros::TimerEvent &);

        /*  Define the mode functions  */
        void vel_mode();

        void pos_mode();

        void land_mode();

        void takeoff_mode();

        void wait_mode();

        void adaptive_learning();

    public:
        MavFsmNode(ros::NodeHandle &nh);

        ~MavFsmNode() {};
        typedef std::shared_ptr<MavFsmNode> Ptr;
    };

}
#endif //MAIN_CONTROLLER_MAV_FSM_H
