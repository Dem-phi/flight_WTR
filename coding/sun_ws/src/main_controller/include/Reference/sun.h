/*
 * @Author: Demphi
 * @Date: 2021-06-25 21:53:08
 * @Email: demphi339413361@outlook.com
 * @Function: Do not edit
 * @YouWant: add you want
 */
//#ifndef MAIN_CONTROLLER_MAV_FSM_H
//#define MAIN_CONTROLLER_MAV_FSM_H
//#include "common_include.h"
//#include "calculate.h"
//#include "mavros_msgs/AttitudeTarget.h"
//#include "geometry_msgs/Quaternion.h"
//#include "geometry_msgs/Vector3.h"
//#include "sensor_msgs/BatteryState.h"
//#include "recognition/circles_msg.h"
//#include "mavros_msgs/ExtendedState.h"
//#include "tf/tf.h"
//#include "std_msgs/Float32MultiArray.h"
//#include "mavros_msgs/ExtendedState.h"
//#include "mavros_msgs/State.h"
//#include "mavros_msgs/CommandBool.h"
//#include "mavros_msgs/SetMode.h"
//#include "std_msgs/String.h"
//
//namespace sun{
//    // callback funtion
///*    class MavFsmNode{
//        private:
//        public:
//    };*/
//    class MAV{
//    private:
//        State* cur_state;
//        geometry_msgs::Pose init_pose_, cur_pose_;
//        typedef enum{
//            takeoff,
//            pose,
//            velocity,
//            land,
//            waiting,
//        } workstate_t;
//        workstate_t work_state_;
////        typedef enum {
////            target_A,
////            obstacle_rec,
////            target_B,
////            obstacle_circle,
////            target_C,
////            back,
////        }goal_t;
////        goal_t plan_target;
//        void rcCallback();
//    public:
//        MAV(){};
//        void update();
//        void Execute(Goal* newGoal);
//        void ChangeState(Goal* newGoal);
//    ;
//
//    class Goal{
//    public:
//        virtual ~Goal(){};
////            virtual void JudgeTarget(MAV*) = 0;
//        virtual void StateEnter(MAV*) = 0;
//        virtual void StateExit(MAV*) = 0;
//        virtual void Execute(MAV*) = 0;
//    };
//
//    class position_A: public Goal{
//    public:
//        position_A(){};
////            virtual void JudgeTarget(MAV* mav);
//        virtual void StateEnter(MAV* mav);
//        virtual void StateExit(MAV* mav);
//        virtual void Execute(MAV* mav);
//    };
//
//    class position_B: public Goal{
//    public:
//        position_B(){};
//
//    };
//}

#ifndef MAIN_CONTROLLER_MAV_FSM_H
#define MAIN_CONTROLLER_MAV_FSM_H

#include "common_include.h"
#include "calculate.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/BatteryState.h"
#include "recognition/circles_msg.h"
#include "mavros_msgs/ExtendedState.h"
#include "tf/tf.h"
#include "std_msgs/Float32MultiArray.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "std_msgs/String.h"

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
        int change_state_times;
        int if_on_ground;
        int test_times;
        int rc_msg_land_;
        bool is_init_pose_;
        bool confirm_X, is_obstacle_init_ = false;
        std_msgs::String remap_target_state;
        float different_vector[6], different_z[2], offload_height;
        std_msgs::Bool L515Switch_on;-
        mavros_msgs::State cur_state;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        tf::Quaternion RQ2;
        std_msgs::Bool switch_on;   // to confirm is B or not
        ros::NodeHandle node_;
        ros::Subscriber vision_sub_, battery_sub_, rc_sub_, odom_sub_, imu_sub_, rgbd_sub_, rec_sub_, circle_sub_, realworld_points_sub_, state_sub_, if_on_ground_sub_;
        ros::Publisher local_pos_pub_, speed_pub_, pose_pub_, B_confirm_pub_, servo_pub_, L515Switch_pub_, remap_pub_;
        ros::ServiceClient arming_client, set_mode_client;
        ros::Timer mav_fsm_;
        geometry_msgs::Pose init_pose_, cur_pose_;
        geometry_msgs::PoseStamped takeoff_pose_, target_pose_;
        geometry_msgs::TwistStamped land_signal_, vel_signal_, plan_signal_;
        geometry_msgs::Vector3 circle_position, servo_state;
        geometry_msgs::Vector3 std_circle_position_0, std_circle_position_1, std_circle_position_2;


        struct if_use_yaml_t{
            bool if_use_detection;
            bool if_use_rec;
            bool if_use_circle;
            bool if_use_B;
            bool if_use_C;
        }if_use_yaml;

        struct circles_docker_t{
            std_msgs::Float32MultiArray x;
            std_msgs::Float32MultiArray y;
            bool is_ready = true;
        }circles_docker;

        struct goalPose_t {
            geometry_msgs::Pose detection_pose;
            geometry_msgs::Pose A_pose;
            geometry_msgs::Pose obstacle_rec_pose;
            geometry_msgs::Pose B_pose;
            geometry_msgs::Pose obstacle_circle_pose;
            geometry_msgs::Pose C_pose;
            geometry_msgs::Pose H_pose;
            geometry_msgs::Pose error_pose;
        } goalPose;

        struct realworld_offload_point_t{
            geometry_msgs::Vector3 rw_A;
            geometry_msgs::Vector3 rw_B;
            geometry_msgs::Vector3 rw_C;
        } realworld_offload_points;

        struct vel_mode_t {
            int vel_x;
            int vel_y;
            int vel_z;
            double rotate;
        } vel_param;

        struct desired_t {
            Vec3 cPosition;
            Vec3 cPositionD;
            Vec3 cPositionDD;

            Mat33 rotation;
            Vec3 bodyRate;
            double thrust;
            double yaw;
            double yawD;
        } desired;

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

        struct feedback_t {
            Vec3 cPosition;
            Vec3 cPositionD;
            Vec3 cPositionDD;

            Mat33 rotation;
            Vec3 bodyRate;
            double thrust;
            double yaw;
            double yawD;
        } feedback;

        struct output_t {
            double force;
            Eigen::Quaterniond quad;
            geometry_msgs::Quaternion orientation;
        } output;

        struct fly_params_t {
            Vec3 k_position, k_velocity, i_position;
            Vec4 limit_vel_max;
            double k_thrust;
        } fp;

        typedef enum {
            takeoff,
            pose,
            velocity,
            land,
            waiting,
            PID_debug,
        } workstate_t;
        workstate_t work_state_;

        typedef enum {
            detection,
            detection_finish,
            target_A,     // z=1
            target_A_2,
            target_A_3,
            target_A_UP,  // z=2
            obstacle_rec, // z=2
            target_B_ready,
            target_B,
            target_B_2,
            target_B_3,
            target_B_UP,  // z=2
            obstacle_circle,
            target_C_ready,
            target_C,     // z=1
            target_C_2,
            target_C_3,
            target_C_UP,  // z=2
            back_ready,
            back,          // z=1
            aiming_level, // z = aiming_height_level.level()
            offload_A,
            offload_B,
            offload_C,
        } target_t;
        target_t plan_target, temp_target;

        void rcCallback(const mavros_msgs::RCInConstPtr &msg);

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);

        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

        void batCallback(const sensor_msgs::BatteryStateConstPtr &msg);

        void stateCallback(const mavros_msgs::StateConstPtr &msg);

        void extented_stateCallback(const mavros_msgs::ExtendedStateConstPtr &msg);

        void realworldCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

        void main_FSM(const ros::TimerEvent &);

        /*  Define the rc of vision  */
/*        void rgbdCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);      //L515 info*/
        void visionCallback(const geometry_msgs::Vector3::ConstPtr &msg);    //camera underneath
/*
        void rec_resultCallback(const std_msgs::Bool::ConstPtr &msg);
*/


        /*  Define the mode callback functions  */
        void vel_mode();

        void pos_mode();

        void land_mode();

        void takeoff_mode();

        void wait_mode();


    public:
        MavFsmNode(ros::NodeHandle &nh);

        ~MavFsmNode() {};
        typedef std::shared_ptr<MavFsmNode> Ptr;
    };

    class Aiming_height_level {
    public:
        Aiming_height_level(){};
        ~Aiming_height_level(){};
        float levels[3] = {1.0, 0.5, 0.3};
        int index = 0;
        float level(){
            return this->levels[this->index];
        }
        void next_level(){
            index++;
        }
        void back(){
            index = 0;
        }
    };
}
#endif //MAIN_CONTROLLER_MAV_FSM_H
