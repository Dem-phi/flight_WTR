#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
ros::Publisher remap;
geometry_msgs::PoseStamped pubPose;
float height;
float last_pub;
bool if_to_obstacle = false, use_block_level = true;


const float obstacle_height_block = 2.0, obstacle_height_circle = 2.0;
const float level_high_block = 1.96, level_low_block = 0.5;
const float level_high_circle = 1.96, level_low_circle = 0.5;

const float init_z = 0.0;

double use_t265_fusion(){

}

#define T265_FUSION
//#define LASER_FUSION
int diff_too_high_cnt = 0;
bool calc_diff_height = true; double diff_hei;
void laserCallback(const std_msgs::Float32 &msg){
    static ros::Time last_time; static float last_z; static float last_eff_z;
    float cur_hei = msg.data;
    ros::Time cur_time = ros::Time::now();
    double dt = (cur_time - last_time).toSec();
    last_time = cur_time;
    float diff_z = (cur_hei - last_z)/dt;
    last_z = cur_hei;

    if(if_to_obstacle){
#ifdef T265_FUSION
        printf("USING T265\n");
        last_eff_z = cur_hei;
        height = cur_hei;
        return;
#endif

#ifdef LASER_FUSION
        printf("USING LASRR\n");
        /* 判断阶跃 */
        if(abs(diff_z) > 0.5){
            return;
        }

        /* */
        float  cur_eff;
        if(use_block_level){
            if(cur_hei < level_low_block){
                cur_eff = cur_hei + obstacle_height_block;
            }
            else{
                cur_eff = cur_hei;
            }
        }else{
            if(cur_hei < level_low_circle){
                cur_eff = cur_hei + obstacle_height_circle;
            }
            else{
                cur_eff = cur_hei;
            }
        }

//    else if (cur_hei > level_high || diff_z < 0){
//        cur_eff = cur_hei;
//    }
        std::cout << abs((cur_eff - last_eff_z) /dt ) << std::endl;
        if(abs((cur_eff - last_eff_z) /dt ) < 3.0)
        {
            diff_too_high_cnt = 0;
            height = cur_eff;
            last_eff_z = cur_eff;
        }else{
            if(diff_too_high_cnt ++ > 3 && cur_hei < 0.5 ){
                last_eff_z = cur_eff;
                height = cur_eff;
            }
        }

#endif

    }
    else{
        last_eff_z = cur_hei;
        height = cur_hei;
        return;
    }
}



void rsCallback(const nav_msgs::Odometry &msg){
    pubPose.pose = msg.pose.pose;
//    if(abs(msg.pose.pose.position.z - height) >= 0.50){
//        if (msg.pose.pose.position.z>height){
//            pubPose.pose.position.z = msg.pose.pose.position.z;
//        }
//        else{
//            pubPose.pose.position.z = height;
//        }
//    }
//    else{
//        pubPose.pose.position.z = height;
//    }
#ifdef T265_FUSION
if(if_to_obstacle){
    if(calc_diff_height){
        diff_hei = msg.pose.pose.position.z - (height -init_z);
        calc_diff_height = false;
    }

    pubPose.pose.position.z = msg.pose.pose.position.z - diff_hei;
    pubPose.header.stamp = ros::Time::now();
    pubPose.header.frame_id = "map";
    remap.publish(pubPose);
    /*std::cout << pubPose.pose.position.z << std::endl;*/

    last_pub = pubPose.pose.position.z;
    return;
}
#endif

    calc_diff_height=true;
    pubPose.pose.position.z = height - init_z;
    pubPose.header.stamp = ros::Time::now();
    pubPose.header.frame_id = "map";
//    float old_x = pubPose.pose.position.x;
//    pubPose.pose.position.x = pubPose.pose.position.y;
//    pubPose.pose.position.y = -old_x;
    remap.publish(pubPose);
    /*std::cout << pubPose.pose.position.z << std::endl;*/
    last_pub = pubPose.pose.position.z;


}

void targetCallback(const std_msgs::StringConstPtr &msg){
    if (msg->data == "obstacle_rec" ||
        msg->data == "target_B_ready" ||
        msg->data == "back_ready"){
        if_to_obstacle = true;
        use_block_level = true;
        return;
    }
    else if( msg->data == "obstacle_circle"||
             msg->data == "target_C_ready" ){
        if_to_obstacle = true;
        use_block_level = false;
        return;
    }
    else{ if_to_obstacle = false; return;}
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"rsRemap");
    ros::NodeHandle nh;
    ros::Subscriber joySub, laser_sub_, target_sub_;
    target_sub_ = nh.subscribe("/sun/target_plan", 1 , targetCallback);
    laser_sub_ = nh.subscribe("/sun/laser_height", 1, laserCallback);
    joySub = nh.subscribe("/camera/odom/sample", 1, rsCallback);
    remap = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);
    ros::spin();
}
