#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>

#define USE_LASER 0

ros::Publisher remap;
geometry_msgs::PoseStamped pubPose;
#if USE_LASER
std_msgs::Int32 init_height;
#endif

void rsCallback(const nav_msgs::Odometry &msg){
    pubPose.pose = msg.pose.pose;
    //pubPose.pose.position.x = -msg.pose.pose.position.x;
    //pubPose.pose.position.y = -msg.pose.pose.position.y;
    pubPose.header.stamp = ros::Time::now();
    pubPose.header.frame_id = "map";
#if USE_LASER
#else
    remap.publish(pubPose);
#endif
}
#if USE_LASER
void laserCallback(const std_msgs::Int32 &msg){
    init_height.data = 100;
    pubPose.pose.position.z = float((msg.data - init_height.data))/1000;
   	remap.publish(pubPose);
}
#endif
int main(int argc, char ** argv){
    ros::init(argc,argv,"rsRemap");
    ros::NodeHandle nh;
    sleep(3);
    ros::Subscriber joySub, laserSub;
    joySub = nh.subscribe("/camera/odom/sample", 1, rsCallback);
#if USE_LASER
    laserSub = nh.subscribe("/sun/height", 1, laserCallback);
#endif
    remap = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);
    ros::spin();
}
