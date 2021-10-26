/*
 * @Author: Demphi
 * @Date: 2021-07-12 13:41:34
 * @email: demphi339413361@outlook.com
 * @Description: Add you want
 * @Github: https://github.com/Dem-phi
 */

#include "ros/ros.h"
#include "signal.h"
#include "stdio.h"
#include "stdlib.h"
#include "std_msgs/Bool.h"

int flag, killa;

void sys_callback(const std_msgs::BoolConstPtr &msg);
pid_t getProcessPidByName (const char *proc_name);

void sys_callback(const std_msgs::BoolConstPtr &msg){
    if(msg->data == false && flag == 0) {
        system("roslaunch /home/demphi/sun_ws_old/src/realsense-ros/realsense2_camera/launch/rs_t265.launch&");
        flag = 1;
    }
    else if(msg->data == true && flag == 1){
        flag = 0;
        killa = getProcessPidByName("/opt/ros/melodic/lib/nodelet/nodelet");
        kill(killa, SIGTERM);
    }
}

pid_t getProcessPidByName (const char *proc_name){
    FILE *fp;
    char buf[100];
    char cmd[200] = {'\0'};
    pid_t pid = -1;
    sprintf(cmd, "pidof %s", proc_name);
    if((fp = popen(cmd, "r")) != NULL){
        if(fgets(buf, 255, fp) != NULL){
            pid = atoi(buf);
        }
    }
    printf("pid-1 = %d \n", (pid-1));
    pclose(fp);
    return (pid-1);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "t265_control");
    ros::NodeHandle nh;
    ros::Subscriber sys_sub_ = nh.subscribe("/sun/t265_ctl", 10, sys_callback);
    ros::spin();
    return 0;
}
