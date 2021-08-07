//
// Created by yunfan on 2020/12/22.
//

#include "common_include.h"
#include "mav_fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_fsm");
    ros::NodeHandle nh("~");
    wtr::MavFsmNode mav(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

