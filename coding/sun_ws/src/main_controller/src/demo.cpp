#include <FiniteStateMachine.h>
#include "time.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    sleep(3);
    FSM FSMachine(nh); 
/*    FSMachine.build_ScheduleTable(
        sun::TAKEOFF,       1.0,
        sun::DETECTING,     2,                      -1, 5,      sun::DETECTING_CIRCLE,
        sun::DOCKING,       sun::DOCKING_CIRCLE,    3,  120,    100,
        sun::TAKEOFF,       2.0,
        sun::DETECTING,     4,                      -1, 5,      sun::DETECTING_RECTANGLE,
        sun::LANDING,
        sun::POSITION,      sun::GOAL_H,
        sun::END
    );*/
     FSMachine.build_ScheduleTable(

        // Put A, Fire B & C
        sun::TAKEOFF, 0.8,
        sun::POSITION, 1.8, 0.1, 0.3,
        sun::LANDING,
        sun::OFFLOADING, 1,

        sun::TAKEOFF, 1.0,
	sun::POSITION, 1.8, 0.1, 2.3,
        sun::FIRE, 0.0, 2.0, 2.3, 2,
        sun::FIRE, 1.8, 1.8, 2.3, 3,

        sun::POSITION, 0.0, 0.0, 2.3,
        sun::POSITION, 0.0, 0.0, 0.5,
        sun::LANDING,

        // Fire A & B & C
        /*
        sun::TAKEOFF, 2.3,
        sun::FIRE, 2.0, 0.0, 2.3, 1,
        sun::FIRE, 0.0, 2.0, 2,3, 2,
        sun::FIRE, 1.8, 1.8, 2.3, 3,
        sun::POSITION, 0.0, 0.0, 2.3,
        sun::POSITION, 0.0, 0.0, 0.5,
        sun::LANDING,
        */
	
	
        sun::END
    );
    FSMachine.set_timer();

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

/*
int main(int argc, char **argv){
    cv::Mat maps[2];
    cv::Mat src, dst;
    cv::VideoCapture ssss(0);
    modify(maps);
    while(ssss.isOpened()){
        ssss.read(src);
        cv::remap(src, dst, maps[0], maps[1], INTER_LINEAR);
        imshow("front", src);
        imshow("after", dst);
        waitKey(1);
    }
}*/
