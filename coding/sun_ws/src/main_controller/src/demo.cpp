#include <FiniteStateMachine.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    
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
        sun::TAKEOFF,       1.0,
        sun::LANDING,
        sun::END
    );
    FSMachine.set_timer();



    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}