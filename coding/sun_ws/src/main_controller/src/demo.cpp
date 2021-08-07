#include <FiniteStateMachine.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;

    FSM FSMachine(nh);
    FSMachine.build_ScheduleTable(
        sun::TAKEOFF,
        sun::DETECTING,
        sun::DOCKING, sun::DOCKING_CIRCLE, 3, 120, 100,
        sun::TAKEOFF,
        sun::DETECTING,
        sun::LANDING,
        sun::END
    );
    FSMachine.set_timer();



    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}