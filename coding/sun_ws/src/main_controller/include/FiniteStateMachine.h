#ifndef FINITE_STATE_MACHINE
#define FINITE_STATE_MACHINE


#include <stdarg.h>
#include <workers_common_include.h>


class FSM{
private:
    // main loop function runs in a specific frequency set by timer
    ros::Timer FSM_timer;
    // subscriber
    ros::Subscriber sub_state, sub_vel;

public:
    ros::NodeHandle nh;
    vector<StateWorker*> Workers;
    int flow = 0;
    StateInfo state_info;





    void loop(const ros::TimerEvent &);
    void build_ScheduleTable(int Schedule, ...);
    void StateCallback(const mavros_msgs::StateConstPtr &msg);
    void VelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void set_timer();

    FSM(ros::NodeHandle &nh);
    ~FSM();
};




// ===================== Realization =====================
FSM::FSM(ros::NodeHandle &nh){
    this->nh = nh;
    this->sub_state = nh.subscribe("/mavros/state", 10, &FSM::StateCallback, this);
    this->sub_vel = nh.subscribe("/mavros/local_position/velocity", 10, &FSM::VelocityCallback, this);
}

FSM::~FSM(){
    // free memory
    for(auto each:this->Workers){
        delete each;
    }
}

void FSM::set_timer(){
    this->FSM_timer = nh.createTimer(ros::Duration(0.02), &FSM::loop, this); 
    return;   
}

// main loop func, where the selected worker works
void FSM::loop(const ros::TimerEvent &){
    // running by schedule table
    if(this->Workers[this->flow]->is_finished()){
        this->flow ++;
        if(this->flow == this->Workers.size()){
            ROS_INFO("Finish ScheduleTable!");
            exit(0);
        }
    }else{
        this->Workers[this->flow]->run(this->state_info);
    }
    // update state_info
    update_bluetooth_data(&this->state_info);
    update_serial_data(&this->state_info);
}



/* @brief: build schedule table
@param: format: enum sun::STATE_TYPE, arg specific params
@param: sun::DOCKING, int type, int level */
void FSM::build_ScheduleTable(int Schedule, ...){
    va_list arg_ptr;
    va_start(arg_ptr, Schedule);

    while(Schedule != sun::END){
        switch (Schedule)
        {
            case sun::DOCKING:{
                int type = va_arg(arg_ptr, int);
                int level = va_arg(arg_ptr, int);
                int bias_x = va_arg(arg_ptr, int);
                int bias_y = va_arg(arg_ptr, int);
                DockingWorker* tmp_worker = new DockingWorker(this->nh, type, level, bias_x, bias_y);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::ADAPTIVE_LEARNING:{
                Adaptive_LearningWorker* tmp_worker = new Adaptive_LearningWorker(this->nh);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::DETECTING:{
                DetectingWorker* tmp_worker = new DetectingWorker(this->nh);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::LANDING:{
                LandingWorker* tmp_worker = new LandingWorker(this->nh);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::MANUAL:{
                ManualWorker* tmp_worker = new ManualWorker(this->nh);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::OFFLOADING:{
                OffloadingWorker* tmp_worker = new OffloadingWorker(this->nh);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::TAKEOFF:{
                TakeoffWorker* tmp_worker = new TakeoffWorker(this->nh);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::WAITING:{
                WaitingWorker* tmp_worker = new WaitingWorker(this->nh); 
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::HEIGHTSERV:{
                float expected_height = va_arg(arg_ptr, double);
                float max_speed = va_arg(arg_ptr, double);
                HeightServWorker* tmp_worker = new HeightServWorker(this->nh, expected_height, max_speed);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }

            default:
                ROS_ERROR("Wrong type of Schedule Table!");
                exit(0);
                break;
        }
        Schedule = va_arg(arg_ptr, int);
    }
    return;
}

// ================ callback ===================
void FSM::StateCallback(const mavros_msgs::StateConstPtr &msg){
    this->state_info.armed = msg->armed;
    this->state_info.connected = msg->connected;
    this->state_info.mode = msg->mode;
    return;
}

void FSM::VelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg){
    this->state_info.linear = msg->twist.linear;
    this->state_info.angular = msg->twist.angular;
    return;
}






#endif