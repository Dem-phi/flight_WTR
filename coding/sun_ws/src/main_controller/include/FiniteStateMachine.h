#ifndef FINITE_STATE_MACHINE
#define FINITE_STATE_MACHINE


#include <stdarg.h>
#include <workers_common_include.h>
#define _MAX_ 11111

class FSM{
private:
    // main loop function runs in a specific frequency set by timer
    ros::Timer FSM_timer;
    // subscriber
    ros::Subscriber sub_state, sub_vel, sub_rc, sub_position;

public:
    ros::NodeHandle nh;
    vector<StateWorker*> Workers;
    int flow = 0;
    StateInfo state_info;

    void loop(const ros::TimerEvent &);
    void build_ScheduleTable(int Schedule, ...);
    void StateCallback(const mavros_msgs::StateConstPtr &msg);
    void VelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void RCCallback(const mavros_msgs::RCInConstPtr &msg);
    void PositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void set_timer();

    FSM(ros::NodeHandle &nh);
    ~FSM();
};

// ===================== Realization =====================
FSM::FSM(ros::NodeHandle &nh){
    this->nh = nh;
    this->sub_state = nh.subscribe("/mavros/state", 10, &FSM::StateCallback, this);
    this->sub_vel = nh.subscribe("/mavros/local_position/velocity", 10, &FSM::VelocityCallback, this);
    this->sub_rc = nh.subscribe("/mavros/rc/in", 10, &FSM::RCCallback, this);
    this->sub_position = nh.subscribe("/mavros/vision_pose/pose", 10, &FSM::PositionCallback, this);
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
    // Emergency land
    if(state_info.emergency_land < 1100 ){
        if(!this->Workers.empty() && flow < _MAX_){
            this->Workers.clear();
            LandingWorker* tmp_worker = new LandingWorker(this->nh);
            this->Workers.push_back((StateWorker*)tmp_worker);
            // Only enter this for once
            flow = _MAX_;
        }
        this->Workers[0]->run(this->state_info);
        return;
    }
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



/* @brief build schedule table
@param format: enum sun::STATE_TYPE, arg specific params
@param sun::DOCKING, int type, int level 
@param sun::ADAPTIVE_LEARNING, int adaptive_type, int max_iter
@param sun::DETECTING, float direction_vector_x, float direction_vector_y, float max_sec, int figure_type
@param sun::LANDING, None
@param sun::TAKEOFF, float height
@param sun::WAITING, None
@param sun::HEIGHTSERV, float expected_height, float max_speed
*/
void FSM::build_ScheduleTable(int Schedule, ...){
    va_list arg_ptr;
    va_start(arg_ptr, Schedule);
    while(Schedule != sun::END){
        switch (Schedule){
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
                int adaptive_type = va_arg(arg_ptr, int);
                int max_iter = va_arg(arg_ptr, int);
                Adaptive_LearningWorker* tmp_worker = new Adaptive_LearningWorker(this->nh, adaptive_type, max_iter);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::DETECTING:{
                float x = va_arg(arg_ptr, double);
                float y = va_arg(arg_ptr, double);
                float max_sec = va_arg(arg_ptr, double);
                int type = va_arg(arg_ptr, int);
                DetectingWorker* tmp_worker = new DetectingWorker(this->nh, x, y, max_sec, type);
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
                float height = va_arg(arg_ptr, double);
                TakeoffWorker* tmp_worker = new TakeoffWorker(this->nh, height);
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
            case sun::POSITION:{
                int type = va_arg(arg_ptr, int);
                PositionWorker* tmp_worker = new PositionWorker(this->nh, type);
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

void FSM::RCCallback(const mavros_msgs::RCInConstPtr &msg){
    this->state_info.emergency_land = msg->channels[6];
#if IF_USE_MANNUL
    this->state_info.manual_takeoff = msg->channels[5];
    //limit x,y velocity to [-1, 1]
    this->state_info.vel_info.linear_x = (msg->channels[1]-sun::MID_PITCH)
            /((sun::MAX_PITCH-sun::MIN_PITCH)/2);
    this->state_info.vel_info.linear_y = (msg->channels[0]-sun::MID_ROLL)
            /((sun::MAX_ROLL-sun::MIN_ROLL)/(-2));
    //limit z velocity to [-0.1, 0.1]
    this->state_info.vel_info.linear_z = (msg->channels[2]-sun::MID_THROOTTLE)
            /((sun::MAX_THROOTTLE-sun::MIN_THROOTTLE)*5);
#endif
    return;
}

void FSM::PositionCallback(const geometry_msgs::PoseStampedConstPtr &msg){
#if IF_USE_T265
    this->state_info.cur_pose = msg->pose;
    return;
#endif
}


#endif