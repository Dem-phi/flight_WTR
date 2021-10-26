#ifndef FINITE_STATE_MACHINE
#define FINITE_STATE_MACHINE


#include <stdarg.h>
#include <workers_common_include.h>
#include <rs_remap.h>
#define _MAX_ 11111

class FSM{
private:
    // main loop function runs in a specific frequency set by timer
    ros::Timer FSM_timer;
    // subscriber
    ros::Subscriber sub_state, sub_vel, sub_rc, sub_position, sub_imu, sub_gogogo, sub_t265_h;
    ros::Publisher servo_load_pub;
    int init_update_flag = 0;

public:
    ros::NodeHandle nh;
    vector<StateWorker*> Workers;
    int flow = 0;
    bool is_imu_init = false;
    int if_load = 0;
    geometry_msgs::Vector3 load_msg;
    StateInfo state_info;
    rs_remap* remaper;

    Quaternionf q_init, q_now;

    void loop(const ros::TimerEvent &);
    void build_ScheduleTable(int Schedule, ...);
    void StateCallback(const mavros_msgs::StateConstPtr &msg);
    void VelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void RCCallback(const mavros_msgs::RCInConstPtr &msg);
    void PositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void ButtonCallback(const std_msgs::BoolConstPtr &msg);
    void ImuCallback(const sensor_msgs::ImuConstPtr &msg);
    void set_timer();

    LandingWorker* emergency_land_worker; 

    FSM(ros::NodeHandle &nh);
    ~FSM();
};

// ===================== Realization =====================


FSM::FSM(ros::NodeHandle &nh){
    this->nh = nh;
    this->emergency_land_worker = new LandingWorker(this->nh);
    this->sub_state = nh.subscribe("/mavros/state", 10, &FSM::StateCallback, this);
    this->sub_imu = nh.subscribe("/mavros/imu/data", 10, &FSM::ImuCallback, this);
    this->sub_vel = nh.subscribe("/mavros/local_position/velocity", 10, &FSM::VelocityCallback, this);
    this->sub_rc = nh.subscribe("/mavros/rc/in", 10, &FSM::RCCallback, this);
    this->sub_position = nh.subscribe("/mavros/vision_pose/pose", 10, &FSM::PositionCallback, this);
    this->sub_gogogo = nh.subscribe("/sun/gogogo", 10, &FSM::ButtonCallback, this);
    this->servo_load_pub = nh.advertise<geometry_msgs::Vector3>("/sun/servo_ctl", 10);
    this->state_info.emergency_land = 100000;
    this->remaper = new rs_remap(nh);
}

FSM::~FSM(){
    // free memory
    for(auto each:this->Workers){
        delete each;
    }
    delete this->emergency_land_worker;
    delete this->remaper;
}

void FSM::set_timer(){
    this->FSM_timer = nh.createTimer(ros::Duration(0.02), &FSM::loop, this); 
    return;   
}

// main loop func, where the selected worker works
void FSM::loop(const ros::TimerEvent &){
//    cout << state_info.cur_pose.orientation.x << " " <<
//    state_info.cur_pose.orientation.y << " " <<
//    state_info.cur_pose.orientation.z << " " <<
//    state_info.cur_pose.orientation.w <<  endl;

/*
    if(
            this->flow == 6  ||
            this->flow == 12 ||
	    this->flow == 18 
            ){ this->remaper->use_t265_height = true; }else{
        this->remaper->use_t265_height = false;
    }
    this->remaper->run(); */
    /*
    if(
	   this->flow == 6 ||
	   this->flow == 7 ||
	   this->flow == 8)
    	   {this->remaper->use_t265_height = true;}
    else{
	    this->remaper->use_t265_height = false;
    }
    this->remaper->run();
    */
    if(this->init_update_flag < 50){
        this->init_update_flag++;
        if(this->init_update_flag == 49){
            this->state_info.init_quater = this->state_info.cur_pose.orientation;
        };
        return;
    }

    // Emergency land
    if(state_info.emergency_land < 1100 ){
        this->emergency_land_worker->run(this->state_info);
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
@param sun::DOCKING, int type, int level 
@param sun::ADAPTIVE_LEARNING, int max_iter, float expected_height
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
                int max_iter = va_arg(arg_ptr, int);
                float expected_height = va_arg(arg_ptr, double);
                Adaptive_LearningWorker* tmp_worker = new Adaptive_LearningWorker(this->nh, max_iter, expected_height);
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
            case sun::DETECTPOSE:{
                int bias = va_arg(arg_ptr, int);
                DetectPoseWorker* tmp_worker = new DetectPoseWorker(this->nh, bias);
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
                int area = va_arg(arg_ptr, int);
                OffloadingWorker* tmp_worker = new OffloadingWorker(this->nh,area);
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
                double x = va_arg(arg_ptr, double);
                double y = va_arg(arg_ptr, double);
                double z = va_arg(arg_ptr, double);
                PositionWorker* tmp_worker = new PositionWorker(this->nh, x, y, z);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
            case sun::FIRE:{
                double x = va_arg(arg_ptr, double);
                double y = va_arg(arg_ptr, double);
                double z = va_arg(arg_ptr, double);
                int area = va_arg(arg_ptr, int);
                PoseOffloadWorker* tmp_worker = new PoseOffloadWorker(this->nh, x, y, z, area);
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
    this->state_info.manual_takeoff = msg->channels[5];
    this->state_info.loading = msg->channels[7];
    this->state_info.load_angle = int((msg->channels[8]-1095)/838 * 180); // limit to [0, 180]

    //limit x,y velocity to [-1, 1]
    this->state_info.vel_info.x = (msg->channels[1]-sun::MID_PITCH)
            /((sun::MAX_PITCH-sun::MIN_PITCH)/2);
    this->state_info.vel_info.y = (msg->channels[0]-sun::MID_ROLL)
            /((sun::MAX_ROLL-sun::MIN_ROLL)/(-2));
    //limit z velocity to [-0.1, 0.1]
    this->state_info.vel_info.z = (msg->channels[2]-sun::MID_THROOTTLE)
            /((sun::MAX_THROOTTLE-sun::MIN_THROOTTLE)*5);
    return;
}

void FSM::PositionCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    this->state_info.cur_pose = msg->pose;
    this->state_info.height = msg->pose.position.z;
    return;
}

void FSM::ImuCallback(const sensor_msgs::ImuConstPtr &msg){
//    if(!this->is_imu_init){
//        this->q_init.x() = msg->orientation.x;
//        this->q_init.y() = msg->orientation.y;
//        this->q_init.z() = msg->orientation.z;
//        this->q_init.w() = msg->orientation.w;
//        this->is_imu_init = true;
//    }
//    this->q_now.x() = msg->orientation.x;
//    this->q_now.y() = msg->orientation.y;
//    this->q_now.z() = msg->orientation.z;
//    this->q_now.w() = msg->orientation.w;
//    Quaternionf q_tmp = q_now*q_init.conjugate();
//    this->state_info.cur_pose.orientation.x = q_tmp.x();
//    this->state_info.cur_pose.orientation.y = q_tmp.y();
//    this->state_info.cur_pose.orientation.z = q_tmp.z();
//    this->state_info.cur_pose.orientation.w = q_tmp.w();

    this->state_info.cur_pose.orientation.x = msg->orientation.x;
    this->state_info.cur_pose.orientation.y = msg->orientation.y;
    this->state_info.cur_pose.orientation.z = msg->orientation.z;
    this->state_info.cur_pose.orientation.w = msg->orientation.w;
    return;
}

void FSM::ButtonCallback(const std_msgs::BoolConstPtr &msg) {
    if (msg->data == true) {
        this->state_info.gogogo = true;
    }

}

#endif
