#ifndef _ALWORKER_
#define _ALWORKER_

#include <StateWorker.h>
#include <PSO.h>
#include <HoughCircle.h>
#include <PIDcontroller.h>
#include <geometry_msgs/TwistStamped.h>
#include <time.h>


class Adaptive_LearningWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    geometry_msgs::TwistStamped msg_vel;
    int adaptive_type, max_iter, iteration = 1, convergence_counter = 0, particle_flow = 0;
    PSO* model_PSO = new PSO(sun::MIN_PID_PARAMS, sun::MAX_PID_PARAMS);
    HoughCircle* model_HOUGH = new HoughCircle();
    
    vector<PIDcontroller> PIDcontrollers;
    vector<float> run_t;
    vector<Vector3f> run_param;
    time_t timer = 0;

    float expected_height = 0.3;
    Vector2f max_Err = Vector2f(0,0);
    bool is_setting_bias = false;

    cv::VideoCapture camera;


    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    void giveBias();
    void clear_msg_vel();
    void update_PID_controller();
    Adaptive_LearningWorker(ros::NodeHandle &nh, int adaptive_type, int max_iter);
    ~Adaptive_LearningWorker();
};

Adaptive_LearningWorker::Adaptive_LearningWorker(ros::NodeHandle &nh, int adaptive_type, int max_iter){
    this->nh = nh;
    this->adaptive_type = adaptive_type;
    this->max_iter = max_iter;
    this->run_param = this->model_PSO->local_best_params;
    for(auto PID_param:this->run_param){
        this->PIDcontrollers.push_back(PIDcontroller(PID_param));
    }
    this->clear_msg_vel();
    this->camera = cv::VideoCapture(sun::camera_port);
    this->pub_vel = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10
    );
}

Adaptive_LearningWorker::~Adaptive_LearningWorker(){
    delete this->model_HOUGH;
    delete this->model_PSO;
}

void Adaptive_LearningWorker::run(StateInfo state_info){
    if(this->timer == 0){
        this->timer = clock();
    }
    // single particle PID controller
    // when reaching convergence
    float time_used = (clock()-this->timer)/(float)CLOCKS_PER_SEC;
    // if reach convergence or use more then 5 seconds
    if(this->convergence_counter >= sun::MAX_CONVERGENCE_COUNTER || time_used > 5.0){
        float time_cost = time_used;
        float fluctuation_cost = this->max_Err.norm();
        // count cost function
        this->run_t.push_back(fluctuation_cost/10.0 + 2*time_cost);
        // update particle flow and PID params vector if all the particles have already moved
        if(this->particle_flow != this->model_PSO->particle_num - 1){
            this->particle_flow ++;
        }else{
            this->particle_flow = 0;
            this->run_param = this->model_PSO->run(this->run_t, this->run_param);
            this->update_PID_controller();
        }
        this->max_Err[0] = 0;
        this->max_Err[1] = 0;
        this->convergence_counter = 0;
        // when learning height locating params, the flight will wander between 0.3 m and 0.5 m
        if(this->expected_height == 0.3){ this->expected_height = 0.5; }
        else{ this->expected_height = 0.3; }
        this->timer = clock();
        publish_vel(&this->pub_vel, this->msg_vel);
        cout << "===========================================================" << endl;
        cout << "iteration: " << this->iteration << "/" << this->max_iter << "\tparticle flow: " << this->particle_flow+1 << "/" << this->model_PSO->particle_num << endl;
        cout << "best cost: " << this->model_PSO->global_best_t << "\tPID params: " 
        << this->model_PSO->global_best_params[0] << " " 
        << this->model_PSO->global_best_params[1] << " " 
        << this->model_PSO->global_best_params[2] << endl; 
        cout << "===========================================================" << endl;
    }
    // when not reaching convergence
    else if(this->convergence_counter < sun::MAX_CONVERGENCE_COUNTER){
        float Err = this->expected_height - state_info.height;
        if(Err > this->max_Err[0]){
            this->max_Err[0] = Err;
        }
        if(Err < sun::HEIGHTLOCKING_TOLERANCE){
            this->convergence_counter ++;
        }else{
            this->convergence_counter = 0;
        }
        Vector2f input = this->PIDcontrollers[this->particle_flow].run(Vector2f(Err, 0));
        this->msg_vel.twist.linear.z = input[0];
        publish_vel(&this->pub_vel, this->msg_vel);
        cout << "\033[1A";
        cout << "\033[K";
        cout << "time used: " << time_used << "\tmax Err: " << this->max_Err << endl;
    }
    this->iteration ++;
}

bool Adaptive_LearningWorker::is_finished(){
    if(this->iteration >= this->max_iter){
        this->camera.release();
        return true;
    }else{
        return false;
    }
}

void Adaptive_LearningWorker::clear_msg_vel(){
    this->msg_vel.twist.linear.x = 0;
    this->msg_vel.twist.linear.y = 0;
    this->msg_vel.twist.linear.z = 0;
    this->msg_vel.twist.angular.x = 0;
    this->msg_vel.twist.angular.y = 0;
    this->msg_vel.twist.angular.z = 0;
    return;
}

void Adaptive_LearningWorker::update_PID_controller(){
    for(int i = 0; i > this->PIDcontrollers.size(); i++){
        this->PIDcontrollers[i].PID_params = this->run_param[i];
    }
    return;
}

#endif

