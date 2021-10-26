#ifndef _DOCKINGWORKER_
#define _DOCKINGWORKER_


#include <StateWorker.h>
#include <PIDcontroller.h>
#include <PPS.h>
#include <MMCS.h>
#include <PFS.h>
#include <HoughCircle.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/TwistStamped.h>



using namespace std;


/* 
@brief Docking worker can locate the center of the given figure accurately and finish the docking work.
@param ros::NodeHandle &nh; 
@param int type; enum param deciding the docking category of figure
@param int level: multi-level selection, you can choose 3, 2, 1, Default 1 when receiving wrong command
@param int bias_x; bias vector. Docking worker will try to attach image_center+bias_vector to figure_center
@param int bias_y;
 */
class DockingWorker:public StateWorker{
public:
    
    virtual void run(StateInfo state_info);
    virtual bool is_finished();


    ros::Publisher pub_vel;
    geometry_msgs::TwistStamped msg_velocity;

    cv::VideoCapture* camera = new cv::VideoCapture;

    PFS model_PFS;
    HoughCircle model_Hough;
    Vector2f bias; 
    Vector2f Err;
    PIDcontroller* model_PID = new PIDcontroller(sun::MULTILEVEL_PID_PARAMS_LEVEL_1);
    int type, level, convergence_counter;
    vector<int> schedule;
    DockingWorker(ros::NodeHandle &nh, int type, int level, int bias_x, int bias_y);
    ~DockingWorker();
};


DockingWorker::DockingWorker(ros::NodeHandle &nh, int type, int level, int bias_x, int bias_y){
    this->nh = nh;
    this->bias[0] = bias_x; this->bias[1] = bias_y;
    this->type = type;
    switch (level)
    {
        case 1:
            this->model_PID->PID_params = sun::MULTILEVEL_PID_PARAMS_LEVEL_1;
            break;
        case 2:
            this->model_PID->PID_params = sun::MULTILEVEL_PID_PARAMS_LEVEL_2;
            break;
        case 3:
            this->model_PID->PID_params = sun::MULTILEVEL_PID_PARAMS_LEVEL_3;
            break;
        default:{
            this->model_PID->PID_params = sun::MULTILEVEL_PID_PARAMS_LEVEL_1;
            break;
        }
    }
    this->pub_vel = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10
    );
    this->camera = NULL;
}

DockingWorker::~DockingWorker(){
    delete this->camera;
    delete this->model_PID;
}



void DockingWorker::run(StateInfo state_info){
    if(this->camera == NULL){
        *this->camera = cv::VideoCapture(sun::camera_port);
    }
    cv::Mat src;
    // if camera cannot be opened, staying still
    if(!this->camera->read(src)){
        ROS_ERROR("Camera frame reading error!");
        this->msg_velocity.twist.linear.x = 0;
        this->msg_velocity.twist.linear.y = 0;
        this->msg_velocity.twist.linear.z = 0;
        this->pub_vel.publish(this->msg_velocity);
    }
    // Use poly-filter method to locate the center of the rectangle and Err
    // The Err vector should point the real center from the expected center because camera image brings a contradictive result
    if(type == sun::DOCKING_RECTANGLE){
        this->model_PFS.rectangle_detect(src);
        Vector2f img_center = Vector2f(model_PFS.src.cols/2, model_PFS.src.rows/2);
        this->Err = -(img_center+bias) + this->model_PFS.centers[0];
    }
    // Use PPS & MMCS to locate the center of circle and Err
    else if(type == sun::DOCKING_CIRCLE){
        // locate center

        PPS model_PPS(src);
        vector<Vector2f> feature_points = model_PPS.findFeaturePoints();
        Vector2f img_center = Vector2f(model_PPS.x/2, model_PPS.y/2);
        MMCS model_MMCS(feature_points, img_center, 1000.0);
        switch(this->level){
            case 1: 
                model_MMCS.discard_noise(sun::V_MIN_LEVEL_1, sun::V_MAX_LEVEL_1);
                break;
            case 2:
                model_MMCS.discard_noise(sun::V_MIN_LEVEL_2, sun::V_MAX_LEVEL_2);
                break;
            case 3:
                model_MMCS.discard_noise(sun::V_MIN_LEVEL_3, sun::V_MAX_LEVEL_3);
                break;
        }
        // model_MMCS.discard_noise(sun::V_MIN_LEVEL_1, sun::V_MAX_LEVEL_1);
        model_MMCS.gradient_descend(0.01, 1000);

        this->Err = -(img_center+bias) + model_MMCS.center;
    }
    // use Hough transform to locate the center of the target and Err
    else{
        this->model_Hough.run(src);
        Vector2f img_center = Vector2f(this->model_Hough.src.cols/2, this->model_Hough.src.rows/2);
        this->Err = Vector2f(this->model_Hough.finded_circles[0][0], this->model_Hough.finded_circles[0][1]) - (img_center+bias);
    }
    // use PIDcontroller to compute the input vector
    Vector2f input;
    input = this->model_PID->run(this->Err);
    this->msg_velocity.twist.linear.x = input[0];
    this->msg_velocity.twist.linear.y = input[1];
    this->msg_velocity.twist.linear.z = 0;
    //this->pub_vel.publish(this->msg_velocity);
    publish_vel(&this->pub_vel, this->msg_velocity);

    return;
}

bool DockingWorker::is_finished(){
    if(this->Err.norm() < sun::DOCKING_ERROR_TOLERANCE){
        this->convergence_counter ++;
    }else{
        this->convergence_counter = 0;
    }
    if(this->convergence_counter > sun::MAX_CONVERGENCE_COUNTER){
        this->camera->release();
        this->camera = NULL;
        return true;
    }
    return false;
}

#endif