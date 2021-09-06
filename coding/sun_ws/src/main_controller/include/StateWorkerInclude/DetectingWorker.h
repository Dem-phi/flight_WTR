#ifndef _DETECTINGWORKER_
#define _DETECTINGWORKER_

#include <StateWorker.h>
#include <time.h>
#include <HoughCircle.h>
#include <PFS.h>


/* 
Fly towards a given direction with a slow velocity(0.4 m/s) and keep detecting the figure
@param float direction_vector_x
@param float direction_vector_y
@param float max_sec; if the running time exceed the max_sec, flight will stay still
@param int figure_type; Circle or Rectangle
 */
class DetectingWorker:public StateWorker
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    geometry_msgs::TwistStamped msg_vel;
    HoughCircle* model_HOUGH = new HoughCircle();
    PFS* model_PFS = new PFS();

    float speed = 0.1;
    float max_sec;
    int figure_type, convergence_counter = 0, max_counter = 30;
    time_t timer = 0;

    cv::VideoCapture camera;
    
    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    DetectingWorker(ros::NodeHandle &nh, float direction_vector_x, float direction_vector_y, float max_sec, int figure_type);
    ~DetectingWorker();
};

DetectingWorker::DetectingWorker(ros::NodeHandle &nh, float direction_vector_x, float direction_vector_y, float max_sec, int figure_type){
    this->nh = nh;
    this->pub_vel = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10
    );
    float norm = sqrt(direction_vector_x*direction_vector_x+direction_vector_y*direction_vector_y);
    this->msg_vel.twist.linear.x = this->speed*direction_vector_x/norm;
    this->msg_vel.twist.linear.y = this->speed*direction_vector_y/norm;
    this->msg_vel.twist.linear.z = 0;
    this->max_sec = max_sec;
    this->figure_type = figure_type;
}

DetectingWorker::~DetectingWorker(){
}

void DetectingWorker::run(StateInfo state_info){
    ROS_INFO("Detecting!");
    if(this->timer == 0){
        timer = clock();
        this->camera = cv::VideoCapture(0);
    }
    if((clock()-this->timer)/(float)CLOCKS_PER_SEC > this->max_sec){
        this->msg_vel.twist.linear.x = 0;
        this->msg_vel.twist.linear.y = 0;
        this->pub_vel.publish(this->msg_vel);
    }else{
        if(this->figure_type == sun::DETECTING_CIRCLE){
            cv::Mat src;
            this->camera.read(src);
            this->model_HOUGH->run(src);
            if(this->model_HOUGH->finded_circles.size() > 0){
                this->convergence_counter ++;
            }else{
                this->convergence_counter = 0;
            }
        }else{
            cv::Mat src;
            this->model_PFS->rectangle_detect(src);
            if(this->model_PFS->is_finded){
                this->convergence_counter ++;
            }else{
                this->convergence_counter = 0;
            }
        }
        this->pub_vel.publish(this->msg_vel);
    }
    return;
}

bool DetectingWorker::is_finished(){
    if(this->convergence_counter >= this->max_counter){
        this->camera.release();
        return true;
    }else{
        return false;
    }
}

#endif