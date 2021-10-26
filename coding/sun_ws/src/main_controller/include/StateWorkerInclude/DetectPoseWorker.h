#ifndef _DETECTPOSEWORKER_H
#define _DETECTPOSEWORKER_H

#include <StateWorker.h>
#include <geometry_msgs/PoseStamped.h>
#include <HoughCircle.h>
#include <vector>
#include <eigen3/Eigen/Dense>

/*!
 * @brief fly to certain area in position state
 * @param int goal_type, for different goal
 *
 */
using namespace Eigen;
class DetectPoseWorker: public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_goal_pose;
    geometry_msgs::PoseStamped goal_pose;

    int wait_circle_num = 30;
    float tolerant_radius = 3.0;

    float bias_x;
    float bias_y;

    bool init_camera = false;
    bool is_send_goal = false;
    int is_find_circle = 0;
    HoughCircle HC;
    vector<Vector2f> circles;
    cv::VideoCapture camera;
    int convergence_counter = 0;
    Matrix3f intrinsic_matrix;
    virtual void run(StateInfo state_info);
    virtual bool is_finished();
    Vector2f KNN_filter();
    DetectPoseWorker(ros::NodeHandle &nh, int bias = 0);
    ~DetectPoseWorker();
};

DetectPoseWorker::DetectPoseWorker(ros::NodeHandle &nh, int bias){
    switch (bias) {
        case 0:{
            this->bias_x = 0;
            this->bias_y = 0;
            break;
        }
        case 1:{
            this->bias_x = -0.25;
            this->bias_y = -0.16;
            break;
        }
        case 2:{
            this->bias_x = -0.17;
            this->bias_y = -0.1;
            break;
        }
        case 3:{
            this->bias_x = 0.13;
            this->bias_y = -0.1;
            break;
        }
        default:{
            this->bias_x = 0.0;
            this->bias_y = 0.0;
            break;
        }
    }
    //this->intrinsic_matrix << 201.7, 0, 325.5, 0, 225.0, 245.0, 0, 0, 1;
    this->intrinsic_matrix << 208.6, 0, 316.2, 0, 212.05, 239.5, 0, 0, 1;
    this->goal_pose.header.frame_id = "body";
    this->nh = nh;
    this->pub_goal_pose = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    return;    
}

DetectPoseWorker::~DetectPoseWorker(){
}

Vector2f DetectPoseWorker::KNN_filter(){
    int a = 0, c = this->wait_circle_num-1, b = (int)((this->wait_circle_num-1)/2);
    int a_r = 0, b_r = 0, c_r = 0;
    Vector2f a_avr = this->circles[a], b_avr = this->circles[b], c_avr = this->circles[c];
    for(int i = 0; i < this->wait_circle_num; i++){
        // cout << a_avr[0] << " " << a_avr[1] << " "
        //      << b_avr[0] << " " << b_avr[1] << " "
        //      << c_avr[0] << " " << c_avr[1] << endl; 
        if((this->circles[i]-this->circles[a]).norm() < this->tolerant_radius){
            a_r ++;
            a_avr = ((float)a_r/(a_r+1))*(a_avr)+(this->circles[i]/(float)(a_r+1));
        }
        if((this->circles[i]-this->circles[b]).norm() < this->tolerant_radius){
            b_r ++;
            b_avr = ((float)b_r/(b_r+1))*(b_avr)+(this->circles[i]/(float)(b_r+1));
        }
        if((this->circles[i]-this->circles[c]).norm() < this->tolerant_radius){
            c_r ++;
            c_avr = ((float)c_r/(c_r+1))*(c_avr)+(this->circles[i]/(float)(c_r+1));
        }
    }
    if(a_r > b_r && a_r > c_r){
        return a_avr;
    }
    if(b_r > a_r && b_r > c_r){
        return b_avr;
    }
    if(c_r > a_r && c_r > b_r){
        return c_avr;
    }
    return a_avr;
}

void DetectPoseWorker::run(StateInfo state_info){
    // check the height then choose a height mode
    if(state_info.cur_pose.position.z < 0.7){
        this->HC.low_detect = true;
    }else{
        this->HC.low_detect = false;
    }
    if(!this->is_send_goal){
        ROS_INFO("Finding Circle ...");
        this->goal_pose.header.stamp = ros::Time::now();

        this->goal_pose.pose.position = state_info.cur_pose.position;
        this->goal_pose.pose.orientation = state_info.init_quater;

        this->pub_goal_pose.publish(this->goal_pose);
        if(!this->init_camera){
            ROS_INFO("Initialized camera!");
            // this->camera.release();
            this->camera = cv::VideoCapture(0);
            this->init_camera = true;
        }
        if(this->is_find_circle++ <= this->wait_circle_num && this->camera.isOpened()){
            this->pub_goal_pose.publish(this->goal_pose);
            Mat src;
            this->camera.read(src);
           // waitKey(1);
    
            if(!src.empty()){
//imshow("window", src);
                this->HC.run(src);
            }else{
                ROS_ERROR("Mat is empty!");
                return;
            }
            this->pub_goal_pose.publish(this->goal_pose);
            if(this->HC.finded_circles.size() > 0){
		        ROS_INFO("Find circle: (%f,%f)", this->HC.finded_circles[0][0], this->HC.finded_circles[0][1]);
                this->circles.push_back(Vector2f(this->HC.finded_circles[0][0],this->HC.finded_circles[0][1]));
            }else{
                this->is_find_circle--;
            }
        }else if(this->is_find_circle > this->wait_circle_num && this->camera.isOpened()){
            this->pub_goal_pose.publish(this->goal_pose);
            Vector2f circle_finded = this->KNN_filter();
            Vector3f tran_c = Vector3f(circle_finded[0], circle_finded[1], 1); 
            Vector3f act_pose = this->intrinsic_matrix.inverse()*tran_c*state_info.cur_pose.position.z;
		   // state_info.cur_pose.position.z;
		   // 0.3 , 0,2
		   // 0.2, 0.2
		    // 0.33, 0.2
            this->goal_pose.pose.position.x = state_info.cur_pose.position.x-act_pose[1]+this->bias_x;
            this->goal_pose.pose.position.y = state_info.cur_pose.position.y-act_pose[0]+this->bias_y;
            this->goal_pose.pose.position.z = state_info.cur_pose.position.z;
            this->goal_pose.pose.orientation = state_info.init_quater;
            this->is_send_goal = true;
            this->pub_goal_pose.publish(this->goal_pose);
            ROS_INFO("goal pose x = %f, y = %f", act_pose[0], act_pose[1]);
        }else if(!this->camera.isOpened()){
            ROS_ERROR("Camera cannot be opened!");
        }
    }
    else{
        // ROS_INFO("Find circle!!!");
        this->goal_pose.header.stamp = ros::Time::now();
        //this->goal_pose.pose.orientation = state_info.cur_pose.orientation;
        this->pub_goal_pose.publish(this->goal_pose);
        if( abs(this->goal_pose.pose.position.x - state_info.cur_pose.position.x) < sun::POSITION_TOLERANCE_X &&
            abs(this->goal_pose.pose.position.y - state_info.cur_pose.position.y) < sun::POSITION_TOLERANCE_Y &&
            abs(this->goal_pose.pose.position.z - state_info.cur_pose.position.z) < sun::POSITION_TOLERANCE_Z ){
            this->convergence_counter ++;
        }else{
            this->convergence_counter = 0;
        }
        return;
    }
}

bool DetectPoseWorker::is_finished(){
    if(this->convergence_counter >= sun::MAX_CONVERGENCE_COUNTER){
        this->camera.release();
        cv::destroyAllWindows();
        return true;
    }
    else{
        return false;
    }
}

#endif
