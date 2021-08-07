#ifndef _PID_CONTROLLOER_
#define _PID_CONTROLLOER_


#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <time.h>

#define infinity MAXFLOAT

using namespace std;
using namespace Eigen;

/* 
@brief: PIDcontroller which provides computed input vector via former state data
@param: Vector3f PID_params; PID params
 */
class PIDcontroller{
public:
    Vector2f Err_last_1 = Vector2f::Zero(2);
    Vector2f Err_last_2 = Vector2f::Zero(2);
    
    float max_error = infinity;
    Vector3f PID_params;


    Vector2f run(Vector2f Err);

    PIDcontroller(Vector3f PID_params);
    ~PIDcontroller();
};

PIDcontroller::PIDcontroller(Vector3f PID_params){
    this->PID_params = PID_params;
    return;
}

PIDcontroller::~PIDcontroller()
{
}

/* 
@brief: give computed input as Vector2f
@param: Vector2f Err; Error vector this sample time shows
 */
Vector2f PIDcontroller::run(Vector2f Err){
    Vector2f velocity = this->PID_params[0]*(Err-this->Err_last_1)+
                        this->PID_params[1]*Err+
                        this->PID_params[2]*(Err-2*this->Err_last_1+this->Err_last_2);
    if(Err.norm() > this->max_error){
        this->max_error = Err.norm();
    }
    return velocity;
}


#endif