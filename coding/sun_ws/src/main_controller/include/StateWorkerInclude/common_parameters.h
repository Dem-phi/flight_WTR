#ifndef _COMMON_PARAMETERS_
#define _COMMON_PARAMETERS_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>


using namespace Eigen;
using namespace cv;


namespace sun{
// velocity constrains
float MAX_LINEAR_VEL = 3;
float MAX_ANGULAR_VEL = 1;

// max convergence counter
int MAX_CONVERGENCE_COUNTER = 300;


// Multi-level PID params
Vector3f MULTILEVEL_PID_PARAMS_LEVEL_1 = Vector3f(6, 1, 0.2);
Vector3f MULTILEVEL_PID_PARAMS_LEVEL_2 = Vector3f(6, 1, 0.2);
Vector3f MULTILEVEL_PID_PARAMS_LEVEL_3 = Vector3f(6, 1, 0.2);

// height locking PID params & tolerance
Vector3f HEIGHTLOCKING_PID_PARAMS = Vector3f(6, 1, 0.2);
float HEIGHTLOCKING_TOLERANCE = 0.05;

// camera port
int camera_port = 0;

// PPS & MMCS parameters
float V_MIN_LEVEL_1 = 5;
float V_MAX_LEVEL_1 = 11;
float V_MIN_LEVEL_2 = 5;
float V_MAX_LEVEL_2 = 11;
float V_MIN_LEVEL_3 = 5;
float V_MAX_LEVEL_3 = 11;

// Docking error tolerance
float DOCKING_ERROR_TOLERANCE = 10;


// opencv draw figure color
cv::Scalar COLOR_DRAW = cv::Scalar(120, 180, 120, 15);









}


#endif