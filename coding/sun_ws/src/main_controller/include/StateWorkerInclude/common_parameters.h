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

//RC channel parameters
    float MAX_PITCH     = 1928;  //forward +x
    float MID_PITCH     = 1509;
    float MIN_PITCH     = 1090;
    float MAX_ROLL      = 1933;  //right -y
    float MID_ROLL      = 1514;
    float MIN_ROLL      = 1095;
    float MAX_THROOTTLE = 1933;
    float MID_THROOTTLE = 1519;
    float MIN_THROOTTLE = 1105;

// velocity constrains
    float MAX_LINEAR_VEL  = 3;
    float MAX_ANGULAR_VEL = 1;

// range of PID parameters
    Vector3f MAX_PID_PARAMS = Vector3f(30, 6, 2);
    Vector3f MIN_PID_PARAMS = Vector3f(20, 4, 0);

// max convergence counter
    int MAX_CONVERGENCE_COUNTER = 300;

// Multi-level PID params
    Vector3f MULTILEVEL_PID_PARAMS_LEVEL_1 = Vector3f(6, 1, 0.2);
    Vector3f MULTILEVEL_PID_PARAMS_LEVEL_2 = Vector3f(6, 1, 0.2);
    Vector3f MULTILEVEL_PID_PARAMS_LEVEL_3 = Vector3f(6, 1, 0.2);

// height locking PID params & tolerance
    Vector3f HEIGHTLOCKING_PID_PARAMS = Vector3f(6, 1, 0.2);
    float    HEIGHTLOCKING_TOLERANCE  = 0.05;

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

// position tolerance
    float POSITION_TOLERANCE_X = 0.08;
    float POSITION_TOLERANCE_Y = 0.08;
    float POSITION_TOLERANCE_Z = 0.05;

// position A B C area
    float A_AREA_X = 2.0;
    float A_AREA_Y = 0;
    float A_AREA_Z = 1.0;
    float B_AREA_X = 0;
    float B_AREA_Y = -2.2;
    float B_AREA_Z = 2.0;
    float C_AREA_X = 2.0;
    float C_AREA_Y = -2.2;
    float C_AREA_Z = 2.0;

}


#endif