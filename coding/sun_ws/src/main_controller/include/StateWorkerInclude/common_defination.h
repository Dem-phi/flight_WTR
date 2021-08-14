#ifndef _COMMON_DEFINATION_
#define _COMMON_DEFINATION_

#define IF_USE_MANNUL 1
#define IF_USE_T265 1

namespace sun{

// StateMode
enum STATE_TYPE{
    TAKEOFF             =   0,
    DETECTING           =   1,
    DOCKING             =   2,
    LANDING             =   3,
    OFFLOADING          =   4,
    ADAPTIVE_LEARNING   =   5,
    MANUAL              =   6,
    WAITING             =   7,
    HEIGHTSERV          =   8,
    POSITION            =   9,
    END                 =   10,
};

// Docking Mode type
enum DOCKING_TYPE{
    DOCKING_CIRCLE      =   0,
    DOCKING_RECTANGLE   =   1, 
    DOCKING_TARGET      =   2,
};

// Adaptive learning type
enum ADAPTIVELEARNING_TYPE{
    ADAPTIVE_HEIGHTLOCK =   0,
    ADAPTIVE_CIRCLE     =   1,  
};

// Detecting figure type
enum DETECTINGFIGURE_TYPE{
    DETECTING_CIRCLE    =   0,
    DETECTING_RECTANGLE =   1,
};

// Goal Pose Type
enum GOALPOSE_TYPE{
    GOAL_A              =   0,
    GOAL_B              =   1,
    GOAL_C              =   2,
    GOAL_H              =   3,
};

}

#endif