#ifndef _COMMON_DEFINATION_
#define _COMMON_DEFINATION_


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
    END                 =   9,
};

// Docking Mode type
enum DOCKING_TYPE{
    DOCKING_CIRCLE      =   0,
    DOCKING_RECTANGLE   =   1, 
    DOCKING_TARGET      =   2,
};




}

#endif